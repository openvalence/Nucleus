// ValencePdSource -- implementation. See ValencePdSource.h for ownership and
// the polling rule, PdSource.h for what the bytes mean.
// Constraints:
// - The PDO and RDO are two reads. A contract that changes between them sets
//   New Contract again, so PD_INT falls again and the pair is re-read.
// - A failed read keeps the last good reading: a bus glitch never moves the
//   verdict, only a reading does.
// - NEVER gpio_config() OR gpio_reset_pin() HERE (BoardPins.h, val-091.72):
//   PD_INT is LP pad 0, set up with gpio_set_direction() and
//   gpio_set_pull_mode(); the I2C pads are the bus owner's.

#include "system/ValencePdSource.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>

#include "geiger/geiger.h"
#include "motion/ValenceMotion.h"
#include "system/BoardPins.h"
#include "system/ValenceAccessoryIo.h"
#include "system/ValenceMotorSwitch.h"

namespace valence {

namespace {

constexpr const char* kTag = "pd";

constexpr gpio_num_t kPinSda = static_cast<gpio_num_t>(BOARD_GPIO_QWIIC_SDA);
constexpr gpio_num_t kPinScl = static_cast<gpio_num_t>(BOARD_GPIO_QWIIC_SCL);
constexpr gpio_num_t kPinInt = static_cast<gpio_num_t>(BOARD_GPIO_PD_INT);

// Standard mode: J14 carries accessories of unknown speed, and the pogo branch
// adds 33 R and a spring contact. A few blocks per event never need more.
constexpr uint32_t kSclHz = 100000;
constexpr int kTimeoutMs = 10;
// A held PD_INT is serviced at most this often: a line stuck low costs ten
// short transactions a second, never one per pass.
constexpr uint32_t kServiceGapMs = 100;
// The input ceilings move with no PD event (a 0x1000 or 0x3120 write), so the
// verdict is re-judged this often regardless.
constexpr uint32_t kJudgeEveryMs = 1000;
// With INT_MASK1 not armed, the contract is polled this often instead.
constexpr uint32_t kPollEveryMs = 1000;

// This module's device on the Qwiic bus; never removed.
i2c_master_dev_handle_t g_dev = nullptr;

// The owner's alone: app_main inside pdSourceBegin(), then the BoardIo task.
pd::Reading g_owned{};
bool g_armTried = false;
bool g_irqArmed = false;
bool g_pushedOk = true;   // what the motor switch last heard: it boots true
uint32_t g_lastServiceMs = 0;
uint32_t g_lastJudgeMs = 0;
uint32_t g_lastPollMs = 0;

// The published copy, written by the owner and read by any task under g_mux.
// Inside the lock: one struct copy, nothing else.
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
pd::Reading g_published{};

void publish(const pd::Reading& r) {
    portENTER_CRITICAL(&g_mux);
    g_published = r;
    portEXIT_CRITICAL(&g_mux);
}

// ---- the transactions -------------------------------------------------------

// One block read: the register number written, then [count][data] read back
// (SLVUCR7 Figure 1-3). out receives the data bytes only. A count short of
// out is ESP_ERR_INVALID_SIZE.
esp_err_t readBlock(uint8_t reg, std::span<uint8_t> out) {
    std::array<uint8_t, 1 + pd::kIntBytes> raw{};
    if (out.size() > pd::kIntBytes) return ESP_ERR_INVALID_ARG;
    const std::span<uint8_t> r(raw.data(), out.size() + 1);
    const esp_err_t e = i2c_master_transmit_receive(g_dev, &reg, 1, r.data(), r.size(), kTimeoutMs);
    if (e != ESP_OK) return e;
    if (!pd::blockHolds(r, out.size())) return ESP_ERR_INVALID_SIZE;
    std::copy(r.begin() + 1, r.end(), out.begin());
    return ESP_OK;
}

// One block write: register number, count, data (SLVUCR7 Figure 1-2).
esp_err_t writeBlock(uint8_t reg, std::span<const uint8_t> data) {
    std::array<uint8_t, 2 + pd::kIntBytes> b{};
    if (data.size() > pd::kIntBytes) return ESP_ERR_INVALID_ARG;
    b[0] = reg;
    b[1] = uint8_t(data.size());
    std::copy(data.begin(), data.end(), b.begin() + 2);
    return i2c_master_transmit(g_dev, b.data(), data.size() + 2, kTimeoutMs);
}

// MODE decides what ACKed at 0x21: APP is ready, BOOT and PTCH are not, any
// other bytes are a stranger. A block too short to hold four is
// ESP_ERR_INVALID_SIZE, which only the boot identification reads as a stranger.
esp_err_t readMode(pd::Reading& r) {
    std::array<uint8_t, pd::kModeBytes> m{};
    const esp_err_t e = readBlock(pd::kRegMode, m);
    if (e != ESP_OK) return e;
    r.mode_text = pd::modeText(m);
    switch (pd::decodeMode(m)) {
        case pd::Mode::app:     r.presence = pd::Presence::ready;     break;
        case pd::Mode::boot:
        case pd::Mode::patch:   r.presence = pd::Presence::not_ready; break;
        case pd::Mode::foreign: r.presence = pd::Presence::foreign;   break;
    }
    return ESP_OK;
}

esp_err_t readContract(pd::Contract& c) {
    std::array<uint8_t, pd::kObjectBytes> pdo{};
    std::array<uint8_t, pd::kObjectBytes> rdo{};
    esp_err_t e = readBlock(pd::kRegActivePdo, pdo);
    if (e == ESP_OK) e = readBlock(pd::kRegActiveRdo, rdo);
    if (e != ESP_OK) return e;
    c = pd::decodeContract(pd::le32(pdo), pd::le32(rdo));
    return ESP_OK;
}

// INT_MASK1 bytes 1-10 reset to 0 (SLVUCR7 4.6): the contract events are armed
// read-modify-write and read back. False leaves the contract polled.
bool armIrq() {
    pd::IntBits mask{};
    if (readBlock(pd::kRegIntMask1, mask) != ESP_OK) return false;
    if (writeBlock(pd::kRegIntMask1, pd::withContractEvents(mask)) != ESP_OK) return false;
    pd::IntBits back{};
    return readBlock(pd::kRegIntMask1, back) == ESP_OK && pd::armsContractEvents(back);
}

// INT_EVENT1 read, then exactly what was read written to INT_CLEAR1. PD_INT
// lets go unless an event landed in between, which is then serviced next.
esp_err_t takeEvents(pd::IntBits& ev) {
    const esp_err_t e = readBlock(pd::kRegIntEvent1, ev);
    if (e != ESP_OK || !pd::anySet(ev)) return e;
    return writeBlock(pd::kRegIntClear1, ev);
}

// MODE and, in APP, the contract, into `r` only when every read landed. A
// controller already identified never turns into a stranger here: foreign
// bytes are a failed read. The IRQ is armed the first time APP is seen.
esp_err_t refresh(pd::Reading& r) {
    pd::Reading next = r;
    esp_err_t e = readMode(next);
    if (e != ESP_OK) return e;
    if (next.presence == pd::Presence::foreign) return ESP_ERR_INVALID_RESPONSE;
    next.contract = pd::Contract{};
    if (next.presence == pd::Presence::ready) {
        if (!g_armTried) {
            g_armTried = true;
            g_irqArmed = armIrq();
            if (!g_irqArmed)
                GLOGW(kTag, "INT_MASK1 did not take: PD_INT stays quiet, the contract is polled every %lu ms",
                      static_cast<unsigned long>(kPollEveryMs));
        }
        e = readContract(next.contract);
        if (e != ESP_OK) return e;
    }
    r = next;
    return ESP_OK;
}

// ---- the verdict ------------------------------------------------------------

// Its own frame, so the census copy is off the stack before any log formats.
[[gnu::noinline]] pd::Ceilings inputCeilings() {
    const MotionCensus c = motionCensus();
    return {c.input_vmax_mm_s, c.input_amax_mm_s2};
}

// Pushes a changed verdict to the motor switch and says so.
void judge() {
    const pd::Assessment a = pd::assess(g_owned, inputCeilings());
    const bool ok = a.motorAllowed();
    if (ok == g_pushedOk) return;
    const motorswitch::State was = motorSwitchStatus().state;
    g_pushedOk = ok;
    motorSwitchSetSourceOk(ok);
    std::array<char, 96> text{};
    pd::describe(text, a);
    if (ok) {
        GLOGW(kTag, "motor power allowed again (enable on the next release): %s", text.data());
    } else {
        const bool cut = was == motorswitch::State::on || was == motorswitch::State::precharging;
        GLOGE(kTag, "motor power %s: %s", cut ? "CUT" : "refused", text.data());
    }
}

void logContract(bool renegotiated) {
    std::array<char, 96> text{};
    pd::describe(text, pd::assess(g_owned, inputCeilings()));
    GLOGW(kTag, "PD %s: %s", renegotiated ? "renegotiated" : "contract changed", text.data());
}

void logBoot() {
    const pd::Reading& r = g_owned;
    if (r.presence != pd::Presence::ready) return;   // judge() names the refusal
    const pd::Contract& c = r.contract;
    if (c.supply == pd::Supply::none) {
        GLOGW(kTag, "TPS26750 at 0x%02x (APP), no explicit contract", unsigned(pd::kAddress));
        return;
    }
    const pd::Profile p = pd::profileFor(c);
    GLOGI(kTag, "TPS26750 at 0x%02x (APP): %.1f V %.2f A %s #%u (%.0f W%s), +BUS %.1f V, motion budget %.0f W%s",
          unsigned(pd::kAddress), double(c.mv) * 1e-3, double(c.ma) * 1e-3, pd::supplyName(c.supply),
          unsigned(c.position), double(c.watts()), c.mismatch ? ", capability mismatch" : "",
          double(p.bus_mv) * 1e-3, double(p.budget_w), p.buck ? ", 48 V build" : "");
}

void failBoot(esp_err_t e, const char* what) {
    g_owned.presence = pd::Presence::bus_error;
    g_owned.bus_err = uint32_t(e);
    GLOGE(kTag, "%s at 0x%02x on G%d/G%d: %s", what, unsigned(pd::kAddress), int(kPinSda), int(kPinScl),
          esp_err_to_name(e));
}

}  // namespace

// ---- public surface ---------------------------------------------------------

bool pdSourceBegin() {
    // The daughterboard pulls PD_INT up (R5, 100k to its LDO); the internal
    // pull-up only gives a board without one a defined high.
    gpio_set_direction(kPinInt, GPIO_MODE_INPUT);
    gpio_set_pull_mode(kPinInt, GPIO_PULLUP_ONLY);

    i2c_master_bus_handle_t bus = qwiicI2cBus();
    esp_err_t e = ESP_OK;
    if (bus == nullptr) {
        failBoot(ESP_ERR_INVALID_STATE, "Qwiic bus not open");
    } else if ((e = i2c_master_probe(bus, pd::kAddress, kTimeoutMs)) == ESP_ERR_NOT_FOUND) {
        GLOGI(kTag, "no PD daughterboard (0x%02x silent): DC input", unsigned(pd::kAddress));
    } else if (e != ESP_OK) {
        failBoot(e, "Qwiic probe failed");
    } else {
        i2c_device_config_t dev{};
        dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev.device_address = pd::kAddress;
        dev.scl_speed_hz = kSclHz;
        if ((e = i2c_master_bus_add_device(bus, &dev, &g_dev)) != ESP_OK) {
            g_dev = nullptr;
            failBoot(e, "device add failed");
        } else if ((e = readMode(g_owned)) == ESP_ERR_INVALID_SIZE) {
            g_owned.presence = pd::Presence::foreign;   // ACKed, but no TI block came back
            g_owned.mode_text = {'?', '?', '?', '?', '\0'};
        } else if (e != ESP_OK) {
            failBoot(e, "MODE unreadable");
        } else if (g_owned.presence != pd::Presence::foreign) {
            // A TPS26750, so writes are safe. Pending events (Plug Insert sets
            // at reset) are cleared BEFORE the contract read: an event after
            // the read holds PD_INT for the first service.
            pd::IntBits ev{};
            if (takeEvents(ev) != ESP_OK) GLOGW(kTag, "INT_EVENT1 not cleared at boot");
            if ((e = refresh(g_owned)) != ESP_OK) failBoot(e, "contract unreadable");
            else logBoot();
        }
    }
    publish(g_owned);
    judge();
    return g_owned.presence == pd::Presence::ready;
}

void pdSourceService(uint32_t nowMs) {
    // A stranger at 0x21 is never addressed again: every register read starts
    // with a write, and its outputs are not ours to drive.
    if (g_dev == nullptr || g_owned.presence == pd::Presence::foreign) return;
    bool reread = false;
    bool renegotiated = false;
    if (gpio_get_level(kPinInt) == 0 && nowMs - g_lastServiceMs >= kServiceGapMs) {
        g_lastServiceMs = nowMs;
        pd::IntBits ev{};
        const esp_err_t e = takeEvents(ev);
        if (e != ESP_OK) {
            GLOGW_EVERY_MS(10000, kTag, "PD_INT low, INT_EVENT1 not taken: %s", esp_err_to_name(e));
        } else {
            if (pd::bitSet(ev, pd::kIntHardReset) || pd::bitSet(ev, pd::kIntCannotProvide))
                GLOGW(kTag, "PD event:%s%s", pd::bitSet(ev, pd::kIntHardReset) ? " hard reset" : "",
                      pd::bitSet(ev, pd::kIntCannotProvide) ? " source cannot provide the request" : "");
            renegotiated = pd::bitSet(ev, pd::kIntNewContract);
            reread = pd::anyContractEvent(ev) || pd::bitSet(ev, pd::kIntPatchLoaded);
        }
    }
    if (!g_irqArmed && nowMs - g_lastPollMs >= kPollEveryMs) {
        g_lastPollMs = nowMs;
        reread = true;
    }
    if (reread) {
        const pd::Reading before = g_owned;
        const esp_err_t e = refresh(g_owned);
        if (e != ESP_OK) {
            GLOGW_EVERY_MS(10000, kTag, "contract re-read failed (%s): the last reading stands",
                           esp_err_to_name(e));
        } else {
            publish(g_owned);
            if (renegotiated || !(g_owned == before)) logContract(renegotiated);
        }
    }
    if (reread || nowMs - g_lastJudgeMs >= kJudgeEveryMs) {
        g_lastJudgeMs = nowMs;
        judge();
    }
}

pd::Assessment pdSourceAssessNow() {
    portENTER_CRITICAL(&g_mux);
    const pd::Reading r = g_published;
    portEXIT_CRITICAL(&g_mux);
    return pd::assess(r, inputCeilings());
}

}  // namespace valence
