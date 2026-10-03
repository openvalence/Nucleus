// ValenceAccessoryIo -- implementation. See ValenceAccessoryIo.h for ownership
// and the ESTOP contract, AccessoryIo.h for the request rules.
// Constraints:
// - BoardIo task only, except the request doors, qwiicI2cBus() and
//   accessoryUartOpen(). LEDC low-speed timer 1 and channels 1 and 2 belong
//   to this file.
// - NEVER gpio_config() OR gpio_reset_pin() HERE (BoardPins.h, val-091.72):
//   pads are set up with gpio_set_direction() and gpio_set_pull_mode() and by
//   the LEDC, I2C and UART drivers' own pin setup, none of which reaches
//   rtc_gpio_deinit() for an HP peripheral.
// - BSS: the request block, two gesture cores and a few words. The I2C and
//   UART drivers allocate from the internal heap once, at their first open;
//   neither is ever closed.

#include "system/ValenceAccessoryIo.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <soc/soc_caps.h>

#include "geiger/geiger.h"
#include "system/BoardPins.h"

namespace valence {

namespace {

using accessory::Dir;
using accessory::Drive;
using accessory::Pwm;

constexpr const char* kTag = "accessory";

constexpr gpio_num_t pin(int n) { return static_cast<gpio_num_t>(n); }

// ---- THE KNOBS --------------------------------------------------------------

// Both PWMs share one timer and this frequency. J10 feeds an off-board MOSFET
// trigger module (Hardware SPEC.md 2026-09-23 accessory-breakouts row), so the
// frequency suits the slowest module input: 1 kHz passes an optocoupled input
// stage that would smear a 20 kHz period, and keeps the inductive-kick heating
// low in a module without a freewheel diode. A small DC pump hums at it. THE
// CALIBRATION KNOB: a direct-gate module with a freewheel diode runs silent at
// 20 kHz, where kDutyBits still fits under PLL_F80M.
constexpr uint32_t kPwmHz = 1000;

// Every LEDC timer on the P4 runs from ONE global clock, and the fan's timer
// (ValenceFan.cpp) holds PLL_F80M: a timer asking for any other clock fails
// "timer clock conflict". Never LEDC_AUTO_CLK here; it tries XTAL first.
constexpr ledc_clk_cfg_t kLedcClock = LEDC_USE_PLL_DIV_CLK;
constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_1;

constexpr uart_port_t kUartPort = static_cast<uart_port_t>(BOARD_UART_ACC);
static_assert(BOARD_UART_ACC >= 1 && BOARD_UART_ACC < SOC_UART_HP_NUM,
              "the header UART is an HP UART, never UART0 or the LP UART");
// The header UART's receive ring, bytes; the driver wants more than the
// 128-byte hardware FIFO.
constexpr int kUartRxBytes = 512;

// ---- the pads, by function --------------------------------------------------

constexpr std::array<int, accessory::kPwmCount> kPwmPin{BOARD_GPIO_PUMP_PWM, BOARD_GPIO_EXT_PWM};
constexpr std::array<ledc_channel_t, accessory::kPwmCount> kPwmChannel{LEDC_CHANNEL_1, LEDC_CHANNEL_2};
constexpr std::array<const char*, accessory::kPwmCount> kPwmName{"pump", "EXT PWM"};
constexpr std::array<int, accessory::kAuxCount> kAuxPin{BOARD_GPIO_AUX1, BOARD_GPIO_AUX2};
constexpr std::array<const char*, accessory::kAuxCount> kAuxName{"AUX1", "AUX2"};
constexpr std::array<int, accessory::kGpioCount> kIoPin{BOARD_GPIO_ACC_IO1, BOARD_GPIO_ACC_IO2,
                                                       BOARD_GPIO_ACC_IO3, BOARD_GPIO_ACC_IO4,
                                                       BOARD_GPIO_ACC_IO5};

// ---- state ------------------------------------------------------------------

// Any task: the requests, the parked gestures, the two claim flags.
accessory::Requests g_req;
std::array<accessory::GestureSlot, accessory::kAuxCount> g_auxParked;
std::atomic<bool> g_pwmUp{false};
std::atomic<bool> g_uartClaimed{false};

// The BoardIo task's: the gesture cores and what each pad shows now.
std::array<button::Button, accessory::kAuxCount> g_aux;
std::array<uint32_t, accessory::kPwmCount> g_dutyOut{};
std::array<Dir, accessory::kGpioCount> g_dirOut{};
std::array<Drive, accessory::kGpioCount> g_driveOut{};
uint32_t g_estopsSeen = 0;

// ---- setup ------------------------------------------------------------------

// A plain HP GPIO input on an LP pad, without gpio_config().
void padInput(int n, gpio_pull_mode_t pull) {
    gpio_set_direction(pin(n), GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin(n), pull);
}

bool pwmBegin() {
    ledc_timer_config_t t{};
    t.speed_mode = kLedcMode;
    t.duty_resolution = static_cast<ledc_timer_bit_t>(accessory::kDutyBits);
    t.timer_num = kLedcTimer;
    t.freq_hz = kPwmHz;
    t.clk_cfg = kLedcClock;
    if (ledc_timer_config(&t) != ESP_OK) return false;
    for (size_t i = 0; i < accessory::kPwmCount; ++i) {
        ledc_channel_config_t c{};
        c.gpio_num = kPwmPin[i];
        c.speed_mode = kLedcMode;
        c.channel = kPwmChannel[i];
        c.timer_sel = kLedcTimer;
        c.duty = 0;
        c.hpoint = 0;
        if (ledc_channel_config(&c) != ESP_OK) return false;
    }
    return true;
}

i2c_master_bus_handle_t openQwiic() {
    i2c_master_bus_config_t bus{};
    bus.i2c_port = -1;   // a free HP port; auto never picks the LP I2C
    bus.sda_io_num = pin(BOARD_GPIO_QWIIC_SDA);
    bus.scl_io_num = pin(BOARD_GPIO_QWIIC_SCL);
    bus.clk_source = I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt = 7;
    // The board fits 2.2k pull-ups; the internal ones only keep a bare stamp's
    // lines from floating.
    bus.flags.enable_internal_pullup = 1;
    i2c_master_bus_handle_t h = nullptr;
    if (i2c_new_master_bus(&bus, &h) != ESP_OK) {
        GLOGE(kTag, "Qwiic I2C bus would not open on SDA G%d, SCL G%d (both HP ports taken?)",
              BOARD_GPIO_QWIIC_SDA, BOARD_GPIO_QWIIC_SCL);
        return nullptr;
    }
    GLOGI(kTag, "Qwiic I2C bus open: SDA G%d, SCL G%d", BOARD_GPIO_QWIIC_SDA, BOARD_GPIO_QWIIC_SCL);
    return h;
}

// ---- the BoardIo pass -------------------------------------------------------

// TODO(val-091.19): PUMP_FLT reaches the P4 only as SV_W_PUMP_FLT in the board
// monitor's STATUS warns (SV_REG_STATUS 0x10, Supervisor.h), read once at boot
// by the self-check and nowhere at runtime. When the runtime poll lands, a set
// bit zeroes the pump request the way accessoryIoEstop() does.
void servicePwm() {
    if (!g_pwmUp.load(std::memory_order_acquire)) return;
    for (size_t i = 0; i < accessory::kPwmCount; ++i) {
        const uint32_t d = g_req.duty(static_cast<Pwm>(i));
        if (d == g_dutyOut[i]) continue;
        ledc_set_duty(kLedcMode, kPwmChannel[i], d);
        ledc_update_duty(kLedcMode, kPwmChannel[i]);
        if ((d == 0) != (g_dutyOut[i] == 0))
            GLOGI(kTag, "%s %s (%.0f%%)", kPwmName[i], d != 0 ? "on" : "off",
                  double(d) * 100.0 / double(accessory::kDutyMax));
        g_dutyOut[i] = d;
    }
}

void serviceGpio() {
    for (uint8_t i = 0; i < accessory::kGpioCount; ++i) {
        const Dir dir = g_req.dir(i);
        if (dir != g_dirOut[i]) {
            // Every pad left boot as an input with a pull-down, which is also
            // an output's released state: only the pull-up input changes it.
            if (dir == Dir::input_pullup) gpio_set_pull_mode(pin(kIoPin[i]), GPIO_PULLUP_ONLY);
            g_dirOut[i] = dir;
            GLOGI(kTag, "IO%u (G%d): %s", unsigned(i + 1), kIoPin[i], accessory::dirName(dir));
        }
        if (dir != Dir::output) continue;
        const Drive d = g_req.drive(i);
        if (d == g_driveOut[i]) continue;
        const gpio_num_t n = pin(kIoPin[i]);
        if (d == Drive::released) {
            gpio_set_direction(n, GPIO_MODE_INPUT);
        } else {
            // Level first, then the driver: the pad never shows a stale level.
            gpio_set_level(n, d == Drive::high ? 1 : 0);
            gpio_set_direction(n, GPIO_MODE_INPUT_OUTPUT);
        }
        g_driveOut[i] = d;
        GLOGD(kTag, "IO%u %s", unsigned(i + 1),
              d == Drive::released ? "released" : (d == Drive::high ? "high" : "low"));
    }
}

void serviceAux(uint32_t nowMs) {
    for (size_t i = 0; i < accessory::kAuxCount; ++i) {
        const bool pressed = gpio_get_level(pin(kAuxPin[i])) == 0;
        const button::Gesture g = g_aux[i].sample(pressed, nowMs);
        if (g == button::Gesture::none) continue;
        if (g_auxParked[i].park(g))
            GLOGW(kTag, "%s %s at the board", kAuxName[i], button::gestureName(g));
        else
            GLOGW(kTag, "%s held past %lu s: ignored as stuck (shorted J13 or a jammed switch?)",
                  kAuxName[i], static_cast<unsigned long>(button::kStuckMs / 1000));
    }
}

}  // namespace

// ---- the doors --------------------------------------------------------------

bool accessoryIoBegin() {
    // Board 10k pull-ups on AUX; the internal ones only cover a bare stamp.
    for (int n : kAuxPin) padInput(n, GPIO_PULLUP_ONLY);
    for (int n : kIoPin) padInput(n, GPIO_PULLDOWN_ONLY);
    const bool pwm = pwmBegin();
    g_pwmUp.store(pwm, std::memory_order_release);
    if (pwm)
        GLOGI(kTag, "accessory host up: pump G%d and EXT PWM G%d at %lu Hz, AUX G%d/G%d, IO1-IO5 released",
              BOARD_GPIO_PUMP_PWM, BOARD_GPIO_EXT_PWM, static_cast<unsigned long>(kPwmHz),
              BOARD_GPIO_AUX1, BOARD_GPIO_AUX2);
    else
        GLOGE(kTag, "LEDC timer %d (pump G%d, EXT PWM G%d) failed: both PWMs stay off",
              int(kLedcTimer), BOARD_GPIO_PUMP_PWM, BOARD_GPIO_EXT_PWM);
    return pwm;
}

void accessoryIoService(uint32_t nowMs) {
    const uint32_t estops = g_req.estops();
    if (estops != g_estopsSeen) {
        g_estopsSeen = estops;
        GLOGI(kTag, "ESTOP: pump, EXT PWM and IO outputs off");
    }
    servicePwm();
    serviceGpio();
    serviceAux(nowMs);
}

void accessoryIoEstop() { g_req.estop(); }

bool accessoryPwmSet(Pwm out, float duty) {
    return g_pwmUp.load(std::memory_order_acquire) && g_req.setDuty(out, duty);
}

bool accessoryGpioConfigure(uint8_t io, Dir dir) {
    return io >= 1 && io <= accessory::kGpioCount && g_req.configure(uint8_t(io - 1), dir);
}

bool accessoryGpioSet(uint8_t io, bool high) {
    return io >= 1 && io <= accessory::kGpioCount && g_req.set(uint8_t(io - 1), high);
}

std::optional<bool> accessoryGpioGet(uint8_t io) {
    if (io < 1 || io > accessory::kGpioCount) return std::nullopt;
    return gpio_get_level(pin(kIoPin[io - 1])) != 0;
}

button::Gesture accessoryAuxTake(accessory::Aux which) {
    const size_t i = size_t(which);
    return i < accessory::kAuxCount ? g_auxParked[i].take() : button::Gesture::none;
}

i2c_master_bus_t* qwiicI2cBus() {
    // The init guard makes the open happen once across tasks.
    static const i2c_master_bus_handle_t bus = openQwiic();
    return bus;
}

std::optional<int> accessoryUartOpen(uint32_t baud) {
    bool unclaimed = false;
    if (!g_uartClaimed.compare_exchange_strong(unclaimed, true)) {
        GLOGW(kTag, "header UART refused: another consumer holds it");
        return std::nullopt;
    }
    // Another driver on this port is an allocation clash in BoardPins.h.
    if (uart_is_driver_installed(kUartPort)) {
        g_uartClaimed.store(false);
        GLOGE(kTag, "header UART%d already belongs to another driver", BOARD_UART_ACC);
        return std::nullopt;
    }
    uart_config_t c{};
    c.baud_rate = int(baud);
    c.data_bits = UART_DATA_8_BITS;
    c.parity = UART_PARITY_DISABLE;
    c.stop_bits = UART_STOP_BITS_1;
    c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    c.source_clk = UART_SCLK_DEFAULT;
    if (uart_driver_install(kUartPort, kUartRxBytes, 0, 0, nullptr, 0) == ESP_OK) {
        if (uart_param_config(kUartPort, &c) == ESP_OK &&
            uart_set_pin(kUartPort, BOARD_GPIO_ACC_UART_TX, BOARD_GPIO_ACC_UART_RX,
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK) {
            // An open header idles high instead of clocking in noise.
            gpio_set_pull_mode(pin(BOARD_GPIO_ACC_UART_RX), GPIO_PULLUP_ONLY);
            GLOGI(kTag, "header UART%d open: TX G%d, RX G%d, %lu baud 8N1", BOARD_UART_ACC,
                  BOARD_GPIO_ACC_UART_TX, BOARD_GPIO_ACC_UART_RX, static_cast<unsigned long>(baud));
            return int(kUartPort);
        }
        uart_driver_delete(kUartPort);
    }
    g_uartClaimed.store(false);
    GLOGE(kTag, "header UART%d would not open at %lu baud", BOARD_UART_ACC,
          static_cast<unsigned long>(baud));
    return std::nullopt;
}

}  // namespace valence
