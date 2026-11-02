// BarnacleLink -- the barnacle's link host: the helper UART and its task (as
// the drive-link door), TRIP as the home sense, and the helper surface
// (BarnacleLink.h)
// Constraints:
// - Barnacle only: src/CMakeLists.txt links this file in place of
//   ValenceDriveLink.cpp and ValenceHomeSense.cpp. See BarnacleLink.h for
//   the task, the crossings and the staleness rules.
// - TRIP is LPG8, an LP pad: set up with gpio_set_direction(),
//   gpio_set_pull_mode() and gpio_set_intr_type(), never gpio_config() or
//   gpio_reset_pin() (BoardPins.h, val-091.72).
// - The TRIP interrupt fires on the FALLING edge (tripped) and runs the
//   arbiter's park first, exactly as ValenceHomeSense.cpp's rise does; the
//   GPIO ISR service is not in IRAM, so a flash write holds it off
//   (TODO(val-sv6) covers both boards).
// - high() and the anomaly watch read the pad twice kHomeSenseDebounceUs
//   apart before they believe a trip; the helper filters the current before
//   it pulls the line (barnacle::kTripOnSamples).
// - The UART has a kRxRingBytes ring and a kTxRingBytes ring: an update
//   chunk (barnacle::kMaxWire) outgrows the 128-byte TX FIFO, and the ring
//   lets uart_write_bytes() copy and return inside the pass.
// - THE HELPER'S UPDATE (bd val-omh3.7) runs on this task: the image is
//   embedded in this firmware (src/CMakeLists.txt), barnacle::UpdateSender is
//   the machine, this file the glue and every log line. The gate is
//   BarnacleLink.h helperUpdateRefusal() (no motion, no P4 update in flight);
//   the start act unhomes through motionUnhome() before BEGIN goes out, never
//   a second unhome. While a transfer owns the link, commands go out every
//   barnacle::kUpdateCommandMs, not every pass.
// - BSS only: the decoder (~260 B), the reading (~90 B), the sender (~80 B),
//   one wire frame (g_out), the queue storage and a few words. Nothing
//   allocates after begin.
// See: BarnacleLink.h, lib/barnacle_link, ValenceDriveLink.h,
// ValenceHomeSense.h, BoardPins.h

#include "system/BarnacleLink.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>
#include <type_traits>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include "barnacle_link/barnacle_update.hpp"
#include "barnacle_shim_image.h"
#include "geiger/geiger.h"
#include "motion/MotionArbiter.h"
#include "motion/ValenceMotion.h"
#include "system/BoardPins.h"
#include "system/ValenceDriveLink.h"
#include "system/ValenceHomeSense.h"
#include "system/ValenceMotorSwitch.h"
#include "system/ValenceOta.h"

#ifndef CONFIG_NUCLEUS_BOARD_BARNACLE
#error "BarnacleLink.cpp is the barnacle's host; src/CMakeLists.txt links it only there"
#endif

// The helper's image (src/CMakeLists.txt target_add_binary_data), in flash.
extern "C" const uint8_t kShimImageStart[] asm("_binary_barnacle_shim_bin_start");
extern "C" const uint8_t kShimImageEnd[] asm("_binary_barnacle_shim_bin_end");

namespace valence {

namespace {

constexpr const char* kTag = "helper";

constexpr uart_port_t kUart = uart_port_t(BOARD_UART_LINK);
static_assert(BOARD_UART_LINK >= 1 && BOARD_UART_LINK < SOC_UART_HP_NUM, "an HP UART, never UART0");
// Two passes of status frames at 500 Hz is ~70 B; the ring holds an outage of
// the task several times that long.
constexpr int kRxRingBytes = 1024;
// Two update chunks' room: the pass never waits on the FIFO.
constexpr int kTxRingBytes = 512;
static_assert(kTxRingBytes >= int(2 * barnacle::kMaxWire), "a chunk and a command fit the TX ring");
// Bytes read per uart_read_bytes() call; the pass loops until the ring is dry.
constexpr size_t kRxChunk = 128;
// The counters' Info line in /diag.
constexpr uint32_t kCensusEveryMs = 60000;
// A helper never heard this long after the link task starts is logged once.
constexpr uint32_t kNeverHeardMs = 3000;

static_assert(barnacle::kHomeTripDetectUs <= kStallDetectUs,
              "the helper's home stall detect must fit the arbiter's contact-to-park budget");
static_assert(barnacle::kCommandPeriodMs == kDriveLinkPeriodMs,
              "one command frame per link pass, outside a helper update");
static_assert(std::is_trivially_copyable_v<HelperReading>, "the mailbox copies it bytewise");
static_assert(std::is_trivially_copyable_v<DriveLinkStatus>, "the mailbox copies it bytewise");

constexpr gpio_num_t pin(int n) { return static_cast<gpio_num_t>(n); }
constexpr gpio_num_t kTripPad = pin(BOARD_GPIO_TRIP);

uint32_t nowMs() { return uint32_t(esp_timer_get_time() / 1000); }
uint32_t nowUs32() { return static_cast<uint32_t>(esp_timer_get_time()); }

// Low is tripped (BoardPins.h).
bool padTripped() { return gpio_get_level(kTripPad) == 0; }

// A trip believed: low now and still low kHomeSenseDebounceUs later.
bool padTrippedConfirmed() {
    if (!padTripped()) return false;
    esp_rom_delay_us(kHomeSenseDebounceUs);
    return padTripped();
}

// ---- state: the link task's alone after driveLinkBegin() ------------------------

barnacle::Decoder g_decoder;
barnacle::SeqTracker g_seq;
StrayTripWatch g_strayWatch;
HelperReading g_reading;        // what the task last published
uint32_t g_lastOkMs = 0;
bool g_wasFresh = false;
uint8_t g_txSeq = 0;
uint32_t g_disagree = 0;        // consecutive passes TRIP and the helper's flag disagree
uint32_t g_nextCensusMs = 0;
uint32_t g_startMs = 0;
bool g_neverLogged = false;
bool g_noCurrent = false;       // the helper's last status carried no current
barnacle::Boxcar<kRestFrames> g_rest;   // status currents, for HelperReading::rest_ma
barnacle::Mode g_mode = barnacle::Mode::idle;
bool g_uartUp = false;          // written once by driveLinkBegin() before the task exists
TaskHandle_t g_task = nullptr;
std::array<uint8_t, barnacle::kMaxWire> g_out{};   // one frame being sent

// The helper's update. g_updateOn is written once by driveLinkBegin() before
// the task exists: false when the embedded image fails its check.
barnacle::UpdateSender g_sender(std::span<const uint8_t>(kShimImageStart, size_t(kShimImageEnd - kShimImageStart)));
bool g_updateOn = false;
const char* g_idleWhy = nullptr;   // this pass's helperUpdateRefusal()
uint32_t g_lastCommandMs = 0;
uint32_t g_progressQuarter = 0;    // the last 25 % step logged

// ---- the crossings ---------------------------------------------------------------

// r | g << 8 | b << 16, written by the BoardIo task (glow), read here.
std::atomic<uint32_t> g_rgb{0};
// The rest current the last home's probe read, written by the probing task.
std::atomic<int32_t> g_homeRestMa{barnacle::kNoCurrent};

StaticQueue_t g_readingQ;
std::array<uint8_t, sizeof(HelperReading)> g_readingStore{};
QueueHandle_t g_readingBox = nullptr;
// The published one's last-frame stamp, so a reader ages it without the task.
std::atomic<uint32_t> g_publishedOkMs{0};

StaticQueue_t g_statusQ;
std::array<uint8_t, sizeof(DriveLinkStatus)> g_statusStore{};
QueueHandle_t g_status = nullptr;

// ---- the UART ----------------------------------------------------------------------

bool uartBegin() {
    uart_config_t cfg{};
    cfg.baud_rate = int(barnacle::kBaud);
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    esp_err_t err = uart_driver_install(kUart, kRxRingBytes, kTxRingBytes, 0, nullptr, 0);
    if (err == ESP_OK) err = uart_param_config(kUart, &cfg);
    if (err == ESP_OK)
        err = uart_set_pin(kUart, BOARD_GPIO_LINK_TX, BOARD_GPIO_LINK_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Q5 only sinks: this pull-up is RX's only high.
    if (err == ESP_OK) err = gpio_pullup_en(pin(BOARD_GPIO_LINK_RX));
    if (err != ESP_OK) {
        GLOGE(kTag, "UART%d on TX G%d RX G%d failed (%s): no helper this boot, home refused",
              int(kUart), BOARD_GPIO_LINK_TX, BOARD_GPIO_LINK_RX, esp_err_to_name(err));
        return false;
    }
    return true;
}

// ---- the helper's update -----------------------------------------------------------------

unsigned helperVersion() {
    return g_sender.haveHelper() ? unsigned(g_sender.helper().shim_version) : unsigned(g_reading.slow.shim_version);
}

unsigned percentSent() {
    return g_sender.chunks() != 0 ? unsigned(g_sender.next() * 100 / g_sender.chunks()) : 0;
}

// The seconds the image takes on the wire alone, at kBaud 8N1.
unsigned wireSeconds() {
    const uint32_t bytes = g_sender.chunks() * uint32_t(barnacle::kMaxWire);
    return unsigned(bytes * 10 / barnacle::kBaud);
}

const char* failReason(const barnacle::SendStep& s) {
    using barnacle::SendEvent;
    using barnacle::SendState;
    if (s.ev == SendEvent::ack_error) return barnacle::ackCodeText(g_sender.lastCode());
    const bool otherImage = s.ev == SendEvent::ident_other || s.ev == SendEvent::ident_other_trial;
    if (s.from == SendState::restarting)
        return otherImage ? "the helper came back on its old firmware: the new image did not start"
                          : "the helper did not come back after restarting";
    if (s.from == SendState::keeping)
        return otherImage ? "the new firmware failed its link check, and the helper went back to its old one"
                          : "the helper went silent before confirming its new firmware";
    return "the helper stopped answering";
}

void logUpdateStep(const barnacle::SendStep& s, uint32_t now) {
    using barnacle::SendAct;
    using barnacle::SendEvent;
    using barnacle::SendState;
    if (s.ignored) {
        GLOGW_EVERY_MS(10000, kTag, "helper update: unexpected %s while %s, ignored (%lu so far)",
                       barnacle::sendEventName(s.ev), barnacle::sendStateName(s.from),
                       static_cast<unsigned long>(g_sender.ignored()));
        return;
    }
    if (s.ev == SendEvent::retry)
        GLOGW_EVERY_MS(5000, kTag, "helper update: no reply at %u %%, retransmitting (try %lu of %lu)", percentSent(),
                       static_cast<unsigned long>(g_sender.tries()), static_cast<unsigned long>(barnacle::kMaxTries));
    if (s.to == SendState::sending && s.act == SendAct::chunk && percentSent() / 25 > g_progressQuarter) {
        g_progressQuarter = percentSent() / 25;
        GLOGI(kTag, "helper update %u %%", g_progressQuarter * 25);
    }
    if (!s.changed()) return;
    const unsigned mine = unsigned(BARNACLE_SHIM_VERSION);
    switch (s.to) {
        case SendState::listen:
            break;
        case SendState::legacy:
            GLOGW(kTag, "helper firmware v%u cannot take updates over the link: flash helper firmware v%u once over "
                        "the R+D board's USB (pio run -d barnacle_shim -t upload); the helper works as before",
                  unsigned(g_reading.slow.shim_version), mine);
            break;
        case SendState::current:
            if (s.act == SendAct::done && g_sender.startedMs())
                GLOGW(kTag, "helper firmware updated to v%u in %lu s", helperVersion(),
                      static_cast<unsigned long>((now - *g_sender.startedMs()) / 1000));
            else if (s.act == SendAct::done)
                GLOGI(kTag, "helper firmware v%u confirmed: it matches the copy this machine carries", helperVersion());
            else
                GLOGI(kTag, "helper firmware v%u matches the copy this machine carries", helperVersion());
            break;
        case SendState::waiting:
            if (s.act == SendAct::abort)
                GLOGW(kTag, "helper update stopped at %u %%: %s; the helper keeps v%u, and the update starts over "
                            "once that stops",
                      percentSent(), g_idleWhy != nullptr ? g_idleWhy : "motion started", helperVersion());
            else if (s.ev == SendEvent::backoff_over)
                GLOGI(kTag, "helper update: retrying (attempt %lu of %lu)",
                      static_cast<unsigned long>(g_sender.attempts() + 1),
                      static_cast<unsigned long>(barnacle::kMaxAttempts));
            else
                GLOGW(kTag, "helper firmware v%u ready to install (the helper runs v%u): installs over the link "
                            "when no motion runs, takes at least %u s and unhomes the machine; %s",
                      mine, helperVersion(), wireSeconds(),
                      g_idleWhy != nullptr ? g_idleWhy : "starting now");
            break;
        case SendState::held:
            GLOGW(kTag, "the helper runs unconfirmed firmware v%u: its update waits until the helper confirms or "
                        "reverts it",
                  helperVersion());
            break;
        case SendState::beginning:
            g_progressQuarter = 0;
            GLOGW(kTag, "helper update started: v%u to v%u, %u KB in %lu chunks; the machine is unhomed, so home "
                        "it again once the update finishes",
                  helperVersion(), mine, unsigned(g_sender.imageBytes() / 1024),
                  static_cast<unsigned long>(g_sender.chunks()));
            break;
        case SendState::sending:
            break;
        case SendState::ending:
            GLOGI(kTag, "helper update: all chunks sent, the helper is checking the image");
            break;
        case SendState::restarting:
            GLOGW(kTag, "helper image verified: the helper restarts on v%u", mine);
            break;
        case SendState::keeping:
            GLOGI(kTag, "helper firmware runs unconfirmed: confirming the link so the helper keeps it");
            break;
        case SendState::failed:
            if (g_sender.attempts() < barnacle::kMaxAttempts)
                GLOGE(kTag, "helper update failed: %s; it keeps v%u, trying again in %lu s", failReason(s),
                      helperVersion(), static_cast<unsigned long>(barnacle::kRetryBackoffMs / 1000));
            else
                GLOGE(kTag, "helper update failed: %s; it keeps v%u", failReason(s), helperVersion());
            break;
        case SendState::gave_up:
            GLOGE(kTag, "helper update stopped after %lu failed attempts: the helper keeps v%u and works as "
                        "before; reboot the machine to try again, or flash the helper firmware over the R+D "
                        "board's USB",
                  static_cast<unsigned long>(g_sender.attempts()), helperVersion());
            break;
    }
}

// Sends what the step's action sends, then logs it. Link task only. An
// attempt's start unhomes before its BEGIN leaves.
void onUpdateStep(const barnacle::SendStep& s, uint32_t now) {
    if (s.act == barnacle::SendAct::start) motionUnhome();
    const size_t n = g_sender.frameFor(s.act, g_txSeq, g_out);
    if (n > 0 && g_uartUp) {
        ++g_txSeq;
        uart_write_bytes(kUart, g_out.data(), n);
    }
    logUpdateStep(s, now);
}

// The embedded copy against the header the build wrote beside it.
bool shimImageSound() {
    constexpr std::array<uint8_t, barnacle::kDigestBytes> kDigest = BARNACLE_SHIM_DIGEST;
    return g_sender.valid() && g_sender.imageBytes() == size_t(BARNACLE_SHIM_BYTES) && g_sender.digest() == kDigest;
}

// ---- the helper's frames --------------------------------------------------------------

void onSlow(const barnacle::Slow& s) {
    if (g_reading.have_slow && helperRestarted(g_reading.slow, s)) {
        ++g_reading.restarts;
        GLOGW(kTag, "helper RESTARTED: boot %u, reset reason %u, up %lu ms",
              unsigned(s.boots), unsigned(s.reset_reason), static_cast<unsigned long>(s.uptime_ms));
        g_seq.reset();
    } else if (!g_reading.have_slow) {
        GLOGI(kTag, "helper shim v%u, boot %u (reset reason %u), %u trips so far",
              unsigned(s.shim_version), unsigned(s.boots), unsigned(s.reset_reason), unsigned(s.trips));
    }
    g_reading.slow = s;
    g_reading.have_slow = true;
}

void onFrame(const barnacle::Frame& f, uint32_t now) {
    if (const std::optional<barnacle::Ident> d = barnacle::asIdent(f)) {
        if (g_updateOn) onUpdateStep(g_sender.onIdent(*d, now), now);
    } else if (const std::optional<barnacle::UpdateAck> a = barnacle::asUpdateAck(f)) {
        if (g_updateOn) onUpdateStep(g_sender.onAck(*a, now), now);
    } else if (const std::optional<barnacle::Status> s = barnacle::asStatus(f)) {
        g_reading.status = *s;
        const bool none = s->current_ma == barnacle::kNoCurrent;
        if (none) {
            g_rest.reset();
            g_reading.rest_ma = barnacle::kNoCurrent;
        } else if (const std::optional<int32_t> m = g_rest.push(s->current_ma)) {
            g_reading.rest_ma = int16_t(*m);
        }
        if (none != g_noCurrent) {
            g_noCurrent = none;
            if (none) GLOGE(kTag, "the helper reads no motor current (no ADC on IO36, or no 0 A zero measured at idle yet): home refused");
            else      GLOGW(kTag, "the helper reads motor current again");
        }
    } else if (const std::optional<barnacle::Slow> s = barnacle::asSlow(f)) {
        onSlow(*s);
        if (g_updateOn)
            if (const std::optional<barnacle::SendStep> st = g_sender.onSlow(now)) onUpdateStep(*st, now);
    } else {
        GLOGW_EVERY_MS(10000, kTag, "frame type 0x%02x, %u B: not a helper frame of this version",
                       unsigned(f.type), unsigned(f.len));
        return;
    }
    g_seq.note(f.seq);
    g_reading.ever = true;
    g_lastOkMs = now;
}

void drainUart(uint32_t now) {
    std::array<uint8_t, kRxChunk> chunk{};
    for (;;) {
        const int n = uart_read_bytes(kUart, chunk.data(), uint32_t(chunk.size()), 0);
        if (n <= 0) return;
        for (int i = 0; i < n; ++i)
            if (const std::optional<barnacle::Frame> f = g_decoder.feed(chunk[size_t(i)])) onFrame(*f, now);
        if (size_t(n) < chunk.size()) return;
    }
}

// ---- freshness, mode, TRIP --------------------------------------------------------------

void judgeFreshness(uint32_t now) {
    const bool fresh = g_reading.ever && now - g_lastOkMs < barnacle::kHelperStaleMs;
    g_reading.fresh = fresh;
    g_reading.age_ms = g_reading.ever ? now - g_lastOkMs : 0;
    if (!g_reading.ever && !g_neverLogged && now - g_startMs >= kNeverHeardMs) {
        g_neverLogged = true;
        GLOGE(kTag, "no frame from the helper in %lu ms (bytes seen: %s): motor current, voltage and "
                    "S2 unread, home refused; check J8 pin 9 to pad 19 and the shim",
              static_cast<unsigned long>(kNeverHeardMs),
              (g_decoder.stats().crc + g_decoder.stats().malformed + g_decoder.stats().overrun) != 0
                  ? "yes, none valid" : "none");
    }
    if (fresh == g_wasFresh) return;
    g_wasFresh = fresh;
    const barnacle::RxStats& rx = g_decoder.stats();
    // Its own flash erases and its restart quiet the helper during an update:
    // the update's lines say how it goes.
    if (g_sender.updating()) {
        if (!fresh) {
            g_seq.reset();
            g_rest.reset();
            g_reading.rest_ma = barnacle::kNoCurrent;
        }
        GLOGI(kTag, "helper link %s during its update", fresh ? "back" : "quiet");
        return;
    }
    if (fresh) {
        GLOGW(kTag, "helper link UP: frames %lu, crc %lu, lost %lu",
              static_cast<unsigned long>(rx.frames), static_cast<unsigned long>(rx.crc),
              static_cast<unsigned long>(g_seq.lost()));
        return;
    }
    g_seq.reset();
    g_rest.reset();
    g_reading.rest_ma = barnacle::kNoCurrent;
    GLOGE(kTag, "helper link LOST: no valid frame for %lu ms (crc %lu, malformed %lu, overrun %lu): "
                "motor current, voltage and S2 unread, home refused",
          static_cast<unsigned long>(barnacle::kHelperStaleMs), static_cast<unsigned long>(rx.crc),
          static_cast<unsigned long>(rx.malformed), static_cast<unsigned long>(rx.overrun));
    if (g_mode == barnacle::Mode::homing)
        GLOGE(kTag, "helper lost during a home cycle: its stall sense is gone, the home fails unhomed");
}

void watchTrip(bool tripped, const MotionCensus& mo) {
    if (g_strayWatch.sample(tripped, g_mode))
        GLOGW(kTag, "ANOMALY: TRIP while %s at %.3f mm, but the helper's trip is active only for homing; "
                    "nothing stops: if it repeats, check J8 pin 7 and the shim version",
              barnacle::modeName(g_mode), double(mo.position_mm));
    // The pad against the helper's own report: a wrong J8 pin 7 crimp or a
    // dead Q4 shows here first.
    const bool helperSays = (g_reading.status.flags & barnacle::kFlagTrip) != 0;
    if (!g_reading.fresh || helperSays == tripped) {
        g_disagree = 0;
        return;
    }
    if (++g_disagree == kTripDisagreePasses)
        GLOGE(kTag, "TRIP reads %s at the P4, the helper reports %s: check J8 pin 7 to pad 7 and the trip circuit (Q4)",
              tripped ? "tripped" : "clear", helperSays ? "tripped" : "clear");
}

void sendCommand(uint32_t now) {
    if (g_sender.transferring() && now - g_lastCommandMs < barnacle::kUpdateCommandMs) return;
    g_lastCommandMs = now;
    const uint32_t rgb = g_rgb.load(std::memory_order_relaxed);
    barnacle::Command c;
    c.mode = g_mode;
    c.r = uint8_t(rgb);
    c.g = uint8_t(rgb >> 8);
    c.b = uint8_t(rgb >> 16);
    c.home_trip_ma = homeTripFrom(kHomeTripMa, int16_t(g_homeRestMa.load(std::memory_order_relaxed)));
    c.run_trip_ma = kRunTripMa;
    const size_t n = barnacle::encode(c, g_txSeq++, g_out);
    if (n > 0) uart_write_bytes(kUart, g_out.data(), n);
}

DriveLinkStatus compose(bool tripped) {
    DriveLinkStatus s;
    s.built = false;    // no Modbus link on this board
    s.uart = false;     // and no RS485 port
    s.alarm = tripped;
    s.ready = g_reading.fresh;
    s.armed = g_mode == barnacle::Mode::running;
    return s;
}

void census(uint32_t now) {
    if (int32_t(now - g_nextCensusMs) < 0) return;
    g_nextCensusMs = now + kCensusEveryMs;
    const barnacle::RxStats& rx = g_decoder.stats();
    GLOGI(kTag, "%s mode=%s upd=%s frames=%lu crc=%lu malformed=%lu version=%lu overrun=%lu lost=%lu "
                "restarts=%lu I=%d V=%u mV trips=%u p4bad=%u",
          g_reading.fresh ? "fresh" : "STALE", barnacle::modeName(g_mode),
          g_updateOn ? barnacle::sendStateName(g_sender.state()) : "off",
          static_cast<unsigned long>(rx.frames), static_cast<unsigned long>(rx.crc),
          static_cast<unsigned long>(rx.malformed), static_cast<unsigned long>(rx.version),
          static_cast<unsigned long>(rx.overrun), static_cast<unsigned long>(g_seq.lost()),
          static_cast<unsigned long>(g_reading.restarts), int(g_reading.status.current_ma),
          unsigned(g_reading.status.motor_mv), unsigned(g_reading.slow.trips),
          unsigned(g_reading.slow.bad_frames));
}

void taskMain(void*) {
    g_startMs = nowMs();
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(kDriveLinkPeriodMs));
        const uint32_t now = nowMs();
        if (g_uartUp) drainUart(now);
        judgeFreshness(now);
        const MotionCensus mo = motionCensus();
        const motorswitch::State power = motorSwitchStatus().state;
        const bool zeroOwed = g_reading.status.current_ma == barnacle::kNoCurrent && !motionRuns(mo);
        g_mode = modeFor(mo.homing, power == motorswitch::State::on, mo.estop, zeroOwed);
        const bool tripped = padTrippedConfirmed();
        watchTrip(tripped, mo);
        if (g_updateOn) {
            g_idleWhy = helperUpdateRefusal(motionRuns(mo), otaInFlight());
            if (const std::optional<barnacle::SendStep> st = g_sender.tick(now, g_idleWhy == nullptr, g_reading.fresh))
                onUpdateStep(*st, now);
        }
        if (g_uartUp) sendCommand(now);
        g_reading.rx = g_decoder.stats();
        g_reading.lost = g_seq.lost();
        g_publishedOkMs.store(g_lastOkMs, std::memory_order_relaxed);
        xQueueOverwrite(g_readingBox, &g_reading);
        const DriveLinkStatus st = compose(tripped);
        xQueueOverwrite(g_status, &st);
        census(now);
    }
}

// ---- the home sense ------------------------------------------------------------------

std::atomic<uint32_t> g_fell_us{0};   // esp_timer's low word at the last trip edge
std::atomic<bool> g_fell{false};
std::atomic<HomeSenseIsr> g_rose_cb{nullptr};
std::atomic<HomeSenseIsr> g_wake{nullptr};

// The park first: nothing ahead of it but the dispatcher.
void onTrip(void*) {
    if (const HomeSenseIsr rose = g_rose_cb.load(std::memory_order_relaxed)) rose();
    g_fell_us.store(nowUs32(), std::memory_order_relaxed);
    g_fell.store(true, std::memory_order_release);
    if (const HomeSenseIsr wake = g_wake.load(std::memory_order_relaxed)) wake();
}

class TripHomeSense final : public HomeSense {
public:
    bool present() const override { return _ready; }

    // Planner task, during a cycle: the stall source is the helper, so a
    // stale helper is a lost sense.
    const char* lost() override {
        if (helperReading().fresh) return nullptr;
        return "the R+D board's helper went silent, so its stall sense is gone: check the J8 link, then home again";
    }

    // The line idles high whether or not a helper drives it, so the helper's
    // freshness is what says the stall source is there.
    Probe probe() override {
        if (!_ready) return Probe::undriven;
        const HelperReading h = helperReading();
        if (!h.fresh) return Probe::undriven;
        if (h.status.current_ma == barnacle::kNoCurrent) {
            GLOGE(kTag, "home refused: the helper reads no motor current (IO36 ADC or its 0 A zero), so no stall can trip");
            return Probe::undriven;
        }
        if (h.rest_ma == barnacle::kNoCurrent) {
            GLOGE(kTag, "home refused: the helper's rest current is not measured yet; retry the home");
            return Probe::undriven;
        }
        if (!barnacle::restCurrentSane(h.rest_ma, kHomeTripMa)) {
            GLOGE(kTag, "home refused: the motor reads %d mA before moving, at least half the %u mA home trip, so "
                        "the home would stall where it stands; the helper's 0 A zero is off: power-cycle the R+D "
                        "board with the motor at rest",
                  int(h.rest_ma), unsigned(kHomeTripMa));
            return Probe::undriven;
        }
        g_homeRestMa.store(h.rest_ma, std::memory_order_relaxed);
        GLOGI(kTag, "home trip %u mA: %u over a %d mA rest (provisional scale until calibrated)",
              unsigned(homeTripFrom(kHomeTripMa, h.rest_ma)), unsigned(kHomeTripMa), int(h.rest_ma));
        const bool tripped = padTripped();
        const bool helperSays = (h.status.flags & barnacle::kFlagTrip) != 0;
        if (tripped != helperSays) {
            GLOGE(kTag, "home refused: TRIP reads %s, the helper reports %s (J8 pin 7 or the trip circuit, Q4)",
                  tripped ? "tripped" : "clear", helperSays ? "tripped" : "clear");
            return Probe::undriven;
        }
        g_fell.store(false, std::memory_order_relaxed);
        return tripped ? Probe::high : Probe::low;
    }

    bool high() override {
        if (!_ready || !padTripped()) return false;
        if (!g_fell.exchange(false, std::memory_order_acquire)) return true;
        const uint32_t age = nowUs32() - g_fell_us.load(std::memory_order_relaxed);
        if (age >= kHomeSenseDebounceUs) return true;
        esp_rom_delay_us(kHomeSenseDebounceUs - age);
        return padTripped();
    }

    void begin(HomeSenseIsr rose, HomeSenseIsr wake) {
        g_rose_cb.store(rose, std::memory_order_relaxed);
        g_wake.store(wake, std::memory_order_relaxed);
        gpio_set_intr_type(kTripPad, GPIO_INTR_NEGEDGE);
        const esp_err_t svc = gpio_install_isr_service(0);
        if (svc != ESP_OK && svc != ESP_ERR_INVALID_STATE) {
            GLOGE(kTag, "TRIP pin LPG%d: GPIO interrupt service failed (%s): home refused", BOARD_GPIO_TRIP,
                  esp_err_to_name(svc));
            return;
        }
        if (gpio_isr_handler_add(kTripPad, &onTrip, nullptr) != ESP_OK || gpio_intr_enable(kTripPad) != ESP_OK) {
            GLOGE(kTag, "TRIP pin LPG%d: falling-edge interrupt not enabled: home refused", BOARD_GPIO_TRIP);
            return;
        }
        _ready = true;
        GLOGI(kTag, "home sense is TRIP on LPG%d: active low, pull-up, the helper's stall (reads %s)",
              BOARD_GPIO_TRIP, padTripped() ? "tripped" : "clear");
    }

private:
    bool _ready = false;
};

TripHomeSense g_sense;

}  // namespace

// ---- the drive-link door ---------------------------------------------------------------

void driveLinkHoldIdle() {
    // TRIP's pull-up before anything reads it: Q4 only sinks.
    gpio_set_direction(kTripPad, GPIO_MODE_INPUT);
    gpio_set_pull_mode(kTripPad, GPIO_PULLUP_ONLY);
}

bool driveLinkBegin() {
    g_readingBox = xQueueCreateStatic(1, sizeof(HelperReading), g_readingStore.data(), &g_readingQ);
    g_status = xQueueCreateStatic(1, sizeof(DriveLinkStatus), g_statusStore.data(), &g_statusQ);
    xQueueOverwrite(g_readingBox, &g_reading);
    const DriveLinkStatus first = compose(false);
    xQueueOverwrite(g_status, &first);
    // A failed UART still runs the task: the TRIP anomaly watch is a pad read.
    g_uartUp = uartBegin();
    g_updateOn = g_uartUp && shimImageSound();
    if (g_updateOn)
        GLOGI(kTag, "helper firmware copy: v%u, %u B, SHA-256 %02x%02x%02x%02x", unsigned(BARNACLE_SHIM_VERSION),
              unsigned(g_sender.imageBytes()), g_sender.digest()[0], g_sender.digest()[1], g_sender.digest()[2],
              g_sender.digest()[3]);
    else if (g_uartUp)
        GLOGE(kTag, "helper firmware copy in this image failed its check (%u B, expected %u): the helper is "
                    "not updated this boot; rebuild the barnacle image",
              unsigned(g_sender.imageBytes()), unsigned(BARNACLE_SHIM_BYTES));
    if (xTaskCreatePinnedToCore(&taskMain, "DriveLnk", kDriveLinkTaskStackBytes, nullptr, 2, &g_task, 0) !=
        pdPASS) {
        g_task = nullptr;
        GLOGE(kTag, "helper link task did not start: no helper this boot, home refused");
        return false;
    }
    if (g_uartUp)
        GLOGI(kTag, "helper link on UART%d, TX G%d RX G%d, %lu 8N1; TRIP on LPG%d", int(kUart),
              BOARD_GPIO_LINK_TX, BOARD_GPIO_LINK_RX, static_cast<unsigned long>(barnacle::kBaud), BOARD_GPIO_TRIP);
    return true;
}

DriveLinkStatus driveLinkStatus() {
    DriveLinkStatus s;
    if (g_status != nullptr) xQueuePeek(g_status, &s, 0);
    return s;
}

// Nothing on this board raises a drive alarm (bd val-4fgu).
bool driveAlarmTake() { return false; }

// No Modbus on this board: every read is refused at once.
bool driveLinkRead(const DriveRead&) { return false; }
bool driveLinkTakeResult(DriveReadResult&) { return false; }

uint32_t driveLinkStackFree() {
    // IDF reports BYTES, not the vanilla FreeRTOS words.
    return g_task != nullptr ? uint32_t(uxTaskGetStackHighWaterMark(g_task)) : 0;
}

// ---- the home-sense door -----------------------------------------------------------------

HomeSense& homeSenseBegin(HomeSenseIsr rose, HomeSenseIsr wake) {
    g_sense.begin(rose, wake);
    return g_sense;
}

// ---- the helper surface ------------------------------------------------------------------

HelperReading helperReading() {
    HelperReading h;
    if (g_readingBox == nullptr || xQueuePeek(g_readingBox, &h, 0) != pdTRUE) return h;
    // Aged here, not by the task: a stalled link task must read stale too.
    if (h.ever) {
        h.age_ms = nowMs() - g_publishedOkMs.load(std::memory_order_relaxed);
        h.fresh = h.age_ms < barnacle::kHelperStaleMs;
    }
    return h;
}

void helperSetRgb(uint8_t r, uint8_t g, uint8_t b) {
    g_rgb.store(uint32_t(r) | (uint32_t(g) << 8) | (uint32_t(b) << 16), std::memory_order_relaxed);
}

std::optional<bool> helperButtonPressed() {
    const HelperReading h = helperReading();
    if (!h.fresh) return std::nullopt;
    return (h.status.flags & barnacle::kFlagButton) != 0;
}

void helperAwaitZero() {
    const uint32_t start = nowMs();
    for (;;) {
        const HelperReading h = helperReading();
        if (h.fresh && h.status.current_ma != barnacle::kNoCurrent) return;
        const uint32_t waited = nowMs() - start;
        if (waited >= kHelperZeroWaitMs || (!h.ever && waited >= kPairAtBootWaitMs)) return;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

bool barnaclePairHeldAtBoot() {
    const uint32_t until = nowMs() + kPairAtBootWaitMs;
    while (!helperReading().fresh && int32_t(until - nowMs()) > 0) vTaskDelay(pdMS_TO_TICKS(10));
    for (int i = 0; i < 5; ++i) {
        if (i) vTaskDelay(pdMS_TO_TICKS(10));
        if (helperButtonPressed() != std::optional<bool>(true)) return false;
    }
    return true;
}

}  // namespace valence
