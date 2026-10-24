// BarnacleLink -- the barnacle's link host: the helper UART and its task (as
// the drive-link door), TRIP as the home sense and the running stop, and the
// helper surface (BarnacleLink.h)
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
// - high() and the running watch read the pad twice kHomeSenseDebounceUs
//   apart before they believe a trip; the helper filters the current before
//   it pulls the line (barnacle::kTripOnSamples).
// - The UART has a kRxRingBytes ring and no TX ring: a command frame is under
//   barnacle::kMaxWire bytes, far inside the 128-byte TX FIFO, so
//   uart_write_bytes() copies and returns.
// - BSS only: the decoder (~40 B), the reading (~90 B), the queue storage and
//   a few words. Nothing allocates after begin.
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

#include "geiger/geiger.h"
#include "motion/MotionArbiter.h"
#include "motion/ValenceMotion.h"
#include "system/BoardPins.h"
#include "system/ValenceDriveLink.h"
#include "system/ValenceHomeSense.h"
#include "system/ValenceMotorSwitch.h"

#ifndef CONFIG_NUCLEUS_BOARD_BARNACLE
#error "BarnacleLink.cpp is the barnacle's host; src/CMakeLists.txt links it only there"
#endif

namespace valence {

namespace {

constexpr const char* kTag = "helper";

constexpr uart_port_t kUart = uart_port_t(BOARD_UART_LINK);
static_assert(BOARD_UART_LINK >= 1 && BOARD_UART_LINK < SOC_UART_HP_NUM, "an HP UART, never UART0");
// Two passes of status frames at 500 Hz is ~70 B; the ring holds an outage of
// the task several times that long.
constexpr int kRxRingBytes = 1024;
// Bytes read per uart_read_bytes() call; the pass loops until the ring is dry.
constexpr size_t kRxChunk = 128;
// The counters' Info line in /diag.
constexpr uint32_t kCensusEveryMs = 60000;
// A helper never heard this long after the link task starts is logged once.
constexpr uint32_t kNeverHeardMs = 3000;

static_assert(barnacle::kTripDetectUs <= kStallDetectUs,
              "the helper's stall detect must fit the arbiter's contact-to-park budget");
static_assert(barnacle::kCommandPeriodMs == kDriveLinkPeriodMs,
              "one command frame per link pass");
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
RunTripWatch g_runWatch;
HelperReading g_reading;        // what the task last published
uint32_t g_lastOkMs = 0;
bool g_wasFresh = false;
uint8_t g_txSeq = 0;
uint32_t g_disagree = 0;        // consecutive passes TRIP and the helper's flag disagree
uint32_t g_nextCensusMs = 0;
uint32_t g_startMs = 0;
bool g_neverLogged = false;
bool g_noCurrent = false;       // the helper's last status carried no current
barnacle::Mode g_mode = barnacle::Mode::idle;
uint32_t g_alarms = 0;
bool g_uartUp = false;          // written once by driveLinkBegin() before the task exists
TaskHandle_t g_task = nullptr;

// ---- the crossings ---------------------------------------------------------------

std::atomic<bool> g_alarmPending{false};
// r | g << 8 | b << 16, written by the BoardIo task (glow), read here.
std::atomic<uint32_t> g_rgb{0};

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
    esp_err_t err = uart_driver_install(kUart, kRxRingBytes, 0, 0, nullptr, 0);
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
    if (const std::optional<barnacle::Status> s = barnacle::asStatus(f)) {
        g_reading.status = *s;
        const bool none = s->current_ma == barnacle::kNoCurrent;
        if (none != g_noCurrent) {
            g_noCurrent = none;
            if (none) GLOGE(kTag, "the helper reads no motor current (its ADC on IO36): home refused");
            else      GLOGW(kTag, "the helper reads motor current again");
        }
    } else if (const std::optional<barnacle::Slow> s = barnacle::asSlow(f)) {
        onSlow(*s);
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
    if (fresh) {
        GLOGW(kTag, "helper link UP: frames %lu, crc %lu, lost %lu",
              static_cast<unsigned long>(rx.frames), static_cast<unsigned long>(rx.crc),
              static_cast<unsigned long>(g_seq.lost()));
        return;
    }
    g_seq.reset();
    GLOGE(kTag, "helper link LOST: no valid frame for %lu ms (crc %lu, malformed %lu, overrun %lu): "
                "motor current, voltage and S2 unread, home refused",
          static_cast<unsigned long>(barnacle::kHelperStaleMs), static_cast<unsigned long>(rx.crc),
          static_cast<unsigned long>(rx.malformed), static_cast<unsigned long>(rx.overrun));
    if (g_mode == barnacle::Mode::homing) {
        GLOGE(kTag, "helper lost during a home cycle: its stall sense is gone, the hub latches ESTOP");
        ++g_alarms;
        g_alarmPending.store(true, std::memory_order_release);
    }
}

void watchTrip(bool tripped, const MotionCensus& mo) {
    if (g_runWatch.sample(tripped, g_mode)) {
        ++g_alarms;
        GLOGE(kTag, "TRIP while running at %.3f mm: the helper saw motor current past %u mA "
                    "(provisional scale until calibrated); the hub latches ESTOP",
              double(mo.position_mm), unsigned(kRunTripMa));
        g_alarmPending.store(true, std::memory_order_release);
    }
    // The pad against the helper's own report: a wrong J8 pin 7 crimp or a
    // dead Q4 shows here first.
    const bool helperSays = (g_reading.status.flags & barnacle::kFlagTrip) != 0;
    if (!g_reading.fresh || helperSays == tripped) {
        g_disagree = 0;
        return;
    }
    if (++g_disagree == kTripDisagreePasses)
        GLOGE(kTag, "TRIP reads %s at the P4, the helper reports %s: check J8 pin 7 to pad 7 and Q4",
              tripped ? "tripped" : "clear", helperSays ? "tripped" : "clear");
}

void sendCommand() {
    const uint32_t rgb = g_rgb.load(std::memory_order_relaxed);
    barnacle::Command c;
    c.mode = g_mode;
    c.r = uint8_t(rgb);
    c.g = uint8_t(rgb >> 8);
    c.b = uint8_t(rgb >> 16);
    c.home_trip_ma = kHomeTripMa;
    c.run_trip_ma = kRunTripMa;
    std::array<uint8_t, barnacle::kMaxWire> out{};
    const size_t n = barnacle::encode(c, g_txSeq++, out);
    if (n > 0) uart_write_bytes(kUart, out.data(), n);
}

DriveLinkStatus compose(bool tripped) {
    DriveLinkStatus s;
    s.built = false;    // no Modbus link on this board
    s.uart = false;     // and no RS485 port
    s.alarm = tripped;
    s.ready = g_reading.fresh;
    s.armed = g_mode == barnacle::Mode::running;
    s.alarms = g_alarms;
    return s;
}

void census(uint32_t now) {
    if (int32_t(now - g_nextCensusMs) < 0) return;
    g_nextCensusMs = now + kCensusEveryMs;
    const barnacle::RxStats& rx = g_decoder.stats();
    GLOGI(kTag, "%s mode=%s frames=%lu crc=%lu malformed=%lu version=%lu overrun=%lu lost=%lu "
                "restarts=%lu I=%d V=%u mV trips=%u p4bad=%u",
          g_reading.fresh ? "fresh" : "STALE", barnacle::modeName(g_mode),
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
        g_mode = modeFor(mo.homing, motorSwitchStatus().state == motorswitch::State::on, mo.estop);
        const bool tripped = padTrippedConfirmed();
        watchTrip(tripped, mo);
        if (g_uartUp) sendCommand();
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

    // The line idles high whether or not a helper drives it, so the helper's
    // freshness is what says the stall source is there.
    Probe probe() override {
        if (!_ready) return Probe::undriven;
        const HelperReading h = helperReading();
        if (!h.fresh) return Probe::undriven;
        if (h.status.current_ma == barnacle::kNoCurrent) {
            GLOGE(kTag, "home refused: the helper reads no motor current (IO36), so no stall can trip");
            return Probe::undriven;
        }
        const bool tripped = padTripped();
        const bool helperSays = (h.status.flags & barnacle::kFlagTrip) != 0;
        if (tripped != helperSays) {
            GLOGE(kTag, "home refused: TRIP reads %s, the helper reports %s (J8 pin 7 or Q4)",
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
            GLOGE(kTag, "LPG%d: GPIO ISR service failed (%s): home refused", BOARD_GPIO_TRIP,
                  esp_err_to_name(svc));
            return;
        }
        if (gpio_isr_handler_add(kTripPad, &onTrip, nullptr) != ESP_OK || gpio_intr_enable(kTripPad) != ESP_OK) {
            GLOGE(kTag, "LPG%d: falling-edge interrupt not armed: home refused", BOARD_GPIO_TRIP);
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
    // A failed UART still runs the task: TRIP while running is a pad read.
    g_uartUp = uartBegin();
    if (xTaskCreatePinnedToCore(&taskMain, "DriveLnk", kDriveLinkTaskStackBytes, nullptr, 2, &g_task, 0) !=
        pdPASS) {
        g_task = nullptr;
        GLOGE(kTag, "helper link task did not start: no helper this boot, TRIP unwatched while running");
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

bool driveAlarmTake() { return g_alarmPending.exchange(false, std::memory_order_acq_rel); }

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
