// barnacle_shim -- the OSSM R+D board's ESP32-WROOM-32D as nucleus-p4b's I/O
// helper: motor current, motor voltage, S2 and the RGB pixel over the
// barnacle link, and TRIP on a current spike
// Constraints:
// - NEVER UART0 (IO1/IO3): the CP2102 owns it, and it is how this board is
//   flashed. Nothing here calls Serial; the link is UART1 on IO13/IO14.
// - The radio is never started: no WiFi or BT call anywhere in this image.
//   The CPU runs at kCpuMhz from the first lines of setup(); APB stays 80 MHz,
//   so the UART's baud is exact.
// - TRIP is IO27 into Q4's base: HIGH pulls J8 pin 7 low, which is TRIPPED.
//   It is driven LOW before anything else in setup(), so a boot never trips.
// - The link's TX is IO14 into Q5, which inverts: the UART inverts TXD so the
//   wire carries normal UART. RX on IO13 is the P4's push-pull drive, not
//   inverted.
// - ONE LOOP, on the Arduino loop task, paced by esp_timer: one current
//   sample every barnacle::kTripSampleUs, the UART drained while it waits. A
//   sample later than two periods restarts the cadence and counts in
//   late_samples. ADC1 is read through ESP-IDF oneshot only (never
//   analogRead), so its unit has one owner.
// - The P4's command sets the mode, the thresholds and the color. Silent past
//   barnacle::kP4SilentMs, the shim holds the last mode and thresholds it was
//   given (the boot defaults if none), sets kFlagP4Silent, and shows the Link
//   system's Degraded look itself through Flux.
// - Current reads provisional mA until kCurrentCalibrated (bd val-omh3.4).
// See: ../lib/barnacle_link/include/barnacle_link/barnacle_link.hpp,
// openvalence.github.io guides/barnacle (the WROOM helper pin map), bd
// val-omh3.3

#include <Arduino.h>

#include <array>
#include <cstdint>
#include <optional>

#include <driver/uart.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_timer.h>

#include "barnacle_link/barnacle_link.hpp"
#include "flux/flux_core.hpp"

namespace {

// ---- pins: the WROOM helper pin map (guides/barnacle) ----------------------------

constexpr int kPinLinkRx = 13;     // J8 pin 2, from the P4's G27; 10k pull-up on the board
constexpr int kPinLinkTx = 14;     // Q5 to J8 pin 9, to the P4's G26
constexpr int kPinTrip = 27;       // Q4 to J8 pin 7: HIGH = tripped on the wire
constexpr int kPinSpareOut = 26;   // Q3 to J8 pin 5, held off
constexpr int kPinSpareIn = 4;     // J8 pin 4
constexpr int kPinButton = 23;     // S2, read active low
constexpr int kPinRgb = 25;        // WS2813 data
// ADC1 channels: IO36 CH0 motor current (CH701 through 10k over 20k), IO39
// CH3 motor voltage (1:40), IO35 CH7 ANA1, IO34 CH6 ANA2.
constexpr adc_channel_t kChCurrent = ADC_CHANNEL_0;
constexpr adc_channel_t kChMotorV = ADC_CHANNEL_3;
constexpr adc_channel_t kChAna1 = ADC_CHANNEL_7;
constexpr adc_channel_t kChAna2 = ADC_CHANNEL_6;

constexpr uart_port_t kUart = UART_NUM_1;

// ---- THE KNOBS ---------------------------------------------------------------------

constexpr uint8_t kShimVersion = 1;
constexpr uint32_t kCpuMhz = 80;

// The CH701's output at 0 A and its slope, both at the sensor pin (before the
// divider). Placeholders: the variant on U6 is not on record.
// TODO(val-omh3.4): read the part marking or calibrate against a clamp meter,
// then set both and kCurrentCalibrated.
constexpr int32_t kCurrentZeroMv = 1650;
constexpr int32_t kCurrentMvPerA = 100;
constexpr bool kCurrentCalibrated = false;
// 10k over 20k: the sensor's volts are the pin's times 3/2.
constexpr int32_t kCurrentDividerNum = 3;
constexpr int32_t kCurrentDividerDen = 2;
// 1:40: the J2 supply's volts are the pin's times this.
// TODO(val-omh3.5): check the ratio against a meter.
constexpr int32_t kMotorVoltsPerPinVolt = 40;

// One slow channel (motor voltage, ANA1, ANA2 in turn) per this many current
// samples: each refreshes every ~9 ms.
constexpr uint32_t kSlowReadEvery = 20;
// The WS2813's per-channel ceiling on top of the P4's own brightness, and the
// fastest it is rewritten.
constexpr uint8_t kRgbCap = 96;
constexpr uint32_t kRgbMinMs = 20;
// The shim's own look while the P4 is silent (Flux), before its gamma.
constexpr uint8_t kOwnBrightness = 96;

// ---- boot count, across resets but not a power cycle -------------------------------

constexpr uint32_t kBootMagic = 0xBA2AC1E5u;
RTC_NOINIT_ATTR uint32_t g_bootMagic;
RTC_NOINIT_ATTR uint16_t g_bootCount;

// ---- ADC1 --------------------------------------------------------------------------

class Adc1 {
public:
    bool begin() {
        adc_oneshot_unit_init_cfg_t unit{};
        unit.unit_id = ADC_UNIT_1;
        if (adc_oneshot_new_unit(&unit, &_unit) != ESP_OK) return false;
        adc_oneshot_chan_cfg_t ch{};
        ch.atten = ADC_ATTEN_DB_12;
        ch.bitwidth = ADC_BITWIDTH_12;
        for (const adc_channel_t c : {kChCurrent, kChMotorV, kChAna1, kChAna2})
            if (adc_oneshot_config_channel(_unit, c, &ch) != ESP_OK) return false;
        adc_cali_line_fitting_config_t cal{};
        cal.unit_id = ADC_UNIT_1;
        cal.atten = ADC_ATTEN_DB_12;
        cal.bitwidth = ADC_BITWIDTH_12;
        _calibrated = adc_cali_create_scheme_line_fitting(&cal, &_cali) == ESP_OK;
        _ok = true;
        return true;
    }

    // Raw counts and pin millivolts; nullopt on a failed read or no calibration.
    struct Sample {
        int raw = 0;
        int mv = 0;
    };
    std::optional<Sample> read(adc_channel_t c) {
        if (!_ok || !_calibrated) return std::nullopt;
        Sample s;
        if (adc_oneshot_read(_unit, c, &s.raw) != ESP_OK) return std::nullopt;
        if (adc_cali_raw_to_voltage(_cali, s.raw, &s.mv) != ESP_OK) return std::nullopt;
        return s;
    }

private:
    adc_oneshot_unit_handle_t _unit = nullptr;
    adc_cali_handle_t _cali = nullptr;
    bool _ok = false;
    bool _calibrated = false;
};

// ---- the pixel ---------------------------------------------------------------------

void writeRgb(uint8_t r, uint8_t g, uint8_t b) {
    rgbLedWrite(kPinRgb, r < kRgbCap ? r : kRgbCap, g < kRgbCap ? g : kRgbCap, b < kRgbCap ? b : kRgbCap);
}

// The shim's own Flux output: gamma here, the cap in writeRgb().
class ShimPixel final : public flux::IGlowOutput {
public:
    size_t pixelCount() const override { return 1; }
    void set(size_t, flux::Rgb c) override { _pending = c; }
    void show() override { _shown = {flux::gamma8(_pending.r), flux::gamma8(_pending.g), flux::gamma8(_pending.b)}; }
    flux::Rgb shown() const { return _shown; }

private:
    flux::Rgb _pending{};
    flux::Rgb _shown{};
};

// ---- state: the loop task's alone ----------------------------------------------------

Adc1 g_adc;
bool g_adcOk = false;
bool g_uartOk = false;

barnacle::Decoder g_decoder;
barnacle::TripDetector g_trip;
barnacle::Command g_cmd{barnacle::Mode::idle, 0, 0, 0, barnacle::kDefaultHomeTripMa, barnacle::kDefaultRunTripMa};
bool g_heardP4 = false;
int64_t g_lastCmdUs = 0;
uint16_t g_badCommands = 0;   // frames that verified but did not parse as a command

int64_t g_nextSampleUs = 0;
int64_t g_nextStatusUs = 0;
uint32_t g_samples = 0;
uint16_t g_late = 0;
uint8_t g_txSeq = 0;
uint32_t g_statusCount = 0;

// The status frame's averages since the last one.
int32_t g_rawSum = 0;
int32_t g_maSum = 0;
int32_t g_sampleCount = 0;

uint16_t g_motorMv = barnacle::kNoMillivolts;
uint16_t g_ana1Mv = barnacle::kNoMillivolts;
uint16_t g_ana2Mv = barnacle::kNoMillivolts;
uint32_t g_slowTurn = 0;

bool g_tripOut = false;
uint8_t g_resetReason = 0;

ShimPixel g_pixel;
flux::GlowEngine g_engine(g_pixel);
flux::Rgb g_ledNow{1, 1, 1};   // never a real color: forces the first write
uint32_t g_ledAtMs = 0;

uint32_t nowMs() { return uint32_t(esp_timer_get_time() / 1000); }

// ---- boot ----------------------------------------------------------------------------

void countBoot() {
    const esp_reset_reason_t why = esp_reset_reason();
    if (why == ESP_RST_POWERON || g_bootMagic != kBootMagic) {
        g_bootMagic = kBootMagic;
        g_bootCount = 0;
    }
    ++g_bootCount;
    g_resetReason = uint8_t(why);
}

bool uartBegin() {
    uart_config_t cfg{};
    cfg.baud_rate = int(barnacle::kBaud);
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    // A TX ring, so a status frame never waits on the FIFO inside a sample slot.
    if (uart_driver_install(kUart, 512, 512, 0, nullptr, 0) != ESP_OK) return false;
    if (uart_param_config(kUart, &cfg) != ESP_OK) return false;
    if (uart_set_pin(kUart, kPinLinkTx, kPinLinkRx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
    return uart_set_line_inverse(kUart, UART_SIGNAL_TXD_INV) == ESP_OK;
}

// ---- the link ------------------------------------------------------------------------

void onCommand(const barnacle::Frame& f, int64_t now) {
    const std::optional<barnacle::Command> c = barnacle::asCommand(f);
    if (!c) {
        ++g_badCommands;
        return;
    }
    g_cmd = *c;
    g_heardP4 = true;
    g_lastCmdUs = now;
}

void drainUart(int64_t now) {
    if (!g_uartOk) return;
    std::array<uint8_t, 64> chunk{};
    size_t waiting = 0;
    if (uart_get_buffered_data_len(kUart, &waiting) != ESP_OK || waiting == 0) return;
    const int n = uart_read_bytes(kUart, chunk.data(), uint32_t(waiting < chunk.size() ? waiting : chunk.size()), 0);
    for (int i = 0; i < n; ++i)
        if (const std::optional<barnacle::Frame> f = g_decoder.feed(chunk[size_t(i)])) onCommand(*f, now);
}

bool p4Silent(int64_t now) {
    return !g_heardP4 || now - g_lastCmdUs > int64_t(barnacle::kP4SilentMs) * 1000;
}

uint16_t badFrames() {
    const barnacle::RxStats& s = g_decoder.stats();
    return uint16_t(s.crc + s.version + s.malformed + s.overrun + g_badCommands);
}

void send(size_t n, const std::array<uint8_t, barnacle::kMaxWire>& out) {
    if (g_uartOk && n > 0) uart_write_bytes(kUart, out.data(), n);
}

void sendStatus(int64_t now) {
    barnacle::Status s;
    if (digitalRead(kPinButton) == LOW) s.flags |= barnacle::kFlagButton;
    if (g_tripOut) s.flags |= barnacle::kFlagTrip;
    if (kCurrentCalibrated) s.flags |= barnacle::kFlagCalibrated;
    if (p4Silent(now)) s.flags |= barnacle::kFlagP4Silent;
    if (digitalRead(kPinSpareIn) == HIGH) s.flags |= barnacle::kFlagSpareIn;
    if (g_sampleCount > 0) {
        s.current_raw = uint16_t(g_rawSum / g_sampleCount);
        s.current_ma = int16_t(constrain(g_maSum / g_sampleCount, -32767, 32767));
    }
    s.motor_mv = g_motorMv;
    g_rawSum = g_maSum = g_sampleCount = 0;
    std::array<uint8_t, barnacle::kMaxWire> out{};
    send(barnacle::encode(s, g_txSeq++, out), out);

    if (++g_statusCount % barnacle::kSlowEvery != 0) return;
    barnacle::Slow w;
    w.uptime_ms = nowMs();
    w.boots = g_bootCount;
    w.reset_reason = g_resetReason;
    w.mode = g_cmd.mode;
    w.home_trip_ma = g_cmd.home_trip_ma;
    w.run_trip_ma = g_cmd.run_trip_ma;
    w.ana1_mv = g_ana1Mv;
    w.ana2_mv = g_ana2Mv;
    w.trips = g_trip.assertions();
    w.bad_frames = badFrames();
    w.late_samples = g_late;
    w.shim_version = kShimVersion;
    send(barnacle::encode(w, g_txSeq++, out), out);
}

// ---- the samples ----------------------------------------------------------------------

// Pin millivolts to the J2 supply's, saturating under kNoMillivolts.
uint16_t motorMv(int pinMv) {
    const int32_t mv = int32_t(pinMv) * kMotorVoltsPerPinVolt;
    return uint16_t(mv < 0 ? 0 : (mv >= barnacle::kNoMillivolts ? barnacle::kNoMillivolts - 1 : mv));
}

uint16_t pinMv(const std::optional<Adc1::Sample>& s) {
    return s ? uint16_t(s->mv) : barnacle::kNoMillivolts;
}

void readSlowChannel() {
    switch (g_slowTurn++ % 3) {
        case 0: {
            const std::optional<Adc1::Sample> s = g_adc.read(kChMotorV);
            g_motorMv = s ? motorMv(s->mv) : barnacle::kNoMillivolts;
            break;
        }
        case 1: g_ana1Mv = pinMv(g_adc.read(kChAna1)); break;
        default: g_ana2Mv = pinMv(g_adc.read(kChAna2)); break;
    }
}

void sampleCurrent() {
    const std::optional<Adc1::Sample> s = g_adc.read(kChCurrent);
    if (!s) return;   // no sample: the detector holds, the status reads none
    const int32_t sensorMv = int32_t(s->mv) * kCurrentDividerNum / kCurrentDividerDen;
    const int32_t ma = (sensorMv - kCurrentZeroMv) * 1000 / kCurrentMvPerA;
    g_rawSum += s->raw;
    g_maSum += ma;
    ++g_sampleCount;
    const bool trip = g_trip.sample(ma, barnacle::activeThreshold(g_cmd.mode, g_cmd.home_trip_ma, g_cmd.run_trip_ma));
    if (trip != g_tripOut) {
        g_tripOut = trip;
        digitalWrite(kPinTrip, trip ? HIGH : LOW);
    }
}

// ---- the pixel's source ---------------------------------------------------------------

void serviceLed(int64_t now) {
    const uint32_t ms = uint32_t(now / 1000);
    if (ms - g_ledAtMs < kRgbMinMs) return;
    g_ledAtMs = ms;
    flux::Rgb want{g_cmd.r, g_cmd.g, g_cmd.b};
    if (p4Silent(now)) {
        g_engine.update(ms);
        want = g_pixel.shown();
    }
    if (want.r == g_ledNow.r && want.g == g_ledNow.g && want.b == g_ledNow.b) return;
    g_ledNow = want;
    writeRgb(want.r, want.g, want.b);
}

}  // namespace

// ---- Arduino entry ------------------------------------------------------------------------

void setup() {
    // TRIP and the spare output first: off is LOW on both transistors.
    pinMode(kPinTrip, OUTPUT);
    digitalWrite(kPinTrip, LOW);
    pinMode(kPinSpareOut, OUTPUT);
    digitalWrite(kPinSpareOut, LOW);
    setCpuFrequencyMhz(kCpuMhz);
    pinMode(kPinButton, INPUT_PULLUP);
    pinMode(kPinSpareIn, INPUT);
    countBoot();
    g_adcOk = g_adc.begin();
    g_uartOk = uartBegin();
    g_engine.setBrightness(kOwnBrightness);
    g_engine.set(flux::System::Link, flux::Status::Degraded);
    // No ADC is a fault the P4 must see at once: current reads none, and the
    // pixel says Safety until a P4 color replaces it.
    if (!g_adcOk) g_engine.set(flux::System::Safety, flux::Status::Urgent);
    const int64_t now = esp_timer_get_time();
    g_nextSampleUs = now;
    g_nextStatusUs = now;
}

void loop() {
    const int64_t now = esp_timer_get_time();
    if (now < g_nextSampleUs) {
        drainUart(now);
        return;
    }
    g_nextSampleUs += barnacle::kTripSampleUs;
    if (now - g_nextSampleUs > int64_t(barnacle::kTripSampleUs)) {
        g_nextSampleUs = now + barnacle::kTripSampleUs;
        ++g_late;
    }
    sampleCurrent();
    if (++g_samples % kSlowReadEvery == 0) readSlowChannel();
    if (now >= g_nextStatusUs) {
        g_nextStatusUs += barnacle::kStatusPeriodUs;
        if (now - g_nextStatusUs > int64_t(barnacle::kStatusPeriodUs)) g_nextStatusUs = now + barnacle::kStatusPeriodUs;
        sendStatus(now);
    }
    serviceLed(now);
}
