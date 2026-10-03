// ValenceFan -- implementation. See ValenceFan.h for ownership, Thermal.h for
// the curve, the loop and the knobs.
// Constraints:
// - BoardIo task only, except fanStatus(). LEDC low-speed timer 0 and
//   channel 0 belong to this file; nothing else in the firmware uses LEDC.
// - BSS: the policy and a few words. The PCNT driver allocates its unit and
//   channel objects from the internal heap once, at fanBegin().

#include "system/ValenceFan.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <optional>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/pulse_cnt.h>
#include <esp_timer.h>

#include "geiger/geiger.h"
#include "system/BoardPins.h"
#include "system/ValenceMotorSwitch.h"

namespace valence {

namespace {

constexpr const char* kTag = "fan";

// ---- THE KNOBS --------------------------------------------------------------

// The fan buck's switching frequency (Hardware SPEC.md 2026-09-23 fan-filter
// row: 100 uH + 22 uF, sized for ~50 kHz).
constexpr uint32_t kPwmHz = 50000;
// Standard PC-fan tach: two open-collector pulses per revolution.
constexpr float kPulsesPerRev = 2.0f;
// Tach edges closer than this are noise (a 10 krpm fan pulses every 3 ms).
constexpr uint32_t kTachGlitchNs = 1000;

constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kLedcChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_bit_t kDutyBits = LEDC_TIMER_10_BIT;
constexpr uint32_t kDutyMax = (1u << 10) - 1u;

// ---- state, the BoardIo task's ----------------------------------------------

bool g_ok = false;
pcnt_unit_handle_t g_tach = nullptr;
thermal::FanPolicy g_policy;
uint32_t g_lastStepMs = 0;
uint64_t g_lastCountUs = 0;
thermal::FanMode g_lastMode = thermal::FanMode::off;
bool g_lastSensor = true;
bool g_lastTach = false;
bool g_lastFitted = true;

// ---- the snapshot, any task -------------------------------------------------

std::atomic<float> g_celsius{thermal::kNaN};
std::atomic<float> g_duty{0.0f};
std::atomic<float> g_rpm{0.0f};
std::atomic<uint8_t> g_mode{0};
std::atomic<bool> g_tachSeen{false};
std::atomic<bool> g_fitted{true};

void writeDuty(float duty) {
    const uint32_t d = uint32_t(std::lround(duty * float(kDutyMax)));
    ledc_set_duty(kLedcMode, kLedcChannel, d > kDutyMax ? kDutyMax : d);
    ledc_update_duty(kLedcMode, kLedcChannel);
}

bool ledcBegin() {
    ledc_timer_config_t t{};
    t.speed_mode = kLedcMode;
    t.duty_resolution = kDutyBits;
    t.timer_num = kLedcTimer;
    t.freq_hz = kPwmHz;
    t.clk_cfg = LEDC_AUTO_CLK;
    if (ledc_timer_config(&t) != ESP_OK) return false;
    ledc_channel_config_t c{};
    c.gpio_num = BOARD_GPIO_FAN_PWM;
    c.speed_mode = kLedcMode;
    c.channel = kLedcChannel;
    c.timer_sel = kLedcTimer;
    c.duty = 0;
    c.hpoint = 0;
    if (ledc_channel_config(&c) != ESP_OK) return false;
    // G27 is the USB OTG 1.1 D+ pad, whose PHY pull-up would fight R701.
    gpio_pullup_dis(static_cast<gpio_num_t>(BOARD_GPIO_FAN_PWM));
    return true;
}

bool tachBegin() {
    pcnt_unit_config_t u{};
    u.low_limit = -1;
    u.high_limit = 32767;
    if (pcnt_new_unit(&u, &g_tach) != ESP_OK) return false;
    pcnt_glitch_filter_config_t f{};
    f.max_glitch_ns = kTachGlitchNs;
    pcnt_chan_config_t ch{};
    ch.edge_gpio_num = BOARD_GPIO_FAN_TACH;
    ch.level_gpio_num = -1;
    pcnt_channel_handle_t chan = nullptr;
    return pcnt_unit_set_glitch_filter(g_tach, &f) == ESP_OK &&
           pcnt_new_channel(g_tach, &ch, &chan) == ESP_OK &&
           pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_HOLD,
                                        PCNT_CHANNEL_EDGE_ACTION_INCREASE) == ESP_OK &&
           pcnt_unit_enable(g_tach) == ESP_OK && pcnt_unit_clear_count(g_tach) == ESP_OK &&
           pcnt_unit_start(g_tach) == ESP_OK;
}

// RPM since the last call, over the time that actually elapsed.
float takeRpm() {
    const uint64_t now = uint64_t(esp_timer_get_time());
    int count = 0;
    const bool ok = g_tach != nullptr && pcnt_unit_get_count(g_tach, &count) == ESP_OK &&
                    pcnt_unit_clear_count(g_tach) == ESP_OK;
    const uint64_t dtUs = now - g_lastCountUs;
    g_lastCountUs = now;
    if (!ok || count <= 0 || dtUs == 0) return 0.0f;
    return float(count) / kPulsesPerRev * 60.0e6f / float(dtUs);
}

constexpr const char* modeName(thermal::FanMode m) {
    switch (m) {
        case thermal::FanMode::kicking: return "kick";
        case thermal::FanMode::running: return "running";
        default:                        return "off";
    }
}

}  // namespace

bool fanBegin() {
    if (!ledcBegin()) {
        GLOGE(kTag, "LEDC on G%d failed: the fan stays off", BOARD_GPIO_FAN_PWM);
        return false;
    }
    if (!tachBegin()) {
        writeDuty(0.0f);
        GLOGE(kTag, "PCNT tach on G%d failed: the fan stays off", BOARD_GPIO_FAN_TACH);
        return false;
    }
    g_lastCountUs = uint64_t(esp_timer_get_time());
    g_ok = true;
    GLOGI(kTag, "fan host up: PWM G%d at %lu Hz, tach G%d", BOARD_GPIO_FAN_PWM,
          static_cast<unsigned long>(kPwmHz), BOARD_GPIO_FAN_TACH);
    return true;
}

void fanService(uint32_t nowMs) {
    if (!g_ok || nowMs - g_lastStepMs < thermal::kStepMs) return;
    g_lastStepMs = nowMs;

    const std::optional<float> volts = motorSwitchThermVolts();
    std::optional<float> celsius;
    if (volts) celsius = thermal::thermCelsius(*volts);
    const float rpm = takeRpm();
    const thermal::FanCommand c = g_policy.step(celsius, rpm, nowMs);
    writeDuty(c.duty);

    if (celsius && c.sensor != g_lastSensor) {
        g_lastSensor = c.sensor;
        if (c.sensor) GLOGI(kTag, "THERM reads %.1f C", double(*celsius));
        else GLOGW(kTag, "THERM unreadable (%.2f V: J7 open, shorted or out of window): fan demand %.0f%%",
                   double(volts.value_or(thermal::kNaN)), double(thermal::kNoSensorFraction * 100.0f));
    }
    if (!c.fitted && g_lastFitted)
        GLOGW(kTag, "no fan fitted: no tach pulse in the %lu ms kick; FAN_PWM off for this boot",
              static_cast<unsigned long>(thermal::kKickMs));
    g_lastFitted = c.fitted;
    if (c.tach && !g_lastTach)
        GLOGI(kTag, "tach seen: top speed %.0f rpm, closing the loop on RPM", double(g_policy.topRpm()));
    g_lastTach = c.tach;
    if (c.stalled)
        GLOGW_EVERY_MS(60000, kTag, "stalled: no tach pulses for %lu ms, kicking at full duty",
                       static_cast<unsigned long>(thermal::kStallMs));
    if (c.mode != g_lastMode) {
        GLOGD(kTag, "%s -> %s (target %.0f%%)", modeName(g_lastMode), modeName(c.mode),
              double(c.target * 100.0f));
        g_lastMode = c.mode;
    }

    g_celsius.store(celsius.value_or(thermal::kNaN), std::memory_order_relaxed);
    g_duty.store(c.duty, std::memory_order_relaxed);
    g_rpm.store(rpm, std::memory_order_relaxed);
    g_mode.store(uint8_t(c.mode), std::memory_order_relaxed);
    g_tachSeen.store(c.tach, std::memory_order_relaxed);
    g_fitted.store(c.fitted, std::memory_order_relaxed);
}

FanStatus fanStatus() {
    FanStatus s;
    s.celsius = g_celsius.load(std::memory_order_relaxed);
    s.duty = g_duty.load(std::memory_order_relaxed);
    s.rpm = g_rpm.load(std::memory_order_relaxed);
    s.mode = thermal::FanMode(g_mode.load(std::memory_order_relaxed));
    s.tach = g_tachSeen.load(std::memory_order_relaxed);
    s.fitted = g_fitted.load(std::memory_order_relaxed);
    return s;
}

}  // namespace valence
