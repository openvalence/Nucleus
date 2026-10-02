#pragma once

// Thermal -- the J7 thermistor as a temperature and the fan's policy: curve,
// hysteresis, spin-up kick, tach loop and stall recovery
// Constraints:
// - Hardware-free and allocation-free. ValenceFan.cpp is the board's host
//   (LEDC duty, PCNT tach, the THERM volts it is handed); suite test_thermal
//   drives this file.
// - SINGLE OWNER: FanPolicy is not safe against a concurrent call.
// - The fan rail is a controller-less buck that runs discontinuous at fan
//   current, so duty does not map to speed: with a tach the policy closes
//   the loop on RPM, and only a fan with no tach runs on open-loop duty
//   (Hardware SPEC.md 2026-09-23 fan-filter row).
// - NaN is "no reading": an open or shorted J7 reads outside the table and
//   becomes NaN, never a clamped edge value. The fan runs kNoSensorFraction
//   then, never off.
// - Units: volts, ohms, degrees C, duty and fraction in 0..1, RPM,
//   milliseconds in the host's clock (wrap-safe).
// See: Hardware flagship/SPEC.md 2026-09-23 fan rows, docs/board-map.md
// (regen clamp, thermal, fan), bd val-091.28

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace valence::thermal {

inline constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// ---- J7 thermistor: THE KNOBS -----------------------------------------------

// The divider on THERM: kPullupOhms from kRailV to G16, the NTC from G16 to
// GND. The 1k series resistor carries no DC and is not in the math.
inline constexpr float kPullupOhms = 10000.0f;
inline constexpr float kRailV      = 3.3f;

// CALIBRATION: added to every reading. Set it from one reference
// thermometer reading taken at room temperature.
inline constexpr float kOffsetC = 0.0f;

struct NtcPoint {
    float celsius;
    float ohms;
};

// J7 takes "any 10k NTC". This is the common 10k B3950 bead:
// R = 10k x exp(3950 x (1/T - 1/298.15 K)). A B3435 or B3380 part reads a few
// degrees off at the ends; swap the table for its datasheet's, coldest first.
inline constexpr std::array<NtcPoint, 15> kNtcCurve{{
    {-20.0f, 105385.0f},
    {-10.0f,  58246.0f},
    {  0.0f,  33621.0f},
    { 10.0f,  20175.0f},
    { 20.0f,  12535.0f},
    { 30.0f,   8037.0f},
    { 40.0f,   5301.0f},
    { 50.0f,   3588.0f},
    { 60.0f,   2486.0f},
    { 70.0f,   1760.0f},
    { 80.0f,   1270.0f},
    { 90.0f,    934.0f},
    {100.0f,    698.0f},
    {110.0f,    529.0f},
    {120.0f,    407.0f},
}};

// ---- the fan: THE KNOBS -----------------------------------------------------

struct FanPoint {
    float celsius;
    float fraction;   // of full speed: of the learned top RPM with a tach,
                      // of full duty without one
};

// Temperature to speed, linear between points, held at the ends. Only read
// while the fan is on (kFanOnC / kFanOffC below).
inline constexpr std::array<FanPoint, 3> kFanCurve{{
    {40.0f, 0.30f},
    {50.0f, 0.55f},
    {60.0f, 1.00f},
}};
// Hysteresis: on at or above kFanOnC, off below kFanOffC.
inline constexpr float kFanOnC  = 40.0f;
inline constexpr float kFanOffC = 35.0f;
// No usable THERM reading (J7 empty, open or shorted): run at this.
inline constexpr float kNoSensorFraction = 0.50f;

// Every start, and every stall retry, runs full duty this long; a DC fan on a
// low rail voltage may not break away from rest. Its RPM at the end is the
// learned top speed the curve's fractions scale.
inline constexpr uint32_t kKickMs = 2000;
// Duty floor while running: below it the buck output cannot keep a fan
// turning.
inline constexpr float kMinDuty = 0.20f;
// Integral gain of the RPM loop, duty per unit of normalized speed error per
// step. The host steps at kStepMs; at 1 s a fan's own lag dominates and 0.3
// settles in a few steps without overshoot.
inline constexpr float kLoopGain = 0.30f;
inline constexpr uint32_t kStepMs = 1000;
// Zero tach pulses this long while running with a tach is a stall: kick again.
inline constexpr uint32_t kStallMs = 5000;

static_assert(kFanOffC < kFanOnC, "fan hysteresis inverted");
static_assert(kMinDuty > 0.0f && kMinDuty < 1.0f, "duty floor out of range");

// ---- the thermistor math ----------------------------------------------------

// NTC resistance from the voltage on G16. NaN at or outside the rails (open:
// pulled to the rail; shorted: at ground) or for a NaN input.
inline float ntcOhms(float volts) {
    if (!(volts > 0.0f) || !(volts < kRailV)) return kNaN;
    return kPullupOhms * volts / (kRailV - volts);
}

// Degrees C from NTC ohms, interpolated in ln(R) between table points (an NTC
// is close to exponential, so this is near-exact between 10 C points).
// NaN outside the table: a reading past either end is a fault, not a value.
inline float ntcCelsius(float ohms) {
    if (!(ohms > 0.0f)) return kNaN;
    if (ohms > kNtcCurve.front().ohms || ohms < kNtcCurve.back().ohms) return kNaN;
    for (size_t i = 1; i < kNtcCurve.size(); ++i) {
        const NtcPoint& hi = kNtcCurve[i - 1];   // colder, more ohms
        const NtcPoint& lo = kNtcCurve[i];
        if (ohms >= lo.ohms) {
            const float t = (std::log(hi.ohms) - std::log(ohms)) /
                            (std::log(hi.ohms) - std::log(lo.ohms));
            return hi.celsius + t * (lo.celsius - hi.celsius) + kOffsetC;
        }
    }
    return kNaN;
}

inline float thermCelsius(float volts) { return ntcCelsius(ntcOhms(volts)); }

// The curve's speed fraction at a temperature, held at both ends.
inline float fanCurveFraction(float celsius) {
    if (celsius <= kFanCurve.front().celsius) return kFanCurve.front().fraction;
    for (size_t i = 1; i < kFanCurve.size(); ++i) {
        const FanPoint& a = kFanCurve[i - 1];
        const FanPoint& b = kFanCurve[i];
        if (celsius <= b.celsius)
            return a.fraction + (celsius - a.celsius) / (b.celsius - a.celsius) * (b.fraction - a.fraction);
    }
    return kFanCurve.back().fraction;
}

// ---- the policy -------------------------------------------------------------

enum class FanMode : uint8_t { off, kicking, running };

struct FanCommand {
    float   duty      = 0.0f;   // 0..1, what the host writes to FAN_PWM
    FanMode mode      = FanMode::off;
    float   target    = 0.0f;   // the demanded speed fraction
    bool    sensor    = false;  // a usable THERM reading drove the demand
    bool    tach      = false;  // a kick has seen tach pulses this boot: closed loop
    bool    stalled   = false;  // this step began a stall retry
};

class FanPolicy {
public:
    // One step, every kStepMs. celsius: nullopt before the first THERM sample
    // (the fan stays off), NaN for an unusable one. rpm: measured over the
    // step just ended.
    FanCommand step(std::optional<float> celsius, float rpm, uint32_t nowMs) {
        FanCommand c;
        c.sensor = celsius.has_value() && std::isfinite(*celsius);
        float demand = 0.0f;
        if (!celsius) {
            _on = false;
        } else if (!c.sensor) {
            _on = true;
            demand = kNoSensorFraction;
        } else {
            if (*celsius >= kFanOnC) _on = true;
            else if (*celsius < kFanOffC) _on = false;
            demand = _on ? fanCurveFraction(*celsius) : 0.0f;
        }
        c.target = demand;

        if (demand <= 0.0f) {
            _mode = FanMode::off;
            _duty = 0.0f;
        } else if (_mode == FanMode::off) {
            startKick(nowMs);
        } else if (_mode == FanMode::kicking) {
            if (nowMs - _kickAtMs >= kKickMs) {
                // Sticky for the boot: a fan that once showed a tach and now
                // shows none at full duty is stalled, not tach-less.
                _tach = _tach || rpm > 0.0f;
                if (rpm > _topRpm) _topRpm = rpm;
                _mode = FanMode::running;
                _duty = clampDuty(demand);   // feed-forward start, the loop trims it
                _zeroSinceMs = nowMs;
            }
        } else if (_tach) {
            if (rpm > 0.0f) {
                _zeroSinceMs = nowMs;
                // No reading can beat the true top speed, so the highest seen
                // is the best estimate of it.
                if (rpm > _topRpm) _topRpm = rpm;
                _duty = clampDuty(_duty + kLoopGain * (demand - rpm / _topRpm));
            } else if (nowMs - _zeroSinceMs >= kStallMs) {
                startKick(nowMs);
                c.stalled = true;
            }
        } else {
            _duty = clampDuty(demand);
        }
        c.duty = _duty;
        c.mode = _mode;
        c.tach = _tach;
        return c;
    }

    float topRpm() const { return _topRpm; }

private:
    static float clampDuty(float d) { return d < kMinDuty ? kMinDuty : (d > 1.0f ? 1.0f : d); }

    void startKick(uint32_t nowMs) {
        _mode = FanMode::kicking;
        _duty = 1.0f;
        _kickAtMs = nowMs;
    }

    FanMode  _mode = FanMode::off;
    bool     _on = false;
    bool     _tach = false;
    float    _duty = 0.0f;
    float    _topRpm = 0.0f;
    uint32_t _kickAtMs = 0;
    uint32_t _zeroSinceMs = 0;
};

}  // namespace valence::thermal
