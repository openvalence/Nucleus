// test_thermal -- native doctest suite for the thermistor curve and the fan policy
// Constraints:
// - Hardware-free: Thermal.h alone. Expected temperatures come from the B3950
//   equation evaluated here, never from the table under test; the fan is a
//   first-order model whose speed lags duty, standing in for the real buck
//   and fan (bench work).
// See: flagship_p4/src/system/Thermal.h, bd val-091.28

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <optional>

#include "../../../flagship_p4/src/system/Thermal.h"

namespace th = valence::thermal;
using th::FanMode;

namespace {

// The B3950 bead the table encodes.
double b3950Ohms(double celsius) {
    return 10000.0 * std::exp(3950.0 * (1.0 / (celsius + 273.15) - 1.0 / 298.15));
}

// The divider voltage for a resistance, the inverse of ntcOhms().
float voltsFor(double ohms) { return float(th::kRailV * ohms / (ohms + th::kPullupOhms)); }

// A fan whose speed approaches top x drive^0.7 with a one-step lag: the
// mapping is deliberately not duty-linear, as the discontinuous buck is not.
struct FanModel {
    float top = 4000.0f;
    float rpm = 0.0f;
    bool tach = true;
    bool jammed = false;
    float advance(float duty) {
        const float goal = duty <= 0.0f ? 0.0f : top * std::pow(duty, 0.7f);
        rpm = jammed ? 0.0f : rpm + 0.6f * (goal - rpm);   // a blocked rotor stops dead
        return tach ? rpm : 0.0f;
    }
};

// Runs the policy against the model for `steps` steps at a fixed temperature.
struct Rig {
    th::FanPolicy p;
    FanModel fan;
    uint32_t t = 10000;
    th::FanCommand last;
    float measured = 0.0f;
    int stalls = 0;
    void run(std::optional<float> celsius, int steps) {
        for (int i = 0; i < steps; ++i) {
            last = p.step(celsius, measured, t);
            if (last.stalled) ++stalls;
            measured = fan.advance(last.duty);
            t += th::kStepMs;
        }
    }
};

}  // namespace

TEST_CASE("thermistor: table points and log interpolation match the B3950 equation") {
    for (double c : {-20.0, 0.0, 25.0, 37.0, 42.5, 65.0, 99.0, 119.0}) {
        CAPTURE(c);
        CHECK(th::ntcCelsius(float(b3950Ohms(c))) == doctest::Approx(c).epsilon(0.002).scale(100.0));
    }
    // Through the divider: 25 C is 10k, half the rail.
    CHECK(th::ntcOhms(th::kRailV / 2.0f) == doctest::Approx(10000.0));
    CHECK(th::thermCelsius(voltsFor(b3950Ohms(30.0))) == doctest::Approx(30.0).epsilon(0.01));
}

TEST_CASE("thermistor: open, shorted and out-of-table readings are NaN, never clamped") {
    CHECK(std::isnan(th::ntcOhms(th::kRailV)));          // open J7: pulled to the rail
    CHECK(std::isnan(th::ntcOhms(th::kRailV + 0.1f)));
    CHECK(std::isnan(th::ntcOhms(0.0f)));                // shorted J7
    CHECK(std::isnan(th::ntcOhms(th::kNaN)));
    CHECK(std::isnan(th::ntcCelsius(float(b3950Ohms(-30.0)))));   // colder than the table
    CHECK(std::isnan(th::ntcCelsius(float(b3950Ohms(130.0)))));   // hotter than the table
    CHECK(std::isnan(th::thermCelsius(3.29f)));          // 3.2 Mohm: an open wire, not -60 C
}

TEST_CASE("fan curve: linear between points, held at both ends") {
    CHECK(th::fanCurveFraction(0.0f) == doctest::Approx(0.30));
    CHECK(th::fanCurveFraction(45.0f) == doctest::Approx(0.425));
    CHECK(th::fanCurveFraction(55.0f) == doctest::Approx(0.775));
    CHECK(th::fanCurveFraction(90.0f) == doctest::Approx(1.0));
}

TEST_CASE("fan: off before the first THERM sample and below the on threshold") {
    Rig r;
    r.run(std::nullopt, 5);
    CHECK(r.last.mode == FanMode::off);
    CHECK(r.last.duty == 0.0f);
    r.run(30.0f, 5);
    CHECK(r.last.mode == FanMode::off);
}

TEST_CASE("fan: hysteresis between kFanOffC and kFanOnC") {
    Rig r;
    r.run(38.0f, 3);                       // between the thresholds, coming from cold
    CHECK(r.last.mode == FanMode::off);
    r.run(40.0f, 3);
    CHECK(r.last.mode != FanMode::off);
    r.run(36.0f, 3);                       // back between: stays on, at the curve floor
    CHECK(r.last.mode != FanMode::off);
    CHECK(r.last.target == doctest::Approx(0.30));
    r.run(34.9f, 1);
    CHECK(r.last.mode == FanMode::off);
    CHECK(r.last.duty == 0.0f);
}

TEST_CASE("fan: every start kicks at full duty, then learns the top speed") {
    Rig r;
    r.run(45.0f, 1);
    CHECK(r.last.mode == FanMode::kicking);
    CHECK(r.last.duty == 1.0f);
    r.run(45.0f, int(th::kKickMs / th::kStepMs));
    CHECK(r.last.mode == FanMode::running);
    CHECK(r.last.tach);
    CHECK(r.p.topRpm() > 0.8f * r.fan.top);
}

TEST_CASE("fan: the tach loop settles on the curve's speed, not its duty") {
    Rig r;
    r.run(45.0f, 30);
    const float want = th::fanCurveFraction(45.0f) * r.p.topRpm();
    CHECK(r.fan.rpm == doctest::Approx(want).epsilon(0.03));
    // The model is not duty-linear, so the settled duty is not the fraction.
    CHECK(std::fabs(r.last.duty - th::fanCurveFraction(45.0f)) > 0.05f);
    r.run(55.0f, 30);
    CHECK(r.fan.rpm == doctest::Approx(th::fanCurveFraction(55.0f) * r.p.topRpm()).epsilon(0.03));
}

TEST_CASE("fan: a stall with a tach is retried with a kick, and recovers") {
    Rig r;
    r.run(50.0f, 20);
    REQUIRE(r.last.mode == FanMode::running);
    r.fan.jammed = true;
    r.run(50.0f, int(th::kStallMs / th::kStepMs) + 2);
    CHECK(r.stalls == 1);
    CHECK(r.last.mode == FanMode::kicking);
    r.fan.jammed = false;
    r.run(50.0f, 30);
    CHECK(r.last.mode == FanMode::running);
    CHECK(r.last.tach);
    CHECK(r.fan.rpm > 0.0f);
}

TEST_CASE("fan: no tach pulse by the end of the kick is no fan fitted: off for the boot") {
    // bd val-30d: a devkit with nothing on J6 and a floating THERM drove a
    // phantom 100 % into nothing.
    Rig r;
    r.fan.tach = false;
    r.run(55.0f, 1 + int(th::kKickMs / th::kStepMs));
    CHECK_FALSE(r.last.fitted);
    CHECK(r.last.mode == FanMode::off);
    CHECK(r.last.duty == 0.0f);
    // Stays off whatever the demand does, and never kicks again.
    r.run(th::kNaN, 10);
    r.run(70.0f, 30);
    CHECK_FALSE(r.last.fitted);
    CHECK(r.last.duty == 0.0f);
    CHECK(r.last.mode == FanMode::off);
    CHECK(r.stalls == 0);
    CHECK(r.last.target > 0.0f);   // the demand is still reported
}

TEST_CASE("fan: a fan with a tach is fitted and is not mistaken for absent") {
    Rig r;
    r.run(55.0f, 20);
    CHECK(r.last.fitted);
    CHECK(r.last.mode == FanMode::running);
}

TEST_CASE("fan: an unusable THERM reading runs the no-sensor fraction, never off") {
    Rig r;
    r.run(th::kNaN, 20);
    CHECK_FALSE(r.last.sensor);
    CHECK(r.last.mode == FanMode::running);
    CHECK(r.last.target == doctest::Approx(th::kNoSensorFraction));
}

TEST_CASE("fan: the duty floor holds a slow demand above stall voltage") {
    Rig r;
    r.fan.top = 400.0f;    // a fan that barely turns: the loop drives duty down hard
    r.run(41.0f, 30);
    CHECK(r.last.mode == FanMode::running);
    CHECK(r.last.duty >= th::kMinDuty);
}

TEST_CASE("thermistor: the sanity window, and the pull-up's direction: an open pin reads cold, never a room") {
    // The divider pulls THERM to the rail, so an open J7 sits at full scale;
    // the ADC's calibrated top lands a little under the rail.
    CHECK(std::isnan(th::thermCelsius(th::kRailV * 0.97f)));
    CHECK(std::isnan(th::thermCelsius(voltsFor(b3950Ohms(-15.0)))));   // below the window
    CHECK(th::thermCelsius(voltsFor(b3950Ohms(-5.0))) == doctest::Approx(-5.0).epsilon(0.01).scale(10.0));
    // The hot side is never discarded short of the table's end.
    CHECK(th::thermCelsius(voltsFor(b3950Ohms(115.0))) == doctest::Approx(115.0).epsilon(0.01));
    // Calibration knob present and neutral by default.
    CHECK(th::kOffsetC == 0.0f);
}
