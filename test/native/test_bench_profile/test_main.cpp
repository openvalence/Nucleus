// test_bench_profile -- native doctest suite for the NUCLEUS_BENCH_NO_MOTOR
// build profile: the motor-power gate is advisory, and only here
// Constraints:
// - This translation unit IS the bench build: the profile is defined before
//   the first include, exactly as the flagship_p4_bench environment's CMake
//   option defines it for every component. test_motion_arbiter compiles the
//   same MotionArbiter.cpp without it and asserts the gate shut.
// - Hardware-free: a synthetic clock and an ideal emitter.
// See: flagship_p4/src/hub/valence_config.h, MotionArbiter.h, bd val-091.58

#define NUCLEUS_BENCH_NO_MOTOR 1

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string_view>

// Named here so the dependency finder builds them; the .cpp below needs all three.
#include "geiger/geiger.h"
#include "kinetic2/engine.hpp"
#include "valence/generated/registry_constants.hpp"

#include "../../../flagship_p4/src/motion/MotionArbiter.cpp"

using valence::MotionArbiter;
using valence::MotionCensus;
using valence::MotionEmitter;
using valence::MotionIntent;
using valence::MotionSource;

namespace {

uint64_t g_now_us = 1'000'000;
uint64_t testNowUs() { return g_now_us; }

class TestEmitter final : public MotionEmitter {
public:
    int32_t count() const override { return n; }
    void steer(float v_mm_s) override {
        const valence::SteerWord w = valence::steerWord(v_mm_s);
        q8 = w.step_q8;
        if (q8 != 0) fwd = w.forward;
    }
    void park() override { q8 = 0; }

    void advance(double dt_s) {
        if (q8 == 0) return;
        const double rate = double(valence::kLpClockHz) * 256.0 / double(q8);
        phase += (fwd ? rate : -rate) * dt_s;
        const double whole = std::trunc(phase);
        phase -= whole;
        n += int32_t(whole);
    }

    int32_t  n = 0;
    uint32_t q8 = 0;
    bool     fwd = true;
    double   phase = 0.0;
};

// The devkit: the switch never reports on.
struct Rig {
    TestEmitter emitter;
    MotionArbiter arb{emitter, &testNowUs};

    Rig() {
        arb.begin(g_now_us);
        arb.setMotorPowered(false);
        arb.setCommissioned(true);
    }

    bool submit(MotionSource src, float target_mm) {
        MotionIntent in;
        in.source = src;
        in.target_mm = target_mm;
        return arb.accept(in, g_now_us);
    }

    void run(uint64_t us) {
        for (uint64_t t = 0; t < us; t += 1000) {
            g_now_us += 1000;
            emitter.advance(1e-3);
            arb.evaluate(g_now_us, 1e-3f);
        }
    }

    MotionCensus census() { return arb.snapshot(g_now_us); }
};

}  // namespace

TEST_CASE("bench profile: the version says so, and the constant is the macro") {
    CHECK(valence::kBenchNoMotor);
    CHECK(std::string_view(FIRMWARE_VERSION).ends_with("-bench"));
}

TEST_CASE("bench profile: motion runs with the switch reporting off, and the census stays honest") {
    auto r = std::make_unique<Rig>();
    r->arb.forceHome(400.0f);
    r->run(1000);
    MotionCensus c = r->census();
    CHECK_FALSE(c.motor_on);   // the switch's word is never dressed up
    CHECK(c.power_gate);
    // A lone sample renders at its grant's latency, and 120 mm in that time is
    // spent (RFC-105 promise 3): the move is a timed segment.
    MotionIntent in;
    in.source = MotionSource::Stream;
    in.target_mm = 120.0f;
    in.duration_us = 1'500'000;
    REQUIRE(r->arb.accept(in, g_now_us));
    r->run(2'000'000);
    c = r->census();
    CHECK(c.position_mm == doctest::Approx(120.0f).epsilon(0.01));
    CHECK(c.rejected == 0);
    CHECK(r->submit(MotionSource::Manual, 40.0f));
}

TEST_CASE("bench profile: every other gate still holds") {
    auto r = std::make_unique<Rig>();
    r->run(1000);
    CHECK_FALSE(r->submit(MotionSource::Stream, 50.0f));   // unhomed
    r->arb.forceHome(400.0f);
    r->run(1000);
    r->arb.pause(true);
    CHECK_FALSE(r->submit(MotionSource::Stream, 50.0f));   // paused
    r->arb.pause(false);
    r->arb.estop(true);
    CHECK_FALSE(r->submit(MotionSource::Manual, 50.0f));   // e-stop
}

TEST_CASE("bench profile: a return renders like a powered one") {
    auto r = std::make_unique<Rig>();
    r->arb.forceHome(400.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 60.0f));
    r->run(4'000'000);
    r->arb.pause(true);
    r->run(1000);
    r->arb.override();
    REQUIRE(r->submit(MotionSource::Manual, 30.0f));
    r->run(3'000'000);
    CHECK(r->arb.returnToPause() == valence::ReturnStart::queued);
    for (int i = 0; i < 6000 && (r->census().returning || r->census().override_mode); ++i) r->run(1000);
    CHECK_FALSE(r->census().override_mode);
    CHECK(r->census().position_mm == doctest::Approx(60.0f).epsilon(0.01));
}
