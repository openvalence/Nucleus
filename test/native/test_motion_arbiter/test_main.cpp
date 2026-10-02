// test_motion_arbiter -- native doctest suite for the shared MotionArbiter core
// Constraints:
// - Hardware-free and deterministic: a synthetic microsecond clock and an ideal
//   emitter that renders the steering word exactly. No IDF, no FreeRTOS.
// - Compiles flagship_p4/src/motion/MotionArbiter.cpp itself, the one copy the
//   board and the sim both link, so a gate change is caught here before either.
//   The STOP cases also compile the pattern generator, the gate's one client.
// See: flagship_p4/src/motion/MotionArbiter.h, bd val-sf7.1

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

// Named here so the dependency finder builds them; the .cpp below needs both.
#include "geiger/geiger.h"
#include "kinetic/kinetic.hpp"

#include "../../../flagship_p4/src/motion/MotionArbiter.cpp"
#include "../../../flagship_p4/src/patterns/AdvancedPattern.cpp"
#include "../../../flagship_p4/src/patterns/PatternEngine.cpp"

using valence::MotionArbiter;
using valence::MotionCensus;
using valence::MotionEmitter;
using valence::MotionIntent;
using valence::MotionSource;
using valence::PatternEngine;
using valence::PatternInputs;
using valence::PatternSettings;

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
    void park() override { q8 = 0; ++parks; }

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
    int      parks = 0;
};

// Built on the heap (rig()): the arbiter holds a KB-scale engine.
struct Rig {
    TestEmitter emitter;
    MotionArbiter arb{emitter, &testNowUs};

    Rig() { arb.begin(g_now_us); }

    bool submit(MotionSource src, float target_mm) {
        MotionIntent in;
        in.source = src;
        in.target_mm = target_mm;
        return arb.accept(in, g_now_us);
    }

    // The board's tick order: the emitter renders the previous word, then the
    // arbiter evaluates and steers.
    void run(uint64_t us) {
        for (uint64_t t = 0; t < us; t += 1000) {
            g_now_us += 1000;
            emitter.advance(1e-3);
            arb.evaluate(g_now_us, 1e-3f);
        }
    }

    MotionCensus census() { return arb.snapshot(g_now_us); }
};

std::unique_ptr<Rig> rig() { return std::make_unique<Rig>(); }

}  // namespace

TEST_CASE("steerWord parks on NaN and crawl, flags the emitter floor, keeps direction") {
    CHECK(valence::steerWord(std::numeric_limits<float>::quiet_NaN()).step_q8 == 0);
    CHECK(valence::steerWord(0.5f * valence::kParkMmS).step_q8 == 0);
    const valence::SteerWord back = valence::steerWord(-20.0f);
    CHECK(back.step_q8 != 0);
    CHECK_FALSE(back.forward);
    CHECK_FALSE(back.floored);
    const valence::SteerWord fast = valence::steerWord(1.0e5f);
    CHECK(fast.floored);
    CHECK(fast.step_q8 == valence::kMinCyclesPerEdge * 256u);
}

TEST_CASE("unhomed: Manual moves, a Stream source is refused") {
    auto r = rig();
    CHECK_FALSE(r->submit(MotionSource::Stream, 100.0f));
    CHECK(r->submit(MotionSource::Manual, 100.0f));
    CHECK(r->census().rejected == 1);
    CHECK(r->census().intents == 1);
}

TEST_CASE("PAUSE refuses every source, Manual included; resume is the only re-arm") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    r->arb.pause(true);
    CHECK(r->census().paused);
    CHECK_FALSE(r->submit(MotionSource::Stream, 100.0f));
    CHECK_FALSE(r->submit(MotionSource::Pattern, 100.0f));
    CHECK_FALSE(r->submit(MotionSource::Manual, 100.0f));
    CHECK(r->census().rejected == 3);
    // A second pause is idempotent and never a second latch.
    r->arb.pause(true);
    CHECK_FALSE(r->submit(MotionSource::Manual, 100.0f));
    r->arb.pause(false);
    CHECK_FALSE(r->census().paused);
    CHECK(r->submit(MotionSource::Stream, 100.0f));
    CHECK(r->submit(MotionSource::Manual, 120.0f));
}

TEST_CASE("PAUSE brakes a moving carriage to rest and holds it") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 300.0f));
    r->run(400'000);
    const float v0 = r->census().velocity_mm_s;
    REQUIRE(std::fabs(v0) > 10.0f);
    const float p0 = r->census().position_mm;
    r->arb.pause(true);
    r->run(2'000'000);
    const MotionCensus c = r->census();
    CHECK_FALSE(c.busy);
    CHECK(c.velocity_mm_s == doctest::Approx(0.0f));
    // A controlled decel in its own direction, short of the old target.
    CHECK(c.position_mm >= p0);
    CHECK(c.position_mm < 300.0f);
    const int32_t held = r->emitter.n;
    r->run(500'000);
    CHECK(r->emitter.n == held);
}

TEST_CASE("e-stop refuses every source, parks on the calling task, drops homed when it cuts power") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 100.0f));
    r->run(50'000);
    REQUIRE(r->emitter.q8 != 0);

    r->arb.estop(true);
    CHECK(r->emitter.q8 == 0);
    CHECK(r->emitter.parks == 1);
    CHECK_FALSE(r->submit(MotionSource::Manual, 50.0f));
    CHECK_FALSE(r->submit(MotionSource::Stream, 50.0f));
    const int32_t held = r->emitter.n;
    r->run(100'000);
    CHECK(r->emitter.n == held);
    const MotionCensus c = r->census();
    CHECK(c.estop);
    CHECK_FALSE(c.homed);
    CHECK_FALSE(c.busy);

    // Release lands in PAUSE, never in motion (SPEC 11.2).
    r->arb.estop(false);
    const MotionCensus after = r->census();
    CHECK_FALSE(after.estop);
    CHECK(after.paused);
    CHECK_FALSE(after.homed);
    CHECK_FALSE(r->submit(MotionSource::Manual, 50.0f));
    // The sequence: release, PAUSE unhomed, home, resume.
    r->arb.forceHome(500.0f);
    r->run(1000);
    CHECK_FALSE(r->submit(MotionSource::Stream, 50.0f));
    r->arb.pause(false);
    CHECK(r->submit(MotionSource::Stream, 50.0f));
}

TEST_CASE("e-stop on a hub that does not cut power is a halt that keeps home") {
    auto r = rig();
    r->arb.setEstopCutsPower(false);
    r->arb.forceHome(500.0f);
    r->run(1000);
    r->arb.estop(true);
    CHECK(r->census().homed);
    r->arb.estop(false);
    CHECK(r->census().paused);
    CHECK(r->census().homed);
    r->arb.pause(false);
    CHECK(r->submit(MotionSource::Stream, 50.0f));
}

TEST_CASE("window clamp: every source held in the window; the jog under override reaches the rail") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->arb.setWindow(100.0f, 200.0f, 400.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Stream, 250.0f));
    CHECK(r->census().demand_mm == doctest::Approx(200.0f));
    REQUIRE(r->submit(MotionSource::Stream, 20.0f));
    CHECK(r->census().demand_mm == doctest::Approx(100.0f));
    REQUIRE(r->submit(MotionSource::Manual, 390.0f));
    CHECK(r->census().demand_mm == doctest::Approx(200.0f));
    r->run(5'000'000);
    r->arb.override();
    REQUIRE(r->submit(MotionSource::Manual, 390.0f));
    CHECK(r->census().demand_mm == doctest::Approx(390.0f));
    REQUIRE(r->submit(MotionSource::Manual, 450.0f));
    CHECK(r->census().demand_mm == doctest::Approx(400.0f));
    REQUIRE(r->submit(MotionSource::Manual, -5.0f));
    CHECK(r->census().demand_mm == doctest::Approx(0.0f));
}

TEST_CASE("a Manual point move lands on target through the emitter, at the jog ceiling") {
    auto r = rig();
    REQUIRE(r->submit(MotionSource::Manual, 60.0f));
    float peak = 0.0f;
    for (int i = 0; i < 4000; ++i) {
        r->run(1000);
        const float v = std::fabs(r->census().velocity_mm_s);
        if (v > peak) peak = v;
    }
    const MotionCensus c = r->census();
    CHECK_FALSE(c.busy);
    CHECK(std::fabs(c.position_mm - 60.0f) <= 2.0f * valence::kMmPerStep);
    CHECK(peak <= DEFAULT_JOG_MAX_SPEED_MM_S * 1.01f);
    CHECK(peak >= DEFAULT_JOG_MAX_SPEED_MM_S * 0.9f);
}

TEST_CASE("a frame move is not motion: force_home re-anchors and parks for one tick") {
    auto r = rig();
    REQUIRE(r->submit(MotionSource::Manual, 30.0f));
    r->run(3'000'000);
    const int32_t before = r->emitter.n;
    REQUIRE(r->census().position_mm == doctest::Approx(30.0f).epsilon(0.01));

    r->arb.forceHome(500.0f);
    r->run(1000);
    CHECK(r->emitter.q8 == 0);
    r->run(100'000);
    CHECK(r->emitter.n == before);
    const MotionCensus c = r->census();
    CHECK(c.homed);
    CHECK(c.position_mm == doctest::Approx(0.0f));
    CHECK(c.plan_mm == doctest::Approx(0.0f).epsilon(0.001));
}

TEST_CASE("Pattern source: gated and window-clamped like Stream, never the live stream") {
    auto r = rig();
    CHECK_FALSE(r->submit(MotionSource::Pattern, 100.0f));   // unhomed
    r->arb.forceHome(500.0f);
    r->arb.setWindow(100.0f, 200.0f, 400.0f);
    r->run(1000);
    r->arb.pause(true);
    CHECK_FALSE(r->submit(MotionSource::Pattern, 150.0f));
    r->arb.pause(false);
    REQUIRE(r->submit(MotionSource::Pattern, 350.0f));
    CHECK(r->census().demand_mm == doctest::Approx(200.0f));
    r->run(5000);
    const MotionCensus c = r->census();
    CHECK(c.busy);
    CHECK_FALSE(c.stream);
    REQUIRE(r->submit(MotionSource::Stream, 150.0f));
    CHECK(r->census().stream);
}

namespace {

PatternSettings runningPattern() {
    PatternSettings s;
    s.frame = {0.0f, DEFAULT_MAX_RAIL_MM, DEFAULT_MAX_SPEED_MM_S, DEFAULT_ACCEL_MM_S2};
    s.running = true;
    s.speed = 40.0f;
    s.depth = 80.0f;
    s.stroke = 60.0f;
    s.sensation = 50.0f;
    return s;
}

// The pattern task's pass, then the motion task's tick: what the board runs,
// one millisecond at a time. Returns true when the generator's intent landed.
bool generatorPass(Rig& r, PatternEngine& gen, MotionIntent* last) {
    const MotionCensus c = r.census();
    PatternInputs in;
    in.homed = c.homed;
    in.estop = c.estop;
    in.paused = c.paused;
    in.stream_active = c.stream;
    in.position_mm = c.position_mm;
    in.velocity_mm_s = c.velocity_mm_s;
    bool landed = false;
    if (const auto it = gen.tick(g_now_us, in)) {
        landed = r.arb.accept(*it, g_now_us);
        if (landed && last != nullptr) *last = *it;
    }
    r.run(1000);
    return landed;
}

}  // namespace

TEST_CASE("pause brakes a running generator, which parks; resume re-arms it with no new start") {
    auto r = rig();
    r->arb.forceHome(DEFAULT_MAX_RAIL_MM);
    r->run(1000);
    auto gen = std::make_unique<PatternEngine>();
    gen->apply(runningPattern());

    // Run until a half-stroke is moving fast: a stop mid-stroke is the case
    // that needs the brake.
    MotionIntent last{};
    int accepted = 0;
    for (int i = 0; i < 3000 && std::fabs(r->census().velocity_mm_s) < 100.0f; ++i)
        accepted += generatorPass(*r, *gen, &last) ? 1 : 0;
    REQUIRE(accepted >= 1);
    REQUIRE(gen->active());
    const float v0 = r->census().velocity_mm_s;
    REQUIRE(std::fabs(v0) >= 100.0f);
    const float p0 = r->census().position_mm;

    // The hub's pause: the latch, then the brake. The generator keeps its
    // settings and parks on the census's paused bit.
    r->arb.pause(true);
    const uint32_t intents = r->census().intents;

    // The preemption window: a half-stroke the pattern task built from the
    // old settings and submits only now.
    CHECK_FALSE(r->arb.accept(last, g_now_us));

    // Never a reversal: the carriage backs off its furthest point by no more
    // than the residual tracking's own settle, which an ordinary 200 mm stroke
    // end measures at 13 steps (0.06 mm) in this rig.
    const int32_t dir = v0 > 0.0f ? 1 : -1;
    int32_t furthest = r->emitter.n;
    int32_t backoff = 0;
    for (int i = 0; i < 2000; ++i) {
        CHECK_FALSE(generatorPass(*r, *gen, nullptr));
        const int32_t n = r->emitter.n * dir;
        if (n > furthest * dir) furthest = r->emitter.n;
        if (furthest * dir - n > backoff) backoff = furthest * dir - n;
    }
    CHECK(backoff <= 20);
    const MotionCensus after = r->census();
    CHECK(after.intents == intents);
    CHECK_FALSE(gen->active());
    CHECK(after.paused);
    CHECK_FALSE(after.busy);
    CHECK(after.velocity_mm_s == doctest::Approx(0.0f));
    // A controlled decel: it stops within braking distance of where it was,
    // in its own direction, never a second stroke.
    const float brake_mm = v0 * v0 / (2.0f * DEFAULT_ACCEL_MM_S2);
    CHECK(std::fabs(after.position_mm - p0) <= brake_mm + 2.0f);

    const int32_t held = r->emitter.n;
    for (int i = 0; i < 2000; ++i) generatorPass(*r, *gen, nullptr);
    CHECK(r->emitter.n == held);

    // resume is what re-arms it; no new start is needed.
    r->arb.pause(false);
    int restarted = 0;
    for (int i = 0; i < 1000; ++i) restarted += generatorPass(*r, *gen, nullptr) ? 1 : 0;
    CHECK(restarted >= 1);
}

TEST_CASE("e-stop closes the Pattern gate: releasing the latch never restarts the generator") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Pattern, 100.0f));
    r->arb.estop(true);
    r->run(1000);
    r->arb.estop(false);
    r->arb.forceHome(500.0f);
    r->arb.pause(false);
    r->run(1000);
    CHECK_FALSE(r->submit(MotionSource::Pattern, 150.0f));
    CHECK(r->submit(MotionSource::Stream, 150.0f));
    r->arb.allowPattern();
    CHECK(r->submit(MotionSource::Pattern, 150.0f));
}

TEST_CASE("a stream sample under PAUSE is refused and counted; no motion intent clears it, "
          "resume does") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    // A live stream mid-stroke: chase points walking away from home.
    for (int i = 1; i <= 20; ++i) {
        REQUIRE(r->submit(MotionSource::Stream, 100.0f + 10.0f * float(i)));
        r->run(10'000);
    }
    REQUIRE(std::fabs(r->census().velocity_mm_s) > 10.0f);

    r->arb.pause(true);
    const uint32_t rejected = r->census().rejected;
    // The client keeps streaming through the PAUSE: every sample is refused.
    for (int i = 0; i < 50; ++i) {
        CHECK_FALSE(r->submit(MotionSource::Stream, 400.0f));
        r->run(10'000);
    }
    CHECK(r->census().rejected == rejected + 50);
    CHECK_FALSE(r->census().busy);
    const int32_t held = r->emitter.n;
    for (int i = 0; i < 200; ++i) {
        CHECK_FALSE(r->submit(MotionSource::Stream, 400.0f));
        r->run(10'000);
    }
    CHECK(r->emitter.n == held);

    // A motion intent is not a re-arm (SPEC 11.1): only resume clears PAUSE.
    CHECK_FALSE(r->submit(MotionSource::Manual, r->census().position_mm + 5.0f));
    CHECK(r->emitter.n == held);
    r->arb.pause(false);
    CHECK(r->submit(MotionSource::Stream, 300.0f));
    r->run(200'000);
    CHECK(r->emitter.n != held);
}

TEST_CASE("override: PAUSE holds every source but the jog, which reaches outside the window") {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->arb.setWindow(100.0f, 200.0f, 400.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Stream, 150.0f));
    r->run(2'000'000);
    r->arb.override();
    const MotionCensus c = r->census();
    CHECK(c.paused);
    CHECK(c.override_mode);
    CHECK_FALSE(r->submit(MotionSource::Stream, 180.0f));
    CHECK_FALSE(r->submit(MotionSource::Pattern, 180.0f));
    REQUIRE(r->submit(MotionSource::Manual, 350.0f));
    CHECK(r->census().demand_mm == doctest::Approx(350.0f));
    r->run(10'000'000);
    CHECK(r->census().position_mm == doctest::Approx(350.0f).epsilon(0.01));
    // resume clears override with PAUSE; the hub refuses that resume while
    // override holds, so this is the arbiter's half only.
    r->arb.pause(false);
    CHECK_FALSE(r->census().override_mode);
}

TEST_CASE("return: a jog-set move back to the paused position, then override drops and PAUSE holds") {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 120.0f));
    r->run(6'000'000);
    REQUIRE(r->census().position_mm == doctest::Approx(120.0f).epsilon(0.01));
    r->arb.pause(true);
    r->run(1000);
    r->arb.override();
    REQUIRE(r->submit(MotionSource::Manual, 40.0f));
    r->run(4'000'000);
    REQUIRE(r->census().position_mm == doctest::Approx(40.0f).epsilon(0.02));

    const uint32_t before = r->census().returns;
    r->arb.returnToPause();
    r->run(2000);
    CHECK(r->census().returning);
    // Jog is refused while the return runs.
    CHECK_FALSE(r->submit(MotionSource::Manual, 10.0f));
    float peak = 0.0f;
    for (int i = 0; i < 6000 && r->census().returning; ++i) {
        r->run(1000);
        peak = std::max(peak, std::fabs(r->census().velocity_mm_s));
    }
    const MotionCensus c = r->census();
    CHECK_FALSE(c.returning);
    CHECK(c.returns == before + 1);
    CHECK_FALSE(c.override_mode);
    CHECK(c.paused);
    CHECK(c.position_mm == doctest::Approx(120.0f).epsilon(0.01));
    CHECK(peak <= DEFAULT_JOG_MAX_SPEED_MM_S * 1.01f);
    // Plain PAUSE again: the jog is refused until resume.
    CHECK_FALSE(r->submit(MotionSource::Manual, 60.0f));
}

TEST_CASE("ESTOP drops override and a running return; release lands in plain PAUSE") {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->run(1000);
    r->arb.override();
    REQUIRE(r->submit(MotionSource::Manual, 200.0f));
    r->run(500'000);
    r->arb.returnToPause();
    r->run(2000);
    r->arb.estop(true);
    r->run(1000);
    CHECK_FALSE(r->census().override_mode);
    CHECK_FALSE(r->census().returning);
    r->arb.estop(false);
    CHECK(r->census().paused);
    CHECK_FALSE(r->census().override_mode);
    CHECK_FALSE(r->submit(MotionSource::Manual, 50.0f));
}
