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

TEST_CASE("paused: Stream refused, Manual still moves") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    r->arb.pause(true);
    CHECK_FALSE(r->submit(MotionSource::Stream, 100.0f));
    CHECK(r->submit(MotionSource::Manual, 100.0f));
    r->arb.pause(false);
    CHECK(r->submit(MotionSource::Stream, 100.0f));
}

TEST_CASE("e-stop refuses every source, parks on the calling task, drops homed") {
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

    r->arb.estop(false);
    CHECK(r->submit(MotionSource::Manual, 50.0f));
    CHECK_FALSE(r->submit(MotionSource::Stream, 50.0f));
}

TEST_CASE("window clamp: Stream held in the window, Manual reaches the whole rail") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->arb.setWindow(100.0f, 200.0f, 400.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Stream, 250.0f));
    CHECK(r->census().demand_mm == doctest::Approx(200.0f));
    REQUIRE(r->submit(MotionSource::Stream, 20.0f));
    CHECK(r->census().demand_mm == doctest::Approx(100.0f));
    REQUIRE(r->submit(MotionSource::Manual, 390.0f));
    CHECK(r->census().demand_mm == doctest::Approx(390.0f));
    REQUIRE(r->submit(MotionSource::Manual, 450.0f));
    CHECK(r->census().demand_mm == doctest::Approx(400.0f));
    REQUIRE(r->submit(MotionSource::Manual, -5.0f));
    CHECK(r->census().demand_mm == doctest::Approx(0.0f));
}

TEST_CASE("a Manual point move lands on target through the emitter, at the user ceiling") {
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
    CHECK(peak <= DEFAULT_USER_MAX_SPEED_MM_S * 1.01f);
    CHECK(peak >= DEFAULT_USER_MAX_SPEED_MM_S * 0.9f);
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

TEST_CASE("stop halts a running generator and no further intent is submitted") {
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

    // The hub's stop, in its order: running drops, then gate plus brake.
    PatternSettings stopped = runningPattern();
    stopped.running = false;
    gen->apply(stopped);
    r->arb.stop();
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
    CHECK_FALSE(after.busy);
    CHECK(after.velocity_mm_s == doctest::Approx(0.0f));
    // A controlled decel: it stops within braking distance of where it was,
    // in its own direction, never a second stroke.
    const float brake_mm = v0 * v0 / (2.0f * DEFAULT_ACCEL_MM_S2);
    CHECK(std::fabs(after.position_mm - p0) <= brake_mm + 2.0f);

    const int32_t held = r->emitter.n;
    for (int i = 0; i < 2000; ++i) generatorPass(*r, *gen, nullptr);
    CHECK(r->emitter.n == held);

    // A start is what reopens it.
    r->arb.allowPattern();
    gen->apply(runningPattern());
    int restarted = 0;
    for (int i = 0; i < 1000; ++i) restarted += generatorPass(*r, *gen, nullptr) ? 1 : 0;
    CHECK(restarted >= 1);
}

TEST_CASE("e-stop closes the Pattern gate: clearing the latch never restarts the generator") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Pattern, 100.0f));
    r->arb.estop(true);
    r->run(1000);
    r->arb.forceHome(500.0f);   // drops the latch and rehomes
    r->run(1000);
    CHECK_FALSE(r->submit(MotionSource::Pattern, 150.0f));
    CHECK(r->submit(MotionSource::Stream, 150.0f));
    r->arb.allowPattern();
    CHECK(r->submit(MotionSource::Pattern, 150.0f));
}

TEST_CASE("a stream bundle after STOP is refused and counted; a manual move clears the latch "
          "and the next bundle after re-arm is accepted") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    // A live stream mid-stroke: chase points walking away from home.
    for (int i = 1; i <= 20; ++i) {
        REQUIRE(r->submit(MotionSource::Stream, 100.0f + 10.0f * float(i)));
        r->run(10'000);
    }
    REQUIRE(std::fabs(r->census().velocity_mm_s) > 10.0f);

    r->arb.stop();
    const uint32_t rejected = r->census().rejected;
    // The client keeps streaming through the STOP: every sample is refused.
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

    // The manual move is accepted through the STOP (SPEC 11.1: it is the new
    // motion intent), and the hub drops its latch on that acceptance. The
    // Stream gate stays closed until the delegate sees the clear latch.
    CHECK(r->submit(MotionSource::Manual, r->census().position_mm + 5.0f));
    r->run(500'000);
    CHECK_FALSE(r->submit(MotionSource::Stream, 300.0f));
    r->arb.allowStream();
    CHECK(r->submit(MotionSource::Stream, 300.0f));
    r->run(200'000);
    CHECK(r->emitter.n != held);
}
