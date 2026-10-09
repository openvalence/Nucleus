// test_motion_arbiter -- native doctest suite for the shared MotionArbiter core
// Constraints:
// - Hardware-free and deterministic: a synthetic microsecond clock and an ideal
//   emitter that renders the steering word exactly, inside the LP core's fence
//   while its lease is live (lp_quad.c). No IDF, no FreeRTOS.
// - Compiles flagship_p4/src/motion/MotionArbiter.cpp itself, the one copy the
//   board and the sim both link, so a gate change is caught here before either.
// - The arbiter plans with Kinetic² (kinetic2::Engine<1>), the only planner.
//   The STOP cases also compile the pattern generator, the gate's one client.
// See: flagship_p4/src/motion/MotionArbiter.h, bd val-sf7.1

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Named here so the dependency finder builds them; the .cpp below needs all three.
#include "geiger/geiger.h"
#include "kinetic2/types.hpp"   // the cases name its anomaly kinds
#include "valence/generated/registry_constants.hpp"

#include "../../../flagship_p4/src/motion/MotionArbiter.cpp"
#include "../../../flagship_p4/src/patterns/advanced/AdvancedPattern.cpp"
#include "../../../flagship_p4/src/patterns/PatternEngine.cpp"

using valence::MotionArbiter;
using valence::MotionCensus;
using valence::MotionEmitter;
using valence::MotionIntent;
using valence::MotionSource;
using valence::AdvancedGenerator;
using valence::ClassicGenerator;
using valence::PatternEngine;
using valence::PatternInputs;
using valence::PatternSettings;

namespace {

uint64_t g_now_us = 1'000'000;

// A window write landed INSIDE a planner tick, as a hub task preempting the
// planner would (bd val-8es): at the clock read that times the solve (after
// the frame-move block, before the strip fills), or at the strip's publish.
// One shot; armed by the case, idle otherwise.
struct WindowPost {
    enum class At : uint8_t { none, clock, publish };
    MotionArbiter* arb = nullptr;
    At at = At::none;
    float lo = 0.0f, hi = 0.0f, rail = 0.0f;
    bool fired = false;
    void fire(At where) {
        if (arb == nullptr || at != where) return;
        at = At::none;
        fired = true;
        arb->setWindow(lo, hi, rail);
    }
};
WindowPost g_post;

uint64_t testNowUs() {
    g_post.fire(WindowPost::At::clock);
    return g_now_us;
}
void testLock(bool hold) {
    if (hold) g_post.fire(WindowPost::At::publish);
}

class TestEmitter final : public MotionEmitter {
public:
    int32_t count() const override { return n; }
    void steer(float v_mm_s) override {
        ++steers;
        const valence::SteerWord w = valence::steerWord(v_mm_s);
        q8 = w.step_q8;
        if (q8 != 0) fwd = w.forward;
    }
    void park() override { q8 = 0; ++parks; }
    void fence(int32_t lo, int32_t hi) override {
        fence_lo = lo;
        fence_hi = hi;
    }
    void renew() override {
        renewed_us = clock_us;
        live = true;
    }
    uint32_t lapses() const override { return lapse_count; }

    // The LP core over dt_s (lp_quad.c): it renders the word it holds while
    // the lease is live and withholds every edge past the fence; kLeaseUs
    // after the last renewal it stores 0 to the word and counts a lapse.
    // Unleased until the first renewal. The core reads the lease once per
    // pass, so its lapse lands within an edge of the instant modeled here.
    void advance(double dt_s) {
        if (frozen) return;
        const int64_t dt_us = std::llround(dt_s * 1e6);
        const int64_t left_us = renewed_us + int64_t(valence::kLeaseUs) - clock_us;
        const bool lapse = live && dt_us > left_us;
        const int64_t run_us = !live ? 0 : lapse ? std::max<int64_t>(left_us, 0) : dt_us;
        clock_us += dt_us;
        render(double(run_us) * 1e-6);
        if (lapse) {
            live = false;
            q8 = 0;
            phase = 0.0;
            ++lapse_count;
        }
    }

    // With on_edge set, calls it after each edge, so a park from inside it
    // stops the render on that edge.
    void render(double dt_s) {
        if (q8 == 0 || !(dt_s > 0.0)) return;
        const double rate = double(valence::kLpClockHz) * 256.0 / double(q8);
        phase += (fwd ? rate : -rate) * dt_s;
        const double whole = std::trunc(phase);
        phase -= whole;
        const int32_t edges = int32_t(whole);
        const int32_t step = edges > 0 ? 1 : -1;
        for (int32_t k = 0; k != edges; k += step) {
            if (step > 0 ? n >= fence_hi : n <= fence_lo) {
                ++fence_hits;
                continue;
            }
            n += step;
            if (!on_edge) continue;
            on_edge();
            if (q8 == 0) {
                phase = 0.0;
                return;
            }
        }
    }

    int32_t  n = 0;
    uint32_t q8 = 0;
    bool     fwd = true;
    double   phase = 0.0;
    int      parks = 0;
    int      steers = 0;
    bool     frozen = false;   // steered, never rendering, no lease: a dead emitter
    std::function<void()> on_edge;
    int32_t  fence_lo = INT32_MIN;   // open until the arbiter writes it, as the core loads it
    int32_t  fence_hi = INT32_MAX;
    uint32_t fence_hits = 0;
    int64_t  clock_us = 0;           // the core's own time, advanced by advance()
    int64_t  renewed_us = 0;
    bool     live = false;
    uint32_t lapse_count = 0;
};

// Built on the heap (rig()): the arbiter holds a KB-scale engine. A rig is a
// powered, commissioned machine unless a case says otherwise: the switch
// host's push and the hub's first-run record are part of the board's boot,
// not of the arbiter's.
struct Rig {
    TestEmitter emitter;
    MotionArbiter arb{emitter, &testNowUs, &testLock};

    explicit Rig(bool powered = true) {
        arb.begin(g_now_us);
        arb.setMotorPowered(powered);
        arb.setCommissioned(true);
    }

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

// A point move renders as a rest-to-rest quintic at its analytic minimum time
// (RFC-105 (k)), up to 1.875 times a cruise: a case that waits for a jog, or a
// home cycle with its point-move backoffs, to finish waits twice the cruise.
constexpr uint64_t moveUs(uint64_t us) { return 2 * us; }

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

TEST_CASE("uncommissioned (RFC-079 first run): Stream and Pattern are refused, Manual moves") {
    auto r = rig();
    r->arb.setCommissioned(false);
    r->arb.forceHome(400.0f);
    r->run(1000);
    CHECK_FALSE(r->submit(MotionSource::Stream, 100.0f));
    CHECK_FALSE(r->submit(MotionSource::Pattern, 100.0f));
    CHECK(r->submit(MotionSource::Manual, 100.0f));
    CHECK(r->census().rejected == 2);
    r->arb.setCommissioned(true);
    // A sample due before the pending jog's knot is behind the newest knot and
    // refused (RFC-105 (c)), so the jog lands first.
    r->run(5'000'000);
    CHECK(r->submit(MotionSource::Stream, 120.0f));
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
    r->run(3'000'000);   // the quintic starts slowly
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

TEST_CASE("motor power off: boots refusing every source, the jog under override included") {
    auto r = std::make_unique<Rig>(false);
    r->arb.forceHome(500.0f);
    r->run(1000);
    CHECK_FALSE(r->census().motor_on);
    // Not the bench profile: the gate is shut with the switch, not advisory.
    REQUIRE_FALSE(valence::kBenchNoMotor);
    CHECK_FALSE(r->census().power_gate);
    CHECK_FALSE(r->submit(MotionSource::Manual, 100.0f));
    CHECK_FALSE(r->submit(MotionSource::Stream, 100.0f));
    CHECK_FALSE(r->submit(MotionSource::Pattern, 100.0f));
    r->arb.override();
    CHECK_FALSE(r->submit(MotionSource::Manual, 100.0f));
    CHECK(r->census().rejected == 4);
    r->run(100'000);
    CHECK(r->emitter.n == 0);
    // Power on is not a loss: home survives it, and the operator's jog moves.
    r->arb.setMotorPowered(true);
    CHECK(r->census().motor_on);
    CHECK(r->census().homed);
    CHECK(r->submit(MotionSource::Manual, 100.0f));
}

TEST_CASE("a power loss mid-move parks on the calling task, drops homed, and nothing resumes") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Stream, 200.0f));
    r->run(50'000);
    REQUIRE(r->emitter.q8 != 0);
    const int parks = r->emitter.parks;

    r->arb.setMotorPowered(false);
    CHECK(r->emitter.q8 == 0);
    CHECK(r->emitter.parks == parks + 1);
    const int32_t held = r->emitter.n;
    r->run(200'000);
    CHECK(r->emitter.n == held);
    MotionCensus c = r->census();
    CHECK_FALSE(c.motor_on);
    CHECK_FALSE(c.homed);
    // The abandoned plan is reset on the owning task, so a release can see rest.
    CHECK_FALSE(c.busy);
    CHECK_FALSE(r->submit(MotionSource::Stream, 100.0f));

    // Power back: still parked, still unhomed; the stream needs a home first.
    r->arb.setMotorPowered(true);
    r->run(200'000);
    CHECK(r->emitter.n == held);
    CHECK_FALSE(r->submit(MotionSource::Stream, 100.0f));
    r->arb.forceHome(500.0f);
    r->run(1000);
    CHECK(r->submit(MotionSource::Stream, 100.0f));
}

TEST_CASE("the release sequence on a power-cutting hub: estop, power off, release, power on, home, resume") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    r->arb.estop(true);
    r->arb.setMotorPowered(false);   // the switch host's push after the cut
    r->run(10'000);
    r->arb.estop(false);
    CHECK(r->census().paused);
    // Pre-charging: released, paused, unpowered. Home lands; resume still
    // finds the power gate shut.
    r->arb.forceHome(500.0f);
    r->run(1000);
    r->arb.pause(false);
    CHECK_FALSE(r->submit(MotionSource::Stream, 50.0f));
    r->arb.setMotorPowered(true);
    CHECK(r->census().homed);
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
    // Two samples arriving in one tick are one knot time; the second is refused.
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Stream, 20.0f));
    CHECK(r->census().demand_mm == doctest::Approx(100.0f));
    REQUIRE(r->submit(MotionSource::Manual, 390.0f));
    CHECK(r->census().demand_mm == doctest::Approx(200.0f));
    r->run(moveUs(5'000'000));
    r->arb.override();
    REQUIRE(r->submit(MotionSource::Manual, 390.0f));
    CHECK(r->census().demand_mm == doctest::Approx(390.0f));
    REQUIRE(r->submit(MotionSource::Manual, 450.0f));
    CHECK(r->census().demand_mm == doctest::Approx(400.0f));
    REQUIRE(r->submit(MotionSource::Manual, -5.0f));
    CHECK(r->census().demand_mm == doctest::Approx(0.0f));
}

TEST_CASE("a Manual move during a stream stops it at the input set, then jogs and lands; a census read changes nothing (val-hlj)") {
    // F: a stream at cruise, then the jog. G: two samples a tick apart, then
    // the jog. Each run twice, the census read every tick and never: the
    // emitter must not tell them apart. The plan ran away to 1,074,850 mm
    // (F) and 37,260 mm (G, census read) before.
    struct Run {
        std::vector<int32_t> n;
        float lo = 1e9f, hi = -1e9f, v = 0.0f, v_jog = 0.0f;
        float dv = 0.0f;   // the emitter's largest speed change in one tick, mm/s
        MotionCensus end{};
    };
    auto go = [](bool cruise, bool read) {
        auto r = rig();
        r->arb.forceHome(500.0f);
        r->arb.setWindow(100.0f, 200.0f, 400.0f);
        r->run(1000);
        Run out;
        bool jog = false;
        uint64_t since = 0;
        auto tick = [&](uint64_t us) {
            for (uint64_t t = 0; t < us; t += 1000, since += 1000) {
                r->run(1000);
                out.n.push_back(r->emitter.n);
                const size_t k = out.n.size();
                if (k >= 3) {
                    const float v1 = float(out.n[k - 1] - out.n[k - 2]) * valence::kMmPerStep * 1e3f;
                    const float v0 = float(out.n[k - 2] - out.n[k - 3]) * valence::kMmPerStep * 1e3f;
                    out.dv = std::max(out.dv, std::fabs(v1 - v0));
                }
                if (!read) continue;
                const MotionCensus c = r->census();
                out.lo = std::min(out.lo, c.plan_mm);
                out.hi = std::max(out.hi, c.plan_mm);
                out.v = std::max(out.v, std::fabs(c.velocity_mm_s));
                if (jog && since >= 50'000) out.v_jog = std::max(out.v_jog, std::fabs(c.velocity_mm_s));
            }
        };
        REQUIRE(r->submit(MotionSource::Stream, 250.0f));
        if (cruise) tick(100'000);
        else {
            tick(1000);
            REQUIRE(r->submit(MotionSource::Stream, 20.0f));
        }
        if (read) (void)r->census();
        REQUIRE(r->submit(MotionSource::Manual, 390.0f));
        jog = true;
        since = 0;
        tick(moveUs(5'000'000));
        out.end = r->census();
        return out;
    };
    for (const bool cruise : {true, false}) {
        CAPTURE(cruise);
        const Run quiet = go(cruise, false), read = go(cruise, true);
        MESSAGE("plan ", read.lo, "..", read.hi, " mm, peak ", read.v, " mm/s, jog ", read.v_jog, " mm/s, emitter step ",
                read.dv, " mm/s per tick, backstops ", read.end.backstops);
        CHECK(quiet.n == read.n);
        CHECK(read.lo >= -0.01f);
        CHECK(read.hi <= 200.01f);
        CHECK(read.v <= DEFAULT_MAX_SPEED_MM_S * 1.001f);
        // The brake ends within 50 ms; from there the jog set holds.
        CHECK(read.v_jog <= DEFAULT_JOG_MAX_SPEED_MM_S * 1.001f);
        // The brake runs under the input set's cap: one tick changes the
        // emitter's speed by at most the plan's accel and the kick, never
        // a stop from 1200 mm/s to the jog cap.
        CHECK(read.dv <= 2.0f * DEFAULT_ACCEL_MM_S2 * 1e-3f + 4.0f * valence::kMmPerStep * 1e3f);
        CHECK_FALSE(read.end.busy);
        CHECK(read.end.position_mm == doctest::Approx(200.0f).epsilon(1e-3));
        CHECK(read.end.backstops == 0);
    }
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
    // A generator's durationless point renders as its stop (the brake), so
    // the generator moves by strokes: a timed one here.
    REQUIRE(r->arb.accept(MotionIntent{MotionSource::Pattern, 350.0f, 50'000}, g_now_us));
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
    auto gen = std::make_unique<ClassicGenerator>();
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

TEST_CASE("e-stop takes the rail from both generators: releasing the latch never restarts one") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    REQUIRE(r->arb.acquireRail(MotionSource::Pattern));
    REQUIRE(r->submit(MotionSource::Pattern, 100.0f));
    r->arb.estop(true);
    r->run(1000);
    r->arb.estop(false);
    r->arb.forceHome(500.0f);
    r->arb.pause(false);
    r->run(1000);
    CHECK_FALSE(r->submit(MotionSource::Pattern, 150.0f));
    CHECK_FALSE(r->submit(MotionSource::Advanced, 150.0f));
    CHECK(r->submit(MotionSource::Stream, 150.0f));
    // Either generator's start reopens it, and it holds the rail.
    CHECK(r->arb.acquireRail(MotionSource::Advanced));
    CHECK(r->submit(MotionSource::Advanced, 150.0f));
    CHECK_FALSE(r->submit(MotionSource::Pattern, 150.0f));
}

TEST_CASE("RFC-093: the two generators never share the rail, and nothing hands it over") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    CHECK_FALSE(r->arb.acquireRail(MotionSource::Stream));
    CHECK_FALSE(r->arb.acquireRail(MotionSource::Manual));

    REQUIRE(r->arb.acquireRail(MotionSource::Pattern));
    CHECK(r->arb.acquireRail(MotionSource::Pattern));          // a repeated start is idempotent
    CHECK_FALSE(r->arb.acquireRail(MotionSource::Advanced));   // SOURCE_CONFLICT
    CHECK(r->submit(MotionSource::Pattern, 100.0f));
    CHECK_FALSE(r->submit(MotionSource::Advanced, 120.0f));
    // A release by the generator that does not hold it changes nothing.
    r->arb.releaseRail(MotionSource::Advanced);
    CHECK_FALSE(r->arb.acquireRail(MotionSource::Advanced));

    // The classic stop frees it; its own brake still lands until another
    // generator takes the rail.
    r->arb.releaseRail(MotionSource::Pattern);
    CHECK(r->submit(MotionSource::Pattern, 110.0f));
    REQUIRE(r->arb.acquireRail(MotionSource::Advanced));
    CHECK(r->submit(MotionSource::Advanced, 140.0f));
    CHECK_FALSE(r->submit(MotionSource::Pattern, 100.0f));
    CHECK_FALSE(r->arb.acquireRail(MotionSource::Pattern));

    // PAUSE suspends the advanced generator as it does every source.
    r->arb.pause(true);
    CHECK_FALSE(r->submit(MotionSource::Advanced, 160.0f));
    r->run(2000);
    r->arb.pause(false);
    CHECK(r->submit(MotionSource::Advanced, 160.0f));
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
    r->run(moveUs(10'000'000));
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

TEST_CASE("return with the power gate shut: at the paused position it arrives on the spot") {
    // bd val-96m: nothing renders unpowered, so a return that waited for a
    // planner arrival held override forever.
    auto r = std::make_unique<Rig>(false);
    r->arb.forceHome(400.0f);
    r->run(1000);
    r->arb.pause(true);
    r->run(1000);
    r->arb.override();
    r->run(1000);
    const uint32_t before = r->census().returns;
    CHECK(r->arb.returnToPause() == valence::ReturnStart::arrived);
    const MotionCensus c = r->census();
    CHECK(c.returns == before + 1);
    CHECK_FALSE(c.override_mode);
    CHECK_FALSE(c.returning);
    CHECK(c.paused);
    // Nothing left to return from.
    CHECK(r->arb.returnToPause() == valence::ReturnStart::none);
}

TEST_CASE("return with the power gate shut: away from the paused position it is refused, override holds") {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 60.0f));
    r->run(4'000'000);
    REQUIRE(r->census().position_mm == doctest::Approx(60.0f).epsilon(0.01));
    r->arb.pause(true);
    r->run(1000);
    r->arb.override();
    REQUIRE(r->submit(MotionSource::Manual, 30.0f));
    r->run(3'000'000);
    REQUIRE(r->census().position_mm == doctest::Approx(30.0f).epsilon(0.02));
    r->arb.setMotorPowered(false);
    r->run(1000);
    const uint32_t before = r->census().returns;
    CHECK(r->arb.returnToPause() == valence::ReturnStart::unpowered);
    r->run(100'000);
    const MotionCensus c = r->census();
    CHECK(c.returns == before);
    CHECK(c.override_mode);
    CHECK_FALSE(c.returning);
}

TEST_CASE("a pause landing with the power gate shut records the parked carriage as the paused position") {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 80.0f));
    r->run(4'000'000);
    REQUIRE(r->census().position_mm == doctest::Approx(80.0f).epsilon(0.01));
    r->arb.setMotorPowered(false);
    r->run(1000);
    r->arb.pause(true);
    r->run(1000);
    r->arb.override();
    // The return target is where the carriage parked, not a stale earlier
    // pause, so there is nothing to travel.
    CHECK(r->arb.returnToPause() == valence::ReturnStart::arrived);
    CHECK_FALSE(r->census().override_mode);
}

TEST_CASE("a 32-segment bundle spanning the 1000 ms schedule horizon parks whole (RFC-087)") {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->run(1000);
    const uint64_t t_base = g_now_us + 20'000;
    for (int i = 0; i < 32; ++i) {
        MotionIntent in;
        in.source = MotionSource::Stream;
        in.target_mm = (i % 2 == 0) ? 260.0f : 140.0f;
        in.duration_us = 30'000;
        in.anchor_us = t_base + uint64_t(i) * 30'000;   // the last start 950 ms ahead
        REQUIRE(r->arb.accept(in, g_now_us));
    }
    r->run(1'100'000);
    const MotionCensus c = r->census();
    CHECK(c.intents == 32);
    CHECK(c.rejected == 0);
    CHECK(c.failures == 0);
}

TEST_CASE("flip: targets mirror in, positions and the window mirror out, the engine stays physical") {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->arb.setWindow(50.0f, 300.0f, 400.0f);
    r->run(1000);
    r->arb.setFlipped(true);
    // Client 100 mm is physical 300 mm.
    REQUIRE(r->submit(MotionSource::Manual, 100.0f));
    r->run(moveUs(10'000'000));
    const MotionCensus c = r->census();
    CHECK(c.position_mm == doctest::Approx(100.0f).epsilon(0.01));
    CHECK(float(c.steps) * valence::kMmPerStep == doctest::Approx(300.0f).epsilon(0.01));
    CHECK(c.win_min == doctest::Approx(100.0f));
    CHECK(c.win_max == doctest::Approx(350.0f));
    // A client target past the mirrored window clamps to it: client 380 is
    // physical 20, under the physical window's 50, so it lands at client 350.
    REQUIRE(r->submit(MotionSource::Stream, 380.0f));
    CHECK(r->census().demand_mm == doctest::Approx(350.0f));
    // Away from the client's 0 reads positive, though the carriage runs toward physical 0.
    REQUIRE(r->submit(MotionSource::Manual, 150.0f));
    r->run(300'000);
    CHECK(r->census().velocity_mm_s > 0.0f);
    r->run(moveUs(10'000'000));
    r->arb.setFlipped(false);
    CHECK(r->census().position_mm == doctest::Approx(250.0f).epsilon(0.01));

    // Home swaps ends: force_home under the flip asserts the client's 0, the
    // physical far end of the stroke.
    r->arb.setFlipped(true);
    r->arb.forceHome(400.0f);
    r->run(1000);
    CHECK(r->census().position_mm == doctest::Approx(0.0f).epsilon(0.001));
    r->arb.setFlipped(false);
    CHECK(r->census().position_mm == doctest::Approx(400.0f).epsilon(0.001));
}

TEST_CASE("RFC-095: a dwell lands as a hold segment, a live plan at rest on the bound") {
    auto r = rig();
    r->arb.forceHome(DEFAULT_MAX_RAIL_MM);
    r->run(1000);
    REQUIRE(r->arb.acquireRail(MotionSource::Advanced));
    PatternSettings s;
    s.frame = {0.0f, DEFAULT_MAX_RAIL_MM, DEFAULT_MAX_SPEED_MM_S, DEFAULT_ACCEL_MM_S2};
    s.adv_running = true;
    s.ap.master.set(40);
    s.ap.setBase(advpat::DEPTH_MAX, 80);
    s.ap.setBase(advpat::DEPTH_MIN, 20);
    s.ap.setBase(advpat::DWELL_CREST, 100);   // one stroke
    auto gen = std::make_unique<AdvancedGenerator>();
    gen->apply(s);

    MotionIntent prev{}, cur{};
    int landed = 0;
    bool held = false;
    for (int i = 0; i < 20000 && !held; ++i) {
        if (!generatorPass(*r, *gen, &cur)) continue;
        ++landed;
        held = landed > 1 && cur.target_mm == prev.target_mm;
        if (!held) {
            const MotionCensus moving = r->census();
            CHECK(moving.busy);
            CHECK_FALSE(moving.plan_hold);
        }
        prev = cur;
    }
    REQUIRE(held);
    REQUIRE(cur.duration_us > 100'000u);

    // Well inside the hold: a timed plan whose start is its end, the carriage
    // still on the bound, nothing moving.
    r->run(cur.duration_us / 2);
    const MotionCensus c = r->census();
    CHECK(c.busy);
    CHECK(c.plan_hold);
    CHECK(double(c.plan_duration_us) == doctest::Approx(double(cur.duration_us)).epsilon(0.001));
    CHECK(c.position_mm == doctest::Approx(cur.target_mm).epsilon(0.001));
    CHECK(std::fabs(c.velocity_mm_s) < 1.0f);
    r->arb.drainAnomalies();
    CHECK(r->census().anom[size_t(kinetic2::AnomalyKind::DwellZeroed)] == 0);
}

// Homed at 0 mm, the window 100..300 mm, under the factory tuning.
std::unique_ptr<Rig> rigAtOrigin() {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->arb.setWindow(100.0f, 300.0f, 400.0f);
    r->run(1000);
    return r;
}

bool tightSegment(Rig& r, uint32_t duration_us) {
    MotionIntent in;
    in.source = MotionSource::Stream;
    in.target_mm = 280.0f;
    in.duration_us = duration_us;
    in.has_end_vel = true;
    in.end_vel_mm_s = 0.0f;
    return r.arb.accept(in, g_now_us);
}

// ---- Kinetic² (bd val-klo) ------------------------------------------------------
// The knot boundary: what an intent becomes, and what the census reads back.

TEST_CASE("Kinetic² RFC-100: plan_flags from the solver, 0 with nothing in flight") {
    namespace pf = valence::plan_flags;
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->arb.setWindow(100.0f, 300.0f, 400.0f);
    r->run(1000);
    // A segment the ceilings meet: nothing spent.
    MotionIntent easy;
    easy.source = MotionSource::Stream;
    easy.target_mm = 120.0f;
    easy.duration_us = 2'000'000;
    REQUIRE(r->arb.accept(easy, g_now_us));
    r->run(1000);
    CHECK(r->census().busy);
    CHECK(r->census().plan_flags == 0);
    r->run(3'000'000);
    REQUIRE_FALSE(r->census().busy);
    CHECK(r->census().plan_flags == 0);

    // The handle renderer never gives a segment time: a deadline the
    // ceilings cannot meet is trimmed (shaped), and stretched is never set.
    auto tight = rigAtOrigin();
    REQUIRE(tightSegment(*tight, 40'000));
    tight->run(1000);
    CHECK((tight->census().plan_flags & pf::shaped) != 0);
    CHECK((tight->census().plan_flags & pf::stretched) == 0);
    tight->run(10'000'000);
    CHECK(tight->census().plan_flags == 0);
    tight->arb.drainAnomalies();
    CHECK(tight->census().anom[size_t(kinetic2::AnomalyKind::KnotTrimmed)] >= 1);

    // The window clamp moved the target: clamped, while that plan is the last.
    auto clamp = rigAtOrigin();
    MotionIntent past;
    past.source = MotionSource::Stream;
    past.target_mm = 350.0f;
    past.duration_us = 5'000'000;
    REQUIRE(clamp->arb.accept(past, g_now_us));
    clamp->run(1000);
    CHECK((clamp->census().plan_flags & pf::clamped) != 0);
}

TEST_CASE("Kinetic² samples: one behind at the grant's latency, a stream that stops rests on the newest, a backwards one refused as kind 5") {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->run(1000);
    // Off the rail end first: a curve at rest on the wall has no room to round.
    REQUIRE(r->submit(MotionSource::Manual, 100.0f));
    r->run(moveUs(5'000'000));
    REQUIRE_FALSE(r->census().busy);
    // A slow ramp of samples, 20 ms apart, then the sender stops: 5 mm/s, so
    // the stop inside one interval stays under the factory jerk ceiling.
    float target = 100.0f;
    for (int i = 0; i < 40; ++i) {
        target += 0.1f;
        REQUIRE(r->submit(MotionSource::Stream, target));
        r->run(1000);
        CHECK(r->census().mode == 2);   // chase: a sample's knot
        r->run(19'000);
    }
    // The chase (Kinetic kin-j6g): a run of samples renders as the fastest
    // legal move to rest on the newest held sample, re-planned per sample.
    // The sender stops: the plan rests on the last sample, never past it,
    // with no brake and no spend to report.
    const float last = target;
    float peak = 0.0f;
    for (int i = 0; i < 2000; ++i) {
        r->run(1000);
        peak = std::max(peak, r->census().plan_mm);
    }
    CHECK(peak <= last + 0.01f);
    CHECK_FALSE(r->census().busy);
    CHECK(r->census().plan_mm == doctest::Approx(last).epsilon(0.0001));
    r->arb.drainAnomalies();
    CHECK(r->census().anomalies == 0);

    // Two samples in one tick share a knot time: the second goes backwards.
    REQUIRE(r->submit(MotionSource::Stream, 10.0f));
    CHECK_FALSE(r->submit(MotionSource::Stream, 12.0f));
    r->arb.drainAnomalies();
    const MotionCensus c = r->census();
    CHECK(c.anom[size_t(kinetic2::AnomalyKind::KnotRefused)] == 1);
    CHECK(c.failures == 1);
    CHECK(c.rejected == 1);
}

TEST_CASE("Kinetic² starved segment stream: the engine brakes a last knot still moving from its own time, as settle") {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->run(1000);
    MotionIntent in;
    in.source = MotionSource::Stream;
    in.target_mm = 100.0f;
    in.duration_us = 1'000'000;
    in.has_end_vel = true;
    in.end_vel_mm_s = 60.0f;   // and then nothing
    REQUIRE(r->arb.accept(in, g_now_us));
    float v_prev = 0.0f, dv_max = 0.0f;
    bool settled = false;
    for (int i = 0; i < 3000; ++i) {
        r->run(1000);
        const MotionCensus c = r->census();
        dv_max = std::max(dv_max, std::fabs(c.velocity_mm_s - v_prev));
        v_prev = c.velocity_mm_s;
        settled = settled || c.mode == 3;
    }
    CHECK(settled);
    // No velocity step: one tick at the input decel is the largest change.
    CHECK(dv_max <= DEFAULT_ACCEL_MM_S2 * 1e-3f * 1.01f);
    const MotionCensus c = r->census();
    CHECK_FALSE(c.busy);
    CHECK(c.position_mm > 100.0f);   // the stop runs on past the knot, at rest
    r->arb.drainAnomalies();
    CHECK(r->census().anom[size_t(kinetic2::AnomalyKind::SettleEngaged)] == 1);
}

// RFC-105 (dd): the brake from a starved knot is the engine's guess that
// nothing follows, so a sample arriving during it re-plans from the braking
// state. An explicit brake would refuse it: its knot is due before the
// brake's end. Here the stream resumes on the line it was moving along.
TEST_CASE("Kinetic² starvation: a sample arriving during the engine's brake re-plans from it, never refused") {
    auto r = rig();
    r->arb.setInputLimits(DEFAULT_MAX_SPEED_MM_S, 100.0f, DEFAULT_INPUT_MAX_JERK_MM_S3);   // a brake of about 0.6 s
    r->arb.forceHome(400.0f);
    r->run(1000);
    MotionIntent in;
    in.source = MotionSource::Stream;
    in.target_mm = 100.0f;
    in.duration_us = 2'000'000;
    in.has_end_vel = true;
    in.end_vel_mm_s = 60.0f;   // and then a gap
    REQUIRE(r->arb.accept(in, g_now_us));
    for (int i = 0; i < 4000 && r->census().mode != 3; ++i) r->run(1000);
    REQUIRE(r->census().mode == 3);
    r->run(20'000);
    MotionCensus c = r->census();
    REQUIRE(c.mode == 3);
    const float p0 = c.plan_mm, v0 = c.velocity_mm_s;
    REQUIRE(v0 > 40.0f);
    const uint64_t t0 = g_now_us;
    const float latency_s = float(valence::sampleLatencyUs(valence::motionDefaultTuning())) * 1e-6f;
    float prev_p = p0, prev_v = v0, jump = 0.0f;
    for (int k = 0; k < 10; ++k) {
        MotionIntent s;
        s.source = MotionSource::Stream;
        s.target_mm = p0 + v0 * (float(g_now_us - t0) * 1e-6f + latency_s);
        CAPTURE(k);
        REQUIRE(r->arb.accept(s, g_now_us));
        for (int t = 0; t < 17; ++t) {
            r->run(1000);
            c = r->census();
            jump = std::max(jump, std::fabs(c.plan_mm - prev_p) - std::max(std::fabs(c.velocity_mm_s), std::fabs(prev_v)) * 1e-3f);
            prev_p = c.plan_mm;
            prev_v = c.velocity_mm_s;
            if (k == 0 && t == 0) CHECK(c.mode == 2);   // toward the sample's knot: the brake no longer renders
        }
    }
    r->run(3'000'000);
    CHECK(jump <= 0.01f);
    r->arb.drainAnomalies();
    c = r->census();
    CHECK(c.rejected == 0);
    CHECK(c.failures == 0);
    CHECK(c.anom[size_t(kinetic2::AnomalyKind::KnotRefused)] == 0);
    CHECK_FALSE(c.busy);
}

// RFC-105 (bb): a knot arriving while the carriage moves keeps the curve
// under it through the reaction horizon and re-plans from there. Measured
// through the arbiter every tick: a submit does not move the plan at that
// instant, and from one tick to the next the plan moves no further than its
// velocity carries in one tick. Liveness is asserted too, so a case whose
// knots were all refused or dropped cannot pass by standing still.
TEST_CASE("Kinetic² continuity: a segment or a sample arriving mid-piece never moves the plan under the carriage") {
    constexpr float kTolMm = 0.01f;
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 200.0f));
    r->run(moveUs(5'000'000));
    REQUIRE_FALSE(r->census().busy);

    float jump_at_submit = 0.0f, jump_per_tick = 0.0f, travel = 0.0f;
    MotionCensus prev = r->census();
    auto submit = [&](const MotionIntent& in) {
        const float before = r->census().plan_mm;
        const bool ok = r->arb.accept(in, g_now_us);
        jump_at_submit = std::max(jump_at_submit, std::fabs(r->census().plan_mm - before));
        return ok;
    };
    auto tick = [&] {
        r->run(1000);
        const MotionCensus c = r->census();
        const float carried = std::max(std::fabs(c.velocity_mm_s), std::fabs(prev.velocity_mm_s)) * 1e-3f;
        jump_per_tick = std::max(jump_per_tick, std::fabs(c.plan_mm - prev.plan_mm) - carried);
        travel += std::fabs(c.plan_mm - prev.plan_mm);
        prev = c;
    };

    SUBCASE("250 ms segments, each arriving 120 ms before its start") {
        uint64_t start = g_now_us + 120'000;
        for (int i = 0; i < 12; ++i) {
            MotionIntent in;
            in.source = MotionSource::Stream;
            in.target_mm = 200.0f + 120.0f * std::sin(0.8f * float(i + 1));
            in.duration_us = 250'000;
            in.anchor_us = start;
            REQUIRE(submit(in));
            for (int t = 0; t < 250; ++t) tick();
            start += 250'000;
        }
        for (int t = 0; t < 500; ++t) tick();
    }
    SUBCASE("a 60 Hz sample scrub: a 150 mm/s ramp, a hold, a 1.5 Hz sweep of 60 mm") {
        for (int i = 0; i < 240; ++i) {
            const float p = i < 40   ? 200.0f + 2.5f * float(i + 1)
                            : i < 60 ? 300.0f
                                     : 300.0f + 60.0f * std::sin(9.424778f * float(i - 60) / 60.0f);
            MotionIntent in;
            in.source = MotionSource::Stream;
            in.target_mm = p;
            (void)submit(in);   // a refusal is counted by the census and checked below
            for (int t = 0; t < (i % 3 == 2 ? 16 : 17); ++t) tick();
        }
        for (int t = 0; t < 500; ++t) tick();
    }
    r->arb.drainAnomalies();
    const MotionCensus c = r->census();
    MESSAGE("at submit ", jump_at_submit, " mm, per tick beyond the velocity ", jump_per_tick, " mm, travel ", travel, " mm");
    CHECK(jump_at_submit <= kTolMm);
    CHECK(jump_per_tick <= kTolMm);
    CHECK(c.rejected == 0);
    CHECK(c.failures == 0);
    CHECK(travel > 500.0f);
}

TEST_CASE("Kinetic² segments: a start past the newest knot holds until it, so the move begins at its start") {
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->run(1000);
    MotionIntent in;
    in.source = MotionSource::Stream;
    in.target_mm = 50.0f;
    in.duration_us = 500'000;
    in.anchor_us = g_now_us + 300'000;
    REQUIRE(r->arb.accept(in, g_now_us));
    r->run(290'000);
    CHECK(r->census().busy);
    CHECK(r->census().plan_hold);   // a live plan, at rest
    CHECK(std::fabs(r->census().velocity_mm_s) < 1e-3f);
    r->run(260'000);
    CHECK(r->census().velocity_mm_s > 1.0f);
    r->run(500'000);
    CHECK(r->census().plan_mm == doctest::Approx(50.0f).epsilon(1e-4));
}

// RFC-087 (bd val-dz9): a bundle replaces every segment queued at or after its
// first start; the segment in flight runs to that start and hands off there.
TEST_CASE("Kinetic² RFC-087 supersede: a bundle 40 ms out replaces the queue from its start, continuous, nothing refused") {
    constexpr float kTolMm = 0.01f;
    auto r = rig();
    r->arb.forceHome(400.0f);
    r->run(1000);

    float jump_at_submit = 0.0f, jump_per_tick = 0.0f;
    MotionCensus prev = r->census();
    auto segment = [&](float mm, uint32_t dur_us, uint64_t start, bool first) {
        MotionIntent in;
        in.source = MotionSource::Stream;
        in.target_mm = mm;
        in.duration_us = dur_us;
        in.anchor_us = start;
        in.supersede = first;   // what onStreamBundle sets on a bundle's first segment
        const float before = r->census().plan_mm;
        const bool ok = r->arb.accept(in, g_now_us);
        jump_at_submit = std::max(jump_at_submit, std::fabs(r->census().plan_mm - before));
        return ok;
    };
    auto tick = [&] {
        r->run(1000);
        const MotionCensus c = r->census();
        const float carried = std::max(std::fabs(c.velocity_mm_s), std::fabs(prev.velocity_mm_s)) * 1e-3f;
        jump_per_tick = std::max(jump_per_tick, std::fabs(c.plan_mm - prev.plan_mm) - carried);
        prev = c;
    };

    // The first bundle: 250 ms of 25 ms segments climbing 0 -> 50 mm.
    const uint64_t t0 = g_now_us + 10'000;
    for (int i = 0; i < 10; ++i) REQUIRE(segment(5.0f * float(i + 1), 25'000, t0 + uint64_t(i) * 25'000, i == 0));
    for (int t = 0; t < 100; ++t) tick();
    // The seek: a bundle starting 40 ms out, back down to 20 mm and holding.
    const uint64_t t_base = g_now_us + 40'000;
    REQUIRE(segment(20.0f, 150'000, t_base, true));
    REQUIRE(segment(20.0f, 50'000, t_base + 150'000, false));
    while (g_now_us + 1000 < t_base) tick();
    const float at_base_mm = prev.plan_mm;
    const float at_base_v = prev.velocity_mm_s;
    tick();
    for (int t = 0; t < 99; ++t) tick();
    const float mid_mm = prev.plan_mm;   // t_base + 100 ms: the queued climb would be near 48 mm
    for (int t = 0; t < 400; ++t) tick();

    r->arb.drainAnomalies();
    const MotionCensus c = r->census();
    MESSAGE("hand-off at ", at_base_mm, " mm moving ", at_base_v, " mm/s, 100 ms later ", mid_mm, " mm, end ", c.plan_mm,
            " mm; at submit ", jump_at_submit, " mm, per tick beyond the velocity ", jump_per_tick, " mm");
    CHECK(at_base_v > 50.0f);   // the climb in flight ran to the hand-off
    CHECK(at_base_mm > 20.0f);
    CHECK(mid_mm < 35.0f);      // the new bundle, not the queue, from t_base
    CHECK(c.plan_mm == doctest::Approx(20.0f).epsilon(1e-3));
    CHECK_FALSE(c.busy);
    CHECK(jump_at_submit <= kTolMm);
    CHECK(jump_per_tick <= kTolMm);
    CHECK(c.intents == 12);
    CHECK(c.rejected == 0);
    CHECK(c.failures == 0);
    CHECK(c.anom[size_t(kinetic2::AnomalyKind::KnotRefused)] == 0);
}

// ---- homing (bd val-dbo, val-zsr, val-cp5) -----------------------------------

namespace {

using valence::HomeSense;
using valence::HomeStart;
using valence::kHomeSafetyMarginMm;
using valence::kHomeSearchMarginMm;
using valence::kMmPerStep;

// Hard stops in the emitter's boot frame, each HIGH at or past it on its side
// of 0: the home stop and, when given, the far stop. `forced` overrides the
// pre-cycle probe; `sticky` holds the line HIGH once it has risen. Wired by
// isr(), every rising edge calls the arbiter's interrupt entry from inside
// the emitter's render, between two edges, as the board's GPIO interrupt does.
class FakeSense final : public HomeSense {
public:
    FakeSense(const TestEmitter& e, float stop, std::optional<float> far_stop = std::nullopt)
        : emitter(e), stop_mm(stop), far_mm(far_stop) {}
    bool present() const override { return true; }
    Probe probe() override {
        if (forced) return *forced;
        return level() ? Probe::high : Probe::low;
    }
    bool high() override {
        const bool h = level();
        risen = risen || h;
        return h || (sticky && risen);
    }
    static bool past(float p, float stop) { return stop < 0.0f ? p <= stop : p >= stop; }
    bool level() const {
        const float p = bootMm();
        return past(p, stop_mm) || (far_mm && past(p, *far_mm));
    }
    float bootMm() const { return float(emitter.n) * kMmPerStep; }

    // The emitter's per-edge hook: the board's rising-edge interrupt.
    void edge() {
        const bool now = level();
        if (now && !was) {
            rises.push_back(emitter.n);
            if (arb != nullptr) {
                arb->homeSenseRose();
                if (emitter.q8 == 0) ++isr_parks;
            }
        }
        was = now;
    }

    const TestEmitter& emitter;
    float stop_mm;
    std::optional<float> far_mm;
    std::optional<Probe> forced;
    bool sticky = false;
    bool risen = false;
    MotionArbiter* arb = nullptr;
    bool was = false;
    std::vector<int32_t> rises;   // the emitter count at each rising edge
    int isr_parks = 0;            // rises the interrupt entry parked on the spot
};

// Edge-level render with the sense's rising edges hooked. `interrupt` false
// keeps the per-edge watch but never calls the arbiter: the tick-side park.
void isr(Rig& r, FakeSense& s, bool interrupt = true) {
    s.arb = interrupt ? &r.arb : nullptr;
    s.was = s.level();
    r.emitter.on_edge = [&s] { s.edge(); };
}

// Runs `us` in 1 ms ticks and records the commanded speed the carriage met the
// stop at, one per rising edge of the stop line (approach or re-touch): the
// speed of the tick before, because the stall parks within the tick it rises.
std::vector<float> contactSpeeds(Rig& r, const FakeSense& s, uint64_t us) {
    std::vector<float> v;
    bool was = s.level();
    float speed = 0.0f;
    for (uint64_t t = 0; t < us; t += 1000) {
        r.run(1000);
        const bool now = s.level();
        if (now && !was) v.push_back(speed);
        was = now;
        speed = std::fabs(r.census().velocity_mm_s);
    }
    return v;
}

// Each datum lies within a tick of re-touch travel and the one-step latency
// correction of its stop: this sense has no latency of its own.
const float kDatumTol = valence::homeTouchMmS(DEFAULT_HOME_SPEED_MM_S) * 0.001f + kMmPerStep;

}  // namespace

// The operator's rulings 2026-10-06: the frame is the usable rail, 0.0 mm a
// safety margin off the home stop's datum, the rail ending one short of the
// far stop's; a 200 mm stop-to-stop rail is a 190 mm usable one.
TEST_CASE("home: two legs, 0.0 mm a safety margin off the home datum, the rail the usable length; it ends at the rail") {
    auto r = rig();
    FakeSense s{r->emitter, -30.0f, 170.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    REQUIRE(r->arb.home() == HomeStart::started);
    CHECK(r->arb.home() == HomeStart::started);   // one cycle; a repeat changes nothing
    r->run(1000);
    CHECK(r->census().homing);
    CHECK_FALSE(r->census().homed);
    CHECK_FALSE(r->submit(MotionSource::Manual, 100.0f));   // the cycle owns the rail
    r->run(20'000'000);
    const MotionCensus c = r->census();
    CHECK_FALSE(c.homing);
    CHECK(c.homed);
    CHECK(c.homes == 1);
    CHECK(c.home_fails == 0);
    CHECK(std::fabs(c.home_rail_mm - (200.0f - 2.0f * kHomeSafetyMarginMm)) <= 2.0f * kDatumTol);
    CHECK(c.rail_mm == c.home_rail_mm);
    CHECK(r->arb.rail() == c.home_rail_mm);
    // The final backoff lands on the rail's far end, the count, not a plan.
    CHECK(std::fabs(c.position_mm - c.home_rail_mm) <= kMmPerStep);
    CHECK(std::fabs(c.plan_mm - c.position_mm) <= kMmPerStep);   // the planner was reset at the count
    const float zeroBoot = s.bootMm() - c.position_mm;
    CHECK(std::fabs(zeroBoot - (-30.0f + kHomeSafetyMarginMm)) <= kDatumTol);
    CHECK_FALSE(s.level());   // backed off the far stop
    CHECK(r->submit(MotionSource::Stream, 50.0f));   // homed: the input set moves
}

TEST_CASE("home: the window clamp never admits a point inside either safety margin") {
    auto r = rig();
    FakeSense s{r->emitter, -30.0f, 170.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    REQUIRE(r->arb.home() == HomeStart::started);
    r->run(20'000'000);
    const float rail = r->census().rail_mm;
    REQUIRE(r->census().homed);
    r->arb.setWindow(0.0f, rail, rail);   // the hub's adoption
    r->run(2000);
    REQUIRE(r->submit(MotionSource::Manual, -kHomeSafetyMarginMm + 1.0f));
    r->run(moveUs(6'000'000));
    CHECK(std::fabs(r->census().position_mm) <= kMmPerStep);
    CHECK(std::fabs(s.bootMm() - (-30.0f + kHomeSafetyMarginMm)) <= kDatumTol + kMmPerStep);
    r->arb.override();   // the jog under override reaches the whole rail, and no further
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, rail + kHomeSafetyMarginMm - 1.0f));
    r->run(moveUs(6'000'000));
    CHECK(std::fabs(r->census().position_mm - rail) <= kMmPerStep);
    CHECK_FALSE(s.level());
}

TEST_CASE("home: the approach steers home_speed on its first tick, never a ramp, and no planner") {
    auto r = rig();
    FakeSense s{r->emitter, -30.0f, 170.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    REQUIRE(r->arb.home() == HomeStart::started);
    const uint32_t plans = r->census().plans;
    r->run(1000);
    static_assert(valence::kHomeRampMs == 0, "this case pins the factory build: no ramp");
    CHECK(r->emitter.q8 == valence::steerWord(-DEFAULT_HOME_SPEED_MM_S).step_q8);
    CHECK_FALSE(r->emitter.fwd);
    CHECK(r->census().velocity_mm_s == doctest::Approx(-DEFAULT_HOME_SPEED_MM_S));
    // Constant: the same word every tick until contact, whatever the jog accel.
    r->arb.setJogLimits(DEFAULT_JOG_MAX_SPEED_MM_S, 1.0f);
    for (int i = 0; i < 500; ++i) {
        r->run(1000);
        REQUIRE(r->emitter.q8 == valence::steerWord(-DEFAULT_HOME_SPEED_MM_S).step_q8);
    }
    CHECK(s.bootMm() == doctest::Approx(-0.5f * DEFAULT_HOME_SPEED_MM_S).epsilon(0.001));
    CHECK(std::fabs(r->census().plan_mm - r->census().position_mm) < 1e-6f);   // the census plan is the count
    r->run(20'000'000);
    CHECK(r->census().homed);
    CHECK(r->census().plans == plans);   // no knot was ever planned
}

// bd val-tib: the operator saw the zero off the stall and moving between homes.
// Every cycle, wherever it starts and after the hub's max_rail write-through,
// puts 0.0 mm on the same emitter count, measures the same rail, and ends with
// the count on the rail's far end.
TEST_CASE("home: consecutive cycles land the same datums and end on the rail's far end") {
    auto r = rig();
    FakeSense s{r->emitter, -30.0f, 170.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    std::optional<float> zero0, rail0;
    for (int i = 0; i < 3; ++i) {
        CAPTURE(i);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(20'000'000);
        const MotionCensus c = r->census();
        REQUIRE(c.homed);
        REQUIRE(c.homes == uint32_t(i + 1));
        CHECK(std::fabs(c.position_mm - c.home_rail_mm) <= kMmPerStep);
        const float zeroBoot = s.bootMm() - c.position_mm;
        CHECK(std::fabs(zeroBoot - (-30.0f + kHomeSafetyMarginMm)) <= kDatumTol);
        if (!zero0) zero0 = zeroBoot;
        if (!rail0) rail0 = c.home_rail_mm;
        CHECK(std::fabs(zeroBoot - *zero0) <= 2.0f * kMmPerStep);
        CHECK(std::fabs(c.home_rail_mm - *rail0) <= 2.0f * kMmPerStep);
        // The hub's adoption (ValenceDevice::adoptMeasuredRail), then a move
        // so the next cycle starts somewhere else.
        r->arb.setWindow(0.0f, 150.0f, c.home_rail_mm);
        r->run(2000);
        CHECK(r->census().position_mm == doctest::Approx(c.position_mm));   // a relabel is not motion
        REQUIRE(r->submit(MotionSource::Manual, 40.0f * float(i + 1)));
        r->run(8'000'000);
        CHECK(std::fabs(r->census().position_mm - 40.0f * float(i + 1)) <= kMmPerStep);
        CHECK(std::fabs(s.bootMm() - (*zero0 + 40.0f * float(i + 1))) <= 3.0f * kMmPerStep);
    }
}

// The bench failure (bd val-tib): the far leg ran its whole search without a
// stall. Positions stay measured from the home datum and the rail at max_rail.
TEST_CASE("home: a far-leg failure leaves positions measured from the home datum and the rail at max_rail") {
    auto r = rig();
    r->arb.setWindow(0.0f, 100.0f, 100.0f);
    r->run(1000);
    FakeSense s{r->emitter, -30.0f, 500.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    for (int i = 0; i < 2; ++i) {
        CAPTURE(i);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(15'000'000);
        const MotionCensus c = r->census();
        REQUIRE(c.home_fails == uint32_t(i + 1));
        REQUIRE(c.home_fail_leg == 1);
        CHECK_FALSE(c.homed);
        CHECK(c.rail_mm == 100.0f);
        // The far search ends max_rail plus both margins plus one search
        // margin from the home datum, which reads minus a margin.
        CHECK(std::fabs(c.position_mm - (100.0f + kHomeSafetyMarginMm + kHomeSearchMarginMm)) <=
              kDatumTol + kMmPerStep);
        CHECK(std::fabs((s.bootMm() - c.position_mm) - (-30.0f + kHomeSafetyMarginMm)) <= kDatumTol);
    }
}

TEST_CASE("home: approach at home_speed held to the jog speed, re-touch at a quarter of it, floored") {
    CHECK(valence::homeTouchMmS(40.0f) == 10.0f);
    CHECK(valence::homeTouchMmS(100.0f) == 25.0f);
    CHECK(valence::homeTouchMmS(20.0f) == valence::kHomeTouchFloorMmS);
    CHECK(valence::homeTouchMmS(6.0f) == 6.0f);   // never above the approach

    SUBCASE("factory") {
        auto r = rig();
        FakeSense s{r->emitter, -30.0f, 170.0f};
        isr(*r, s);
        r->arb.setHomeSense(s);
        REQUIRE(r->arb.home() == HomeStart::started);
        const std::vector<float> v = contactSpeeds(*r, s, 20'000'000);
        REQUIRE(v.size() == 4);   // home approach, home re-touch, far approach, far re-touch
        CHECK(v[0] == doctest::Approx(DEFAULT_HOME_SPEED_MM_S));
        CHECK(v[1] == doctest::Approx(10.0f));
        CHECK(v[2] == doctest::Approx(DEFAULT_HOME_SPEED_MM_S));
        CHECK(v[3] == doctest::Approx(10.0f));
        CHECK(r->census().homed);
    }
    SUBCASE("a live home_speed, held to the jog speed") {
        auto r = rig();
        valence::MotionTuning t = valence::motionDefaultTuning();
        t.home_speed = 20.0f;
        r->arb.applyTuning(t);
        r->arb.setJogLimits(15.0f, DEFAULT_JOG_ACCEL_MM_S2);
        FakeSense s{r->emitter, -30.0f, 70.0f};
        isr(*r, s);
        r->arb.setHomeSense(s);
        REQUIRE(r->arb.home() == HomeStart::started);
        const std::vector<float> v = contactSpeeds(*r, s, 30'000'000);
        REQUIRE(v.size() == 4);
        CHECK(v[0] == doctest::Approx(15.0f));
        CHECK(v[1] == doctest::Approx(valence::kHomeTouchFloorMmS));
        CHECK(v[2] == doctest::Approx(15.0f));
        CHECK(v[3] == doctest::Approx(valence::kHomeTouchFloorMmS));
        CHECK(r->census().homed);
        CHECK(r->census().home_rail_mm == doctest::Approx(100.0f - 2.0f * kHomeSafetyMarginMm).epsilon(0.002));
    }
}

TEST_CASE("home: each backoff is the safety margin, counted on the step count from the stall") {
    auto r = rig();
    FakeSense s{r->emitter, -30.0f, 170.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    REQUIRE(r->arb.home() == HomeStart::started);
    // The farthest the carriage gets from each stall before its re-touch.
    const int32_t margin = int32_t(std::lround(kHomeSafetyMarginMm * valence::kStepsPerMm));
    int32_t back_home = std::numeric_limits<int32_t>::min();
    int32_t back_far = std::numeric_limits<int32_t>::max();
    for (int t = 0; t < 20'000 && !r->census().homed; ++t) {
        r->run(1000);
        if (s.rises.size() == 1) back_home = std::max(back_home, r->emitter.n);
        if (s.rises.size() == 3) back_far = std::min(back_far, r->emitter.n);
    }
    REQUIRE(s.rises.size() == 4);
    CHECK(std::abs(back_home - (s.rises[0] + margin)) <= 1);
    CHECK(std::abs(back_far - (s.rises[2] - margin)) <= 1);
    // The final backoff: from the far datum to the rail's far end.
    CHECK(std::abs(r->emitter.n - (s.rises[3] - margin)) <= 2);
}

TEST_CASE("home: under the flip the home leg runs to the far end, 0.0 mm is there, and it ends at the client's far end") {
    auto r = rig();
    r->arb.setFlipped(true);
    FakeSense s{r->emitter, 40.0f, -160.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    REQUIRE(r->arb.home() == HomeStart::started);
    r->run(20'000'000);
    const MotionCensus c = r->census();
    REQUIRE(c.homed);
    CHECK(c.home_rail_mm == doctest::Approx(200.0f - 2.0f * kHomeSafetyMarginMm).epsilon(0.002));
    // Client frame: 0 a margin off the physical high stop, the carriage at the
    // rail's other end, a margin off the low one.
    CHECK(std::fabs(c.position_mm - c.home_rail_mm) <= kMmPerStep);
    CHECK(s.bootMm() == doctest::Approx(-160.0f + kHomeSafetyMarginMm).epsilon(0.002));
}

TEST_CASE("home: a failure on either leg ends unhomed, names the leg, and stores no rail") {
    SUBCASE("home end: no stall across max_rail") {
        auto r = rig();
        r->arb.setWindow(0.0f, 30.0f, 30.0f);   // max_rail 30: a 60 mm search
        r->run(1000);
        FakeSense s{r->emitter, -1000.0f};
        isr(*r, s);
        r->arb.setHomeSense(s);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(6'000'000);
        const MotionCensus c = r->census();
        CHECK_FALSE(c.homing);
        CHECK_FALSE(c.homed);
        CHECK(c.homes == 0);
        CHECK(c.home_fails == 1);
        CHECK(c.home_fail_leg == 0);
        CHECK(std::string(c.home_fail_why) == "no stall across max_rail");
        CHECK_FALSE(c.busy);
        CHECK(s.bootMm() == doctest::Approx(-(30.0f + 2.0f * kHomeSafetyMarginMm + 2.0f * kHomeSearchMarginMm))
                                .epsilon(0.001));
        CHECK(r->submit(MotionSource::Manual, 10.0f));   // the rail is free again
    }
    SUBCASE("far end: no stall across max_rail from the home datum") {
        auto r = rig();
        r->arb.setWindow(0.0f, 100.0f, 100.0f);
        r->run(1000);
        FakeSense s{r->emitter, -30.0f, 500.0f};
        isr(*r, s);
        r->arb.setHomeSense(s);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(15'000'000);
        const MotionCensus c = r->census();
        CHECK_FALSE(c.homing);
        CHECK_FALSE(c.homed);
        CHECK(c.homes == 0);
        CHECK(c.home_fails == 1);
        CHECK(c.home_fail_leg == 1);
        CHECK(std::string(c.home_fail_why) == "no stall across max_rail");
        CHECK(c.home_rail_mm == 0.0f);
        CHECK(s.bootMm() ==
              doctest::Approx(-30.0f + 100.0f + 2.0f * kHomeSafetyMarginMm + kHomeSearchMarginMm).epsilon(0.001));
    }
    SUBCASE("a stop-to-stop rail under the floor plus both margins") {
        auto r = rig();
        FakeSense s{r->emitter, -5.0f, -5.0f + MIN_RAIL_MM + 2.0f * kHomeSafetyMarginMm - 1.0f};
        isr(*r, s);
        r->arb.setHomeSense(s);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(20'000'000);
        const MotionCensus c = r->census();
        CHECK_FALSE(c.homed);
        CHECK(c.home_fail_leg == 1);
        CHECK(std::string(c.home_fail_why) == "the usable rail between the safety margins is under the rail floor");
    }
    SUBCASE("home end: a line still HIGH after the backoff fails before any re-touch") {
        auto r = rig();
        FakeSense s{r->emitter, -30.0f, 170.0f};
        s.sticky = true;
        isr(*r, s);
        r->arb.setHomeSense(s);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(5'000'000);
        const MotionCensus c = r->census();
        CHECK_FALSE(c.homed);
        CHECK(c.home_fails == 1);
        CHECK(c.home_fail_leg == 0);
        CHECK(std::string(c.home_fail_why) == "still on the stop after the backoff (lower home_speed)");
    }
    SUBCASE("far end: ESTOP during the far approach parks at once and ends the cycle") {
        auto r = rig();
        FakeSense s{r->emitter, -30.0f, 170.0f};
        isr(*r, s);
        r->arb.setHomeSense(s);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(3'000'000);   // the home leg takes about 1.4 s at the factory speeds
        REQUIRE(r->census().homing);
        REQUIRE(r->emitter.q8 != 0);   // seeking
        r->arb.estop(true);
        CHECK(r->emitter.q8 == 0);     // on the calling task, before any tick
        const int32_t n = r->emitter.n;
        r->run(10'000);
        const MotionCensus c = r->census();
        CHECK(r->emitter.n == n);
        CHECK_FALSE(c.homing);
        CHECK_FALSE(c.homed);
        CHECK(c.home_fail_leg == 1);
        CHECK(std::string(c.home_fail_why) == "ESTOP");
    }
}

TEST_CASE("home: the deadline covers every leg at its constant speed and 2 s, and ends a cycle whose count never moves") {
    // max_rail 500 at the factory speeds: (530 + 520) mm of seeks and three
    // 5 mm backoffs at 40 mm/s, two 15 mm re-touches at 10 mm/s, no ramp.
    CHECK(valence::homeCycleS(500.0f, 40.0f) == doctest::Approx(1065.0f / 40.0f + 3.0f));

    auto r = rig();
    r->arb.setWindow(0.0f, 200.0f, 200.0f);
    r->run(1000);
    FakeSense s{r->emitter, -30.0f, 170.0f};
    r->arb.setHomeSense(s);
    r->emitter.frozen = true;   // a dead emitter: steered, never counting
    REQUIRE(r->arb.home() == HomeStart::started);
    const float budgetS = valence::homeCycleS(200.0f, DEFAULT_HOME_SPEED_MM_S) +
                          float(valence::kHomeTimeoutMarginUs) * 1e-6f;
    r->run(uint64_t((budgetS - 0.3f) * 1e6f));
    CHECK(r->census().homing);
    CHECK(r->emitter.q8 != 0);
    r->run(600'000);
    const MotionCensus c = r->census();
    CHECK_FALSE(c.homing);
    CHECK_FALSE(c.homed);
    CHECK(c.home_fail_leg == 0);
    CHECK(std::string(c.home_fail_why) == "timed out");
    CHECK(r->emitter.q8 == 0);   // parked
    CHECK_FALSE(c.busy);
}

TEST_CASE("home: ESTOP and PAUSE abort the cycle unhomed") {
    SUBCASE("ESTOP") {
        auto r = rig();
        FakeSense s{r->emitter, -100.0f};
        isr(*r, s);
        r->arb.setHomeSense(s);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(1'000'000);
        r->arb.estop(true);
        r->run(10'000);
        const MotionCensus c = r->census();
        CHECK_FALSE(c.homing);
        CHECK_FALSE(c.homed);
        CHECK(c.home_fail_leg == 0);
        CHECK(r->emitter.q8 == 0);
        CHECK(r->arb.home() == HomeStart::estop);
    }
    SUBCASE("PAUSE, latched before the cycle and asked again during it") {
        auto r = rig();
        r->arb.pause(true);   // SPEC 11.1: the home verb runs under PAUSE
        r->run(1000);
        FakeSense s{r->emitter, -100.0f};
        isr(*r, s);
        r->arb.setHomeSense(s);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(1'000'000);
        CHECK(r->census().homing);
        const float moved = s.bootMm();
        CHECK(moved < -5.0f);   // the approach ran under the latch
        r->arb.pause(true);
        r->run(1000);
        CHECK(r->emitter.q8 == 0);   // a seek has no brake: it parks on the next tick
        const float at = s.bootMm();
        r->run(500'000);
        const MotionCensus c = r->census();
        CHECK_FALSE(c.homing);
        CHECK_FALSE(c.homed);
        CHECK_FALSE(c.busy);
        CHECK(std::string(c.home_fail_why) == "paused");
        CHECK(s.bootMm() == at);
        CHECK(s.bootMm() > -100.0f);   // stopped short of the stop
    }
}

TEST_CASE("home: refused without a sense, with an undriven line, or with a stall already read") {
    auto r = rig();
    CHECK(r->arb.home() == HomeStart::no_sense);   // the absent default
    FakeSense s{r->emitter, -30.0f};
    r->arb.setHomeSense(s);
    s.forced = HomeSense::Probe::undriven;
    CHECK(r->arb.home() == HomeStart::undriven);
    s.forced = HomeSense::Probe::high;
    CHECK(r->arb.home() == HomeStart::sense_high);
    r->run(10'000);
    CHECK_FALSE(r->census().homing);
    CHECK(r->census().intents == 0);   // nothing moved
    s.forced.reset();
    CHECK(r->arb.home() == HomeStart::started);
}

// The operator's ruling 2026-10-06: the stall stops the carriage at once. The
// interrupt parks the emitter on the edge that rises, inside the render and
// before the next tick, and the count never moves further into the stop; the
// next move is the backoff, away from it.
TEST_CASE("home: every stall parks from the interrupt on its own edge; nothing runs on into the stop") {
    auto r = rig();
    FakeSense s{r->emitter, -30.0f, 170.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    REQUIRE(r->arb.home() == HomeStart::started);
    int32_t worst_overrun = 0;
    size_t seen = 0;
    int32_t dir = -1;   // the home approach runs toward the low stop
    for (int t = 0; t < 40'000 && !r->census().homed; ++t) {
        const int32_t before = r->emitter.n;
        r->run(1000);
        if (s.rises.size() > seen) {
            seen = s.rises.size();
            dir = s.rises.back() > before ? 1 : -1;
            CHECK(r->emitter.q8 == 0);   // still parked when the tick that saw it ends
            for (int k = 0; k < 20; ++k) {
                worst_overrun = std::max(worst_overrun, dir * (r->emitter.n - s.rises.back()));
                r->run(1000);
            }
        }
    }
    MESSAGE(s.rises.size(), " contacts, ", s.isr_parks, " parked in the interrupt, worst overrun ",
            worst_overrun, " steps");
    CHECK(s.rises.size() == 4);   // home approach, home re-touch, far approach, far re-touch
    CHECK(s.isr_parks == 4);
    CHECK(worst_overrun == 0);
    CHECK(r->census().homed);
}

TEST_CASE("home: without the interrupt the tick parks, one tick of travel past contact at most") {
    auto r = rig();
    FakeSense s{r->emitter, -30.0f, 170.0f};
    isr(*r, s, false);
    r->arb.setHomeSense(s);
    REQUIRE(r->arb.home() == HomeStart::started);
    int32_t worst_overrun = 0;
    size_t seen = 0;
    for (int t = 0; t < 40'000 && !r->census().homed; ++t) {
        const int32_t before = r->emitter.n;
        r->run(1000);
        if (s.rises.size() > seen) {
            seen = s.rises.size();
            const int32_t dir = s.rises.back() > before ? 1 : -1;
            r->run(1000);   // the tick that reads the line parks
            worst_overrun = std::max(worst_overrun, dir * (r->emitter.n - s.rises.back()));
        }
    }
    CHECK(s.rises.size() == 4);
    CHECK(s.isr_parks == 0);
    CHECK(worst_overrun <= int32_t(std::ceil(DEFAULT_HOME_SPEED_MM_S * 0.001f * valence::kStepsPerMm)));
    CHECK(r->census().homed);
    CHECK(std::fabs(r->census().home_rail_mm - (200.0f - 2.0f * kHomeSafetyMarginMm)) <=
          2.0f * (DEFAULT_HOME_SPEED_MM_S * 0.001f + kDatumTol));
}

TEST_CASE("home: a rise the confirming read rejects parks the seek for one tick, then it seeks on") {
    auto r = rig();
    FakeSense s{r->emitter, -30.0f, 170.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    REQUIRE(r->arb.home() == HomeStart::started);
    r->run(100'000);
    REQUIRE(r->emitter.q8 != 0);
    r->arb.homeSenseRose();   // a coupled spike: the line is LOW again by the read
    CHECK(r->emitter.q8 == 0);
    const int32_t n = r->emitter.n;
    r->run(1000);   // the read rejects it and the seek steers again
    CHECK(r->emitter.n == n);
    CHECK(r->emitter.q8 == valence::steerWord(-DEFAULT_HOME_SPEED_MM_S).step_q8);
    r->arb.homeSenseRose();   // armed again: a second spike parks too
    CHECK(r->emitter.q8 == 0);
    r->run(20'000'000);
    CHECK(r->census().homed);
    CHECK(std::fabs(r->census().home_rail_mm - (200.0f - 2.0f * kHomeSafetyMarginMm)) <= 2.0f * kDatumTol);
}

TEST_CASE("home: the interrupt entry does nothing while no seek is armed") {
    auto r = rig();
    REQUIRE(r->submit(MotionSource::Manual, 50.0f));
    r->run(100'000);
    REQUIRE(r->emitter.q8 != 0);
    r->arb.homeSenseRose();   // no cycle: a jog is not the interrupt's
    CHECK(r->emitter.q8 != 0);
    FakeSense s{r->emitter, -1000.0f, 1000.0f};
    r->arb.setHomeSense(s);
    r->run(moveUs(3'000'000));
    REQUIRE(r->arb.home() == HomeStart::started);
    r->run(1000);
    REQUIRE(r->emitter.q8 != 0);
    r->arb.estop(true);
    r->run(1000);
    r->arb.homeSenseRose();   // the cycle ended: disarmed
    CHECK(r->census().home_fails == 1);
}

// ---- the position backstop and the stall rule (bd val-1w8) -------------------

namespace {

// The velocity the emitter holds, mm/s, signed: what it renders until the
// next steer.
double steeredMmS(const TestEmitter& e) {
    if (e.q8 == 0) return 0.0;
    const double v = double(valence::kLpClockHz) * 256.0 / double(e.q8) / double(valence::kStepsPerMm);
    return e.fwd ? v : -v;
}

// The 2026-10-06 incident's machine: rail 268.11 mm, window [92, 176], jog
// 200 mm/s and 200 mm/s^2, input 1000 mm/s, 50,000 mm/s^2, 10,000,000 mm/s^3,
// the carriage parked mid-window.
constexpr float kIncWinLo = 92.0f;
constexpr float kIncWinHi = 176.0f;
constexpr float kIncRail  = 268.11f;
constexpr float kIncVmax  = 1000.0f;
constexpr float kIncAmax  = 50000.0f;

std::unique_ptr<Rig> incidentRig() {
    auto r = rig();
    r->arb.setWindow(kIncWinLo, kIncWinHi, kIncRail);
    r->arb.forceHome(kIncRail);
    r->arb.setJogLimits(200.0f, 200.0f);
    r->arb.setInputLimits(kIncVmax, kIncAmax, 10'000'000.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 134.0f));
    r->run(moveUs(3'000'000));
    REQUIRE(std::fabs(r->arb.positionMm() - 134.0f) <= 2.0f * valence::kMmPerStep);
    return r;
}

// One tick `us` after the last: the emitter renders the word it holds the
// whole time (open loop, as the LP core does while the HP side stalls), then
// the arbiter evaluates with the measured interval.
void lateTick(Rig& r, uint64_t us) {
    g_now_us += us;
    r.emitter.advance(double(us) * 1e-6);
    r.arb.evaluate(g_now_us, float(us) * 1e-6f);
}

// The most any single steer may ask for: vmax plus one capped tick of amax.
constexpr double kSteerCeiling = double(kIncVmax) + double(kIncAmax) * double(valence::kTickDtCapS);
constexpr float  kStepTol = 2.0f * valence::kMmPerStep;

}  // namespace

TEST_CASE("backstop: a brake that runs past the window is held at its edge, flagged, counted once") {
    auto r = incidentRig();
    // An input accel far under the jog's: the PAUSE brake plans at the input
    // set (brakeToRest()) and so runs tens of mm past the window's edge. The
    // pause lands while the jog still accelerates, so the brake starts from a
    // decel no higher than its own ceiling and never reverses.
    r->arb.setInputLimits(kIncVmax, 20.0f, 10'000'000.0f);
    REQUIRE(r->submit(MotionSource::Manual, kIncWinHi));
    for (int i = 0; i < 5000 && r->arb.positionMm() < 145.0f; ++i) r->run(1000);
    REQUIRE(r->census().velocity_mm_s > 20.0f);
    r->arb.pause(true);
    float worst = 0.0f, plan_peak = 0.0f;
    bool flagged = false;
    for (int i = 0; i < 8000; ++i) {
        r->run(1000);
        worst = std::max(worst, r->arb.positionMm());
        if (i % 20 == 0) {
            const MotionCensus c = r->census();
            plan_peak = std::max(plan_peak, c.plan_mm);
            if (c.plan_flags & valence::plan_flags::clamped) flagged = true;
        }
    }
    const MotionCensus c = r->census();
    MESSAGE("plan peak ", plan_peak, " mm, carriage peak ", worst, " mm");
    CHECK(plan_peak > kIncWinHi + 10.0f);   // the brake really ran past the edge
    CHECK(worst <= kIncWinHi + kStepTol);
    CHECK(c.position_mm == doctest::Approx(kIncWinHi).epsilon(0.0001));
    CHECK(c.backstops == 1);
    CHECK(flagged);
    // The paused position is where the carriage rests, not where the plan does.
    CHECK(c.demand_mm == doctest::Approx(kIncWinHi));
}

TEST_CASE("backstop: override lifts it to the asserted rail and nothing else does") {
    auto r = incidentRig();
    // Every source outside override ends inside the window.
    REQUIRE(r->submit(MotionSource::Manual, 250.0f));
    r->run(moveUs(3'000'000));
    CHECK(r->arb.positionMm() == doctest::Approx(kIncWinHi).epsilon(0.0001));
    REQUIRE(r->submit(MotionSource::Stream, 20.0f));
    r->run(moveUs(3'000'000));
    CHECK(r->arb.positionMm() == doctest::Approx(kIncWinLo).epsilon(0.0001));
    // Under override the jog reaches past the window, and the rail ends it.
    r->arb.override();
    REQUIRE(r->submit(MotionSource::Manual, 250.0f));
    r->run(moveUs(5'000'000));
    CHECK(r->arb.positionMm() == doctest::Approx(250.0f).epsilon(0.0001));
    REQUIRE(r->submit(MotionSource::Manual, 400.0f));
    float worst = 0.0f;
    for (int i = 0; i < 3000; ++i) {
        r->run(1000);
        worst = std::max(worst, r->arb.positionMm());
    }
    CHECK(worst <= kIncRail + kStepTol);
    CHECK(r->arb.positionMm() == doctest::Approx(kIncRail).epsilon(0.0001));
    // The return lands back in the window and override drops with it.
    REQUIRE(r->arb.returnToPause() == valence::ReturnStart::queued);
    r->run(moveUs(5'000'000));
    const MotionCensus c = r->census();
    CHECK_FALSE(c.override_mode);
    CHECK(c.position_mm == doctest::Approx(kIncWinLo).epsilon(0.0001));
    CHECK(c.backstops == 0);   // accept()'s target clamp held every case: no plan left its frame
}

TEST_CASE("stall: a 283 ms tick is counted and re-anchored, never a burst") {
    auto r = incidentRig();
    MotionIntent seg;
    seg.source = MotionSource::Stream;
    seg.target_mm = kIncWinHi;
    seg.duration_us = 200'000;
    REQUIRE(r->arb.accept(seg, g_now_us));
    r->run(50'000);
    REQUIRE(std::fabs(steeredMmS(r->emitter)) > 100.0);
    // The worst residual a stall can leave: the emitter renders nothing while
    // the plan runs to its end.
    const float plan_before = r->census().plan_mm;
    r->emitter.frozen = true;
    lateTick(*r, 283'000);
    r->emitter.frozen = false;
    const float behind = kIncWinHi - r->arb.positionMm();
    REQUIRE(behind > 20.0f);
    const double v = steeredMmS(r->emitter);
    MESSAGE("after the stall: ", behind, " mm behind the plan, steered ", v, " mm/s");
    CHECK(r->census().stalls == 1);
    CHECK(std::fabs(v) <= kSteerCeiling);
    // The steer is the plan's mean velocity across the stall plus the residual
    // kick, one capped tick of amax: never the catch-up the residual asks for.
    const double mean = double(kIncWinHi - plan_before) / 0.283;
    CHECK(std::fabs(v) <= mean + double(kIncAmax) * double(valence::kTickDtCapS) + 1.0);
    const float before = r->arb.positionMm();
    r->run(1000);
    CHECK(std::fabs(r->arb.positionMm() - before) <= float(kSteerCeiling) * 1e-3f + kStepTol);
    // The residual closes at the bounded kick, without passing the edge.
    float worst = 0.0f;
    for (int i = 0; i < 3000; ++i) {
        r->run(1000);
        worst = std::max(worst, r->arb.positionMm());
    }
    CHECK(worst <= kIncWinHi + kStepTol);
    CHECK(r->arb.positionMm() == doctest::Approx(kIncWinHi).epsilon(0.0001));
    CHECK(r->census().stalls == 1);
}

TEST_CASE("backstop: the incident's jog scrub, without stalls, with isolated 283 ms stalls, and in a stall storm") {
    // storm: every tick 283 ms late for 3 s, as the P4 ran with a 283 ms
    // solve behind every 20 Hz move.
    enum Mode { none, isolated, storm };
    for (const Mode mode : {none, isolated, storm}) {
        CAPTURE(int(mode));
        auto r = incidentRig();
        uint32_t lcg = 20261006u;
        auto rnd = [&lcg] {
            lcg = lcg * 1664525u + 1013904223u;
            return lcg >> 8;
        };
        uint32_t injected = 0, over_ceiling = 0, pushed_out = 0, overran = 0;
        float lo_seen = r->arb.positionMm(), hi_seen = lo_seen;
        const uint64_t t0 = g_now_us;
        uint64_t next_move = t0;
        while (g_now_us - t0 < 10'000'000) {
            if (g_now_us >= next_move) {
                // scrub.py's finger: a 1 Hz triangle across the window plus a
                // 7 Hz wobble, at 20 Hz.
                const double s = double(g_now_us - t0) * 1e-6;
                const double tri = 2.0 * std::fabs(s - std::floor(s + 0.5));
                double p = kIncWinLo + (kIncWinHi - kIncWinLo) * tri + 8.0 * std::sin(s * 2.0 * 3.14159265358979 * 7.0);
                p = std::fmin(std::fmax(p, double(kIncWinLo)), double(kIncWinHi));
                r->submit(MotionSource::Manual, float(p));
                next_move += 50'000;
            }
            const uint64_t at = g_now_us - t0;
            const bool late = mode == storm ? (at >= 3'000'000 && at < 6'000'000)
                                            : (mode == isolated && rnd() % 400 == 0);
            if (late) {
                // The core renders the steer it holds for kLeaseUs and stops.
                const int32_t n0 = r->emitter.n;
                const double v0 = steeredMmS(r->emitter);
                lateTick(*r, 283'000);
                ++injected;
                const double moved = std::fabs(double(r->emitter.n - n0)) * double(valence::kMmPerStep);
                if (moved > std::fabs(v0) * double(valence::kLeaseUs) * 1e-6 + 2.0 * double(valence::kMmPerStep)) ++overran;
            } else {
                r->run(1000);
            }
            const float pos = r->arb.positionMm();
            const double v = steeredMmS(r->emitter);
            lo_seen = std::min(lo_seen, pos);
            hi_seen = std::max(hi_seen, pos);
            if (std::fabs(v) > kSteerCeiling) ++over_ceiling;
            // The demand never leaves the window: a steer never carries the
            // carriage past an edge within kTickDtCapS, nor further outside.
            const double reach = double(pos) + v * double(valence::kTickDtCapS);
            if ((v > 0.0 && reach > kIncWinHi + kStepTol) || (v < 0.0 && reach < kIncWinLo - kStepTol)) ++pushed_out;
        }
        const MotionCensus c = r->census();
        MESSAGE("mode ", int(mode), ", stalls ", injected, ": carriage in [", lo_seen, ", ", hi_seen, "] mm, census stalls ", c.stalls,
                ", backstops ", c.backstops, ", lapses ", c.lease_lapses, ", fence hits ", r->emitter.fence_hits,
                ", plan_us_max ", c.plan_us_max);
        CHECK(over_ceiling == 0);
        CHECK(pushed_out == 0);
        CHECK(c.stalls == injected);
        // Every stall lapses the lease once and renders at most kLeaseUs of
        // the steer it held; the fence keeps the count in the window either
        // way, rounded outward by under a step.
        CHECK(c.lease_lapses == injected);
        CHECK(overran == 0);
        CHECK(lo_seen >= kIncWinLo - valence::kMmPerStep);
        CHECK(hi_seen <= kIncWinHi + valence::kMmPerStep);
        if (mode != none) REQUIRE(injected > 5);
    }
}

// ---- the LP fence and lease (operator ruling 2026-10-06, bd val-fi5) ----------

namespace {

// A fence word as the millimeters the carriage reads at that count.
float fenceMm(Rig& r, int32_t c) { return r.arb.positionMm() + float(c - r.emitter.n) * valence::kMmPerStep; }

// The fence holds [lo, hi] mm, rounded outward by under one step.
void checkFence(Rig& r, float lo, float hi) {
    const float flo = fenceMm(r, r.emitter.fence_lo);
    const float fhi = fenceMm(r, r.emitter.fence_hi);
    CAPTURE(flo);
    CAPTURE(fhi);
    CHECK(flo <= lo + 1e-3f);
    CHECK(flo > lo - valence::kMmPerStep - 1e-3f);
    CHECK(fhi >= hi - 1e-3f);
    CHECK(fhi < hi + valence::kMmPerStep + 1e-3f);
}

}  // namespace

TEST_CASE("fence: the window, the asserted rail under override and through its return, then the window again") {
    auto r = incidentRig();
    checkFence(*r, kIncWinLo, kIncWinHi);
    // The jog under override reaches past the window: the fence moved to the
    // rail before its first steer, or an edge would have been withheld.
    r->arb.override();
    REQUIRE(r->submit(MotionSource::Manual, 250.0f));
    r->run(moveUs(5'000'000));
    CHECK(r->arb.positionMm() == doctest::Approx(250.0f).epsilon(0.0001));
    checkFence(*r, 0.0f, kIncRail);
    REQUIRE(r->arb.returnToPause() == valence::ReturnStart::queued);
    r->run(1000);
    checkFence(*r, 0.0f, kIncRail);
    r->run(moveUs(5'000'000));
    CHECK_FALSE(r->census().override_mode);
    checkFence(*r, kIncWinLo, kIncWinHi);
    CHECK(r->emitter.fence_hits == 0);
}

TEST_CASE("fence: a home cycle searches inside the wide frame and is never fenced; the window follows the new frame") {
    auto r = rig();
    FakeSense s{r->emitter, -30.0f, 170.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    REQUIRE(r->arb.home() == HomeStart::started);
    // max_rail plus a safety margin and two search margins past each end.
    const float wide = DEFAULT_MAX_RAIL_MM + 2.0f * (kHomeSafetyMarginMm + 2.0f * kHomeSearchMarginMm);
    int ticks = 0, off = 0;
    for (int i = 0; i < 20'000; ++i) {
        r->run(1000);
        if (!r->census().homing) break;
        ++ticks;
        const float span = float(r->emitter.fence_hi - r->emitter.fence_lo) * kMmPerStep;
        if (std::fabs(span - wide) > 2.0f * kMmPerStep) ++off;
        if (r->emitter.n < r->emitter.fence_lo || r->emitter.n > r->emitter.fence_hi) ++off;
    }
    const MotionCensus c = r->census();
    REQUIRE(c.homes == 1);
    CHECK(ticks > 1000);
    CHECK(off == 0);
    CHECK(r->emitter.fence_hits == 0);
    // The backstop's frame in the measured frame: the window held in the rail.
    r->run(1000);
    checkFence(*r, 0.0f, c.rail_mm);
    r->arb.setWindow(20.0f, 150.0f, c.rail_mm);
    r->run(2000);
    checkFence(*r, 20.0f, 150.0f);
    CHECK(r->emitter.fence_hits == 0);
}

TEST_CASE("fence: a steer a 3.9 ms stall carries past the window stops on the fence, the count exactly on its edge") {
    // The backstop brake case, with a 3.9 ms stall after every tick: inside
    // the lease, so the core renders each steer for the whole stall, and the
    // wall bound only promised the edge within kTickDtCapS.
    auto r = incidentRig();
    r->arb.setInputLimits(kIncVmax, 20.0f, 10'000'000.0f);
    REQUIRE(r->submit(MotionSource::Manual, kIncWinHi));
    for (int i = 0; i < 5000 && r->arb.positionMm() < 145.0f; ++i) r->run(1000);
    REQUIRE(r->census().velocity_mm_s > 20.0f);
    r->arb.pause(true);
    int32_t peak = r->emitter.n;
    for (int i = 0; i < 4000; ++i) {
        r->run(1000);
        lateTick(*r, 3900);
        peak = std::max(peak, r->emitter.n);
    }
    const MotionCensus c = r->census();
    MESSAGE("fence hits ", r->emitter.fence_hits, ", peak count ", peak, ", fence ", r->emitter.fence_hi, ", rest ",
            c.position_mm, " mm");
    CHECK(r->emitter.fence_hits > 0);
    CHECK(peak == r->emitter.fence_hi);
    CHECK(r->emitter.n == r->emitter.fence_hi);
    CHECK(c.stalls == 4000);
    CHECK(c.lease_lapses == 0);
    checkFence(*r, kIncWinLo, kIncWinHi);
}

TEST_CASE("lease: a 283 ms stall stops the core kLeaseUs after the last tick; the next tick counts it and steers no burst") {
    auto r = incidentRig();
    MotionIntent seg;
    seg.source = MotionSource::Stream;
    seg.target_mm = kIncWinHi;
    seg.duration_us = 200'000;
    REQUIRE(r->arb.accept(seg, g_now_us));
    r->run(50'000);
    const double v0 = steeredMmS(r->emitter);
    REQUIRE(std::fabs(v0) > 100.0);
    const float plan_before = r->census().plan_mm;
    const int32_t n0 = r->emitter.n;
    lateTick(*r, 283'000);
    const double moved = double(r->emitter.n - n0) * double(kMmPerStep);
    const double v = steeredMmS(r->emitter);
    const MotionCensus c = r->census();
    MESSAGE("held ", v0, " mm/s, moved ", moved, " mm through the stall, then steered ", v, " mm/s");
    // The steer it held, for kLeaseUs, and nothing after.
    CHECK(std::fabs(moved - v0 * double(valence::kLeaseUs) * 1e-6) <= 2.0 * double(kMmPerStep));
    CHECK(r->emitter.lapse_count == 1);
    CHECK(c.lease_lapses == 1);
    CHECK(c.stalls == 1);
    // The resume is the plan's mean velocity across the stall plus one capped
    // kick, never the catch-up the residual asks for.
    const double mean = double(kIncWinHi - plan_before) / 0.283;
    CHECK(std::fabs(v) <= mean + double(kIncAmax) * double(valence::kTickDtCapS) + 1.0);
    float worst = 0.0f;
    for (int i = 0; i < 3000; ++i) {
        r->run(1000);
        worst = std::max(worst, r->arb.positionMm());
    }
    CHECK(worst <= kIncWinHi + kMmPerStep);
    CHECK(r->arb.positionMm() == doctest::Approx(kIncWinHi).epsilon(0.0001));
    CHECK(r->census().lease_lapses == 1);
    CHECK(r->emitter.fence_hits == 0);
}

// The board's run() takes now and dt BEFORE the drain, so a stall in a solve
// there lands in an evaluate() whose dt_s reads one tick. The residual the
// lapse left must stay with the bounded kick: as feedforward over that dt_s
// it is an uncommanded reversal (measured: -600 mm/s against a forward plan).
TEST_CASE("lease: a stall inside the drain resumes along the plan, never a reversal") {
    auto r = incidentRig();
    MotionIntent seg;
    seg.source = MotionSource::Stream;
    seg.target_mm = kIncWinHi;
    seg.duration_us = 200'000;
    REQUIRE(r->arb.accept(seg, g_now_us));
    r->run(50'000);
    const double v0 = steeredMmS(r->emitter);
    REQUIRE(v0 > 100.0);
    const uint64_t now = g_now_us + 1000;
    r->emitter.advance(0.284);
    r->arb.evaluate(now, 1e-3f);
    const double v1 = steeredMmS(r->emitter);
    MESSAGE("held ", v0, " mm/s, first steer after the lapse ", v1, " mm/s");
    CHECK(r->census().lease_lapses == 1);
    CHECK(v1 > 0.0);
    CHECK(v1 <= kSteerCeiling);
}

TEST_CASE("lease: the core is unleased until the first tick, and an e-stop parks it as before") {
    auto r = rig();
    CHECK_FALSE(r->emitter.live);   // begin() steers 0 and renews nothing
    r->run(1000);
    CHECK(r->emitter.live);
    r->arb.forceHome(400.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 100.0f));
    r->run(500'000);
    REQUIRE(r->emitter.q8 != 0);
    const int parks = r->emitter.parks;
    r->arb.estop(true);
    CHECK(r->emitter.q8 == 0);
    CHECK(r->emitter.parks == parks + 1);
    r->run(10'000);
    CHECK(r->emitter.q8 == 0);
    CHECK(r->emitter.live);   // the tick still renews: a lease is the HP alive, not motion allowed
    CHECK(r->census().lease_lapses == 0);
}

// Operator ruling 2026-10-06 (bd val-d11): a jog is live. The newest target
// supersedes every move queued behind the one in flight, so a scrub never
// builds a timeline and the carriage turns toward the newest target within
// the reaction horizon plus a tick.
TEST_CASE("jog is live: a 20 Hz scrub never queues, redirects on receipt, nothing refused") {
    auto r = rig();
    r->arb.forceHome(268.0f);
    r->arb.setWindow(92.0f, 176.0f, 268.0f);
    r->arb.setJogLimits(200.0f, 200.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 134.0f));
    r->run(1'500'000);
    int late_turns = 0;
    for (int i = 0; i < 60; ++i) {
        const float target = (i % 2 == 0) ? 170.0f : 98.0f;
        REQUIRE(r->submit(MotionSource::Manual, target));
        // The redirection is the ACCELERATION, bounded by the jog set: past the
        // reaction horizon the velocity changes toward the newest target (a
        // reversal from 100 mm/s under 200 mm/s^2 takes half a second, so the
        // velocity itself cannot flip within one cadence).
        r->run(6000);
        const float v0 = r->census().velocity_mm_s;
        r->run(14'000);
        const MotionCensus c = r->census();
        const float toward = target - c.plan_mm;
        const bool turned = std::fabs(toward) < 0.5f || (c.velocity_mm_s - v0) * toward > 0.0f
                            || (c.velocity_mm_s * toward > 0.0f && std::fabs(c.velocity_mm_s) >= 199.0f);
        if (!turned) ++late_turns;
        r->run(30'000);
    }
    r->run(2'000'000);
    r->arb.drainAnomalies();
    const MotionCensus c = r->census();
    MESSAGE("late turns ", late_turns, " of 60");
    CHECK(late_turns == 0);
    CHECK(c.rejected == 0);
    CHECK(c.failures == 0);
    CHECK(c.anom[size_t(kinetic2::AnomalyKind::KnotRefused)] == 0);
    CHECK_FALSE(c.busy);
    CHECK(std::fabs(c.position_mm - 98.0f) <= 2.0f * valence::kMmPerStep);
}

// A C1 script streamed as Phosphor's funscript player sends it: one bundle per
// span, 250 ms ahead, PCHIP slopes as end velocities, the bundle's first segment
// superseding (RFC-087). The machine renders the author's cubics: no trim, no
// stretch, and the plan within a step of the PCHIP curve after the first span.
TEST_CASE("C1 script, a bundle per span: the author's cubics render with nothing spent") {
    auto r = rig();
    r->arb.forceHome(268.0f);
    r->arb.setWindow(92.0f, 176.0f, 268.0f);
    r->arb.setInputLimits(1000.0f, 50000.0f, 10000000.0f);
    r->run(1000);
    constexpr float kPeriodMs = 1300.0f, kKnotMs = 100.0f, kAmp = 42.0f, kMid = 134.0f;
    constexpr int kKnots = 60;
    float T[kKnots], P[kKnots], M[kKnots];
    for (int i = 0; i < kKnots; ++i) { T[i] = float(i) * kKnotMs; P[i] = kMid + kAmp * std::sin(6.2831853f * T[i] / kPeriodMs); }
    for (int i = 0; i < kKnots; ++i) {
        if (i == 0 || i == kKnots - 1) { M[i] = 0.0f; continue; }
        const float a = (P[i] - P[i - 1]) / kKnotMs, b = (P[i + 1] - P[i]) / kKnotMs;
        M[i] = a * b <= 0.0f ? 0.0f : 2.0f / (1.0f / a + 1.0f / b);   // mm per ms
    }
    auto pchip = [&](float t_ms) {
        int i = int(t_ms / kKnotMs); if (i < 0) i = 0; if (i > kKnots - 2) i = kKnots - 2;
        const float h = kKnotMs, u = (t_ms - T[i]) / h;
        const float h00 = 2*u*u*u - 3*u*u + 1, h10 = u*u*u - 2*u*u + u, h01 = -2*u*u*u + 3*u*u, h11 = u*u*u - u*u;
        return h00 * P[i] + h10 * h * M[i] + h01 * P[i + 1] + h11 * h * M[i + 1];
    };
    // Move to the first knot and rest there.
    REQUIRE(r->submit(MotionSource::Manual, P[0]));
    r->run(1'500'000);
    const uint64_t t0 = g_now_us + 300'000;
    int next = 1; float worst = 0.0f; int worst_at = 0;
    for (int ms = 0; ms < int(T[kKnots - 1]) + 300; ++ms) {
        while (next < kKnots && uint64_t(T[next - 1] * 1000.0f) <= g_now_us + 250'000 - t0) {
            MotionIntent in;
            in.source = MotionSource::Stream;
            in.target_mm = P[next];
            in.duration_us = uint32_t(kKnotMs * 1000.0f);
            in.has_end_vel = true;
            in.end_vel_mm_s = M[next] * 1000.0f;
            in.anchor_us = t0 + uint64_t(T[next - 1] * 1000.0f);
            in.supersede = true;
            REQUIRE(r->arb.accept(in, g_now_us));
            ++next;
        }
        r->run(1000);
        const float t_ms = float(int64_t(g_now_us) - int64_t(t0)) * 1e-3f;
        if (t_ms >= 2.0f * kKnotMs && t_ms <= T[kKnots - 1] - kKnotMs) {
            const float err = std::fabs(r->census().plan_mm - pchip(t_ms));
            if (err > worst) { worst = err; worst_at = int(t_ms); }
        }
    }
    r->arb.drainAnomalies();
    const MotionCensus c = r->census();
    MESSAGE("worst plan error ", worst, " mm at ", worst_at, " ms; trims ", c.anom[size_t(kinetic2::AnomalyKind::KnotTrimmed)],
            " failed ", c.failures, " refused ", c.rejected);
    CHECK(c.rejected == 0);
    CHECK(c.failures == 0);
    // The first span from rest starts at zero acceleration where the author's
    // cubic does not (Kinetic kin-tt8): one trim there, none after it.
    CHECK(c.anom[size_t(kinetic2::AnomalyKind::KnotTrimmed)] <= 1);
    CHECK(worst <= 0.5f);
}

// ---- a bundle start microseconds off the newest knot (Kinetic kin-554) ---------
// Each 0x2101 bundle's start is resolved from the sender's stamp, so a span
// that tiles the previous one in the sender's clock lands a few microseconds
// early or late here: two clock reads in the sender and its integer rounding.

TEST_CASE("a segment stream whose starts miss the newest knot by microseconds renders the author's curve: no hold, no flush (kin-554)") {
    // The operator's ADSR play on the hub (2026-10-08): full
    // strokes 5 to 95 percent of a 100 mm window, every end velocity 0, each
    // span 125 ms ahead, one per bundle. The plan ran past 135 mm to 169 mm
    // and the backstop held the carriage at the window's edge for 350 ms. The
    // through case passes 50 percent moving at the player's knot slope, 300
    // mm/s: a hold a few microseconds after it stopped the plan dead in a tick.
    struct Span { float mm; uint32_t ms; float v; };
    const Span adsr[] = {{135, 300, 0}, {45, 633, 0}, {135, 300, 0}, {45, 634, 0}, {135, 300, 0}, {45, 600, 0},
                         {135, 633, 0}, {45, 600, 0}, {135, 300, 0}, {45, 634, 0}, {135, 300, 0}, {45, 633, 0}};
    const Span through[] = {{90, 150, 300}, {135, 150, 0}, {90, 150, -300}, {45, 150, 0}, {90, 150, 300}, {135, 150, 0},
                            {90, 150, -300}, {45, 150, 0}, {90, 150, 300}, {135, 150, 0}, {90, 150, -300}, {45, 150, 0}};
    const int64_t skews[] = {-3, 3, -30, 30, -3, 900, -900, 3, -3, 0, 7, -7};
    for (const Span* script : {adsr, through}) {
        {
            auto r = rig();
            r->arb.forceHome(268.0f);
            r->arb.setWindow(40.0f, 140.0f, 268.0f);
            r->arb.setInputLimits(1200.0f, 100000.0f, 2e7f);
            r->run(1000);
            REQUIRE(r->submit(MotionSource::Manual, 45.0f));
            r->run(2'000'000);
            r->arb.drainAnomalies();
            const MotionCensus before = r->census();
            const uint64_t t0 = g_now_us + 200'000;
            uint64_t start = t0;
            float lo = 1e9f, hi = -1e9f;
            size_t next = 0;
            const uint64_t end = t0 + 6'500'000;
            while (g_now_us < end) {
                while (next < 12 && start <= g_now_us + 125'000) {
                    MotionIntent in;
                    in.source = MotionSource::Stream;
                    in.target_mm = script[next].mm;
                    in.duration_us = script[next].ms * 1000u;
                    in.has_end_vel = true;
                    in.end_vel_mm_s = script[next].v;
                    in.anchor_us = uint64_t(int64_t(start) + skews[next]);
                    in.supersede = true;
                    REQUIRE(r->arb.accept(in, g_now_us));
                    start += in.duration_us;
                    ++next;
                }
                r->run(1000);
                const float p = r->census().plan_mm;
                lo = std::fmin(lo, p);
                hi = std::fmax(hi, p);
            }
            r->arb.drainAnomalies();
            const MotionCensus c = r->census();
            auto added = [&](kinetic2::AnomalyKind k) { return c.anom[size_t(k)] - before.anom[size_t(k)]; };
            CAPTURE(script == adsr);
            MESSAGE("plan ", lo, "..", hi, " mm; over ceiling ", added(kinetic2::AnomalyKind::PieceOverCeiling), ", refused ",
                    added(kinetic2::AnomalyKind::KnotRefused), ", backstops ", c.backstops - before.backstops);
            CHECK(hi <= 135.0f + 0.1f);
            CHECK(lo >= 45.0f - 0.1f);
            CHECK(c.backstops == before.backstops);
            CHECK(added(kinetic2::AnomalyKind::PieceOverCeiling) == 0);
            CHECK(added(kinetic2::AnomalyKind::KnotRefused) == 0);
            CHECK(c.failures == before.failures);
        }
    }
}

// ---- the planner and the steer (bd val-8rt) ----------------------------------
// The board runs planTick() and steerTick() on two tasks; evaluate() is both on
// one. These cases drive the halves apart, as a long solve or a late planner
// lays them out on the board.

namespace {

// One millisecond: the emitter renders the word it holds, the clock moves.
void elapse(Rig& r) {
    g_now_us += 1000;
    r.emitter.advance(1e-3);
}
void planOnly(Rig& r) { r.arb.planTick(g_now_us, 1e-3f); }
void steerOnly(Rig& r) { r.arb.steerTick(g_now_us, 1e-3f); }

// A homed rig moving a Stream segment toward `target` over `us`, `run_us` in.
std::unique_ptr<Rig> movingRig(float target, uint32_t us, uint64_t run_us) {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->run(1000);
    MotionIntent seg;
    seg.source = MotionSource::Stream;
    seg.target_mm = target;
    seg.duration_us = us;
    REQUIRE(r->arb.accept(seg, g_now_us));
    r->run(run_us);
    return r;
}

// The kick's bound, one tick of the input accel: what a steer may carry with
// no feedforward in it.
constexpr double kKickMax = double(DEFAULT_ACCEL_MM_S2) * 1e-3;

}  // namespace

TEST_CASE("strip: filled once, it steers the next 5 ms as the single-threaded path does") {
    // Through a knot: the segment ends 3 ms after the strip is filled.
    const uint64_t start = g_now_us;
    double one[5] = {}, two[5] = {};
    {
        auto r = movingRig(120.0f, 30'000, 27'000);
        for (double& v : one) {
            r->run(1000);
            v = steeredMmS(r->emitter);
        }
    }
    g_now_us = start;
    {
        auto r = movingRig(120.0f, 30'000, 27'000);
        for (double& v : two) {
            elapse(*r);
            steerOnly(*r);
            v = steeredMmS(r->emitter);
        }
    }
    for (int i = 0; i < 5; ++i) {
        CAPTURE(i);
        CHECK(std::fabs(one[i]) > 1.0);
        CHECK(std::fabs(one[i] - two[i]) <= 1e-4);
    }
}

TEST_CASE("strip: a planner later than the strip holds its last entry, no feedforward, one stall counted") {
    auto r = movingRig(400.0f, 1'000'000, 100'000);
    REQUIRE(std::fabs(steeredMmS(r->emitter)) > 100.0);
    // The strip's whole length with the planner silent: the plan renders.
    for (size_t k = 1; k < valence::kStripLen; ++k) {
        elapse(*r);
        steerOnly(*r);
    }
    CHECK(std::fabs(steeredMmS(r->emitter)) > 100.0);
    CHECK(r->arb.plannerStalls() == 0);
    // Past its end the last entry holds: what is left is the kick back to it.
    for (int k = 0; k < 3; ++k) {
        elapse(*r);
        steerOnly(*r);
        CAPTURE(k);
        CHECK(std::fabs(steeredMmS(r->emitter)) <= kKickMax + 1.0);
    }
    CHECK(r->arb.plannerStalls() == 1);
    CHECK(r->census().stalls == 0);   // the steer itself was never late
    // The planner back: the plan renders again, nothing more counted.
    for (int k = 0; k < 20; ++k) {
        elapse(*r);
        planOnly(*r);
        steerOnly(*r);
    }
    CHECK(std::fabs(steeredMmS(r->emitter)) > 100.0);
    CHECK(r->arb.plannerStalls() == 1);
}

TEST_CASE("strip: a reset re-anchors the next steer, never a burst (frame move, e-stop, power)") {
    SUBCASE("a frame move mid-move: the steer sees the flag before the planner") {
        auto r = movingRig(300.0f, 400'000, 100'000);
        REQUIRE(std::fabs(steeredMmS(r->emitter)) > 100.0);
        r->arb.forceHome(500.0f);
        elapse(*r);
        steerOnly(*r);
        CHECK(r->emitter.q8 == 0);
        planOnly(*r);
        elapse(*r);
        steerOnly(*r);
        CHECK(r->emitter.q8 == 0);
        planOnly(*r);
        elapse(*r);
        steerOnly(*r);
        CHECK(std::fabs(steeredMmS(r->emitter)) <= kKickMax);
        // Held where the reset found the carriage: the tick it coasted before
        // the park, and nothing after.
        const float held = r->arb.positionMm();
        r->run(100'000);
        CHECK(std::fabs(r->arb.positionMm() - held) <= 2.0f * valence::kMmPerStep);
        CHECK(r->census().plan_mm == doctest::Approx(r->census().position_mm).epsilon(0.001));
    }
    SUBCASE("an e-stop and its release, both inside one planner silence") {
        auto r = movingRig(300.0f, 400'000, 100'000);
        REQUIRE(std::fabs(steeredMmS(r->emitter)) > 100.0);
        r->arb.estop(true);
        for (int k = 0; k < 3; ++k) {
            elapse(*r);
            steerOnly(*r);
            CHECK(r->emitter.q8 == 0);
        }
        r->arb.estop(false);
        // The planner never saw it: the strip still carries the plan, which
        // went on while the emitter was parked. That is residual, not
        // feedforward.
        elapse(*r);
        steerOnly(*r);
        CHECK(std::fabs(steeredMmS(r->emitter)) <= kKickMax + 1.0);
    }
    SUBCASE("an e-stop the planner resets for, released into PAUSE") {
        auto r = movingRig(300.0f, 400'000, 100'000);
        r->arb.estop(true);
        elapse(*r);
        planOnly(*r);
        steerOnly(*r);
        r->arb.estop(false);
        elapse(*r);
        planOnly(*r);
        steerOnly(*r);
        CHECK(std::fabs(steeredMmS(r->emitter)) <= kKickMax);
        CHECK(r->census().paused);
    }
    SUBCASE("motor power lost and restored") {
        auto r = movingRig(300.0f, 400'000, 100'000);
        r->arb.setMotorPowered(false);
        elapse(*r);
        steerOnly(*r);
        planOnly(*r);
        elapse(*r);
        planOnly(*r);
        steerOnly(*r);
        r->arb.setMotorPowered(true);
        elapse(*r);
        steerOnly(*r);
        CHECK(r->emitter.q8 == 0);   // the planner's strip is still the empty one
        planOnly(*r);
        elapse(*r);
        steerOnly(*r);
        CHECK(std::fabs(steeredMmS(r->emitter)) <= kKickMax);
    }
}

TEST_CASE("strip: a plan published after the steer passed its reaction horizon lands as a residual, counted") {
    auto r = movingRig(200.0f, 200'000, 0);
    const uint64_t t_a = g_now_us;
    r->run(100'000);
    // A successor frees the first segment's end: past the reaction horizon the
    // curve no longer comes to rest there.
    MotionIntent next;
    next.source = MotionSource::Stream;
    next.target_mm = 300.0f;
    next.duration_us = 200'000;
    next.anchor_us = t_a + 200'000;
    REQUIRE(r->arb.accept(next, g_now_us));
    // A 60 ms solve: the steer renders the strip it holds, the old plan.
    double v_prev = 0.0;
    for (int k = 0; k < 60; ++k) {
        elapse(*r);
        steerOnly(*r);
        v_prev = steeredMmS(r->emitter);
    }
    elapse(*r);
    planOnly(*r);
    steerOnly(*r);
    const double v = steeredMmS(r->emitter);
    MESSAGE("steered ", v_prev, " then ", v, " mm/s at the new plan; late plans ", r->arb.latePlans());
    CHECK(r->arb.latePlans() == 1);
    // One tick of the plan's own accel and the kick: never the gap over 1 ms.
    CHECK(std::fabs(v - v_prev) <= double(DEFAULT_ACCEL_MM_S2) * 1e-3 + kKickMax + 1.0);
    r->run(400'000);
    CHECK(r->census().position_mm == doctest::Approx(300.0f).epsilon(0.001));
}

// Measured on the machine 2026-10-08 (0.1.27, the solve on the tick): a
// funscript play logged STALL lines nearly every second and LEASE LAPSE lines
// through the dense parts; the lease expired behind a 2 to 28 ms solve and the
// emitter stopped mid-stroke. With the solve on the planner the steer renews
// and steers every tick, and a slow planner is never a steer stall.
TEST_CASE("strip: a planTick longer than the lease leaves the steer renewing and steering, nothing counted") {
    auto r = movingRig(400.0f, 600'000, 100'000);
    REQUIRE(std::fabs(steeredMmS(r->emitter)) > 100.0);
    MotionIntent next;
    next.source = MotionSource::Stream;
    next.target_mm = 100.0f;
    next.duration_us = 300'000;
    next.anchor_us = g_now_us + 500'000;
    REQUIRE(r->arb.accept(next, g_now_us));
    // The planner's 3 * kLeaseUs solve: the steer runs every tick meanwhile.
    const MotionCensus before = r->census();
    int parked = 0;
    for (uint32_t k = 0; k < 3 * valence::kLeaseTicks; ++k) {
        elapse(*r);
        steerOnly(*r);
        CHECK(r->emitter.live);
        if (r->emitter.q8 == 0) ++parked;
    }
    elapse(*r);
    planOnly(*r);
    steerOnly(*r);
    const MotionCensus c = r->census();
    CHECK(parked == 0);
    CHECK(r->emitter.lapse_count == 0);
    CHECK(c.lease_lapses == before.lease_lapses);
    CHECK(c.stalls == before.stalls);   // the planner was slow, the steer never late
    CHECK(r->arb.plannerStalls() == 0);   // nor did the strip run dry
    CHECK(std::fabs(steeredMmS(r->emitter)) > 100.0);
}

TEST_CASE("home: the steer never steers while the seek producer owns the emitter, and the cycle completes") {
    auto r = rig();
    FakeSense s{r->emitter, -30.0f, 170.0f};
    isr(*r, s);
    r->arb.setHomeSense(s);
    REQUIRE(r->arb.home() == HomeStart::started);
    int cycle_ticks = 0, steered_in_cycle = 0;
    for (int ms = 0; ms < 20'000; ++ms) {
        elapse(*r);
        planOnly(*r);
        const bool cycle = r->census().homing;
        const int before = r->emitter.steers;
        steerOnly(*r);
        if (cycle) {
            ++cycle_ticks;
            if (r->emitter.steers != before) ++steered_in_cycle;
        }
        if (!cycle && ms > 0) break;
    }
    MESSAGE("cycle ticks ", cycle_ticks);
    CHECK(cycle_ticks > 100);
    CHECK(steered_in_cycle == 0);
    const MotionCensus c = r->census();
    CHECK(c.homed);
    CHECK(c.home_fails == 0);
    CHECK(c.lease_lapses == 0);   // the planner renewed the seek's lease
}

// Measured on the machine 2026-10-08: the steer's cap was the input set's for
// every plan, so a 1500 mm/s jog under a 1000 mm/s input set rendered at
// 1050 mm/s, fell behind its plan, and closed the rest at the kick: 227 mm
// in 1.05 s.
TEST_CASE("jog above the input set: the steer follows the jog set's ceiling, never a crawl at the kick") {
    auto r = rig();
    r->arb.forceHome(500.0f);
    r->arb.setInputLimits(1000.0f, 50000.0f, 5'000'000.0f);
    r->arb.setJogLimits(1500.0f, 20000.0f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 227.0f));
    double peak = 0.0;
    int arrived_ms = 0;
    for (int ms = 1; ms <= 1500 && arrived_ms == 0; ++ms) {
        r->run(1000);
        peak = std::max(peak, std::fabs(steeredMmS(r->emitter)));
        if (std::fabs(r->arb.positionMm() - 227.0f) <= 2.0f * valence::kMmPerStep && !r->census().busy) arrived_ms = ms;
    }
    // The point move's own time: 227 mm at 1500 mm/s is 151 ms of cruise, a
    // point move up to 1.875 times that (RFC-105 (k)), then the settle; the
    // input set's cap took 1,050 ms.
    MESSAGE("peak ", peak, " mm/s, arrived at ", arrived_ms, " ms");
    CHECK(peak >= 1450.0);
    CHECK(peak <= 1500.0 + kKickMax + 1.0);
    CHECK(arrived_ms > 0);
    CHECK(arrived_ms <= 350);
}

// bd val-17u: the travel window stays writable at any time (SPEC 11). Moved
// mid-stream, the pending knots are window shares and stay; the plan runs on
// from its own state in the new frame and re-targets, never a reset.
TEST_CASE("window moved mid-stream by a fifth of the rail: the plan re-targets in place, never stops, never bursts") {
    constexpr float kRail = 400.0f;
    float lo = 0.0f, hi = 320.0f;
    auto r = rig();
    r->arb.setWindow(lo, hi, kRail);
    r->arb.forceHome(kRail);
    r->run(1000);

    // A monotone sweep, share 0.1 to 0.9 in 20 segments of 100 ms, each
    // sent 120 ms before its start with the sweep's velocity as its end
    // velocity (the last one ends at rest). The hub maps shares through the
    // window in force when it sends.
    constexpr int kSegs = 20;
    constexpr int kMoveAt = 10;
    const float rate = 0.8f / (float(kSegs) * 0.1f);   // shares/s
    const uint64_t first = g_now_us + 120'000;
    const uint64_t last_knot = first + uint64_t(kSegs) * 100'000;
    auto segment = [&](int i) {
        MotionIntent in;
        in.source = MotionSource::Stream;
        in.target_mm = lo + (0.1f + rate * 0.1f * float(i + 1)) * (hi - lo);
        in.duration_us = 100'000;
        in.anchor_us = first + uint64_t(i) * 100'000;
        in.has_end_vel = true;
        in.end_vel_mm_s = i + 1 < kSegs ? rate * (hi - lo) : 0.0f;
        return r->arb.accept(in, g_now_us);
    };

    const double ceiling = double(DEFAULT_MAX_SPEED_MM_S) + kKickMax;
    double worst_steer = 0.0, slowest_steer = 1e9, slowest_plan = 1e9;
    float jump = 0.0f;
    MotionCensus prev = r->census();
    auto tick = [&] {
        r->run(1000);
        const MotionCensus c = r->census();
        const double steer = steeredMmS(r->emitter);
        worst_steer = std::max(worst_steer, std::fabs(steer));
        // Moving from the first start until the stream's last segment begins.
        if (g_now_us > first + 50'000 && g_now_us < last_knot - 100'000) {
            slowest_steer = std::min(slowest_steer, steer);
            slowest_plan = std::min(slowest_plan, double(c.velocity_mm_s));
        }
        const float carried = std::max(std::fabs(c.velocity_mm_s), std::fabs(prev.velocity_mm_s)) * 1e-3f;
        jump = std::max(jump, std::fabs(c.plan_mm - prev.plan_mm) - carried);
        prev = c;
    };

    for (int i = 0; i < kSegs; ++i) {
        if (i == kMoveAt) {
            const float was = r->arb.positionMm();
            lo += 0.2f * kRail;
            hi += 0.2f * kRail;
            r->arb.setWindow(lo, hi, kRail);
            MESSAGE("window moved at ", was, " mm, ", r->arb.engine().pending(), " knots pending");
            REQUIRE(r->arb.engine().pending() > 0);
        }
        REQUIRE(segment(i));
        for (int t = 0; t < 100; ++t) tick();
    }
    for (int t = 0; t < 500; ++t) tick();

    const MotionCensus c = r->census();
    const float end_mm = lo + 0.9f * (hi - lo);
    MESSAGE("steer peak ", worst_steer, " mm/s, slowest steer ", slowest_steer, " plan ", slowest_plan,
            " mm/s, plan jump beyond the velocity ", jump, " mm, end ", c.position_mm, " mm");
    CHECK(slowest_plan > 1.0);
    CHECK(slowest_steer > 1.0);
    CHECK(worst_steer <= ceiling);
    CHECK(jump <= 0.01f);
    CHECK(r->arb.plannerStalls() == 0);
    CHECK(c.emitter_faults == 0);
    CHECK(c.rejected == 0);
    CHECK(c.failures == 0);
    CHECK_FALSE(c.busy);
    CHECK(c.position_mm >= lo);
    CHECK(c.position_mm <= hi);
    CHECK(c.position_mm == doctest::Approx(end_mm).epsilon(1e-3));
}

TEST_CASE("window moved mid-stream with the steer ahead of the planner: parked while the flag is up, then steered from the plan, never a burst") {
    auto r = movingRig(400.0f, 1'000'000, 100'000);
    REQUIRE(std::fabs(steeredMmS(r->emitter)) > 100.0);
    r->arb.setWindow(100.0f, 500.0f, 500.0f);
    elapse(*r);
    steerOnly(*r);
    CHECK(r->emitter.q8 == 0);
    planOnly(*r);
    CHECK(r->arb.engine().pending() > 0);   // re-targeted, not reset
    double worst = 0.0, last = 0.0;
    for (int k = 0; k < 20; ++k) {
        elapse(*r);
        steerOnly(*r);
        planOnly(*r);
        last = steeredMmS(r->emitter);
        worst = std::max(worst, std::fabs(last));
    }
    CHECK(worst <= double(DEFAULT_MAX_SPEED_MM_S) + kKickMax);
    CHECK(last > 100.0);
    CHECK(r->arb.plannerStalls() == 0);
}

// bd val-8es: setWindow() runs on the hub task and the strip fills on the
// planner. The val-17u sweep with the window dragged 80 mm in 2 mm writes,
// each landed at a different point of the split ticks: before the planner,
// inside its solve after the frame-move block, at the strip's publish, and
// between the planner and the steer. Every published strip lies in one frame:
// no step inside it beyond the speed ceiling, and it continues the strip
// before it in mm across every re-target.
TEST_CASE("window dragged mid-stream, writes landed inside planner ticks: every published strip is in one frame") {
    constexpr float kRail = 400.0f;
    float lo = 0.0f, hi = 320.0f;   // the hub's window, which it maps shares through
    auto r = rig();
    r->arb.setWindow(lo, hi, kRail);
    r->arb.forceHome(kRail);
    r->run(1000);
    struct Disarm {
        ~Disarm() { g_post = WindowPost{}; }
    } disarm;

    constexpr int kSegs = 20;
    const float rate = 0.8f / (float(kSegs) * 0.1f);   // shares/s
    const uint64_t first = g_now_us + 120'000;
    const uint64_t last_knot = first + uint64_t(kSegs) * 100'000;
    auto segment = [&](int i) {
        MotionIntent in;
        in.source = MotionSource::Stream;
        in.target_mm = lo + (0.1f + rate * 0.1f * float(i + 1)) * (hi - lo);
        in.duration_us = 100'000;
        in.anchor_us = first + uint64_t(i) * 100'000;
        in.has_end_vel = true;
        in.end_vel_mm_s = i + 1 < kSegs ? rate * (hi - lo) : 0.0f;
        return r->arb.accept(in, g_now_us);
    };
    auto shift = [&] {
        lo += 2.0f;
        hi += 2.0f;
    };

    // From the sweep's middle, one move every 10 ms: a write at the top of the
    // tick, then a second one inside the solve, at the publish, or after the
    // planner, in turn.
    constexpr int kMoves = 20;
    const uint64_t drag_from = first + 1'000'000;
    int moves = 0, sent = 0;
    std::array<int, 2> inside{};
    const float step_max = float(DEFAULT_MAX_SPEED_MM_S) * 1e-3f;
    float inner = 0.0f, handoff = 0.0f;
    int strips = 0;
    double worst_steer = 0.0;
    auto prev = r->arb.publishedStrip();
    while (g_now_us < last_knot + 500'000) {
        while (sent < kSegs && g_now_us + 120'000 >= first + uint64_t(sent) * 100'000) REQUIRE(segment(sent++));
        elapse(*r);
        int site = -1;
        if (moves < kMoves && g_now_us >= drag_from && (g_now_us - drag_from) % 10'000 == 0) {
            site = moves++ % 3;
            shift();
            r->arb.setWindow(lo, hi, kRail);
        }
        if (site == 0 || site == 1) {
            shift();
            g_post = WindowPost{&r->arb, site == 0 ? WindowPost::At::clock : WindowPost::At::publish, lo, hi, kRail};
        }
        planOnly(*r);
        if (site == 0 || site == 1) {
            if (g_post.fired) ++inside[size_t(site)];
            else r->arb.setWindow(lo, hi, kRail);
            g_post = WindowPost{};
        }

        const auto s = r->arb.publishedStrip();
        if (s.n > 0) {
            ++strips;
            for (size_t i = 1; i < s.n; ++i) inner = std::max(inner, std::fabs(s.p_mm[i] - s.p_mm[i - 1]));
            if (prev.n > 0 && (s.gen == prev.gen || s.continuous) && s.t0_us >= prev.t0_us) {
                const uint64_t k = (s.t0_us - prev.t0_us) / valence::kMotionTickUs;
                if (k < prev.n) handoff = std::max(handoff, std::fabs(s.p_mm[0] - prev.p_mm[k]));
            }
        }
        prev = s;

        if (site == 2) {
            shift();
            r->arb.setWindow(lo, hi, kRail);
        }
        steerOnly(*r);
        worst_steer = std::max(worst_steer, std::fabs(steeredMmS(r->emitter)));
    }

    const MotionCensus c = r->census();
    MESSAGE(strips, " strips, step inside one ", inner, " mm (ceiling ", step_max, "), hand-off ", handoff,
            " mm, writes inside the solve ", inside[0], ", at the publish ", inside[1], ", steer peak ", worst_steer,
            " mm/s, end ", c.position_mm, " mm");
    REQUIRE(moves == kMoves);
    CHECK(lo == doctest::Approx(80.0f));
    CHECK(inside[0] > 0);
    CHECK(inside[1] > 0);
    CHECK(inner <= step_max);
    CHECK(handoff <= 0.01f);
    CHECK(worst_steer <= double(DEFAULT_MAX_SPEED_MM_S) + kKickMax);
    CHECK(r->arb.plannerStalls() == 0);
    CHECK(c.emitter_faults == 0);
    CHECK(c.rejected == 0);
    CHECK(c.failures == 0);
    CHECK_FALSE(c.busy);
    CHECK(c.position_mm == doctest::Approx(lo + 0.9f * (hi - lo)).epsilon(1e-3));
}

// bd val-4dt, val-83q: the val-17u sweep with the window dragged 80 mm at
// 400 mm/s, a 2 mm write and a second one a tick later every 10 ms, at every
// phase of the drag against the knots. Re-planned from the write, one landing
// inside a knot's horizon trimmed that knot to rest on the write's state: the
// plan ran out and back (-336 mm/s) or held a tick at the knot (128 -> 0 ->
// 90 mm/s). Never backward past kin-554's tolerance, never a held tick, never
// past a ceiling.
TEST_CASE("window dragged mid-stream at every phase against the knots: never out and back, never a held tick") {
    constexpr float kRail = 400.0f;
    constexpr int kSegs = 20;
    for (uint64_t off = 0; off < 10'000; off += 1'000) {
        float lo = 0.0f, hi = 320.0f;
        auto r = rig();
        r->arb.setWindow(lo, hi, kRail);
        r->arb.forceHome(kRail);
        r->run(1000);
        const float rate = 0.8f / (float(kSegs) * 0.1f);   // shares/s
        const uint64_t first = g_now_us + 120'000;
        const uint64_t last_knot = first + uint64_t(kSegs) * 100'000;
        auto segment = [&](int i) {
            MotionIntent in;
            in.source = MotionSource::Stream;
            in.target_mm = lo + (0.1f + rate * 0.1f * float(i + 1)) * (hi - lo);
            in.duration_us = 100'000;
            in.anchor_us = first + uint64_t(i) * 100'000;
            in.has_end_vel = true;
            in.end_vel_mm_s = i + 1 < kSegs ? rate * (hi - lo) : 0.0f;
            return r->arb.accept(in, g_now_us);
        };
        auto shift = [&] {
            lo += 2.0f;
            hi += 2.0f;
            r->arb.setWindow(lo, hi, kRail);
        };
        const uint64_t drag_from = first + 1'000'000 + off;
        int sent = 0, writes = 0, held = 0;
        float top = -1e9f, back = 0.0f, fastest = 0.0f;
        double worst_steer = 0.0;
        MotionCensus prev = r->census(), prev2 = prev;
        while (g_now_us < last_knot + 500'000) {
            while (sent < kSegs && g_now_us + 120'000 >= first + uint64_t(sent) * 100'000) REQUIRE(segment(sent++));
            elapse(*r);
            if (writes < 40 && g_now_us >= drag_from && (g_now_us - drag_from) % 10'000 <= 1'000) {
                shift();
                ++writes;
            }
            planOnly(*r);
            steerOnly(*r);
            const MotionCensus c = r->census();
            if (g_now_us >= drag_from) {
                top = std::max(top, c.plan_mm);
                back = std::max(back, top - c.plan_mm);
            }
            fastest = std::max(fastest, std::fabs(c.velocity_mm_s));
            worst_steer = std::max(worst_steer, std::fabs(steeredMmS(r->emitter)));
            if (g_now_us < last_knot - 100'000 && std::fabs(prev.velocity_mm_s) < 1.0f &&
                std::fabs(prev2.velocity_mm_s) > 10.0f && std::fabs(c.velocity_mm_s) > 10.0f)
                ++held;
            prev2 = prev;
            prev = c;
        }
        const MotionCensus c = r->census();
        CAPTURE(off);
        MESSAGE("drag at +", off / 1000, " ms: back ", back, " mm, held ", held, ", plan peak ", fastest,
                " mm/s, steer peak ", worst_steer, " mm/s");
        REQUIRE(writes == 40);
        CHECK(back <= 2e-3f * (hi - lo));   // kin-554's tolerance
        CHECK(held == 0);
        CHECK(fastest <= DEFAULT_MAX_SPEED_MM_S);
        CHECK(worst_steer <= double(DEFAULT_MAX_SPEED_MM_S) + kKickMax);
        CHECK(r->arb.plannerStalls() == 0);
        CHECK(c.emitter_faults == 0);
        CHECK(c.rejected == 0);
        CHECK(c.failures == 0);
        CHECK_FALSE(c.busy);
        CHECK(c.position_mm == doctest::Approx(lo + 0.9f * (hi - lo)).epsilon(1e-3));
    }
}

// ---- the stream's expectation (Kinetic Engine::expect, bd val-g62) -----------
// A Stream segment carries the hub's quiet window (MotionIntent::expect_us), so
// its free knot renders through toward a provisional successor; the hub's
// quiet release, a brake, an e-stop and any other intent end it.

namespace {

constexpr float    kKnotTol = 0.05f;      // mm, about ten steps
constexpr uint32_t kQuietUs = 500'000;    // registry stream_quiet_release_ms
constexpr uint32_t kSpanUs  = 100'000;
constexpr float    kStepMm  = 10.0f;      // per span
constexpr float    kChord   = kStepMm / (float(kSpanUs) * 1e-6f);   // 100 mm/s

// A homed 268 mm rail, the window 40..240, resting at 50 mm.
std::unique_ptr<Rig> streamRig() {
    auto r = rig();
    r->arb.forceHome(268.0f);
    r->arb.setWindow(40.0f, 240.0f, 268.0f);
    r->arb.setInputLimits(1000.0f, 20000.0f, 2e7f);
    r->run(1000);
    REQUIRE(r->submit(MotionSource::Manual, 50.0f));
    r->run(1'500'000);
    return r;
}

// The plan's velocity through the stream's middle: from its third knot to
// its second-to-last, past the start from rest and before the end.
struct Through {
    float at_knot = 1e9f;   // the slowest at a knot's time
    float least   = 1e9f;   // the slowest anywhere
    float most    = 0.0f;
};

// `knots` free same-direction segments from 50 mm, one per bundle, each sent
// 125 ms before its start, with expect_us as given; then `tail_us` more.
// at_last() runs right after the last submit.
Through streamUp(Rig& r, int knots, uint32_t expect_us, uint64_t tail_us,
                 const std::function<void()>& at_last = nullptr) {
    Through th;
    const uint64_t t0 = g_now_us + 200'000;
    const uint64_t from = t0 + 3 * kSpanUs, to = t0 + uint64_t(knots - 1) * kSpanUs;
    auto tick = [&] {
        r.run(1000);
        if (g_now_us < from || g_now_us > to) return;
        const float v = r.census().velocity_mm_s;
        if ((g_now_us - t0) % kSpanUs == 0) th.at_knot = std::fmin(th.at_knot, v);
        th.least = std::fmin(th.least, v);
        th.most = std::fmax(th.most, v);
    };
    int next = 0;
    while (next < knots) {
        const uint64_t start = t0 + uint64_t(next) * kSpanUs;
        if (start > g_now_us + 125'000) {
            tick();
            continue;
        }
        MotionIntent in;
        in.source = MotionSource::Stream;
        in.target_mm = 50.0f + kStepMm * float(next + 1);
        in.duration_us = kSpanUs;
        in.anchor_us = start;
        in.supersede = true;
        in.expect_us = expect_us;
        REQUIRE(r.arb.accept(in, g_now_us));
        if (++next == knots && at_last) at_last();
    }
    for (uint64_t t = 0; t < tail_us; t += 1000) tick();
    return th;
}

}  // namespace

TEST_CASE("expect: free same-direction knots streamed 125 ms ahead pass at chord speed; without it the stream surges and sags") {
    constexpr int kKnots = 12;
    Through with, without;
    {
        auto r = streamRig();
        with = streamUp(*r, kKnots, kQuietUs, 1'000'000);
        r->arb.drainAnomalies();
        const MotionCensus c = r->census();
        CHECK(c.rejected == 0);
        CHECK(c.anom[size_t(kinetic2::AnomalyKind::PieceOverCeiling)] == 0);
    }
    {
        auto r = streamRig();
        without = streamUp(*r, kKnots, 0, 1'000'000);
    }
    MESSAGE("with expect: ", with.at_knot / kChord, " of the chord at the slowest knot, ", with.least / kChord, "..",
            with.most / kChord, " between; without: ", without.at_knot / kChord, ", ", without.least / kChord, "..",
            without.most / kChord);
    CHECK(with.at_knot >= 0.95f * kChord);
    CHECK(with.least >= 0.95f * kChord);
    CHECK(with.most <= 1.05f * kChord);
    CHECK(without.least < 0.95f * kChord);
}

TEST_CASE("expect: the quiet release with the last knot pending re-solves it to land at rest, never braked past") {
    constexpr int kKnots = 8;
    const float last = 50.0f + kStepMm * float(kKnots);
    auto r = streamRig();
    // A session dropping mid-stream: the hub releases the stream while its
    // last segment is still ahead.
    streamUp(*r, kKnots, kQuietUs, 1'500'000, [&] {
        CHECK(r->arb.expectUntil() == g_now_us + kQuietUs);
        r->arb.releaseRail(MotionSource::Stream);
    });
    CHECK(r->arb.expectUntil() == 0);
    const MotionCensus c = r->census();
    CHECK_FALSE(c.busy);
    CHECK(std::fabs(c.plan_mm - last) <= kKnotTol);
    CHECK(std::fabs(c.position_mm - last) <= kKnotTol);

    // The same stream simply ending: its last knot is passed moving and the
    // engine's brake stops past it; the release comes after it played out.
    auto s = streamRig();
    streamUp(*s, kKnots, kQuietUs, 1'500'000);
    MESSAGE("a stream that ends before its release rests ", s->census().plan_mm - last, " mm past its last knot");
    s->arb.releaseRail(MotionSource::Stream);
    s->run(1000);
    CHECK(s->arb.expectUntil() == 0);
}

TEST_CASE("expect: PAUSE, ESTOP, a jog and a sample end it; a Stream segment without expect_us sets none") {
    auto streaming = [](Rig& r) {
        MotionIntent in;
        in.source = MotionSource::Stream;
        in.target_mm = 120.0f;
        in.duration_us = 400'000;
        in.expect_us = kQuietUs;
        REQUIRE(r.arb.accept(in, g_now_us));
        r.run(50'000);
        REQUIRE(r.arb.expectUntil() != 0);
    };
    {
        auto r = streamRig();
        streaming(*r);
        r->arb.pause(true);
        r->run(1000);
        CHECK(r->arb.expectUntil() == 0);
    }
    {
        auto r = streamRig();
        streaming(*r);
        r->arb.estop(true);
        r->run(1000);
        CHECK(r->arb.expectUntil() == 0);
    }
    {
        auto r = streamRig();
        streaming(*r);
        REQUIRE(r->submit(MotionSource::Manual, 60.0f));
        CHECK(r->arb.expectUntil() == 0);
    }
    {
        auto r = streamRig();
        streaming(*r);
        MotionIntent in;   // a 0x2100 sample, ahead of the segment's knot
        in.source = MotionSource::Stream;
        in.target_mm = 130.0f;
        in.anchor_us = g_now_us + 400'000;
        REQUIRE(r->arb.accept(in, g_now_us));
        CHECK(r->arb.expectUntil() == 0);
    }
    {
        auto r = streamRig();
        MotionIntent in;
        in.source = MotionSource::Stream;
        in.target_mm = 120.0f;
        in.duration_us = 400'000;
        REQUIRE(r->arb.accept(in, g_now_us));
        CHECK(r->arb.expectUntil() == 0);
    }
}
