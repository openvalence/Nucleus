// test_pattern_engine -- native doctest suite for the shared pattern generator
// Constraints:
// - Hardware-free and deterministic: a synthetic microsecond clock and a
//   synthetic motion plane. No IDF, no FreeRTOS, no arbiter.
// - Compiles flagship_p4/src/patterns/{PatternEngine,AdvancedPattern}.cpp
//   themselves, the one copy the board and the sim both link.
// See: flagship_p4/src/patterns/PatternEngine.h, bd val-091.12

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "../../../flagship_p4/src/patterns/AdvancedPattern.cpp"
#include "../../../flagship_p4/src/patterns/PatternEngine.cpp"
#include "../../../flagship_p4/src/patterns/PatternPresetStore.h"

using valence::MotionIntent;
using valence::MotionSource;
using valence::PatternEngine;
using valence::PatternInputs;
using valence::PatternPresetStore;
using valence::PatternSettings;

namespace {

constexpr float kWinMin = 20.0f;
constexpr float kWinMax = 220.0f;

PatternSettings baseSettings() {
    PatternSettings s;
    s.frame = {kWinMin, kWinMax, 950.0f, 50000.0f};
    s.running = true;
    s.speed = 40.0f;
    s.depth = 80.0f;
    s.stroke = 60.0f;
    s.sensation = 50.0f;
    return s;
}

PatternSettings advancedSettings() {
    PatternSettings s = baseSettings();
    s.ap_mode = true;
    s.ap.master.set(60);
    s.ap.setBase(advpat::DEPTH_MAX, 90);
    s.ap.setBase(advpat::DEPTH_MIN, 10);
    // A lane set: depth breathes in over 3 strokes, speed-in over 5.
    s.ap.byId(advpat::DEPTH_MAX)->modifier.set(40, 3, 1, 3, 0, 0);
    s.ap.byId(advpat::SPEED_IN)->modifier.set(50, 5, 0, 5, 2, 20);
    return s;
}

PatternInputs homedIdle() {
    PatternInputs in;
    in.homed = true;
    in.position_mm = kWinMin;
    return in;
}

struct Emitted {
    uint64_t at_us;
    MotionIntent it;
};

// Ticks every millisecond for `ms` and collects every intent, with the plane
// as `in` throughout (the carriage position is irrelevant to scheduling once
// the first stroke has a predecessor target).
std::vector<Emitted> run(PatternEngine& e, uint64_t& now_us, uint32_t ms, const PatternInputs& in) {
    std::vector<Emitted> out;
    for (uint32_t k = 0; k < ms; ++k) {
        if (const auto it = e.tick(now_us, in)) out.push_back({now_us, *it});
        now_us += 1000;
    }
    return out;
}

bool sameIntent(const MotionIntent& a, const MotionIntent& b) {
    return a.source == b.source && a.target_mm == b.target_mm && a.duration_us == b.duration_us &&
           a.has_end_vel == b.has_end_vel && a.end_vel_mm_s == b.end_vel_mm_s;
}

}  // namespace

TEST_CASE("classic pattern: deterministic for a fixed knob set and clock") {
    for (uint8_t pat = 0; pat < PatternSettings::kPatternCount; ++pat) {
        CAPTURE(int(pat));
        auto a = std::make_unique<PatternEngine>();
        auto b = std::make_unique<PatternEngine>();
        PatternSettings s = baseSettings();
        s.pattern = pat;
        s.sensation = 80.0f;
        a->apply(s);
        b->apply(s);
        uint64_t ta = 20'000'000, tb = 20'000'000;
        const auto ra = run(*a, ta, 20000, homedIdle());
        const auto rb = run(*b, tb, 20000, homedIdle());
        REQUIRE(ra.size() == rb.size());
        CHECK(ra.size() >= 4);
        for (size_t i = 0; i < ra.size(); ++i) {
            CHECK(ra[i].at_us == rb[i].at_us);
            CHECK(sameIntent(ra[i].it, rb[i].it));
        }
    }
}

TEST_CASE("classic stroke: waveform half-strokes inside the window, reversing at rest") {
    PatternEngine e;
    e.apply(baseSettings());
    uint64_t now = 5'000'000;
    const auto r = run(e, now, 10000, homedIdle());
    REQUIRE(r.size() >= 4);
    // SimpleStroke at depth 80 %, stroke 60 %: in to 80 % of the window, out
    // to 20 %.
    const float span = kWinMax - kWinMin;
    CHECK(r[0].it.target_mm == doctest::Approx(kWinMin + 0.8f * span).epsilon(0.001));
    CHECK(r[1].it.target_mm == doctest::Approx(kWinMin + 0.2f * span).epsilon(0.001));
    for (const auto& x : r) {
        CHECK(x.it.source == MotionSource::Pattern);
        CHECK(x.it.target_mm >= kWinMin);
        CHECK(x.it.target_mm <= kWinMax);
        CHECK(x.it.duration_us > 0);
        CHECK(x.it.has_end_vel);
        CHECK(x.it.end_vel_mm_s == 0.0f);
    }
    // One half-stroke per its own duration: the next is due exactly when the
    // previous lands, to the tick.
    for (size_t i = 1; i < r.size(); ++i)
        CHECK(r[i].at_us - r[i - 1].at_us <= uint64_t(r[i - 1].it.duration_us) + 1000u);
}

TEST_CASE("advanced generator: a lane set is deterministic and stays in the window") {
    auto a = std::make_unique<PatternEngine>();
    auto b = std::make_unique<PatternEngine>();
    a->apply(advancedSettings());
    b->apply(advancedSettings());
    uint64_t ta = 3'000'000, tb = 3'000'000;
    const auto ra = run(*a, ta, 30000, homedIdle());
    const auto rb = run(*b, tb, 30000, homedIdle());
    REQUIRE(ra.size() == rb.size());
    REQUIRE(ra.size() >= 12);
    float deepest = kWinMin, shallowest_in = kWinMax;
    for (size_t i = 0; i < ra.size(); ++i) {
        CHECK(sameIntent(ra[i].it, rb[i].it));
        CHECK(ra[i].it.target_mm >= kWinMin);
        CHECK(ra[i].it.target_mm <= kWinMax);
        if (i % 2 == 0) {
            if (ra[i].it.target_mm > deepest) deepest = ra[i].it.target_mm;
            if (ra[i].it.target_mm < shallowest_in) shallowest_in = ra[i].it.target_mm;
        }
    }
    // The depth lane really modulates: in-strokes do not all land at 90 %.
    CHECK(deepest - shallowest_in > 10.0f);
}

TEST_CASE("gates: no stroke while unhomed, e-stopped, paused, stopped or yielding") {
    const PatternInputs homed = homedIdle();
    auto gated = [&](PatternInputs in, PatternSettings s) {
        PatternEngine e;
        e.apply(s);
        uint64_t now = 1'000'000;
        return run(e, now, 3000, in).size();
    };
    PatternInputs in = homed;
    in.homed = false;
    CHECK(gated(in, baseSettings()) == 0);
    in = homed;
    in.estop = true;
    CHECK(gated(in, baseSettings()) == 0);
    in = homed;
    in.paused = true;
    CHECK(gated(in, baseSettings()) == 0);
    in = homed;
    in.stream_active = true;
    CHECK(gated(in, baseSettings()) == 0);
    PatternSettings off = baseSettings();
    off.running = false;
    CHECK(gated(homed, off) == 0);
    // Speed 0 holds: the factory default must never move the machine.
    PatternSettings still = baseSettings();
    still.speed = 0.0f;
    CHECK(gated(homed, still) == 0);
    CHECK(gated(homed, PatternSettings{}) == 0);
}

TEST_CASE("stopping mid-stroke brakes once, at the braking point") {
    PatternEngine e;
    e.apply(baseSettings());
    uint64_t now = 1'000'000;
    PatternInputs in = homedIdle();
    REQUIRE(e.tick(now, in).has_value());   // first half-stroke out
    now += 10'000;
    in.position_mm = 100.0f;
    in.velocity_mm_s = 300.0f;
    PatternSettings stop = baseSettings();
    stop.running = false;
    e.apply(stop);
    const auto brake = e.tick(now, in);
    REQUIRE(brake.has_value());
    CHECK(brake->source == MotionSource::Pattern);
    CHECK(brake->duration_us == 0);
    // v^2 / 2a = 300^2 / 100000 = 0.9 mm past the carriage, in its direction.
    CHECK(brake->target_mm == doctest::Approx(100.9f));
    CHECK_FALSE(e.tick(now + 1000, in).has_value());
    CHECK_FALSE(e.active());
}

TEST_CASE("a stream taking over does not get a brake from the generator") {
    PatternEngine e;
    e.apply(baseSettings());
    uint64_t now = 1'000'000;
    PatternInputs in = homedIdle();
    REQUIRE(e.tick(now, in).has_value());
    in.stream_active = true;
    CHECK_FALSE(e.tick(now + 1000, in).has_value());
    CHECK_FALSE(e.active());
}

TEST_CASE("background_run decides what a source release does") {
    PatternSettings s = baseSettings();
    s.background_run = false;
    CHECK(s.ownerReleased());
    CHECK_FALSE(s.running);

    s = baseSettings();
    s.background_run = true;
    CHECK_FALSE(s.ownerReleased());
    CHECK(s.running);

    s.running = false;
    CHECK_FALSE(s.ownerReleased());
}

TEST_CASE("advanced knobs clamp and the depth pair never crosses") {
    advpat::Settings ap;
    ap.setBase(advpat::DEPTH_MAX, 80);
    ap.setBase(advpat::DEPTH_MIN, 10);
    CHECK(ap.max_depth.value == 80);
    CHECK(ap.min_depth.value == 10);
    ap.setBase(advpat::DEPTH_MIN, 95);   // above max: clamps to it
    CHECK(ap.min_depth.value == 80);
    ap.setBase(advpat::DEPTH_MAX, 5);    // below min: clamps to it
    CHECK(ap.max_depth.value == 80);
    advpat::Modifier m;
    m.set(-4, 999, 30, 0, 26, 250);
    CHECK(m.amplitude == 0);
    CHECK(m.in_step == 25);
    CHECK(m.in_wait == 25);
    CHECK(m.out_step == 1);
    CHECK(m.out_wait == 25);
    CHECK(m.offset == 100);
}

TEST_CASE("preset payload round-trips and the store bumps its generation") {
    PatternSettings a = advancedSettings();
    a.ap.setBase(advpat::SPEED_OUT, 33);
    const auto payload = a.capturePreset();
    REQUIRE(payload.size() == PatternPresetStore::kPayloadBytes);

    PatternSettings b;
    b.applyPreset(payload);
    CHECK(b.ap_mode);
    CHECK(b.ap.out_speed.value == 33);
    CHECK(b.ap.byId(advpat::SPEED_IN)->modifier == a.ap.byId(advpat::SPEED_IN)->modifier);
    // Depths and master speed are never carried by a preset.
    CHECK(b.ap.master.value == 0);
    CHECK(b.ap.max_depth.value == 10);

    PatternPresetStore st;
    const uint16_t g0 = st.generation();
    CHECK(st.save(3, "slow tease", payload));
    CHECK(st.count() == 1);
    CHECK(st.slot(3)->nameView() == "slow tease");
    CHECK(st.rename(3, "renamed"));
    CHECK_FALSE(st.rename(4, "empty slot"));
    CHECK_FALSE(st.save(24, "out of range", payload));
    CHECK_FALSE(st.save(0, "", payload));
    CHECK_FALSE(st.save(0, "a name that is far too long for 32 bytes", payload));
    CHECK(st.remove(3));
    CHECK(st.slot(3) == nullptr);
    CHECK(st.generation() == uint16_t(g0 + 3));
}
