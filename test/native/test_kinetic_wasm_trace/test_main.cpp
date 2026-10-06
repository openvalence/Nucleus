// test_kinetic_wasm_trace -- the native half of the kinetic.wasm determinism
// proof: a fixed 60 s segment script through the offline planner's C ABI,
// written to test/fixtures/kinetic_trace.json for tools/kinetic-wasm/check.mjs
// Constraints:
// - Compiles tools/kinetic-wasm/kinetic_wasm.cpp itself, the same ABI source
//   em++ builds, so the two runs differ only in compiler and libm.
// - ONE FIXTURE PER KERNEL: kinetic_trace.json from the native environment,
//   kinetic2_trace.json from native_kinetic2 (NUCLEUS_KINETIC2), each replayed
//   by check.mjs against the wasm built with the same switch.
// - THE FIXTURE IS REGENERATED ON EVERY RUN and is deterministic: a diff in
//   git means the planner's output moved, which is the planner change's to
//   explain in its own commit. The SCRIPT lives in the fixture, so check.mjs
//   replays it and never restates it.
// - Every 1 ms sample is covered by the per-block FNV-1a hashes (all 64 bytes
//   of kinetic_sample); p/v/a are also listed every kTraceEveryMs so a
//   mismatch can be measured in ULPs.
// See: tools/kinetic-wasm/README.md, bd val-eff

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

// Named here so the dependency finder builds them; the .cpp files below need all three.
#include "geiger/geiger.h"
#include "kinetic/kinetic.hpp"
#include "valence/generated/registry_constants.hpp"

#include "../../../flagship_p4/src/motion/MotionArbiter.cpp"
#include "../../../tools/kinetic-wasm/kinetic_wasm.cpp"

namespace {

// ---- the script -------------------------------------------------------------

constexpr float    kVmax = 1000.0f;      // valence_config.h factory input set
constexpr float    kAmax = 50000.0f;
constexpr float    kJmax = 2000000.0f;
constexpr float    kRail = 500.0f;
constexpr float    kWinLo = 100.0f;
constexpr float    kWinHi = 400.0f;
constexpr uint32_t kHorizonMs = 250;
constexpr uint32_t kSteps = 60000;       // 60 s at 1 ms
constexpr uint32_t kLeadMs = 120;        // the player stamps each start this far ahead
constexpr uint32_t kBlock = 100;         // samples per hash
constexpr uint32_t kTraceEveryMs = 50;
constexpr int16_t  kUnspec = int16_t(valence::limits::segment_end_vel_unspecified);

struct Event {
    uint32_t tick;      // submitted at this many ms, before the step that follows it
    bool     tune;      // true: set the infeasible policy to `pos`
    uint16_t pos;
    uint16_t dur;
    int16_t  endv;
    uint32_t start_ms;
    uint8_t  fam;
};

uint16_t posAt(double x) { return uint16_t(std::lround(x < 0.0 ? 0.0 : x > 10000.0 ? 10000.0 : x)); }

void segment(std::vector<Event>& ev, uint32_t start_ms, uint16_t pos, uint16_t dur, int16_t endv,
             uint8_t fam) {
    ev.push_back({start_ms - kLeadMs, false, pos, dur, endv, start_ms, fam});
}

// Moderate swings at 250 ms (C1 cubic declared), one re-steer mid-segment, a
// 1.5 s gap after a segment that ends moving (the settle brake), then
// quintic swings with full-window strokes the ceilings cannot meet (Blend),
// then the same under Stretch (the Ruckig guard).
std::vector<Event> script() {
    std::vector<Event> ev;
    for (uint32_t i = 0; i < 80; ++i) {
        const uint32_t s = 500 + 250 * i;
        const auto at = [](uint32_t k) { return 5000.0 + 3000.0 * std::sin(0.55 * k) + 800.0 * std::sin(1.7 * k); };
        int16_t endv = kUnspec;
        if (i % 3 == 1) endv = int16_t(std::lround((at(i + 1) - at(i - 1)) / 5.0));
        if (i == 79) endv = 400;   // ends moving into the gap
        segment(ev, s, posAt(at(i)), 250, endv, 1);
        if (i == 40) segment(ev, s + 125, 9000, 250, kUnspec, 1);   // the re-steer
    }
    for (uint32_t i = 0; i < 112; ++i) {
        const uint32_t s = 22000 + 250 * i;
        double x = 5000.0 + 3500.0 * std::sin(0.9 * i);
        if (i % 8 == 6) x = 0.0;
        if (i % 8 == 7) x = 10000.0;
        segment(ev, s, posAt(x), 250, i % 4 == 0 ? int16_t(0) : kUnspec, 0);
    }
    ev.push_back({50000, true, 0, 0, 0, 0, 0});
    for (uint32_t i = 0; i < 38; ++i) {
        const uint32_t s = 50200 + 250 * i;
        segment(ev, s, posAt(i % 2 == 0 ? 500.0 : 9500.0), 250, kUnspec, 1);
    }
    return ev;
}

// ---- the record -------------------------------------------------------------

uint64_t fnv1a(uint64_t h, const unsigned char* p, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        h ^= p[i];
        h *= 0x100000001b3ull;
    }
    return h;
}

std::string num(double x) {
    char b[40];
    std::snprintf(b, sizeof(b), "%.17g", x);
    return b;
}

std::string hex64(uint64_t x) {
    char b[20];
    std::snprintf(b, sizeof(b), "%016llx", static_cast<unsigned long long>(x));
    return b;
}

}  // namespace

TEST_CASE("kinetic.wasm trace: the 60 s script, recorded for the wasm twin") {
    kinetic_handle* h = kinetic_create(kVmax, kAmax, kJmax, kRail, kHorizonMs);
    REQUIRE(h != nullptr);
    REQUIRE(kinetic_set_window(h, kWinLo, kWinHi) == 1);
    const std::vector<Event> ev = script();
    REQUIRE(std::is_sorted(ev.begin(), ev.end(),
                           [](const Event& a, const Event& b) { return a.tick < b.tick; }));

    std::vector<uint64_t> hashes;
    std::string trace;
    uint64_t hash = 0xcbf29ce484222325ull;
    std::array<uint32_t, 4> kinds{};
    uint32_t anom_mask = 0, accepted = 0, refused = 0, settled = 0, flags_seen = 0;
    size_t next = 0;
    kinetic_sample s{};
    for (uint32_t k = 0; k < kSteps; ++k) {
        for (; next < ev.size() && ev[next].tick == k; ++next) {
            const Event& e = ev[next];
            if (e.tune) {
                kinetic_tuning t;
                kinetic_default_tuning(&t);
                t.infeasible_policy = uint8_t(e.pos);
                kinetic_set_tuning(h, &t);
                continue;
            }
            const int r = kinetic_submit_segment(h, e.pos, e.dur, e.endv, double(e.start_ms) * 1000.0, e.fam);
            (r == 1 ? accepted : refused) += 1;
        }
        kinetic_step(h, 0.001, &s);
        REQUIRE(std::isfinite(s.p));
        REQUIRE(std::isfinite(s.v));
        REQUIRE(std::isfinite(s.a));
        unsigned char bytes[sizeof(kinetic_sample)];
        std::memcpy(bytes, &s, sizeof(s));
        hash = fnv1a(hash, bytes, sizeof(bytes));
        if ((k + 1) % kBlock == 0) {
            hashes.push_back(hash);
            hash = 0xcbf29ce484222325ull;
        }
        if ((k + 1) % kTraceEveryMs == 0) {
            if (!trace.empty()) trace += ",\n";
            trace += "[" + num(s.p) + "," + num(s.v) + "," + num(s.a) + "]";
        }
        if (s.plan_kind < kinds.size()) ++kinds[s.plan_kind];
        if (s.mode == uint8_t(kinetic::Mode::Settle)) ++settled;   // the style ordinal under both kernels
        anom_mask |= s.anomalies;
        flags_seen |= s.flags;
    }
    REQUIRE(next == ev.size());
    CHECK(s.t_us == double(kSteps) * 1000.0);

    // The script reached what it exists to reach (T10: assert the load landed).
    CHECK(refused == 0);
    CHECK(accepted == ev.size() - 1);
    CHECK(kinds[uint8_t(kinetic::PlanKind::Quintic)] > 0);
    if constexpr (!valence::kKinetic2) {
        CHECK(kinds[uint8_t(kinetic::PlanKind::Cubic)] > 0);
        CHECK(kinds[uint8_t(kinetic::PlanKind::Ruckig)] > 0);
    }
    CHECK(settled > 0);
    CHECK((flags_seen & KINETIC_FLAG_SHAPED) != 0);
    CHECK((flags_seen & KINETIC_FLAG_FALLBACK) != 0);
    kinetic_destroy(h);

    std::string events;
    for (const Event& e : ev) {
        if (!events.empty()) events += ",\n";
        events += e.tune ? "[" + std::to_string(e.tick) + ",\"tune\"," + std::to_string(e.pos) + "]"
                         : "[" + std::to_string(e.tick) + ",\"seg\"," + std::to_string(e.pos) + "," +
                               std::to_string(e.dur) + "," + std::to_string(e.endv) + "," +
                               std::to_string(e.start_ms * 1000ull) + "," + std::to_string(e.fam) + "]";
    }
    std::string hex;
    for (const uint64_t x : hashes) hex += (hex.empty() ? "\"" : ",\"") + hex64(x) + "\"";

    const std::filesystem::path out = std::filesystem::path(__FILE__).parent_path() / ".." / ".." / "fixtures" /
                                      (valence::kKinetic2 ? "kinetic2_trace.json" : "kinetic_trace.json");
    std::filesystem::create_directories(out.parent_path());
    std::ofstream f(out, std::ios::binary | std::ios::trunc);
    REQUIRE(f.good());
    f << "{\n\"about\": \"Generated by test/native/test_kinetic_wasm_trace; replayed by "
         "tools/kinetic-wasm/check.mjs. Do not edit.\",\n"
      << "\"create\": [" << num(kVmax) << "," << num(kAmax) << "," << num(kJmax) << "," << num(kRail) << ","
      << kHorizonMs << "],\n"
      << "\"window\": [" << num(kWinLo) << "," << num(kWinHi) << "],\n"
      // check.mjs sizes its kinetic_tuning buffer from this; absent means 52.
      << (valence::kKinetic2 ? "\"tuning_bytes\": " + std::to_string(sizeof(kinetic_tuning)) + ",\n" : std::string())
      << "\"dt_s\": 0.001,\n\"steps\": " << kSteps << ",\n\"block\": " << kBlock << ",\n"
      << "\"trace_every\": " << kTraceEveryMs << ",\n"
      << "\"summary\": {\"accepted\": " << accepted << ", \"anomaly_mask\": " << anom_mask
      << ", \"plan_kind_ticks\": [" << kinds[0] << "," << kinds[1] << "," << kinds[2] << "," << kinds[3]
      << "], \"settle_ticks\": " << settled << "},\n"
      << "\"events\": [\n" << events << "\n],\n"
      << "\"hashes\": [" << hex << "],\n"
      << "\"trace\": [\n" << trace << "\n]\n}\n";
    REQUIRE(f.good());
    MESSAGE("fixture: " << std::filesystem::absolute(out).string());
}

// StreamIntent.h is the hub's own 0x2100/0x2101 decode (ValenceDevice.cpp
// calls it), so the renderer's input path is checked here against the wire
// contract: SPEC 5.4's sentinel, the zero-duration drop, the window mapping.
TEST_CASE("StreamIntent: one wire sample to one intent") {
    using valence::MotionSource;
    const auto seg = valence::segmentIntent(5000, 250, kUnspec, 100.0f, 300.0f, 1, 777);
    REQUIRE(seg.has_value());
    CHECK(seg->source == MotionSource::Stream);
    CHECK(seg->target_mm == 250.0f);
    CHECK(seg->duration_us == 250000u);
    CHECK(seg->anchor_us == 777u);
    CHECK(seg->curve_family == 1);
    CHECK_FALSE(seg->has_end_vel);

    const auto rest = valence::segmentIntent(10000, 40, 0, 100.0f, 300.0f, 0, 0);
    REQUIRE(rest.has_value());
    CHECK(rest->target_mm == 400.0f);
    CHECK(rest->has_end_vel);              // 0 is a real slope: arrive at rest
    CHECK(rest->end_vel_mm_s == 0.0f);

    const auto fast = valence::segmentIntent(0, 100, -2000, 100.0f, 300.0f, 0, 0);
    REQUIRE(fast.has_value());
    CHECK(fast->target_mm == 100.0f);
    CHECK(fast->end_vel_mm_s == -600.0f);  // -2 window/s over a 300 mm window

    CHECK_FALSE(valence::segmentIntent(5000, 0, kUnspec, 100.0f, 300.0f, 1, 0).has_value());

    const auto pt = valence::pointIntent(2500, 0, 100.0f, 300.0f, 5);
    CHECK(pt.target_mm == 175.0f);
    CHECK(pt.duration_us == 0u);
    CHECK_FALSE(pt.has_end_vel);           // a point's 0 means no hint
    CHECK(valence::pointIntent(2500, 500, 100.0f, 300.0f, 5).end_vel_mm_s == 150.0f);
}
