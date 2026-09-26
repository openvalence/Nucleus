// test_stored_state -- native doctest suite for the persisted-state blob codecs
// Constraints:
// - Hardware-free: the codecs the P4 (NVS) and the host twin (files) share,
//   exercised with no storage at all.
// - Every rejection must leave the outputs (or the store) untouched: that is
//   what makes "factory values stand" true rather than hoped.
// See: flagship_p4/src/hub/StoredState.h, flagship_p4/src/patterns/PatternPresetStore.h,
// bd val-091.11.2, val-wcm

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cstddef>
#include <cstring>
#include <limits>
#include <span>

// Named directly so the library finder adds lib/valence; StoredState.h
// reaches it only through ValenceCatalog.h, which the finder does not follow.
#include "valence/channel/catalog.hpp"

#include "../../../flagship_p4/src/hub/StoredState.h"
#include "../../../flagship_p4/src/patterns/PatternPresetStore.h"

using valence::MotionTuning;
using valence::PatternPresetStore;
using valence::StoredConfig;
namespace stored = valence::stored;

namespace {

constexpr float kFactoryGuard = 1.25f;

// A tuning set with every field off its zero value and inside its bounds.
MotionTuning sampleTuning() {
    MotionTuning t;
    t.jmax_ovr = 150000.0f;
    t.vmax_ovr = 3.5f;
    t.amax_ovr = 42.0f;
    t.chase_ff = true;
    t.chase_accel_ff = false;
    t.chase_gain = 0.8f;
    t.chase_lookahead = 1.3f;
    t.chase_dense_us = 35000;
    t.chase_aim_extrap = true;
    t.handoff_k = 2.0f;
    t.curve_policy = 2;
    t.infeasible_policy = 1;
    t.smooth_budget = 0.25f;
    t.amplitude_budget = 0.6f;
    t.blend_steps = 4;
    t.settle_grace_us = 30000;
    t.overshoot_guard = kFactoryGuard;
    return t;
}

StoredConfig sampleConfig() {
    StoredConfig c;
    c.window_min = 20.0f;
    c.window_max = 180.0f;
    c.user_speed = 75.0f;
    c.input_jerk = 900000.0f;
    c.max_rail = 240.0f;
    return c;
}

std::array<std::byte, stored::kConfigBlobBytes> encodedConfig(uint16_t gen = 7) {
    std::array<std::byte, stored::kConfigBlobBytes> b{};
    REQUIRE(stored::encodeConfig(b, sampleConfig(), sampleTuning(), gen) == b.size());
    return b;
}

}  // namespace

TEST_CASE("config blob: round trip carries config, tuning and cfg_gen") {
    const auto b = encodedConfig(4242);
    StoredConfig c;
    MotionTuning t;
    uint16_t gen = 0;
    REQUIRE(stored::decodeConfig(b, kFactoryGuard, c, t, gen));
    CHECK(c == sampleConfig());
    CHECK(t == sampleTuning());
    CHECK(gen == 4242);
}

TEST_CASE("config blob: overshoot is stored as on/off and re-derived from the factory multiplier") {
    MotionTuning off = sampleTuning();
    off.overshoot_guard = 0.0f;
    std::array<std::byte, stored::kConfigBlobBytes> b{};
    REQUIRE(stored::encodeConfig(b, sampleConfig(), off, 1) == b.size());
    StoredConfig c;
    MotionTuning t;
    uint16_t gen = 0;
    REQUIRE(stored::decodeConfig(b, kFactoryGuard, c, t, gen));
    CHECK(t.overshoot_guard == 0.0f);

    const auto on = encodedConfig();
    REQUIRE(stored::decodeConfig(on, 2.0f, c, t, gen));
    CHECK(t.overshoot_guard == 2.0f);   // a new engine factory value wins
}

TEST_CASE("config blob: every rejection leaves the factory values standing") {
    const StoredConfig factoryCfg{};
    const MotionTuning factoryTune{};
    auto expectRejected = [&](std::span<const std::byte> in) {
        StoredConfig c = factoryCfg;
        MotionTuning t = factoryTune;
        uint16_t gen = 99;
        CHECK_FALSE(stored::decodeConfig(in, kFactoryGuard, c, t, gen));
        CHECK(c == factoryCfg);
        CHECK(t == factoryTune);
        CHECK(gen == 99);
    };

    SUBCASE("version mismatch") {
        auto b = encodedConfig();
        b[4] = std::byte{stored::kConfigVersion + 1};
        expectRejected(b);
    }
    SUBCASE("the v1 layout (40 B, u16 version 1)") {
        std::array<std::byte, 40> v1{};
        const uint32_t magic = stored::kConfigMagic;
        const uint16_t version = 1, gen = 3;
        std::memcpy(v1.data(), &magic, 4);
        std::memcpy(v1.data() + 4, &version, 2);
        std::memcpy(v1.data() + 6, &gen, 2);
        expectRejected(v1);
    }
    SUBCASE("truncated") {
        const auto b = encodedConfig();
        expectRejected(std::span<const std::byte>(b).first(b.size() - 1));
        expectRejected(std::span<const std::byte>(b).first(7));
        expectRejected({});
    }
    SUBCASE("oversized") {
        std::array<std::byte, stored::kConfigBlobBytes + 1> big{};
        const auto b = encodedConfig();
        std::memcpy(big.data(), b.data(), b.size());
        expectRejected(big);
    }
    SUBCASE("wrong magic") {
        auto b = encodedConfig();
        b[0] = std::byte{0};
        expectRejected(b);
    }
    SUBCASE("cfg_gen 0") {
        std::array<std::byte, stored::kConfigBlobBytes> b{};
        REQUIRE(stored::encodeConfig(b, sampleConfig(), sampleTuning(), 0) == b.size());
        expectRejected(b);
    }
    SUBCASE("out-of-range tuning is rejected whole, never clamped") {
        MotionTuning t = sampleTuning();
        t.blend_steps = 11;
        std::array<std::byte, stored::kConfigBlobBytes> b{};
        REQUIRE(stored::encodeConfig(b, sampleConfig(), t, 1) == b.size());
        expectRejected(b);
    }
    SUBCASE("non-finite config is rejected whole") {
        StoredConfig c = sampleConfig();
        c.user_accel = std::numeric_limits<float>::quiet_NaN();
        std::array<std::byte, stored::kConfigBlobBytes> b{};
        REQUIRE(stored::encodeConfig(b, c, sampleTuning(), 1) == b.size());
        expectRejected(b);
    }
}

TEST_CASE("config blob: encode refuses a short buffer") {
    std::array<std::byte, stored::kConfigBlobBytes - 1> small{};
    CHECK(stored::encodeConfig(small, sampleConfig(), sampleTuning(), 1) == 0);
}

// ---- presets -------------------------------------------------------------------

namespace {

PatternPresetStore::Payload payload(uint8_t seed) {
    PatternPresetStore::Payload p{};
    for (size_t i = 0; i < p.size(); ++i) p[i] = uint8_t(seed + i);
    return p;
}

// The store is ~1.7 KB; the encoded blob the same. Statics keep both off the
// test's stack, the habit the firmware needs.
std::array<std::byte, PatternPresetStore::kBlobBytes> g_blob{};

}  // namespace

TEST_CASE("presets blob: round trip keeps every slot and the generation") {
    static PatternPresetStore a;
    REQUIRE(a.save(0, "first", payload(1)));
    REQUIRE(a.save(23, "last slot", payload(9)));
    REQUIRE(a.save(5, "gone", payload(3)));
    REQUIRE(a.remove(5));
    REQUIRE(a.rename(0, "renamed"));
    REQUIRE(a.encode(g_blob) == g_blob.size());

    static PatternPresetStore b;
    REQUIRE(b.decode(g_blob));
    CHECK(b.generation() == a.generation());
    CHECK(b.count() == 2);
    REQUIRE(b.slot(0) != nullptr);
    CHECK(b.slot(0)->nameView() == "renamed");
    CHECK(b.slot(0)->payload == payload(1));
    REQUIRE(b.slot(23) != nullptr);
    CHECK(b.slot(23)->nameView() == "last slot");
    CHECK(b.slot(5) == nullptr);
}

TEST_CASE("presets blob: every rejection leaves the store untouched") {
    static PatternPresetStore src;
    REQUIRE(src.save(2, "keep", payload(7)));
    REQUIRE(src.encode(g_blob) == g_blob.size());

    static PatternPresetStore dst;
    REQUIRE(dst.save(1, "already here", payload(4)));
    const uint16_t gen = dst.generation();
    auto expectRejected = [&](std::span<const std::byte> in) {
        CHECK_FALSE(dst.decode(in));
        CHECK(dst.generation() == gen);
        CHECK(dst.count() == 1);
        REQUIRE(dst.slot(1) != nullptr);
        CHECK(dst.slot(1)->nameView() == "already here");
    };

    SUBCASE("version mismatch") {
        auto b = g_blob;
        b[4] = std::byte{PatternPresetStore::kBlobVersion + 1};
        expectRejected(b);
    }
    SUBCASE("truncated") {
        expectRejected(std::span<const std::byte>(g_blob).first(g_blob.size() - 1));
        expectRejected({});
    }
    SUBCASE("a name with no terminator in its slot") {
        auto b = g_blob;
        for (size_t k = 0; k < PatternPresetStore::kNameMax; ++k) b[7 + k] = std::byte{'x'};
        expectRejected(b);
    }
    SUBCASE("garbage after a name's terminator") {
        auto b = g_blob;
        const size_t slot2 = 7 + 2 * (PatternPresetStore::kNameMax + PatternPresetStore::kPayloadBytes);
        b[slot2 + PatternPresetStore::kNameMax - 1] = std::byte{'z'};
        expectRejected(b);
    }
}
