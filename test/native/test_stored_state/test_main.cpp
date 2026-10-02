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
#include "valence/wire/messages/store_item.hpp"

#include "../../../flagship_p4/src/hub/StoredState.h"
#include "../../../flagship_p4/src/patterns/PatternPresetStore.h"

using valence::MotionTuning;
using valence::PatternPresetStore;
using valence::StoredConfig;
namespace stored = valence::stored;

namespace {

constexpr float kFactoryGuard = 1.25f;

// The decode out-param the config cases do not inspect.
valence::StoredModes g_modes;

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
    c.jog_speed = 75.0f;
    c.input_jerk = 900000.0f;
    c.max_rail = 240.0f;
    return c;
}

std::array<std::byte, stored::kConfigBlobBytes> encodedConfig(uint16_t gen = 7) {
    std::array<std::byte, stored::kConfigBlobBytes> b{};
    REQUIRE(stored::encodeConfig(b, sampleConfig(), sampleTuning(), valence::StoredModes{}, gen) == b.size());
    return b;
}

}  // namespace

TEST_CASE("config blob: round trip carries config, tuning and cfg_gen") {
    const auto b = encodedConfig(4242);
    StoredConfig c;
    MotionTuning t;
    uint16_t gen = 0;
    REQUIRE(stored::decodeConfig(b, kFactoryGuard, c, t, g_modes, gen));
    CHECK(c == sampleConfig());
    CHECK(t == sampleTuning());
    CHECK(gen == 4242);
}

TEST_CASE("config blob: overshoot is stored as on/off and re-derived from the factory multiplier") {
    MotionTuning off = sampleTuning();
    off.overshoot_guard = 0.0f;
    std::array<std::byte, stored::kConfigBlobBytes> b{};
    REQUIRE(stored::encodeConfig(b, sampleConfig(), off, valence::StoredModes{}, 1) == b.size());
    StoredConfig c;
    MotionTuning t;
    uint16_t gen = 0;
    REQUIRE(stored::decodeConfig(b, kFactoryGuard, c, t, g_modes, gen));
    CHECK(t.overshoot_guard == 0.0f);

    const auto on = encodedConfig();
    REQUIRE(stored::decodeConfig(on, 2.0f, c, t, g_modes, gen));
    CHECK(t.overshoot_guard == 2.0f);   // a new engine factory value wins
}

TEST_CASE("config blob: every rejection leaves the factory values standing") {
    const StoredConfig factoryCfg{};
    const MotionTuning factoryTune{};
    auto expectRejected = [&](std::span<const std::byte> in) {
        StoredConfig c = factoryCfg;
        MotionTuning t = factoryTune;
        uint16_t gen = 99;
        CHECK_FALSE(stored::decodeConfig(in, kFactoryGuard, c, t, g_modes, gen));
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
        REQUIRE(stored::encodeConfig(b, sampleConfig(), sampleTuning(), valence::StoredModes{}, 0) == b.size());
        expectRejected(b);
    }
    SUBCASE("out-of-range tuning is rejected whole, never clamped") {
        MotionTuning t = sampleTuning();
        t.blend_steps = 11;
        std::array<std::byte, stored::kConfigBlobBytes> b{};
        REQUIRE(stored::encodeConfig(b, sampleConfig(), t, valence::StoredModes{}, 1) == b.size());
        expectRejected(b);
    }
    SUBCASE("non-finite config is rejected whole") {
        StoredConfig c = sampleConfig();
        c.jog_accel = std::numeric_limits<float>::quiet_NaN();
        std::array<std::byte, stored::kConfigBlobBytes> b{};
        REQUIRE(stored::encodeConfig(b, c, sampleTuning(), valence::StoredModes{}, 1) == b.size());
        expectRejected(b);
    }
}

TEST_CASE("config blob: encode refuses a short buffer") {
    std::array<std::byte, stored::kConfigBlobBytes - 1> small{};
    CHECK(stored::encodeConfig(small, sampleConfig(), sampleTuning(), valence::StoredModes{}, 1) == 0);
}

// ---- presets -------------------------------------------------------------------

namespace {

PatternPresetStore::Payload payload(uint8_t seed) {
    PatternPresetStore::Payload p{};
    for (size_t i = 0; i < p.size(); ++i) p[i] = uint8_t(seed + i);
    return p;
}

// The store is ~2.1 KB; the encoded blob the same. Statics keep both off the
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

// A v2 blob byte for byte as the firmware wrote it before the jog rename, when
// bytes 15..22 held user_speed and user_accel. The layout is positional, so the
// rename orphans nothing: the same bytes decode into jog_speed and jog_accel.
TEST_CASE("config blob: a pre-rename v2 blob decodes into the jog fields") {
    std::array<std::byte, 4 + 1 + 2 + 32 + 32 + 8 + 7> b{};
    size_t n = 0;
    auto put = [&](const auto v) {
        std::memcpy(b.data() + n, &v, sizeof v);
        n += sizeof v;
    };
    put(uint32_t(0x56434647u));  // "VCFG"
    put(uint8_t(2));
    put(uint16_t(31));
    // window_min, window_max, user_speed, user_accel, input_speed, input_accel,
    // input_jerk, max_rail
    for (float v : {10.0f, 190.0f, 64.0f, 333.0f, 800.0f, 40000.0f, 1500000.0f, 300.0f}) put(v);
    for (float v : {0.0f, 0.0f, 0.0f, 0.5f, 1.0f, 2.0f, 0.1f, 0.5f}) put(v);
    put(uint32_t(30000));   // chase_dense_us
    put(uint32_t(20000));   // settle_grace_us
    for (uint8_t v : {uint8_t(1), uint8_t(0), uint8_t(1), uint8_t(1), uint8_t(0), uint8_t(3), uint8_t(1)}) put(v);
    REQUIRE(n == b.size());

    StoredConfig c;
    MotionTuning t;
    uint16_t gen = 0;
    REQUIRE(stored::decodeConfig(b, kFactoryGuard, c, t, g_modes, gen));
    CHECK(gen == 31);
    CHECK(c.jog_speed == 64.0f);
    CHECK(c.jog_accel == 333.0f);
    CHECK(c.input_speed == 800.0f);
    CHECK(c.max_rail == 300.0f);
}

TEST_CASE("config blob: the schedule horizon round trips; a v2 blob migrates to the 250 ms default") {
    std::array<std::byte, stored::kConfigBlobBytes> b{};
    valence::StoredModes m;
    m.horizon = 2;
    REQUIRE(stored::encodeConfig(b, sampleConfig(), sampleTuning(), m, 9) == b.size());
    StoredConfig c;
    MotionTuning t;
    valence::StoredModes got;
    uint16_t gen = 0;
    REQUIRE(stored::decodeConfig(b, kFactoryGuard, c, t, got, gen));
    CHECK(got.horizon == 2);
    CHECK(valence::kHorizonMs[got.horizon] == 1000);

    // The same bytes cut back to the v2 layout and relabeled v2: the tail the
    // old firmware never wrote takes the factory value.
    std::array<std::byte, stored::kConfigV2Bytes> v2{};
    std::memcpy(v2.data(), b.data(), v2.size());
    v2[4] = std::byte{2};
    got.horizon = 1;
    REQUIRE(stored::decodeConfig(v2, kFactoryGuard, c, t, got, gen));
    CHECK(got.horizon == 0);
    CHECK(valence::kHorizonMs[got.horizon] == 250);

    // An ordinal past the select's options is rejected whole.
    b[stored::kConfigV3Bytes - 1] = std::byte{3};
    CHECK_FALSE(stored::decodeConfig(b, kFactoryGuard, c, t, got, gen));
    // A v3 label on v2 bytes is a length mismatch, never a misread.
    v2[4] = std::byte{3};
    CHECK_FALSE(stored::decodeConfig(v2, kFactoryGuard, c, t, got, gen));
}

TEST_CASE("config blob: v4 carries the flip; a v3 blob migrates to unflipped") {
    std::array<std::byte, stored::kConfigBlobBytes> b{};
    valence::StoredModes m;
    m.horizon = 1;
    m.flipped = true;
    REQUIRE(stored::encodeConfig(b, sampleConfig(), sampleTuning(), m, 12) == b.size());
    StoredConfig c;
    MotionTuning t;
    valence::StoredModes got;
    uint16_t gen = 0;
    REQUIRE(stored::decodeConfig(b, kFactoryGuard, c, t, got, gen));
    CHECK(got.flipped);
    CHECK(got.horizon == 1);

    std::array<std::byte, stored::kConfigV3Bytes> v3{};
    std::memcpy(v3.data(), b.data(), v3.size());
    v3[4] = std::byte{3};
    REQUIRE(stored::decodeConfig(v3, kFactoryGuard, c, t, got, gen));
    CHECK_FALSE(got.flipped);
    CHECK(got.horizon == 1);

    // The flip byte is a bool: anything but 0 or 1 is rejected whole.
    b[stored::kConfigV4Bytes - 1] = std::byte{2};
    CHECK_FALSE(stored::decodeConfig(b, kFactoryGuard, c, t, got, gen));
}

TEST_CASE("config blob: v5 carries the first-run record; an older blob migrates uncommissioned") {
    std::array<std::byte, stored::kConfigBlobBytes> b{};
    valence::StoredModes m;
    m.flipped = true;
    m.setup_written = valence::kSetupRequiredMask;
    REQUIRE(valence::commissioned(m));
    REQUIRE(stored::encodeConfig(b, sampleConfig(), sampleTuning(), m, 14) == b.size());
    StoredConfig c;
    MotionTuning t;
    valence::StoredModes got;
    uint16_t gen = 0;
    REQUIRE(stored::decodeConfig(b, kFactoryGuard, c, t, got, gen));
    CHECK(got.setup_written == valence::kSetupRequiredMask);
    CHECK(valence::commissioned(got));
    CHECK(got.flipped);

    // A blob from before the record: nothing proves the owner confirmed the
    // geometry, so the hub starts uncommissioned and the rest is kept.
    std::array<std::byte, stored::kConfigV4Bytes> v4{};
    std::memcpy(v4.data(), b.data(), v4.size());
    v4[4] = std::byte{4};
    REQUIRE(stored::decodeConfig(v4, kFactoryGuard, c, t, got, gen));
    CHECK(got.setup_written == 0);
    CHECK_FALSE(valence::commissioned(got));
    CHECK(got.flipped);

    // A partial pass persists as written and is still uncommissioned.
    m.setup_written = 0x7F;   // max_rail never written
    REQUIRE(stored::encodeConfig(b, sampleConfig(), sampleTuning(), m, 15) == b.size());
    REQUIRE(stored::decodeConfig(b, kFactoryGuard, c, t, got, gen));
    CHECK(got.setup_written == 0x7F);
    CHECK_FALSE(valence::commissioned(got));
    CHECK_FALSE(valence::commissioned(valence::StoredModes{}));   // factory-fresh
}

// RFC-073: every BLOB_REQ item carries SHA-256 over its payload, so a receiver
// decides BLOB_DONE status 1 on its own.
TEST_CASE("preset item: the digest rides every item and a flipped payload byte fails it") {
    static PatternPresetStore st;
    REQUIRE(st.save(4, "digest me", payload(11)));
    std::array<std::byte, 160> out{};
    REQUIRE(PatternPresetStore::itemMaxBytes(13) <= out.size());

    const size_t n = st.encodeItem(4, "pattern.frayd", out);
    REQUIRE(n > 0);
    CHECK(n <= PatternPresetStore::itemMaxBytes(13));
    auto item = valence::decodeStoreItem(std::span<const std::byte>(out.data(), n));
    REQUIRE(item.isOk());
    CHECK(item.value().slot == 4);
    CHECK(item.value().name == "digest me");
    CHECK(item.value().kind == "pattern.frayd");
    REQUIRE(item.value().payload.size() == PatternPresetStore::kPayloadBytes);
    CHECK(std::memcmp(item.value().payload.data(), st.slot(4)->payload.data(),
                      PatternPresetStore::kPayloadBytes) == 0);
    CHECK(item.value().has_digest);
    CHECK(valence::storeItemDoneStatus(item.value()) == valence::BlobDoneStatus::VerifiedComplete);

    // The payload bstr is the only run of the slot's bytes in the item: flip
    // its first byte where it actually sits on the wire.
    const auto* at = item.value().payload.data();
    const size_t off = size_t(at - out.data());
    out[off] = std::byte(uint8_t(out[off]) ^ 0x01u);
    auto bent = valence::decodeStoreItem(std::span<const std::byte>(out.data(), n));
    REQUIRE(bent.isOk());
    CHECK(valence::storeItemDoneStatus(bent.value()) == valence::BlobDoneStatus::HashMismatch);

    // Empty and out-of-range slots encode nothing; a short buffer neither.
    CHECK(st.encodeItem(5, "pattern.frayd", out) == 0);
    CHECK(st.encodeItem(PatternPresetStore::kCapacity, "pattern.frayd", out) == 0);
    std::array<std::byte, 64> tiny{};
    CHECK(st.encodeItem(4, "pattern.frayd", tiny) == 0);
}
