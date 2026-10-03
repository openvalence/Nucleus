#pragma once

// StoredState -- the config-generation state the hub keeps across a reboot
// (0x1000 config, 0x1030 modes, 0x1120-0x1122 tuning, cfg_gen) and its blob
// codec, hardware-free
// Constraints:
// - HARDWARE-FREE and header-only: the P4 composition (NVS), the host twin (a
//   file) and the native suite all run this one codec.
// - ONE BLOB PER CONCERN, opening [magic u32][version u8], decoded only at the
//   exact length its version defines. An unknown version, a short or long
//   read, a non-finite or out-of-range value: the blob is REJECTED WHOLE, the
//   factory values stand, and the ConfigReject names the check that refused
//   it for the boot log. A layout change never misreads bytes, and nothing is
//   clamped into shape at boot (a clamp is a config change, which §4.2 would
//   make bump cfg_gen in the middle of restoring it).
// - LAYOUT CHANGES APPEND AND MIGRATE: every version from kConfigOldestVersion
//   up still decodes, its missing tail taking the factory value, and the range
//   checks run on the extended values. A firmware update never orphans a
//   stored setting; the next write is the current version. A NEWER version is
//   refused whole, never read as the prefix this firmware knows.
// - A TUNING FIELD APPENDED LATER takes the engine's factory value, passed in
//   as factoryGuard is, never MotionTuning{}'s zero.
// - THE FIRST-RUN RECORD SURVIVES MIGRATION: every layout from v5 on carries
//   setup_written and decode keeps it, because losing it re-gates content
//   motion on an upgraded machine (RFC-079). Older blobs have no record and
//   decode uncommissioned.
// - cfg_gen RIDES WITH EVERY VALUE IT COVERS. Both 0x3000 and the tuning
//   writers bump it, so all of them share this one blob: one NVS write is
//   atomic, and a generation can never land without its values or the reverse.
// - Fields are copied one at a time in host byte order. The blob never leaves
//   the device that wrote it, so no wire endianness applies, and struct padding
//   never reaches storage.
// - overshoot_clamp is stored as the wire's on/off, not the engine multiplier:
//   "on" is whatever the running engine's factory multiplier is.
// See: ValenceHub.cpp (NVS key and wear), ValenceDevice.h, bd val-091.11.2

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <span>

#include "ValenceCatalog.h"
#include "motion/ValenceMotion.h"

namespace valence {

// The 0x1000 snapshot and the 0x3000 writer both speak exactly these eight
// values.
struct StoredConfig {
    float window_min  = factory::window_min;
    float window_max  = factory::window_max;
    float jog_speed  = factory::jog_speed;
    float jog_accel  = factory::jog_accel;
    float input_speed = factory::input_speed;
    float input_accel = factory::input_accel;
    float input_jerk  = factory::input_jerk;
    float max_rail    = factory::max_rail;

    bool operator==(const StoredConfig&) const = default;
};

// The first-run pass (RFC-079 setup): the 0x3000 keys an owner must write
// once, bit k-1 for key k. Every one of the eight is the machine's geometry
// or a ceiling, so all eight are required; the 0x3120 ceiling overrides are
// not (0 derives them from these).
inline constexpr uint8_t kSetupRequiredMask = 0xFF;

// The stored 0x1030 modes that are not engine tuning, and the first-run record.
struct StoredModes {
    uint8_t horizon = 0;   // schedule_horizon ordinal: kHorizonMs (ValenceCatalog.h)
    bool    flipped = false;   // axis.flipped (RFC-088): how the rail is mounted
    // 0x3000 keys accepted at least once, same bits as kSetupRequiredMask.
    // Only ever gains bits; a factory-fresh or migrated hub starts at 0.
    uint8_t setup_written = 0;

    bool operator==(const StoredModes&) const = default;
};

// True once the owner has written every required setup field. Until then the
// arbiter refuses every content source (MotionArbiter::setCommissioned()).
inline bool commissioned(const StoredModes& m) {
    return (m.setup_written & kSetupRequiredMask) == kSetupRequiredMask;
}

// ---- tuning bounds ------------------------------------------------------------
// What 0x3120 clamps to and what a stored set must sit inside. Mirrors the
// catalog's kinetic-* min/max, which mirror the engine's own clamps.
namespace tuning_bounds {
inline constexpr float    jmax_ovr_max      = 2000000.0f;
inline constexpr float    vmax_ovr_max      = 20.0f;
inline constexpr float    amax_ovr_max      = 500.0f;
inline constexpr float    chase_gain_max    = 1.5f;
inline constexpr float    lookahead_max     = 8.0f;
inline constexpr float    dense_ms_min      = 10.0f;
inline constexpr float    dense_ms_max      = 500.0f;
inline constexpr float    handoff_k_max     = 8.0f;
inline constexpr uint8_t  curve_policy_max  = 2;
inline constexpr uint8_t  infeasible_max    = 1;
inline constexpr float    budget_max        = 1.0f;
inline constexpr uint8_t  blend_steps_min   = 1;
inline constexpr uint8_t  blend_steps_max   = 10;
inline constexpr float    settle_ms_max     = 200.0f;
}  // namespace tuning_bounds

// "on" is the engine's factory multiplier, never a number chosen here.
inline float overshootGuardFor(bool on, float factoryGuard) {
    return on ? (factoryGuard > 0.0f ? factoryGuard : 1.0f) : 0.0f;
}

// ---- config blob ----------------------------------------------------------------

namespace stored {

inline constexpr uint32_t kConfigMagic   = 0x56434647u;  // "VCFG"
inline constexpr uint8_t  kConfigVersion = 5;            // bump on ANY layout change
// v1, the 40 B struct dump this codec replaced (u16 version), is retired:
// refused, never migrated.
inline constexpr uint8_t  kConfigOldestVersion = 2;
// v2: magic 4, version 1, cfg_gen 2, config 8 x f32, tuning 8 x f32 + 2 x u32 + 7 x u8
inline constexpr size_t   kConfigV2Bytes = 4 + 1 + 2 + 32 + 32 + 8 + 7;
// v3 appends the schedule_horizon ordinal (u8), v4 the flip (u8, 0/1), v5
// the setup_written mask (u8).
inline constexpr size_t   kConfigV3Bytes = kConfigV2Bytes + 1;
inline constexpr size_t   kConfigV4Bytes = kConfigV3Bytes + 1;
inline constexpr size_t   kConfigBlobBytes = kConfigV4Bytes + 1;

// 0 for a version this firmware cannot read.
inline constexpr size_t configBytesFor(uint8_t version) {
    return version == 2 ? kConfigV2Bytes
         : version == 3 ? kConfigV3Bytes
         : version == 4 ? kConfigV4Bytes
         : version == 5 ? kConfigBlobBytes : 0;
}

static_assert([] {
    for (unsigned v = kConfigOldestVersion; v <= kConfigVersion; ++v)
        if (configBytesFor(uint8_t(v)) == 0) return false;
    return configBytesFor(kConfigVersion) == kConfigBlobBytes;
}(), "every readable config version needs its length in configBytesFor()");

// Why decodeConfig() refused a blob, one value per check.
enum class ConfigReject : uint8_t {
    BadMagic,         // not a config blob
    RetiredVersion,   // older than kConfigOldestVersion
    NewerVersion,     // written by a newer firmware
    BadSize,          // shorter than the header, or not its version's length
    NoGeneration,     // a stored cfg_gen of 0
    BadConfig,        // a 0x1000 value non-finite or outside its bounds
    BadTuning,        // a 0x1120-0x1122 value outside its bounds, or a flag byte not 0/1
    BadModes,         // a 0x1030 value outside its bounds, or the flip byte not 0/1
};

inline const char* configRejectName(ConfigReject r) {
    switch (r) {
        case ConfigReject::BadMagic:       return "bad magic";
        case ConfigReject::RetiredVersion: return "retired layout";
        case ConfigReject::NewerVersion:   return "newer firmware's layout";
        case ConfigReject::BadSize:        return "length does not match its version";
        case ConfigReject::NoGeneration:   return "cfg_gen 0";
        case ConfigReject::BadConfig:      return "machine config out of range";
        case ConfigReject::BadTuning:      return "kinetic tuning out of range";
        case ConfigReject::BadModes:       return "machine modes out of range";
    }
    return "unknown";
}

namespace detail {
template <typename T>
void put(std::span<std::byte> out, size_t& n, T v) {
    std::memcpy(out.data() + n, &v, sizeof(T));
    n += sizeof(T);
}
template <typename T>
T get(std::span<const std::byte> in, size_t& n) {
    T v;
    std::memcpy(&v, in.data() + n, sizeof(T));
    n += sizeof(T);
    return v;
}
inline bool in(float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; }
}  // namespace detail

inline bool configValid(const StoredConfig& c) {
    using detail::in;
    return in(c.window_min,  0.0f, ceiling::rail_mm)
        && in(c.window_max,  0.0f, ceiling::rail_mm)
        && c.window_min < c.window_max
        && in(c.jog_speed,  ceiling::speed_min, ceiling::speed_max)
        && in(c.jog_accel,  ceiling::accel_min, ceiling::accel_max)
        && in(c.input_speed, ceiling::speed_min, ceiling::speed_max)
        && in(c.input_accel, ceiling::accel_min, ceiling::accel_max)
        && in(c.input_jerk,  ceiling::jerk_min,  ceiling::jerk_max)
        && in(c.max_rail,    ceiling::rail_min,  ceiling::rail_mm);
}

inline bool tuningValid(const MotionTuning& t) {
    using detail::in;
    namespace b = tuning_bounds;
    return in(t.jmax_ovr, 0.0f, b::jmax_ovr_max)
        && in(t.vmax_ovr, 0.0f, b::vmax_ovr_max)
        && in(t.amax_ovr, 0.0f, b::amax_ovr_max)
        && in(t.chase_gain, 0.0f, b::chase_gain_max)
        && in(t.chase_lookahead, 0.0f, b::lookahead_max)
        && in(float(t.chase_dense_us) / 1000.0f, b::dense_ms_min, b::dense_ms_max)
        && in(t.handoff_k, 0.0f, b::handoff_k_max)
        && t.curve_policy <= b::curve_policy_max
        && t.infeasible_policy <= b::infeasible_max
        && in(t.smooth_budget, 0.0f, b::budget_max)
        && in(t.amplitude_budget, 0.0f, b::budget_max)
        && t.blend_steps >= b::blend_steps_min && t.blend_steps <= b::blend_steps_max
        && in(float(t.settle_grace_us) / 1000.0f, 0.0f, b::settle_ms_max);
}

inline bool modesValid(const StoredModes& m) { return m.horizon < kHorizonMs.size(); }

// Returns bytes written: kConfigBlobBytes, or 0 when `out` is too small.
inline size_t encodeConfig(std::span<std::byte> out, const StoredConfig& c,
                           const MotionTuning& t, const StoredModes& m, uint16_t cfgGen) {
    using detail::put;
    if (out.size() < kConfigBlobBytes) return 0;
    size_t n = 0;
    put(out, n, kConfigMagic);
    put(out, n, kConfigVersion);
    put(out, n, cfgGen);
    for (float v : {c.window_min, c.window_max, c.jog_speed, c.jog_accel,
                    c.input_speed, c.input_accel, c.input_jerk, c.max_rail})
        put(out, n, v);
    for (float v : {t.jmax_ovr, t.vmax_ovr, t.amax_ovr, t.chase_gain, t.chase_lookahead,
                    t.handoff_k, t.smooth_budget, t.amplitude_budget})
        put(out, n, v);
    put(out, n, t.chase_dense_us);
    put(out, n, t.settle_grace_us);
    for (uint8_t v : {uint8_t(t.chase_ff), uint8_t(t.chase_accel_ff), uint8_t(t.chase_aim_extrap),
                      t.curve_policy, t.infeasible_policy, t.blend_steps,
                      uint8_t(t.overshoot_guard > 0.0f)})
        put(out, n, v);
    put(out, n, m.horizon);
    put(out, n, uint8_t(m.flipped));
    put(out, n, m.setup_written);
    return n;
}

// All-or-nothing: an error leaves every output untouched, so the caller's
// factory values stand. factoryGuard is the running engine's overshoot
// multiplier. Stack cost: the ~100 B of staged values.
inline std::expected<void, ConfigReject> decodeConfig(std::span<const std::byte> in, float factoryGuard,
                                                      StoredConfig& cfgOut, MotionTuning& tuneOut,
                                                      StoredModes& modesOut, uint16_t& genOut) {
    using detail::get;
    using Err = std::unexpected<ConfigReject>;
    if (in.size() < 5) return Err(ConfigReject::BadSize);
    size_t n = 0;
    if (get<uint32_t>(in, n) != kConfigMagic) return Err(ConfigReject::BadMagic);
    const uint8_t version = get<uint8_t>(in, n);
    if (version < kConfigOldestVersion) return Err(ConfigReject::RetiredVersion);
    if (version > kConfigVersion) return Err(ConfigReject::NewerVersion);
    if (in.size() != configBytesFor(version)) return Err(ConfigReject::BadSize);
    const uint16_t gen = get<uint16_t>(in, n);
    if (gen == 0) return Err(ConfigReject::NoGeneration);

    StoredConfig c;
    for (float* f : {&c.window_min, &c.window_max, &c.jog_speed, &c.jog_accel,
                     &c.input_speed, &c.input_accel, &c.input_jerk, &c.max_rail})
        *f = get<float>(in, n);

    MotionTuning t;
    for (float* f : {&t.jmax_ovr, &t.vmax_ovr, &t.amax_ovr, &t.chase_gain, &t.chase_lookahead,
                     &t.handoff_k, &t.smooth_budget, &t.amplitude_budget})
        *f = get<float>(in, n);
    t.chase_dense_us  = get<uint32_t>(in, n);
    t.settle_grace_us = get<uint32_t>(in, n);
    std::array<uint8_t, 7> b{};
    for (uint8_t& v : b) v = get<uint8_t>(in, n);
    for (size_t i : {0u, 1u, 2u, 6u})
        if (b[i] > 1) return Err(ConfigReject::BadTuning);
    t.chase_ff          = b[0] != 0;
    t.chase_accel_ff    = b[1] != 0;
    t.chase_aim_extrap  = b[2] != 0;
    t.curve_policy      = b[3];
    t.infeasible_policy = b[4];
    t.blend_steps       = b[5];
    t.overshoot_guard   = overshootGuardFor(b[6] != 0, factoryGuard);

    StoredModes m;
    if (version >= 3) m.horizon = get<uint8_t>(in, n);
    if (version >= 4) {
        const uint8_t f = get<uint8_t>(in, n);
        if (f > 1) return Err(ConfigReject::BadModes);
        m.flipped = f != 0;
    }
    if (version >= 5) m.setup_written = get<uint8_t>(in, n);

    if (!configValid(c)) return Err(ConfigReject::BadConfig);
    if (!tuningValid(t)) return Err(ConfigReject::BadTuning);
    if (!modesValid(m)) return Err(ConfigReject::BadModes);
    cfgOut = c;
    tuneOut = t;
    modesOut = m;
    genOut = gen;
    return {};
}

}  // namespace stored
}  // namespace valence
