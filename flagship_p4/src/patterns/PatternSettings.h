#pragma once

// PatternSettings -- every knob both generators run on, as one value: the
// classic pattern set, the advanced set, each generator's own run flag, the
// background_run policy, and the stroke frame they map into
// Constraints:
// - HARDWARE-FREE value type. The hub delegate OWNS the live copy (it is the
//   one writer: intents, presets, e-stop, source release) and hands WHOLE
//   copies to the generator's task through ValencePattern.h. The generator
//   never writes one back; nothing here is shared memory.
// - Every setter clamps to the catalog's own bounds (ValenceCatalog.h,
//   pattern-state / pattern-advanced / pattern-adv-mod-*), so the ECHO a
//   delegate builds from the fields afterward is the post-clamp truth.
// - The preset payload layout is wire-visible through BLOB_REQ exports: 4
//   base scalars, the six percent knobs' modulators (6 bytes each, BaseId
//   order), the two dwells (u16 little-endian, 0.01 strokes, RFC-095), then
//   the dwells' two modulators. A payload stored before RFC-095 is the first
//   kLegacyPresetPayloadBytes of this one and the store zero-extends it,
//   which reads as no dwell and no dwell modulation. Each modulator's first
//   byte is its amount, 0 = no modulation (RFC-066); the retired 100 = off
//   bytes migrate through migrateRetiredAmount() and are never read as they
//   stand.
// See: PatternEngine.h, ValencePattern.h, Valence SPEC.md §8.7, §11.3,
// RENDERING.md §10.1

#include <array>
#include <cstddef>
#include <cstdint>

#include "advanced/AdvancedPattern.h"

namespace valence {

// The stroke frame, in millimeters: the machine-driven window every stroke is
// mapped into, and the input ceilings its speed knob, its brake and every
// half-stroke's least time are scaled by.
// Filled by the delegate from the 0x1000 config it owns (C-1).
struct PatternFrame {
    float win_min     = 0.0f;
    float win_max     = 0.0f;
    float input_speed = 0.0f;   // mm/s
    float input_accel = 0.0f;   // mm/s2
    float input_jerk  = 0.0f;   // mm/s3; 0 bounds nothing

    bool operator==(const PatternFrame&) const = default;
};

struct PatternSettings {
    // The seven core StrokeEngine patterns, index-aligned with the catalog's
    // pattern-state option labels.
    static constexpr uint8_t kPatternCount = 7;
    static constexpr size_t  kLegacyPresetPayloadBytes = 4 + 6 * advpat::PERCENT_BASE_COUNT;
    static constexpr size_t  kPresetDwellAt = kLegacyPresetPayloadBytes;
    static constexpr size_t  kPresetPayloadBytes = 4 + 6 * advpat::BASE_COUNT + 2 * 2;
    using PresetPayload = std::array<uint8_t, kPresetPayloadBytes>;

    // Where BaseId `id`'s modulator sits in a payload.
    static constexpr size_t presetModAt(uint8_t id) {
        return id < advpat::PERCENT_BASE_COUNT
                   ? 4 + size_t(id) * 6
                   : kPresetDwellAt + 4 + size_t(id - advpat::PERCENT_BASE_COUNT) * 6;
    }

    // Defaults are the catalog's `default` annotations: nothing moves until an
    // operator dials in a speed, a depth and a stroke.
    bool    running        = false;   // the classic generator's run/stop
    uint8_t pattern        = 0;
    float   speed          = 0.0f;    // 0..100 % of the input speed ceiling
    float   depth          = 0.0f;    // 0..100 % into the window
    float   stroke         = 0.0f;    // 0..100 % of the depth
    float   sensation      = 50.0f;   // 0..100, 50 neutral
    bool    background_run = false;   // registry source.background_run, both generators
    // The advanced generator's run/stop, independent of `running`: the two
    // are separate sources and the arbiter never lets both hold the rail
    // (RFC-093).
    bool    adv_running    = false;
    advpat::Settings ap{};
    PatternFrame frame{};

    void setPattern(int idx) {
        if (idx < 0) idx = 0;
        if (idx >= kPatternCount) idx = kPatternCount - 1;
        pattern = uint8_t(idx);
    }
    static float percent(float v) { return v < 0.0f ? 0.0f : (v > 100.0f ? 100.0f : v); }

    // SPEC §11.3 / RENDERING §10.1: the session that owned a generator is
    // gone. `run` is that generator's flag, `running` or `adv_running`.
    // background_run false (the default) stops it; true keeps it running
    // unattended, stoppable by e-stop. Returns true when it stopped one.
    bool ownerReleased(bool& run) const {
        if (!run || background_run) return false;
        run = false;
        return true;
    }

    // The live advanced set a preset captures: speeds, accels, dwells, every
    // modulator. Never the depths or master speed -- a preset changes stroke
    // character, never the operator's window or throttle.
    PresetPayload capturePreset() const {
        PresetPayload p{};
        p[0] = uint8_t(ap.in_speed.value);
        p[1] = uint8_t(ap.out_speed.value);
        p[2] = uint8_t(ap.in_accel.value);
        p[3] = uint8_t(ap.out_accel.value);
        p[kPresetDwellAt + 0] = uint8_t(ap.dwell_crest.value & 0xFFu);
        p[kPresetDwellAt + 1] = uint8_t(ap.dwell_crest.value >> 8);
        p[kPresetDwellAt + 2] = uint8_t(ap.dwell_trough.value & 0xFFu);
        p[kPresetDwellAt + 3] = uint8_t(ap.dwell_trough.value >> 8);
        for (uint8_t id = 0; id < advpat::BASE_COUNT; ++id) {
            const advpat::Modifier& m = ap.byId(id)->modifier;
            const size_t b = presetModAt(id);
            p[b + 0] = m.amount;
            p[b + 1] = m.in_step;
            p[b + 2] = m.in_wait;
            p[b + 3] = m.out_step;
            p[b + 4] = m.out_wait;
            p[b + 5] = m.offset;
        }
        return p;
    }

    // The inverse, through the same clamps an intent takes. It starts nothing:
    // the advanced generator's run flag is its writer's alone.
    void applyPreset(const PresetPayload& p) {
        ap.setBase(advpat::SPEED_IN, p[0]);
        ap.setBase(advpat::SPEED_OUT, p[1]);
        ap.setBase(advpat::ACCEL_IN, p[2]);
        ap.setBase(advpat::ACCEL_OUT, p[3]);
        ap.setBase(advpat::DWELL_CREST, p[kPresetDwellAt + 0] | (p[kPresetDwellAt + 1] << 8));
        ap.setBase(advpat::DWELL_TROUGH, p[kPresetDwellAt + 2] | (p[kPresetDwellAt + 3] << 8));
        for (uint8_t id = 0; id < advpat::BASE_COUNT; ++id) {
            const size_t b = presetModAt(id);
            ap.byId(id)->modifier.set(p[b + 0], p[b + 1], p[b + 2], p[b + 3], p[b + 4], p[b + 5]);
        }
    }

    // A payload written under the retired 100 = off amount, re-expressed in
    // RFC-066's 0 = no modulation so the same preset strokes the same:
    // amount = 100 - stored. A stored byte over 100 loaded as 100 (off) under
    // the old clamp, so it migrates to 0. Only the six modulators such a
    // payload had: the zero-extended dwell modulators already mean none.
    static void migrateRetiredAmount(PresetPayload& p) {
        for (uint8_t id = 0; id < advpat::PERCENT_BASE_COUNT; ++id) {
            uint8_t& a = p[presetModAt(id)];
            a = uint8_t(100 - (a > 100 ? 100 : a));
        }
    }

    bool operator==(const PatternSettings&) const = default;
};

}  // namespace valence
