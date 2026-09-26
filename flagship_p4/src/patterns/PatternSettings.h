#pragma once

// PatternSettings -- every knob the pattern generator runs on, as one value:
// the classic pattern set, the advanced set, the run and background_run
// policy, and the stroke frame they map into
// Constraints:
// - HARDWARE-FREE value type. The hub delegate OWNS the live copy (it is the
//   one writer: intents, presets, e-stop, source release) and hands WHOLE
//   copies to the generator's task through ValencePattern.h. The generator
//   never writes one back; nothing here is shared memory.
// - Every setter clamps to the catalog's own bounds (ValenceCatalog.h,
//   pattern-state / pattern-advanced / pattern-adv-mod-*), so the ECHO a
//   delegate builds from the fields afterward is the post-clamp truth.
// - The preset payload layout is wire-visible through BLOB_REQ exports and is
//   the archive's: 4 base scalars then 6 lanes of 6 bytes, BaseId order.
// See: PatternEngine.h, ValencePattern.h, Valence SPEC.md §8.7, §11.3,
// RENDERING.md §10.1

#include <array>
#include <cstddef>
#include <cstdint>

#include "AdvancedPattern.h"

namespace valence {

// The stroke frame, in millimeters: the machine-driven window every stroke is
// mapped into, and the input ceilings its speed knob and brake are scaled by.
// Filled by the delegate from the 0x1000 config it owns (C-1).
struct PatternFrame {
    float win_min     = 0.0f;
    float win_max     = 0.0f;
    float input_speed = 0.0f;   // mm/s
    float input_accel = 0.0f;   // mm/s2

    bool operator==(const PatternFrame&) const = default;
};

struct PatternSettings {
    // The seven core StrokeEngine patterns, index-aligned with the catalog's
    // pattern-state option labels.
    static constexpr uint8_t kPatternCount = 7;
    static constexpr size_t  kPresetPayloadBytes = 4 + 6 * advpat::BASE_COUNT;
    using PresetPayload = std::array<uint8_t, kPresetPayloadBytes>;

    // Defaults are the catalog's `default` annotations: nothing moves until an
    // operator dials in a speed, a depth and a stroke.
    bool    running        = false;
    uint8_t pattern        = 0;
    float   speed          = 0.0f;    // 0..100 % of the input speed ceiling
    float   depth          = 0.0f;    // 0..100 % into the window
    float   stroke         = 0.0f;    // 0..100 % of the depth
    float   sensation      = 50.0f;   // 0..100, 50 neutral
    bool    background_run = false;   // registry source.background_run
    bool    ap_mode        = false;   // the advanced generator instead of the classic set
    advpat::Settings ap{};
    PatternFrame frame{};

    void setPattern(int idx) {
        if (idx < 0) idx = 0;
        if (idx >= kPatternCount) idx = kPatternCount - 1;
        pattern = uint8_t(idx);
    }
    static float percent(float v) { return v < 0.0f ? 0.0f : (v > 100.0f ? 100.0f : v); }

    // SPEC §11.3 / RENDERING §10.1: the session that owned this source is gone.
    // background_run false (the default) stops the generator; true keeps it
    // running unattended, stoppable by e-stop. Returns true when it stopped a
    // running generator.
    bool ownerReleased() {
        if (!running || background_run) return false;
        running = false;
        return true;
    }

    // The live advanced lane set a preset captures: speeds, accels, six lanes.
    // Never the depths or master speed -- a preset changes stroke character,
    // never the operator's window or throttle.
    PresetPayload capturePreset() const {
        PresetPayload p{};
        p[0] = ap.in_speed.value;
        p[1] = ap.out_speed.value;
        p[2] = ap.in_accel.value;
        p[3] = ap.out_accel.value;
        for (uint8_t id = 0; id < advpat::BASE_COUNT; ++id) {
            const advpat::Modifier& m = ap.byId(id)->modifier;
            const size_t b = 4 + size_t(id) * 6;
            p[b + 0] = m.amplitude;
            p[b + 1] = m.in_step;
            p[b + 2] = m.in_wait;
            p[b + 3] = m.out_step;
            p[b + 4] = m.out_wait;
            p[b + 5] = m.offset;
        }
        return p;
    }

    // The inverse, through the same clamps an intent takes, and it engages the
    // advanced generator: a loaded preset is a pattern being asked for.
    void applyPreset(const PresetPayload& p) {
        ap.setBase(advpat::SPEED_IN, p[0]);
        ap.setBase(advpat::SPEED_OUT, p[1]);
        ap.setBase(advpat::ACCEL_IN, p[2]);
        ap.setBase(advpat::ACCEL_OUT, p[3]);
        for (uint8_t id = 0; id < advpat::BASE_COUNT; ++id) {
            const size_t b = 4 + size_t(id) * 6;
            ap.byId(id)->modifier.set(p[b + 0], p[b + 1], p[b + 2], p[b + 3], p[b + 4], p[b + 5]);
        }
        ap_mode = true;
    }

    bool operator==(const PatternSettings&) const = default;
};

}  // namespace valence
