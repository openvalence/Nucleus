#pragma once

// AdvancedPattern -- the fray-d Advanced Penetration stroke math: eight base
// controls, one cyclic modulator per control, one plan per half-stroke
// Constraints:
// - HARDWARE-FREE and pure: no clock, no task, no motion call. The pattern
//   engine owns an instance, turns a StrokePlan into a MotionIntent, and hands
//   it to the arbiter's door like every other input source.
// - Plain values, no volatile: a Settings is OWNED by one task and crosses
//   tasks only as a copy (ValencePattern.h).
// - BaseId order is the order of the pattern-advanced-cmd keys
//   (ValenceCatalog.h apBaseKey / apModKeyBase) and of the preset payload;
//   append, never reorder.
// - LINEAR KNOBS (operator ruling 2026-10-03): speed = master % x half % x
//   the input ceiling, accel knob = accel %. Do not ease them: a client
//   predicts the stroke from the percent (the advgen.master registry note,
//   Phosphor's master x k / half / k link).
// See: https://github.com/fray-d/OSSM-Lite (CERN-OHL-S v2, the algorithm's
// origin), ValenceCatalog.h (pattern-advanced, pattern-adv-mod-*)

#include <cstdint>

namespace advpat {

enum BaseId : uint8_t {
    DEPTH_MAX  = 0,   // in-stroke target ("Depth 1")
    DEPTH_MIN  = 1,   // out-stroke target ("Depth 2")
    SPEED_IN   = 2,
    SPEED_OUT  = 3,
    ACCEL_IN   = 4,
    ACCEL_OUT  = 5,
    // RFC-095: the hold at each bound, value in 0.01 strokes.
    DWELL_CREST  = 6,   // at max_depth, after the in-stroke
    DWELL_TROUGH = 7,   // at min_depth, after the out-stroke
    BASE_COUNT = 8
};

// BaseId 0..5 are 0..100 percent knobs; the dwells follow them.
constexpr uint8_t PERCENT_BASE_COUNT = 6;

// ---- modifier ---------------------------------------------------------------

// One modulator (RFC-066): in_step strokes rising toward the full swing,
// in_wait held, out_step falling back, out_wait at rest. amount 0 is no
// modulation, 100 the full swing.
struct Modifier {
    uint8_t amount    = 0;    // 0..100
    uint8_t in_step   = 1;    // 1..25
    uint8_t in_wait   = 0;    // 0..25
    uint8_t out_step  = 1;    // 1..25
    uint8_t out_wait  = 0;    // 0..25
    uint8_t offset    = 0;    // 0..100

    uint8_t stepCount() const { return uint8_t(in_step + in_wait + out_step + out_wait); }
    bool    active() const { return amount > 0 && stepCount() > 0; }

    // 0..1 multiplier on the control's swing for a cycle index.
    float modification(int cycle) const;

    // Every field clamped to its catalog bound.
    void set(int amount, int in_step, int in_wait, int out_step, int out_wait, int offset);

    bool operator==(const Modifier&) const = default;
};

// ---- base control -----------------------------------------------------------

// A knob with a modulator: 0..100 percent, or 0..65535 hundredths of a stroke
// for a dwell. invert_ref marks the control whose modulator swings toward its
// MAX bound (DEPTH_MIN pulls toward max depth).
struct BaseControl {
    uint16_t value;
    uint16_t min_value;   // dynamic for the depth pair (coupled)
    uint16_t max_value;
    bool     invert_ref;
    Modifier modifier{};

    void set(int v) {
        if (v < int(min_value)) v = min_value;
        if (v > int(max_value)) v = max_value;
        value = uint16_t(v);
    }

    float modifiedValue(int stroke_count) const;
    float normalizedModified(int stroke_count) const { return modifiedValue(stroke_count) / 100.0f; }

    bool operator==(const BaseControl&) const = default;
};

// ---- stroke plan ------------------------------------------------------------

// What one half-stroke asks of the machine, in unitless fractions.
struct StrokePlan {
    bool  moving      = false;  // false: master speed is 0, hold position
    float target_frac = 0.0f;   // 0..1 within the stroke window
    float speed_frac  = 0.0f;   // 0..1 of the input speed ceiling
    float accel_knob  = 0.0f;   // 0..1: accel = minAccel * (1 + 9 * knob)
    // RFC-095: the hold at this half's bound once it lands, in strokes (one
    // stroke is one in-half plus one out-half). Additive: the halves keep
    // their own speed and acceleration.
    float dwell_strokes = 0.0f;
};

// ---- settings ---------------------------------------------------------------

// The whole advanced control set. Defaults are the catalog's `default`
// annotations: shallow 10 % max depth and master speed 0, so a fresh boot
// cannot lunge.
struct Settings {
    BaseControl master    {0,   0, 100, false};  // no modulator
    BaseControl max_depth {10,  0, 100, false};
    BaseControl min_depth {0,   0, 100, true};
    BaseControl in_speed  {100, 1, 100, false};
    BaseControl out_speed {100, 1, 100, false};
    BaseControl in_accel  {40,  0, 100, false};
    BaseControl out_accel {40,  0, 100, false};
    BaseControl dwell_crest  {0, 0, 65535, false};
    BaseControl dwell_trough {0, 0, 65535, false};

    BaseControl*       byId(uint8_t id);
    const BaseControl* byId(uint8_t id) const;

    // Clamped write of one base control. The depth pair re-couples after
    // every write, so min can never cross max.
    void setBase(uint8_t id, int v);

    void coupleDepths() {
        max_depth.min_value = min_depth.value;
        min_depth.max_value = max_depth.value;
    }

    // Even stroke_count is the in-stroke (toward max_depth), odd the out.
    StrokePlan planStroke(uint32_t stroke_count) const;

    bool operator==(const Settings&) const = default;
};

}  // namespace advpat
