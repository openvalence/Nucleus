// AdvancedPattern -- fray-d's modifier and half-stroke math, float throughout
// Constraints:
// - Pure: see AdvancedPattern.h. Integer steps are promoted to float where the
//   original u8 arithmetic truncated; behavior is otherwise the original's.
// See: AdvancedPattern.h

#include "AdvancedPattern.h"

#include <cmath>

namespace advpat {

namespace {
int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

// ---- modifier ---------------------------------------------------------------

float Modifier::modification(int cycle) const {
    const float ratio = float(100 - amplitude) / 100.0f;
    if (cycle < 0) return 1.0f - ratio;
    const int steps = stepCount();
    if (steps > 0) {
        cycle = (cycle + offset) % steps;
        if (cycle < in_step) return 1.0f - ratio / float(in_step) * float(cycle + 1);
        cycle -= in_step;
        if (cycle < in_wait) return 1.0f - ratio;
        cycle -= in_wait;
        if (cycle < out_step) return 1.0f - ratio + ratio / float(out_step) * float(cycle + 1);
    }
    return 1.0f;
}

void Modifier::set(int amp, int is, int iw, int os, int ow, int off) {
    amplitude = uint8_t(clampInt(amp, 0, 100));
    in_step   = uint8_t(clampInt(is, 1, 25));
    in_wait   = uint8_t(clampInt(iw, 0, 25));
    out_step  = uint8_t(clampInt(os, 1, 25));
    out_wait  = uint8_t(clampInt(ow, 0, 25));
    offset    = uint8_t(clampInt(off, 0, 100));
}

// ---- base control -----------------------------------------------------------

float BaseControl::modifiedValue(int stroke_count) const {
    if (!modifier.active()) return float(value);
    const float difference = float(value) - float(invert_ref ? max_value : min_value);
    const int steps = modifier.stepCount();
    // Two half-strokes (one in, one out) advance the lane by one step.
    const int cycle = stroke_count < 0 ? -1 : (stroke_count / 2) % steps;
    return float(value) - difference * (1.0f - modifier.modification(cycle));
}

// An ease curve, pow(1 - pow(1 - x, e), 1/e): gentle at the bottom of the knob,
// resolution at the top.
float BaseControl::rampedModified(float curve_exp, int stroke_count) const {
    const float x = normalizedModified(stroke_count);
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return 1.0f;
    return std::pow(1.0f - std::pow(1.0f - x, curve_exp), 1.0f / curve_exp);
}

// ---- settings ---------------------------------------------------------------

BaseControl* Settings::byId(uint8_t id) {
    switch (id) {
        case DEPTH_MAX: return &max_depth;
        case DEPTH_MIN: return &min_depth;
        case SPEED_IN:  return &in_speed;
        case SPEED_OUT: return &out_speed;
        case ACCEL_IN:  return &in_accel;
        case ACCEL_OUT: return &out_accel;
        default:        return nullptr;
    }
}

const BaseControl* Settings::byId(uint8_t id) const {
    return const_cast<Settings*>(this)->byId(id);
}

void Settings::setBase(uint8_t id, int v) {
    BaseControl* c = byId(id);
    if (c == nullptr) return;
    c->set(v);
    if (id == DEPTH_MAX || id == DEPTH_MIN) coupleDepths();
}

StrokePlan Settings::planStroke(uint32_t stroke_count) const {
    StrokePlan p{};
    const float master_frac = float(master.value) / 100.0f;
    if (master_frac <= 0.0f) return p;
    const float master_ramp =
        std::pow(1.0f - std::pow(1.0f - master_frac, SPEED_CURVE_EXP), 1.0f / SPEED_CURVE_EXP);

    const int  sc = int(stroke_count);
    const bool in_stroke = (stroke_count % 2u) == 0u;
    p.moving = true;
    if (in_stroke) {
        p.target_frac = max_depth.normalizedModified(sc);
        p.speed_frac  = master_ramp * in_speed.normalizedModified(sc);
        p.accel_knob  = in_accel.rampedModified(ACCEL_CURVE_EXP, sc);
    } else {
        p.target_frac = min_depth.normalizedModified(sc);
        p.speed_frac  = master_ramp * out_speed.normalizedModified(sc);
        p.accel_knob  = out_accel.rampedModified(ACCEL_CURVE_EXP, sc);
    }
    if (p.target_frac < 0.0f) p.target_frac = 0.0f;
    if (p.target_frac > 1.0f) p.target_frac = 1.0f;
    return p;
}

}  // namespace advpat
