// PatternEngine -- half-stroke scheduling, the pattern-to-millimeter mapping,
// and the brake
// Constraints:
// - HARDWARE-FREE (PatternEngine.h). Compiled verbatim by the board, the host
//   twin and test_pattern_engine.
// - A half-stroke is a WAVEFORM intent: a target, the duration the pattern's
//   own speed and accel imply, and zero end velocity (a stroke reverses AT
//   rest). The arbiter plans it at arrival under the INPUT ceilings; an
//   infeasible one is the engine's infeasible policy to resolve, never ours.
// See: PatternEngine.h, lib/strokeengine_patterns/VENDORED.md

#include "PatternEngine.h"

#include <cmath>

namespace valence {
namespace {

// The vendored patterns count in abstract steps; one full window is this many.
constexpr int kAbstractSteps = 10000;

// A half-stroke shorter than this is not travel worth planning: the pattern's
// rhythm is kept, nothing is sent.
constexpr float kMinTravelMm = 0.25f;

// How long a pattern that has nothing to send right now waits before asking
// again: a StopNGo pause is polled at this rate, a zero-travel stroke rests
// at least this long.
constexpr uint32_t kPollUs = 10000;
constexpr uint32_t kRestUs = 50000;

// Bounds on one half-stroke's duration. The floor keeps a degenerate knob set
// from flooding the intent queue; the ceiling keeps a crawl at 1 % speed a
// finite plan.
constexpr float kMinStrokeS = 0.005f;
constexpr float kMaxStrokeS = 60.0f;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

uint32_t strokeUs(float seconds) { return uint32_t(clampf(seconds, kMinStrokeS, kMaxStrokeS) * 1e6f); }

}  // namespace

// ---- settings ---------------------------------------------------------------

void PatternEngine::apply(const PatternSettings& s) {
    const bool start = s.running && !_s.running;
    _s = s;
    if (start) {
        _stroke_index = 0;
        _have_prev = false;
    }
}

// ---- the gate ---------------------------------------------------------------

bool PatternEngine::wantsMotion(const PatternInputs& in) const {
    // The arbiter refuses a Pattern intent on each of these too; gating here
    // keeps the generator from spending strokes into a refusal, and yielding to
    // a live stream keeps two machine-driven sources from interleaving plans.
    if (!_s.running || !in.homed || in.estop || in.paused || in.stream_active) return false;
    if (!(_s.frame.win_max > _s.frame.win_min) || !(_s.frame.input_speed > 0.0f)) return false;
    if (_s.ap_mode) return _s.ap.master.value > 0;
    return _s.speed > 0.0f && _s.stroke > 0.0f;
}

// ---- the tick ---------------------------------------------------------------

std::optional<MotionIntent> PatternEngine::tick(uint64_t now_us, const PatternInputs& in) {
    if (!wantsMotion(in)) {
        // Stopping mid-stroke brakes: a generator that reads stopped while its
        // last half-stroke runs on for seconds at a slow knob is a machine
        // moving under a UI that says it is not. A live stream already
        // superseded the plan, and the arbiter owns the e-stop park.
        std::optional<MotionIntent> out;
        const bool mid_stroke = _active && now_us < _stroke_end_us;
        if (mid_stroke && in.homed && !in.estop && !in.paused && !in.stream_active) out = brake(in);
        _active = false;
        _stroke_end_us = 0;
        return out;
    }
    if (!_active) {
        _active = true;
        _have_prev = false;   // resuming: plan the first stroke from where the carriage IS
        _due_us = now_us;
    }
    if (now_us < _due_us) return std::nullopt;

    const std::optional<Stroke> st = _s.ap_mode ? nextAdvanced(in) : nextClassic(now_us, in);
    if (!st) {
        _due_us = now_us + kPollUs;
        return std::nullopt;
    }
    ++_stroke_index;
    _due_us = now_us + st->duration_us;
    _stroke_end_us = _due_us;
    _have_prev = true;
    _prev_target_mm = st->target_mm;
    if (!st->moves) return std::nullopt;

    MotionIntent it;
    it.source       = MotionSource::Pattern;
    it.target_mm    = st->target_mm;
    it.duration_us  = st->duration_us;
    it.has_end_vel  = true;
    it.end_vel_mm_s = 0.0f;
    return it;
}

// ---- classic patterns -------------------------------------------------------

Pattern& PatternEngine::patternAt(uint8_t idx) {
    switch (idx) {
        case 1:  return _teasing;
        case 2:  return _robo;
        case 3:  return _half;
        case 4:  return _deeper;
        case 5:  return _stopngo;
        case 6:  return _insist;
        default: return _simple;
    }
}

std::optional<PatternEngine::Stroke> PatternEngine::nextClassic(uint64_t now_us,
                                                                const PatternInputs& in) {
    const PatternFrame& f = _s.frame;
    const float span = f.win_max - f.win_min;
    const int stroke_steps = int(_s.stroke / 100.0f * float(kAbstractSteps));
    const int depth_steps  = int(_s.depth / 100.0f * float(kAbstractSteps));
    const float max_sps = f.input_speed * float(kAbstractSteps) / span;

    // The speed knob is a fraction of the INPUT ceiling as actual peak carriage
    // speed. The trapezoid family cruises at 1.5 x stroke / half-time, so a
    // full stroke at that peak takes 3 x stroke / peak.
    const float peak = _s.speed / 100.0f * max_sps;
    const float time_of_stroke = peak > 1.0f ? 3.0f * float(stroke_steps) / peak : 0.0f;
    if (!(time_of_stroke > 0.0f)) return Stroke{false, fromMm(in), kRestUs};

    Pattern& p = patternAt(_s.pattern);
    arduino_compat::g_now_ms = uint32_t(now_us / 1000u);
    // Order is the archive's and it matters: Insist re-derives its timing in
    // setStroke() and setSensation() from the time set first.
    p.setTimeOfStroke(time_of_stroke);
    p.setStroke(stroke_steps);
    p.setDepth(depth_steps);
    p.setSensation((_s.sensation - 50.0f) * 2.0f);
    p.setSpeedLimit(unsigned(max_sps), unsigned(max_sps), 1);
    const motionParameter mp = p.nextTarget(_stroke_index);
    if (mp.skip) return std::nullopt;

    const float mm_per_step = span / float(kAbstractSteps);
    const float target = clampf(f.win_min + float(mp.stroke) * mm_per_step, f.win_min, f.win_max);
    const float v = float(mp.speed) * mm_per_step;
    const float a = float(mp.acceleration) * mm_per_step;
    const float d = std::fabs(target - fromMm(in));

    // This half-stroke's OWN duration (cruise plus one ramp), so an asymmetric
    // pattern keeps its asymmetry; a uniform half-time would erase sensation.
    float t = (v > 1.0f && d > 0.05f) ? d / v + (a > 1.0f ? v / a : 0.0f) : time_of_stroke * 0.5f;
    // Past the input ceiling, SLOW the stroke rather than let the planner
    // shrink it: the knob means speed, and the stroke is the operator's.
    if (v > f.input_speed) t *= v / f.input_speed;
    return Stroke{d >= kMinTravelMm, target, strokeUs(t)};
}

// ---- the advanced generator -------------------------------------------------

std::optional<PatternEngine::Stroke> PatternEngine::nextAdvanced(const PatternInputs& in) {
    const PatternFrame& f = _s.frame;
    const advpat::StrokePlan sp = _s.ap.planStroke(_stroke_index);
    const float target = f.win_min + sp.target_frac * (f.win_max - f.win_min);
    const float d = std::fabs(target - fromMm(in));
    if (!sp.moving || d < kMinTravelMm) return Stroke{false, target, kRestUs};

    // fray-d: cruise at speed_frac of the input ceiling; accel spans 1x..10x
    // the least that reaches that speed over this distance (1x a triangle,
    // 10x nearly all cruise). The duration is that trapezoid's own.
    float v = sp.speed_frac * f.input_speed;
    if (v < 1.0f) v = 1.0f;
    const float a = (v * v / d) * (1.0f + 9.0f * sp.accel_knob);
    return Stroke{true, target, strokeUs(d / v + v / a)};
}

// ---- the brake --------------------------------------------------------------

// A point intent at the carriage's braking point under the input accel: the
// arbiter plans it at arrival as a stop from the live (p, v).
MotionIntent PatternEngine::brake(const PatternInputs& in) const {
    const PatternFrame& f = _s.frame;
    const float a = f.input_accel > 0.0f ? f.input_accel : 1.0f;
    const float v = in.velocity_mm_s;
    MotionIntent it;
    it.source    = MotionSource::Pattern;
    it.target_mm = clampf(in.position_mm + v * std::fabs(v) / (2.0f * a), f.win_min, f.win_max);
    return it;
}

}  // namespace valence
