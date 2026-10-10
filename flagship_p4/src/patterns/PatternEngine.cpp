// PatternEngine -- half-stroke scheduling and the brake, shared by both
// generators, and each generator's own pattern-to-millimeter mapping
// Constraints:
// - HARDWARE-FREE (PatternEngine.h). Compiled verbatim by the board, the host
//   twin and test_pattern_engine.
// - A half-stroke is a WAVEFORM intent: a target, the duration the pattern's
//   own speed and accel imply, and zero end velocity (a stroke reverses AT
//   rest). The arbiter plans it at arrival under the INPUT ceilings, and
//   tick() never sends one shorter than the planner renders under them
//   (fitSeconds()): a stroke is slowed to fit, never trimmed (bd val-hnq).
// See: PatternEngine.h, lib/strokeengine_patterns/VENDORED.md

#include "PatternEngine.h"

#include <algorithm>
#include <cmath>

#include "kinetic2/profile.hpp"

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

// One hold segment's ceiling, the half-stroke's for the same reason; a longer
// dwell is sent as consecutive segments.
constexpr uint64_t kMaxHoldUs = uint64_t(kMaxStrokeS * 1e6f);

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

uint32_t strokeUs(float seconds) { return uint32_t(clampf(seconds, kMinStrokeS, kMaxStrokeS) * 1e6f); }

// The planner renders a rest-to-rest half-stroke as a Bezier piece, slower
// than the time-optimal profile under the same ceilings: up to 1.12 of it
// across test_motion_arbiter's narrow-window sweep. This keeps a margin.
constexpr float kRenderSlack = 1.2f;

// The least time of a d mm half-stroke the planner renders whole under the
// frame's input ceilings: Kinetic's fastest rest-to-rest move (Profile::point),
// widened by kRenderSlack. Anything shorter keeps its deadline and is trimmed
// toward its start. 0 while any ceiling is unset.
float fitSeconds(float d, const PatternFrame& f) {
    if (!(f.input_speed > 0.0f && f.input_accel > 0.0f && f.input_jerk > 0.0f)) return 0.0f;
    float t = 0.0f;
    (void)kinetic2::Profile::point(kinetic2::State{0.0f, 0.0f, 0.0f}, d, 0,
                                   kinetic2::Limits{f.input_speed, f.input_accel, f.input_jerk}, 0.0f, &t);
    return t * kRenderSlack;
}

}  // namespace

// ---- settings ---------------------------------------------------------------

void PatternEngine::apply(const PatternSettings& s) {
    const bool start = running(s) && !running(_s);
    _s = s;
    if (start) {
        _stroke_index = 0;
        _have_prev = false;
        _holding = false;
        _hold_owed_us = 0;
    }
}

// ---- the gate ---------------------------------------------------------------

bool PatternEngine::wantsMotion(const PatternInputs& in) const {
    // The arbiter refuses a generator intent on each of these too; gating here
    // keeps the generator from spending strokes into a refusal, and yielding to
    // a live stream keeps two machine-driven sources from interleaving plans.
    if (!running(_s) || !in.homed || in.estop || in.paused || in.stream_active) return false;
    if (!(_s.frame.win_max > _s.frame.win_min) || !(_s.frame.input_speed > 0.0f)) return false;
    return moves(_s);
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
        // Only PAUSE keeps a dwell, and only one already being held: a half
        // cut short never reached its bound, and every other gate ends the run
        // as the operator would read it.
        const bool pause_only =
            in.paused && running(_s) && in.homed && !in.estop && !in.stream_active;
        if (!pause_only) {
            _hold_owed_us = 0;
        } else if (_active) {
            _hold_owed_us = (_holding && now_us < _due_us) ? _hold_owed_us + (_due_us - now_us) : 0;
        }
        _active = false;
        _holding = false;
        _stroke_end_us = 0;
        return out;
    }
    if (!_active) {
        _active = true;
        _have_prev = false;   // resuming: plan the first stroke from where the carriage IS
        _due_us = now_us;
    }
    if (now_us < _due_us) return std::nullopt;

    if (_hold_owed_us > 0) {
        // At the bound just reached, or where the carriage IS after a pause.
        const uint64_t h = _hold_owed_us < kMaxHoldUs ? _hold_owed_us : kMaxHoldUs;
        _hold_owed_us -= h;
        const float at = fromMm(in);
        _holding = true;
        _due_us = now_us + h;
        _stroke_end_us = _due_us;
        _have_prev = true;
        _prev_target_mm = at;
        MotionIntent it;
        it.source       = _source;
        it.target_mm    = at;
        it.duration_us  = uint32_t(h);
        it.has_end_vel  = true;
        it.end_vel_mm_s = 0.0f;
        return it;
    }
    _holding = false;

    const std::optional<Stroke> st = next(now_us, in);
    if (!st) {
        _due_us = now_us + kPollUs;
        return std::nullopt;
    }
    ++_stroke_index;
    // Both generators time a stroke by their own knobs; neither may ask for
    // one the input ceilings cannot render whole.
    const uint32_t duration_us =
        st->moves ? std::max(st->duration_us, strokeUs(fitSeconds(std::fabs(st->target_mm - fromMm(in)), _s.frame)))
                  : st->duration_us;
    _due_us = now_us + duration_us;
    _stroke_end_us = _due_us;
    _have_prev = true;
    _prev_target_mm = st->target_mm;
    _hold_owed_us = st->moves ? st->hold_us : 0;
    if (!st->moves) return std::nullopt;

    MotionIntent it;
    it.source       = _source;
    it.target_mm    = st->target_mm;
    it.duration_us  = duration_us;
    it.has_end_vel  = true;
    it.end_vel_mm_s = 0.0f;
    return it;
}

// ---- classic patterns -------------------------------------------------------

Pattern& ClassicGenerator::patternAt(uint8_t idx) {
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

std::optional<PatternEngine::Stroke> ClassicGenerator::next(uint64_t now_us, const PatternInputs& in) {
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

// Counts strokes, never time: the clock argument is unused.
std::optional<PatternEngine::Stroke> AdvancedGenerator::next(uint64_t, const PatternInputs& in) {
    if (_stroke_index == 0) _prev_half_us = 0;
    const PatternFrame& f = _s.frame;
    const advpat::StrokePlan sp = _s.ap.planStroke(_stroke_index);
    const float target = f.win_min + sp.target_frac * (f.win_max - f.win_min);
    const float d = std::fabs(target - fromMm(in));
    if (!sp.moving || d < kMinTravelMm) {
        _prev_half_us = 0;
        return Stroke{false, target, kRestUs};
    }

    const uint32_t half_us = strokeUs(advpat::halfStrokeSeconds(sp, f.input_speed, d));

    // RFC-095: the dwell's clock is one stroke, this half plus the one before
    // it; the first half of a run has no partner and counts twice.
    const uint64_t stroke_us = uint64_t(half_us) + (_prev_half_us != 0 ? _prev_half_us : half_us);
    _prev_half_us = half_us;
    const uint64_t hold_us = uint64_t(sp.dwell_strokes * float(stroke_us) + 0.5f);
    return Stroke{true, target, half_us, hold_us};
}

// ---- the brake --------------------------------------------------------------

// A point intent at the carriage's braking point under the input accel: the
// arbiter plans it at arrival as a stop from the live (p, v). Stamped with this
// generator's source, so it lands while the rail is still free or still its
// own, and is refused once the other generator holds it (RFC-093).
MotionIntent PatternEngine::brake(const PatternInputs& in) const {
    const PatternFrame& f = _s.frame;
    const float a = f.input_accel > 0.0f ? f.input_accel : 1.0f;
    const float v = in.velocity_mm_s;
    MotionIntent it;
    it.source    = _source;
    it.target_mm = clampf(in.position_mm + v * std::fabs(v) / (2.0f * a), f.win_min, f.win_max);
    return it;
}

}  // namespace valence
