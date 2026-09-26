// MotionArbiter -- the gates, the clamp, the limit sets and the feedforward,
// compiled by the board and by the host twin alike
// Constraints:
// - HARDWARE-FREE (MotionArbiter.h). Geiger is the log path and binds its own
//   platform layer; on a host it is mute, never a second log path.
// - The plan is computed AT INTENT ARRIVAL from the actual (p, v, a). The tick
//   only EVALUATES it. Nothing here plans on a clock.
// See: MotionArbiter.h, .claude/rules/motion-control.md, bd val-091.4

#include "MotionArbiter.h"

#include <cmath>

#include "geiger/geiger.h"

namespace valence {
namespace {

constexpr const char* kTag = "motion";

// Residual tracking. The emitter renders a VELOCITY, so a step it could not
// emit on time is a step nothing else repays: the residual against position
// truth is closed here, and only once it exceeds one whole step so the
// quantization of the count itself never becomes commanded jitter.
constexpr float kTrackHz = 50.0f;

// A rendered direction reversal has to clear this much travel before it counts
// as a stroke, so dither around a standstill never inflates the odometer.
constexpr float kStrokeMinMm = 1.0f;

}  // namespace

// ---- the steering word ------------------------------------------------------

SteerWord steerWord(float v_mm_s) {
    SteerWord w;
    const float mag = std::fabs(v_mm_s);
    // Written so a NaN parks: !(mag >= park), not (mag < park).
    if (!(mag >= kParkMmS)) return w;
    float cyc = kLpClockHz / (mag * kStepsPerMm);
    if (cyc < static_cast<float>(kMinCyclesPerEdge)) {
        cyc = static_cast<float>(kMinCyclesPerEdge);
        w.floored = true;
    }
    w.step_q8 = static_cast<uint32_t>(cyc * 256.0f + 0.5f);
    w.forward = !(v_mm_s < 0.0f);
    return w;
}

// ---- cross-task -------------------------------------------------------------

void MotionArbiter::estop(bool on) {
    _estop = on;
    if (!on) {
        _estop_settled = false;
        return;
    }
    // Park on the CALLING task. The ENGINE is not touched here: it belongs to
    // the owning task, which resets it on the next tick (see evaluate()).
    _pattern_stopped.store(true);   // clearing the latch never restarts the generator
    _emitter.park();
    _homed = false;      // an abandoned plan leaves the carriage where it fell
    GLOGW(kTag, "ESTOP: emitter parked at %.3f mm", double(positionMm()));
}

void MotionArbiter::stop() {
    _pattern_stopped.store(true);
    _stream_stopped.store(true);
    _brake_req.store(true);
}

void MotionArbiter::setWindow(float lo, float hi, float rail) {
    if (!(std::isfinite(lo) && std::isfinite(hi) && std::isfinite(rail))) return;
    if (!(hi > lo)) return;
    _win_min = lo;
    _win_max = hi;
    _rail    = rail > 0.0f ? rail : DEFAULT_MAX_RAIL_MM;
    _frame_moved = true;   // normalized units now mean different millimeters
}

float MotionArbiter::forceHome(float stroke_mm) {
    // Written so a NaN takes the default: !(x >= 1), not (x < 1).
    float stroke = stroke_mm;
    if (!(stroke >= 1.0f)) stroke = 250.0f;
    if (stroke > _rail) stroke = _rail;
    _origin    = _emitter.count();
    _frame_moved = true;   // 0.0 mm now means a different emitter count
    _rail      = stroke;
    _estop     = false;
    _estop_settled = false;
    _homed     = true;
    // The engine reseeds itself at rest on the next accepted intent; nothing
    // here may call into it, this runs on the hub task.
    GLOGW(kTag, "FORCE HOME: homed asserted at 0.0 mm, stroke %.1f mm, e-stop cleared "
                "-- no homing cycle ran (RFC-025)", double(stroke));
    return stroke;
}

void MotionArbiter::noteStream(uint32_t bundles, uint32_t samples, uint32_t dropped) {
    _sync_bundles.fetch_add(bundles, std::memory_order_relaxed);
    _sync_samples.fetch_add(samples, std::memory_order_relaxed);
    _sync_dropped.fetch_add(dropped, std::memory_order_relaxed);
}

// ---- gates ------------------------------------------------------------------

void MotionArbiter::begin(uint64_t now_us) {
    _emitter.steer(0.0f);            // PARKED until an intent lands
    _origin    = _emitter.count();
    _odo_steps = _origin;
    _engine.resetAt(toNorm(0.0f), now_us);
    _p_cmd_mm = 0.0f;
}

bool MotionArbiter::accept(const MotionIntent& in, uint64_t now_us) {
    // E-stop is the one gate no source bypasses.
    if (_estop) {
        ++_rejected;
        GLOGW_EVERY_MS(1000, kTag, "REJECT: e-stop");
        return false;
    }
    // Manual bypasses the rest (the push-to-home case): an operator must be
    // able to move an unhomed or paused machine, and only to move it.
    if (in.source != MotionSource::Manual) {
        if (!_homed) {
            ++_rejected;
            GLOGW_EVERY_MS(1000, kTag, "REJECT: not homed");
            return false;
        }
        if (_paused) {
            ++_rejected;
            GLOGW_EVERY_MS(1000, kTag, "REJECT: paused");
            return false;
        }
        if (in.source == MotionSource::Pattern && _pattern_stopped.load()) {
            ++_rejected;
            GLOGW_EVERY_MS(1000, kTag, "REJECT: pattern stopped");
            return false;
        }
        if (in.source == MotionSource::Stream && _stream_stopped.load()) {
            ++_rejected;
            GLOGW_EVERY_MS(1000, kTag, "REJECT: stream stopped");
            return false;
        }
    }

    const bool manual = in.source == MotionSource::Manual;

    // Window clamp. Manual reaches the whole asserted rail; a machine-driven
    // source is held inside the configured window, itself held inside the rail.
    float target = in.target_mm;
    const float lo = manual ? 0.0f : (_win_min > 0.0f ? _win_min : 0.0f);
    const float hi_win = _win_max < _rail ? _win_max : _rail;
    const float hi = manual ? _rail : hi_win;
    if (target < lo) target = lo;
    if (target > hi) target = hi;
    // THROTTLED, because a stream that overhangs the window clamps EVERY
    // sample: at 50 Hz the unthrottled line is the log (T27's shape in the
    // logging dimension).
    if (target != in.target_mm)
        GLOGW_EVERY_MS(1000, kTag, "WINDOW CLAMP: %.2f -> %.2f mm",
                       double(in.target_mm), double(target));

    // Limit-set selection. Ceilings are clamps, never targets; a deadline-less
    // Manual point move is the ratified exception and plans AT the user set.
    // TODO(val-091.4): the soft-start cap, which shapes the INPUT set only and
    // has no source to shape yet.
    const float s  = span();
    _engine.setLimits(limitsFor(manual));

    // Plan from the machine's ACTUAL state. At rest that state is the emitter's
    // count and nothing else: an engine that re-seeds from its own idea of
    // where it stopped carries every move's sub-step residue into the next one.
    // Mid-plan the engine's own (p, v, a) IS the continuous state, and reseeding
    // there would be a discontinuity, so the door is rest only.
    if (!_engine.isBusy(now_us)) {
        _engine.resetAt(toNorm(positionMm()), now_us);
        _p_cmd_mm = positionMm();
    }

    kinetic::Command cmd;
    cmd.target       = toNorm(target);
    cmd.has_duration = in.duration_us > 0;
    cmd.duration_us  = in.duration_us;
    cmd.has_end_vel  = in.has_end_vel;
    cmd.end_vel      = in.end_vel_mm_s / s;
    cmd.client_curve_family = in.curve_family;
    // An anchor already in the past is not a schedule, it is arrival: passing
    // it through would spend a schedule slot to say "now".
    cmd.has_anchor   = in.anchor_us > now_us;
    cmd.anchor_us    = in.anchor_us;
    const uint64_t t0 = _now_us();
    const bool ok = _engine.commit(cmd, now_us);
    const uint32_t plan_us = uint32_t(_now_us() - t0);
    _plan_us_last = plan_us;
    if (plan_us > _plan_us_max) _plan_us_max = plan_us;
    _plan_us_avg += (float(plan_us) - _plan_us_avg) * 0.125f;
    if (!ok) {
        ++_rejected;
        GLOGW_EVERY_MS(1000, kTag, "REJECT: plan failed for %.2f mm", double(target));
        return false;
    }
    ++_intents;
    _demand_mm = target;
    _stream    = in.source == MotionSource::Stream;
    return true;
}

kinetic::Limits MotionArbiter::limitsFor(bool manual) const {
    const float s = span();
    kinetic::Limits lim;
    lim.vmax = manual ? _user_v / s : inputVmaxMm() / s;
    lim.amax = manual ? _user_a / s : inputAmaxMm() / s;
    lim.jmax = _ovr_j > 0.0f ? _ovr_j : _in_j / s;
    return lim;
}

// ---- the tick ---------------------------------------------------------------

// SPEC 11.1 STOP, owning task only: the engine's own SETTLE brake, planned
// from the plan's (p, v, a) -- continuous with what the emitter is rendering,
// unlike a census read -- at the input decel, and never a reversal. It also
// drops every scheduled plan (Engine::brake).
void MotionArbiter::brakeToRest(uint64_t now_us) {
    _engine.setLimits(limitsFor(false));
    [[maybe_unused]] const float v = _engine.velocityAt(now_us) * span();   // log only
    if (!_engine.brake(now_us)) return;
    _demand_mm = toMm(_engine.snapshot(now_us).target);   // where it comes to rest
    GLOGI(kTag, "STOP: braking from %.1f mm/s", double(v));
}

void MotionArbiter::applyTuning(const MotionTuning& t) {
    // A COPY of the live config with the tuning fields replaced, so the
    // limits accept() last set ride through untouched. Takes effect at the
    // next plan; an in-flight trajectory keeps the config it was planned under.
    kinetic::Config c = _engine.config();
    c.chase_feedforward           = t.chase_ff;
    c.chase_accel_ff              = t.chase_accel_ff;
    c.chase_ff_gain               = t.chase_gain;
    c.chase_lookahead             = t.chase_lookahead;
    c.chase_dense_us              = t.chase_dense_us;
    c.chase_aim_accel_extrap      = t.chase_aim_extrap;
    c.handoff_chord_factor        = t.handoff_k;
    c.curve_policy                = static_cast<kinetic::CurvePolicy>(t.curve_policy);
    c.infeasible_policy           = t.infeasible_policy == 0 ? kinetic::InfeasiblePolicy::Stretch
                                                             : kinetic::InfeasiblePolicy::Blend;
    c.infeasible_smooth_budget    = t.smooth_budget;
    c.infeasible_amplitude_budget = t.amplitude_budget;
    c.infeasible_blend_steps      = t.blend_steps;
    c.settle_grace_us             = t.settle_grace_us;
    c.overshoot_guard             = t.overshoot_guard;
    _engine.setConfig(c);
    _ovr_v = t.vmax_ovr;
    _ovr_a = t.amax_ovr;
    _ovr_j = t.jmax_ovr;
}

void MotionArbiter::evaluate(uint64_t now_us, float dt_s) {
    if (_estop) {
        _brake_req.store(false);   // park already stopped it
        _emitter.park();
        // ONCE per latch, and on the task that owns the engine: without it the
        // abandoned plan keeps reading busy and canClearEstop() -- which asks
        // exactly that -- would never let the latch drop (SPEC 11.2).
        if (!_estop_settled) {
            _engine.resetAt(toNorm(positionMm()), now_us);
            _p_cmd_mm = positionMm();
            _estop_settled = true;
        }
        return;
    }

    // A FRAME MOVE IS NOT MOTION. force_home re-origins the count and a window
    // change rescales normalized units, so both make plan_mm and position_mm
    // jump by up to the whole rail while the carriage stands still. Left alone
    // the feedforward differences that jump and demands 10^5 mm/s, the emitter
    // floor clamps it, and the emitter renders a saturated burst -- unrequested
    // travel at its maximum rate, plus thousands of missed deadlines
    // (measured, bd val-091.13). Re-anchor the plan and the feedforward's own
    // previous sample instead, and park for this one tick: the next accepted
    // intent plans from the new frame's rest.
    if (_frame_moved) {
        _frame_moved = false;
        _engine.resetAt(toNorm(positionMm()), now_us);
        _p_cmd_mm = positionMm();
        _emitter.steer(0.0f);
        return;
    }

    if (_brake_req.exchange(false)) brakeToRest(now_us);

    // The one side-effecting sample per tick: it promotes scheduled plans and
    // engages SETTLE when a plan ends still moving.
    const float p_plan_mm = toMm(_engine.positionAt(now_us));

    // Feedforward: the plan's OWN mean velocity across the interval that just
    // elapsed. Summed over a move this telescopes to exactly the plan's
    // displacement, which is why the emitter needs no other position input.
    float v = (p_plan_mm - _p_cmd_mm) / dt_s;
    _p_cmd_mm = p_plan_mm;

    // Residual against POSITION TRUTH, deadbanded at one step. It exists for
    // the ramp out of rest: the opening velocity is small, so the first edge
    // period is long, and the emitter cannot render the plan's first steps on
    // time however often it is re-steered.
    const float err_mm = p_plan_mm - positionMm();
    if (std::fabs(err_mm) > kMmPerStep) {
        // BOUNDED, because the residual is proportional to an error no ceiling
        // shaped and is therefore the one term that can hand the emitter a
        // demand the machine cannot make. Its ceiling is the accel limit's own
        // answer to "how much velocity may one tick add".
        const float kick_max = inputAmaxMm() * dt_s;
        float kick = err_mm * kTrackHz;
        if (kick >  kick_max) kick =  kick_max;
        if (kick < -kick_max) kick = -kick_max;
        v += kick;
    }
    // The emitter floor is a FAULT DETECTOR, never a shaper (architecture.md
    // section 2), so the arbiter holds its own last word. The bound is the sum
    // of the two terms that make it: the plan, under the speed ceiling by
    // construction, plus a correction already capped at one tick of accel.
    // Deliberately NOT the bare ceiling -- at a demand that sits ON vmax, a
    // tracking correction has to be allowed above it or the residual can never
    // close (measured: 25 ms of added lag on the ceiling-limited run).
    const float v_cap = inputVmaxMm() + inputAmaxMm() * dt_s;
    if (v >  v_cap) v =  v_cap;
    if (v < -v_cap) v = -v_cap;

    _emitter.steer(v);
}

// Drains the engine's anomaly ring into the per-kind table 0x1111 publishes.
// A kind past the table is DROPPED rather than folded into a neighbor: a
// miscounted kind reads as a diagnosis that never happened.
void MotionArbiter::drainAnomalies() {
    kinetic::Anomaly a;
    while (_engine.popAnomaly(a)) {
        if (a.kind < kAnomalyKinds) ++_anom[a.kind];
        ++_anomalies;
    }
}

MotionCensus MotionArbiter::snapshot(uint64_t now_us) {
    const kinetic::Snapshot s = _engine.snapshot(now_us);
    const float s_mm = span();

    const int32_t steps = _emitter.count();
    const int32_t d_steps = steps - _odo_steps;
    _odo_steps = steps;
    if (d_steps != 0) {
        const float d_mm = float(d_steps) * kMmPerStep;
        _distance_mm += std::fabs(d_mm);
        const int32_t dir = d_steps > 0 ? 1 : -1;
        if (dir != _stroke_dir) {
            if (_stroke_run_mm >= kStrokeMinMm) ++_strokes;
            _stroke_dir = dir;
            _stroke_run_mm = 0.0f;
        }
        _stroke_run_mm += std::fabs(d_mm);
    }

    const float vel_mm_s = s.vel * s_mm;
    if (std::fabs(vel_mm_s) > _peak_mm_s) _peak_mm_s = std::fabs(vel_mm_s);

    MotionCensus c{};
    c.steps          = steps - _origin;
    c.position_mm    = float(c.steps) * kMmPerStep;
    c.plan_mm        = toMm(s.pos);
    c.target_mm      = toMm(s.target);
    c.velocity_mm_s  = vel_mm_s;
    c.residual_steps = static_cast<int32_t>(std::lround((c.plan_mm - c.position_mm) * kStepsPerMm));
    c.win_min        = _win_min;
    c.win_max        = _win_max;
    c.rail_mm        = _rail;
    c.intents        = _intents;
    c.rejected       = _rejected;
    c.homed          = _homed;
    c.estop          = _estop;
    c.paused         = _paused;
    c.busy           = _engine.isBusy(now_us);
    c.mode           = s.mode;
    c.plan_kind      = s.plan_kind;
    c.plan_start     = s.start;
    c.plan_end       = s.target;
    c.plan_cur       = s.pos;
    c.plan_vel       = s.vel;
    c.plan_duration_us = uint32_t(s.duration_s * 1e6f);
    c.plan_elapsed_us  = uint32_t(s.elapsed_s * 1e6f);
    c.plans          = s.plans;
    c.failures       = s.failures;
    c.anomalies      = _anomalies;
    c.anom           = _anom;
    c.plan_us_last   = _plan_us_last;
    c.plan_us_max    = _plan_us_max;
    c.plan_us_avg    = _plan_us_avg;
    c.demand_mm      = _demand_mm;
    c.distance_mm    = _distance_mm;
    c.peak_mm_s      = _peak_mm_s;
    c.strokes        = _strokes;
    c.stream_bundles = _sync_bundles.load(std::memory_order_relaxed);
    c.stream_samples = _sync_samples.load(std::memory_order_relaxed);
    c.stream_dropped = _sync_dropped.load(std::memory_order_relaxed);
    // The source stops owning motion when motion stops, so the 0x1100 stream
    // flag falls on its own rather than latching until the next Manual move.
    c.stream         = _stream && c.busy;
    return c;
}

// ---- factory tuning ---------------------------------------------------------

MotionTuning motionDefaultTuning() {
    const kinetic::Config cfg{};
    MotionTuning t;
    t.chase_ff         = cfg.chase_feedforward;
    t.chase_accel_ff   = cfg.chase_accel_ff;
    t.chase_gain       = cfg.chase_ff_gain;
    t.chase_lookahead  = cfg.chase_lookahead;
    t.chase_dense_us   = cfg.chase_dense_us;
    t.chase_aim_extrap = cfg.chase_aim_accel_extrap;
    t.handoff_k        = cfg.handoff_chord_factor;
    t.curve_policy     = uint8_t(cfg.curve_policy);
    // The catalog select is 0 stretch / 1 blend; the engine's own ordinals are
    // pinned at 0 and 5 by what is already persisted elsewhere in the
    // ecosystem, so the mapping is explicit rather than a cast.
    t.infeasible_policy = cfg.infeasible_policy == kinetic::InfeasiblePolicy::Stretch ? 0 : 1;
    t.smooth_budget    = cfg.infeasible_smooth_budget;
    t.amplitude_budget = cfg.infeasible_amplitude_budget;
    t.blend_steps      = cfg.infeasible_blend_steps;
    t.settle_grace_us  = cfg.settle_grace_us;
    t.overshoot_guard  = cfg.overshoot_guard;
    return t;
}

}  // namespace valence
