// SimMotion -- motion/ValenceMotion.h on a desktop: the REAL kinetic::Engine
// behind a host copy of the P4 arbiter's gates, with an ideal emitter
// Constraints:
// - SINGLE-THREADED (SimMotion.h). The P4 hands intents across a FreeRTOS
//   queue to its motion task; here submit() and simMotionTick() share one
//   thread, so the queue is a plain ring and nothing is locked.
// - accept(), evaluate() and applyTuning() MIRROR MotionArbiter in
//   flagship_p4/src/motion/ValenceMotion.cpp BY HAND: same gate order, same
//   window clamp, same limit-set selection, same rest-only reseed, same
//   frame-move re-anchor. Change one and change the other in the same commit
//   until the arbiter is lifted into a hardware-free core.
// TODO(val-sf7.1): compile the P4 arbiter's gates here instead of this copy.
// - THE EMITTER IS IDEAL. Position truth is the plan sampled at the tick and
//   quantized to whole steps, so late edges, re-steers, residuals and emitter
//   faults read 0. That is the honest answer for a machine with no LP core,
//   not a measurement of one.
// - Millimeters on the interface, normalized 0..1 window units inside the
//   engine, exactly as on the P4.
// See: SimMotion.h, flagship_p4/src/motion/ValenceMotion.h

#include "SimMotion.h"

#include <array>
#include <cmath>
#include <optional>

#include "hub/ValenceDevice.h"
#include "hub/valence_config.h"
#include "kinetic/kinetic.hpp"
#include "motion/ValenceMotion.h"

namespace valence {
namespace {

// Machine constants, mirrored from ValenceMotion.cpp (see the file header).
constexpr float kStepsPerMm = 208.608f;
constexpr float kMmPerStep  = 1.0f / kStepsPerMm;
constexpr float kLpClockHz  = 40.0e6f;
constexpr float kParkMmS    = 0.0115f;
constexpr size_t kIntentQueueDepth = 40;
constexpr float kStrokeMinMm = 1.0f;

class SimArbiter {
public:
    void begin(uint64_t now_us) {
        _engine.resetAt(toNorm(0.0f), now_us);
        _p_cmd_mm = 0.0f;
        _prev_us = now_us;
        refreshSnapshot(now_us);
    }

    bool submit(const MotionIntent& in) {
        if (_count == _queue.size()) return false;
        _queue[(_head + _count) % _queue.size()] = in;
        ++_count;
        return true;
    }

    void estop(bool on) {
        _estop = on;
        if (!on) {
            _estop_settled = false;
            return;
        }
        _step_q8 = 0;
        _homed = false;
    }

    void pause(bool on) { _paused = on; }
    void setUserLimits(float v, float a) { _user_v = v; _user_a = a; }
    void setInputLimits(float v, float a, float j) { _in_v = v; _in_a = a; _in_j = j; }
    // The P4 overwrites a depth-one queue; one pending slot is the same thing
    // on one thread.
    void setTuning(const MotionTuning& t) { _tune_pending = t; }

    void setWindow(float lo, float hi, float rail) {
        if (!(std::isfinite(lo) && std::isfinite(hi) && std::isfinite(rail))) return;
        if (!(hi > lo)) return;
        _win_min = lo;
        _win_max = hi;
        _rail = rail > 0.0f ? rail : DEFAULT_MAX_RAIL_MM;
        _frame_moved = true;
    }

    float forceHome(float stroke_mm) {
        float stroke = stroke_mm;
        if (!(stroke >= 1.0f)) stroke = 250.0f;
        if (stroke > _rail) stroke = _rail;
        _origin = _phys_steps;
        _frame_moved = true;
        _rail = stroke;
        _estop = false;
        _estop_settled = false;
        _homed = true;
        return stroke;
    }

    void noteStream(uint32_t b, uint32_t s, uint32_t d) {
        _sync_bundles += b;
        _sync_samples += s;
        _sync_dropped += d;
    }

    void tick(uint64_t now_us) {
        if (_tune_pending) {
            applyTuning(*_tune_pending);
            _tune_pending.reset();
        }
        while (_count > 0) {
            const MotionIntent in = _queue[_head];
            _head = (_head + 1) % _queue.size();
            --_count;
            accept(in, now_us);
        }
        const float dt_s = float(now_us - _prev_us) * 1e-6f;
        if (dt_s > 0.0f) {
            _prev_us = now_us;
            evaluate(now_us, dt_s);
        }
        drainAnomalies();
        refreshSnapshot(now_us);
    }

    MotionCensus census() const { return _pub; }

private:
    float positionMm() const { return float(_phys_steps - _origin) * kMmPerStep; }
    float span() const { return _win_max - _win_min; }
    float toNorm(float mm) const { return (mm - _win_min) / span(); }
    float toMm(float norm) const { return _win_min + norm * span(); }
    float inputVmaxMm() const { return _ovr_v > 0.0f ? _ovr_v * span() : _in_v; }
    float inputAmaxMm() const { return _ovr_a > 0.0f ? _ovr_a * span() : _in_a; }

    void applyTuning(const MotionTuning& t) {
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

    void steer(float v_mm_s) {
        const float mag = std::fabs(v_mm_s);
        if (!(mag >= kParkMmS)) {
            _step_q8 = 0;
            return;
        }
        _step_q8 = uint32_t(kLpClockHz / (mag * kStepsPerMm) * 256.0f + 0.5f);
    }

    bool accept(const MotionIntent& in, uint64_t now_us) {
        if (_estop) {
            ++_rejected;
            return false;
        }
        if (in.source != MotionSource::Manual) {
            if (!_homed || _paused) {
                ++_rejected;
                return false;
            }
        }
        const bool manual = in.source == MotionSource::Manual;

        float target = in.target_mm;
        const float lo = manual ? 0.0f : (_win_min > 0.0f ? _win_min : 0.0f);
        const float hi_win = _win_max < _rail ? _win_max : _rail;
        const float hi = manual ? _rail : hi_win;
        if (target < lo) target = lo;
        if (target > hi) target = hi;

        const float s = span();
        kinetic::Limits lim;
        lim.vmax = manual ? _user_v / s : inputVmaxMm() / s;
        lim.amax = manual ? _user_a / s : inputAmaxMm() / s;
        lim.jmax = _ovr_j > 0.0f ? _ovr_j : _in_j / s;
        _engine.setLimits(lim);

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
        cmd.has_anchor   = in.anchor_us > now_us;
        cmd.anchor_us    = in.anchor_us;
        const uint64_t t0 = deviceNowUs();
        const bool ok = _engine.commit(cmd, now_us);
        const uint32_t plan_us = uint32_t(deviceNowUs() - t0);
        _plan_us_last = plan_us;
        if (plan_us > _plan_us_max) _plan_us_max = plan_us;
        _plan_us_avg += (float(plan_us) - _plan_us_avg) * 0.125f;
        if (!ok) {
            ++_rejected;
            return false;
        }
        ++_intents;
        _demand_mm = target;
        _stream = !manual;
        return true;
    }

    void evaluate(uint64_t now_us, float dt_s) {
        if (_estop) {
            _step_q8 = 0;
            if (!_estop_settled) {
                _engine.resetAt(toNorm(positionMm()), now_us);
                _p_cmd_mm = positionMm();
                _estop_settled = true;
            }
            return;
        }
        // A frame move is not motion: re-anchor and park for this one tick,
        // exactly as the P4 does (ValenceMotion.cpp, evaluate()).
        if (_frame_moved) {
            _frame_moved = false;
            _engine.resetAt(toNorm(positionMm()), now_us);
            _p_cmd_mm = positionMm();
            steer(0.0f);
            return;
        }
        const float p_plan_mm = toMm(_engine.positionAt(now_us));
        const float v = (p_plan_mm - _p_cmd_mm) / dt_s;
        _p_cmd_mm = p_plan_mm;
        steer(v);
        const int64_t steps = _origin + int64_t(std::lround(p_plan_mm * kStepsPerMm));
        _edges += uint32_t(std::llabs(steps - _phys_steps));
        _phys_steps = steps;
    }

    void drainAnomalies() {
        kinetic::Anomaly a;
        while (_engine.popAnomaly(a)) {
            if (a.kind < kAnomalyKinds) ++_anom[a.kind];
            ++_anomalies;
        }
    }

    void refreshSnapshot(uint64_t now_us) {
        const kinetic::Snapshot s = _engine.snapshot(now_us);
        const float s_mm = span();

        const int64_t d_steps = _phys_steps - _odo_steps;
        _odo_steps = _phys_steps;
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
        c.steps          = int32_t(_phys_steps - _origin);
        c.position_mm    = float(c.steps) * kMmPerStep;
        c.plan_mm        = toMm(s.pos);
        c.target_mm      = toMm(s.target);
        c.velocity_mm_s  = vel_mm_s;
        c.residual_steps = int32_t(std::lround((c.plan_mm - c.position_mm) * kStepsPerMm));
        c.win_min        = _win_min;
        c.win_max        = _win_max;
        c.rail_mm        = _rail;
        c.edges          = _edges;
        c.step_q8        = _step_q8;
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
        c.stream_bundles = _sync_bundles;
        c.stream_samples = _sync_samples;
        c.stream_dropped = _sync_dropped;
        c.stream         = _stream && c.busy;
        _pub = c;
    }

    kinetic::Engine _engine{};
    std::array<MotionIntent, kIntentQueueDepth> _queue{};
    size_t _head = 0;
    size_t _count = 0;

    float _win_min = 0.0f;
    float _win_max = DEFAULT_MAX_RAIL_MM;
    float _rail    = DEFAULT_MAX_RAIL_MM;
    float _user_v = DEFAULT_USER_MAX_SPEED_MM_S;
    float _user_a = DEFAULT_USER_ACCEL_MM_S2;
    float _in_v   = DEFAULT_MAX_SPEED_MM_S;
    float _in_a   = DEFAULT_ACCEL_MM_S2;
    float _in_j   = DEFAULT_INPUT_MAX_JERK_MM_S3;
    float _ovr_v  = 0.0f;
    float _ovr_a  = 0.0f;
    float _ovr_j  = 0.0f;
    std::optional<MotionTuning> _tune_pending;

    int64_t _phys_steps = 0;   // the ideal emitter's absolute count
    int64_t _origin = 0;       // the count that means 0.0 mm
    float _p_cmd_mm = 0.0f;
    uint64_t _prev_us = 0;
    uint32_t _step_q8 = 0;

    bool _homed = false;
    bool _estop = false;
    bool _paused = false;
    bool _estop_settled = false;
    bool _frame_moved = false;

    int64_t _odo_steps = 0;
    int32_t _stroke_dir = 0;
    float _stroke_run_mm = 0.0f;

    uint32_t _intents = 0;
    uint32_t _rejected = 0;
    uint32_t _edges = 0;
    uint32_t _plan_us_last = 0;
    uint32_t _plan_us_max = 0;
    float _plan_us_avg = 0.0f;
    float _demand_mm = 0.0f;
    bool _stream = false;
    uint32_t _anomalies = 0;
    std::array<uint32_t, kAnomalyKinds> _anom{};
    float _distance_mm = 0.0f;
    float _peak_mm_s = 0.0f;
    uint32_t _strokes = 0;
    uint32_t _sync_bundles = 0;
    uint32_t _sync_samples = 0;
    uint32_t _sync_dropped = 0;

    MotionCensus _pub{};
};

// File scope, not a stack local: the engine holds KB-scale Ruckig state.
SimArbiter g_arb;

}  // namespace

void simMotionTick(uint64_t now_us) { g_arb.tick(now_us); }

// ---- motion/ValenceMotion.h ----------------------------------------------------

bool motionBegin() {
    g_arb.begin(deviceNowUs());
    return true;
}
bool motionSubmit(const MotionIntent& in) { return g_arb.submit(in); }
void motionEstop() { g_arb.estop(true); }
void motionEstopClear() { g_arb.estop(false); }
void motionPause(bool on) { g_arb.pause(on); }
void motionSetUserLimits(float v, float a) { g_arb.setUserLimits(v, a); }
void motionSetInputLimits(float v, float a, float j) { g_arb.setInputLimits(v, a, j); }
void motionSetWindow(float lo, float hi, float rail) { g_arb.setWindow(lo, hi, rail); }
void motionNoteStream(uint32_t b, uint32_t s, uint32_t d) { g_arb.noteStream(b, s, d); }
float motionForceHome(float stroke_mm) { return g_arb.forceHome(stroke_mm); }
MotionCensus motionCensus() { return g_arb.census(); }

void motionSetTuning(const MotionTuning& t) { g_arb.setTuning(t); }

// Mirrors ValenceMotion.cpp's motionDefaultTuning().
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
    t.infeasible_policy = cfg.infeasible_policy == kinetic::InfeasiblePolicy::Stretch ? 0 : 1;
    t.smooth_budget    = cfg.infeasible_smooth_budget;
    t.amplitude_budget = cfg.infeasible_amplitude_budget;
    t.blend_steps      = cfg.infeasible_blend_steps;
    t.settle_grace_us  = cfg.settle_grace_us;
    t.overshoot_guard  = cfg.overshoot_guard;
    return t;
}

}  // namespace valence
