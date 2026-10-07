// MotionArbiter -- the gates, the clamp, the limit sets and the feedforward,
// compiled by the board and by the host twin alike
// Constraints:
// - HARDWARE-FREE (MotionArbiter.h). Geiger is the log path and binds its own
//   platform layer; on a host it is mute, never a second log path.
// - An intent becomes knots AT ARRIVAL, continuing from the engine's actual
//   (p, v, a); the window is solved at the next sample, and the tick only
//   EVALUATES it. Nothing here plans on a clock.
// See: MotionArbiter.h, .claude/rules/motion-control.md, bd val-091.4, bd val-z1k

#include "MotionArbiter.h"

#include <cmath>

#include "geiger/geiger.h"
#include "valence/generated/registry_constants.hpp"

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

bool isGenerator(MotionSource s) { return s == MotionSource::Pattern || s == MotionSource::Advanced; }

[[maybe_unused]] const char* sourceName(uint8_t id) {
    return id < kMotionSourceNames.size() ? kMotionSourceNames[id] : "none";
}

static_assert(kAnomalyKinds <= 32, "drainAnomalies() reports the kinds as one word");

// Option ordinals of the 0x1110 style and 0x1111 mode selects and of the
// 0x1111 plan_kind select (ValenceCatalog.h). Wire values: never renumber.
enum class PlanStyle : uint8_t { idle = 0, waveform = 1, chase = 2, settle = 3 };
constexpr uint8_t kPlanKindQuintic = 1;

class AbsentHomeSense final : public HomeSense {
public:
    bool present() const override { return false; }
    Probe probe() override { return Probe::undriven; }
    bool high() override { return false; }
};

AbsentHomeSense g_absent_sense;

static_assert(MotionTuning{}.lookahead_us == kinetic2::Config{}.lookahead_us &&
                  MotionTuning{}.corner == uint8_t(kinetic2::Config{}.corner) &&
                  MotionTuning{}.react_us == kinetic2::Config{}.react_us,
              "MotionTuning's Kinetic² defaults are kinetic2::Config's");

// RFC-105 (k): a point move is a knot at the least time a rest-to-rest
// quintic of `d` is legal under every ceiling, plus the 1 ms margin Kinetic's
// oracle test proves it legal with (tests/test_kinetic2_oracle.cpp). A ceiling
// of 0 or a non-finite one parks the knot 600 s out, where the solver spends it.
uint32_t parkUs(float d, const kinetic2::Limits& L) {
    const float t = std::fmax(std::fmax(1.875f * d / L.vmax, std::sqrt(5.7735f * d / L.amax)),
                              std::cbrt(60.0f * d / L.jmax));
    return !(t < 600.0f) ? 600000000u : uint32_t(t * 1e6f) + 1000u;
}

// The count from the origin at which positionMm() first reads at or past `mm`
// outward: the least count reading at or above it (up), the greatest reading
// at or below it (down). A fence there never cuts off a point the backstop
// admits and never admits a whole step past one.
int32_t fenceCount(float mm, bool up) {
    int32_t c = int32_t(std::lround(mm * kStepsPerMm));
    if (up) {
        while (float(c) * kMmPerStep < mm) ++c;
        while (float(c - 1) * kMmPerStep >= mm) --c;
    } else {
        while (float(c) * kMmPerStep > mm) --c;
        while (float(c + 1) * kMmPerStep <= mm) ++c;
    }
    return c;
}

}  // namespace

HomeSense& noHomeSense() { return g_absent_sense; }

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
    // ESTOP drops override (SPEC 11.1); release lands in plain PAUSE.
    _override.store(false);
    _return_req.store(false);
    if (!on) {
        // Release lands in PAUSE (SPEC 11.2): latched BEFORE the e-stop gate
        // opens, so no intent finds both gates open in between.
        pause(true);
        _estop_settled = false;
        _estop = false;
        return;
    }
    _estop = true;
    // Park on the CALLING task. The ENGINE is not touched here: it belongs to
    // the owning task, which resets it on the next tick (see evaluate()).
    _rail_gen.store(kRailClosed);   // releasing the latch never restarts a generator
    _emitter.park();
    // A power cut leaves the carriage limp wherever it coasted: the position
    // reference is gone. A halt keeps power, so it keeps home.
    if (_cuts_power) _homed = false;
    GLOGW(kTag, "ESTOP: emitter parked at %.3f mm%s", double(positionMm()),
          _cuts_power ? ", unhomed" : "");
}

bool MotionArbiter::acquireRail(MotionSource generator) {
    if (!isGenerator(generator)) return false;
    const uint8_t want = uint8_t(generator);
    uint8_t held = _rail_gen.load();
    while (held == kRailFree || held == kRailClosed) {
        if (_rail_gen.compare_exchange_weak(held, want)) return true;
    }
    if (held == want) return true;
    GLOGW(kTag, "SOURCE_CONFLICT: %s start refused, rail owned by %s", sourceName(want), sourceName(held));
    return false;
}

void MotionArbiter::releaseRail(MotionSource generator) {
    uint8_t held = uint8_t(generator);
    _rail_gen.compare_exchange_strong(held, kRailFree);
}

void MotionArbiter::setMotorPowered(bool on) {
    if (!_powered.exchange(on) || on) return;
    // A loss, not a boot: no plan renders into the dead drive, and no return
    // queued before it starts when power comes back.
    _return_req.store(false);
    _power_settled.store(false);
    _emitter.park();
    _homed = false;
    GLOGW(kTag, "MOTOR POWER LOST: emitter parked at %.3f mm, unhomed", double(positionMm()));
}

void MotionArbiter::pause(bool on) {
    if (!on) {
        // A brake the owning task has not run yet belongs to the pause being
        // cleared; left set it would brake the first plan after resume.
        _brake_req.store(false);
        _override.store(false);
        _paused.store(false);
        return;
    }
    // A home cycle runs under a latched PAUSE (SPEC 11.1 admits the home
    // verb), so a pause asked for during one brakes it whatever the latch.
    const bool homing = _homing.load();
    if (homing) _home_abort.store(true);
    if (!_paused.exchange(true) || homing) _brake_req.store(true);
}

void MotionArbiter::override() {
    pause(true);
    _override.store(true);
}

ReturnStart MotionArbiter::returnToPause() {
    if (!_override.load()) return ReturnStart::none;
    if (powerGateOpen()) {
        _return_req.store(true);
        return ReturnStart::queued;
    }
    // Nothing renders while the gate is shut, so there is no plan to wait
    // for: the emitter count is still (any task), and so is the paused
    // position, which evaluate() records at rest even while unpowered.
    const float dist = std::fabs(positionMm() - _pause_pos_mm.load());
    if (dist > kMmPerStep) {
        GLOGW(kTag, "RETURN refused: motor power off, %.2f mm from the paused position", double(dist));
        return ReturnStart::unpowered;
    }
    _override.store(false);
    _returns.fetch_add(1);
    GLOGI(kTag, "RETURN: already at the paused position, override off, PAUSE holds");
    return ReturnStart::arrived;
}

void MotionArbiter::setWindow(float lo, float hi, float rail) {
    if (!(std::isfinite(lo) && std::isfinite(hi) && std::isfinite(rail))) return;
    if (!(hi > lo)) return;
    _win_min = lo;
    _win_max = hi;
    _rail    = rail > 0.0f ? rail : DEFAULT_MAX_RAIL_MM;
    _max_rail = _rail;
    _frame_moved = true;   // normalized units now mean different millimeters
}

HomeStart MotionArbiter::home() {
    if (!_sense->present()) return HomeStart::no_sense;
    if (_estop) return HomeStart::estop;
    if (!powerGateOpen()) return HomeStart::unpowered;
    if (_homing.exchange(true)) return HomeStart::started;
    // Probed with _homing held, so the owning task never reads the line
    // while the probe drives its pull.
    const HomeSense::Probe p = _sense->probe();
    if (p != HomeSense::Probe::low) {
        _homing.store(false);
        GLOGW(kTag, "HOME refused: the home sense %s", p == HomeSense::Probe::high
                        ? "already reads a stall" : "is not driven (sensor unwired or down)");
        return p == HomeSense::Probe::high ? HomeStart::sense_high : HomeStart::undriven;
    }
    _home_req.store(true);
    return HomeStart::started;
}

void MotionArbiter::homeSenseRose() {
    // One rise trips one armed seek; anything else is not this interrupt's.
    uint32_t armed = kSeekArmed;
    if (!_seek.compare_exchange_strong(armed, kSeekTripping)) return;
    // The trip is ordered before the park: homeSteer() steers, fences and
    // reads the word, so a steer racing this park is parked again behind it.
    std::atomic_thread_fence(std::memory_order_seq_cst);
    _emitter.park();
    _seek_hit.store(_emitter.count(), std::memory_order_relaxed);
    _seek.store(kSeekLatched, std::memory_order_release);
}

float MotionArbiter::forceHome(float stroke_mm) {
    // Written so a NaN takes the default: !(x >= 1), not (x < 1).
    float stroke = stroke_mm;
    if (!(stroke >= 1.0f)) stroke = 250.0f;
    if (stroke > _rail) stroke = _rail;
    // Home swaps ends under the flip (SPEC 9.6): the carriage is asserted at
    // the client's 0, which is the physical far end of the stroke.
    const int32_t far_steps = _flipped.load() ? int32_t(std::lround(stroke * kStepsPerMm)) : 0;
    _origin    = _emitter.count() - far_steps;
    _frame_moved = true;   // 0.0 mm now means a different emitter count
    _rail      = stroke;
    _homed     = true;
    // Under PAUSE the recorded rest point is in the old frame: a brake request
    // at rest re-records it in the new one on the owning task.
    if (_paused.load()) _brake_req.store(true);
    // The engine reseeds itself at rest on the next accepted intent; nothing
    // here may call into it, this runs on the hub task. The e-stop latch is
    // NOT touched: the hub's release path drops it (estop(false)), into PAUSE.
    GLOGW(kTag, "FORCE HOME: homed asserted at 0.0 mm, stroke %.1f mm "
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
    _lapses_seen = _emitter.lapses();   // the boot liveness burst's own lapse
    resetEngine(toNorm(0.0f), now_us);
    _p_cmd_mm = 0.0f;
}

bool MotionArbiter::accept(const MotionIntent& asked, uint64_t now_us) {
    // The flip's way in (setFlipped()): a target in the mirrored frame is the
    // physical point rail minus it, and a velocity reverses.
    MotionIntent in = asked;
    if (_flipped.load()) {
        in.target_mm    = _rail - in.target_mm;
        in.end_vel_mm_s = -in.end_vel_mm_s;
    }
    // E-stop is the one gate no source bypasses.
    if (_estop) {
        ++_rejected;
        GLOGW_EVERY_MS(1000, kTag, "REJECT: e-stop");
        return false;
    }
    // A home cycle owns the rail from its request to its end.
    if (_homing.load()) {
        ++_rejected;
        GLOGW_EVERY_MS(1000, kTag, "REJECT: homing");
        return false;
    }
    // Neither does motor power: a plan rendered into an unpowered drive moves
    // position truth and not the carriage. The bench profile renders it
    // anyway, on purpose: a devkit has no drive to desync.
    // ONE load of the switch's word, and the profile tested as the constexpr
    // it is: a second load could see power arrive in between and log a bench
    // bypass on a release image, and the release image compiles no bypass.
    if (!_powered.load()) {
        if constexpr (!kBenchNoMotor) {
            ++_rejected;
            GLOGW_EVERY_MS(1000, kTag, "REJECT: motor power off");
            return false;
        } else if (!_bench_noted) {
            _bench_noted = true;
            GLOGW(kTag, "BENCH: motor power gate bypassed");
        }
    }
    // PAUSE suspends every source (SPEC 11.1). Under override the operator's
    // jog is the one motion accepted, except while the return runs.
    const bool jog = in.source == MotionSource::Manual && _override.load();
    if (_paused.load() && !jog) {
        ++_rejected;
        GLOGW_EVERY_MS(1000, kTag, "REJECT: paused");
        return false;
    }
    if (jog && _returning) {
        ++_rejected;
        GLOGW_EVERY_MS(1000, kTag, "REJECT: returning to the paused position");
        return false;
    }
    // The pause brake the owning task has not run yet runs FIRST: it records
    // the paused position, and run after the jog it would brake the jog.
    if (jog && _brake_req.exchange(false)) brakeToRest(now_us);
    // The jog plans in the rail frame, entered only at rest: a frame change
    // under a moving plan would be a discontinuity.
    if (jog && !_rail_frame) {
        if (_engine.isBusy(now_us)) {
            ++_rejected;
            GLOGW_EVERY_MS(1000, kTag, "REJECT: jog waits for the pause brake to finish");
            return false;
        }
        setRailFrame(true, now_us);
    }
    // Manual bypasses the rest (the push-to-home case): an operator must be
    // able to move an unhomed machine, and only to move it.
    if (in.source != MotionSource::Manual) {
        if (!_commissioned.load()) {
            ++_rejected;
            GLOGW_EVERY_MS(1000, kTag, "REJECT: not commissioned (setup fields never written)");
            return false;
        }
        if (!_homed) {
            ++_rejected;
            GLOGW_EVERY_MS(1000, kTag, "REJECT: not homed");
            return false;
        }
        if (isGenerator(in.source)) {
            const uint8_t held = _rail_gen.load();
            if (held == kRailClosed) {
                ++_rejected;
                GLOGW_EVERY_MS(1000, kTag, "REJECT: generators stopped by e-stop");
                return false;
            }
            if (held != kRailFree && held != uint8_t(in.source)) {
                ++_rejected;
                GLOGW_EVERY_MS(1000, kTag, "REJECT: %s, rail owned by %s",
                               sourceName(uint8_t(in.source)), sourceName(held));
                return false;
            }
        }
    }

    const bool manual = in.source == MotionSource::Manual;

    // Window clamp. The jog under override reaches the whole asserted rail
    // (SPEC 11.1 lifts the window); every other intent, Manual included, is
    // held inside the configured window, itself held inside the rail. A homed
    // rail ends kHomeSafetyMarginMm short of each stop's datum, so nothing
    // here admits a point inside either safety margin: never widen [0, rail].
    float target = in.target_mm;
    const float lo = jog ? 0.0f : (_win_min > 0.0f ? _win_min : 0.0f);
    const float hi_win = _win_max < _rail ? _win_max : _rail;
    const float hi = jog ? _rail : hi_win;
    if (target < lo) target = lo;
    if (target > hi) target = hi;
    // THROTTLED, because a stream that overhangs the window clamps EVERY
    // sample: at 50 Hz the unthrottled line is the log (T27's shape in the
    // logging dimension).
    if (target != in.target_mm)
        GLOGW_EVERY_MS(1000, kTag, "WINDOW CLAMP: %.2f -> %.2f mm",
                       double(in.target_mm), double(target));

    // Limit-set selection. Ceilings are clamps, never targets; a deadline-less
    // Manual point move is the ratified exception and plans AT the jog set.
    // TODO(val-091.4): the soft-start cap, which shapes the INPUT set only and
    // has no source to shape yet.
    return plan(target, in, limitsFor(manual), now_us);
}

bool MotionArbiter::plan(float target, const MotionIntent& in, const EngineLimits& lim, uint64_t now_us) {
    if (!submitKnots(toNorm(target), in, lim, now_us)) {
        ++_rejected;
        GLOGW_EVERY_MS(1000, kTag, "REJECT: knot refused for %.2f mm", double(target));
        return false;
    }
    ++_intents;
    _k2_window_clamped = target != in.target_mm;
    _demand_mm = target;
    _stream    = in.source == MotionSource::Stream;
    return true;
}

void MotionArbiter::notePlanCost(uint32_t us) {
    _plan_us_last = us;
    if (us > _plan_us_max) _plan_us_max = us;
    _plan_us_avg += (float(us) - _plan_us_avg) * 0.125f;
}

void MotionArbiter::resetEngine(float p_norm, uint64_t now_us) {
    _engine.resetAt(p_norm, now_us);
    _k2_newest_us   = now_us;
    _k2_newest_p    = p_norm;
    _k2_brake_to_us = 0;
    _k2_starved     = false;
    _k2_chase       = false;
}

bool MotionArbiter::brakeEngine(uint64_t at_us) {
    // The engine stops from its own state at at_us and reports neither the
    // profile's end nor its length: recomputed here from the same state and
    // limits by the same function, so the census reads exactly what renders.
    const kinetic2::State s = _engine.stateAt(0, at_us);
    const kinetic2::Profile pr = kinetic2::Profile::brake(s, at_us, _engine.config().limits);
    _engine.brake(at_us);
    _k2_dirty = true;
    _k2_brake_from_us = at_us;
    _k2_brake_from_p  = s.p;
    _k2_brake_to_us   = pr.n > 0 ? pr.end_us() : at_us;
    _k2_brake_to_p    = pr.n > 0 ? pr.end().p : s.p;
    _k2_starved       = false;
    _k2_newest_us     = _k2_brake_to_us;
    _k2_newest_p      = _k2_brake_to_p;
    return pr.n > 0;
}

// ---- the Kinetic² boundary ----------------------------------------------------
// Every conversion from an intent to the engine's knots (bd val-klo, RFC-105).
// The engine sees knots, a brake and resets; nothing here reads a wire format.

bool MotionArbiter::submitKnots(float p, const MotionIntent& in, const EngineLimits& lim, uint64_t now_us) {
    const bool manual = in.source == MotionSource::Manual;
    // The engine solves its whole pending window under the config it holds at
    // the next sample, so these limits and this policy apply to every knot
    // still pending, not to this one alone. A Manual move never trims its
    // amplitude: a jog that lands short of its target is a wrong answer, so it
    // stretches.
    kinetic2::Config cfg = _engine.config();
    cfg.limits = lim;
    cfg.policy = manual ? kinetic2::Policy::Stretch : _k2_policy;
    _engine.setConfig(cfg);

    // At rest the state is the emitter's count and nothing else: a reseed from
    // the engine's own idea of where it stopped carries every move's sub-step
    // residue into the next one. pending() first: isBusy() solves, and a
    // bundle must not solve once per sample.
    if (_engine.pending() == 0 && !_engine.isBusy(now_us)) {
        resetEngine(toNorm(positionMm()), now_us);
        _p_cmd_mm = positionMm();
    }

    // A generator's stop arrives as a point at its braking distance; it
    // renders as the brake itself.
    if (in.duration_us == 0 && isGenerator(in.source)) {
        brakeEngine(now_us);
        _k2_chase = false;
        return true;
    }

    kinetic2::Knot k;
    if (in.duration_us > 0) {
        // A 0x2101 segment, or a generator's stroke: a knot at its start plus
        // its duration with the sender's end velocity. A start past the
        // newest knot leaves a gap the sender meant as a rest, so the curve
        // holds until the start.
        const uint64_t start = in.anchor_us > now_us ? in.anchor_us : now_us;
        // RFC-087 supersede: a bundle replaces every knot queued at or after
        // its first start; the motion in flight hands off there, or at the
        // reaction horizon when the start is not past it
        // (Engine::truncateAfter). The start is then never a rest.
        if (in.supersede && _engine.truncateAfter(start, now_us) > 0) {
            const kinetic2::Knot h = _engine.newest();
            _k2_newest_us = start > h.t_us ? start : h.t_us;
            _k2_newest_p  = h.p;
            _k2_dirty = true;
        }
        if (start > _k2_newest_us) {
            kinetic2::Knot hold;
            hold.t_us   = start;
            hold.p      = _k2_newest_p;
            hold.has_v  = true;
            hold.v      = 0.0f;
            hold.family = kinetic2::Family::C2;
            if (!_engine.submit(hold, now_us)) return false;
            _k2_newest_us = start;
            _k2_dirty = true;
        }
        // The curve policy (0 follow, 1 C1, 2 C2) overrides the sender's family.
        const uint8_t fam = _k2_curve_policy == 1 ? 1 : _k2_curve_policy == 2 ? 2 : in.curve_family;
        k = kinetic2::knotFromSegment(p, in.duration_us, in.has_end_vel, in.end_vel_mm_s / span(), start,
                                      fam <= 3 ? kinetic2::Family(fam) : kinetic2::Family::Unspecified);
        _k2_chase = false;
    } else if (manual) {
        // A jog or a return: a sample (RFC-105 (n)) whose latency is the park
        // time from the newest knot under the jog set, at an authored rest: a
        // free last knot keeps its secant and the engine brakes past it
        // (RFC-105 (dd)).
        const uint64_t from = _k2_newest_us > now_us ? _k2_newest_us : now_us;
        k = kinetic2::knotFromSample(p, from, parkUs(std::fabs(p - _k2_newest_p), lim));
        k.has_v = true;   // v = 0
        _k2_chase = false;
    } else {
        // A 0x2100 sample: one behind, at the grant's latency (RFC-105 promise
        // 1). A future anchor is the sample's arrival (RFC-084).
        const uint64_t arrival = in.anchor_us > now_us ? in.anchor_us : now_us;
        k = kinetic2::knotFromSample(p, arrival, _k2_latency_us);
        _k2_chase = true;
    }
    if (!_engine.submit(k, now_us)) return false;
    _k2_newest_us = k.t_us;
    _k2_newest_p  = k.p;
    _k2_dirty = true;
    ++_k2_plans;
    // A knot accepted during a starvation brake re-planned from it: the brake
    // no longer renders. An explicit brake still does until its end.
    if (_k2_starved) _k2_brake_to_us = 0;
    _k2_starved = false;
    return true;
}

kinetic2::State MotionArbiter::sampleEngine(uint64_t now_us) {
    // The first sample after a submit solves the window: that is the plan's
    // cost, so it is what plan_us_* times.
    // TODO(val-8rt): the solve is unbounded and runs on the motion tick, and
    // the emitter renders the last steer open loop meanwhile; it moves to a
    // planner task once Kinetic offers a bounded or resumable solve.
    const bool timed = _k2_dirty;
    const uint64_t t0 = timed ? _now_us() : 0;
    // A starved stream: the last knot is due and still moving. The engine
    // brakes from it at its own time and a knot arriving meanwhile re-plans
    // from the braking state (RFC-105 (dd)), so the arbiter never brakes here:
    // an explicit brake would refuse that knot. The engine does not report the
    // profile; it is recomputed for the census from the same state and limits,
    // before stateAt() retires the knot. solved() first: it may drop knots.
    if (_engine.pending() > 0) {
        (void)_engine.solved(0, 0);
        const size_t n = _engine.pending();
        const kinetic2::Solved& k = _engine.solved(0, n > 0 ? n - 1 : 0);
        if (n > 0 && k.t_us <= now_us && (std::fabs(k.v) > 1e-6f || std::fabs(k.a) > 1e-6f)) {
            const kinetic2::Profile pr =
                kinetic2::Profile::brake(kinetic2::State{k.p, k.v, k.a}, k.t_us, _engine.config().limits);
            _k2_brake_from_us = k.t_us;
            _k2_brake_from_p  = k.p;
            _k2_brake_to_us   = pr.end_us();
            _k2_brake_to_p    = pr.end().p;
            _k2_newest_us     = _k2_brake_to_us;
            _k2_newest_p      = _k2_brake_to_p;
            _k2_starved       = true;
        }
    }
    const kinetic2::State s = _engine.stateAt(0, now_us);
    if (timed) notePlanCost(uint32_t(_now_us() - t0));
    _k2_dirty = false;
    return s;
}

void MotionArbiter::setRailFrame(bool on, uint64_t now_us) {
    _rail_frame = on;
    resetEngine(toNorm(positionMm()), now_us);
    _p_cmd_mm = positionMm();
}

EngineLimits MotionArbiter::limitsFor(bool manual) const {
    const float s = span();
    EngineLimits lim;
    lim.vmax = manual ? _jog_v / s : inputVmaxMm() / s;
    lim.amax = manual ? _jog_a / s : inputAmaxMm() / s;
    lim.jmax = _ovr_j > 0.0f ? _ovr_j : _in_j / s;
    return lim;
}

// ---- the tick ---------------------------------------------------------------

// SPEC 11.1 PAUSE, owning task only: the engine's own brake, planned from the
// plan's (p, v, a) -- continuous with what the emitter is rendering, unlike a
// census read -- at the input decel, and never a reversal. It also drops every
// pending knot (Engine::brake).
// The rest point is recorded as the paused position, the one place a
// `return` goes back to.
void MotionArbiter::brakeToRest(uint64_t now_us) {
    _engine.setLimits(limitsFor(false));
    [[maybe_unused]] const float v = sampleEngine(now_us).v * span();   // log only
    if (!brakeEngine(now_us)) {
        _pause_pos_mm.store(positionMm());
        return;
    }
    // Where it comes to rest: the brake's end, held by the backstop
    // (evaluate()) when the brake would carry it past the frame's edge.
    float rest = toMm(_k2_brake_to_p);
    const float lo = std::fmin(backstopLo(), _p_cmd_mm);
    const float hi = std::fmax(backstopHi(), _p_cmd_mm);
    if (rest < lo) rest = lo;
    if (rest > hi) rest = hi;
    _demand_mm = rest;
    _pause_pos_mm.store(_demand_mm);
    GLOGI(kTag, "PAUSE: braking from %.1f mm/s", double(v));
}

void MotionArbiter::applyTuning(const MotionTuning& t) {
    // A COPY of the live config with the tuning fields replaced, so the
    // limits accept() last set ride through untouched. The engine reads the
    // policy, the amplitude floor (amplitude_budget), the lookahead, the
    // corner and the reaction horizon; the curve policy and the samples
    // latency stay here, at the knot boundary. Every other member is accepted
    // and unread. Takes effect at the next solve, which re-plans every pending
    // knot under it.
    kinetic2::Config c = _engine.config();
    c.policy          = t.infeasible_policy == 0 ? kinetic2::Policy::Stretch : kinetic2::Policy::Blend;
    c.amplitude_floor = t.amplitude_budget;
    c.lookahead_us    = t.lookahead_us;
    c.corner          = t.corner == 1 ? kinetic2::Corner::Cubic : kinetic2::Corner::Continuous;
    c.react_us        = t.react_us;
    _engine.setConfig(c);
    _k2_policy       = c.policy;
    _k2_curve_policy = t.curve_policy;
    _k2_latency_us   = sampleLatencyUs(t);
    _ovr_v = t.vmax_ovr;
    _ovr_a = t.amax_ovr;
    _ovr_j = t.jmax_ovr;
    // Written so a NaN takes the floor: !(x >= floor), not (x < floor).
    _home_speed = !(t.home_speed >= MIN_HOME_SPEED_MM_S) ? MIN_HOME_SPEED_MM_S : t.home_speed;
}

void MotionArbiter::evaluate(uint64_t now_us, float dt_s) {
    // A lapse is the emitter's own stop, kLeaseUs after the last renewal.
    // NEVER re-anchor _p_cmd_mm to the count here: the residual would become
    // feedforward over a dt_s the host measured before the stall (a stall in
    // a solve lands in the tick it delays at ~1 ms), an uncommanded reversal.
    // The residual closes at tick()'s bounded kick.
    const uint32_t lapses = _emitter.lapses();
    if (lapses != _lapses_seen) {
        _lease_lapses += lapses - _lapses_seen;
        _lapses_seen = lapses;
        GLOGW_EVERY_MS(1000, kTag, "LEASE LAPSE: the LP core stopped itself at %.3f mm", double(positionMm()));
    }
    tick(now_us, dt_s);
    _emitter.renew();
}

void MotionArbiter::tick(uint64_t now_us, float dt_s) {
    if (_estop) {
        _brake_req.store(false);   // park already stopped it
        _returning = false;        // estop() dropped override with it
        _emitter.park();
        if (_homing.load()) homeEnd("ESTOP", now_us);
        // ONCE per latch, and on the task that owns the engine: without it the
        // abandoned plan keeps reading busy and canClearEstop() -- which asks
        // exactly that -- would never let the latch drop (SPEC 11.2).
        if (!_estop_settled) {
            resetEngine(toNorm(positionMm()), now_us);
            _p_cmd_mm = positionMm();
            _estop_settled = true;
        }
        return;
    }

    // Unpowered: parked, and the abandoned plan reset ONCE per loss on this
    // task, for the same reason as the e-stop branch above. A pause landing
    // here brakes nothing, but its rest point is still recorded: the parked
    // carriage is where a `return` goes back to (returnToPause()).
    if (!powerGateOpen()) {
        if (_brake_req.exchange(false)) _pause_pos_mm.store(positionMm());
        _returning = false;
        _emitter.park();
        if (_homing.load()) homeEnd("motor power off", now_us);
        if (!_power_settled.exchange(true)) {
            resetEngine(toNorm(positionMm()), now_us);
            _p_cmd_mm = positionMm();
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
        // A cycle counted in the old frame: its seek or backoff is void.
        if (_homing.load()) homeEnd("the travel window changed", now_us);
        resetEngine(toNorm(positionMm()), now_us);
        _p_cmd_mm = positionMm();
        _emitter.steer(0.0f);
        return;
    }

    // A pause ends a cycle, which parks: a seek has no brake to run.
    if (_home_abort.exchange(false) && _homing.load()) homeEnd("paused", now_us);
    if (_brake_req.exchange(false)) brakeToRest(now_us);
    // While a cycle runs the seek producer is the emitter's one steerer and
    // the plan-tracking path below never runs: the cycle owns the rail.
    if (_home_req.exchange(false)) homeStart(now_us);
    if (_home != HomePhase::idle) return homeStep(now_us, dt_s);

    // RETURN (SPEC 11.1): an ordinary Manual plan at the jog set, planned here
    // because the engine is this task's. The rail clamp still applies.
    if (_return_req.exchange(false) && _override.load()) {
        MotionIntent back;
        back.source = MotionSource::Manual;
        float target = _pause_pos_mm.load();
        if (target < 0.0f) target = 0.0f;
        if (target > _rail) target = _rail;
        back.target_mm = target;
        _returning = plan(target, back, limitsFor(true), now_us);
        if (!_returning) GLOGW(kTag, "RETURN: plan failed, override held");
        else GLOGI(kTag, "RETURN: to the paused position %.2f mm", double(target));
    }

    // The one side-effecting sample per tick: it solves the window after a
    // submit and records the brake the engine takes when the timeline runs
    // dry still moving (sampleEngine()).
    // PLANNED CHANGE (bd val-klo): the RFC-103 oscillator (kinetic2/oscillator.hpp)
    // is additive on this sampled state, after the planner and before the
    // backstop, and is not wired yet.
    float p_plan_mm = toMm(sampleEngine(now_us).p);

    // Arrival ends the return: override drops, plain PAUSE stays.
    if (_returning && !_engine.isBusy(now_us)) {
        _returning = false;
        _override.store(false);
        _returns.fetch_add(1);
        GLOGI(kTag, "RETURN: arrived, override off, PAUSE holds");
    }
    // Override gone (return, resume or ESTOP): back to the window frame, at
    // rest.
    if (_rail_frame && !_override.load() && !_engine.isBusy(now_us)) setRailFrame(false, now_us);

    // THE POSITION BACKSTOP (operator ruling 2026-10-06, bd val-1w8). The
    // demand never leaves the backstop's frame (backstopLo()..backstopHi()),
    // whatever the plan does: a curve that bulges, a brake that is not aware
    // of the window, a frame gone stale. The frame widens only to the previous
    // demand, so a carriage left outside it is never pulled in by the clamp,
    // only by a plan. A home cycle never reaches here (homeStep() above): the
    // seek has to reach the stops. The LP core's fence is the second line, and
    // the only one an HP stall cannot skip (syncFence()).
    float lo = backstopLo();
    float hi = backstopHi();
    if (_p_cmd_mm < lo) lo = _p_cmd_mm;
    if (_p_cmd_mm > hi) hi = _p_cmd_mm;
    const bool held = p_plan_mm < lo || p_plan_mm > hi;
    if (held) {
        const float raw = p_plan_mm;
        p_plan_mm = p_plan_mm < lo ? lo : hi;
        // An engagement is a plan more than one step past the edge, counted
        // once at its onset and held until the plan is back inside: a move
        // that lands ON the edge reads a float's rounding past it every time.
        if (!_backstop_on && std::fabs(raw - p_plan_mm) > kMmPerStep) {
            _backstop_on = true;
            ++_backstops;
            GLOGW_EVERY_MS(1000, kTag, "BACKSTOP: plan at %.2f mm held at %.2f mm", double(raw), double(p_plan_mm));
        }
    } else {
        _backstop_on = false;
    }

    // A LATE TICK IS A STALL, NEVER A BURST (bd val-1w8). The kick and the
    // cap below are priced over at most kTickDtCapS, whatever the clock did;
    // what the carriage fell behind during a stall closes at that bounded
    // kick, never as a catch-up.
    const bool stall = dt_s > kTickDtCapS;
    const float dt = stall ? kTickDtCapS : dt_s;
    if (stall) {
        ++_stalls;
        GLOGW_EVERY_MS(1000, kTag, "STALL: tick %.1f ms after the last, kick and cap held to kTickDtCapS",
                       double(dt_s * 1e3f));
    }

    // Feedforward: the plan's OWN mean velocity across the interval that just
    // elapsed. Summed over a move this telescopes to exactly the plan's
    // displacement, which is why the emitter needs no other position input.
    // The mean of a legal curve is under its ceiling over any interval, so a
    // stall renders no burst through it. Into a held edge it is the distance
    // left to the edge, never the plan's displacement past it.
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
        const float kick_max = inputAmaxMm() * dt;
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
    const float v_cap = inputVmaxMm() + inputAmaxMm() * dt;
    if (v >  v_cap) v =  v_cap;
    if (v < -v_cap) v = -v_cap;

    // THE WALL BOUND. The emitter renders v open loop until the next steer, so
    // v may carry the carriage, from position truth, at most to the backstop's
    // edge within kTickDtCapS and never further outside it. Exact for any tick
    // up to kTickDtCapS late; past it the LP core's fence holds the count in
    // the frame, and its lease stops the steer kLeaseUs after the last tick.
    const float pos = positionMm();
    const float v_hi = std::fmax((hi - pos) / kTickDtCapS, 0.0f);
    const float v_lo = std::fmin((lo - pos) / kTickDtCapS, 0.0f);
    if (v > v_hi) v = v_hi;
    if (v < v_lo) v = v_lo;

    syncFence();
    _emitter.steer(v);
}

void MotionArbiter::syncFence() {
    // A home cycle searches a stop past each datum's safety margin by up to
    // two search margins (MotionArbiter.h, homing), in the frame of its last
    // relabel; otherwise the backstop's frame. The emitter orders the two
    // stores.
    const bool seeking = _home != HomePhase::idle;
    const float reach = kHomeSafetyMarginMm + 2.0f * kHomeSearchMarginMm;
    const int32_t lo = _origin + fenceCount(seeking ? -reach : backstopLo(), false);
    const int32_t hi = _origin + fenceCount(seeking ? _max_rail + reach : backstopHi(), true);
    if (lo == _fence_lo && hi == _fence_hi) return;
    _emitter.fence(lo, hi);
    _fence_lo = lo;
    _fence_hi = hi;
}

// ---- homing -----------------------------------------------------------------
// Owning task only, except homeSenseRose(). The seek producer: every leg is a
// constant velocity steered straight onto the emitter, ended by a count or a
// stall, and the engine holds at the count until the cycle ends and is reset
// there. The plan-tracking path, its velocity cap and its residual kick never
// steer during a cycle, and the window clamp never sees one: an approach has
// to reach a stop the window excludes. Positions are the physical frame's
// throughout (MotionArbiter.h, homing): the start is declared at the far
// stop's datum, the home end's stalls take its label, and only the measured
// rail relabels after that. Under the flip the home end is the physical far
// end (SPEC 9.6): the legs swap direction and the physical frame keeps 0 at
// the low end.

void MotionArbiter::homeOrigin(int32_t count, float at_mm, uint64_t now_us) {
    _origin = count - int32_t(std::lround(at_mm * kStepsPerMm));
    resetEngine(toNorm(positionMm()), now_us);
    _p_cmd_mm = positionMm();
}

void MotionArbiter::homeLeg(HomePhase phase, float end_mm, float v_mm_s, uint64_t now_us) {
    _home = phase;
    _leg_end = _origin + int32_t(std::lround(end_mm * kStepsPerMm));
    _leg_dir = _leg_end >= _emitter.count() ? 1 : -1;
    _leg_v = v_mm_s;
    _leg_start_us = now_us;
}

void MotionArbiter::homeSteer(float v_mm_s) {
    _seek_v = v_mm_s;
    syncFence();
    _emitter.steer(v_mm_s);
    // Pairs with homeSenseRose()'s fence: either this read sees the trip, or
    // the interrupt's park lands after the steer above.
    std::atomic_thread_fence(std::memory_order_seq_cst);
    const uint32_t s = _seek.load();
    if (s == kSeekTripping || s == kSeekLatched) {
        _emitter.park();
        _seek_v = 0.0f;
    }
}

bool MotionArbiter::homeDrive(uint64_t now_us, float dt_s) {
    const int32_t left = _leg_dir * (_leg_end - _emitter.count());
    if (left <= 0) {
        _emitter.park();
        _seek_v = 0.0f;
        return true;
    }
    float v = _leg_v;
    if constexpr (kHomeRampMs > 0) {
        const float ramp = float(now_us - _leg_start_us) * 1e-3f / float(kHomeRampMs);
        if (ramp < 1.0f) v *= ramp;
    }
    // The last tick renders the remainder, so a counted leg lands on its count
    // and never past it into the safety margin.
    const float left_mm = float(left) * kMmPerStep;
    if (left_mm < v * dt_s) v = left_mm / dt_s;
    homeSteer(float(_leg_dir) * v);
    return false;
}

void MotionArbiter::homeSeek(bool touch, bool armed, uint64_t now_us) {
    _sense_armed = armed;
    const float end = touch ? homeAway(_home_leg, _home_at_mm, -kHomeSearchMarginMm) : homePast(_home_leg);
    homeLeg(touch ? HomePhase::touch : HomePhase::approach, end, touch ? _touch_v : _home_v, now_us);
    if (armed) _seek.store(kSeekArmed);
}

void MotionArbiter::homeStart(uint64_t now_us) {
    if (!_homing.load()) return;   // ended between the request and this tick
    _home_leg = 0;
    if (_engine.isBusy(now_us)) return homeEnd("the machine is moving", now_us);
    _home_flip = _flipped.load();
    // The position reference is replaced from here: a cycle that does not
    // finish leaves the machine unhomed, never homed at a stale origin.
    _homed = false;
    _home_v = _jog_v < _home_speed ? _jog_v : _home_speed;
    _touch_v = homeTouchMmS(_home_v);
    _rail = _max_rail;
    _home_deadline_us = now_us + uint64_t(homeCycleS(_max_rail, _home_v) * 1e6f) + kHomeTimeoutMarginUs;
    // The carriage is declared at the far stop's datum: max_rail plus both
    // margins from the home stop's.
    homeOrigin(_emitter.count(), homeAway(0, homeStopAt(0), _max_rail + 2.0f * kHomeSafetyMarginMm), now_us);
    GLOGI(kTag, "HOME: approaching the home end at %.1f mm/s (re-touch %.1f mm/s), searching %.0f mm",
          double(_home_v), double(_touch_v),
          double(_max_rail + 2.0f * (kHomeSafetyMarginMm + kHomeSearchMarginMm)));
    homeSeek(false, true, now_us);   // home() probed the line LOW
}

void MotionArbiter::homeStep(uint64_t now_us, float dt_s) {
    if (now_us >= _home_deadline_us) return homeEnd("timed out", now_us);
    switch (_home) {
        case HomePhase::approach:
        case HomePhase::touch: {
            const bool touch = _home == HomePhase::touch;
            const uint32_t s = _seek.load(std::memory_order_acquire);
            // The interrupt is between its park and its latch: parked, and
            // read on the next tick.
            if (s == kSeekTripping) return;
            if (s == kSeekLatched) {
                // The interrupt parked on the rise; the confirming read decides.
                if (_sense->high()) {
                    _seek.store(kSeekIdle);
                    return homeContact(touch, _seek_hit.load(std::memory_order_relaxed), now_us);
                }
                _seek.store(kSeekArmed);
                GLOGW_EVERY_MS(1000, kTag, "HOME: a rise that did not confirm parked the seek for a tick");
            } else if (_sense->high()) {
                // A rise the interrupt did not trip on (it raced the arming,
                // or the host has no interrupt): park on this tick.
                if (_sense_armed) {
                    const uint32_t was = _seek.exchange(kSeekIdle);
                    _emitter.park();
                    const int32_t hit =
                        was == kSeekLatched ? _seek_hit.load(std::memory_order_relaxed) : _emitter.count();
                    return homeContact(touch, hit, now_us);
                }
            } else if (!_sense_armed) {
                _sense_armed = true;
                _seek.store(kSeekArmed);
            }
            if (homeDrive(now_us, dt_s))
                return homeEnd(touch ? "no stall on the slow re-touch" : "no stall across max_rail", now_us);
            return;
        }
        case HomePhase::clear:
            // The level at rest decides.
            if (!homeDrive(now_us, dt_s)) return;
            if (_sense->high()) return homeEnd("still on the stop after the backoff (lower home_speed)", now_us);
            return homeSeek(true, true, now_us);
        case HomePhase::finish:
            if (!homeDrive(now_us, dt_s)) return;
            if (_sense->high())
                return homeEnd("still on the far stop after the final backoff (lower home_speed)", now_us);
            _homed = true;
            _max_rail = _rail;
            _home_rail_mm = _rail;
            ++_homes;
            homeEnd(nullptr, now_us);
            GLOGI(kTag, "HOME: homed, rail %.2f mm between the %.1f mm safety margins", double(_rail),
                  double(kHomeSafetyMarginMm));
            return;
        case HomePhase::idle:
            return;
    }
}

void MotionArbiter::homeContact(bool touch, int32_t hit, uint64_t now_us) {
    _seek_v = 0.0f;
    _home_hit = hit;
    if (!touch) {
        // The home end's stall takes its label at once, so a cycle that ends
        // from here on leaves positions measured from the home datum. A far
        // stall is only read: the home datum keeps its label.
        if (_home_leg == 0) homeOrigin(hit, homeStopAt(0), now_us);
        _home_at_mm = float(hit - _origin) * kMmPerStep;
        return homeLeg(HomePhase::clear, homeAway(_home_leg, _home_at_mm, kHomeSafetyMarginMm), _home_v, now_us);
    }
    // The contact came kHomeSenseLatencyUs of re-touch travel before the
    // sense did: the datum is moved back along the leg by it.
    const int32_t lag = int32_t(std::lround(_touch_v * float(kHomeSenseLatencyUs) * 1e-6f * kStepsPerMm));
    _home_datum[_home_leg] = hit + (homeTowardLow(_home_leg) ? lag : -lag);
    if (_home_leg == 1) return homeMeasure(now_us);
    homeOrigin(_home_datum[0], homeStopAt(0), now_us);
    _home_leg = 1;
    GLOGI(kTag, "HOME: home datum taken, approaching the far end");
    homeSeek(false, false, now_us);   // still pressed on the home stop
}

void MotionArbiter::homeMeasure(uint64_t now_us) {
    const int32_t steps = _home_datum[1] - _home_datum[0];
    const float stops = float(steps < 0 ? -steps : steps) * kMmPerStep;
    const float len = stops - 2.0f * kHomeSafetyMarginMm;
    if (!(len >= MIN_RAIL_MM))
        return homeEnd("the usable rail between the safety margins is under the rail floor", now_us);
    // The physical frame: 0 a safety margin off the low end's datum, the rail
    // the usable length. A completed cycle makes it max_rail too; the hub
    // stores it and pushes it back (setWindow()).
    _rail = len;
    homeOrigin(_home_flip ? _home_datum[1] : _home_datum[0], -kHomeSafetyMarginMm, now_us);
    homeLeg(HomePhase::finish, _home_flip ? 0.0f : len, _home_v, now_us);
}

void MotionArbiter::homeEnd(const char* why, uint64_t now_us) {
    _seek.store(kSeekIdle);
    if (why != nullptr) {
        ++_home_fails;
        _home_fail_leg = _home_leg;
        _home_fail_why = why;
        GLOGI(kTag, "HOME failed at the %s: %s", _home_leg == 0 ? "home end" : "far end", why);
    }
    // A cycle that ran stops where it is and the planner starts again from
    // the count, as after an e-stop. One refused at its start never steered:
    // whatever moves is the planner's own and is left alone.
    if (_home != HomePhase::idle) {
        _emitter.park();
        _seek_v = 0.0f;
        resetEngine(toNorm(positionMm()), now_us);
        _p_cmd_mm = positionMm();
    }
    _home = HomePhase::idle;
    _home_req.store(false);
    _homing.store(false);
}

// Drains the engine's anomaly ring into the per-kind table 0x1111 publishes.
// A kind past the table is DROPPED rather than folded into a neighbor: a
// miscounted kind reads as a diagnosis that never happened.
uint32_t MotionArbiter::drainAnomalies() {
    uint32_t kinds = 0;
    EngineAnomaly a;
    while (_engine.popAnomaly(a)) {
        if (a.kind < kAnomalyKinds) {
            ++_anom[a.kind];
            kinds |= 1u << a.kind;
        }
        ++_anomalies;
        // A failure is a dropped knot or a refused one.
        if (a.kind == uint8_t(kinetic2::AnomalyKind::PlanFailed) || a.kind == uint8_t(kinetic2::AnomalyKind::KnotRefused))
            ++_k2_failures;
    }
    return kinds;
}

MotionArbiter::PlanRead MotionArbiter::readPlan(uint64_t now_us) {
    PlanRead r;
    const kinetic2::State st = sampleEngine(now_us);
    r.pos      = st.p;
    r.vel      = st.v;
    r.start    = st.p;
    r.target   = st.p;
    r.plans    = _k2_plans;
    r.failures = _k2_failures;
    if (now_us < _k2_brake_to_us) {
        // A brake renders: PAUSE, a generator's stop, a starved stream.
        r.mode       = uint8_t(PlanStyle::settle);
        r.start      = _k2_brake_from_p;
        r.target     = _k2_brake_to_p;
        r.duration_s = float(_k2_brake_to_us - _k2_brake_from_us) * 1e-6f;
        r.elapsed_s  = now_us > _k2_brake_from_us ? float(now_us - _k2_brake_from_us) * 1e-6f : 0.0f;
    } else if (_engine.pending() > 0) {
        // The piece toward the first pending knot, as the solver decided it.
        const kinetic2::Solved& k = _engine.solved(0, 0);
        r.mode       = uint8_t(_k2_chase ? PlanStyle::chase : PlanStyle::waveform);
        r.plan_kind  = kPlanKindQuintic;
        r.start      = k.from.p;
        r.target     = k.p;
        r.duration_s = float(k.t_us - k.from_us) * 1e-6f;
        r.elapsed_s  = now_us > k.from_us ? float(now_us - k.from_us) * 1e-6f : 0.0f;
        // RFC-100 from the solver: fallback is never set, nothing falls back.
        if (k.share < 1.0f) r.flags |= plan_flags::shaped;
        if (k.stretched_s > 0.0f) r.flags |= plan_flags::stretched;
        if (k.clamped || _k2_window_clamped) r.flags |= plan_flags::clamped;
    }
    return r;
}

MotionCensus MotionArbiter::snapshot(uint64_t now_us) {
    const PlanRead s = readPlan(now_us);
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

    // While a cycle runs the seek producer is the plan: its steered velocity,
    // the count as the plan position, and its leg's end as the target. The
    // engine only holds at the cycle's last re-anchor.
    const bool seeking = _home != HomePhase::idle;
    const float vel_mm_s = seeking ? _seek_v : s.vel * s_mm;
    if (std::fabs(vel_mm_s) > _peak_mm_s) _peak_mm_s = std::fabs(vel_mm_s);

    MotionCensus c{};
    c.steps          = steps - _origin;
    c.position_mm    = float(c.steps) * kMmPerStep;
    c.plan_mm        = seeking ? c.position_mm : toMm(s.pos);
    c.target_mm      = seeking ? float(_leg_end - _origin) * kMmPerStep : toMm(s.target);
    c.velocity_mm_s  = vel_mm_s;
    c.residual_steps = static_cast<int32_t>(std::lround((c.plan_mm - c.position_mm) * kStepsPerMm));
    c.win_min        = _win_min;
    c.win_max        = _win_max;
    c.rail_mm        = _rail;
    c.input_vmax_mm_s  = inputVmaxMm();
    c.input_amax_mm_s2 = inputAmaxMm();
    c.intents        = _intents;
    c.rejected       = _rejected;
    c.stalls         = _stalls;
    c.backstops      = _backstops;
    c.lease_lapses   = _lease_lapses;
    c.homed          = _homed;
    c.estop          = _estop;
    c.motor_on       = _powered.load();
    c.power_gate     = powerGateOpen();
    c.paused         = _paused.load();
    c.override_mode  = _override.load();
    c.returning      = _returning;
    c.returns        = _returns.load();
    c.homing         = _homing.load();
    c.homes          = _homes;
    c.home_rail_mm   = _home_rail_mm;
    c.home_fails     = _home_fails;
    c.home_fail_leg  = _home_fail_leg;
    c.home_fail_why  = _home_fail_why;
    c.busy           = seeking || _engine.isBusy(now_us);
    c.mode           = s.mode;
    c.plan_kind      = s.plan_kind;
    c.plan_start     = s.start;
    c.plan_end       = s.target;
    c.plan_cur       = s.pos;
    c.plan_vel       = s.vel;
    c.plan_duration_us = uint32_t(s.duration_s * 1e6f);
    c.plan_elapsed_us  = uint32_t(s.elapsed_s * 1e6f);
    // An RFC-095 dwell is one of these: the generator sends it as a hold
    // segment so a client reads a live plan, never a stalled source.
    c.plan_hold = c.busy && s.mode == uint8_t(PlanStyle::waveform) &&
                  std::fabs(s.target - s.start) < limits::segment_dwell_span;
    // A SETTLE brake is the engine's own plan, never a bent command; the
    // backstop holding the demand at the frame's edge is one, whatever runs.
    c.plan_flags = c.busy && s.mode != uint8_t(PlanStyle::settle) ? s.flags : 0;
    if (_backstop_on) c.plan_flags |= plan_flags::clamped;
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
    // The flip's way out: every position and window in the mirrored frame,
    // every velocity reversed. steps stays the emitter's own count.
    if (_flipped.load()) {
        c.position_mm    = _rail - c.position_mm;
        c.plan_mm        = _rail - c.plan_mm;
        c.target_mm      = _rail - c.target_mm;
        c.demand_mm      = _rail - c.demand_mm;
        c.velocity_mm_s  = -c.velocity_mm_s;
        c.residual_steps = -c.residual_steps;
        c.win_min        = _rail - _win_max;
        c.win_max        = _rail - _win_min;
        c.plan_start     = 1.0f - c.plan_start;
        c.plan_end       = 1.0f - c.plan_end;
        c.plan_cur       = 1.0f - c.plan_cur;
        c.plan_vel       = -c.plan_vel;
    }
    return c;
}

// ---- factory tuning ---------------------------------------------------------

EngineConfig MotionArbiter::engineConfig() {
    return kinetic2::Config{};
}

MotionTuning motionDefaultTuning() {
    MotionTuning t;
    // Applied at the knot boundary, not by the engine: the samples grant's
    // latency (sampleLatencyUs()) and the curve policy. Both are published
    // with their catalog default (0x1122), so they never change here alone.
    t.chase_dense_us   = 60000;
    t.curve_policy     = 0;   // follow client
    // The members the engine reads take its factory values (applyTuning()).
    // The catalog select is 0 stretch / 1 blend, so the mapping is explicit
    // rather than a cast.
    const kinetic2::Config k2{};
    t.infeasible_policy = k2.policy == kinetic2::Policy::Stretch ? 0 : 1;
    t.amplitude_budget  = k2.amplitude_floor;
    t.lookahead_us      = k2.lookahead_us;
    t.corner            = uint8_t(k2.corner);
    t.react_us          = k2.react_us;
    return t;
}

}  // namespace valence
