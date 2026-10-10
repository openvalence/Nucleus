// MotionArbiter -- the gates, the clamp, the limit sets and the feedforward,
// compiled by the board and by the host twin alike
// Constraints:
// - HARDWARE-FREE (MotionArbiter.h). Geiger is the log path and binds its own
//   platform layer; on a host it is mute, never a second log path.
// - An intent becomes knots AT ARRIVAL, continuing from the engine's actual
//   (p, v, a); the window is solved at the next sample, on the planner, and
//   the steer only reads the strip. Nothing here plans on a clock.
// See: MotionArbiter.h, .claude/rules/motion-control.md, bd val-091.4, bd val-z1k

#include "MotionArbiter.h"

#include <algorithm>
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

// A late plan is counted past one step and logged past ten: a gap of a step
// or two is the ordinary re-plan seam (bd val-alf).
constexpr float kLatePlanLogMm = 10.0f * kMmPerStep;

bool isGenerator(MotionSource s) { return s == MotionSource::Pattern || s == MotionSource::Advanced; }

[[maybe_unused]] const char* sourceName(uint8_t id) {
    return id < kMotionSourceNames.size() ? kMotionSourceNames[id] : "none";
}

static_assert(kAnomalyKinds <= 32, "drainAnomalies() reports the kinds as one word");

// Option ordinals of the 0x1110 style and 0x1111 mode selects and of the
// 0x1111 plan_kind select (ValenceCatalog.h). Wire values: never renumber.
enum class PlanStyle : uint8_t { idle = 0, waveform = 1, chase = 2, settle = 3 };
constexpr uint8_t kPlanKindBezier = 1;

class AbsentHomeSense final : public HomeSense {
public:
    bool present() const override { return false; }
    Probe probe() override { return Probe::undriven; }
    bool high() override { return false; }
};

AbsentHomeSense g_absent_sense;

// The oscillator's shapes are the registry's osc_shapes numbers on the wire.
static_assert(uint8_t(kinetic2::OscShape::Sine) == osc_shapes::sine &&
                  uint8_t(kinetic2::OscShape::Square) == osc_shapes::square &&
                  uint8_t(kinetic2::OscShape::Saw) == osc_shapes::saw &&
                  uint8_t(kinetic2::OscShape::SawReverse) == osc_shapes::saw_reverse,
              "kinetic2::OscShape is the registry's osc_shapes");

static_assert(MotionTuning{}.smoothness == kinetic2::Config{}.smoothness &&
                  MotionTuning{}.handle_floor == kinetic2::Config{}.handle_floor &&
                  MotionTuning{}.trim_max == kinetic2::Config{}.trim_max &&
                  MotionTuning{}.react_us == kinetic2::Config{}.react_us,
              "MotionTuning's Kinetic² defaults are kinetic2::Config's");

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
    // the planner, which resets it on its next tick (planStep()).
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
    if (generator == MotionSource::Stream) {
        _stream_quiet.store(true);
        return;
    }
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
    // position, which planTick() records at rest even while unpowered.
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
    // The flag before the post: raised after it, a planner that applied the
    // post in between would see the flag again and move the frame twice.
    _frame_moved = true;
    _win_seq.fetch_add(1);   // odd: being written
    _win_req_lo.store(lo);
    _win_req_hi.store(hi);
    _win_req_rail.store(rail > 0.0f ? rail : DEFAULT_MAX_RAIL_MM);
    _win_seq.fetch_add(1);   // even: whole
}

bool MotionArbiter::takeWindow() {
    // Mid-write or nothing new: the next pass takes it. Never spin here, the
    // writer may be the task this one preempted.
    const uint32_t seq = _win_seq.load();
    if ((seq & 1u) != 0 || seq == _win_taken) return false;
    const float lo   = _win_req_lo.load();
    const float hi   = _win_req_hi.load();
    const float rail = _win_req_rail.load();
    if (_win_seq.load() != seq) return false;   // rewritten while read
    _win_taken = seq;
    // The flag before the fields, fenced: a steer preempting between them
    // parks instead of reading a window half old and half new.
    _frame_moved = true;   // normalized units now mean different millimeters
    std::atomic_thread_fence(std::memory_order_seq_cst);
    _win_min  = lo;
    _win_max  = hi;
    _rail     = rail;
    _max_rail = rail;
    return true;
}

void MotionArbiter::setOscillator(const MotionOsc& o) {
    _osc_seq.fetch_add(1);   // odd: being written
    _osc_req_on.store(o.enabled);
    _osc_req_shape.store(o.shape);
    _osc_req_hz.store(o.frequency_hz);
    _osc_req_amp.store(o.amplitude);
    _osc_req_crest.store(o.dwell_crest);
    _osc_req_trough.store(o.dwell_trough);
    _osc_seq.fetch_add(1);   // even: whole
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
    _origin_moved = true;  // before the flag: the planner resets on it, never reseeds
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
    // An intent sent after a window write is mapped by the hub through that
    // window: it clamps and plans in it. The engine moves to it at the next
    // planTick(), before any strip fills.
    takeWindow();
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
    // The ceiling set the plan in flight was planned under: steerTick()'s cap.
    // A jog that stops the motion in flight first takes the jog set's when
    // that brake ends (planStep()): capped at 50 mm/s, a 1200 mm/s brake
    // was a one-tick stop at the emitter.
    if (in.source != MotionSource::Manual) _jog_after_us = 0;
    _plan_manual.store(in.source == MotionSource::Manual && _jog_after_us == 0);
    return true;
}

void MotionArbiter::notePlanCost(uint32_t us) {
    _plan_us_last = us;
    if (us > _plan_us_max) _plan_us_max = us;
    _plan_us_avg += (float(us) - _plan_us_avg) * 0.125f;
}

void MotionArbiter::resetEngine(float p_norm, uint64_t now_us) {
    _engine.resetAt(p_norm, now_us);   // clears the expectation
    _k2_expect_us   = 0;
    _k2_newest_us   = now_us;
    _k2_newest_p    = p_norm;
    _k2_brake_to_us = 0;
    _k2_starved     = false;
    _k2_chase       = false;
    _plan_read = PlanRead{};
    _plan_read.pos = _plan_read.start = _plan_read.target = p_norm;
    _plan_busy = false;
    _seg_read    = SegRead{p_norm, p_norm, now_us, now_us, 0};
    _seg_eng_us  = now_us;
    _seg_through = false;
    // The steer re-anchors its feedforward here and steers nothing from a
    // strip published before this reset (steerTick()); the oscillation stops
    // with the plan it rode.
    _anchor_mm   = positionMm();
    _osc.reset();
    _osc_head = 0.0f;
    _strip_continuous = false;
    _strip_stale = true;
    _strip_gen.fetch_add(1);
    _eng_lo   = frameLo();
    _eng_span = span();
}

void MotionArbiter::reseedEngine(uint64_t now_us) {
    // The plan's own curve, not the count: the engine restates it and keeps
    // it through the reaction horizon or the next knot (Engine::reframe,
    // RFC-105 (bb)), so the strip the steer follows runs on in mm and the
    // carriage's lag stays the kick's to close, as through any re-plan.
    // Never a re-plan from now: a write inside a knot's horizon trims that
    // knot to rest on the write's state (bd val-4dt, val-83q). Ceilings scale
    // with the curve, so it keeps its mm ceilings until the next intent sets
    // the new frame's.
    (void)sampleEngine(now_us);
    const float k = _eng_span / span();
    const float c = (_eng_lo - frameLo()) / span();
    auto restate = [&](float p) { return p * k + c; };
    EngineLimits lim = _engine.config().limits;
    lim.vmax *= k;
    lim.amax *= k;
    lim.jmax *= k;
    _engine.setLimits(lim);
    _engine.reframe(k, c, now_us);
    _k2_dirty = true;
    _k2_brake_from_p = restate(_k2_brake_from_p);
    _k2_brake_to_p   = restate(_k2_brake_to_p);
    _seg_read.from_p = restate(_seg_read.from_p);
    _seg_read.to_p   = restate(_seg_read.to_p);
    // The plan the strip was cut from, and the oscillation in millimeters:
    // its amplitude is a window share, so it moves to the new window's
    // through its fade, never in a step.
    for (float& p : _plan_ext) p = restate(p);
    _osc.rescale(k);
    if (_engine.pending() == 0) {
        const kinetic2::Knot h = _engine.newest();
        _k2_newest_us = h.t_us;
        _k2_newest_p  = h.p;
    }
    _strip_continuous = true;
    _strip_stale = true;
    _eng_lo   = frameLo();
    _eng_span = span();
}

bool MotionArbiter::brakeEngine(uint64_t at_us) {
    // The engine stops from its own state at at_us and reports neither the
    // profile's end nor its length: recomputed here from the same state and
    // limits by the same function, so the census reads exactly what renders.
    const kinetic2::State s = _engine.stateAt(0, at_us);
    const kinetic2::Profile pr = kinetic2::Profile::brake(s, at_us, _engine.config().limits);
    _engine.brake(at_us);
    setExpect(0);
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

void MotionArbiter::setExpect(uint64_t until_us) {
    _engine.expect(0, until_us);
    _k2_expect_us = until_us;
}

// ---- the Kinetic² boundary ----------------------------------------------------
// Every conversion from an intent to the engine's knots (bd val-klo, RFC-105).
// The engine sees knots, a brake and resets; nothing here reads a wire format.

bool MotionArbiter::submitKnots(float p, const MotionIntent& in, const EngineLimits& lim, uint64_t now_us) {
    const bool manual = in.source == MotionSource::Manual;
    // The engine solves its whole pending window under the config it holds at
    // the next sample, so these limits apply to every knot still pending, not
    // to this one alone.
    kinetic2::Config cfg = _engine.config();
    cfg.limits = lim;
    _engine.setConfig(cfg);

    // At rest the state is the emitter's count and nothing else: a reseed from
    // the engine's own idea of where it stopped carries every move's sub-step
    // residue into the next one. pending() first: isBusy() solves, and a
    // bundle must not solve once per sample.
    if (_engine.pending() == 0 && !_engine.isBusy(now_us)) resetEngine(toNorm(positionMm()), now_us);

    // Engine::expect, after the reset above (a reset clears it): a Stream
    // segment's free knot renders through while more segments are expected;
    // every other intent (a sample, a jog, a generator taking the rail) ends
    // the expectation. Never a sample's: the kernel ignores samples.
    const bool streamed = in.source == MotionSource::Stream && in.duration_us > 0 && in.expect_us > 0;
    setExpect(streamed ? now_us + in.expect_us : 0);

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
        // A start within a tick of the newest knot IS that knot: the sender
        // tiled its spans and two of its clock reads moved the stamp by
        // microseconds. Held there, a span ending moving stopped dead in one
        // tick (PieceOverCeiling); flushed there, the knot the bundle meant to
        // keep was replaced (Kinetic kin-554).
        const bool tiles = start < _k2_newest_us + kMotionTickUs && _k2_newest_us < start + kMotionTickUs;
        // RFC-087 supersede: a bundle replaces every knot queued at or after
        // its first start; the motion in flight hands off there, or at the
        // reaction horizon when the start is not past it
        // (Engine::truncateAfter). The start is then never a rest.
        if (in.supersede && !tiles && _engine.truncateAfter(start, now_us) > 0) {
            const kinetic2::Knot h = _engine.newest();
            _k2_newest_us = start > h.t_us ? start : h.t_us;
            _k2_newest_p  = h.p;
            _k2_dirty = true;
        }
        if (!tiles && start > _k2_newest_us) {
            kinetic2::Knot hold;
            hold.t_us   = start;
            hold.p      = _k2_newest_p;
            hold.has_v  = true;
            hold.v      = 0.0f;
            if (!_engine.submit(hold, now_us)) return false;
            _k2_newest_us = start;
            _k2_dirty = true;
        }
        k = kinetic2::knotFromSegment(p, in.duration_us, in.has_end_vel, in.end_vel_mm_s / span(), start);
        _k2_chase = false;
    } else if (manual) {
        // Motion the jog set did not plan (a stream, a pattern, segments) is
        // stopped first as PAUSE stops it: the engine's brake at the input
        // set, and the jog chains from its end. The jog set cannot stop it:
        // from 1200 mm/s at 200 mm/s^2 its fastest stop runs 3.6 m (val-hlj).
        // The plan in flight is read under the input set it was planned
        // under, whether or not a sample solved it since the last intent.
        _jog_after_us = 0;
        _engine.setLimits(limitsFor(false));
        if (!_plan_manual.load() && _engine.isBusy(now_us)) {
            brakeEngine(now_us);
            _jog_after_us = _k2_brake_to_us;
        }
        _engine.setConfig(cfg);
        // A jog is LIVE (operator ruling 2026-10-06, RFC-105 (n) amended): the
        // newest target supersedes every move still queued; the motion in
        // flight hands off at the reaction horizon (Engine::truncateAfter) and
        // the new move chains from there. A scrub never builds a queue.
        if (_engine.truncateAfter(now_us, now_us) > 0) {
            const kinetic2::Knot h = _engine.newest();
            _k2_newest_us = h.t_us;
            _k2_newest_p  = h.p;
            _k2_dirty = true;
        }
        // The knot is due as soon as possible: a sample at rest is the HARD
        // junction (types.hpp junctionOf), which the engine renders as the
        // time-optimal move to rest from the newest knot's state
        // (Profile::point: velocity change, cruise, brake), stretched, never
        // trimmed: a jog that lands short is a wrong answer. It times itself,
        // with no anomaly for a sample. A deadline padded here only slowed
        // the cruise to meet it (RFC-105 (xx)).
        const uint64_t from = _k2_newest_us > now_us ? _k2_newest_us : now_us;
        k = kinetic2::knotFromSample(p, from, kMotionTickUs);
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

void MotionArbiter::takeOscillator() {
    // Mid-write or nothing new: the next tick takes it, never a spin.
    const uint32_t seq = _osc_seq.load();
    if ((seq & 1u) != 0 || seq == _osc_taken) return;
    kinetic2::OscParams p;
    p.enabled = _osc_req_on.load();
    const uint8_t shape = _osc_req_shape.load();
    p.shape = kinetic2::OscShape(shape <= uint8_t(kinetic2::OscShape::SawReverse) ? shape : 0);
    // Written so a NaN reads 0: the delegate clamped, this is the backstop.
    const float hz = _osc_req_hz.load(), amp = _osc_req_amp.load();
    p.frequency = !(hz > 0.0f) ? 0.0f : std::fmin(hz, OSC_MAX_HZ);
    // A window share is the engine's unit in the window frame.
    p.amplitude = !(amp > 0.0f) ? 0.0f : std::fmin(amp, 1.0f);
    p.dwell_crest = _osc_req_crest.load();
    p.dwell_trough = _osc_req_trough.load();
    if (_osc_seq.load() != seq) return;   // rewritten while read
    _osc_taken = seq;
    _osc.set(p);
}

kinetic2::State MotionArbiter::sampleEngine(uint64_t now_us) {
    // The first sample after a submit solves the window: that is the plan's
    // cost, so it is what plan_us_* times. The plan changed: the strip
    // refills whole.
    const bool timed = _k2_dirty;
    if (timed) _strip_stale = true;
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
}

EngineLimits MotionArbiter::limitsFor(bool manual) const {
    const float s = span();
    EngineLimits lim;
    lim.vmax = manual ? _jog_v / s : inputVmaxMm() / s;
    lim.amax = manual ? _jog_a / s : inputAmaxMm() / s;
    lim.jmax = _ovr_j > 0.0f ? _ovr_j : _in_j / s;
    return lim;
}

// ---- the planner's tick -----------------------------------------------------

// SPEC 11.1 PAUSE, planner task only: the engine's own brake, planned from the
// plan's (p, v, a) -- continuous with what the emitter is rendering, unlike a
// census read -- at the input decel, and never a reversal. It also drops every
// pending knot (Engine::brake).
// The rest point is recorded as the paused position, the one place a
// `return` goes back to.
void MotionArbiter::brakeToRest(uint64_t now_us) {
    _jog_after_us = 0;   // a jog waiting out a brake is dropped with it
    _engine.setLimits(limitsFor(false));
    const kinetic2::State st = sampleEngine(now_us);
    [[maybe_unused]] const float v = st.v * span();   // log only
    // The oscillation stops with the motion (SPEC 9.7): this tick's refill
    // carries the cut, and an idle carriage comes to rest on the plan.
    _osc_cut = _osc.active();
    _osc.reset();
    if (!brakeEngine(now_us)) {
        _pause_pos_mm.store(_osc_cut ? toMm(st.p) : positionMm());
        return;
    }
    // Where it comes to rest: the brake's end, held by the backstop
    // (steerTick()) when the brake would carry it past the frame's edge.
    // _p_cmd_mm is the steer's, read as one word.
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
    // limits accept() last set ride through untouched. The engine reads
    // smoothness, handle_floor, trim_max and the reaction horizon; the samples
    // latency stays here, at the knot boundary. Takes effect at the next
    // solve, which re-plans every knot not yet committed under it.
    kinetic2::Config c = _engine.config();
    c.smoothness   = t.smoothness;
    c.handle_floor = t.handle_floor;
    c.trim_max     = t.trim_max;
    c.react_us     = t.react_us;
    _engine.setConfig(c);
    _k2_latency_us = sampleLatencyUs(t);
    _ovr_v = t.vmax_ovr;
    _ovr_a = t.amax_ovr;
    _ovr_j = t.jmax_ovr;
    // Written so a NaN takes the floor: !(x >= floor), not (x < floor).
    _home_speed = !(t.home_speed >= MIN_HOME_SPEED_MM_S) ? MIN_HOME_SPEED_MM_S : t.home_speed;
}

void MotionArbiter::planTick(uint64_t now_us, float dt_s) { fillStrip(now_us, planStep(now_us, dt_s)); }

bool MotionArbiter::planStep(uint64_t now_us, float dt_s) {
    // FIRST, before any reset, reseed or fill: the frame changes only here
    // and in accept(), so every strip is built in one frame (bd val-8es).
    takeWindow();
    takeOscillator();
    if (_estop) {
        _brake_req.store(false);   // park already stopped it
        _returning = false;        // estop() dropped override with it
        // The steer parks too while the latch holds: the same store twice.
        _emitter.park();
        if (_homing.load()) homeEnd("ESTOP", now_us);
        // ONCE per latch, and on the task that owns the engine: without it the
        // abandoned plan keeps reading busy and canClearEstop() -- which asks
        // exactly that -- would never let the latch drop (SPEC 11.2).
        if (!_estop_settled) {
            resetEngine(toNorm(positionMm()), now_us);
            _estop_settled = true;
        }
        return false;
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
        if (!_power_settled.exchange(true)) resetEngine(toNorm(positionMm()), now_us);
        return false;
    }

    // A FRAME MOVE IS NOT MOTION. force_home re-origins the count and a window
    // change rescales normalized units, so both make plan_mm and position_mm
    // jump by up to the whole rail while the carriage stands still. Left alone
    // the feedforward differences that jump and demands 10^5 mm/s, the emitter
    // floor clamps it, and the emitter renders a saturated burst -- unrequested
    // travel at its maximum rate, plus thousands of missed deadlines
    // (measured, bd val-091.13). A window change with motion planned
    // re-targets in place (bd val-17u): the pending knots are window shares
    // and stay, and the plan's own curve is restated in the new frame
    // (reseedEngine()), so the strip runs on in mm. force_home, a home cycle,
    // or a carriage at rest with nothing pending resets at the carriage: the
    // steer parks for this tick (an empty strip) and re-anchors at the reset,
    // and the next accepted intent plans from the new frame's rest. A rest
    // the oscillator is riding reseeds instead, so the oscillation runs on.
    // The generation moves BEFORE the flag drops, so no strip of the old
    // frame steers in between.
    if (_frame_moved) {
        _strip_gen.fetch_add(1);
        _frame_moved = false;
        const bool origin_moved = _origin_moved;
        _origin_moved = false;
        // A cycle counted in the old frame: its seek or backoff is void.
        const bool homing = _homing.load();
        if (homing) homeEnd("the travel window changed", now_us);
        if (origin_moved || homing || (_engine.pending() == 0 && !_engine.isBusy(now_us) && !_osc.active())) {
            resetEngine(toNorm(positionMm()), now_us);
            return false;
        }
        reseedEngine(now_us);
    }

    // A pause ends a cycle, which parks: a seek has no brake to run.
    if (_home_abort.exchange(false) && _homing.load()) homeEnd("paused", now_us);
    if (_brake_req.exchange(false)) brakeToRest(now_us);
    // The stream's quiet release: nothing more is coming. An expectation
    // already rendered into a pending knot is fixed at its solve, so the
    // window re-solves without it and the newest knot lands at rest instead
    // of being passed moving and braked past.
    if (_stream_quiet.exchange(false) && _k2_expect_us != 0) {
        setExpect(0);
        if (_engine.pending() > 0) reseedEngine(now_us);
    }
    // While a cycle runs the seek producer is the emitter's one steerer and
    // renews its lease here; the steer stands aside (steerTick()).
    if (_home_req.exchange(false)) homeStart(now_us);
    if (_home != HomePhase::idle) {
        homeStep(now_us, dt_s);
        if (_home != HomePhase::idle) _emitter.renew();
        return false;
    }

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
    // dry still moving (sampleEngine()). The strip (fillStrip()) reads the
    // plan after it without side effects, and sums the oscillator into it.
    const kinetic2::State st = sampleEngine(now_us);
    const bool busy = _engine.isBusy(now_us);
    _plan_read = readPlan(st, now_us);
    _plan_busy = busy;
    // The jog set's ceiling holds until the plan planned under it has ended.
    if (!busy) {
        _plan_manual.store(false);
        _jog_after_us = 0;
    } else if (_jog_after_us != 0 && now_us >= _jog_after_us) {
        _plan_manual.store(true);
        _jog_after_us = 0;
    }

    // Arrival ends the return: override drops, plain PAUSE stays.
    if (_returning && !busy) {
        _returning = false;
        _override.store(false);
        _returns.fetch_add(1);
        GLOGI(kTag, "RETURN: arrived, override off, PAUSE holds");
    }
    // Override gone (return, resume or ESTOP): back to the window frame, at
    // rest.
    if (_rail_frame && !_override.load() && !busy) setRailFrame(false, now_us);
    return true;
}

void MotionArbiter::fillStrip(uint64_t now_us, bool live) {
    PlanStrip& s = _strip;
    s.gen = _strip_gen.load();
    s.anchor_mm = _anchor_mm;
    s.continuous = _strip_continuous;
    s.cut = false;
    constexpr uint64_t kSpanUs = uint64_t(kStripLen) * kMotionTickUs;
    // The plan from t0 on, engine frame, in _plan_ext after the kOscEdge
    // ticks before t0. Read this far ahead only while the oscillator renders:
    // its look-ahead past the strip, and its edge.
    float* plan = _plan_ext.data() + kOscEdge;
    const size_t ahead = _osc.idle() ? kStripLen : kPlanExt - kOscEdge;
    if (!live) {
        s.n = 0;
        _osc.reset();
        _osc_head = 0.0f;
    } else if (_strip_stale || s.n == 0 || now_us < s.t0_us || now_us - s.t0_us >= kSpanUs ||
               ahead > _plan_ahead) {
        // A whole refill: one walk of the window, one piece per knot interval.
        // The ticks before t0 are what the last strip planned there, for the
        // oscillator's differences; after a reset, the plan's own start.
        std::array<float, kOscEdge> before{};
        for (size_t i = 0; i < kOscEdge; ++i) {
            const float at = float(int64_t(now_us) - int64_t(s.t0_us)) / float(kMotionTickUs) + float(i);
            const float x = std::fmin(std::fmax(at, 0.0f), float(kPlanExt - 1));
            const size_t j = std::min(size_t(x), kPlanExt - 2);
            before[i] = _plan_ext[j] + (_plan_ext[j + 1] - _plan_ext[j]) * (x - float(j));
        }
        const bool had = s.n != 0 && now_us >= s.t0_us && now_us - s.t0_us < kSpanUs;
        _strip_stale = false;
        s.t0_us = now_us;
        s.n = uint16_t(kStripLen);
        s.cut = _osc_cut;
        _osc_cut = false;
        ++s.plan;
        _plan_ahead = ahead;
        _engine.peek(0, now_us, kMotionTickUs, ahead, plan);
        for (size_t i = 0; i < kOscEdge; ++i) _plan_ext[i] = had ? before[i] : plan[0];
    } else if (const size_t k = size_t((now_us - s.t0_us) / kMotionTickUs); k > 0) {
        // The same plan, advanced by the ticks elapsed: only the new tail is
        // read from the engine.
        std::copy(_plan_ext.begin() + k, _plan_ext.end(), _plan_ext.begin());
        s.t0_us += uint64_t(k) * kMotionTickUs;
        _engine.peek(0, s.t0_us + uint64_t(_plan_ahead - k) * kMotionTickUs, kMotionTickUs, k,
                     plan + (_plan_ahead - k));
    }
    if (s.n != 0) {
        // THE OSCILLATOR (RFC-103): summed into the plan before the backstop,
        // so the steer follows it like any plan. It yields first under the
        // input set, inside the window, and holds at nothing while the
        // machine may not move on its own (SPEC 9.7, 11.1).
        // ponytail: it re-renders the whole strip every tick (a few tens of
        // us on the P4); shifting its envelope with the strip is the upgrade
        // if the planner's tick ever shows it.
        std::array<float, kStripLen> osc{};
        if (!_osc.idle()) {
            const bool hold = _paused.load() || !_homed || !_commissioned.load();
            _osc.render(s.t0_us, kMotionTickUs, _plan_ext.data(), kStripLen, limitsFor(false),
                        toNorm(backstopLo()), toNorm(backstopHi()), hold, osc.data());
        }
        _osc_head = osc[0];
        for (size_t i = 0; i < kStripLen; ++i) s.p_mm[i] = toMm(plan[i] + osc[i]);
    }
    // ponytail: the whole strip is copied under the lock every tick (~0.5 KB);
    // a ring with a published head is the upgrade if the copy ever shows.
    if (_lock) _lock(true);
    _strip_pub = s;
    if (_lock) _lock(false);
}

bool MotionArbiter::stripAt(const PlanStrip& s, uint64_t t_us, float& p_mm) {
    if (s.n == 0) return false;
    if (t_us < s.t0_us) {
        p_mm = s.p_mm[0];
        return false;
    }
    const uint64_t at = t_us - s.t0_us;
    const uint64_t k = at / kMotionTickUs;
    const uint64_t last = uint64_t(s.n) - 1;
    if (k >= last) {
        p_mm = s.p_mm[last];
        return k == last && at % kMotionTickUs == 0;
    }
    // On a grid point: the entry itself, bit for bit.
    const float f = float(at % kMotionTickUs) / float(kMotionTickUs);
    p_mm = s.p_mm[k] + (s.p_mm[k + 1] - s.p_mm[k]) * f;
    return true;
}

bool MotionArbiter::stillFor(uint64_t now_us, uint32_t window_us) const {
    if (_lock) _lock(true);
    const PlanStrip& s = _strip_pub;
    bool still = true;
    if (s.n > 0) {
        const size_t last = size_t(s.n) - 1;
        const uint64_t end_us = now_us + window_us;
        const size_t i0 = now_us > s.t0_us ? size_t(std::min<uint64_t>((now_us - s.t0_us) / kMotionTickUs, last)) : 0;
        // The entry at or after the window's end, so the scan covers it.
        const size_t i1 = end_us > s.t0_us
                              ? size_t(std::min<uint64_t>((end_us - s.t0_us + kMotionTickUs - 1) / kMotionTickUs, last))
                              : 0;
        float lo = s.p_mm[i0], hi = lo;
        for (size_t i = i0 + 1; i <= i1; ++i) {
            lo = std::fmin(lo, s.p_mm[i]);
            hi = std::fmax(hi, s.p_mm[i]);
        }
        still = hi - lo < kMmPerStep;
    }
    if (_lock) _lock(false);
    return still;
}

// ---- the steer's tick -------------------------------------------------------

void MotionArbiter::steerTick(uint64_t now_us, float dt_s) {
    // A lapse is the emitter's own stop, kLeaseUs after the last renewal.
    // NEVER re-anchor _p_cmd_mm to the count here: the residual would become
    // feedforward over a dt_s measured before the stall, an uncommanded
    // reversal. The residual closes at the bounded kick below.
    const uint32_t lapses = _emitter.lapses();
    if (lapses != _lapses_seen) {
        _lease_lapses += lapses - _lapses_seen;
        _lapses_seen = lapses;
        GLOGW_EVERY_MS(1000, kTag, "LEASE LAPSE: the LP core stopped itself at %.3f mm", double(positionMm()));
    }

    // The newest strip, after reading where the one it replaces put the plan
    // now.
    float p_was = 0.0f;
    const bool was_in = stripAt(_steer_strip, now_us, p_was);
    if (_lock) _lock(true);
    _steer_strip = _strip_pub;
    if (_lock) _lock(false);
    const PlanStrip& s = _steer_strip;
    float p_plan_mm = 0.0f;
    const bool in = stripAt(s, now_us, p_plan_mm);
    if (s.gen != _seen_gen) {
        // An engine reset: the feedforward starts again from where the
        // carriage stood at it, once per reset. A reseed's plan runs on in mm
        // from the command in flight.
        _seen_gen = s.gen;
        _seen_plan = s.plan;
        if (!s.continuous) _p_cmd_mm = s.anchor_mm;
    } else if (s.plan != _seen_plan) {
        _seen_plan = s.plan;
        // A NEW PLAN IS A RESIDUAL, NEVER A BURST. A solve longer than the
        // reaction horizon lands a plan the carriage has already left: the
        // previous command moves by the gap between the two plans now, so
        // the feedforward keeps the old plan's velocity for this tick and the
        // bounded kick closes the gap. The move never widens the backstop's
        // frame.
        if (was_in && in) {
            const float d = p_plan_mm - p_was;
            if (std::fabs(d) > kMmPerStep && !s.cut) {
                ++_late_plans;
                _late_plan_max_mm = std::fmax(_late_plan_max_mm, std::fabs(d));
                if (std::fabs(d) > kLatePlanLogMm)
                    GLOGW_EVERY_MS(1000, kTag, "LATE PLAN: %.3f mm off the strip in flight, closed by the kick (largest %.3f mm)",
                                   double(d), double(_late_plan_max_mm));
            }
            const float lo = std::fmin(backstopLo(), _p_cmd_mm);
            const float hi = std::fmax(backstopHi(), _p_cmd_mm);
            _p_cmd_mm = std::fmin(std::fmax(_p_cmd_mm + d, lo), hi);
        }
    }

    // The gates. The planner applies the same ones on its own tick; these
    // hold the emitter until it has.
    if (_estop || !powerGateOpen()) {
        _emitter.park();   // the planner parks too: the same store twice
        _steer_gap = true;
        _emitter.renew();
        return;
    }
    // A home cycle's seek producer steers, fences and renews on the planner:
    // two producers never steer in one tick.
    if (_home != HomePhase::idle) {
        _steer_gap = true;
        return;
    }
    // A frame move, an engine reset the planner has not published yet, or no
    // plan: parked for this tick.
    if (_frame_moved || s.n == 0 || s.gen != _strip_gen.load()) {
        _emitter.steer(0.0f);
        _steer_gap = true;
        _emitter.renew();
        return;
    }
    // Past the strip's end the planner is late: the last entry holds, so the
    // carriage stops there.
    if (!in && now_us > s.t0_us) {
        if (!_strip_starved) {
            _strip_starved = true;
            ++_planner_stalls;
        }
        GLOGW_EVERY_MS(1000, kTag, "PLANNER STALL: the strip ended %.1f ms ago, holding at %.2f mm",
                       double(now_us - s.t0_us - uint64_t(s.n - 1) * kMotionTickUs) * 1e-3, double(p_plan_mm));
    } else {
        _strip_starved = false;
    }

    // THE POSITION BACKSTOP (operator ruling 2026-10-06, bd val-1w8). The
    // demand never leaves the backstop's frame (backstopLo()..backstopHi()),
    // whatever the plan does: a curve that bulges, a brake that is not aware
    // of the window, a frame gone stale. The frame widens only to the previous
    // demand, so a carriage left outside it is never pulled in by the clamp,
    // only by a plan. A home cycle never reaches here (the gate above): the
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
    // The first steer after a gated tick starts from the plan itself: what
    // the plan did while the emitter was held is residual, never feedforward.
    if (_steer_gap) {
        _steer_gap = false;
        _p_cmd_mm = p_plan_mm;
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
    // BOUNDED, because the residual is proportional to an error no ceiling
    // shaped and is therefore the one term that can hand the emitter a
    // demand the machine cannot make. Its ceiling is the accel limit's own
    // answer to "how much velocity may one tick add".
    const float kick_max = inputAmaxMm() * dt;
    const float err_mm = p_plan_mm - positionMm();
    if (std::fabs(err_mm) > kMmPerStep) {
        float kick = err_mm * kTrackHz;
        if (kick >  kick_max) kick =  kick_max;
        if (kick < -kick_max) kick = -kick_max;
        v += kick;
    }
    // The emitter floor is a FAULT DETECTOR, never a shaper (architecture.md
    // section 2), so the arbiter holds its own last word. The bound is the sum
    // of the two terms that make it: the plan, under the speed ceiling it was
    // planned under (the jog set's for a Manual plan, the input set's
    // otherwise), plus the correction, capped at one tick of accel above.
    // Deliberately NOT the bare ceiling -- at a demand that sits ON vmax, a
    // tracking correction has to be allowed above it or the residual can never
    // close (measured: 25 ms of added lag on the ceiling-limited run). Never
    // the input set's alone under a jog planned above it: a 1500 mm/s jog
    // under a 1000 mm/s input set rendered at 1050 mm/s and crawled the rest
    // at the kick (measured 2026-10-08: 227 mm in 1.05 s).
    const float v_cap = (_plan_manual.load() ? _jog_v : inputVmaxMm()) + kick_max;
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
    _emitter.renew();
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
// Planner task only, except homeSenseRose(). The seek producer: every leg is a
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
    }
    // After the reset: the steer stands aside until its strip is published.
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
        // A failure is a refused knot.
        if (a.kind == uint8_t(kinetic2::AnomalyKind::KnotRefused)) ++_k2_failures;
    }
    return kinds;
}

MotionArbiter::PlanRead MotionArbiter::readPlan(const kinetic2::State& st, uint64_t now_us) {
    PlanRead r;
    r.pos      = st.p;
    r.vel      = st.v;
    r.start    = st.p;
    r.target   = st.p;
    uint64_t seg_us = 0;
    const kinetic2::State seg = _engine.segStart(0, &seg_us);
    const bool seg_moved = seg_us != _seg_eng_us;
    _seg_eng_us = seg_us;
    SegRead s;
    if (now_us < _k2_brake_to_us) {
        // A brake renders: PAUSE, a generator's stop, a starved stream.
        r.mode       = uint8_t(PlanStyle::settle);
        r.start      = _k2_brake_from_p;
        r.target     = _k2_brake_to_p;
        r.duration_s = float(_k2_brake_to_us - _k2_brake_from_us) * 1e-6f;
        r.elapsed_s  = now_us > _k2_brake_from_us ? float(now_us - _k2_brake_from_us) * 1e-6f : 0.0f;
        s = SegRead{_k2_brake_from_p, _k2_brake_to_p, _k2_brake_from_us, _k2_brake_to_us, 0};
        _seg_through = false;
    } else if (_engine.pending() > 0) {
        // The segment in flight: from its authored start (Engine::segStart,
        // the knot retired before it or the accept on an axis at rest) to the
        // first pending knot at its solved time. A commit through a knot
        // (Engine::commitHorizon) moves segStart to that knot at its time,
        // still ahead: until then the piece in flight runs toward it from
        // where the last read's began, or from the knot that read was
        // heading for once that has passed. The plan sitting on the knot at
        // its time tells it from a re-plan off a brake, which leaves
        // segStart at the brake's end (val-0ep).
        const kinetic2::Solved& k = _engine.solved(0, 0);
        bool through = seg_us > now_us && _seg_through;
        if (seg_us > now_us && seg_moved) {
            float at_seg = 0.0f;
            _engine.peek(0, seg_us, kMotionTickUs, 1, &at_seg);
            through = std::fabs(at_seg - seg.p) < 1e-5f;
            if (through) {
                const bool passed = _seg_read.to_us <= now_us;
                s.from_p  = passed ? _seg_read.to_p : _seg_read.from_p;
                s.from_us = passed ? _seg_read.to_us : _seg_read.from_us;
                s.flags   = passed ? 0 : _seg_read.flags;
            }
        } else if (through) {
            s = _seg_read;
        }
        if (through) {
            s.to_p  = seg.p;
            s.to_us = seg_us;
        } else {
            s = SegRead{seg.p, k.p, seg_us, k.t_us, 0};
            // RFC-100 from the solver (registry plan_flags).
            if (k.share < 1.0f) s.flags |= plan_flags::shaped;
            if (k.stretched_s > 0.0f) s.flags |= plan_flags::stretched;
            if (k.clamped) s.flags |= plan_flags::clamped;
        }
        _seg_through = through;
        r.mode       = uint8_t(_k2_chase ? PlanStyle::chase : PlanStyle::waveform);
        r.plan_kind  = kPlanKindBezier;
        r.start      = s.from_p;
        r.target     = s.to_p;
        r.duration_s = s.to_us > s.from_us ? float(s.to_us - s.from_us) * 1e-6f : 0.0f;
        r.elapsed_s  = now_us > s.from_us ? std::fmin(float(now_us - s.from_us) * 1e-6f, r.duration_s) : 0.0f;
        r.flags      = s.flags;
        if (_k2_window_clamped) r.flags |= plan_flags::clamped;
    } else {
        s = SegRead{st.p, st.p, now_us, now_us, 0};
        _seg_through = false;
    }
    _seg_read = s;
    return r;
}

MotionCensus MotionArbiter::snapshot(uint64_t) {
    const PlanRead s = _plan_read;
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
    // The plan's position with the oscillation summed in: what the steer is
    // following, so the residual stays a tracking error.
    c.plan_mm        = seeking ? c.position_mm : toMm(s.pos + _osc_head);
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
    c.busy           = seeking || _plan_busy;
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
    // RFC-103: the ceilings or the window cut the oscillation below its ask.
    if (_osc.shaped()) c.plan_flags |= plan_flags::clamped;
    c.osc_active     = _osc.active();
    c.osc_amplitude  = _osc.amplitudeEffective();
    c.plans          = _k2_plans;
    c.failures       = _k2_failures;
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
    // latency (sampleLatencyUs()), published with its catalog default
    // (0x1122), so it never changes here alone.
    t.chase_dense_us = 60000;
    // The members the engine reads take its factory values (applyTuning()).
    const kinetic2::Config k2{};
    t.smoothness   = k2.smoothness;
    t.handle_floor = k2.handle_floor;
    t.trim_max     = k2.trim_max;
    t.react_us     = k2.react_us;
    return t;
}

}  // namespace valence
