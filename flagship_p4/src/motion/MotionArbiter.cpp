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

// Residual tracking (steerTick()). The emitter renders a VELOCITY, so a step
// it could not emit on time is a step nothing else repays: the residual
// against position truth is closed at this rate, and only past a whole step,
// so the quantization of the count itself never becomes commanded jitter.
constexpr float kTrackHz = 50.0f;
constexpr float kResidualDeadbandMm = kMmPerStep;

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
constexpr uint8_t kPlanKindBezier = 1;

class AbsentHomeSense final : public HomeSense {
public:
    bool present() const override { return false; }
    Probe probe() override { return Probe::undriven; }
    bool high() override { return false; }
};

AbsentHomeSense g_absent_sense;

// An osc-drive stream is live from a sample's arrival until this long past the
// newest stamp, whichever session published it. It is the stage's own quiet
// window too: a stream is quiet once the stage asks nothing of it.
constexpr uint64_t kOscQuietUs = uint64_t(limits::stream_quiet_release_ms) * 1000u;
static_assert(kOscQuietUs == MotionOscillator::kDriveQuietUs,
              "the osc-drive stream's quiet window is the oscillator's");

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

// The state a grid sequence is in at its newest sample p0 (p1, p2 the two
// before): the second difference is the acceleration the steer renders
// across the last two ticks, and the velocity is the last tick's mean
// carried to its end by half of it. Differences first, so the subtraction
// of two nearby floats is exact.
kinetic2::State gridState(float p2, float p1, float p0) {
    constexpr float dt = float(kMotionTickUs) * 1e-6f;
    const float d1 = p0 - p1, d2 = p1 - p2;
    kinetic2::State s;
    s.p = p0;
    s.a = (d1 - d2) / (dt * dt);
    s.v = d1 / dt + 0.5f * s.a * dt;
    return s;
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
    _osc_req_hz.store(o.frequency_hz);
    _osc_req_amp.store(o.amplitude);
    _osc_req_crest.store(o.dwell_crest);
    _osc_req_trough.store(o.dwell_trough);
    _osc_req_fdrive.store(o.frequency_drive.drive);
    _osc_req_adrive.store(o.amplitude_drive.drive);
    const MotionOscDrive* d[2] = {&o.frequency_drive, &o.amplitude_drive};
    std::array<std::atomic<float>, 4>* m[2] = {&_osc_req_fmap, &_osc_req_amap};
    for (int k = 0; k < 2; ++k) {
        (*m[k])[0].store(d[k]->in_min);
        (*m[k])[1].store(d[k]->in_max);
        (*m[k])[2].store(d[k]->out_min);
        (*m[k])[3].store(d[k]->out_max);
    }
    _osc_seq.fetch_add(1);   // even: whole
}

void MotionArbiter::postOscDrive(float amplitude, float frequency, uint64_t at_us) {
    const uint32_t head = _oscd_head.load(std::memory_order_relaxed);
    if (head - _oscd_tail.load(std::memory_order_acquire) >= _oscd_ring.size()) {
        _oscd_ring_lost.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    _oscd_ring[head % _oscd_ring.size()] = {at_us, amplitude, frequency};
    _oscd_head.store(head + 1, std::memory_order_release);
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
    // A refused plan's stop (handover()) renders to its end: nothing joins it.
    if (now_us < _held_until_us) {
        ++_rejected;
        GLOGW_EVERY_MS(1000, kTag, "REJECT: a late plan's stop is braking to rest");
        return false;
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
        if (_engine.isBusy(now_us) || joining(now_us)) {
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
    _plan_manual = in.source == MotionSource::Manual && _jog_after_us == 0;
    return true;
}

void MotionArbiter::notePlanCost(uint32_t us) {
    _plan_us_last = us;
    if (us > _plan_us_max) _plan_us_max = us;
    _plan_us_avg += (float(us) - _plan_us_avg) * 0.125f;
}

void MotionArbiter::resetEngine(float p_norm, uint64_t now_us, bool rebase) {
    _engine.resetAt(p_norm, now_us);   // clears the expectation
    _lead_us_avg    = 0.0f;
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
    // The oscillation stops with the plan it rode.
    _osc.reset();
    _osc_head = 0.0f;
    _strip_stale = true;
    if (rebase) {
        // The steer re-anchors its feedforward here and steers nothing from a
        // strip published before this reset (steerTick()); the connector and
        // a refused plan's hold go with the trajectory they joined.
        _anchor_mm = positionMm();
        _conn_h_us = _conn_end_us = 0;
        _held_until_us = 0;
        _strip_gen.fetch_add(1);
    }
    _eng_lo   = frameLo();
    _eng_span = span();
}

void MotionArbiter::reseedEngine(uint64_t now_us) {
    // The plan's own curve, not the count: the engine restates it and keeps
    // it through the reaction horizon or the next knot (Engine::reframe,
    // RFC-105 (bb)), so the strip the steer follows runs on in mm and the
    // refill's handover finds nothing to join.
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
    // The plan the strip was cut from and the oscillator are millimeters,
    // which a frame move leaves alone (Valence RFC-112): never restated.
    if (_engine.pending() == 0) {
        const kinetic2::Knot h = _engine.newest();
        _k2_newest_us = h.t_us;
        _k2_newest_p  = h.p;
    }
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
    // The reaction horizon for this submit (adaptiveReactUs): the lead of this
    // knot joins the smoothed lead first, so a stream's first knots converge
    // within four. A jog's lead is nothing (due at once), a sample's its
    // latency, a segment's its end past now. Set before the supersede and jog
    // hand-offs below (Engine::truncateAfter commits at the horizon too).
    {
        const uint64_t due = in.duration_us > 0 ? in.anchor_us + in.duration_us
                           : manual              ? now_us
                                                 : now_us + _k2_latency_us;
        const float lead = due > now_us ? float(due - now_us) : 0.0f;
        // Per source: a jog after a stream takes its own lead at once, a new
        // stream starts from its first knot's and converges within four.
        if (in.source != _lead_source) { _lead_source = in.source; _lead_us_avg = lead; }
        else _lead_us_avg += (lead - _lead_us_avg) * 0.25f;
        _react_us_now = adaptiveReactUs(_plan_us_avg, _lead_us_avg, _react_floor_us);
        cfg.react_us = _react_us_now;
        if (_react_us_now + 1000 < _react_us_logged || _react_us_now > _react_us_logged + 1000) {
            _react_us_logged = _react_us_now;
            GLOGI_EVERY_MS(1000, kTag, "horizon: react %lu us (plan avg %lu us, lead avg %lu us, floor %lu us)",
                           static_cast<unsigned long>(_react_us_now), static_cast<unsigned long>(_plan_us_avg),
                           static_cast<unsigned long>(_lead_us_avg), static_cast<unsigned long>(_react_floor_us));
        }
    }
    _engine.setConfig(cfg);

    // At rest the state is the emitter's count and nothing else: a reseed from
    // the engine's own idea of where it stopped carries every move's sub-step
    // residue into the next one. pending() first: isBusy() solves, and a
    // bundle must not solve once per sample.
    // Never under a connector: the trajectory still moves toward the
    // engine's rest, and the new knot plans from that rest.
    if (_engine.pending() == 0 && !joining(now_us) && !_engine.isBusy(now_us))
        resetEngine(toNorm(positionMm()), now_us);

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
        if (!_plan_manual && _engine.isBusy(now_us)) {
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
    // Never set p.shape: Kinetic's default sine is the only waveform (Valence RFC-112).
    // Written so a NaN reads 0: the delegate clamped, this is the backstop.
    const float hz = _osc_req_hz.load(), amp = _osc_req_amp.load();
    p.frequency = !(hz > 0.0f) ? 0.0f : std::fmin(hz, OSC_MAX_HZ);
    p.amplitude = !(amp > 0.0f) ? 0.0f : std::fmin(amp, OSC_MAX_AMPLITUDE_MM);
    p.dwell_crest = _osc_req_crest.load();
    p.dwell_trough = _osc_req_trough.load();
    auto drive = [](uint8_t d, const std::array<std::atomic<float>, 4>& m) {
        MotionOscDrive out;
        out.drive = d <= 3 ? d : 0;
        out.in_min = m[0].load();
        out.in_max = m[1].load();
        out.out_min = m[2].load();
        out.out_max = m[3].load();
        return out;
    };
    const MotionOscDrive fd = drive(_osc_req_fdrive.load(), _osc_req_fmap);
    const MotionOscDrive ad = drive(_osc_req_adrive.load(), _osc_req_amap);
    if (_osc_seq.load() != seq) return;   // rewritten while read
    _osc_taken = seq;
    _osc_base = p;
    _osc_fdrive = fd;
    _osc_adrive = ad;
    _oscd_plan_next_us = 0;   // a change asks its plan point at the next tick
    setOscStage();
}

void MotionArbiter::setOscStage() {
    kinetic2::OscParams p = _osc_base;
    p.enabled = p.enabled || _osc_streaming;
    // Driven by a live input (a speed or position drive, or an axis drive
    // with a live stream) is the driven mode: a sine, frequency and amplitude
    // from the points alone (dwells and the two fields unused). An
    // axis drive with no live stream is exactly fixed. A change of mode
    // passes through rest (the kernel's fade out, then in).
    p.driven = oscPlanBound() || _osc_streaming;
    _osc.set(p);
}

bool MotionArbiter::oscAxisBound() const {
    return _osc_fdrive.drive == osc_drives::axis || _osc_adrive.drive == osc_drives::axis;
}

bool MotionArbiter::oscPlanBound() const {
    auto plan = [](uint8_t d) { return d == osc_drives::speed || d == osc_drives::position; };
    return plan(_osc_fdrive.drive) || plan(_osc_adrive.drive);
}

bool MotionArbiter::oscStreamLive(uint64_t t_us) const {
    return _oscd_have && (t_us < _oscd_last.t_us || t_us - _oscd_last.t_us < kOscQuietUs);
}

void MotionArbiter::streamOscillator(uint64_t now_us) {
    // A sample still in the ring is live by its arrival: the stage is enabled
    // before this tick's plan read, which reaches its look-ahead only then.
    const bool waiting = _oscd_head.load(std::memory_order_acquire) != _oscd_tail.load(std::memory_order_relaxed);
    const bool streaming = oscAxisBound() && (waiting || oscStreamLive(now_us));
    if (streaming == _osc_streaming) return;
    _osc_streaming = streaming;
    setOscStage();
}

void MotionArbiter::feedOscillator(uint64_t t0_us, const float* plan, bool render) {
    // Drained every tick, rendering or not, so a sample never waits in the
    // ring for an oscillator that is off.
    const bool axis = oscAxisBound();
    for (uint32_t tail = _oscd_tail.load(std::memory_order_relaxed);
         tail != _oscd_head.load(std::memory_order_acquire); ++tail) {
        _oscd_last = _oscd_ring[tail % _oscd_ring.size()];
        _oscd_have = true;
        _oscd_tail.store(tail + 1, std::memory_order_release);
        if (render && axis) drivePoint(_oscd_last.t_us, plan, t0_us);
    }
    if (!render) return;
    if (oscPlanBound() && t0_us >= _oscd_plan_next_us) {
        _oscd_plan_next_us = t0_us + uint64_t(kOscPlanDriveTicks) * kMotionTickUs;
        drivePoint(t0_us + kOscDriveLeadUs, plan, t0_us);
    }
}

void MotionArbiter::drivePoint(uint64_t t_us, const float* plan, uint64_t t0_us) {
    // The plan at t_us, clamped into what fillStrip() read: the position in
    // mm, client frame, and the speed in mm/s by the grid's central difference.
    constexpr int64_t kLast = int64_t(kStripLen + kOscLook) - 1;
    const int64_t i = std::clamp<int64_t>((int64_t(t_us) - int64_t(t0_us)) / int64_t(kMotionTickUs), 0, kLast);
    const float pos = _flipped.load() ? _rail - plan[i] : plan[i];
    const float speed = std::fabs(plan[i + 1] - plan[i - 1]) / (2e-6f * float(kMotionTickUs));
    // bd val-o9r (a), ahead of the pinned SPEC 9.7: with no live stream an
    // axis-driven parameter is its field, as fixed is, while osc.enabled
    // holds; without it there is no hand to return to, and it reads 0.
    const bool live = oscStreamLive(t_us);
    auto driven = [&](const MotionOscDrive& d, float own, float axis, float top) {
        if (d.drive == osc_drives::fixed) return std::fmin(own, top);
        if (d.drive == osc_drives::axis && !live) return _osc_base.enabled ? std::fmin(own, top) : 0.0f;
        const float in = d.drive == osc_drives::speed ? speed : d.drive == osc_drives::position ? pos : axis;
        if (!std::isfinite(in) || !(d.in_max != d.in_min)) return 0.0f;
        // SPEC 8.11 linear_clamp.
        const float c = std::fmin(std::fmax(in, std::fmin(d.in_min, d.in_max)), std::fmax(d.in_min, d.in_max));
        const float out = d.out_min + (c - d.in_min) * (d.out_max - d.out_min) / (d.in_max - d.in_min);
        return !(out > 0.0f) ? 0.0f : std::fmin(out, top);
    };
    const float hz = driven(_osc_fdrive, _osc_base.frequency, _oscd_last.frequency, OSC_MAX_HZ);
    const float amp = driven(_osc_adrive, _osc_base.amplitude, _oscd_last.amplitude, OSC_MAX_AMPLITUDE_MM);
    if (!_osc.drive(t_us, hz, amp)) {
        ++_oscd_dropped;
        GLOGW_EVERY_MS(1000, kTag, "OSC DRIVE: point dropped, the oscillator holds its most (%lu dropped)",
                       static_cast<unsigned long>(_oscd_dropped));
    }
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
    // The oscillation stops with the motion (SPEC 9.7): the next refill's
    // handover fades it through a connector, and an idle carriage comes to
    // rest on the plan.
    const bool cut = _osc.active();
    _osc.reset();
    _handover_quiet = true;
    if (!brakeEngine(now_us)) {
        _pause_pos_mm.store(cut || joining(now_us) ? toMm(st.p) : positionMm());
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
    _react_floor_us = t.react_us;
    c.react_us     = t.react_us;   // the floor; each submit raises it to cover the solve (adaptiveReactUs)
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
    // A reset's generation moves BEFORE the flags drop, so the steer never
    // takes a strip of the old origin in between; a reseed's strip runs on in
    // mm and keeps its generation, so the steer never holds a tick for it.
    if (_frame_moved) {
        const bool origin_moved = _origin_moved;
        // A cycle counted in the old frame: its seek or backoff is void.
        const bool homing = _homing.load();
        const bool reset = origin_moved || homing ||
                           (_engine.pending() == 0 && !_engine.isBusy(now_us) && !_osc.active() &&
                            !joining(now_us));
        if (reset) _strip_gen.fetch_add(1);
        _frame_moved = false;
        _origin_moved = false;
        if (homing) homeEnd("the travel window changed", now_us);
        if (reset) {
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
    // because the engine is this task's. The rail clamp still applies. A
    // refused plan's stop runs first: the request waits for its rest.
    if (now_us >= _held_until_us && _return_req.exchange(false) && _override.load()) {
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
    // Busy while the engine plans motion or the connector still moves the
    // trajectory toward its rest.
    const kinetic2::State st = sampleEngine(now_us);
    const bool busy = _engine.isBusy(now_us) || joining(now_us);
    _plan_read = readPlan(st, now_us);
    _plan_busy = busy;
    // The jog set's ceiling holds until the plan planned under it has ended.
    if (!busy) {
        _plan_manual = false;
        _jog_after_us = 0;
    } else if (_jog_after_us != 0 && now_us >= _jog_after_us) {
        _plan_manual = true;
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
    constexpr uint64_t kSpanUs = uint64_t(kStripLen) * kMotionTickUs;
    // The plan from t0 on, mm, in _plan_ext after the kOscEdge ticks before
    // t0. Read this far ahead only while the oscillator renders: its
    // look-ahead past the strip, and its edge.
    float* plan = _plan_ext.data() + kOscEdge;
    streamOscillator(now_us);
    const size_t ahead = _osc.idle() ? kStripLen : kPlanExt - kOscEdge;
    // A lapse the steer asked to be re-planned from: a whole refill joined
    // from the carriage at rest.
    const uint32_t replan = _replan_req.load();
    const bool from_rest = live && replan != _replan_seen;
    bool refilled = false;
    if (!live) {
        s.n = 0;
        _osc.reset();
        _osc_head = 0.0f;
    } else if (from_rest || _strip_stale || s.n == 0 || (now_us >= s.t0_us && now_us - s.t0_us >= kSpanUs) ||
               ahead > _plan_ahead) {
        // A whole refill, from the clock after this tick's solve: the steer
        // rendered the strip in flight until then (handover()). One walk of
        // the window, one piece per knot interval. The ticks before t0 are
        // what the last strip planned there, for the oscillator's
        // differences; after a reset, the plan's own start.
        const uint64_t t0 = std::max(now_us, _now_us());
        std::array<float, kOscEdge> before{};
        for (size_t i = 0; i < kOscEdge; ++i) {
            const float at = float(int64_t(t0) - int64_t(s.t0_us)) / float(kMotionTickUs) + float(i);
            const float x = std::fmin(std::fmax(at, 0.0f), float(kPlanExt - 1));
            const size_t j = std::min(size_t(x), kPlanExt - 2);
            before[i] = _plan_ext[j] + (_plan_ext[j + 1] - _plan_ext[j]) * (x - float(j));
        }
        const bool had = s.n != 0 && t0 >= s.t0_us && t0 - s.t0_us < kSpanUs;
        _strip_stale = false;
        s.t0_us = t0;
        s.n = uint16_t(kStripLen);
        _plan_ahead = ahead;
        _engine.peek(0, t0, kMotionTickUs, ahead, plan);
        for (size_t i = 0; i < ahead; ++i) plan[i] = toMm(plan[i]);
        for (size_t i = 0; i < kOscEdge; ++i) _plan_ext[i] = had ? before[i] : plan[0];
        refilled = true;
    } else if (now_us >= s.t0_us) {
        // The same plan, advanced by the ticks elapsed: only the new tail is
        // read from the engine.
        if (const size_t k = size_t((now_us - s.t0_us) / kMotionTickUs); k > 0) {
            std::copy(_plan_ext.begin() + k, _plan_ext.end(), _plan_ext.begin());
            s.t0_us += uint64_t(k) * kMotionTickUs;
            float* tail = plan + (_plan_ahead - k);
            _engine.peek(0, s.t0_us + uint64_t(_plan_ahead - k) * kMotionTickUs, kMotionTickUs, k, tail);
            for (size_t i = 0; i < k; ++i) tail[i] = toMm(tail[i]);
        }
    }
    feedOscillator(s.t0_us, plan, s.n != 0 && !_osc.idle());
    if (s.n != 0) {
        // THE OSCILLATOR (RFC-103): summed into the plan before the backstop,
        // so the steer follows it like any plan. It yields first under the
        // input set, inside the window, and holds at nothing while the
        // machine may not move on its own (SPEC 9.7, 11.1). It runs in mm,
        // rail frame: the plan, the ceilings, the window and its amplitude
        // alike, so a window move changes none of them (Valence RFC-112).
        // ponytail: it re-renders the whole strip every tick (a few tens of
        // us on the P4); shifting its envelope with the strip is the upgrade
        // if the planner's tick ever shows it.
        std::array<float, kStripLen> osc{};
        if (!_osc.idle()) {
            const bool hold = _paused.load() || !_homed || !_commissioned.load();
            EngineLimits lim = limitsFor(false);
            lim.vmax *= span();
            lim.amax *= span();
            lim.jmax *= span();
            _osc.render(s.t0_us, kMotionTickUs, _plan_ext.data(), kStripLen, lim, backstopLo(), backstopHi(),
                        hold, osc.data());
        }
        _osc_head = osc[0];
        for (size_t i = 0; i < kStripLen; ++i) s.p_mm[i] = plan[i] + osc[i];
        if (refilled) {
            handover(s, from_rest, now_us);
            if (joining(now_us)) _plan_busy = true;   // the census reads the join at once
        }
        applyConnector(s);
        s.lo_mm = backstopLo();
        s.hi_mm = backstopHi();
        s.lim = renderLimits();
        // THE TAIL (bd val-cy1g): the stop the steer plays past the last
        // entry while the planner is late, from the state the strip ends in
        // as the steer renders it (the plan, the oscillator and the
        // connector), certified here so the steer never computes one.
        const size_t n = s.n;
        s.tail = certifiedStop(gridState(s.p_mm[n - 3], s.p_mm[n - 2], s.p_mm[n - 1]),
                               s.t0_us + uint64_t(n - 1) * kMotionTickUs, s.lim, s.lo_mm, s.hi_mm, s.tail_hard);
    }
    if (from_rest) _replan_seen = replan;
    s.replan = _replan_seen;
    // ponytail: the whole strip is copied under the lock every tick (~0.7 KB);
    // a ring with a published head is the upgrade if the copy ever shows.
    if (_lock) _lock(true);
    _strip_pub = s;
    if (_lock) _lock(false);
}

void MotionArbiter::handover(PlanStrip& s, bool from_rest, uint64_t now_us) {
    // The trajectory in flight: the strip last published, which this task
    // alone writes, so it is read here without the lock; after a lapse, the
    // carriage at rest where the LP core stopped it.
    const PlanStrip& old = _strip_pub;
    const bool quiet = _handover_quiet;
    _handover_quiet = false;
    // After a rebasing reset the steer re-anchors at the carriage: nothing
    // to join.
    if (!from_rest && (old.n == 0 || old.gen != s.gen)) {
        _conn = kinetic2::Profile{};
        _conn_h_us = _conn_end_us = 0;
        return;
    }

    constexpr int h = int(kHandoverTicks);
    constexpr float dt = float(kMotionTickUs) * 1e-6f;
    const int n = int(s.n);
    const float rest = positionMm();
    auto tAt = [&](int k) { return uint64_t(int64_t(s.t0_us) + int64_t(k) * int64_t(kMotionTickUs)); };
    // The trajectory in flight at k in [-3, h], index k + 3.
    std::array<float, h + 4> o{};
    for (int k = -3; k <= h; ++k) o[size_t(k + 3)] = from_rest ? rest : stripAt(old, tAt(k));
    // The candidate, index k + 3 for k in [-3, n): the trajectory in flight
    // before the handover, the new one from it.
    std::array<float, kStripLen + 3> c{};
    float* const at = c.data() + 3;
    for (int k = -3; k < 0; ++k) at[k] = o[size_t(k + 3)];
    const EngineLimits lim = renderLimits();
    const float lo = std::fmin(backstopLo(), o[2]);
    const float hi = std::fmax(backstopHi(), o[2]);

    // The grid's own differences: the steer renders x[k] - x[k-1] over each
    // tick, so these are its velocity, acceleration and jerk. x[k] is read
    // for k in [from - 3, to).
    struct Peaks {
        float v = 0.0f, a = 0.0f, j = 0.0f, lo = 1e30f, hi = -1e30f;
    };
    auto peaksOf = [&](const float* x, int from, int to) {
        Peaks pk;
        for (int k = from; k < to; ++k) {
            const float d1 = x[k] - x[k - 1], d2 = x[k - 1] - x[k - 2], d3 = x[k - 2] - x[k - 3];
            pk.v = std::fmax(pk.v, std::fabs(d1) / dt);
            pk.a = std::fmax(pk.a, std::fabs(d1 - d2) / (dt * dt));
            pk.j = std::fmax(pk.j, std::fabs((d1 - d2) - (d2 - d3)) / (dt * dt * dt));
            pk.lo = std::fmin(pk.lo, x[k]);
            pk.hi = std::fmax(pk.hi, x[k]);
        }
        return pk;
    };
    // The set's ceilings, or the plan's own peak where a piece already runs
    // over one (PieceOverCeiling is the solver's to report), with the float
    // rounding of a strip position in those differences.
    const Peaks own = peaksOf(s.p_mm.data(), 3, n);
    const Peaks was = peaksOf(o.data() + 3, 0, h + 1);
    const float ulp = 2.4e-7f * std::fmax(std::fmax(std::fabs(lo), std::fabs(hi)), 1.0f);
    const float cap_v = std::fmax(lim.vmax, std::fmax(own.v, was.v)) * 1.01f + 2.0f * ulp / dt;
    const float cap_a = std::fmax(lim.amax, std::fmax(own.a, was.a)) * 1.02f + 4.0f * ulp / (dt * dt);
    const float cap_j = std::fmax(lim.jmax, std::fmax(own.j, was.j)) * 1.02f + 8.0f * ulp / (dt * dt * dt);
    auto legal = [&](int from, int to) {
        const Peaks pk = peaksOf(at, from, std::min(to, n));
        return pk.v <= cap_v && pk.a <= cap_a && pk.j <= cap_j && pk.lo >= lo - kMmPerStep && pk.hi <= hi + kMmPerStep;
    };

    // 1. The new plan with the connector in flight. The engine plans every
    // change from its own state at the sample, continuous with what it
    // planned before, so a plan on time joins here unchanged.
    for (int k = 0; k < n; ++k) at[k] = s.p_mm[size_t(k)] + connAt(tAt(k), s.p_mm[size_t(k)]);
    if (legal(0, 3)) return;

    // The rest joins at the handover, kHandoverTicks on: the trajectory in
    // flight through the tick before it, its gap to the new plan there as a
    // state on the grid, which a connector closes.
    const uint64_t t_h = tAt(h);
    for (int k = 0; k < h; ++k) at[k] = o[size_t(k + 3)];
    const kinetic2::State gap = gridState(o[size_t(h + 1)] - s.p_mm[size_t(h - 2)],
                                          o[size_t(h + 2)] - s.p_mm[size_t(h - 1)],
                                          o[size_t(h + 3)] - s.p_mm[size_t(h)]);
    auto keep = [&](const kinetic2::Profile& e, uint64_t end_us) {
        _conn = e;
        _conn_h_us = t_h;
        _conn_end_us = end_us;
        _conn_pre_t0 = s.t0_us;
        for (int k = 0; k < h; ++k) _conn_pre[size_t(k)] = o[size_t(k + 3)];
    };

    // 2. A connector: the gap brought to rest at 0 (Profile::point), summed
    // with the plan, under the set and then shares of it, the first whose sum
    // holds every ceiling and the frame. Past the strip the plan is unseen,
    // so a connector longer than the strip joins only a plan at rest there.
    const uint64_t t_last = tAt(n - 1);
    for (const float share : {1.0f, 0.5f, 0.25f}) {
        EngineLimits le = lim;
        le.vmax *= share;
        le.amax *= share;
        le.jmax *= share;
        const kinetic2::Profile e = kinetic2::Profile::point(gap, 0.0f, t_h, le);
        if (e.n <= 0) continue;
        const uint64_t end = e.end_us();
        if (end > t_last && _engine.isBusy(t_last)) continue;
        for (int k = h; k < n; ++k) {
            const uint64_t t = tAt(k);
            at[k] = s.p_mm[size_t(k)] + (t < end ? e.at(t).p : 0.0f);
        }
        if (!legal(h, n)) continue;
        keep(e, std::max(t_h, end));
        if (!quiet) {
            ++_late_plans;
            _late_plan_max_mm = std::fmax(_late_plan_max_mm, std::fabs(gap.p));
            GLOGW_EVERY_MS(1000, kTag,
                           "LATE PLAN: %.3f mm, %.1f mm/s off the trajectory in flight, joined over %.1f ms "
                           "(largest %.3f mm)",
                           double(gap.p), double(gap.v), double(end - t_h) * 1e-3, double(_late_plan_max_mm));
        }
        return;
    }

    // 3. Refused: no connector joins it inside the set. The trajectory in
    // flight stops instead, certified from its state at the handover; the
    // engine holds that stop's rest and every intent is refused until the
    // carriage is there. Not incorporated, and counted.
    bool hard = false;
    kinetic2::Profile stop =
        certifiedStop(gridState(o[size_t(h + 1)], o[size_t(h + 2)], o[size_t(h + 3)]), t_h, lim, backstopLo(),
                      backstopHi(), hard);
    if (hard) ++_hard_stops;
    const float p_rest = stop.end().p;
    resetEngine(toNorm(p_rest), now_us, false);
    _strip_stale = false;   // this refill publishes the reset
    _held_until_us = stop.n > 0 ? stop.end_us() : 0;
    if (_paused.load()) _pause_pos_mm.store(p_rest);
    stop.s0.p -= p_rest;   // the stop as a gap over the rest
    keep(stop, std::max(t_h, stop.end_us()));
    _plan_ext.fill(p_rest);
    s.p_mm.fill(p_rest);
    _osc_head = 0.0f;
    ++_plans_refused;
    GLOGW(kTag, "LATE PLAN REFUSED: %.3f mm, %.1f mm/s off the trajectory in flight, no connector holds the set; "
                "stopping at %.2f mm",
          double(gap.p), double(gap.v), double(p_rest));
}

float MotionArbiter::connAt(uint64_t t, float plan_mm) const {
    if (t >= _conn_end_us) return 0.0f;
    if (t < _conn_h_us) {
        const uint64_t j = t > _conn_pre_t0 ? (t - _conn_pre_t0) / kMotionTickUs : 0;
        return _conn_pre[size_t(std::min<uint64_t>(j, kHandoverTicks - 1))] - plan_mm;
    }
    return _conn.at(t).p;
}

void MotionArbiter::applyConnector(PlanStrip& s) const {
    for (size_t i = 0; i < s.n; ++i) {
        const uint64_t t = s.t0_us + uint64_t(i) * kMotionTickUs;
        if (t >= _conn_end_us) return;
        s.p_mm[i] += connAt(t, s.p_mm[i]);
    }
}

kinetic2::Profile MotionArbiter::certifiedStop(const kinetic2::State& s, uint64_t at_us, const EngineLimits& lim,
                                               float lo, float hi, bool& hard) {
    hard = false;
    const float flo = std::fmin(lo, s.p) - kMmPerStep;
    const float fhi = std::fmax(hi, s.p) + kMmPerStep;
    // Profile::brake stops along the velocity, or the acceleration from rest;
    // a stop that turns back (float residue at a vanishing speed) is braked
    // from the velocity alone, which cannot.
    const float sgn = s.v != 0.0f ? (s.v > 0.0f ? 1.0f : -1.0f) : (s.a > 0.0f ? 1.0f : -1.0f);
    auto stop = [&](float k) {
        EngineLimits l = lim;
        l.amax *= k;
        l.jmax *= k;
        kinetic2::Profile pr = kinetic2::Profile::brake(s, at_us, l);
        if (sgn * (pr.end().p - s.p) < 0.0f) pr = kinetic2::Profile::brake(kinetic2::State{s.p, s.v, 0.0f}, at_us, l);
        return pr;
    };
    auto inside = [&](const kinetic2::Profile& pr) {
        const float e = pr.end().p;
        return e >= flo && e <= fhi;
    };
    kinetic2::Profile pr = stop(1.0f);
    if (inside(pr)) return pr;
    // Harder, to the edge: the least multiple of the set's accel and jerk
    // that stops inside, by bisection; past kHardStopMax the backstop holds.
    hard = true;
    float k_in = kHardStopMax, k_out = 1.0f;
    if (!inside(stop(k_in))) return stop(k_in);
    for (int i = 0; i < 12; ++i) {
        const float mid = 0.5f * (k_in + k_out);
        if (inside(stop(mid))) k_in = mid;
        else k_out = mid;
    }
    return stop(k_in);
}

EngineLimits MotionArbiter::renderLimits() const {
    EngineLimits l = limitsFor(_plan_manual);
    const float sp = span();
    l.vmax *= sp;
    l.amax *= sp;
    l.jmax *= sp;
    return l;
}

float MotionArbiter::stripAt(const PlanStrip& s, uint64_t t_us) {
    if (s.n == 0) return 0.0f;
    if (t_us <= s.t0_us) return s.p_mm[0];
    const uint64_t at = t_us - s.t0_us;
    const uint64_t k = at / kMotionTickUs;
    const uint64_t last = uint64_t(s.n) - 1;
    if (k >= last) return at == last * kMotionTickUs ? s.p_mm[last] : s.tail.at(t_us).p;
    // On a grid point: the entry itself, bit for bit.
    const float f = float(at % kMotionTickUs) / float(kMotionTickUs);
    return s.p_mm[k] + (s.p_mm[k + 1] - s.p_mm[k]) * f;
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
        // Past the last entry the tail renders: it ends at its rest.
        if (end_us > s.t0_us + uint64_t(last) * kMotionTickUs) {
            const float e = s.tail.end().p;
            lo = std::fmin(lo, e);
            hi = std::fmax(hi, e);
        }
        still = hi - lo < kMmPerStep;
    }
    if (_lock) _lock(false);
    return still;
}

// ---- the steer's tick -------------------------------------------------------

void MotionArbiter::steerTick(uint64_t now_us, float dt_s) {
    // A LAPSE IS THE LP CORE'S OWN STOP, kLeaseUs after the last renewal: the
    // carriage stands wherever it stopped. The plan's velocity is never the
    // next steer from there, and _p_cmd_mm is never re-anchored to the count
    // over a dt_s measured before the stall (an uncommanded reversal): the
    // steer parks and asks the planner for a strip joined from the carriage
    // at rest (fillStrip(), handover()), and steers that strip.
    const uint32_t lapses = _emitter.lapses();
    if (lapses != _lapses_seen) {
        _lease_lapses += lapses - _lapses_seen;
        _lapses_seen = lapses;
        _replan_req.store(++_replan_want);
        GLOGW_EVERY_MS(1000, kTag, "LEASE LAPSE: the LP core stopped itself at %.3f mm, re-planning from there",
                       double(positionMm()));
    }
    // A LATE TICK IS A STALL, NEVER A BURST (bd val-1w8): counted here,
    // whichever gate the tick takes; the residual and the cap below are
    // priced over at most kTickDtCapS, whatever the clock did.
    const bool stall = dt_s > kTickDtCapS;
    if (stall) {
        ++_stalls;
        GLOGW_EVERY_MS(1000, kTag, "STALL: tick %.1f ms after the last, residual and cap held to kTickDtCapS",
                       double(dt_s * 1e3f));
    }

    if (_lock) _lock(true);
    _steer_strip = _strip_pub;
    if (_lock) _lock(false);
    const PlanStrip& s = _steer_strip;
    if (s.gen != _seen_gen) {
        // An engine reset: the feedforward starts again from where the
        // carriage stood at it, once per reset.
        _seen_gen = s.gen;
        _p_cmd_mm = s.anchor_mm;
        _res_v = _res_a = 0.0f;
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
    // A re-origin the planner has not reset for, an engine reset it has not
    // published, no plan, or a lapse it has not answered: parked for this
    // tick. A window move is none of these: every strip carries the frame
    // it was filled in.
    if (_origin_moved || s.n == 0 || s.gen != _strip_gen.load() || s.replan != _replan_want) {
        _emitter.steer(0.0f);
        _steer_gap = true;
        _emitter.renew();
        return;
    }
    // THE TRAJECTORY. Consecutive strips of a gen are one (handover()): a
    // new plan reaches the steer already joined to the one it was rendering.
    // Past the last entry the planner is late and the strip's tail, a stop
    // certified at its fill, renders to rest (bd val-cy1g).
    float p_plan_mm = stripAt(s, now_us);
    if (now_us > s.t0_us + uint64_t(s.n - 1) * kMotionTickUs) {
        if (!_strip_starved) {
            _strip_starved = true;
            ++_planner_stalls;
            if (s.tail_hard) ++_hard_tails;
        }
        GLOGW_EVERY_MS(1000, kTag, "PLANNER STALL: the strip ended %.1f ms ago, stopping on its tail at %.2f mm",
                       double(now_us - s.t0_us - uint64_t(s.n - 1) * kMotionTickUs) * 1e-3, double(p_plan_mm));
    } else {
        _strip_starved = false;
    }

    // THE POSITION BACKSTOP (operator ruling 2026-10-06, bd val-1w8). The
    // demand never leaves the backstop's frame (the strip's lo_mm..hi_mm),
    // whatever the plan does: a curve that bulges, a brake that is not aware
    // of the window, a frame gone stale. The frame widens only to the previous
    // demand, so a carriage left outside it is never pulled in by the clamp,
    // only by a plan. A home cycle never reaches here (the gate above): the
    // seek has to reach the stops. The LP core's fence is the second line, and
    // the only one an HP stall cannot skip (syncFence()).
    float lo = s.lo_mm;
    float hi = s.hi_mm;
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
    // The first steer after a gated tick starts from the plan itself. Every
    // gate but a lapse (answered above) follows an engine reset at the
    // carriage, so the plan starts where the carriage stands.
    if (_steer_gap) {
        _steer_gap = false;
        _p_cmd_mm = p_plan_mm;
        _res_v = _res_a = 0.0f;
        _ff_prev = 0.0f;
    }

    // What the carriage fell behind during a stall the LP core rendered
    // through closes at the residual, never as a catch-up.
    const float dt = stall ? kTickDtCapS : dt_s;

    // Feedforward: the trajectory's OWN mean velocity across the interval
    // that just elapsed, rendered over the next. Summed over a move this
    // telescopes to exactly its displacement, which is why the emitter needs
    // no other position input. The mean of a legal curve is under its
    // ceiling over any interval, so a stall renders no burst through it.
    // Into a held edge it is the distance left to the edge, never the plan's
    // displacement past it.
    // THE RESIDUAL against position truth (bd val-xfy2): what the emitter
    // could not render on time (the ramp out of rest, whose first edge
    // periods are long) and the count's quantization past the deadband,
    // measured against where the steers so far sent the carriage: the
    // previous command, one tick of travel behind the plan by construction
    // of the feedforward, never the plan now. A velocity offset of at most
    // one tick of the set's accel, reached under the accel the plan leaves
    // free and changed by at most one tick of the set's jerk: it never joins
    // a plan (handover() does) and never steps the velocity.
    const float c_max = s.lim.amax * dt;
    const float err_mm = _p_cmd_mm - positionMm();
    const float v_ff = (p_plan_mm - _p_cmd_mm) / dt_s;
    _p_cmd_mm = p_plan_mm;
    const float want = std::fabs(err_mm) > kResidualDeadbandMm ? std::clamp(err_mm * kTrackHz, -c_max, c_max) : 0.0f;
    const float room = std::fmax(s.lim.amax - std::fabs(v_ff - _ff_prev) / dt, 0.0f);
    _ff_prev = v_ff;
    const float dv = want - _res_v;
    const float a_to =
        std::copysign(std::fmin(std::fmin(room, std::sqrt(2.0f * s.lim.jmax * std::fabs(dv))), std::fabs(dv) / dt), dv);
    const float da = s.lim.jmax * dt;
    _res_a = std::clamp(a_to, _res_a - da, _res_a + da);
    _res_v = std::clamp(_res_v + _res_a * dt, -c_max, c_max);
    float v = v_ff + _res_v;

    // The emitter floor is a FAULT DETECTOR, never a shaper (architecture.md
    // section 2), so the arbiter holds its own last word: the vmax of the set
    // the plan renders under (the jog set's for a Manual plan and the RETURN,
    // the input set's otherwise), plus the residual's bound. Never the bare
    // ceiling -- at a demand that sits ON vmax the residual has to be allowed
    // above it or it can never close (measured: 25 ms of added lag on the
    // ceiling-limited run). Never the input set's alone under a jog planned
    // above it: a 1500 mm/s jog under a 1000 mm/s input set rendered at 1050
    // mm/s and crawled the rest (measured 2026-10-08: 227 mm in 1.05 s).
    const float v_cap = s.lim.vmax + c_max;
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

    syncFence(s.lo_mm, s.hi_mm);
    _emitter.steer(v);
    _emitter.renew();
}

void MotionArbiter::syncFence(float lo_mm, float hi_mm) {
    // A home cycle searches a stop past each datum's safety margin by up to
    // two search margins (MotionArbiter.h, homing), in the frame of its last
    // relabel; otherwise the backstop's frame. The emitter orders the two
    // stores.
    const bool seeking = _home != HomePhase::idle;
    const float reach = kHomeSafetyMarginMm + 2.0f * kHomeSearchMarginMm;
    const int32_t lo = _origin + fenceCount(seeking ? -reach : lo_mm, false);
    const int32_t hi = _origin + fenceCount(seeking ? _max_rail + reach : hi_mm, true);
    if (lo == _fence_lo && hi == _fence_hi) return;
    _emitter.fence(lo, hi);
    _fence_lo = lo;
    _fence_hi = hi;
}

// ---- homing -----------------------------------------------------------------
// Planner task only, except homeSenseRose(). The seek producer: every leg is a
// constant velocity steered straight onto the emitter, ended by a count or a
// stall, and the engine holds at the count until the cycle ends and is reset
// there. The plan-tracking path, its velocity cap and its residual never
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
    syncFence(backstopLo(), backstopHi());
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
    if (_engine.isBusy(now_us) || joining(now_us)) return homeEnd("the machine is moving", now_us);
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
bool MotionArbiter::popAnomaly(MotionAnomaly& out) {
    EngineAnomaly a;
    if (!_engine.popAnomaly(a)) return false;
    if (a.kind < kAnomalyKinds) ++_anom[a.kind];
    ++_anomalies;
    // A failure is a refused knot.
    if (a.kind == uint8_t(kinetic2::AnomalyKind::KnotRefused)) ++_k2_failures;
    out.kind   = a.kind;
    out.seq    = a.seq;
    out.t_us   = a.t_us;
    // The engine plans the physical frame; a flipped client's window runs the
    // other way (RFC-088).
    out.target = _flipped.load() ? 1.0f - a.target : a.target;
    out.detail = a.detail;
    return true;
}

uint32_t MotionArbiter::drainAnomalies() {
    uint32_t kinds = 0;
    MotionAnomaly a;
    while (popAnomaly(a))
        if (a.kind < kAnomalyKinds) kinds |= 1u << a.kind;
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

MotionCensus MotionArbiter::snapshot(uint64_t now_us) {
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
    c.sample_us      = now_us;
    c.steps          = steps - _origin;
    c.position_mm    = float(c.steps) * kMmPerStep;
    // The strip's head, the plan with the oscillation and the connector
    // summed in: what the steer is following, so the residual stays a
    // tracking error.
    c.plan_mm        = seeking ? c.position_mm : _strip.n != 0 ? _strip.p_mm[0] : toMm(s.pos) + _osc_head;
    c.target_mm      = seeking ? float(_leg_end - _origin) * kMmPerStep : toMm(s.target);
    c.velocity_mm_s  = vel_mm_s;
    c.residual_steps = static_cast<int32_t>(std::lround((c.plan_mm - c.position_mm) * kStepsPerMm));
    c.win_min        = _win_min;
    c.win_max        = _win_max;
    c.rail_mm        = _rail;
    c.input_vmax_mm_s  = inputVmaxMm();
    c.input_amax_mm_s2 = inputAmaxMm();
    c.input_jmax_mm_s3 = inputJmaxMm();
    c.intents        = _intents;
    c.rejected       = _rejected;
    c.stalls         = _stalls;
    c.backstops      = _backstops;
    c.lease_lapses   = _lease_lapses;
    c.planner_stalls = _planner_stalls;
    c.late_plans     = _late_plans;
    c.plans_refused  = _plans_refused;
    c.hard_stops     = _hard_stops + _hard_tails;
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
    c.osc_stream_live = oscStreamLive(now_us);
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
