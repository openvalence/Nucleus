#pragma once

// ValenceMotion -- the motion path's one door: submit an intent, the arbiter
// gates it, the engine plans it, the HP sampler evaluates the plan and the LP
// core renders the edges
// Constraints:
// - THE ARBITER IS THE SOLE CALLER of the emitter (architecture.md section 2).
//   Nothing outside ValenceMotion.cpp writes the LP core's steering, fence or
//   lease words, except main.cpp's boot liveness burst before this path
//   starts, and no input source reaches the engine except through
//   motionSubmit().
// - motionSubmit() may be called from any task. It enqueues and wakes the
//   planner task, which plans AT INTENT ARRIVAL, not on the tick; the steer
//   task renders the plan every tick (ValenceMotion.cpp, the two tasks).
// - Position truth is the LP core's signed edge count, never a number this
//   side integrates. motionCensus().position_mm is that count in millimeters.
// - Millimeters everywhere on this interface. The engine's normalized 0..1
//   window units stop at the .cpp boundary, except for the fields named
//   plan_* below, which are published verbatim on normalized-unit channels.
// - motionCensus() is a COPY taken under the arbiter's spinlock, and it is the
//   ONLY cross-task read of motion state. Nothing else may touch the engine
//   from another task: it mutates on every sample.
// See: .claude/rules/motion-control.md, bd val-091.4, bd val-091.11

#include <array>
#include <cstdint>

#include "hub/valence_config.h"

namespace valence {

// One per kinetic2::AnomalyKind, INCLUDING its index-0 placeholder. The
// 0x1111 per-kind fields and the motion-anomaly kinds are indexed by it, so
// this number and those lists move together or the table re-points.
inline constexpr uint8_t kAnomalyKinds = 7;

// Origin of an intent. It picks the ceiling SET and the gating, nothing else.
// The value is the SPEC 11.4 source id the hub publishes on control-owner, and
// the hub library reports ids 0..3 only: a fifth source needs a Valence change.
enum class MotionSource : uint8_t {
    Manual = 0,   // operator-driven, the jog: jog ceilings, bypasses homed;
                  // under PAUSE only with override (SPEC 11.1)
    Stream = 1,   // machine-driven: input ceilings, every gate applies
    Pattern = 2,  // machine-driven, the classic generator: gated and clamped as
                  // Stream is, but never counted as the live stream (0x1100)
    Advanced = 3  // machine-driven, the advanced generator: gated as Pattern is.
                  // The two generators never share the rail (RFC-093)
};

// Each source's name, indexed by its id. The catalog's control-owner option
// labels are the same strings, pinned by a static_assert in ValenceDevice.cpp.
inline constexpr std::array<const char*, 4> kMotionSourceNames{"Jog", "Stream", "Classic", "Advanced"};

// A point move, or a waveform span when duration_us is nonzero. A Pattern or
// Advanced point is that generator's stop and renders as the brake
// (MotionArbiter.cpp, the Kinetic² boundary): a generator moves by spans.
struct MotionIntent {
    MotionSource source       = MotionSource::Manual;
    float        target_mm    = 0.0f;
    uint32_t     duration_us  = 0;       // 0 = point move, planned at the set's ceilings
    float        end_vel_mm_s = 0.0f;
    bool         has_end_vel  = false;
    // Due time in the engine's clock (esp_timer). 0 = plan at arrival. An
    // anchored commit keeps release jitter between the stream's pacing and the
    // commit out of the rendered geometry; the engine bounds both the lead and
    // how many anchored plans may be parked at once.
    uint64_t     anchor_us    = 0;
    // RFC-087 supersede (SPEC "Supersede, the segments flush"): set on the
    // first segment of a c2h segments bundle only. The arbiter replaces every
    // knot queued at or after its start before planning it. Read only on the
    // segment path.
    bool         supersede    = false;
    // A Stream segment only: more segments are expected for this long after
    // its arrival, the grant's quiet window (SPEC 11.4, RFC-098), so its free
    // knot renders through (Kinetic Engine::expect). 0 = none.
    uint32_t     expect_us    = 0;
};

// One cross-task snapshot of the whole motion plane, refreshed on the motion
// task. Every field is consistent with every other, which is what lets the hub
// publish 0x1100 / 0x1110 / 0x1111 / 0x1020 out of a single read.
struct MotionCensus {
    // ---- position truth and the plan ----
    int32_t  steps          = 0;     // LP signed edge count since the origin
    float    position_mm    = 0.0f;  // that count, in millimeters
    float    plan_mm        = 0.0f;  // where the plan says the carriage should be
                                     // RIGHT NOW. Not the target -- see
                                     // target_mm. Publishing this as tgt makes
                                     // the aim render on top of the position.
    float    demand_mm      = 0.0f;  // last accepted target, post window clamp
    float    target_mm      = 0.0f;  // where the ACTIVE PLAN ends: the aim, not
                                     // the plan's position right now. 0x1100's
                                     // tgt_10um is this and nothing else.
    float    velocity_mm_s  = 0.0f;  // the plan's velocity, signed
    int32_t  residual_steps = 0;     // plan minus rendered, in steps
    float    win_min        = 0.0f;
    float    win_max        = 0.0f;
    float    rail_mm        = 0.0f;  // the rail every source is clamped to:
                                     // max_rail, force_home's stroke, or what
                                     // the home cycle measured
    // The input set's ceilings as the engine plans them, overrides applied:
    // what the PD source's budget judges (PdSource.h).
    float    input_vmax_mm_s  = 0.0f;
    float    input_amax_mm_s2 = 0.0f;

    // ---- emitter ----
    uint32_t edges          = 0;
    uint32_t late           = 0;   // LP deadlines already past on arrival
    uint32_t resteers       = 0;   // LP waits cut short by a new steering word
    uint32_t catchups       = 0;   // LP re-steers whose new period was already
                                   // spent: arithmetic on a rising ramp, not a
                                   // miss (see lp_quad.c)
    uint32_t step_q8        = 0;   // the live steering word; 0 means PARKED
    uint32_t emitter_faults = 0;   // demands past the emitter floor: a FAULT
                                   // DETECTOR count, never a shaping knob
    uint32_t fence_hits     = 0;   // edges the LP fence withheld (lp_quad.c)

    // ---- arbiter ----
    uint32_t intents        = 0;
    uint32_t rejected       = 0;   // denied by a gate
    uint32_t stalls         = 0;   // steer ticks later than kTickDtCapS
                                   // (MotionArbiter.h): each steered without
                                   // a catch-up burst
    uint32_t backstops      = 0;   // position backstop engagements: the plan
                                   // left the active frame and the demand was
                                   // held at its edge (MotionArbiter.cpp)
    uint32_t lease_lapses   = 0;   // the LP core stopped itself on a lease no
                                   // tick renewed for kLeaseUs (MotionArbiter.h)
    uint32_t stack_free     = 0;   // planner task stack high-water headroom, bytes
    bool     homed          = false;
    bool     estop          = false;
    bool     motor_on       = false;  // the motor switch is `on`, as the switch reports it
    bool     power_gate     = false;  // the power gate admits motion: motor_on, or
                                      // always in the bench profile (valence_config.h)
    bool     paused         = false;
    bool     override_mode  = false;  // SPEC 11.1 override: the rail is the operator's
    bool     returning      = false;  // the `return` move is running
    uint32_t returns        = 0;      // completed returns since boot; each one
                                      // is the hub's cue to drop its override bit
    bool     homing         = false;  // a home cycle is asked for or running
    uint32_t homes          = 0;      // completed home cycles since boot; each
                                      // one is the hub's cue to clear home_required
                                      // and to store home_rail_mm as max_rail
    float    home_rail_mm   = 0.0f;   // the usable rail the last completed
                                      // cycle measured: far datum minus home
                                      // datum minus both safety margins
                                      // (MotionArbiter.h); 0 = none since boot
    uint32_t home_fails     = 0;      // cycles that ended unhomed since boot;
                                      // each one is the hub's cue to log why
    uint8_t  home_fail_leg  = 0;      // where the last one ended: 0 the home
                                      // end, 1 the far end
    // Why the last one ended. A STRING LITERAL, static storage, so a census
    // copy on another task never dangles. nullptr = none since boot.
    const char* home_fail_why = nullptr;
    bool     busy           = false;
    bool     stream         = false;  // a Stream intent is the live source

    // ---- the active plan, normalized, as 0x1110 publishes it ----
    uint8_t  mode             = 0;   // the 0x1111 mode select's ordinals (MotionArbiter.cpp PlanStyle)
    uint8_t  plan_kind        = 0;   // the 0x1111 plan_kind select's ordinals
    float    plan_start       = 0.0f;
    float    plan_end         = 0.0f;
    float    plan_cur         = 0.0f;
    float    plan_vel         = 0.0f;  // normalized units/s
    uint32_t plan_duration_us = 0;
    uint32_t plan_elapsed_us  = 0;
    bool     plan_hold        = false;  // the active plan is a hold: timed, its
                                        // start its end (SPEC 9.6)
    // RFC-100 plan.flags: registry plan_flags bits of the piece toward the
    // first pending knot, as the solver decided it; 0 while nothing is
    // pending or a brake is in flight.
    uint8_t  plan_flags       = 0;

    // ---- planner diagnostics, as 0x1111 publishes them ----
    uint32_t plans          = 0;
    uint32_t failures       = 0;
    uint32_t anomalies      = 0;   // every kind, summed
    std::array<uint32_t, kAnomalyKinds> anom{};  // indexed by kinetic2::AnomalyKind
    uint32_t plan_us_last   = 0;
    uint32_t plan_us_max    = 0;
    float    plan_us_avg    = 0.0f;
    uint32_t stream_bundles = 0;
    uint32_t stream_samples = 0;
    uint32_t stream_dropped = 0;

    // ---- odometer, as 0x1020 publishes it ----
    uint32_t strokes        = 0;   // rendered direction reversals
    float    distance_mm    = 0.0f;
    float    peak_mm_s      = 0.0f;
    // ---- the oscillator (RFC-103), as 0x1140 publishes it ----
    bool     osc_active     = false;  // rendering a nonzero amplitude at the plan's head
    float    osc_amplitude  = 0.0f;   // that amplitude, window share (osc.amplitude_effective)
};

// The oscillator's parameters (RFC-103, SPEC 9.7) as 0x3140 speaks them: the
// frequency in Hz, the amplitude a share of the window (the peak, half the
// swing), the shape an osc_shapes number, each dwell a share of the moving
// cycle. The defaults are the catalog's.
struct MotionOsc {
    bool    enabled      = false;
    float   frequency_hz = 10.0f;
    float   amplitude    = 0.01f;
    uint8_t shape        = 0;
    float   dwell_crest  = 0.0f;
    float   dwell_trough = 0.0f;
    bool operator==(const MotionOsc&) const = default;
};

// The engine's tuning as the 0x1030 / 0x1120 / 0x1122 cards speak it. The hub
// delegate OWNS the live set (seeded from motionDefaultTuning(), written by
// 0x3030 / 0x3120) and hands every change to motionSetTuning(); the engine
// only ever holds what the delegate last pushed. Every member is read: a
// member the planner stops reading leaves this struct and the wire together.
struct MotionTuning {
    // Input-set ceiling overrides, NORMALIZED window units. 0 = derive the
    // ceiling from the mm input limits. jmax applies to both sets, because the
    // jog set has no jerk of its own; vmax and amax to the input set only.
    float    jmax_ovr          = 0.0f;
    float    vmax_ovr          = 0.0f;
    float    amax_ovr          = 0.0f;
    // The samples grant's schedule_latency_us, less the motion tick
    // (sampleLatencyUs()).
    uint32_t chase_dense_us    = 0;
    // 0x1030 home_speed, mm/s: the home cycle's approach (MotionArbiter.h,
    // homing). Not engine tuning; it rides this set to reach the planner.
    float    home_speed        = DEFAULT_HOME_SPEED_MM_S;
    // Kinetic² planner options (RFC-108), kinetic2::Config's members of the
    // same names, static_asserted against its defaults.
    float    smoothness        = 0.0f;   // free knots: 0 crisp .. 1 smooth
    float    handle_floor      = 0.15f;  // shortest handle a ceiling fit may leave, share of the piece
    float    trim_max          = 1.0f;   // farthest a knot is trimmed, share of the window span
    uint32_t react_us          = 4000;   // the reaction horizon: a knot arriving mid-motion re-plans from this far ahead

    bool operator==(const MotionTuning&) const = default;
};

// Each motion task's stack (the planner's and the steer's), in bytes, and the
// ONE home for that number (C-1): the create sites and main.cpp's high-water
// watch table read it here, so the reported total can never drift from the
// allocated one. Neither is measured under Kinetic² (bd val-4q1); never size
// one down without that measurement.
inline constexpr uint32_t kMotionTaskStackBytes = 24576;

// The sampler period, microseconds, and the ONE home for it (C-1). The S3
// product evaluated its plan at 1 kHz and that number is kept deliberately: it
// is the cadence the whole engine was benched at, the LP core renders every
// edge between ticks regardless, and a faster tick buys nothing because the
// emitter re-reads its steering word mid-wait anyway.
// It is also the hub's whole internal hop for a scheduled plan: the tick at
// or after a plan's anchor steers the emitter with the motion of the interval
// that just elapsed, so execution trails a stamp by one tick.
// schedule_latency_us (RFC-059) is built on it.
inline constexpr uint32_t kMotionTickUs = 1000;

// RFC-059 schedule_latency_us of a samples grant, the one home the hub's grant
// and the arbiter both read. It is exact, not a budget: a sample is a knot
// this long after it arrives (RFC-105 promise 1).
constexpr uint32_t sampleLatencyUs(const MotionTuning& t) { return t.chase_dense_us + kMotionTickUs; }

// Brings up the engine, the arbiter and both motion tasks. The emitter is PARKED
// until an intent is accepted. Must run BEFORE hubBegin(): the hub's boot
// publish of every motion STATE channel reads motionCensus().
bool motionBegin();

// Returns false when a gate denied the intent.
bool motionSubmit(const MotionIntent& intent);

// Absolute, and the one gate no source bypasses. Parks the emitter on the
// calling task before returning, so it does not wait for the motion tick; on
// the board it also drops the motor switch first (SPEC 11.2, H1 path) and
// last zeroes the accessory outputs (ValenceAccessoryIo.h).
void motionEstop();
// The RELEASE (SPEC 11.2): lands in PAUSE, never in motion.
void motionEstopClear();
// The motor switch's host pushes `on` here (MotionArbiter::setMotorPowered()).
// Any task, never blocks; a loss parks the emitter before it returns.
void motionSetMotorPowered(bool on);
// The hub pushes its first-run record here (MotionArbiter::setCommissioned()).
// Any task, never blocks.
void motionSetCommissioned(bool on);
// SPEC 11.1 PAUSE. Any task, never blocks: on refuses every intent from this
// call on, then brakes the plan in flight to rest at the input decel on the
// planner. off is `resume`, the only clear.
void motionPause(bool on);
// RFC-093: a generator's start acquires the rail and its stop releases it
// (MotionArbiter::acquireRail()). False is SOURCE_CONFLICT: the other
// generator holds it. e-stop takes the rail from both; only a start reopens
// it. Any task, never blocks.
bool motionAcquireRail(MotionSource generator);
void motionReleaseRail(MotionSource generator);
// The hub's estop_cuts_power declaration: true, an ESTOP leaves the machine
// unhomed. The composition declares it once on the Hub; the delegate hands
// the Hub's answer here at attach.
void motionSetEstopCutsPower(bool cuts);
// What a `return` request did (MotionArbiter::returnToPause()).
enum class ReturnStart : uint8_t {
    none,        // no override latched: nothing to return from
    queued,      // the jog-set move back is planned on the planner
    arrived,     // unpowered, already at the paused position: override dropped
                 // and returns counted on the spot
    unpowered,   // unpowered and away from the paused position: refused,
                 // override still latched
};
// What a home request did (MotionArbiter::home()).
enum class HomeStart : uint8_t {
    started,     // the cycle is queued on the planner, or already running
    no_sense,    // this build has no home sense line (BoardPins.h)
    undriven,    // nothing drives the sense line: its source is unwired or down
    sense_high,  // the sense already reads a stall: no seek can find the stop
    estop,       // ESTOP latched
    unpowered,   // the motor power gate is shut
};
// Home op 1: the two-leg cycle, home end then far end, each stall re-touched
// slowly for its datum; the usable rail between the safety margins is
// measured (MotionArbiter.h, the homing constants). Any task, never blocks.
// The outcome is the census:
// homing falls, homes counts a completed cycle and home_rail_mm carries its
// measurement; home_fails counts one that ended unhomed. ESTOP, PAUSE, a
// power loss or a window change aborts it, unhomed.
HomeStart motionHome();

// SPEC 11.1 override / return. Any task, never blocks. override latches PAUSE
// first; return answers what it did (ReturnStart).
void motionOverride();
ReturnStart motionReturn();
// RFC-088: the direction flip (MotionArbiter::setFlipped()). Any task; the
// delegate gates it to a homed rail at rest with no source and no override.
void motionSetFlipped(bool on);

// Ceiling sets, in millimeters. Ceilings are clamps, never targets; the one
// exception is a deadline-less Manual point move, which plans AT the jog
// ceilings (architecture.md section 2).
void motionSetJogLimits(float speed_mm_s, float accel_mm_s2);
void motionSetInputLimits(float speed_mm_s, float accel_mm_s2, float jerk_mm_s3);

// The stroke window a Stream source is held inside, plus the physical rail
// every source is held inside. Both come from the 0x1000 machine config, so
// the arbiter and what 0x1000 publishes cannot disagree.
void motionSetWindow(float min_mm, float max_mm, float rail_mm);

// Stream-ingress counters for 0x1111. The hub delegate owns them -- it is the
// only decoder of a bundle -- and pushes them here so ONE census answers the
// whole channel.
void motionNoteStream(uint32_t bundles, uint32_t samples, uint32_t dropped);

// *** HAZARD, RFC-025. DECLARES the machine homed at 0.0 mm and ASSERTS a
// stroke nothing measured. On a rig with a motor attached that is a collision
// hazard: the arbiter will plan moves across a window that may not physically
// exist. It never touches the e-stop latch itself: the hub delegate releases a
// held latch after it, into PAUSE (SPEC 11.2), deliberately -- on a motorless
// rig a latch is the resting state, so gating this op behind it would make it
// unusable for its entire purpose. Control-gated and rate-capped on the wire
// (0x3101 op 2). Returns the stroke actually adopted.
float motionForceHome(float stroke_mm);

MotionCensus motionCensus();
// Any task. The published plan holds the carriage still from now for
// window_us, or no plan renders (MotionArbiter::stillFor()); a home cycle
// moves without one, so read census.homing too.
bool motionStillFor(uint32_t window_us);
// The steer task's stack high-water headroom, bytes; the planner's is the
// census's stack_free. 0 before motionBegin(). Any task.
uint32_t motionSteerStackFree();

// The engine's factory tuning, read from a default-constructed kinetic Config.
// Pure: touches no engine and no task, so any task may call it.
MotionTuning motionDefaultTuning();

// Hands a whole tuning set to the planner, which applies it before it
// plans the next intent. Any task; never blocks (a newer set overwrites an
// unapplied older one, which is the only one that matters). Values arrive
// already clamped to the catalog bounds, which mirror the engine's own clamps.
void motionSetTuning(const MotionTuning& t);

// RFC-103: the oscillator's parameters, clamped by the delegate
// (MotionArbiter::setOscillator()). Any task, never blocks; the planner takes
// the newest at its next tick.
void motionSetOscillator(const MotionOsc& o);

}  // namespace valence
