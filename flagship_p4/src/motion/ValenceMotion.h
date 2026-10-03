#pragma once

// ValenceMotion -- the motion path's one door: submit an intent, the arbiter
// gates it, the engine plans it, the HP sampler evaluates the plan and the LP
// core renders the edges
// Constraints:
// - THE ARBITER IS THE SOLE CALLER of the emitter (architecture.md section 2).
//   Nothing outside ValenceMotion.cpp writes ulp_g_step_q8 or ulp_g_dir, and
//   no input source reaches the engine except through motionSubmit().
// - motionSubmit() may be called from any task. It enqueues and wakes the
//   motion task, which plans AT INTENT ARRIVAL, not on the tick.
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

namespace valence {

// One per kinetic::AnomalyType, INCLUDING its index-0 placeholder and the
// retired kind that holds its ordinal. The enum is append-only and the 0x1111
// per-kind field list is indexed by it, so this number and that list move
// together or the table re-points.
inline constexpr uint8_t kAnomalyKinds = 11;

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

// A point move, or a waveform span when duration_us is nonzero.
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
    // RFC-030 declared curve family, registry curve_families numbering.
    // 0 = unspecified. Read only on the waveform path.
    uint8_t      curve_family = 0;
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
    float    rail_mm        = 0.0f;  // the stroke force_home asserted, 0 = none
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

    // ---- arbiter ----
    uint32_t intents        = 0;
    uint32_t rejected       = 0;   // denied by a gate
    uint32_t stack_free     = 0;   // motion task stack high-water headroom, bytes
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
    bool     busy           = false;
    bool     stream         = false;  // a Stream intent is the live source

    // ---- the active plan, normalized, as 0x1110 publishes it ----
    uint8_t  mode             = 0;   // kinetic::Mode
    uint8_t  plan_kind        = 0;   // kinetic::PlanKind
    float    plan_start       = 0.0f;
    float    plan_end         = 0.0f;
    float    plan_cur         = 0.0f;
    float    plan_vel         = 0.0f;  // normalized units/s
    uint32_t plan_duration_us = 0;
    uint32_t plan_elapsed_us  = 0;
    bool     plan_hold        = false;  // the active plan is a hold: timed, its
                                        // start its end (SPEC 9.6)

    // ---- planner diagnostics, as 0x1111 publishes them ----
    uint32_t plans          = 0;
    uint32_t failures       = 0;
    uint32_t anomalies      = 0;   // every kind, summed
    std::array<uint32_t, kAnomalyKinds> anom{};  // indexed by kinetic::AnomalyType
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
};

// The engine's tuning as the 0x1030 / 0x1120-0x1122 cards speak it. The hub
// delegate OWNS the live set (seeded from motionDefaultTuning(), written by
// 0x3030 / 0x3120) and hands every change to motionSetTuning(); the engine
// only ever holds what the delegate last pushed.
struct MotionTuning {
    // Input-set ceiling overrides, NORMALIZED window units. 0 = derive the
    // ceiling from the mm input limits. jmax applies to both sets, because the
    // jog set has no jerk of its own; vmax and amax to the input set only.
    float    jmax_ovr          = 0.0f;
    float    vmax_ovr          = 0.0f;
    float    amax_ovr          = 0.0f;
    bool     chase_ff          = false;
    bool     chase_accel_ff    = false;
    float    chase_gain        = 0.0f;
    float    chase_lookahead   = 0.0f;
    uint32_t chase_dense_us    = 0;
    bool     chase_aim_extrap  = false;
    float    handoff_k         = 0.0f;
    uint8_t  curve_policy      = 0;   // catalog ordinal: 0 follow, 1 C1, 2 C2
    uint8_t  infeasible_policy = 0;   // catalog ordinal: 0 stretch, 1 blend
    float    smooth_budget     = 0.0f;
    float    amplitude_budget  = 0.0f;
    uint8_t  blend_steps       = 0;
    uint32_t settle_grace_us   = 0;
    // 0x1030 overshoot_clamp. kinetic's overshoot_guard, 0 = disarmed; "on" is
    // the engine's own factory multiplier, never a value this side invents.
    float    overshoot_guard   = 0.0f;

    bool operator==(const MotionTuning&) const = default;
};

// The motion task's stack, in bytes, and the ONE home for that number (C-1):
// the create site and main.cpp's high-water watch table both read it here, so
// the reported total can never drift from the allocated one. The measurement
// that set it is on the create site in ValenceMotion.cpp.
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

// Brings up the engine, the arbiter and the motion task. The emitter is PARKED
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
// motion task. off is `resume`, the only clear.
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
    queued,      // the jog-set move back is planned on the motion task
    arrived,     // unpowered, already at the paused position: override dropped
                 // and returns counted on the spot
    unpowered,   // unpowered and away from the paused position: refused,
                 // override still latched
};
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

// The engine's factory tuning, read from a default-constructed kinetic Config.
// Pure: touches no engine and no task, so any task may call it.
MotionTuning motionDefaultTuning();

// Hands a whole tuning set to the motion task, which applies it before it
// plans the next intent. Any task; never blocks (a newer set overwrites an
// unapplied older one, which is the only one that matters). Values arrive
// already clamped to the catalog bounds, which mirror the engine's own clamps.
void motionSetTuning(const MotionTuning& t);

}  // namespace valence
