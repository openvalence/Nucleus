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
enum class MotionSource : uint8_t {
    Manual = 0,   // operator-driven: user ceilings, bypasses every gate but e-stop
    Stream = 1    // machine-driven: input ceilings, every gate applies
};

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
    bool     paused         = false;
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

// The engine's live tuning, for the 0x1120/0x1121/0x1122 cards. Every value is
// what the engine actually holds. NOTHING on this board writes them, which is
// why those cards publish an all-zero enabled_mask (bd val-091.11).
struct MotionTuning {
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
};

// The motion task's stack, in bytes, and the ONE home for that number (C-1):
// the create site and main.cpp's high-water watch table both read it here, so
// the reported total can never drift from the allocated one. The measurement
// that set it is on the create site in ValenceMotion.cpp.
inline constexpr uint32_t kMotionTaskStackBytes = 24576;

// Brings up the engine, the arbiter and the motion task. The emitter is PARKED
// until an intent is accepted. Must run BEFORE hubBegin(): the hub's boot
// publish of every motion STATE channel reads motionCensus().
bool motionBegin();

// Returns false when a gate denied the intent.
bool motionSubmit(const MotionIntent& intent);

// Absolute, and the one gate no source bypasses. Parks the emitter on the
// calling task before returning, so it does not wait for the motion tick.
void motionEstop();
void motionEstopClear();
void motionPause(bool on);

// Ceiling sets, in millimeters. Ceilings are clamps, never targets; the one
// exception is a deadline-less Manual point move, which plans AT the user
// ceilings (architecture.md section 2).
void motionSetUserLimits(float speed_mm_s, float accel_mm_s2);
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
// exist. It also CLEARS THE E-STOP LATCH, deliberately -- on a motorless rig a
// latch is the resting state, so gating this op behind it would make it
// unusable for its entire purpose. Control-gated and rate-capped on the wire
// (0x3101 op 2). Returns the stroke actually adopted.
float motionForceHome(float stroke_mm);

MotionCensus motionCensus();
MotionTuning motionTuning();

}  // namespace valence
