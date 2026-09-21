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
//   window units stop at the .cpp boundary.
// See: .claude/rules/motion-control.md, bd val-091.4

#include <cstdint>

namespace valence {

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
};

struct MotionCensus {
    int32_t  steps          = 0;   // LP signed edge count since the origin
    float    position_mm    = 0.0f;  // that count, in millimeters
    float    plan_mm        = 0.0f;  // where the plan says the carriage should be
    int32_t  residual_steps = 0;   // plan minus rendered, in steps
    uint32_t edges          = 0;
    uint32_t late           = 0;   // LP deadlines already past on arrival
    uint32_t resteers       = 0;   // LP waits cut short by a new steering word
    uint32_t catchups       = 0;   // LP re-steers whose new period was already
                                   // spent: arithmetic on a rising ramp, not a
                                   // miss (see lp_quad.c)
    uint32_t intents        = 0;
    uint32_t rejected       = 0;   // denied by a gate
    uint32_t emitter_faults = 0;   // demands past the emitter floor: a FAULT
                                   // DETECTOR count, never a shaping knob
    uint32_t stack_free     = 0;   // motion task stack high-water headroom, bytes
    bool     homed          = false;
    bool     estop          = false;
    bool     busy           = false;
};

// Brings up the engine, the arbiter and the motion task. The emitter is PARKED
// until an intent is accepted.
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

MotionCensus motionCensus();

#ifdef VALENCE_BENCH_MOTION
// BENCH ONLY, and compiled only into this image: VALENCE_BENCH_MOTION is set
// in flagship_p4/src/CMakeLists.txt and nowhere else, so a product build that
// reuses this file cannot reach either of these.
//
// This board has no drive and no encoder, so it CANNOT home. The arbiter's
// homed gate is real code; this asserts the postcondition a homing ritual
// would have established -- carriage at 0 mm, window 0..max rail -- so the
// gated path can be exercised on a bench that has nothing to home against.
void motionBenchAssumeHomed();

// Starts the bench intent task: the +20 / -20 / reversal sequence, repeated,
// reporting the rendered-versus-planned step count each cycle. THE SEAM THIS
// STANDS IN FOR is the hub delegate's move intent; see ValenceHub.cpp.
void motionBenchStart();
#endif

}  // namespace valence
