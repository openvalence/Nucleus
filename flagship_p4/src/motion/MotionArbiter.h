#pragma once

// MotionArbiter -- the motion path's policy core: every safety gate, the window
// clamp, limit-set selection, the engine it plans with, the feedforward it
// steers the emitter by, and the census it publishes
// Constraints:
// - HARDWARE-FREE. No IDF, FreeRTOS or board header here or in the .cpp: the
//   board (ValenceMotion.cpp) and the host twin (sim/valencesim) compile this
//   one copy, so a gate change lands once and both run it. The emitter and the
//   clock are handed IN; a host owns the task, the queues and the lock.
// - THE ARBITER IS THE SOLE CALLER of the emitter (architecture.md section 2):
//   steer() from begin() and evaluate(), park() from estop() and from
//   evaluate() while the latch holds. Nothing else commands it.
// - OWNING-TASK methods (begin, applyTuning, accept, evaluate, drainAnomalies,
//   snapshot) touch the engine and run on ONE task, the host's motion task.
//   accept() calls commit(), which nests KB-scale Ruckig temporaries on that
//   task's stack (T1, memory-budget.md T21).
// - CROSS-TASK methods (estop, pause, override, returnToPause, allowPattern,
//   setEstopCutsPower, setMotorPowered, setCommissioned, the limit and window setters,
//   forceHome, noteStream) never touch the engine.
//   They write flags and scalars the owning task reads on its next pass;
//   estop() and a power loss also park the emitter on the CALLING task,
//   because an e-stop that waits for a tick is not one.
// - The object holds a kinetic::Engine (KB-scale). Host it at file scope in
//   INTERNAL RAM: never a stack local, never PSRAM, which is unreachable while
//   the flash cache is off and the sampler must not fault during an OTA write.
// - Millimeters on this interface, normalized 0..1 window units inside the
//   engine. The two never mix in one expression.
// See: ValenceMotion.h, .claude/rules/motion-control.md, bd val-sf7.1

#include <array>
#include <atomic>
#include <cstdint>

#include "ValenceMotion.h"
#include "hub/valence_config.h"
#include "kinetic/kinetic.hpp"

namespace valence {

// ---- machine constants ------------------------------------------------------

// One quadrature transition is one step. 20 mm/s is 4,172 transitions/s.
inline constexpr float kStepsPerMm = 208.608f;
inline constexpr float kMmPerStep  = 1.0f / kStepsPerMm;

// The LP core's clock, MEASURED, not declared: 50,000 edges per 5 s at 4,000
// cycles per edge with RTC_FAST on the XTAL. Its config home is
// sdkconfig.defaults; the measurement lives in .claude/rules/motion-control.md.
inline constexpr float kLpClockHz = 40.0e6f;

// cycles_per_edge = f_LP / (|v| * steps_per_mm), carried in Q8. The Q8 product
// overflows a 32-bit word below this speed: 1.024e10 / (2^32 * 208.608). One
// step at that rate takes 417 ms, so there is nothing under it worth steering.
inline constexpr float kParkMmS = 0.0115f;

// The emitter's own floor, ~4,790 mm/s: five times the machine's speed ceiling
// and therefore unreachable through the engine. A demand past it is a FAULT
// DETECTOR reading, never a shaper -- the ceilings are enforced in the engine.
inline constexpr uint32_t kMinCyclesPerEdge = 40;

// Intent queue depth, for whichever queue a host puts in front of accept(). A
// bundle carries up to limits::bundle_max_samples (32) samples and the hub
// delegate submits them in one pass, so the queue has to absorb a whole bundle
// plus whatever the previous one left. THE QUEUE IS NOT THE SCHEDULE: an
// anchored commit past the engine's own kScheduleDepth is refused by the
// engine and counted as a plan failure, which is the honest place for that
// ceiling to live.
inline constexpr uint32_t kIntentQueueDepth = 40;

// ---- the steering word ------------------------------------------------------

// What the emitter's two shared words must hold for a velocity. step_q8 == 0
// is PARKED, and a NaN parks. floored means the demand passed the emitter's
// own floor and was clamped to it: count it, never shape with it.
struct SteerWord {
    uint32_t step_q8 = 0;
    bool     forward = true;
    bool     floored = false;
};

SteerWord steerWord(float v_mm_s);

// ---- the emitter seam -------------------------------------------------------

// Whatever renders edges from a velocity and counts them. The count IS
// position truth; nothing on the arbiter's side integrates one.
class MotionEmitter {
public:
    // Signed edge count. Any task.
    virtual int32_t count() const = 0;
    // Owning task only.
    virtual void steer(float v_mm_s) = 0;
    // Any task: stop rendering now. One word store, no lock, no engine.
    virtual void park() = 0;

protected:
    ~MotionEmitter() = default;
};

// ---- the arbiter ------------------------------------------------------------

class MotionArbiter {
public:
    // The engine's clock, microseconds. Read only to time commit().
    using Clock = uint64_t (*)();

    MotionArbiter(MotionEmitter& emitter, Clock now_us) : _emitter(emitter), _now_us(now_us) {}

    // Owning task.
    // Before the owning task exists, or on it. Parks and zeroes the origin.
    void begin(uint64_t now_us);
    void applyTuning(const MotionTuning& t);
    bool accept(const MotionIntent& in, uint64_t now_us); // gates, clamp, commit
    void evaluate(uint64_t now_us, float dt_s);
    void drainAnomalies();
    // Every census field the arbiter owns. The emitter's counters (edges,
    // late, resteers, catchups, step_q8, emitter_faults) and stack_free are
    // the host's to fill: they are facts about its hardware and its task.
    MotionCensus snapshot(uint64_t now_us);

    // Any task.
    // SPEC 11.2. on: parks the emitter on the CALLING task, closes the Pattern
    // gate, and drops homed when the hub declares estop_cuts_power. off is the
    // RELEASE and lands in PAUSE, never in motion.
    void estop(bool on);
    // SPEC 11.1 PAUSE. on: latches, THEN asks the owning task to brake the
    // plan in flight to rest at the input decel. That order is the guarantee:
    // a half-stroke or a stream sample queued before the pause is either
    // refused at accept() or already accepted and braked. off is `resume`, the
    // only clear; the hub refuses it while ESTOP, override or home_required
    // holds, so this never sees those cases.
    void pause(bool on);
    // SPEC 11.1 OVERRIDE: latches PAUSE first (override never exists without
    // it), then hands the rail to the operator: a Manual intent is the one
    // motion accepted, at the jog set, anywhere on the rail. ESTOP drops it.
    void override();
    // SPEC 11.1 RETURN: the owning task plans a jog-set move back to where the
    // pause brought the machine to rest; on arrival override drops and
    // returns() counts once. Manual intents are refused while it runs. A no-op
    // without override.
    void returnToPause() { if (_override.load()) _return_req.store(true); }
    // Reopens the Pattern gate estop(true) closed. The generator's own start
    // is the one caller.
    void allowPattern() { _pattern_stopped.store(false); }
    // The hub's estop_cuts_power declaration (SPEC 11.2): true, the motor is
    // limp after an ESTOP and the position reference is gone.
    void setEstopCutsPower(bool cuts) { _cuts_power = cuts; }
    // The motor switch's word, pushed by its host on every entry into and exit
    // from `on` (ValenceMotorSwitch.h). Off refuses EVERY intent, the jog
    // under override included, like the e-stop gate. Losing power parks the
    // emitter on the CALLING task and drops homed: a limp carriage keeps no
    // position reference. Boots off: nothing moves before the switch says on.
    void setMotorPowered(bool on);
    // The hub's first-run record (RFC-079 setup, StoredState.h): false until
    // the owner has written every required setup field once. False refuses
    // Stream and Pattern; Manual still moves, exactly as unhomed, because an
    // owner commissioning a machine must be able to move it. Boots false.
    void setCommissioned(bool on) { _commissioned.store(on); }
    void setJogLimits(float v, float a) { _jog_v = v; _jog_a = a; }
    // RFC-088 (SPEC 9.6): with the flip on, position 0 is the far end. Every
    // intent target is mirrored against the rail on the way in, and every
    // position, velocity and window in snapshot() on the way out; the engine,
    // the emitter and the window this class holds stay physical. The hub
    // delegate gates the change (at rest, homed, no source, no override).
    void setFlipped(bool on) { _flipped.store(on); }
    void setInputLimits(float v, float a, float j) { _in_v = v; _in_a = a; _in_j = j; }
    void setWindow(float lo, float hi, float rail);
    float forceHome(float stroke_mm);
    void noteStream(uint32_t bundles, uint32_t samples, uint32_t dropped);

    float positionMm() const { return float(_emitter.count() - _origin) * kMmPerStep; }
    float winMin() const { return _win_min; }
    float winMax() const { return _win_max; }
    float rail() const { return _rail; }

private:
    // The engine's construction config: kinetic's defaults plus the protocol
    // values kinetic is handed rather than spelling (the dwell span).
    static kinetic::Config engineConfig();

    // The engine's frame: its normalized 0..1 is the travel window, or the
    // whole rail while the operator jogs under override (the engine clamps to
    // its frame, so lifting the window means widening the frame).
    float frameLo() const { return _rail_frame ? 0.0f : _win_min; }
    float span() const { return _rail_frame ? _rail : _win_max - _win_min; }
    float toNorm(float mm) const { return (mm - frameLo()) / span(); }
    float toMm(float norm) const { return frameLo() + norm * span(); }
    float winSpan() const { return _win_max - _win_min; }
    // The INPUT set's ceilings as the engine plans them, in mm: the mm limit,
    // or the normalized override scaled by the CURRENT window. evaluate()'s
    // tracking cap reads the same answer, so a plan an override allowed is
    // never capped below its own ceiling at render time.
    float inputVmaxMm() const { return _ovr_v > 0.0f ? _ovr_v * winSpan() : _in_v; }
    float inputAmaxMm() const { return _ovr_a > 0.0f ? _ovr_a * winSpan() : _in_a; }
    // Moves the engine's frame at rest: reseeds it at the carriage, so the
    // move is a relabeling, never motion.
    void setRailFrame(bool on, uint64_t now_us);
    kinetic::Limits limitsFor(bool manual) const;
    void brakeToRest(uint64_t now_us);
    // Plans `target` (already clamped, mm) from the machine's actual state.
    // Owning task; counts the plan cost and a failure as a rejection.
    bool plan(float target, const MotionIntent& in, bool manual, uint64_t now_us);

    MotionEmitter& _emitter;
    Clock          _now_us;

    kinetic::Engine _engine{engineConfig()};

    float _win_min = 0.0f;
    float _win_max = DEFAULT_MAX_RAIL_MM;
    float _rail    = DEFAULT_MAX_RAIL_MM;

    float _jog_v = DEFAULT_JOG_MAX_SPEED_MM_S;
    float _jog_a = DEFAULT_JOG_ACCEL_MM_S2;
    float _in_v   = DEFAULT_MAX_SPEED_MM_S;
    float _in_a   = DEFAULT_ACCEL_MM_S2;
    float _in_j   = DEFAULT_INPUT_MAX_JERK_MM_S3;
    // Normalized ceiling overrides from 0x3120, 0 = derived. Owning task only:
    // written by applyTuning(), read by accept() and evaluate().
    float _ovr_v = 0.0f;
    float _ovr_a = 0.0f;
    float _ovr_j = 0.0f;

    int32_t _origin   = 0;       // the emitter count that means 0.0 mm
    float   _p_cmd_mm = 0.0f;    // the plan position at the previous tick

    volatile bool _homed  = false;
    volatile bool _estop  = false;
    // Written by setMotorPowered() on the switch host's task. _power_settled
    // re-arms on a loss and is spent once by evaluate() on the owning task.
    std::atomic<bool> _powered{false};
    std::atomic<bool> _commissioned{false};
    std::atomic<bool> _power_settled{false};
    volatile bool _cuts_power = true;
    bool _estop_settled = false;  // the engine has been reset since the latch
    // Written by pause()/estop()/allowPattern() on any task. Atomics, not
    // volatile: the gate store must be visible before the brake request is.
    std::atomic<bool> _paused{false};
    std::atomic<bool> _override{false};
    std::atomic<bool> _flipped{false};
    std::atomic<bool> _return_req{false};
    std::atomic<bool> _pattern_stopped{false};
    std::atomic<bool> _brake_req{false};
    // Owning task only. _pause_pos_mm is where the last pause brought the
    // machine to rest (the return target); _returns counts completed returns.
    bool     _rail_frame  = false;
    bool     _returning   = false;
    float    _pause_pos_mm = 0.0f;
    uint32_t _returns     = 0;
    // Set by setWindow()/forceHome() on any task, consumed by evaluate() on
    // the owning task: the mm FRAME moved, the carriage did not.
    volatile bool _frame_moved = false;

    // Odometer state, owning task only.
    int32_t _odo_steps = 0;       // emitter count at the previous snapshot
    int32_t _stroke_dir = 0;      // sign of the run in progress
    float   _stroke_run_mm = 0.0f;

    // Owning-task counters. They live outside any published census so a host
    // publishes only what snapshot() built, under its own lock.
    uint32_t _intents  = 0;
    uint32_t _rejected = 0;
    uint32_t _plan_us_last = 0;
    uint32_t _plan_us_max  = 0;
    float    _plan_us_avg  = 0.0f;
    float    _demand_mm = 0.0f;
    bool     _stream    = false;
    uint32_t _anomalies = 0;
    std::array<uint32_t, kAnomalyKinds> _anom{};
    float    _distance_mm = 0.0f;
    float    _peak_mm_s   = 0.0f;
    uint32_t _strokes     = 0;

    // Stream ingress, written through noteStream() by the hub. Relaxed
    // atomics: counters nothing orders against, and folding them into the
    // census under a lock would put the motion lock on the bundle decode path.
    std::atomic<uint32_t> _sync_bundles{0};
    std::atomic<uint32_t> _sync_samples{0};
    std::atomic<uint32_t> _sync_dropped{0};
};

}  // namespace valence
