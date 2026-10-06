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
//   task's stack (T1, memory-budget.md T21). Under Kinetic² the window solve
//   runs lazily in the first sample after a submit, on the same task, with a
//   copy of the pending knots on its stack.
// - THE KERNEL IS A BUILD SWITCH (ValenceMotion.h NUCLEUS_KINETIC2). Every
//   conversion between this interface and Kinetic²'s knots lives in
//   MotionArbiter.cpp's Kinetic² boundary section, nowhere else.
// - CROSS-TASK methods (estop, pause, override, returnToPause, acquireRail,
//   releaseRail, setEstopCutsPower, setMotorPowered, setCommissioned, the limit and window setters,
//   forceHome, home, noteStream) never touch the engine.
//   They write flags and scalars the owning task reads on its next pass;
//   estop() and a power loss also park the emitter on the CALLING task,
//   because an e-stop that waits for a tick is not one.
// - The object holds the engine (KB-scale either way). Host it at file scope in
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
#if NUCLEUS_KINETIC2
#include "kinetic2/engine.hpp"
#include "kinetic2/sources.hpp"
#else
#include "kinetic/kinetic.hpp"
#endif

namespace valence {

// The kernel the arbiter plans with (ValenceMotion.h kKinetic2). Kinetic² is
// one axis here; its timeline holds 64 knots, two full 32-sample bundles.
#if NUCLEUS_KINETIC2
using MotionEngine = kinetic2::Engine<1, 64>;
using EngineLimits = kinetic2::Limits;
using EngineConfig = kinetic2::Config;
using EngineAnomaly = kinetic2::Anomaly;
#else
using MotionEngine = kinetic::Engine;
using EngineLimits = kinetic::Limits;
using EngineConfig = kinetic::Config;
using EngineAnomaly = kinetic::Anomaly;
#endif

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

// ---- homing -----------------------------------------------------------------
// Home op 1 runs two legs, the home end first, then the far end. Each leg:
// approach at the home speed (the home_speed tuning, held to the jog speed)
// until the home sense reads HIGH; the stop profile (PAUSE's brake); back off
// kHomeRetouchBackoffMm at the home speed, where the line must read LOW;
// re-approach at homeTouchMmS(), and that stall, moved back by the touch
// speed times kHomeSenseLatencyUs, is the leg's datum. The home end's datum is
// 0.0 mm and the far end's, minus it, is the measured rail length, which
// becomes the rail. Then kHomeBackoffMm off the far end, where the line must
// read LOW again and the count has caught the plan, and homed. The home
// approach searches max_rail plus kHomeFrameMarginMm, so a retry reaches the
// home stop from where a failed far approach stopped; the far approach
// searches max_rail plus kHomeSearchMarginMm from the home datum; a re-touch
// reaches kHomeSearchMarginMm past the stall it backed off from. No stall within
// either, a rail under MIN_RAIL_MM, or no end by homeCycleS() plus
// kHomeTimeoutMarginUs fails it unhomed.
//
// A CYCLE PUBLISHES THE PHYSICAL FRAME. The engine frame is the rail widened
// by kHomeFrameMarginMm at each end, so every target past a stop is inside
// the engine's clamp, and the home end's stall reads its own label (0.0 mm, or
// max_rail under the flip) from the first approach on. Every position
// published during a cycle, and after one that fails, is measured from the
// home stop; the far leg relabels nothing (bd val-tib).
//
// THE STALL IS THE SENSE SOURCE'S DECISION. The input contract: one level,
// active HIGH, push-pull, HIGH while the motor is pressed against a stop. On
// the bench the source is an external current-sense board (s3-home-sense
// src/main.cpp, the same names): the INA228 converts the shunt every
// kStallConversionUs; HIGH after kStallOnSamples consecutive samples at or
// over kStallOnA (1.0 A), or on the first sample at or over kStallOnA that
// rose faster than kStallRiseAPerMs (5 A/ms) across kStallRiseSamples (2);
// LOW after kStallOffSamples (3) under kStallOffA (0.5 A). This side confirms
// the line with two reads kHomeSenseDebounceUs apart, its rise wakes the
// motion task (ValenceHomeSense.cpp), and nothing else. kHomeSenseLatencyUs is
// the two ends' shared contact-to-brake figure; a change to the source's
// numbers moves it here.
inline constexpr uint32_t kStallConversionUs    = 150;    // the S3's INA228 shunt conversion
inline constexpr uint32_t kStallOnSamples       = 3;      // the S3's N
inline constexpr uint32_t kStallDetectUs        = kStallOnSamples * kStallConversionUs;
inline constexpr float    kHomeRetouchBackoffMm = 5.0f;   // from an approach's stall
inline constexpr float    kHomeBackoffMm        = 2.0f;   // final, from the far datum
inline constexpr float    kHomeSearchMarginMm   = 10.0f;
inline constexpr float    kHomeFrameMarginMm    = 2.0f * kHomeSearchMarginMm;
inline constexpr float    kHomeTouchDivisor     = 4.0f;
inline constexpr float    kHomeTouchFloorMmS    = 8.0f;
inline constexpr uint32_t kHomeSenseDebounceUs  = 100;    // the board's two reads of a rise
inline constexpr uint32_t kHomeTimeoutMarginUs  = 2000000;
inline constexpr uint32_t kHomeSenseLatencyUs   = kStallDetectUs + kHomeSenseDebounceUs;

// The re-touch speed for an approach speed: a quarter of it, floored, never
// above the approach itself.
constexpr float homeTouchMmS(float home_mm_s) {
    const float quarter = home_mm_s / kHomeTouchDivisor;
    const float floored = quarter > kHomeTouchFloorMmS ? quarter : kHomeTouchFloorMmS;
    return floored < home_mm_s ? floored : home_mm_s;
}

// How much longer than distance over speed a speed-bound point move takes.
// Kinetic² renders one as a rest-to-rest quintic at its analytic minimum time,
// whose peak speed is 1.875 times its mean (RFC-105 (k)); Kinetic cruises at
// the speed.
inline constexpr float kPointMoveSlowdown = kKinetic2 ? 1.875f : 1.0f;

// The cycle's own time, before kHomeTimeoutMarginUs: both approaches across
// their whole search and the three backoffs at the home speed, both re-touches
// across their whole reach at the touch speed, and an accel and a decel ramp
// on each of the seven moves.
constexpr float homeCycleS(float max_rail_mm, float home_mm_s, float accel_mm_s2) {
    const float fast = (max_rail_mm + kHomeFrameMarginMm) + (max_rail_mm + kHomeSearchMarginMm) +
                       2.0f * kHomeRetouchBackoffMm + kHomeBackoffMm;
    const float slow = 2.0f * (kHomeRetouchBackoffMm + kHomeSearchMarginMm);
    return kPointMoveSlowdown * (fast / home_mm_s + slow / homeTouchMmS(home_mm_s)) +
           7.0f * home_mm_s / accel_mm_s2;
}

// At the factory speeds. A faster home speed is checked where it runs: the
// line must read LOW after each backoff, or the cycle fails.
static_assert(DEFAULT_HOME_SPEED_MM_S * float(kHomeSenseLatencyUs) * 1e-6f < 0.5f * kHomeRetouchBackoffMm,
              "the approach's overrun past contact must stay inside half the re-touch backoff");
static_assert(homeTouchMmS(DEFAULT_HOME_SPEED_MM_S) * float(kHomeSenseLatencyUs) * 1e-6f < 0.5f * kHomeBackoffMm,
              "the re-touch's overrun past contact must stay inside half the final backoff");
static_assert(kHomeSearchMarginMm > kHomeRetouchBackoffMm,
              "a re-touch must reach past the stall it backed off from");

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

// ---- the home sense seam -----------------------------------------------------

// Whatever reports the carriage pressed against a hard stop, either end: a
// level, HIGH while pressed.
class HomeSense {
public:
    enum class Probe : uint8_t { low, high, undriven };
    // This build has a sense line. Any task; constant for the build.
    virtual bool present() const = 0;
    // The line's state before a cycle. undriven: nothing holds it, so a seek
    // would never see a stall. The calling task, never while a cycle runs.
    virtual Probe probe() = 0;
    // Owning task, during a cycle: HIGH now, a fresh rise confirmed (the
    // board reads it twice, kHomeSenseDebounceUs apart).
    virtual bool high() = 0;

protected:
    ~HomeSense() = default;
};

// The sense of a build without a line: absent, so home() refuses no_sense
// rather than seeking open-loop into the stop.
HomeSense& noHomeSense();

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
    // Returns the kinds drained, bit k = anomaly kind k (kinetic::AnomalyType,
    // or kinetic2::AnomalyKind, which keeps its numbers and appends).
    uint32_t drainAnomalies();
    // Every census field the arbiter owns. The emitter's counters (edges,
    // late, resteers, catchups, step_q8, emitter_faults) and stack_free are
    // the host's to fill: they are facts about its hardware and its task.
    MotionCensus snapshot(uint64_t now_us);

    // Any task.
    // SPEC 11.2. on: parks the emitter on the CALLING task, takes the rail
    // from both generators, and drops homed when the hub declares
    // estop_cuts_power. off is the RELEASE and lands in PAUSE, never in motion.
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
    // returns() counts once. Manual intents are refused while it runs. `none`
    // without override. With the power gate shut nothing can render, so it is
    // decided here, on the calling task: a carriage already within one step
    // of the paused position has arrived (override drops, returns() counts);
    // one further away is refused `unpowered` and override holds.
    ReturnStart returnToPause();
    // SPEC 11.4, RFC-093: the classic and advanced generators are two sources
    // that never share the rail. A generator's start acquires it: true when
    // the rail is free, closed by e-stop, or already this generator's; false
    // is SOURCE_CONFLICT while the other holds it, and the reason goes to the
    // log channel. Its stop releases it, a no-op unless `generator` holds it.
    // A released generator's own brake still lands until the other acquires.
    // Any source but Pattern or Advanced is refused.
    bool acquireRail(MotionSource generator);
    void releaseRail(MotionSource generator);
    // The hub's estop_cuts_power declaration (SPEC 11.2): true, the motor is
    // limp after an ESTOP and the position reference is gone.
    void setEstopCutsPower(bool cuts) { _cuts_power = cuts; }
    // The motor switch's word, pushed by its host on every entry into and exit
    // from `on` (ValenceMotorSwitch.h). Off refuses EVERY intent, the jog
    // under override included, like the e-stop gate. Losing power parks the
    // emitter on the CALLING task and drops homed: a limp carriage keeps no
    // position reference. Boots off: nothing moves before the switch says on.
    // The bench profile (kBenchNoMotor, valence_config.h) makes the gate
    // ADVISORY: off still reads off in the census, but admits motion, and the
    // first intent it admits logs "BENCH: motor power gate bypassed" once.
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
    // Before the owning task runs. Until then, and on a build without one,
    // the sense is absent and home() answers no_sense.
    void setHomeSense(HomeSense& sense) { _sense = &sense; }
    // Home op 1 (ValenceMotion.h motionHome()): refused here on what the
    // calling task can see, the sense probed last; started queues the cycle
    // for the owning task. One cycle at a time: a request while one runs
    // answers started and changes nothing.
    HomeStart home();
    void noteStream(uint32_t bundles, uint32_t samples, uint32_t dropped);

    float positionMm() const { return float(_emitter.count() - _origin) * kMmPerStep; }
#if NUCLEUS_KINETIC2
    // Owning task. The planned state at now_us, engine frame, for host tooling
    // (tools/kinetic-wasm): the same sample evaluate() and snapshot() take, so
    // reading it at their time changes nothing. Nothing on the board calls it.
    kinetic2::State planState(uint64_t now_us) { return sampleEngine(now_us); }
#else
    // Owning task. Read-only, for host tooling that evaluates the plan in
    // double with no side effect (Engine::planView + evalPiece): the offline
    // planner, tools/kinetic-wasm. Nothing on the board reads the engine here.
    const MotionEngine& engine() const { return _engine; }
#endif
    float winMin() const { return _win_min; }
    float winMax() const { return _win_max; }
    float rail() const { return _rail; }

private:
    // The engine's construction config: the kernel's defaults plus the
    // protocol values it is handed rather than spelling (Kinetic's dwell span).
    static EngineConfig engineConfig();

    // The active plan as the census reads it, in the engine frame, from
    // whichever kernel this build runs. mode and plan_kind are kinetic::Mode's
    // and kinetic::PlanKind's ordinals under both.
    struct PlanRead {
        float    pos = 0.0f, vel = 0.0f, start = 0.0f, target = 0.0f;
        float    duration_s = 0.0f, elapsed_s = 0.0f;
        uint8_t  mode = 0, plan_kind = 0, flags = 0;
        uint32_t plans = 0, failures = 0;
    };
    PlanRead readPlan(uint64_t now_us);
    // Forgets the plan and holds at `p_norm` from now_us. Every resetAt() goes
    // through here, so the Kinetic² boundary state resets with the engine.
    void resetEngine(float p_norm, uint64_t now_us);
    // Stops as fast as the engine's current limits allow, from its own
    // state at at_us. False when there was nothing moving to stop.
    bool brakeEngine(uint64_t at_us);
    void notePlanCost(uint32_t us);

    // The engine's frame: its normalized 0..1 is the travel window, or the
    // whole rail while the operator jogs under override (the engine clamps to
    // its frame, so lifting the window means widening the frame), widened by
    // _rail_margin at each end while a home cycle holds it.
    float frameLo() const { return _rail_frame ? -_rail_margin : _win_min; }
    float span() const { return _rail_frame ? _rail + 2.0f * _rail_margin : _win_max - _win_min; }
    float toNorm(float mm) const { return (mm - frameLo()) / span(); }
    float toMm(float norm) const { return frameLo() + norm * span(); }
    float winSpan() const { return _win_max - _win_min; }
    // The INPUT set's ceilings as the engine plans them, in mm: the mm limit,
    // or the normalized override scaled by the CURRENT window. evaluate()'s
    // tracking cap reads the same answer, so a plan an override allowed is
    // never capped below its own ceiling at render time.
    float inputVmaxMm() const { return _ovr_v > 0.0f ? _ovr_v * winSpan() : _in_v; }
    float inputAmaxMm() const { return _ovr_a > 0.0f ? _ovr_a * winSpan() : _in_a; }
    // Moves the engine's frame at rest, dropping a home cycle's margin:
    // reseeds it at the carriage, so the move is a relabeling, never motion.
    void setRailFrame(bool on, uint64_t now_us);
    EngineLimits limitsFor(bool manual) const;
    void brakeToRest(uint64_t now_us);
    // The power gate as accept(), evaluate() and returnToPause() apply it.
    bool powerGateOpen() const { return kBenchNoMotor || _powered.load(); }
    // Plans `target` (already clamped, mm) from the machine's actual state
    // under `lim`. Owning task; counts the plan cost and a failure as a
    // rejection.
    bool plan(float target, const MotionIntent& in, const EngineLimits& lim, uint64_t now_us);
#if NUCLEUS_KINETIC2
    // The Kinetic² boundary (MotionArbiter.cpp): an intent becomes a knot, a
    // brake or a refusal here and nowhere else.
    bool submitKnots(float target_norm, const MotionIntent& in, const EngineLimits& lim, uint64_t now_us);
    // The one door to the engine's stateAt(): a knot that ends the timeline
    // still moving is braked from its own time first (RFC-105 (a)).
    kinetic2::State sampleEngine(uint64_t now_us);
#endif
    // The home cycle, owning task only (MotionArbiter.cpp, homing). A leg:
    // approach, approach_stop, clear (the backoff), touch, touch_stop; then
    // the far leg, or finish (the final backoff).
    enum class HomePhase : uint8_t { idle, approach, approach_stop, clear, touch, touch_stop, finish };
    void homeStart(uint64_t now_us);
    void homeStep(uint64_t now_us);
    // The far datum is in: the rail is measured, the frame becomes it, and
    // the final backoff is planned.
    void homeMeasure(uint64_t now_us);
    // Plans the leg's approach (`touch` false) or re-touch at its speed, and
    // arms the sense: at once when the line is known LOW, else on its first
    // LOW read (the far approach starts pressed against the home stop).
    void homeSeek(bool touch, bool armed, uint64_t now_us);
    // Leg geometry in the physical frame, mm: the label of the leg's end of
    // the rail, the point `d` back off the stop from `at_mm` (a negative `d`
    // reaches past it), and the end of an approach's search (the home one to
    // the frame's edge, MotionArbiter.h homing).
    bool  homeTowardLow(uint8_t leg) const { return (leg == 0) != _home_flip; }
    float homeStopAt(uint8_t leg) const { return homeTowardLow(leg) ? 0.0f : _rail; }
    float homeAway(uint8_t leg, float at_mm, float d) const { return homeTowardLow(leg) ? at_mm + d : at_mm - d; }
    float homePast(uint8_t leg) const {
        return homeAway(leg, homeStopAt(leg), leg == 0 ? -kHomeFrameMarginMm : -kHomeSearchMarginMm);
    }
    // Re-origins the count so the carriage reads `at_mm` and reseeds the
    // engine there: a relabeling, never motion.
    void homeOrigin(int32_t count, float at_mm, uint64_t now_us);
    // Ends the cycle; a failure records `why` and the leg for the census.
    void homeEnd(const char* why);
    bool homePlan(float target_mm, float v_mm_s, uint64_t now_us);

    MotionEmitter& _emitter;
    Clock          _now_us;

    MotionEngine _engine{engineConfig()};

#if NUCLEUS_KINETIC2
    // Kinetic² boundary state, owning task only. The newest knot the engine
    // holds (or the rest point after a reset, or a brake's end): a segment
    // starting after it is a rest until its start, and a jog chains from it.
    uint64_t _k2_newest_us = 0;
    float    _k2_newest_p  = 0.0f;
    // The brake in flight, engine frame, which the engine does not report.
    uint64_t _k2_brake_from_us = 0, _k2_brake_to_us = 0;
    float    _k2_brake_from_p  = 0.0f, _k2_brake_to_p = 0.0f;
    bool     _k2_chase = false;           // the newest knot is a sample's
    bool     _k2_window_clamped = false;  // the last plan's target was window-clamped
    bool     _k2_dirty = false;           // submitted since the last sample: it solves
    uint32_t _k2_plans = 0, _k2_failures = 0;
    // From applyTuning(): the tuning's policy (a Manual move overrides it
    // with Stretch), the curve policy (0 follow, 1 C1, 2 C2) and the samples
    // grant's latency (sampleLatencyUs()).
    kinetic2::Policy _k2_policy = kinetic2::Config{}.policy;
    uint8_t  _k2_curve_policy = 0;
    uint32_t _k2_latency_us   = sampleLatencyUs(motionDefaultTuning());
#endif

    float _win_min = 0.0f;
    float _win_max = DEFAULT_MAX_RAIL_MM;
    float _rail    = DEFAULT_MAX_RAIL_MM;
    // The configured max_rail, which force_home's stroke never shrinks: the
    // home cycle's search distance.
    float _max_rail = DEFAULT_MAX_RAIL_MM;

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
    // Written by pause()/estop() on any task. Atomics, not volatile: the gate
    // store must be visible before the brake request is.
    std::atomic<bool> _paused{false};
    std::atomic<bool> _override{false};
    std::atomic<bool> _flipped{false};
    std::atomic<bool> _return_req{false};
    std::atomic<bool> _brake_req{false};
    // Which generator holds the rail: a MotionSource id, kRailFree, or
    // kRailClosed after an e-stop until a start. ONE word so an e-stop racing
    // a start can never leave the gate open with no holder.
    static constexpr uint8_t kRailFree   = 0xFF;
    static constexpr uint8_t kRailClosed = 0xFE;
    std::atomic<uint8_t> _rail_gen{kRailFree};
    // Owning task only.
    bool     _rail_frame  = false;
    float    _rail_margin = 0.0f;   // mm past each rail end: kHomeFrameMarginMm in a cycle
    bool     _returning   = false;
    // Where the last pause brought the machine to rest (the return target),
    // and the completed returns. Written by the owning task and, for an
    // unpowered arrival, by returnToPause() on the hub task: atomics, so the
    // two never tear a value or lose an increment.
    std::atomic<float>    _pause_pos_mm{0.0f};
    std::atomic<uint32_t> _returns{0};
    // The bench bypass has been logged this boot. Owning task only.
    bool     _bench_noted = false;
    // Set by setWindow()/forceHome() on any task, consumed by evaluate() on
    // the owning task: the mm FRAME moved, the carriage did not.
    volatile bool _frame_moved = false;

    // The home cycle. _homing is set by home() and cleared only by the owning
    // task when the cycle ends; _home_req hands the start across; pause()
    // sets _home_abort. The rest is the owning task's.
    HomeSense* _sense = &noHomeSense();
    std::atomic<bool> _homing{false};
    std::atomic<bool> _home_req{false};
    std::atomic<bool> _home_abort{false};
    HomePhase _home = HomePhase::idle;
    uint8_t   _home_leg = 0;      // 0 the home end, 1 the far end
    bool      _home_flip = false; // the flip as the cycle started
    uint64_t  _home_deadline_us = 0;
    bool      _sense_armed = false;
    int32_t   _home_hit = 0;      // the emitter count where the sense fired
    float     _home_at_mm = 0.0f; // this leg's approach stall, as labeled
    std::array<int32_t, 2> _home_datum{};   // each leg's corrected datum, emitter counts
    float     _home_speed = DEFAULT_HOME_SPEED_MM_S;   // the tuning, applyTuning()
    float     _home_v = 0.0f;     // this cycle's approach speed, mm/s
    float     _touch_v = 0.0f;    // this cycle's re-touch speed, mm/s
    uint32_t  _homes = 0;
    float     _home_rail_mm = 0.0f;
    uint32_t  _home_fails = 0;
    uint8_t   _home_fail_leg = 0;
    const char* _home_fail_why = nullptr;   // a string literal

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
#if !NUCLEUS_KINETIC2
    uint8_t  _plan_flags = 0;   // RFC-100: registry plan_flags, set by plan()
#endif
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
