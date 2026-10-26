#pragma once

// MotionArbiter -- the motion path's policy core: every safety gate, the window
// clamp, limit-set selection, the engine it plans with, the feedforward it
// steers the emitter by, and the census it publishes
// Constraints:
// - HARDWARE-FREE. No IDF, FreeRTOS or board header here or in the .cpp: the
//   board (ValenceMotion.cpp) and the host twin (sim/neutrino) compile this
//   one copy, so a gate change lands once and both run it. The emitter and the
//   clock are handed IN; a host owns the task, the queues and the lock.
// - THE ARBITER IS THE SOLE CALLER of the emitter (architecture.md section 2):
//   steer() from begin(), steerTick() and the seek producer; park() from
//   estop(), a power loss, homeSenseRose() in the home sense's interrupt,
//   planTick() and steerTick(); fence() and renew() from steerTick(), and
//   from planTick() while a home cycle runs. Nothing else commands it. Two
//   producers steer it, mutually exclusive by the cycle's ownership of the
//   rail: the plan-tracking feedforward (steerTick()), and while a home cycle
//   runs the seek producer (homing, below, on the planner), which the engine
//   never sees.
// - TWO OWNING TASKS, on one core, the steer's priority above the planner's,
//   so the planner never runs inside a steerTick() (ValenceMotion.cpp).
//   PLANNER-TASK methods (begin, applyTuning, accept, planTick,
//   drainAnomalies, snapshot, planState, engine) own the engine. The window
//   solve runs lazily in the first sample after a submit, on that task, with
//   a copy of the pending knots and the solver's fixed arrays on its stack:
//   KB-scale temporaries (T1, memory-budget.md T21).
//   STEER-TASK: steerTick() owns the steer, renew() and fence() outside a
//   home cycle, _p_cmd_mm, the residual and the backstop, stall, strip and
//   lapse counters. It never touches the engine and never reads the window:
//   the plan reaches it as the STRIP (PlanStrip), with the frame and the
//   limit set it was filled under, which planTick() publishes and steerTick()
//   copies, each under the host's Lock.
//   evaluate() is planTick() then steerTick() on one task: the host twin's
//   and the native suites' entry.
// - Every conversion between this interface and Kinetic²'s knots lives in
//   MotionArbiter.cpp's Kinetic² boundary section, nowhere else.
// - CROSS-TASK methods (estop, pause, override, returnToPause, acquireRail,
//   releaseRail, setEstopCutsPower, setMotorPowered, setCommissioned, the limit and window setters,
//   setOscillator, forceHome, home, noteStream) never touch the engine.
//   They write flags and scalars the owning tasks read on their next pass,
//   which the planner applies as events of the ownership table (ArbMode);
//   setWindow() and setOscillator() post requests the planner applies
//   (takeWindow(), takeOscillator());
//   estop() and a power loss also park the emitter on the CALLING task,
//   because an e-stop that waits for a tick is not one.
// - homeSenseRose() is the one INTERRUPT-CONTEXT method; its own comment
//   says what it may touch.
// - The object holds the engine (KB-scale). Host it at file scope in
//   INTERNAL RAM: never a stack local, never PSRAM, which is unreachable while
//   the flash cache is off and the sampler must not fault during an OTA write.
// - Millimeters on this interface, normalized 0..1 window units inside the
//   engine. The two never mix in one expression.
// See: ValenceMotion.h, .claude/rules/motion-control.md, bd val-sf7.1

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "ValenceMotion.h"
#include "hub/valence_config.h"
#include "kinetic2/engine.hpp"
#include "kinetic2/oscillator.hpp"
#include "kinetic2/sources.hpp"

namespace valence {

// The planner, Kinetic² (operator ruling 2026-10-06, bd val-z1k): one axis
// here; its timeline holds 64 knots, two full 32-sample bundles.
using MotionEngine = kinetic2::Engine<1, 64>;
using EngineLimits = kinetic2::Limits;
using EngineConfig = kinetic2::Config;
using EngineAnomaly = kinetic2::Anomaly;

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

// The steer word's floor, ~4,790 mm/s, is NOT the LP's real rate: measured
// 2026-10-07 the core sustains about 460 kHz (2,200 mm/s) and falls behind a
// faster plan (val-d66). MAX_SPEED_MM_S (2,000) keeps every plan under that;
// this floor stays a FAULT DETECTOR reading, never a shaper.
inline constexpr uint32_t kMinCyclesPerEdge = 40;

// Intent queue depth, for whichever queue a host puts in front of accept(). A
// bundle carries up to limits::bundle_max_samples (32) samples and the hub
// delegate submits them in one pass, so the queue has to absorb a whole bundle
// plus whatever the previous one left. THE QUEUE IS NOT THE SCHEDULE: a knot
// past the engine's timeline (MotionEngine, 64 knots) is refused by the engine
// and counted as a plan failure, which is the honest place for that ceiling
// to live.
inline constexpr uint32_t kIntentQueueDepth = 40;

// The longest interval a steer's residual and velocity cap are priced over:
// two sampler periods. A steer later than this is a STALL (steerTick()),
// counted and steered without the catch-up the elapsed time would ask for.
// Also the wall bound's horizon on a strip the wall could not keep in its
// frame (PlanStrip::in_frame): no steer carries the carriage past the
// backstop's edge within this long. On a strip it kept, one tick.
inline constexpr float kTickDtCapS = 2.0f * float(kMotionTickUs) * 1e-6f;

// THE LP LEASE (operator ruling 2026-10-06, bd val-fi5): steerTick() renews it
// on every tick (planTick(), after each seek step, while a home cycle runs),
// and the emitter stops itself, the store park() makes, once
// it has not been renewed for this long. An HP stall therefore renders the
// last steer for at most kLeaseUs. Longer than kTickDtCapS, so a tick late
// enough to lapse it is already counted a stall.
inline constexpr uint32_t kLeaseTicks = 4;
inline constexpr uint32_t kLeaseUs    = kLeaseTicks * kMotionTickUs;
// kLeaseUs in LP cycles, which the board writes to the LP core before it runs
// (main.cpp start_lp_core()).
inline constexpr uint32_t kLeaseCycles = kLeaseUs * uint32_t(kLpClockHz / 1.0e6f);
static_assert(float(kLeaseUs) * 1e-6f > kTickDtCapS, "a lapse must be rarer than a stall");

// THE STRIP (bd val-8rt): the plan's positions at kMotionTickUs spacing, this
// many ahead of the planner's last tick, so a window solve up to that long on
// the planner never leaves the steer without a plan. Past its end the steer
// plays the strip's tail, a certified stop from its last state, and counts a
// planner stall (bd val-cy1g).
inline constexpr size_t kStripLen = 128;
// THE HANDOVER (bd val-xfy2): a whole refill keeps the trajectory in flight
// for this many ticks past the clock after its solve, the steer's next tick
// with a tick to spare, and joins the new plan from its state there.
inline constexpr size_t kHandoverTicks = 2;
static_assert(kHandoverTicks >= 2 && kHandoverTicks + 3 < kStripLen,
              "the join reads the trajectory in flight two ticks before it");
// A stop the set's own decel would carry out of the frame brakes harder, up
// to this multiple of its accel and jerk; past that the steer's clamp holds
// (the last line). The tail's stop and the wall's alike.
inline constexpr float kHardStopMax = 16.0f;

// THE OSCILLATOR (RFC-103, bd val-dzf): Kinetic²'s stage, summed into the
// strip, its fade 150 ticks at most. It reads the plan kOscLook ticks past the
// strip's end, and kOscEdge before and after for its differences: the plan
// buffer the strip is cut from is kPlanExt long, from kOscEdge before t0.
using MotionOscillator = kinetic2::Oscillator<kStripLen, 150>;
inline constexpr size_t kOscEdge = MotionOscillator::kPlanEdge;
inline constexpr size_t kOscLook = MotionOscillator::lookahead(kMotionTickUs);
inline constexpr size_t kPlanExt = kOscEdge + kStripLen + kOscLook + kOscEdge;
static_assert(kOscDriveLeadUs == MotionOscillator::driveLeadUs(kMotionTickUs),
              "the osc-drive grant's schedule latency is the oscillator's drive lead");
static_assert(kOscDriveLeadUs / kMotionTickUs < kStripLen + kOscLook, "the plan read must reach the lead");
// While a speed or position drive is bound, a plan drive point every this
// many ticks. The oscillator holds its points from a fade behind its head to
// the lead ahead: these and a 50 Hz axis over that span stay under kDriveMax.
inline constexpr uint32_t kOscPlanDriveTicks = 16;
inline constexpr uint64_t kOscDriveHeldUs = kOscDriveLeadUs + MotionOscillator::kFadeMaxUs;
static_assert(kOscDriveHeldUs / (kOscPlanDriveTicks * kMotionTickUs) + kOscDriveHeldUs / 20000 <
                  MotionOscillator::kDriveMax,
              "plan points and a 50 Hz axis across the held span must fit the oscillator's drive points");

// ---- homing -----------------------------------------------------------------
// Home op 1 is the arbiter's own motion path, never the planner's (operator
// ruling 2026-10-06, bd val-cp5): every leg steers the emitter directly at a
// constant velocity and ends on a step count or a stall. Two legs, the home
// end first, then the far end. Each leg: approach at the home speed (the
// home_speed tuning, held to the jog speed) until the home sense rises, which
// parks the emitter from the interrupt (homeSenseRose()); back off
// kHomeSafetyMarginMm counted on the step count, where the line must read
// LOW; re-approach at homeTouchMmS(), and that stall, moved back by the touch
// speed times kHomeSenseLatencyUs, is the leg's datum. The far datum minus the
// home datum is the stop-to-stop length; minus two kHomeSafetyMarginMm it is
// the usable rail, which becomes the rail and max_rail. Then
// kHomeSafetyMarginMm off the far datum, which is the rail's far end, where the
// line must read LOW again, and homed; the planner is reset there. The home
// approach searches max_rail plus both margins plus two kHomeSearchMarginMm,
// so a retry reaches the home stop from where a failed far approach stopped;
// the far approach searches max_rail plus both margins plus one from the home
// datum; a re-touch reaches kHomeSearchMarginMm past the stall it backed off
// from. No stall within either, a usable rail under MIN_RAIL_MM, or no end by
// homeCycleS() plus kHomeTimeoutMarginUs fails it unhomed.
//
// A CYCLE PUBLISHES THE PHYSICAL FRAME: 0.0 mm is kHomeSafetyMarginMm off the
// low stop's datum, the rail ends kHomeSafetyMarginMm short of the high one,
// and the stall the home leg takes reads its datum's label (minus the margin,
// or the rail plus it under the flip) from the first approach on. Every
// position published during a cycle, and after one that fails, is measured
// from the home datum; the far leg relabels nothing (bd val-tib).
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
// planner task (ValenceHomeSense.cpp), and nothing else. kHomeSenseLatencyUs is
// the two ends' shared contact-to-park figure; a change to the source's
// numbers moves it here.
inline constexpr uint32_t kStallConversionUs    = 150;    // the S3's INA228 shunt conversion
inline constexpr uint32_t kStallOnSamples       = 3;      // the S3's N
inline constexpr uint32_t kStallDetectUs        = kStallOnSamples * kStallConversionUs;
// A SAFETY MARGIN, NEVER A COMMANDABLE POSITION AND NEVER A CONVENIENCE
// (operator ruling 2026-10-06): the rail ends this far short of each stop's
// datum, the window clamp never admits a point inside it, and every backoff of
// the cycle is this distance.
inline constexpr float    kHomeSafetyMarginMm   = 5.0f;
inline constexpr float    kHomeSearchMarginMm   = 10.0f;
inline constexpr float    kHomeTouchDivisor     = 4.0f;
inline constexpr float    kHomeTouchFloorMmS    = 8.0f;
inline constexpr uint32_t kHomeSenseDebounceUs  = 100;    // the board's two reads of a rise
inline constexpr uint32_t kHomeTimeoutMarginUs  = 2000000;
inline constexpr uint32_t kHomeSenseLatencyUs   = kStallDetectUs + kHomeSenseDebounceUs;
// A linear velocity ramp over this many ms at the start of every leg, from
// rest to the leg's speed; 0 = none, the first steer is the full speed. For a
// drive that faults on a step-rate step: a build constant, never a wire field.
inline constexpr uint32_t kHomeRampMs           = 0;
inline constexpr uint32_t kHomeLegs             = 7;   // two approaches, two re-touches, three backoffs

// The re-touch speed for an approach speed: a quarter of it, floored, never
// above the approach itself.
constexpr float homeTouchMmS(float home_mm_s) {
    const float quarter = home_mm_s / kHomeTouchDivisor;
    const float floored = quarter > kHomeTouchFloorMmS ? quarter : kHomeTouchFloorMmS;
    return floored < home_mm_s ? floored : home_mm_s;
}

// The cycle's own time, before kHomeTimeoutMarginUs, every leg at its constant
// speed: both approaches across their whole search and the three backoffs at
// the home speed, both re-touches across their whole reach at the touch speed,
// and half of kHomeRampMs for each leg. The 1 ms dwell between legs is inside
// kHomeTimeoutMarginUs.
constexpr float homeCycleS(float max_rail_mm, float home_mm_s) {
    const float stops = max_rail_mm + 2.0f * kHomeSafetyMarginMm;
    const float seek = (stops + 2.0f * kHomeSearchMarginMm) + (stops + kHomeSearchMarginMm);
    const float slow = 2.0f * (kHomeSafetyMarginMm + kHomeSearchMarginMm);
    const float backoff = 3.0f * kHomeSafetyMarginMm;
    return (seek + backoff) / home_mm_s + slow / homeTouchMmS(home_mm_s) +
           float(kHomeLegs) * 0.5e-3f * float(kHomeRampMs);
}

// At the factory speeds. A faster home speed is checked where it runs: the
// line must read LOW after each backoff, or the cycle fails. Each stall's
// overrun past contact, its speed times the contact-to-park latency, stays
// inside half the safety margin the backoff then retreats by, so the line has
// the other half to fall in and the carriage never ends a leg nearer the stop
// than half a margin.
static_assert(DEFAULT_HOME_SPEED_MM_S * float(kHomeSenseLatencyUs) * 1e-6f < 0.5f * kHomeSafetyMarginMm,
              "the approach's overrun past contact must stay inside half the safety margin");
static_assert(homeTouchMmS(DEFAULT_HOME_SPEED_MM_S) * float(kHomeSenseLatencyUs) * 1e-6f < 0.5f * kHomeSafetyMarginMm,
              "the re-touch's overrun past contact must stay inside half the safety margin");
static_assert(kHomeSearchMarginMm > kHomeSafetyMarginMm,
              "a re-touch must reach past the stall it backed off from");

// ---- the ownership table ----------------------------------------------------
// Who may feed the planner: ONE mode, written by the planner task alone, moved
// only by the rules below, (mode, event) -> (next, action) (bd val-5ozv.1).
// The cross-task methods post the SPEC 11 latches and their requests as words,
// exactly as before; the planner turns each into an event at the point of
// planStep() or accept() that applies it, so the table moves no work in time.
// - A cell no rule names is ILLEGAL: apply() holds the mode, logs it, counts it
//   (census arb_illegal), and a native build (MOTION_ARB_STRICT) aborts. The
//   board only reports: a detector never adjudicates (memory-budget.md).
// - Every change of mode logs one line (from, event, to, cause) and counts
//   (census arb_transitions); past kArbLogBurst lines in kArbLogWindowUs the
//   lines are counted, and the next one logged says how many.
// - The generator rail word (_rail_gen) is not a mode: it stays one word
//   compare-and-swapped on the calling task, so an e-stop racing a start never
//   leaves the gate open with no holder. Run feeds a generator while it admits.
// - The handover, the connector, the wall and a refused plan's hold are the
//   ONE trajectory, not modes: accept() refuses during the hold in any mode.
enum class ArbMode : uint8_t {
    Run,           // no latch: Stream and Manual feed; a generator as the rail word admits
    Paused,        // PAUSE: nothing feeds
    Jog,           // PAUSE and override: the operator's Manual alone, in the rail frame
    Returning,     // PAUSE and override, the RETURN plan in flight: nothing feeds
    HomeQueued,    // home() posted, the cycle not started: nothing feeds
    HomeApproach,  // the seek producer owns the emitter (homing, above), one mode per leg kind
    HomeClear,
    HomeTouch,
    HomeFinish,
    Unpowered,     // the power gate shut: nothing renders. Boots here
    Estop,         // ESTOP latched
};
inline constexpr size_t kArbModes = size_t(ArbMode::Estop) + 1;

enum class ArbEvent : uint8_t {
    EstopOn,        // the ESTOP latch seen set (planStep())
    EstopOff,       // seen released; the release latched PAUSE first
    PowerOff,       // the power gate seen shut (planStep())
    PowerOn,        // seen open
    Pause,          // the PAUSE latch seen set
    Resume,         // seen clear
    Override,       // the override latch seen set
    OverrideOff,    // seen clear under PAUSE: an unpowered RETURN arrived
    Brake,          // the pause brake request taken
    HomeRequest,    // home() posted a request
    HomeCancel,     // home() withdrew it: the sense refused
    HomeAbort,      // pause() ended the request or the cycle
    HomeStart,      // the request taken at rest: the home approach
    HomeMoving,     // the request taken in motion: refused
    HomeContact,    // an approach's stall confirmed: the backoff
    HomeCleared,    // a backoff on its count, the line LOW: the re-touch
    HomeDatum,      // the home end's re-touch stall: the far approach
    HomeMeasured,   // the far end's re-touch stall: the final backoff
    HomeDone,       // the final backoff on its count, the line LOW: homed
    HomeFail,       // a leg ended without its stall, a stop still pressed, a short rail, a timeout
    WindowEdit,     // a session's live window edit taken (setWindow(), WindowPost::edit)
    WindowAdopt,    // the stored window adopted: boot, homing, a config restore (WindowPost::adopt)
    Reorigin,      // force_home re-origined the count (forceHome())
    Return,         // a RETURN request taken, its plan accepted
    ReturnFailed,   // taken, its plan refused
    ReturnArrived,  // the RETURN plan at rest
};
inline constexpr size_t kArbEvents = size_t(ArbEvent::ReturnArrived) + 1;

// What the code at the event's point does. The table names it; that code is
// the action, so a cell and its call site never disagree on what runs.
enum class ArbAct : uint8_t {
    illegal,     // no rule: held, logged, counted
    none,        // the mode alone moves
    settle,      // the engine reset at the carriage once per latch or loss; a cycle ends first
    brake,       // brakeToRest(): to rest at the input decel, the paused position recorded
    record,      // the paused position recorded at the parked carriage, nothing braked
    queue,       // every intent refused until the request ends
    refuse,      // the request dropped, nothing steered
    start,       // re-origined at the far datum's label, the approach steered
    leg,         // the next leg steered
    endCycle,    // parked, the engine reset at the count, the request dropped
    homed,       // the rail measured, homed, the engine reset at the count
    reframe,     // the engine reset or reseeded in the new frame; a jog re-clamped, a carriage left outside glides
    recheck,     // reframe, and the paused position held in the new window; a RETURN re-planned to it
    defer,       // the edit held, applied in the mode the cycle or the latch ends in
    reorigin,    // the engine reset at the carriage in the new origin; nothing glides
    planReturn,  // a jog-set plan to the paused position
    arrive,      // override dropped, the return counted
    adopt,       // recheck, but nothing starts a glide: a carriage outside waits for the next command
};

// layer: the next mode is the one the SPEC 11.1 latches name (layer() in the
// arbiter: Jog under override, else Paused under PAUSE, else Run).
struct ArbCell {
    ArbMode next  = ArbMode::Run;
    ArbAct  act   = ArbAct::illegal;
    bool    layer = false;
};
struct ArbRule {
    ArbMode  from;
    ArbEvent ev;
    ArbCell  cell;
};

namespace arb {
using M = ArbMode;
using E = ArbEvent;
using A = ArbAct;
constexpr ArbCell to(M next, A act) { return ArbCell{next, act, false}; }
constexpr ArbCell toLayer(A act) { return ArbCell{M::Run, act, true}; }

// THE RULES. Every legal cell, one line each; whatever is not here is illegal.
// "race" marks a cell only a post landing between two of the planner's reads
// of one tick reaches; it does what the code did before the table.
inline constexpr ArbRule kRules[] = {
    // The ESTOP latch: from everywhere. From Estop itself, a home request
    // that raced the latch is refused.
    {M::Run, E::EstopOn, to(M::Estop, A::settle)},
    {M::Paused, E::EstopOn, to(M::Estop, A::settle)},
    {M::Jog, E::EstopOn, to(M::Estop, A::settle)},
    {M::Returning, E::EstopOn, to(M::Estop, A::settle)},
    {M::HomeQueued, E::EstopOn, to(M::Estop, A::settle)},
    {M::HomeApproach, E::EstopOn, to(M::Estop, A::settle)},
    {M::HomeClear, E::EstopOn, to(M::Estop, A::settle)},
    {M::HomeTouch, E::EstopOn, to(M::Estop, A::settle)},
    {M::HomeFinish, E::EstopOn, to(M::Estop, A::settle)},
    {M::Unpowered, E::EstopOn, to(M::Estop, A::settle)},
    {M::Estop, E::EstopOn, to(M::Estop, A::refuse)},
    {M::Estop, E::EstopOff, to(M::Paused, A::none)},
    // The power gate: from everywhere but Estop, whose branch runs first.
    // From Unpowered itself, a home request that raced the loss is refused.
    {M::Run, E::PowerOff, to(M::Unpowered, A::settle)},
    {M::Paused, E::PowerOff, to(M::Unpowered, A::settle)},
    {M::Jog, E::PowerOff, to(M::Unpowered, A::settle)},
    {M::Returning, E::PowerOff, to(M::Unpowered, A::settle)},
    {M::HomeQueued, E::PowerOff, to(M::Unpowered, A::settle)},
    {M::HomeApproach, E::PowerOff, to(M::Unpowered, A::settle)},
    {M::HomeClear, E::PowerOff, to(M::Unpowered, A::settle)},
    {M::HomeTouch, E::PowerOff, to(M::Unpowered, A::settle)},
    {M::HomeFinish, E::PowerOff, to(M::Unpowered, A::settle)},
    {M::Unpowered, E::PowerOff, to(M::Unpowered, A::refuse)},
    {M::Unpowered, E::PowerOn, toLayer(A::none)},
    // The SPEC 11.1 layer. A cycle owns the rail until it ends and a RETURN
    // until it arrives, so neither reads the layer meanwhile; pause() ends a
    // cycle by HomeAbort. Resume is refused by the hub under override.
    {M::Run, E::Pause, to(M::Paused, A::none)},
    {M::Run, E::Override, to(M::Jog, A::none)},
    {M::Paused, E::Override, to(M::Jog, A::none)},
    {M::Paused, E::Resume, to(M::Run, A::none)},
    {M::Jog, E::OverrideOff, to(M::Paused, A::none)},
    // The pause brake. Only pause() and forceHome() under PAUSE post it, so in
    // Run it is a PAUSE the layer has not read yet (race).
    {M::Run, E::Brake, to(M::Paused, A::brake)},
    {M::Paused, E::Brake, to(M::Paused, A::brake)},
    {M::Jog, E::Brake, to(M::Jog, A::brake)},
    {M::Returning, E::Brake, to(M::Returning, A::brake)},
    {M::HomeQueued, E::Brake, to(M::HomeQueued, A::brake)},     // race: before its HomeAbort
    {M::HomeApproach, E::Brake, to(M::HomeApproach, A::brake)}, // race: before its HomeAbort
    {M::HomeClear, E::Brake, to(M::HomeClear, A::brake)},       // race
    {M::HomeTouch, E::Brake, to(M::HomeTouch, A::brake)},       // race
    {M::HomeFinish, E::Brake, to(M::HomeFinish, A::brake)},     // race
    {M::Unpowered, E::Brake, to(M::Unpowered, A::record)},
    // Home op 1. The request queues from the layer; a RETURN in flight reads
    // it at its arrival.
    {M::Run, E::HomeRequest, to(M::HomeQueued, A::queue)},
    {M::Paused, E::HomeRequest, to(M::HomeQueued, A::queue)},
    {M::Jog, E::HomeRequest, to(M::HomeQueued, A::queue)},   // race: an override after the request
    {M::HomeQueued, E::HomeCancel, toLayer(A::none)},
    {M::HomeQueued, E::HomeAbort, toLayer(A::refuse)},
    {M::HomeApproach, E::HomeAbort, toLayer(A::endCycle)},
    {M::HomeClear, E::HomeAbort, toLayer(A::endCycle)},
    {M::HomeTouch, E::HomeAbort, toLayer(A::endCycle)},
    {M::HomeFinish, E::HomeAbort, toLayer(A::endCycle)},
    {M::Returning, E::HomeAbort, to(M::Returning, A::refuse)},   // race: after HomeQueued's Return
    {M::HomeQueued, E::HomeStart, to(M::HomeApproach, A::start)},
    {M::HomeQueued, E::HomeMoving, toLayer(A::refuse)},
    {M::HomeApproach, E::HomeContact, to(M::HomeClear, A::leg)},
    {M::HomeClear, E::HomeCleared, to(M::HomeTouch, A::leg)},
    {M::HomeTouch, E::HomeDatum, to(M::HomeApproach, A::leg)},
    {M::HomeTouch, E::HomeMeasured, to(M::HomeFinish, A::leg)},
    {M::HomeFinish, E::HomeDone, toLayer(A::homed)},
    {M::HomeApproach, E::HomeFail, toLayer(A::endCycle)},
    {M::HomeClear, E::HomeFail, toLayer(A::endCycle)},
    {M::HomeTouch, E::HomeFail, toLayer(A::endCycle)},
    {M::HomeFinish, E::HomeFail, toLayer(A::endCycle)},
    // A window edit (bd val-5ozv.2): the motion layer's, a carriage the edit
    // left outside gliding in (THE GLIDE, below). Under PAUSE the paused
    // position is held in the new window; under override the jog keeps the
    // rail (SPEC 11.1) and a RETURN lands in the new window. A cycle, an
    // ESTOP and a power loss hold it, and it applies in the mode they end in.
    {M::Run, E::WindowEdit, to(M::Run, A::reframe)},
    {M::Paused, E::WindowEdit, to(M::Paused, A::recheck)},
    {M::Jog, E::WindowEdit, to(M::Jog, A::recheck)},
    {M::Returning, E::WindowEdit, to(M::Returning, A::recheck)},
    {M::HomeQueued, E::WindowEdit, to(M::HomeQueued, A::defer)},
    {M::HomeApproach, E::WindowEdit, to(M::HomeApproach, A::defer)},
    {M::HomeClear, E::WindowEdit, to(M::HomeClear, A::defer)},
    {M::HomeTouch, E::WindowEdit, to(M::HomeTouch, A::defer)},
    {M::HomeFinish, E::WindowEdit, to(M::HomeFinish, A::defer)},
    {M::Unpowered, E::WindowEdit, to(M::Unpowered, A::defer)},
    {M::Estop, E::WindowEdit, to(M::Estop, A::defer)},
    // The stored window adopted (operator ruling 2026-10-25: motion is only
    // ever commanded): at boot, after homing, after a config restore. Never a
    // glide; a carriage it leaves outside waits there, and the next commanded
    // motion brings it in through the handover. A glide already in flight
    // re-aims into the adopted window. Held as an edit is.
    {M::Run, E::WindowAdopt, to(M::Run, A::adopt)},
    {M::Paused, E::WindowAdopt, to(M::Paused, A::adopt)},
    {M::Jog, E::WindowAdopt, to(M::Jog, A::adopt)},
    {M::Returning, E::WindowAdopt, to(M::Returning, A::adopt)},
    {M::HomeQueued, E::WindowAdopt, to(M::HomeQueued, A::defer)},
    {M::HomeApproach, E::WindowAdopt, to(M::HomeApproach, A::defer)},
    {M::HomeClear, E::WindowAdopt, to(M::HomeClear, A::defer)},
    {M::HomeTouch, E::WindowAdopt, to(M::HomeTouch, A::defer)},
    {M::HomeFinish, E::WindowAdopt, to(M::HomeFinish, A::defer)},
    {M::Unpowered, E::WindowAdopt, to(M::Unpowered, A::defer)},
    {M::Estop, E::WindowAdopt, to(M::Estop, A::defer)},
    // force_home: a reset at the carriage outside a cycle, the end of one
    // inside (the hub refuses it during one, SPEC 11.1: race); held through
    // an ESTOP or a power loss.
    {M::Run, E::Reorigin, to(M::Run, A::reorigin)},
    {M::Paused, E::Reorigin, to(M::Paused, A::reorigin)},
    {M::Jog, E::Reorigin, to(M::Jog, A::reorigin)},
    {M::Returning, E::Reorigin, to(M::Returning, A::reorigin)},
    {M::HomeQueued, E::Reorigin, toLayer(A::refuse)},
    {M::HomeApproach, E::Reorigin, toLayer(A::endCycle)},
    {M::HomeClear, E::Reorigin, toLayer(A::endCycle)},
    {M::HomeTouch, E::Reorigin, toLayer(A::endCycle)},
    {M::HomeFinish, E::Reorigin, toLayer(A::endCycle)},
    {M::Unpowered, E::Reorigin, to(M::Unpowered, A::defer)},
    {M::Estop, E::Reorigin, to(M::Estop, A::defer)},
    // SPEC 11.1 RETURN. A second request re-plans; a refused plan hands the
    // rail back to the jog.
    {M::Jog, E::Return, to(M::Returning, A::planReturn)},
    {M::Returning, E::Return, to(M::Returning, A::planReturn)},
    {M::HomeQueued, E::Return, to(M::Returning, A::planReturn)},   // race: override and return mid-tick
    {M::Jog, E::ReturnFailed, to(M::Jog, A::planReturn)},
    {M::Returning, E::ReturnFailed, to(M::Jog, A::planReturn)},
    {M::HomeQueued, E::ReturnFailed, to(M::HomeQueued, A::planReturn)},   // race
    {M::Returning, E::ReturnArrived, to(M::Paused, A::arrive)},
};

constexpr bool rulesUnique() {
    for (size_t i = 0; i < std::size(kRules); ++i)
        for (size_t j = i + 1; j < std::size(kRules); ++j)
            if (kRules[i].from == kRules[j].from && kRules[i].ev == kRules[j].ev) return false;
    return true;
}
static_assert(rulesUnique(), "one rule per cell");

using Table = std::array<std::array<ArbCell, kArbEvents>, kArbModes>;
inline constexpr Table kTable = [] {
    Table t{};
    for (const ArbRule& r : kRules) t[size_t(r.from)][size_t(r.ev)] = r.cell;
    return t;
}();
}  // namespace arb

constexpr ArbCell arbCell(ArbMode m, ArbEvent e) { return arb::kTable[size_t(m)][size_t(e)]; }
constexpr bool arbSeeking(ArbMode m) { return m >= ArbMode::HomeApproach && m <= ArbMode::HomeFinish; }

// Who may feed the planner in each mode, by MotionSource. A feed still meets
// the gates that are not modes: a refused plan's hold, commissioned, homed and
// the rail word (accept()).
enum class ArbFeed : uint8_t { feed, jog, estop, unpowered, homing, paused, returning };
inline constexpr std::array<std::array<ArbFeed, 4>, kArbModes> kArbFeed = [] {
    using F = ArbFeed;
    std::array<std::array<ArbFeed, 4>, kArbModes> t{};
    //                                       Manual        Stream     Pattern    Advanced
    t[size_t(ArbMode::Run)]          = {{F::feed,      F::feed,   F::feed,   F::feed}};
    t[size_t(ArbMode::Paused)]       = {{F::paused,    F::paused, F::paused, F::paused}};
    t[size_t(ArbMode::Jog)]          = {{F::jog,       F::paused, F::paused, F::paused}};
    t[size_t(ArbMode::Returning)]    = {{F::returning, F::paused, F::paused, F::paused}};
    for (ArbMode m : {ArbMode::HomeQueued, ArbMode::HomeApproach, ArbMode::HomeClear, ArbMode::HomeTouch,
                      ArbMode::HomeFinish})
        t[size_t(m)] = {{F::homing, F::homing, F::homing, F::homing}};
    t[size_t(ArbMode::Unpowered)]    = {{F::unpowered, F::unpowered, F::unpowered, F::unpowered}};
    t[size_t(ArbMode::Estop)]        = {{F::estop,     F::estop,  F::estop,  F::estop}};
    return t;
}();
static_assert(kMotionSourceNames.size() == 4, "the feed table has one column per MotionSource");

// When a flash write may run in each mode (bd val-y3da, operator 2026-10-25:
// settings change during motion and never stop it). On a build that runs
// code from flash, a write turns the flash cache off and no task on either
// core runs until it ends (~18 ms measured on the 0.1.68 bench): the steer
// cannot renew the lease, and the LP core parks the carriage at whatever
// speed it had. So a write waits for a mode where that stops nothing; the
// census carries the rule (MotionCensus::flash) and motionFlashOk() adds the
// "nothing moves" half. A cycle and a RETURN own the rail until they end:
// hold. Estop and Unpowered render nothing: open.
inline constexpr std::array<FlashRule, kArbModes> kArbFlash = [] {
    std::array<FlashRule, kArbModes> t{};
    t.fill(FlashRule::hold);
    t[size_t(ArbMode::Run)]       = FlashRule::rest;
    t[size_t(ArbMode::Paused)]    = FlashRule::rest;
    t[size_t(ArbMode::Jog)]       = FlashRule::rest;
    t[size_t(ArbMode::Unpowered)] = FlashRule::open;
    t[size_t(ArbMode::Estop)]     = FlashRule::open;
    return t;
}();

inline constexpr std::array<const char*, kArbModes> kArbModeNames{
    "Run", "Paused", "Jog", "Returning", "HomeQueued", "HomeApproach", "HomeClear", "HomeTouch", "HomeFinish",
    "Unpowered", "Estop"};
inline constexpr std::array<const char*, kArbEvents> kArbEventNames{
    "estop", "release", "power off", "power on", "pause", "resume", "override", "override off", "brake",
    "home request", "home cancel", "home abort", "home start", "home moving", "home contact", "home cleared",
    "home datum", "home measured", "home done", "home fail", "window edit", "window adopt", "reorigin", "return",
    "return failed", "return arrived"};

// The transition log's burst: this many lines per window, the rest counted.
inline constexpr uint32_t kArbLogBurst = 8;
inline constexpr uint64_t kArbLogWindowUs = 1000000;

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
    // The steer task, or the planner while a home cycle runs; never both.
    virtual void steer(float v_mm_s) = 0;
    // Any task: stop rendering now. One word store, no lock, no engine.
    virtual void park() = 0;
    // As steer(). THE FENCE: no edge takes count() below lo or above hi;
    // an edge past it is withheld and counted, the period unchanged. Written
    // before any steer into a new frame.
    virtual void fence(int32_t lo, int32_t hi) = 0;
    // As steer(), every tick. THE LEASE: unrenewed for kLeaseUs, the
    // emitter parks itself and counts a lapse; a renewal lets it render the
    // next steer.
    virtual void renew() = 0;
    // Lapses since the emitter started. Any task.
    virtual uint32_t lapses() const = 0;

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
    // Planner task, during a cycle: HIGH now, a fresh rise confirmed (the
    // board reads it twice, kHomeSenseDebounceUs apart).
    virtual bool high() = 0;

protected:
    ~HomeSense() = default;
};

// The sense of a build without a line: absent, so home() refuses no_sense
// rather than seeking open-loop into the stop.
HomeSense& noHomeSense();

// ---- the arbiter ------------------------------------------------------------

// The reaction horizon the hub chooses per submit (operator ruling 2026-10-20,
// bd val-668m): as long as the planner needs to cover its own solve
// (2 x the smoothed solve cost + 4 ms), never under the react_ms setting (the
// floor), never over half the stream's measured lead (a short-lead client keeps
// its through points moving: at a 100 ms lead, 32 ms rested every knot), never
// over kReactCapUs (imperceptible on a click, inside the 128 ms strip). A solve
// past lead / 2 lands late and is counted (LATE PLAN), as before.
inline constexpr uint32_t kReactCapUs = 64000;
constexpr uint32_t adaptiveReactUs(float plan_us_avg, float lead_us_avg, uint32_t floor_us) {
    float want = 2.0f * plan_us_avg + 4000.0f;
    const float half_lead = 0.5f * lead_us_avg;
    const float cap = half_lead < float(kReactCapUs) ? half_lead : float(kReactCapUs);
    if (want > cap) want = cap;
    const float fl = float(floor_us);
    return uint32_t(want < fl ? fl : want);
}

// The segment seam (bd val-7t48): a segment starting at most kSegmentSeamUs
// past the newest knot is that knot's scheduled successor (SPEC 9.6), never a
// rest, unless that knot is itself at rest. Its knot keeps the author's end, so
// the gap joins its span: slower by the gap, never faster, no knot moved.
// Past the seam, or after a knot at rest, the gap is a rest: the curve holds at
// the newest knot until the start. Constraint: 16 ms covers the stamp error of
// a sender's hub clock (Phosphor's door is up to 8 ms late on one CLOCK
// exchange and steps when it adopts another, either way), so do not narrow it
// toward the tick: a v = 0 hold 1 to 4 ms after a knot passed at 250 mm/s made
// both spans rest-to-rest at 1.5x their chord, 350 to 550 mm/s and a piece over
// the ceiling at every hold. A sender that means a shorter rest sends it as a
// hold segment (SPEC 9.6). Story: motion-control.md, the Kinetic boundary.
inline constexpr uint64_t kSegmentSeamUs = 16000;
constexpr bool segmentContinues(uint64_t start_us, uint64_t newest_us, bool newest_rests) {
    return !newest_rests && start_us > newest_us && start_us - newest_us <= kSegmentSeamUs;
}

class MotionArbiter {
public:
    // The engine's clock, microseconds. Read only to time the window solve.
    using Clock = uint64_t (*)();
    // The host's lock around the strip: hold(true) before, hold(false) after
    // a publish (planner) or a copy (steer), on the calling task. Held for one
    // PlanStrip copy; never blocks, logs or allocates inside. Null when one
    // task runs both halves (evaluate()).
    using Lock = void (*)(bool hold);

    MotionArbiter(MotionEmitter& emitter, Clock now_us, Lock lock = nullptr)
        : _emitter(emitter), _now_us(now_us), _lock(lock) {}

    // Planner task.
    // Before either task exists, or on the planner. Parks and zeroes the origin.
    void begin(uint64_t now_us);
    void applyTuning(const MotionTuning& t);
    bool accept(const MotionIntent& in, uint64_t now_us); // gates, clamp, commit
    // The window setWindow() posted, then the gates, the brake and return
    // requests, the home cycle's seek producer, the one side-effecting engine
    // sample, then the strip's publish. dt_s: the interval since the previous
    // planTick().
    void planTick(uint64_t now_us, float dt_s);
    // Steer task. The strip at now_us (its tail past its end), clamped to
    // the strip's backstop frame, as the feedforward and the residual; the
    // fence, the steer and the lease. dt_s: the interval since the previous steerTick(); past
    // kTickDtCapS it is a stall. Steers nothing while a home cycle runs.
    void steerTick(uint64_t now_us, float dt_s);
    // One task, both halves in order: the host twin's and the tests' tick.
    void evaluate(uint64_t now_us, float dt_s) {
        planTick(now_us, dt_s);
        steerTick(now_us, dt_s);
    }
    // The census's counters (snapshot()), read alone by host tests: runs past
    // the strip's end (the steer's, counted at onset), plans joined through a
    // connector, plans refused for a stop, and stops braked harder than the
    // set to stay in the frame (the planner's).
    uint32_t plannerStalls() const { return _planner_stalls; }
    uint32_t latePlans() const { return _late_plans; }
    uint32_t plansRefused() const { return _plans_refused; }
    uint32_t hardStops() const { return _hard_stops; }
    uint32_t glides() const { return _glides; }
    // The engine's oldest anomaly, counted for the census as it leaves; false
    // when none is left.
    bool popAnomaly(MotionAnomaly& out);
    // Every anomaly, counted and dropped. Returns the kinds, bit k =
    // kinetic2::AnomalyKind k.
    uint32_t drainAnomalies();
    // Every census field the arbiter owns. The emitter's counters (edges,
    // late, resteers, catchups, step_q8, emitter_faults) and stack_free are
    // the host's to fill: they are facts about its hardware and its task.
    // Reads the steer task's counters and _backstop_on as single words, and
    // the plan as the last planTick() sampled it: never the engine.
    MotionCensus snapshot(uint64_t now_us);

    // Any task.
    // SPEC 11.2. on: parks the emitter on the CALLING task, takes the rail
    // from both generators, and drops homed when the hub declares
    // estop_cuts_power. off is the RELEASE and lands in PAUSE, never in motion.
    void estop(bool on);
    // SPEC 11.1 PAUSE. on: latches, THEN asks the planner to brake the
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
    // SPEC 11.1 RETURN: the planner plans a jog-set move back to where the
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
    // releaseRail(Stream) is the hub's quiet release of the stream (SPEC
    // 11.4, RFC-098): the planner drops the stream's expectation and, with
    // knots still pending, re-solves them so the newest lands at rest.
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
    // ONE WRITER, the hub task (bd val-8es): it posts the window and raises
    // the frame-move flag; the planner applies it at its next accept() or
    // planTick(), so winMin(), winMax() and rail() read it from then on. Only
    // WindowPost::edit may glide (ArbEvent::WindowAdopt): the default never
    // moves the carriage.
    void setWindow(float lo, float hi, float rail, WindowPost post = WindowPost::adopt);
    // RFC-103 (SPEC 9.7): the oscillator's parameters, already clamped by the
    // delegate. ONE WRITER, the hub task: a post the planner takes at its next
    // tick (takeOscillator()). The oscillator rides whatever owns the rail and
    // the rest between, yields first under the input set's ceilings, never
    // leaves the window, and renders nothing under ESTOP, PAUSE (cut at once,
    // the phase back at the trough), unhomed or uncommissioned.
    void setOscillator(const MotionOsc& o);
    // SPEC 9.7: one osc-drive stream sample, each field 0 .. 1, stamped at_us
    // on this clock. ONE WRITER, the hub task: a single-producer ring the
    // planner drains every tick (feedOscillator()); a full ring drops the
    // sample and counts it (oscDriveDropped()). While an axis drive is bound
    // a live stream renders the oscillator whatever osc.enabled holds (bd
    // val-o9r (b), ahead of the pinned SPEC); PAUSE, ESTOP and the gates of
    // setOscillator() hold it as they hold the hand.
    void postOscDrive(float amplitude, float frequency, uint64_t at_us);
    // Planner task. Drive points the ring or the oscillator refused (its
    // kDriveMax points held): counted, never waited on.
    uint32_t oscDriveDropped() const { return _oscd_dropped + _oscd_ring_lost.load(); }
    float forceHome(float stroke_mm);
    // Before either owning task runs. Until then, and on a build without one,
    // the sense is absent and home() answers no_sense.
    void setHomeSense(HomeSense& sense) { _sense = &sense; }
    // Home op 1 (ValenceMotion.h motionHome()): refused here on what the
    // calling task can see, the sense probed last; started queues the cycle
    // for the planner. One cycle at a time: a request while one runs
    // answers started and changes nothing.
    HomeStart home();
    // INTERRUPT CONTEXT: the home sense's rising edge, before the planner is
    // woken. While a seek leg is armed it parks the emitter and latches the
    // step count for the planner's next tick; otherwise it does nothing.
    // It touches the seek word, the latched count, and the emitter's park()
    // and count() (on the board one 32-bit store and one 32-bit load in the LP
    // core's shared memory), and nothing else: no log, no allocation, no
    // engine, no float, no FreeRTOS call. Any core; it never blocks.
    void homeSenseRose();
    void noteStream(uint32_t bundles, uint32_t samples, uint32_t dropped);

    float positionMm() const { return float(_emitter.count() - _origin) * kMmPerStep; }
    // Planner task. The planned state at now_us, engine frame, for host tooling
    // (tools/kinetic-wasm): the same sample planTick() takes, so reading it at
    // that time changes nothing. Nothing on the board calls it.
    kinetic2::State planState(uint64_t now_us) { return sampleEngine(now_us); }
    // Planner task. The engine itself, for host tooling only (tools/kinetic-wasm
    // reads its pending knots as solved). Reading solved() solves a dirty
    // window: read it after evaluate(), never between accept() and evaluate().
    // Nothing on the board calls it.
    MotionEngine& engine() { return _engine; }
    // Planner task. The expectation the engine holds (Engine::expect), engine
    // clock; 0 when none. For host tests; nothing on the board calls it.
    uint64_t expectUntil() const { return _k2_expect_us; }
    // Any task: the mode the planner applied last (ArbMode).
    ArbMode arbMode() const { return _mode.load(); }
    float winMin() const { return _win_min; }
    float winMax() const { return _win_max; }
    float rail() const { return _rail; }

private:
    // The engine's construction config: the kernel's defaults.
    static EngineConfig engineConfig();

    // The active plan as the census reads it, in the engine frame. mode and
    // plan_kind are the 0x1111 selects' option ordinals (MotionArbiter.cpp
    // PlanStyle, kPlanKindBezier).
    struct PlanRead {
        float    pos = 0.0f, vel = 0.0f, start = 0.0f, target = 0.0f;
        float    duration_s = 0.0f, elapsed_s = 0.0f;
        uint8_t  mode = 0, plan_kind = 0, flags = 0;
    };
    // From the state sampleEngine() just returned; samples and solves nothing.
    PlanRead readPlan(const kinetic2::State& st, uint64_t now_us);
    // The plan as the planner's last sample read it (planStep(), a reset):
    // the census reads this and never the engine, so a read never changes
    // the plan (val-hlj).
    PlanRead _plan_read{};
    bool     _plan_busy = false;
    // The segment the last read reported (engine frame, engine clock) and
    // Engine::segStart's time at that read. A commit through a knot moves
    // segStart to the knot while the committed curve still renders toward
    // it; until the knot's time the read keeps the piece that began before
    // it (val-0ep).
    struct SegRead {
        float    from_p = 0.0f, to_p = 0.0f;
        uint64_t from_us = 0, to_us = 0;
        uint8_t  flags = 0;
    };
    SegRead  _seg_read{};
    uint64_t _seg_eng_us = 0;
    bool     _seg_through = false;

    // THE STRIP: the commanded position at t0_us + i * kMotionTickUs, mm,
    // for i in [0, n): the plan, the oscillator and the connector summed. n
    // is 0 (steer nothing) or kStripLen. gen is the engine reset it follows:
    // the steer re-anchors its feedforward at anchor_mm, the carriage's
    // position at that reset, once per gen. Consecutive strips of one gen are
    // one continuous trajectory (handover()). replan is the steer's lapse
    // request it answers (_replan_req).
    struct PlanStrip {
        uint64_t t0_us = 0;
        uint16_t n = 0;
        uint32_t gen = 0;
        uint32_t replan = 0;
        float    anchor_mm = 0.0f;
        // The backstop's frame and the limit set, mm, the strip was filled
        // under: the steer's clamp, fence, cap and residual read these.
        float    lo_mm = 0.0f;
        float    hi_mm = 0.0f;
        EngineLimits lim{};
        // Past the last entry: a stop from its state, certified at the fill
        // (certifiedStop()), absolute times, mm; tail_hard when it brakes
        // harder than lim to stay in the frame.
        kinetic2::Profile tail{};
        bool     tail_hard = false;
        // Every entry and the tail stay in the frame (applyWall(), the
        // tail's certification): the steer's wall bound is priced over one
        // tick. False when no stop up to kHardStopMax kept them in: the last
        // line, priced over kTickDtCapS.
        bool     in_frame = true;
        std::array<float, kStripLen> p_mm{};
    };

public:
    // The strip as last published, copied under the lock, for host tests.
    // Nothing on the board calls it.
    PlanStrip publishedStrip() const {
        if (_lock) _lock(true);
        const PlanStrip s = _strip_pub;
        if (_lock) _lock(false);
        return s;
    }

private:
    // planTick() up to the strip: false when no plan renders this tick (the
    // e-stop, power and frame-move branches, a home cycle).
    bool planStep(uint64_t now_us, float dt_s);
    // Refills or advances _strip and publishes it; an empty one when !live.
    void fillStrip(uint64_t now_us, bool live);
    // A whole refill's join to the trajectory in flight (the strip last
    // published, or the carriage at rest after a lapse): the new plan with
    // the connector in flight when that holds the set across t0; else kept
    // through kHandoverTicks and joined through a validated connector, or
    // refused for a certified stop from it. s.p_mm holds the new plan on
    // entry and on return; applyConnector() makes it the trajectory.
    void handover(PlanStrip& s, bool from_rest, uint64_t now_us);
    // THE GLIDE (bd val-5ozv.2, operator ruling 2026-10-25): a window edit
    // that leaves the carriage outside the new window moves it in, never an
    // edge brake and a wait. Asked for on the planner (glideIn()), built at
    // the next whole refill's handover from the trajectory in flight there:
    // its velocity first brought inside the jog set at the set it renders
    // under (Profile::converge), then the jog set's fastest move to rest at
    // aim_mm, or where it comes to rest at the jog set held in the window
    // (aim NaN); a glide that would leave the rail stops inside it first and
    // glides from that rest. It is the connector over the engine held at its
    // rest, as a refused plan's stop is: every intent is refused until it
    // ends, the strip's frame widened to the glide's extent meanwhile.
    void glideIn(float aim_mm, const char* why, bool from_rest);
    // The glide from s at at_us (mm, absolute times) into [lo, hi], and when
    // its converge ends (at_us when it has none); false when it does not fit
    // one profile.
    bool glideProfile(const kinetic2::State& s, uint64_t at_us, float aim_mm, float lo, float hi,
                      kinetic2::Profile& out, uint64_t& conv_end_us) const;
    bool gliding(uint64_t t) const { return t < _glide_end_us; }
    // planStep() after a window edit's reset or reseed: the cell's action on
    // the motion in flight, and the edit's one log line unless a glide's
    // build logs it.
    void windowEdited(ArbAct act, uint64_t now_us);
    // A window edit an ESTOP, a power loss or a cycle holds: counted and
    // logged once per hold, applied when the hold ends.
    void holdWindow(const char* until, uint64_t now_us);
    // The connector in flight at t over a plan at plan_mm there: before the
    // handover the trajectory it kept, minus the plan; then its gap.
    float connAt(uint64_t t, float plan_mm) const;
    // Adds the connector in flight to s.p_mm (the plan and the oscillator).
    void applyConnector(PlanStrip& s) const;
    // A connector, or a refused plan's stop, still moves the trajectory at t
    // toward the engine's rest: the engine is not the whole plan yet.
    bool joining(uint64_t t) const { return _conn.n > 0 && t < _conn_end_us; }
    // The fastest stop from s at at_us under lim, mm: never a reversal, at
    // rest at its end, inside [lo, hi] widened to s.p and by slop. A stop
    // that would leave it brakes harder, the least multiple of amax and jmax
    // up to kHardStopMax that keeps it in, counted.
    kinetic2::Profile certifiedStop(const kinetic2::State& s, uint64_t at_us, const EngineLimits& lim, float lo,
                                    float hi, bool& hard, float slop = kMmPerStep);
    // THE WALL (bd val-vj1j): where s.p_mm leaves the strip's frame by more
    // than a step, it stops instead, at the edge: from the latest entry whose
    // stop under s.lim rests in the frame, its jerk eased to rest on the
    // edge, else harder from the first (certifiedStop(), counted). Kept from
    // its commit until it rests and the plan is back in the frame
    // (fillStrip()). Returns false when no stop up to kHardStopMax keeps the
    // strip in.
    bool applyWall(PlanStrip& s);
    // The wall moves the trajectory at t toward its rest.
    bool walling(uint64_t t) const { return _wall_on && t < _wall.end_us(); }
    // Committed: no fill from t0_us on could start the wall earlier, and the
    // steer may render it. It holds through every fill until its rest.
    static constexpr size_t kWallFirst = 2;
    bool wallCommitted(uint64_t t0_us) const {
        return _wall_on && _wall_from_us <= t0_us + kWallFirst * kMotionTickUs;
    }
    // The limit set the plan in flight renders under, mm: the jog set's for
    // a Manual plan and the RETURN, the input set's otherwise.
    EngineLimits renderLimits() const;
    // The strip's trajectory at t_us, mm: its first entry before t0_us, the
    // two entries around t_us interpolated, and its tail past the last.
    static float stripAt(const PlanStrip& s, uint64_t t_us);
    // Writes the emitter's fence for the frame [lo_mm, hi_mm], or the home
    // cycle's search while one runs. Before every nonzero steer.
    void syncFence(float lo_mm, float hi_mm);
    // Forgets the plan and holds at `p_norm` from now_us. Every resetAt() goes
    // through here, so the Kinetic² boundary state resets with the engine.
    // rebase: the steer re-anchors at the carriage (a new gen) and the
    // connector is dropped. Without it the trajectory in flight renders on
    // and the engine joins it at rest (a refused plan's stop).
    void resetEngine(float p_norm, uint64_t now_us, bool rebase = true);
    // The window moved under a plan in flight: the pending knots stay and the
    // plan's curve is restated in the new frame (bd val-17u), kept through
    // the reaction horizon or the next knot (bd val-4dt).
    void reseedEngine(uint64_t now_us);
    // Planner task: applies the window setWindow() last posted, if one is
    // new and whole, and raises the frame-move flag. True when it applied.
    bool takeWindow();
    // Planner task: hands the oscillator the parameters setOscillator() last
    // posted, if new and whole.
    void takeOscillator();
    // Planner task: the stage's parameters from the post, enabled while
    // osc.enabled is or a live stream renders it (streamOscillator()). A
    // parameter driven by a live input (speed, position, or axis with a live
    // stream) is the driven mode: a sine whose frequency and amplitude follow
    // the points feedOscillator() gives it. Otherwise the fixed mode, the
    // sine with its dwells.
    void setOscStage();
    bool oscAxisBound() const;
    bool oscPlanBound() const;
    // An osc-drive sample has been drained and t_us is under
    // stream_quiet_release_ms past the newest stamp.
    bool oscStreamLive(uint64_t t_us) const;
    // Planner task, from fillStrip() every tick before the plan read: a
    // stream live at now (a sample waiting in the ring counts) enables and
    // drives the stage while an axis drive is bound; quiet, the hand state
    // again, through rest.
    void streamOscillator(uint64_t now_us);
    // Planner task, from fillStrip() every tick with the plan from t0
    // (kOscEdge read before it). It drains the osc-drive ring; rendering,
    // each sample becomes a drive point at its stamp, and while a speed or
    // position drive is bound a point at t0 + kOscDriveLeadUs every
    // kOscPlanDriveTicks from the plan there. Each parameter through its own
    // drive: fixed its field, speed and position the plan, axis the newest
    // sample, or with no live stream its field while osc.enabled holds and 0
    // otherwise (bd val-o9r (a)).
    void feedOscillator(uint64_t t0_us, const float* plan, bool render);
    // One drive point at t_us, the axis sample's values or the newest held.
    void drivePoint(uint64_t t_us, const float* plan, uint64_t t0_us);
    // Stops as fast as the engine's current limits allow, from its own
    // state at at_us. False when there was nothing moving to stop.
    bool brakeEngine(uint64_t at_us);
    // Engine::expect on axis 0, mirrored for expectUntil(). Every submit sets
    // it (a Stream segment's horizon, else 0), and a brake clears it.
    void setExpect(uint64_t until_us);
public:
    // Books one solve's cost (the planner's own); public so the host tests can
    // drive the horizon with a solve cost they choose.
    void notePlanCost(uint32_t us);
    // The reaction horizon the last submit chose (adaptiveReactUs).
    uint32_t reactHorizonUs() const { return _react_us_now; }
    // The smoothed lead of the accepted knots: due time minus arrival, hub clock.
    uint32_t leadUsAvg() const { return uint32_t(_lead_us_avg); }
private:

    // The engine's frame: its normalized 0..1 is the travel window, or the
    // whole rail while the operator jogs under override (the engine clamps to
    // its frame, so lifting the window means widening the frame).
    float frameLo() const { return _rail_frame ? 0.0f : _win_min; }
    float span() const { return _rail_frame ? _rail : _win_max - _win_min; }
    float toNorm(float mm) const { return (mm - frameLo()) / span(); }
    float toMm(float norm) const { return frameLo() + norm * span(); }
    float winSpan() const { return _win_max - _win_min; }
    // The position backstop's frame, mm (steerTick()): the configured window
    // held inside the rail, or the asserted rail while the engine plans in
    // the rail frame (override's jog and its return, SPEC 11.1).
    float backstopLo() const { return _rail_frame ? 0.0f : (_win_min > 0.0f ? _win_min : 0.0f); }
    float backstopHi() const { return _rail_frame ? _rail : (_win_max < _rail ? _win_max : _rail); }
    // The INPUT set's ceilings as the engine plans them, in mm: the mm limit,
    // or the normalized override scaled by the CURRENT window. steerTick()'s
    // tracking cap reads the same answer, so a plan an override allowed is
    // never capped below its own ceiling at render time.
    float inputVmaxMm() const { return _ovr_v > 0.0f ? _ovr_v * winSpan() : _in_v; }
    float inputAmaxMm() const { return _ovr_a > 0.0f ? _ovr_a * winSpan() : _in_a; }
    float inputJmaxMm() const { return _ovr_j > 0.0f ? _ovr_j * winSpan() : _in_j; }
    // Moves the engine's frame at rest: reseeds it at the carriage, so the
    // move is a relabeling, never motion.
    void setRailFrame(bool on, uint64_t now_us);
    EngineLimits limitsFor(bool manual) const;
    void brakeToRest(uint64_t now_us);
    // The power gate as accept(), both ticks and returnToPause() apply it.
    bool powerGateOpen() const { return kBenchNoMotor || _powered.load(); }
    // Plans `target` (already clamped, mm) from the machine's actual state
    // under `lim`. Planner task; counts the plan cost and a failure as a
    // rejection.
    bool plan(float target, const MotionIntent& in, const EngineLimits& lim, uint64_t now_us);
    // The Kinetic² boundary (MotionArbiter.cpp): an intent becomes a knot, a
    // brake or a refusal here and nowhere else.
    bool submitKnots(float target_norm, const MotionIntent& in, const EngineLimits& lim, uint64_t now_us);
    // The one door to the engine's stateAt(): records the brake the engine
    // takes from a knot that ends the timeline still moving (RFC-105 (dd)).
    kinetic2::State sampleEngine(uint64_t now_us);
    // The ownership table (ArbMode above), planner task only. apply() looks
    // up (mode, ev), moves the mode, logs and counts the change, and returns
    // the cell's action; an illegal cell holds the mode and returns illegal.
    // Callers run the action at the point the event is taken.
    ArbAct apply(ArbEvent ev, const char* cause, uint64_t now_us);
    // The mode the SPEC 11.1 latches name: Jog under override, else Paused
    // under PAUSE, else Run.
    ArbMode layer() const;
    // Applies every posted change that carries no action: the release, the
    // gate open again, the layer, a home request posted or withdrawn. An
    // ESTOP latch or a shut gate carries one, so it is left to planStep() and
    // reported: the mode returned is then Estop or Unpowered. accept() gates
    // on what this returns.
    ArbMode syncLatches(uint64_t now_us);
    ArbMode mode() const { return _mode.load(); }
    bool seeking() const { return arbSeeking(_mode.load()); }
    // The home cycle, planner task only (MotionArbiter.cpp, homing). A leg:
    // approach, clear (the backoff), touch; then the far leg, or finish (the
    // final backoff): HomeApproach..HomeFinish.
    void homeStart(uint64_t now_us);
    void homeStep(uint64_t now_us, float dt_s);
    // A seek's stall at emitter count `hit`, the emitter already parked.
    void homeContact(bool touch, int32_t hit, uint64_t now_us);
    // The far datum is in: the rail is measured, the frame becomes it, and
    // the final backoff starts.
    void homeMeasure(uint64_t now_us);
    // Starts the leg's approach (`touch` false) or re-touch at its speed, and
    // arms the sense: at once when the line is known LOW, else on its first
    // LOW read (the far approach starts pressed against the home stop).
    void homeSeek(bool touch, bool armed, ArbEvent ev, uint64_t now_us);
    // Starts a constant-velocity leg toward `end_mm` at v_mm_s, in the mode
    // `ev` moves to, before the sense is armed.
    void homeLeg(ArbEvent ev, float end_mm, float v_mm_s, uint64_t now_us);
    // The seek producer's one tick: steers the leg's velocity, or parks and
    // answers true once the count has reached the leg's end.
    bool homeDrive(uint64_t now_us, float dt_s);
    // Steers v_mm_s, then parks again if the interrupt tripped meanwhile.
    void homeSteer(float v_mm_s);
    // Leg geometry in the physical frame, mm: the label of the leg's stop
    // datum (a safety margin outside the rail), the point `d` back off the
    // stop from `at_mm` (a negative `d` reaches past it), and the end of an
    // approach's search (MotionArbiter.h homing).
    bool  homeTowardLow(uint8_t leg) const { return (leg == 0) != _home_flip; }
    float homeStopAt(uint8_t leg) const {
        return homeTowardLow(leg) ? -kHomeSafetyMarginMm : _rail + kHomeSafetyMarginMm;
    }
    float homeAway(uint8_t leg, float at_mm, float d) const { return homeTowardLow(leg) ? at_mm + d : at_mm - d; }
    float homePast(uint8_t leg) const {
        return homeAway(leg, homeStopAt(leg), (leg == 0 ? -2.0f : -1.0f) * kHomeSearchMarginMm);
    }
    // Re-origins the count so the carriage reads `at_mm` and reseeds the
    // engine there: a relabeling, never motion.
    void homeOrigin(int32_t count, float at_mm, uint64_t now_us);
    // Ends the request or the cycle by `ev`: parks, disarms the interrupt,
    // and resets the planner at the count when a leg ran, then moves the mode.
    // A failure records `why` (nullptr: homed) and the leg for the census.
    void homeEnd(ArbEvent ev, const char* why, uint64_t now_us);

    MotionEmitter& _emitter;
    Clock          _now_us;
    Lock           _lock;

    MotionEngine _engine{engineConfig()};

    // The strip (PlanStrip). _strip, _strip_stale and _anchor_mm are the
    // planner's: the strip it advances, a plan change since its last refill,
    // and the position of the last engine reset. _strip_pub is the shared
    // copy, written only by the planner and only under _lock. _strip_gen
    // counts engine resets that rebase (resetEngine(), planner the one
    // writer): a published strip behind it is stale and steers nothing. The
    // rest is the steer's: its copy of the strip, the gen it last took, a
    // tick it did not steer from the plan, a run past the strip's end in
    // progress, and its counter.
    PlanStrip _strip{};
    bool      _strip_stale = true;
    float     _anchor_mm = 0.0f;
    // The frame the engine's units were last set in (resetEngine(),
    // reseedEngine()): a reseed restates the plan out of it.
    float     _eng_lo = 0.0f;
    float     _eng_span = DEFAULT_MAX_RAIL_MM;
    PlanStrip _strip_pub{};
    // The plan the strip is cut from, mm, on the strip's grid from
    // kOscEdge ticks before t0 (fillStrip()), and the oscillator summed into
    // it: its offset at t0.
    std::array<float, kPlanExt> _plan_ext{};
    MotionOscillator _osc{};
    float     _osc_head = 0.0f;
    size_t    _plan_ahead = 0;   // entries from t0 the last refill read
    std::atomic<uint32_t> _strip_gen{0};
    PlanStrip _steer_strip{};
    uint32_t  _seen_gen = 0;
    bool      _steer_gap = false;
    bool      _strip_starved = false;
    uint32_t  _planner_stalls = 0;
    uint32_t  _hard_tails = 0;   // stalls whose tail brakes harder than the set
    // The residual correction (steerTick()): its velocity and acceleration,
    // and the previous feedforward, whose change is the plan's acceleration.
    float     _res_v = 0.0f;
    float     _res_a = 0.0f;
    float     _ff_prev = 0.0f;
    // A lapse's re-plan (bd val-xfy2): the steer counts its requests and
    // parks until a strip answers the newest; the planner answers at its
    // next fill (_replan_seen, PlanStrip::replan).
    uint32_t  _replan_want = 0;
    std::atomic<uint32_t> _replan_req{0};
    uint32_t  _replan_seen = 0;

    // THE CONNECTOR (handover(), planner task): the strip is the plan plus
    // this. Before _conn_h_us the trajectory the steer was rendering
    // (_conn_pre, from _conn_pre_t0 on the grid); from it to _conn_end_us the
    // plan plus _conn, a gap closing to 0. A refused plan's stop is the same
    // shape over the engine held at its rest; until _held_until_us every
    // intent is refused.
    kinetic2::Profile _conn{};
    uint64_t  _conn_h_us = 0;
    uint64_t  _conn_end_us = 0;
    uint64_t  _conn_pre_t0 = 0;
    std::array<float, kHandoverTicks> _conn_pre{};
    uint64_t  _held_until_us = 0;
    // The next handover follows a brake or a cut (brakeToRest()): a fade,
    // never a late plan.
    bool      _handover_quiet = false;
    uint32_t  _late_plans = 0;
    float     _late_plan_max_mm = 0.0f;   // the largest late-plan gap since boot
    uint32_t  _plans_refused = 0;
    uint32_t  _hard_stops = 0;

    // THE GLIDE (glideIn(), planner task). Asked for: built at the next
    // whole refill (handover()), intents refused from the ask. In flight:
    // until _glide_end_us, the strip's frame widened to [_glide_lo,
    // _glide_hi]. _glide_again: the glide stopped first inside the rail and
    // glides again from that rest.
    bool        _glide_req = false;
    bool        _glide_again = false;
    bool        _glide_rest = false;    // from the carriage at rest, not the strip
    float       _glide_aim_mm = 0.0f;
    const char* _glide_why = "";
    uint64_t    _glide_end_us = 0;
    float       _glide_lo = 0.0f, _glide_hi = 0.0f;
    uint32_t    _glides = 0;
    float       _glide_max_mm = 0.0f;
    // The window an edit replaced (takeWindow(), the first post since the
    // last edit applied) and the trajectory then, for the edit's decision
    // and its log line.
    bool        _win_old_valid = false;
    float       _win_old_min = 0.0f, _win_old_max = 0.0f, _win_old_rail = 0.0f;
    uint32_t    _window_edits = 0;
    uint32_t    _window_held = 0;
    bool        _win_held = false;

    // THE WALL (applyWall(), planner task): a stop to the frame's edge, mm,
    // absolute times, rendered for every entry after _wall_from_us.
    // Committed by wallCommitted(); until then each fill finds it again.
    kinetic2::Profile _wall{};
    uint64_t  _wall_from_us = 0;
    bool      _wall_on = false;
    bool      _wall_ok = true;        // it keeps the strip in the frame
    bool      _wall_hard = false;
    bool      _wall_counted = false;
    uint32_t  _walls = 0;             // census backstops, counted at commit

    // Kinetic² boundary state, planner task only. The newest knot the engine
    // holds (or the rest point after a reset, or a brake's end): a segment
    // starting after it is a rest until its start, and a jog chains from it.
    uint64_t _k2_newest_us = 0;
    float    _k2_newest_p  = 0.0f;
    // The brake in flight, engine frame, which the engine does not report.
    // starved: the engine's own brake from a starved knot, which a new knot
    // replaces; otherwise an explicit brake, which renders to its end.
    uint64_t _k2_brake_from_us = 0, _k2_brake_to_us = 0;
    float    _k2_brake_from_p  = 0.0f, _k2_brake_to_p = 0.0f;
    bool     _k2_starved = false;
    bool     _k2_chase = false;           // the newest knot is a sample's
    bool     _k2_window_clamped = false;  // the last plan's target was window-clamped
    bool     _k2_dirty = false;           // submitted since the last sample: it solves
    uint64_t _k2_expect_us = 0;           // the engine's expectation (setExpect())
    // The hub released the stream (releaseRail(Stream)); the planner consumes it.
    std::atomic<bool> _stream_quiet{false};
    // The plan in flight was planned under the jog set (a Manual move, the
    // RETURN): set by plan(), cleared once the engine is idle. Planner task
    // only; the steer reads the set from the strip (PlanStrip::lim).
    bool _plan_manual = false;
    // Planner only: a jog that braked the motion in flight first, waiting for
    // that brake's end to take the jog set's cap; 0 when none.
    uint64_t _jog_after_us = 0;
    uint32_t _k2_plans = 0, _k2_failures = 0;
    // From applyTuning(): the samples grant's latency (sampleLatencyUs()).
    uint32_t _k2_latency_us   = sampleLatencyUs(motionDefaultTuning());

    // The frame in force. The planner writes the window (takeWindow()), so
    // a strip is never filled half in each frame; steerTick() reads it after
    // the frame-move flag.
    float _win_min = 0.0f;
    float _win_max = DEFAULT_MAX_RAIL_MM;
    float _rail    = DEFAULT_MAX_RAIL_MM;
    // The configured max_rail, which force_home's stroke never shrinks: the
    // home cycle's search distance.
    float _max_rail = DEFAULT_MAX_RAIL_MM;
    // The window setWindow() posted: a seqlock, odd while the hub task
    // writes it. _win_taken, the planner's, is the post last applied.
    std::atomic<uint32_t> _win_seq{0};
    std::atomic<float>    _win_req_lo{0.0f};
    std::atomic<float>    _win_req_hi{0.0f};
    std::atomic<float>    _win_req_rail{0.0f};
    std::atomic<bool>     _win_req_edit{false};
    uint32_t              _win_taken = 0;
    // The post last taken was an adoption (WindowPost::adopt): planStep()
    // applies ArbEvent::WindowAdopt for it. Planner task.
    bool                  _win_adopt = false;
    // A post at or under this sequence was written before a completed cycle
    // measured the rail: taken late (a cycle holds it), it keeps the
    // measured rail. The hub pushes the measured one back as a newer post.
    uint32_t              _win_rail_kept = 0;
    // The oscillator's parameters setOscillator() posted, the same seqlock.
    std::atomic<uint32_t> _osc_seq{0};
    std::atomic<bool>     _osc_req_on{false};
    std::atomic<float>    _osc_req_hz{0.0f};
    std::atomic<float>    _osc_req_amp{0.0f};
    std::atomic<float>    _osc_req_crest{0.0f};
    std::atomic<float>    _osc_req_trough{0.0f};
    std::atomic<uint8_t>  _osc_req_fdrive{0};
    std::atomic<uint8_t>  _osc_req_adrive{0};
    std::array<std::atomic<float>, 4> _osc_req_fmap{};   // in_min, in_max, out_min, out_max
    std::array<std::atomic<float>, 4> _osc_req_amap{};
    uint32_t              _osc_taken = 0;
    // The parameters as posted, before the drives, the amplitude in mm
    // (planner task).
    kinetic2::OscParams   _osc_base{};
    MotionOscDrive        _osc_fdrive{.drive = 0};
    MotionOscDrive        _osc_adrive{.drive = 0};
    // The osc-drive samples: single producer (the hub task) writes a slot,
    // then publishes head; single consumer (the planner) reads, then
    // publishes tail.
    struct OscDriveSample {
        uint64_t t_us = 0;
        float    amplitude = 0.0f;
        float    frequency = 0.0f;
    };
    std::array<OscDriveSample, 16> _oscd_ring{};
    std::atomic<uint32_t> _oscd_head{0};
    std::atomic<uint32_t> _oscd_tail{0};
    std::atomic<uint32_t> _oscd_ring_lost{0};
    // The newest sample drained (planner task), its stamp the quiet test's.
    bool                  _oscd_have = false;
    OscDriveSample        _oscd_last{};
    uint64_t              _oscd_plan_next_us = 0;
    uint32_t              _oscd_dropped = 0;
    // A live stream enables and drives the stage while an axis drive is
    // bound (streamOscillator()). Planner task.
    bool                  _osc_streaming = false;

    float _jog_v = DEFAULT_JOG_MAX_SPEED_MM_S;
    float _jog_a = DEFAULT_JOG_ACCEL_MM_S2;
    float _in_v   = DEFAULT_MAX_SPEED_MM_S;
    float _in_a   = DEFAULT_ACCEL_MM_S2;
    float _in_j   = DEFAULT_INPUT_MAX_JERK_MM_S3;
    // Normalized ceiling overrides from 0x3120, 0 = derived. Written by
    // applyTuning() on the planner, read on the planner.
    float _ovr_v = 0.0f;
    float _ovr_a = 0.0f;
    float _ovr_j = 0.0f;

    int32_t _origin   = 0;       // the emitter count that means 0.0 mm
    // The commanded position at the previous steer. Steer task; brakeToRest()
    // on the planner reads it as one word.
    float   _p_cmd_mm = 0.0f;
    // The fence as last written, emitter counts, by whichever task steers
    // (syncFence()). The LP core loads it open.
    int32_t _fence_lo = INT32_MIN;
    int32_t _fence_hi = INT32_MAX;

    volatile bool _homed  = false;
    volatile bool _estop  = false;
    // Written by setMotorPowered() on the switch host's task. _power_settled
    // re-arms on a loss and is spent once by planTick().
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
    // Planner task only.
    bool     _rail_frame  = false;
    // Where the last pause brought the machine to rest (the return target),
    // and the completed returns. Written by the planner and, for an
    // unpowered arrival, by returnToPause() on the hub task: atomics, so the
    // two never tear a value or lose an increment.
    std::atomic<float>    _pause_pos_mm{0.0f};
    std::atomic<uint32_t> _returns{0};
    // The bench bypass has been logged this boot. Planner task only.
    bool     _bench_noted = false;
    // Set by setWindow()/forceHome() on any task and by takeWindow(),
    // consumed by planTick(): the mm FRAME moved, the carriage did not.
    std::atomic<bool> _frame_moved{false};
    // Set by forceHome() before _frame_moved: the count re-origined, so the
    // planner resets rather than reseeds, and the steer parks while it is up
    // (the strip it holds is in the old origin's mm).
    std::atomic<bool> _origin_moved{false};

    // THE MODE (ArbMode): written by the planner task alone, through apply().
    // An atomic, because the steer reads it to stand aside while the seek
    // producer owns the emitter (seeking()). The census counts every change
    // and every illegal event; the log's burst window runs on the planner's
    // clock.
    std::atomic<ArbMode> _mode{ArbMode::Unpowered};
    uint32_t _arb_transitions = 0;
    uint32_t _arb_illegal = 0;
    uint64_t _arb_log_from_us = 0;
    uint32_t _arb_log_n = 0;
    uint32_t _arb_log_lost = 0;

    // The home cycle. _homing is set by home() and cleared only by the
    // planner when the request or the cycle ends; _home_req hands the start
    // across; pause() sets _home_abort. The rest is the planner's.
    HomeSense* _sense = &noHomeSense();
    std::atomic<bool> _homing{false};
    std::atomic<bool> _home_req{false};
    std::atomic<bool> _home_abort{false};
    uint8_t   _home_leg = 0;      // 0 the home end, 1 the far end
    bool      _home_flip = false; // the flip as the cycle started
    uint64_t  _home_deadline_us = 0;
    bool      _sense_armed = false;
    // The seek leg in flight: its end on the emitter count, its direction on
    // that count, its speed, its start, and the velocity last steered (the
    // census reads it while a cycle runs).
    int32_t   _leg_end = 0;
    int32_t   _leg_dir = 1;
    float     _leg_v = 0.0f;
    uint64_t  _leg_start_us = 0;
    float     _seek_v = 0.0f;
    // The interrupt's handshake with the planner. The task stores armed
    // (and idle); homeSenseRose() alone moves armed -> tripping -> latched,
    // storing _seek_hit before latched (release). 32-bit words: a native
    // compare-and-swap on the HP cores, safe in interrupt context.
    static constexpr uint32_t kSeekIdle     = 0;
    static constexpr uint32_t kSeekArmed    = 1;
    static constexpr uint32_t kSeekTripping = 2;
    static constexpr uint32_t kSeekLatched  = 3;
    std::atomic<uint32_t> _seek{kSeekIdle};
    std::atomic<int32_t>  _seek_hit{0};
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

    // Odometer state, planner task only.
    int32_t _odo_steps = 0;       // emitter count at the previous snapshot
    int32_t _stroke_dir = 0;      // sign of the run in progress
    float   _stroke_run_mm = 0.0f;

    // Counters outside any published census, so a host publishes only what
    // snapshot() built, under its own lock. _stalls through _lease_lapses
    // are the steer task's; the rest the planner's.
    uint32_t _intents  = 0;
    uint32_t _rejected = 0;
    uint32_t _stalls    = 0;       // steers later than kTickDtCapS
    uint32_t _clamps = 0;          // the last line: the steer held the demand
                                   // at the edge, counted at onset
    bool     _backstop_on = false; // a clamp is in progress
    uint32_t _lapses_seen = 0;     // the emitter's lapses() at the last steer
    uint32_t _lease_lapses = 0;    // lapses since begin()
    uint32_t _plan_us_last = 0;
    uint32_t _plan_us_max  = 0;
    float    _plan_us_avg  = 0.0f;
    // Lead of the accepted knots (the knot's due time minus its arrival),
    // EMA alpha 1/4: about the last four knots. 0 after a reset.
    float    _lead_us_avg  = 0.0f;
    MotionSource _lead_source = MotionSource::Manual;   // the source the lead is measured for
    uint32_t _react_us_now = 0;    // the horizon the last submit chose
    uint32_t _react_floor_us = kinetic2::Config{}.react_us;   // the react_ms setting (applyTuning)
    uint32_t _react_us_logged = 0; // the last horizon logged (one line per change, throttled)
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
