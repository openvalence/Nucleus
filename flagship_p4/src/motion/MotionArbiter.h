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
//   home cycle, _p_cmd_mm and the backstop, stall and strip counters. It
//   never touches the engine: the plan reaches it as the STRIP (PlanStrip),
//   which planTick() publishes and steerTick() copies, each under the host's
//   Lock.
//   evaluate() is planTick() then steerTick() on one task: the host twin's
//   and the native suites' entry.
// - Every conversion between this interface and Kinetic²'s knots lives in
//   MotionArbiter.cpp's Kinetic² boundary section, nowhere else.
// - CROSS-TASK methods (estop, pause, override, returnToPause, acquireRail,
//   releaseRail, setEstopCutsPower, setMotorPowered, setCommissioned, the limit and window setters,
//   setOscillator, forceHome, home, noteStream) never touch the engine.
//   They write flags and scalars the owning tasks read on their next pass;
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

// The longest interval a steer's residual kick and velocity cap are priced
// over: two sampler periods. A steer later than this is a STALL (steerTick()),
// counted and steered without the catch-up the elapsed time would ask for.
// Also the wall bound's horizon: no steer carries the carriage past the
// backstop's edge within this long.
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
// holds the last entry and counts a planner stall.
inline constexpr size_t kStripLen = 128;

// THE OSCILLATOR (RFC-103, bd val-dzf): Kinetic²'s stage, summed into the
// strip, its fade 150 ticks at most. It reads the plan kOscLook ticks past the
// strip's end, and kOscEdge before and after for its differences: the plan
// buffer the strip is cut from is kPlanExt long, from kOscEdge before t0.
using MotionOscillator = kinetic2::Oscillator<kStripLen, 150>;
inline constexpr size_t kOscEdge = MotionOscillator::kPlanEdge;
inline constexpr size_t kOscLook = MotionOscillator::lookahead(kMotionTickUs);
inline constexpr size_t kPlanExt = kOscEdge + kStripLen + kOscLook + kOscEdge;

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
    // Steer task. The strip at now_us, clamped to the backstop's frame, as
    // the feedforward and the residual kick; the fence, the steer and the
    // lease. dt_s: the interval since the previous steerTick(); past
    // kTickDtCapS it is a stall. Steers nothing while a home cycle runs.
    void steerTick(uint64_t now_us, float dt_s);
    // One task, both halves in order: the host twin's and the tests' tick.
    void evaluate(uint64_t now_us, float dt_s) {
        planTick(now_us, dt_s);
        steerTick(now_us, dt_s);
    }
    // Steer task's counters, for host tests and logs, never a census field:
    // ticks that ran past the strip's end (counted at onset), and new plans
    // that landed more than a step off the strip the steer was rendering.
    uint32_t plannerStalls() const { return _planner_stalls; }
    uint32_t latePlans() const { return _late_plans; }
    // Returns the kinds drained, bit k = kinetic2::AnomalyKind k.
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
    // planTick(), so winMin(), winMax() and rail() read it from then on.
    void setWindow(float lo, float hi, float rail);
    // RFC-103 (SPEC 9.7): the oscillator's parameters, already clamped by the
    // delegate. ONE WRITER, the hub task: a post the planner takes at its next
    // tick (takeOscillator()). The oscillator rides whatever owns the rail and
    // the rest between, yields first under the input set's ceilings, never
    // leaves the window, and renders nothing under ESTOP, PAUSE (cut at once,
    // the phase back at the trough), unhomed or uncommissioned.
    void setOscillator(const MotionOsc& o);
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

    // THE STRIP: the plan's position at t0_us + i * kMotionTickUs, mm, for i
    // in [0, n). n is 0 (steer nothing) or kStripLen. gen is the engine reset
    // it follows: the steer re-anchors its feedforward at anchor_mm, the
    // carriage's position at that reset, once per gen. plan moves with every
    // whole refill (a submit, a brake, a reset, an expired strip); between
    // refills the strip only advances.
    struct PlanStrip {
        uint64_t t0_us = 0;
        uint16_t n = 0;
        uint32_t gen = 0;
        uint32_t plan = 0;
        float    anchor_mm = 0.0f;
        // The gen follows a reseed (reseedEngine()): the plan runs on in mm,
        // so the steer keeps its command and never re-anchors.
        bool     continuous = false;
        // This refill cut the oscillation (a PAUSE): the gap it leaves at
        // now is the oscillator's, closed by the kick, never a late plan.
        bool     cut = false;
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
    // The strip at t_us, interpolated between the two entries around it,
    // into p_mm (clamped to the first or last entry outside it). False when
    // t_us lies outside [t0_us, the last entry] or the strip is empty.
    static bool stripAt(const PlanStrip& s, uint64_t t_us, float& p_mm);
    // Writes the emitter's fence for the frame in force: the backstop's, or
    // the home cycle's search while one runs. Before every nonzero steer.
    void syncFence();
    // Forgets the plan and holds at `p_norm` from now_us. Every resetAt() goes
    // through here, so the Kinetic² boundary state resets with the engine.
    void resetEngine(float p_norm, uint64_t now_us);
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
    // Stops as fast as the engine's current limits allow, from its own
    // state at at_us. False when there was nothing moving to stop.
    bool brakeEngine(uint64_t at_us);
    // Engine::expect on axis 0, mirrored for expectUntil(). Every submit sets
    // it (a Stream segment's horizon, else 0), and a brake clears it.
    void setExpect(uint64_t until_us);
    void notePlanCost(uint32_t us);

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
    // The home cycle, planner task only (MotionArbiter.cpp, homing). A leg:
    // approach, clear (the backoff), touch; then the far leg, or finish (the
    // final backoff).
    enum class HomePhase : uint8_t { idle, approach, clear, touch, finish };
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
    void homeSeek(bool touch, bool armed, uint64_t now_us);
    // Starts a constant-velocity leg in `phase` toward `end_mm` at v_mm_s.
    void homeLeg(HomePhase phase, float end_mm, float v_mm_s, uint64_t now_us);
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
    // Ends the cycle: parks, disarms the interrupt, and resets the planner at
    // the count. A failure records `why` and the leg for the census.
    void homeEnd(const char* why, uint64_t now_us);

    MotionEmitter& _emitter;
    Clock          _now_us;
    Lock           _lock;

    MotionEngine _engine{engineConfig()};

    // The strip (PlanStrip). _strip, _strip_stale and _anchor_mm are the
    // planner's: the strip it advances, a plan change since its last refill,
    // and the position of the last engine reset. _strip_pub is the shared
    // copy, touched only under _lock. _strip_gen counts engine resets
    // (resetEngine(), planner the one writer): a published strip behind it is
    // stale and steers nothing. The rest is the steer's: its copy of the
    // strip, the gen and plan it last took, a tick it did not steer from the
    // plan, a run past the strip's end in progress, and its two counters.
    PlanStrip _strip{};
    bool      _strip_stale = true;
    float     _anchor_mm = 0.0f;
    bool      _strip_continuous = false;
    // The frame the engine's units were last set in (resetEngine(),
    // reseedEngine()): a reseed restates the plan out of it.
    float     _eng_lo = 0.0f;
    float     _eng_span = DEFAULT_MAX_RAIL_MM;
    PlanStrip _strip_pub{};
    // The plan the strip is cut from, engine frame, on the strip's grid from
    // kOscEdge ticks before t0 (fillStrip()), and the oscillator summed into
    // it: its offset at t0, and a PAUSE's cut for the next refill to carry.
    std::array<float, kPlanExt> _plan_ext{};
    MotionOscillator _osc{};
    float     _osc_head = 0.0f;
    bool      _osc_cut = false;
    size_t    _plan_ahead = 0;   // entries from t0 the last refill read
    std::atomic<uint32_t> _strip_gen{0};
    PlanStrip _steer_strip{};
    uint32_t  _seen_gen = 0;
    uint32_t  _seen_plan = 0;
    bool      _steer_gap = false;
    bool      _strip_starved = false;
    uint32_t  _planner_stalls = 0;
    uint32_t  _late_plans = 0;
    float     _late_plan_max_mm = 0.0f;   // the largest late-plan gap since boot

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
    // RETURN): set by plan(), cleared once the engine is idle. Planner the
    // writer, steerTick() the reader.
    std::atomic<bool> _plan_manual{false};
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
    uint32_t              _win_taken = 0;
    // The oscillator's parameters setOscillator() posted, the same seqlock.
    std::atomic<uint32_t> _osc_seq{0};
    std::atomic<bool>     _osc_req_on{false};
    std::atomic<uint8_t>  _osc_req_shape{0};
    std::atomic<float>    _osc_req_hz{0.0f};
    std::atomic<float>    _osc_req_amp{0.0f};
    std::atomic<float>    _osc_req_crest{0.0f};
    std::atomic<float>    _osc_req_trough{0.0f};
    uint32_t              _osc_taken = 0;

    float _jog_v = DEFAULT_JOG_MAX_SPEED_MM_S;
    float _jog_a = DEFAULT_JOG_ACCEL_MM_S2;
    float _in_v   = DEFAULT_MAX_SPEED_MM_S;
    float _in_a   = DEFAULT_ACCEL_MM_S2;
    float _in_j   = DEFAULT_INPUT_MAX_JERK_MM_S3;
    // Normalized ceiling overrides from 0x3120, 0 = derived. Written by
    // applyTuning() on the planner, read by accept() and by steerTick() (one
    // word each).
    float _ovr_v = 0.0f;
    float _ovr_a = 0.0f;
    float _ovr_j = 0.0f;

    int32_t _origin   = 0;       // the emitter count that means 0.0 mm
    // The plan position at the previous steer. Steer task; brakeToRest() on
    // the planner reads it as one word.
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
    // Planner task only; steerTick() reads _rail_frame as one word.
    bool     _rail_frame  = false;
    bool     _returning   = false;
    // Where the last pause brought the machine to rest (the return target),
    // and the completed returns. Written by the planner and, for an
    // unpowered arrival, by returnToPause() on the hub task: atomics, so the
    // two never tear a value or lose an increment.
    std::atomic<float>    _pause_pos_mm{0.0f};
    std::atomic<uint32_t> _returns{0};
    // The bench bypass has been logged this boot. Planner task only.
    bool     _bench_noted = false;
    // Set by setWindow()/forceHome() on any task and by takeWindow(),
    // consumed by planTick(), read by steerTick(): the mm FRAME moved, the
    // carriage did not.
    std::atomic<bool> _frame_moved{false};
    // Set by forceHome() before _frame_moved: the count re-origined, so the
    // planner resets rather than reseeds.
    std::atomic<bool> _origin_moved{false};

    // The home cycle. _homing is set by home() and cleared only by the
    // planner when the cycle ends; _home_req hands the start across; pause()
    // sets _home_abort. The rest is the planner's. _home is not idle exactly
    // while the seek producer owns the emitter: an atomic, because the steer
    // reads it to stand aside.
    HomeSense* _sense = &noHomeSense();
    std::atomic<bool> _homing{false};
    std::atomic<bool> _home_req{false};
    std::atomic<bool> _home_abort{false};
    std::atomic<HomePhase> _home{HomePhase::idle};
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
    uint32_t _backstops = 0;       // backstop engagements, counted at onset
    bool     _backstop_on = false; // an engagement is in progress
    uint32_t _lapses_seen = 0;     // the emitter's lapses() at the last steer
    uint32_t _lease_lapses = 0;    // lapses since begin()
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
