// ValenceMotion -- the arbiter, the engine's host task, and the LP core's
// steering word
// Constraints:
// - ONE DOOR OUT: steerLp() is the only writer of the emitter's shared words,
//   it is static to this file, and only the motion task calls it. That is the
//   MotionArbiter sole-caller rule in code (architecture.md section 2).
// - The engine and its task stack live in INTERNAL RAM, never PSRAM: the HP
//   side is unreachable while the flash cache is off, and the sampler must not
//   fault during an OTA write. Only the LP core is immune.
// - commit() nests KB-scale Ruckig temporaries on the CALLING stack, so it
//   runs on the motion task and nowhere else (T1, memory-budget.md T21).
// - The plan is computed AT INTENT ARRIVAL from the engine's actual (p, v, a).
//   The tick only EVALUATES it. Nothing here plans on a clock.
// - THE ENGINE IS TOUCHED BY THE MOTION TASK ONLY. Every cross-task reader
//   goes through _pub, a plain POD the tick refreshes under _mux; census()
//   copies it under the same lock and calls nothing.
// - Millimeters on the public interface, normalized 0..1 window units inside
//   the engine. The two never mix in one expression.
// See: ValenceMotion.h, .claude/rules/motion-control.md, bd val-091.4

#include "ValenceMotion.h"

#include <atomic>
#include <cmath>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "geiger/geiger.h"
#include "hub/valence_config.h"
#include "kinetic/kinetic.hpp"
#include "ulp_main.h"

namespace valence {
namespace {

constexpr const char* kTag = "motion";

// ---- machine constants ------------------------------------------------------

// One quadrature transition is one step. 20 mm/s is 4,172 transitions/s.
constexpr float kStepsPerMm = 208.608f;
constexpr float kMmPerStep  = 1.0f / kStepsPerMm;

// The LP core's clock, MEASURED, not declared: 50,000 edges per 5 s at 4,000
// cycles per edge with RTC_FAST on the XTAL. Its config home is
// sdkconfig.defaults; the measurement lives in .claude/rules/motion-control.md.
constexpr float kLpClockHz = 40.0e6f;

// Sampler period. The S3 product evaluated its plan at 1 kHz and that number
// is kept deliberately: it is the cadence the whole engine was benched at, the
// LP core renders every edge between ticks regardless, and a faster tick buys
// nothing because the emitter re-reads its steering word mid-wait anyway.
constexpr uint32_t kTickUs = 1000;

// cycles_per_edge = f_LP / (|v| * steps_per_mm), carried in Q8. The Q8 product
// overflows a 32-bit word below this speed: 1.024e10 / (2^32 * 208.608). One
// step at that rate takes 417 ms, so there is nothing under it worth steering.
constexpr float kParkMmS = 0.0115f;

// The emitter's own floor, ~4,790 mm/s: five times the machine's speed ceiling
// and therefore unreachable through the engine. A demand past it is a FAULT
// DETECTOR reading, never a shaper -- the ceilings are enforced in the engine.
constexpr uint32_t kMinCyclesPerEdge = 40;

// Residual tracking. The emitter renders a VELOCITY, so a step it could not
// emit on time is a step nothing else repays: the residual against position
// truth is closed here, at 50 Hz, and only once it exceeds one whole step so
// the quantization of the count itself never becomes commanded jitter.
constexpr float kTrackHz = 50.0f;

// How often the cross-task snapshot is refreshed. 50 Hz feeds a 60 Hz 0x1100
// and a 45 Hz 0x1110 with one engine sample per refresh instead of one per
// tick, which is the whole reason it is not simply done at kTickUs: the
// snapshot calls Engine::snapshot(), and an instrument billed at the tick rate
// is the class that manufactures the fault it observes (memory-budget.md T27).
constexpr uint32_t kPublishUs = 20000;

// Intent queue depth. A bundle carries up to limits::bundle_max_samples (32)
// samples and the hub delegate submits them in one pass, so the queue has to
// absorb a whole bundle plus whatever the previous one left; the motion task
// drains it within one tick. THE QUEUE IS NOT THE SCHEDULE: an anchored commit
// past the engine's own kScheduleDepth is refused by the engine and counted as
// a plan failure, which is the honest place for that ceiling to live.
constexpr uint32_t kIntentQueueDepth = 40;

// A rendered direction reversal has to clear this much travel before it counts
// as a stroke, so dither around a standstill never inflates the odometer.
constexpr float kStrokeMinMm = 1.0f;

// ---- the emitter's one door -------------------------------------------------

uint32_t g_emitter_faults = 0;

inline int32_t lpSteps() { return static_cast<int32_t>(ulp_g_pos); }

void steerLp(float v_mm_s) {
    const float mag = std::fabs(v_mm_s);
    // Written so a NaN parks: !(mag >= park), not (mag < park).
    if (!(mag >= kParkMmS)) {
        ulp_g_step_q8 = 0;
        return;
    }
    float cyc = kLpClockHz / (mag * kStepsPerMm);
    if (cyc < static_cast<float>(kMinCyclesPerEdge)) {
        cyc = static_cast<float>(kMinCyclesPerEdge);
        ++g_emitter_faults;
    }
    const uint32_t q8 = static_cast<uint32_t>(cyc * 256.0f + 0.5f);
    // DIRECTION FIRST, and this ordering is load-bearing. The LP core reads the
    // two words independently, so an interleaving is possible; writing dir
    // first makes the only reachable one "one edge at the OLD period in the NEW
    // direction", a timing slip of at most one period. Writing step first
    // instead allows one edge in the WRONG direction, which is a permanent
    // two-step position error nothing detects.
    ulp_g_dir     = (v_mm_s < 0.0f) ? 0u : 1u;
    ulp_g_step_q8 = q8;
}

// ---- MotionArbiter ----------------------------------------------------------
// The command gate: arbitration, limit-set selection, every safety gate, the
// window clamp, and the engine it hands accepted intents to.

class MotionArbiter {
public:
    bool begin();
    bool submit(const MotionIntent& in);     // any task: enqueue and wake
    void estop(bool on);
    void pause(bool on) { _paused = on; }
    void setUserLimits(float v, float a) { _user_v = v; _user_a = a; }
    void setInputLimits(float v, float a, float j) { _in_v = v; _in_a = a; _in_j = j; }
    void setWindow(float lo, float hi, float rail);
    float forceHome(float stroke_mm);
    void noteStream(uint32_t bundles, uint32_t samples, uint32_t dropped);
    MotionCensus census() const;

private:
    static void taskTrampoline(void* self) { static_cast<MotionArbiter*>(self)->run(); }
    void run();
    void drain(uint64_t now_us);
    bool accept(const MotionIntent& in, uint64_t now_us);   // gates, clamp, commit
    void evaluate(uint64_t now_us, float dt_s);
    void refreshSnapshot(uint64_t now_us);
    void drainAnomalies();

    float positionMm() const { return float(lpSteps() - _lp_origin) * kMmPerStep; }
    float span() const { return _win_max - _win_min; }
    float toNorm(float mm) const { return (mm - _win_min) / span(); }
    float toMm(float norm) const { return _win_min + norm * span(); }

    // The engine is a member so it lands in this object's storage, which is a
    // file-scope static in internal RAM. Never move it to PSRAM.
    kinetic::Engine _engine{};

    TaskHandle_t  _task = nullptr;
    QueueHandle_t _queue = nullptr;

    float _win_min = 0.0f;
    float _win_max = DEFAULT_MAX_RAIL_MM;
    float _rail    = DEFAULT_MAX_RAIL_MM;

    float _user_v = DEFAULT_USER_MAX_SPEED_MM_S;
    float _user_a = DEFAULT_USER_ACCEL_MM_S2;
    float _in_v   = DEFAULT_MAX_SPEED_MM_S;
    float _in_a   = DEFAULT_ACCEL_MM_S2;
    float _in_j   = DEFAULT_INPUT_MAX_JERK_MM_S3;

    int32_t _lp_origin = 0;      // the LP count that means 0.0 mm
    float   _p_cmd_mm  = 0.0f;   // the plan position at the previous tick

    volatile bool _homed  = false;
    volatile bool _estop  = false;
    volatile bool _paused = false;
    bool _estop_settled = false;  // the engine has been reset since the latch

    // Odometer state, motion task only.
    int32_t _odo_steps = 0;       // LP count at the previous refresh
    int32_t _stroke_dir = 0;      // sign of the run in progress
    float   _stroke_run_mm = 0.0f;

    // Counters the motion task owns. They live OUTSIDE _pub so nothing writes
    // the snapshot except refreshSnapshot(), under the lock: a field updated
    // in place would be read half-torn by census() on the hub task.
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

    // Stream ingress, written by the hub task through noteStream(). Relaxed
    // atomics: they are counters nothing orders against, and making them part
    // of the snapshot would need the hub to take the motion lock on the path
    // that decodes a bundle.
    std::atomic<uint32_t> _sync_bundles{0};
    std::atomic<uint32_t> _sync_samples{0};
    std::atomic<uint32_t> _sync_dropped{0};

    // THE cross-task snapshot. Written by refreshSnapshot() on the motion
    // task, read by census() on any task, both under _mux.
    mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    MotionCensus _pub{};
};

MotionArbiter g_arb;

bool MotionArbiter::begin() {
    _queue = xQueueCreate(kIntentQueueDepth, sizeof(MotionIntent));
    if (_queue == nullptr) return false;
    steerLp(0.0f);                       // the emitter is PARKED until an intent lands
    _lp_origin = lpSteps();
    _odo_steps = lpSteps();
    _engine.resetAt(toNorm(0.0f), static_cast<uint64_t>(esp_timer_get_time()));
    _p_cmd_mm = 0.0f;
    refreshSnapshot(static_cast<uint64_t>(esp_timer_get_time()));
    // Core 1 with the hub, at a higher priority than it: the tick is a
    // polynomial evaluation and two 32-bit stores, and commit() runs only when
    // an intent arrives. Core 0 keeps app_main and the esp_hosted SDIO service.
    // RAISED 16,384 -> 24,576 ON A MEASUREMENT, which is the direction T21
    // permits: under the val-091.11 stream proof -- a 0.8 Hz sine at 50 Hz,
    // 499 accepted intents in 10 s, every one a commit() nesting KB-scale
    // Ruckig temporaries -- the deepest free was 1,296 B of 16,384, 8 %
    // headroom [verified 2026-09-21 -- census().stack_free over COM15]. The
    // idle mark before that run read 12,316 B, which is exactly why an idle
    // high-water mark is not a sizing number.
    const BaseType_t ok = xTaskCreatePinnedToCore(&MotionArbiter::taskTrampoline, "Motion",
                                                  kMotionTaskStackBytes, this, 6, &_task, 1);
    if (ok != pdPASS) return false;
    GLOGI(kTag, "motion path up: window %.1f..%.1f mm, rail %.1f mm, %.3f steps/mm, %lu us tick",
          double(_win_min), double(_win_max), double(_rail), double(kStepsPerMm),
          static_cast<unsigned long>(kTickUs));
    return true;
}

bool MotionArbiter::submit(const MotionIntent& in) {
    if (_queue == nullptr) return false;
    if (xQueueSend(_queue, &in, 0) != pdTRUE) {
        GLOGW_EVERY_MS(1000, kTag, "DROP: intent queue full");
        return false;
    }
    // On arrival, never on a tick: the task is woken now and plans now.
    if (_task != nullptr) xTaskNotifyGive(_task);
    return true;
}

void MotionArbiter::estop(bool on) {
    _estop = on;
    if (!on) {
        _estop_settled = false;
        return;
    }
    // Park on the CALLING task. E-stop that waits for a tick is not an e-stop,
    // and the store is a single 32-bit word with one writer either way. The
    // ENGINE is not touched here: it belongs to the motion task, which resets
    // it on the next tick (see evaluate()).
    ulp_g_step_q8 = 0;
    _homed = false;      // an abandoned plan leaves the carriage where it fell
    GLOGW(kTag, "ESTOP: emitter parked at %.3f mm", double(positionMm()));
}

void MotionArbiter::setWindow(float lo, float hi, float rail) {
    if (!(std::isfinite(lo) && std::isfinite(hi) && std::isfinite(rail))) return;
    if (!(hi > lo)) return;
    _win_min = lo;
    _win_max = hi;
    _rail    = rail > 0.0f ? rail : DEFAULT_MAX_RAIL_MM;
}

float MotionArbiter::forceHome(float stroke_mm) {
    // Written so a NaN takes the default: !(x >= 1), not (x < 1).
    float stroke = stroke_mm;
    if (!(stroke >= 1.0f)) stroke = 250.0f;
    if (stroke > _rail) stroke = _rail;
    _lp_origin = lpSteps();
    _rail      = stroke;
    _estop     = false;
    _estop_settled = false;
    _homed     = true;
    // The engine reseeds itself at rest on the next accepted intent; nothing
    // here may call into it, this runs on the hub task.
    GLOGW(kTag, "FORCE HOME: homed asserted at 0.0 mm, stroke %.1f mm, e-stop cleared "
                "-- no homing cycle ran (RFC-025)", double(stroke));
    return stroke;
}

void MotionArbiter::noteStream(uint32_t bundles, uint32_t samples, uint32_t dropped) {
    _sync_bundles.fetch_add(bundles, std::memory_order_relaxed);
    _sync_samples.fetch_add(samples, std::memory_order_relaxed);
    _sync_dropped.fetch_add(dropped, std::memory_order_relaxed);
}

// ---- gates ------------------------------------------------------------------

bool MotionArbiter::accept(const MotionIntent& in, uint64_t now_us) {
    // E-stop is the one gate no source bypasses.
    if (_estop) {
        ++_rejected;
        GLOGW_EVERY_MS(1000, kTag, "REJECT: e-stop");
        return false;
    }
    // Manual bypasses the rest (the push-to-home case): an operator must be
    // able to move an unhomed or paused machine, and only to move it.
    if (in.source != MotionSource::Manual) {
        if (!_homed) {
            ++_rejected;
            GLOGW_EVERY_MS(1000, kTag, "REJECT: not homed");
            return false;
        }
        if (_paused) {
            ++_rejected;
            GLOGW_EVERY_MS(1000, kTag, "REJECT: paused");
            return false;
        }
    }

    const bool manual = in.source == MotionSource::Manual;

    // Window clamp. Manual reaches the whole asserted rail; a machine-driven
    // source is held inside the configured window, itself held inside the rail.
    float target = in.target_mm;
    const float lo = manual ? 0.0f : (_win_min > 0.0f ? _win_min : 0.0f);
    const float hi_win = _win_max < _rail ? _win_max : _rail;
    const float hi = manual ? _rail : hi_win;
    if (target < lo) target = lo;
    if (target > hi) target = hi;
    // THROTTLED, because a stream that overhangs the window clamps EVERY
    // sample: at 50 Hz the unthrottled line is the log, and a log that floods
    // on a normal condition is a log nobody reads (T27's shape in the logging
    // dimension).
    if (target != in.target_mm)
        GLOGW_EVERY_MS(1000, kTag, "WINDOW CLAMP: %.2f -> %.2f mm",
                       double(in.target_mm), double(target));

    // Limit-set selection. Ceilings are clamps, never targets; a deadline-less
    // Manual point move is the ratified exception and plans AT the user set.
    // NOT IMPLEMENTED IN v0: the soft-start cap, which shapes the INPUT set
    // only and has no source to shape here (see bd val-091.4).
    const float sp = manual ? _user_v : _in_v;
    const float ac = manual ? _user_a : _in_a;
    const float s  = span();
    kinetic::Limits lim;
    lim.vmax = sp / s;
    lim.amax = ac / s;
    lim.jmax = _in_j / s;
    _engine.setLimits(lim);

    // Plan from the machine's ACTUAL state. At rest that state is the LP core's
    // count and nothing else: an engine that re-seeds from its own idea of
    // where it stopped carries every move's sub-step residue into the next one.
    // Mid-plan the engine's own (p, v, a) IS the continuous state, and reseeding
    // there would be a discontinuity, so the door is rest only.
    if (!_engine.isBusy(now_us)) {
        _engine.resetAt(toNorm(positionMm()), now_us);
        _p_cmd_mm = positionMm();
    }

    kinetic::Command cmd;
    cmd.target       = toNorm(target);
    cmd.has_duration = in.duration_us > 0;
    cmd.duration_us  = in.duration_us;
    cmd.has_end_vel  = in.has_end_vel;
    cmd.end_vel      = in.end_vel_mm_s / s;
    cmd.client_curve_family = in.curve_family;
    // An anchor already in the past is not a schedule, it is arrival: passing
    // it through would spend a schedule slot to say "now".
    cmd.has_anchor   = in.anchor_us > now_us;
    cmd.anchor_us    = in.anchor_us;
    const int64_t t0 = esp_timer_get_time();
    const bool ok = _engine.commit(cmd, now_us);
    const uint32_t plan_us = uint32_t(esp_timer_get_time() - t0);
    _plan_us_last = plan_us;
    if (plan_us > _plan_us_max) _plan_us_max = plan_us;
    _plan_us_avg += (float(plan_us) - _plan_us_avg) * 0.125f;
    if (!ok) {
        ++_rejected;
        GLOGW_EVERY_MS(1000, kTag, "REJECT: plan failed for %.2f mm", double(target));
        return false;
    }
    ++_intents;
    _demand_mm = target;
    _stream    = !manual;
    return true;
}

// ---- the tick ---------------------------------------------------------------

void MotionArbiter::drain(uint64_t now_us) {
    MotionIntent in;
    while (xQueueReceive(_queue, &in, 0) == pdTRUE) accept(in, now_us);
}

void MotionArbiter::evaluate(uint64_t now_us, float dt_s) {
    if (_estop) {
        ulp_g_step_q8 = 0;
        // ONCE per latch, and on the task that owns the engine: without it the
        // abandoned plan keeps reading busy and canClearEstop() -- which asks
        // exactly that -- would never let the latch drop (SPEC 11.2).
        if (!_estop_settled) {
            _engine.resetAt(toNorm(positionMm()), now_us);
            _p_cmd_mm = positionMm();
            _estop_settled = true;
        }
        return;
    }

    // The one side-effecting sample per tick: it promotes scheduled plans and
    // engages SETTLE when a plan ends still moving.
    const float p_plan_mm = toMm(_engine.positionAt(now_us));

    // Feedforward: the plan's OWN mean velocity across the interval that just
    // elapsed. Summed over a move this telescopes to exactly the plan's
    // displacement, which is why the emitter needs no other position input.
    float v = (p_plan_mm - _p_cmd_mm) / dt_s;
    _p_cmd_mm = p_plan_mm;

    // Residual against POSITION TRUTH, deadbanded at one step. It exists for
    // the ramp out of rest: the opening velocity is small, so the first edge
    // period is long, and the emitter cannot render the plan's first steps on
    // time however often it is re-steered.
    const float err_mm = p_plan_mm - positionMm();
    if (std::fabs(err_mm) > kMmPerStep) v += err_mm * kTrackHz;

    steerLp(v);
}

// Drains the engine's anomaly ring into the per-kind table 0x1111 publishes.
// Motion task only. A kind past the table is DROPPED rather than folded into
// a neighbor: a miscounted kind reads as a diagnosis that never happened.
void MotionArbiter::drainAnomalies() {
    kinetic::Anomaly a;
    while (_engine.popAnomaly(a)) {
        if (a.kind < kAnomalyKinds) ++_anom[a.kind];
        ++_anomalies;
    }
}

void MotionArbiter::refreshSnapshot(uint64_t now_us) {
    const kinetic::Snapshot s = _engine.snapshot(now_us);
    const float s_mm = span();

    const int32_t steps = lpSteps();
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

    const float vel_mm_s = s.vel * s_mm;
    if (std::fabs(vel_mm_s) > _peak_mm_s) _peak_mm_s = std::fabs(vel_mm_s);

    MotionCensus c{};
    c.steps          = steps - _lp_origin;
    c.position_mm    = float(c.steps) * kMmPerStep;
    c.plan_mm        = toMm(s.pos);
    c.target_mm      = toMm(s.target);
    c.velocity_mm_s  = vel_mm_s;
    c.residual_steps = static_cast<int32_t>(std::lround((c.plan_mm - c.position_mm) * kStepsPerMm));
    c.win_min        = _win_min;
    c.win_max        = _win_max;
    c.rail_mm        = _rail;
    c.edges          = ulp_g_edges;
    c.late           = ulp_g_late;
    c.resteers       = ulp_g_resteer;
    c.catchups       = ulp_g_catchup;
    c.step_q8        = ulp_g_step_q8;
    c.emitter_faults = g_emitter_faults;
    c.intents        = _intents;
    c.rejected       = _rejected;
    // IDF reports this in BYTES, not the vanilla FreeRTOS words.
    c.stack_free     = _task ? uxTaskGetStackHighWaterMark(_task) : 0;
    c.homed          = _homed;
    c.estop          = _estop;
    c.paused         = _paused;
    c.busy           = _engine.isBusy(now_us);
    c.mode           = s.mode;
    c.plan_kind      = s.plan_kind;
    c.plan_start     = s.start;
    c.plan_end       = s.target;
    c.plan_cur       = s.pos;
    c.plan_vel       = s.vel;
    c.plan_duration_us = uint32_t(s.duration_s * 1e6f);
    c.plan_elapsed_us  = uint32_t(s.elapsed_s * 1e6f);
    c.plans          = s.plans;
    c.failures       = s.failures;
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

    portENTER_CRITICAL(&_mux);
    _pub = c;
    portEXIT_CRITICAL(&_mux);
}

void MotionArbiter::run() {
    uint64_t prev_us = static_cast<uint64_t>(esp_timer_get_time());
    uint64_t next_pub_us = prev_us;
    for (;;) {
        // Wakes on an intent OR on the tick, whichever comes first. dt is
        // MEASURED, so an early wake costs nothing and an intent never waits
        // out the tick to be planned.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kTickUs / 1000));
        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
        drain(now_us);
        const float dt_s = float(now_us - prev_us) * 1e-6f;
        if (dt_s <= 0.0f) continue;
        prev_us = now_us;
        evaluate(now_us, dt_s);
        if (now_us >= next_pub_us) {
            next_pub_us = now_us + kPublishUs;
            drainAnomalies();
            refreshSnapshot(now_us);
        }
    }
}

MotionCensus MotionArbiter::census() const {
    portENTER_CRITICAL(&_mux);
    const MotionCensus c = _pub;
    portEXIT_CRITICAL(&_mux);
    return c;
}

}  // namespace

// ---- the public door --------------------------------------------------------

bool motionBegin() { return g_arb.begin(); }
bool motionSubmit(const MotionIntent& in) { return g_arb.submit(in); }
void motionEstop() { g_arb.estop(true); }
void motionEstopClear() { g_arb.estop(false); }
void motionPause(bool on) { g_arb.pause(on); }
void motionSetUserLimits(float v, float a) { g_arb.setUserLimits(v, a); }
void motionSetInputLimits(float v, float a, float j) { g_arb.setInputLimits(v, a, j); }
void motionSetWindow(float lo, float hi, float rail) { g_arb.setWindow(lo, hi, rail); }
void motionNoteStream(uint32_t b, uint32_t s, uint32_t d) { g_arb.noteStream(b, s, d); }
float motionForceHome(float stroke_mm) { return g_arb.forceHome(stroke_mm); }
MotionCensus motionCensus() { return g_arb.census(); }

// The engine's defaults ARE its live values: nothing on this board writes the
// tuning (bd val-091.11), so a default-constructed Config is exactly what the
// engine holds and reading it needs no cross-task access.
MotionTuning motionTuning() {
    const kinetic::Config cfg{};
    MotionTuning t;
    t.chase_ff         = cfg.chase_feedforward;
    t.chase_accel_ff   = cfg.chase_accel_ff;
    t.chase_gain       = cfg.chase_ff_gain;
    t.chase_lookahead  = cfg.chase_lookahead;
    t.chase_dense_us   = cfg.chase_dense_us;
    t.chase_aim_extrap = cfg.chase_aim_accel_extrap;
    t.handoff_k        = cfg.handoff_chord_factor;
    t.curve_policy     = uint8_t(cfg.curve_policy);
    // The catalog select is 0 stretch / 1 blend; the engine's own ordinals are
    // pinned at 0 and 5 by what is already persisted elsewhere in the
    // ecosystem, so the mapping is explicit rather than a cast.
    t.infeasible_policy = cfg.infeasible_policy == kinetic::InfeasiblePolicy::Stretch ? 0 : 1;
    t.smooth_budget    = cfg.infeasible_smooth_budget;
    t.amplitude_budget = cfg.infeasible_amplitude_budget;
    t.blend_steps      = cfg.infeasible_blend_steps;
    t.settle_grace_us  = cfg.settle_grace_us;
    return t;
}

}  // namespace valence
