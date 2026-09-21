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
// - Millimeters on the public interface, normalized 0..1 window units inside
//   the engine. The two never mix in one expression.
// See: ValenceMotion.h, .claude/rules/motion-control.md, bd val-091.4

#include "ValenceMotion.h"

#include <cmath>
#include <cstdio>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "vlog/vlog.h"
#include "hub/valence_config.h"
#include "vmotion/vmotion.hpp"
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
    void assumeHomed(float at_mm);
    MotionCensus census();

private:
    static void taskTrampoline(void* self) { static_cast<MotionArbiter*>(self)->run(); }
    void run();
    void drain(uint64_t now_us);
    bool accept(const MotionIntent& in, uint64_t now_us);   // gates, clamp, commit
    void evaluate(uint64_t now_us, float dt_s);

    float positionMm() const { return float(lpSteps() - _lp_origin) * kMmPerStep; }
    float span() const { return _win_max - _win_min; }
    float toNorm(float mm) const { return (mm - _win_min) / span(); }
    float toMm(float norm) const { return _win_min + norm * span(); }

    // The engine is a member so it lands in this object's storage, which is a
    // file-scope static in internal RAM. Never move it to PSRAM.
    vmotion::Engine _engine{};

    TaskHandle_t  _task = nullptr;
    QueueHandle_t _queue = nullptr;

    float _win_min = 0.0f;
    float _win_max = DEFAULT_MAX_RAIL_MM;

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

    uint32_t _intents  = 0;
    uint32_t _rejected = 0;
    float    _plan_mm  = 0.0f;
};

MotionArbiter g_arb;

bool MotionArbiter::begin() {
    _queue = xQueueCreate(8, sizeof(MotionIntent));
    if (_queue == nullptr) return false;
    steerLp(0.0f);                       // the emitter is PARKED until an intent lands
    _lp_origin = lpSteps();
    _engine.resetAt(toNorm(0.0f), static_cast<uint64_t>(esp_timer_get_time()));
    _p_cmd_mm = 0.0f;
    // Core 1 with the hub, at a higher priority than it: the tick is a
    // polynomial evaluation and two 32-bit stores, and commit() runs only when
    // an intent arrives. Core 0 keeps app_main and the esp_hosted SDIO service.
    // 16 KB is SIZED, NOT MEASURED -- census().stack_free is the number that
    // decides whether it stays (memory-budget.md T21).
    const BaseType_t ok = xTaskCreatePinnedToCore(&MotionArbiter::taskTrampoline, "Motion",
                                                  16384, this, 6, &_task, 1);
    if (ok != pdPASS) return false;
    SLOGI(kTag, "motion path up: window %.1f..%.1f mm, %.3f steps/mm, %lu us tick",
          double(_win_min), double(_win_max), double(kStepsPerMm),
          static_cast<unsigned long>(kTickUs));
    return true;
}

bool MotionArbiter::submit(const MotionIntent& in) {
    if (_queue == nullptr) return false;
    if (xQueueSend(_queue, &in, 0) != pdTRUE) {
        SLOGW(kTag, "DROP: intent queue full");
        return false;
    }
    // On arrival, never on a tick: the task is woken now and plans now.
    if (_task != nullptr) xTaskNotifyGive(_task);
    return true;
}

void MotionArbiter::estop(bool on) {
    _estop = on;
    if (!on) return;
    // Park on the CALLING task. E-stop that waits for a tick is not an e-stop,
    // and the store is a single 32-bit word with one writer either way.
    ulp_g_step_q8 = 0;
    _homed = false;      // an abandoned plan leaves the carriage where it fell
    SLOGW(kTag, "ESTOP: emitter parked at %.3f mm", double(positionMm()));
}

void MotionArbiter::assumeHomed(float at_mm) {
    _lp_origin = lpSteps() - static_cast<int32_t>(at_mm * kStepsPerMm);
    _engine.resetAt(toNorm(at_mm), static_cast<uint64_t>(esp_timer_get_time()));
    _p_cmd_mm = at_mm;
    _homed    = true;
}

// ---- gates ------------------------------------------------------------------

bool MotionArbiter::accept(const MotionIntent& in, uint64_t now_us) {
    // E-stop is the one gate no source bypasses.
    if (_estop) {
        ++_rejected;
        SLOGW(kTag, "REJECT: e-stop");
        return false;
    }
    // Manual bypasses the rest (the push-to-home case): an operator must be
    // able to move an unhomed or paused machine, and only to move it.
    if (in.source != MotionSource::Manual) {
        if (!_homed) {
            ++_rejected;
            SLOGW(kTag, "REJECT: not homed");
            return false;
        }
        if (_paused) {
            ++_rejected;
            SLOGW(kTag, "REJECT: paused");
            return false;
        }
    }

    const bool manual = in.source == MotionSource::Manual;

    // Window clamp. Manual reaches the whole physical rail; a machine-driven
    // source is held inside the configured window.
    float target = in.target_mm;
    const float lo = manual ? 0.0f : _win_min;
    const float hi = manual ? DEFAULT_MAX_RAIL_MM : _win_max;
    if (target < lo) target = lo;
    if (target > hi) target = hi;
    if (target != in.target_mm)
        SLOGW(kTag, "WINDOW CLAMP: %.2f -> %.2f mm", double(in.target_mm), double(target));

    // Limit-set selection. Ceilings are clamps, never targets; a deadline-less
    // Manual point move is the ratified exception and plans AT the user set.
    // NOT IMPLEMENTED IN v0: the soft-start cap, which shapes the INPUT set
    // only and has no source to shape here (see bd val-091.4).
    const float sp = manual ? _user_v : _in_v;
    const float ac = manual ? _user_a : _in_a;
    const float s  = span();
    vmotion::Limits lim;
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

    vmotion::Command cmd;
    cmd.target       = toNorm(target);
    cmd.has_duration = in.duration_us > 0;
    cmd.duration_us  = in.duration_us;
    cmd.has_end_vel  = in.has_end_vel;
    cmd.end_vel      = in.end_vel_mm_s / s;
    if (!_engine.commit(cmd, now_us)) {
        ++_rejected;
        SLOGW(kTag, "REJECT: plan failed for %.2f mm", double(target));
        return false;
    }
    ++_intents;
    return true;
}

// ---- the tick ---------------------------------------------------------------

void MotionArbiter::drain(uint64_t now_us) {
    MotionIntent in;
    while (xQueueReceive(_queue, &in, 0) == pdTRUE) accept(in, now_us);
}

void MotionArbiter::evaluate(uint64_t now_us, float dt_s) {
    if (_estop) { ulp_g_step_q8 = 0; return; }

    // The one side-effecting sample per tick: it promotes scheduled plans and
    // engages SETTLE when a plan ends still moving.
    const float p_plan_mm = toMm(_engine.positionAt(now_us));
    _plan_mm = p_plan_mm;

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

void MotionArbiter::run() {
    uint64_t prev_us = static_cast<uint64_t>(esp_timer_get_time());
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
    }
}

MotionCensus MotionArbiter::census() {
    MotionCensus c;
    c.steps          = lpSteps() - _lp_origin;
    c.position_mm    = float(c.steps) * kMmPerStep;
    c.plan_mm        = _plan_mm;
    c.residual_steps = static_cast<int32_t>(std::lround((_plan_mm - c.position_mm) * kStepsPerMm));
    c.edges          = ulp_g_edges;
    c.late           = ulp_g_late;
    c.resteers       = ulp_g_resteer;
    c.catchups       = ulp_g_catchup;
    c.intents        = _intents;
    c.rejected       = _rejected;
    c.emitter_faults = g_emitter_faults;
    // IDF reports this in BYTES, not the vanilla FreeRTOS words.
    c.stack_free     = _task ? uxTaskGetStackHighWaterMark(_task) : 0;
    c.homed          = _homed;
    c.estop          = _estop;
    c.busy           = _engine.isBusy(static_cast<uint64_t>(esp_timer_get_time()));
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
MotionCensus motionCensus() { return g_arb.census(); }

#ifdef VALENCE_BENCH_MOTION

// ---- bench sequence ---------------------------------------------------------
// BENCH ONLY. No rail, no drive, no encoder is attached to this board: the
// proof is the scope on LPG15/LPG12 and the LP core's own edge count.

namespace {

constexpr float kBenchLegMm    = 20.0f;
constexpr float kBenchSpeedMmS = 20.0f;    // set as the USER CEILING, because a
                                           // deadline-less manual point move
                                           // plans at that set's ceilings
constexpr uint32_t kBenchSteps = 4172;     // 20 mm * 208.608 steps/mm, rounded

void benchReport(const char* leg, int32_t want_steps) {
    const MotionCensus c = motionCensus();
    printf("[bench] %-10s steps=%+ld want=%+ld err=%+ld  pos=%.4f mm plan=%.4f mm  "
           "late=%lu catchup=%lu resteer=%lu faults=%lu  stack_free=%lu\n",
           leg, static_cast<long>(c.steps), static_cast<long>(want_steps),
           static_cast<long>(c.steps - want_steps), double(c.position_mm),
           double(c.plan_mm), static_cast<unsigned long>(c.late),
           static_cast<unsigned long>(c.catchups),
           static_cast<unsigned long>(c.resteers),
           static_cast<unsigned long>(c.emitter_faults),
           static_cast<unsigned long>(c.stack_free));
}

void benchSubmit(MotionSource src, float mm) {
    MotionIntent in;
    in.source    = src;
    in.target_mm = mm;
    motionSubmit(in);
}

void benchTask(void*) {
    vTaskDelay(pdMS_TO_TICKS(10000));

    // The homed gate is REAL CODE and this proves it: a machine-driven intent
    // before the assume-homed call must be refused.
    const uint32_t rej0 = motionCensus().rejected;
    benchSubmit(MotionSource::Stream, kBenchLegMm);
    vTaskDelay(pdMS_TO_TICKS(50));
    printf("[bench] homed gate: rejected %lu -> %lu (expect +1)\n",
           static_cast<unsigned long>(rej0),
           static_cast<unsigned long>(motionCensus().rejected));

    motionSetUserLimits(kBenchSpeedMmS, DEFAULT_USER_ACCEL_MM_S2);
    motionBenchAssumeHomed();

    for (uint32_t cycle = 1;; ++cycle) {
        printf("\n[bench] ---- cycle %lu: %.0f mm at %.0f mm/s, accel %.0f mm/s2 ----\n",
               static_cast<unsigned long>(cycle), double(kBenchLegMm),
               double(kBenchSpeedMmS), double(DEFAULT_USER_ACCEL_MM_S2));

        benchSubmit(MotionSource::Manual, kBenchLegMm);
        vTaskDelay(pdMS_TO_TICKS(2500));
        benchReport("out", static_cast<int32_t>(kBenchSteps));

        benchSubmit(MotionSource::Manual, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(2500));
        benchReport("back", 0);

        // Reversal mid-move: the second intent lands while the first is still
        // running, so the engine replans from the live (p, v, a) and the
        // emitter turns around without a gap.
        benchSubmit(MotionSource::Manual, kBenchLegMm);
        vTaskDelay(pdMS_TO_TICKS(600));
        benchSubmit(MotionSource::Manual, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(3000));
        benchReport("reversal", 0);

        vTaskDelay(pdMS_TO_TICKS(6000));
    }
}

}  // namespace

void motionBenchAssumeHomed() {
    g_arb.assumeHomed(0.0f);
    SLOGW("motion", "BENCH: assumed homed at 0.0 mm, window 0..%.0f mm -- this board "
          "has no drive and no encoder and cannot home", double(DEFAULT_MAX_RAIL_MM));
}

void motionBenchStart() {
    xTaskCreatePinnedToCore(&benchTask, "MotionBench", 4096, nullptr, 3, nullptr, 1);
}

#endif  // VALENCE_BENCH_MOTION

}  // namespace valence
