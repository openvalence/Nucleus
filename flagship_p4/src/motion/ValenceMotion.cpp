// ValenceMotion -- the arbiter's board host: the planner and steer tasks, the
// queues, the census and strip locks, and the LP core's steering words
// Constraints:
// - EVERY GATE LIVES IN MotionArbiter (hardware-free, shared with the host
//   twin). Nothing here decides whether an intent moves the machine; a gate
//   written here is a gate the sim does not run.
// - ONE DOOR OUT: LpEmitter is the only writer of the emitter's shared words,
//   it is private to this file, and only the arbiter calls it. That is the
//   MotionArbiter sole-caller rule in code (architecture.md section 2).
// - The engine (inside the arbiter) and both task stacks live in INTERNAL
//   RAM, never PSRAM: the HP side is unreachable while the flash cache is off,
//   and neither task may fault during an OTA write. Only the LP core is immune.
// - TWO TASKS ON CORE 1 (bd val-8rt). The planner ("Planner", priority 5, the
//   hub's) drains the queues, plans, solves and publishes the strip and the
//   census. The steer ("Motion", priority 6) reads the strip and steers every
//   tick, so no solve delays a steer. The steer's priority over the planner
//   on one core is what keeps the two producers apart (MotionArbiter.h):
//   never move either task to core 0, which keeps app_main and esp_hosted.
// - The window solve puts KB-scale temporaries on the CALLING stack (a copy of
//   the pending knots and the solver's fixed arrays), so the engine is sampled
//   on the planner and nowhere else (T1, memory-budget.md T21).
// - THE ENGINE IS TOUCHED BY THE PLANNER ONLY. The steer reads the plan as the
//   arbiter's strip under g_strip_mux, and the hub task's persist gate scans
//   it there (motionStillFor()). Every other cross-task reader goes
//   through _pub, a plain POD the planner refreshes under _mux; census()
//   copies it under the same lock and calls nothing.
// See: ValenceMotion.h, MotionArbiter.h, .claude/rules/motion-control.md,
// bd val-091.4

#include "ValenceMotion.h"

#include <atomic>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "MotionArbiter.h"
#include "geiger/geiger.h"
#include "system/ValenceAccessoryIo.h"
#include "system/ValenceHomeSense.h"
#include "system/ValenceMotorSwitch.h"
#include "ulp_main.h"

namespace valence {
namespace {

constexpr const char* kTag = "motion";

// ---- task constants ---------------------------------------------------------

// How often the cross-task snapshot is refreshed. 50 Hz feeds a 60 Hz 0x1100
// and a 45 Hz 0x1110 with one census build and anomaly drain per refresh
// instead of one per tick, which is the whole reason it is not simply done at
// kMotionTickUs: an instrument billed at the tick rate is the class that
// manufactures the fault it observes (memory-budget.md T27). The snapshot
// reads the plan planTick() sampled; it never samples the engine.
constexpr uint32_t kPublishUs = 20000;

// ---- the emitter's one door -------------------------------------------------

uint64_t espNowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }

// The LP core's shared words. steer() is the only writer of ulp_g_dir and the
// arbiter its only caller; park() is one 32-bit store with one writer either
// way. park() and count() run in the home sense's interrupt too
// (MotionArbiter::homeSenseRose()): one aligned store and one aligned load in
// LP memory, nothing else, so neither may grow a lock, a log or a counter.
// That interrupt is not IRAM-resident: it waits out a flash write.
// steer(), fence() and renew() run on the steer task, or on the planner while
// a home cycle runs, never on both in one tick (MotionArbiter::steerTick()):
// one writer at a time for the direction, fence and lease words.
class LpEmitter final : public MotionEmitter {
public:
    int32_t count() const override { return static_cast<int32_t>(ulp_g_pos); }

    void steer(float v_mm_s) override {
        const SteerWord w = steerWord(v_mm_s);
        if (w.step_q8 == 0) {
            ulp_g_step_q8 = 0;
            return;
        }
        if (w.floored) ++_faults;
        // DIRECTION FIRST, and this ordering is load-bearing. The LP core reads
        // the two words independently, so an interleaving is possible; writing
        // dir first makes the only reachable one "one edge at the OLD period in
        // the NEW direction", a timing slip of at most one period. Writing step
        // first instead allows one edge in the WRONG direction, which is a
        // permanent two-step position error nothing detects.
        ulp_g_dir     = w.forward ? 1u : 0u;
        ulp_g_step_q8 = w.step_q8;
    }

    void park() override { ulp_g_step_q8 = 0; }

    void fence(int32_t lo, int32_t hi) override {
        // THE NARROWING BOUND FIRST, a memory fence after each store. The LP
        // core reads one word per edge (hi forward, lo reverse), so between
        // the stores it sees one old bound and one new one; storing the bound
        // that moves inward first keeps that pair inside the old fence or the
        // new one, and makes it their intersection whenever one bound
        // narrows while the other widens (a shift). Never outside their union.
        if (lo > static_cast<int32_t>(ulp_g_fence_lo)) {
            ulp_g_fence_lo = static_cast<uint32_t>(lo);
            std::atomic_thread_fence(std::memory_order_seq_cst);
            ulp_g_fence_hi = static_cast<uint32_t>(hi);
        } else {
            ulp_g_fence_hi = static_cast<uint32_t>(hi);
            std::atomic_thread_fence(std::memory_order_seq_cst);
            ulp_g_fence_lo = static_cast<uint32_t>(lo);
        }
        // Before the steer that follows.
        std::atomic_thread_fence(std::memory_order_seq_cst);
    }

    // Any change renews; read back, so main.cpp's boot burst value is moved.
    void renew() override { ulp_g_lease = ulp_g_lease + 1u; }

    uint32_t lapses() const override { return ulp_g_lapses; }

    uint32_t faults() const { return _faults; }

private:
    uint32_t _faults = 0;   // steer() is its one writer, on whichever task steers
};

// The strip's lock (MotionArbiter::Lock): held for one PlanStrip copy on either
// motion task, or one stillFor() scan on the hub task, never anything else.
portMUX_TYPE g_strip_mux = portMUX_INITIALIZER_UNLOCKED;
void stripLock(bool hold) {
    if (hold) portENTER_CRITICAL(&g_strip_mux);
    else portEXIT_CRITICAL(&g_strip_mux);
}

static_assert(configTICK_RATE_HZ >= 1000000 / kMotionTickUs, "a one-tick wait must be at most kMotionTickUs");

// ---- the motion tasks -------------------------------------------------------
// The arbiter's host: the queues in front of it, the two tasks that own it,
// and the locks the census and the strip cross tasks under. Every gate lives
// in MotionArbiter.

class MotionTask {
public:
    bool begin();
    bool submit(const MotionIntent& in);     // any task: enqueue and wake
    void pause(bool on);                     // any task: gate, brake request, wake
    void override();                         // any task: pause, then the mode, wake
    ReturnStart returnToPause();             // any task: request, wake
    HomeStart home();                        // any task: request, wake
    void setTuning(const MotionTuning& t);   // any task: overwrite the one slot
    MotionCensus census() const;
    MotionArbiter& arbiter() { return _arb; }
    void wakeFromIsr();                      // the home sense's rise
    uint32_t steerStackFree() const { return _steer ? uint32_t(uxTaskGetStackHighWaterMark(_steer)) : 0; }

private:
    static void planTrampoline(void* self) { static_cast<MotionTask*>(self)->planRun(); }
    static void steerTrampoline(void* self) { static_cast<MotionTask*>(self)->steerRun(); }
    void planRun();
    void steerRun();
    void drain(uint64_t now_us);
    void refreshSnapshot(uint64_t now_us);

    // Declaration order is construction order: the arbiter binds the emitter.
    // Both are members so they land in this object's storage, which is a
    // file-scope static in internal RAM. Never move it to PSRAM.
    LpEmitter     _emitter;
    MotionArbiter _arb{_emitter, &espNowUs, &stripLock};

    // Every wake (an intent, a request, the home sense) is the planner's; the
    // steer only runs on the tick.
    TaskHandle_t  _planner = nullptr;
    TaskHandle_t  _steer = nullptr;
    QueueHandle_t _queue = nullptr;
    // Depth ONE, written with xQueueOverwrite: the newest tuning set is the
    // only one worth applying, and the writer never waits on the planner.
    QueueHandle_t _tuneQueue = nullptr;

    // THE cross-task snapshot. Written by refreshSnapshot() on the planner,
    // read by census() on any task, both under _mux.
    mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    MotionCensus _pub{};
};

MotionTask g_motion;

// The home sense's interrupt, in this order: the arbiter parks an armed seek,
// then the planner is woken to read it: the seek producer, homeStep(), runs
// there, and the steer has nothing to do with a rise.
void parkSeekFromIsr() { g_motion.arbiter().homeSenseRose(); }
void wakeMotionFromIsr() { g_motion.wakeFromIsr(); }

// Before begin() has created the task the handle is null and nothing wakes.
void MotionTask::wakeFromIsr() {
    BaseType_t woken = pdFALSE;
    if (_planner != nullptr) vTaskNotifyGiveFromISR(_planner, &woken);
    portYIELD_FROM_ISR(woken);
}

bool MotionTask::begin() {
    _queue = xQueueCreate(kIntentQueueDepth, sizeof(MotionIntent));
    if (_queue == nullptr) return false;
    _tuneQueue = xQueueCreate(1, sizeof(MotionTuning));
    if (_tuneQueue == nullptr) return false;
    // Neither task exists yet, so this caller is the arbiter's one owner.
    _arb.setHomeSense(homeSenseBegin(&parkSeekFromIsr, &wakeMotionFromIsr));
    _arb.begin(espNowUs());
    refreshSnapshot(espNowUs());
    // Both on core 1 with the hub; core 0 keeps app_main and the esp_hosted
    // SDIO service. The planner AT the hub's priority, never above it: a
    // window solve time-slices with the hub instead of starving it. The steer
    // above both: a strip copy, an interpolation and two 32-bit stores per
    // tick. Each stack is kMotionTaskStackBytes, the planner's carrying the
    // solve; neither is measured under Kinetic² (bd val-4q1): size them only
    // on a high-water mark taken under a real motion workload, never an idle
    // one.
    if (xTaskCreatePinnedToCore(&MotionTask::planTrampoline, "Planner", kMotionTaskStackBytes, this, 5, &_planner, 1) !=
        pdPASS)
        return false;
    if (xTaskCreatePinnedToCore(&MotionTask::steerTrampoline, "Motion", kMotionTaskStackBytes, this, 6, &_steer, 1) !=
        pdPASS)
        return false;
    GLOGI(kTag, "motion path up: window %.1f..%.1f mm, rail %.1f mm, %.3f steps/mm, %lu us tick",
          double(_arb.winMin()), double(_arb.winMax()), double(_arb.rail()), double(kStepsPerMm),
          static_cast<unsigned long>(kMotionTickUs));
    return true;
}

bool MotionTask::submit(const MotionIntent& in) {
    if (_queue == nullptr) return false;
    if (xQueueSend(_queue, &in, 0) != pdTRUE) {
        GLOGW_EVERY_MS(1000, kTag, "DROP: intent queue full");
        return false;
    }
    // On arrival, never on a tick: the planner is woken now and plans now.
    if (_planner != nullptr) xTaskNotifyGive(_planner);
    return true;
}

void MotionTask::pause(bool on) {
    _arb.pause(on);
    // Brakes at arrival, not on the tick.
    if (on && _planner != nullptr) xTaskNotifyGive(_planner);
}

void MotionTask::override() {
    _arb.override();
    if (_planner != nullptr) xTaskNotifyGive(_planner);
}

ReturnStart MotionTask::returnToPause() {
    const ReturnStart r = _arb.returnToPause();
    if (r == ReturnStart::queued && _planner != nullptr) xTaskNotifyGive(_planner);
    return r;
}

HomeStart MotionTask::home() {
    const HomeStart r = _arb.home();
    if (r == HomeStart::started && _planner != nullptr) xTaskNotifyGive(_planner);
    return r;
}

void MotionTask::setTuning(const MotionTuning& t) {
    if (_tuneQueue == nullptr) return;
    xQueueOverwrite(_tuneQueue, &t);
    if (_planner != nullptr) xTaskNotifyGive(_planner);
}

void MotionTask::drain(uint64_t now_us) {
    // Tuning BEFORE intents: an intent that arrived after a tuning write plans
    // under it.
    MotionTuning t;
    if (xQueueReceive(_tuneQueue, &t, 0) == pdTRUE) _arb.applyTuning(t);
    MotionIntent in;
    while (xQueueReceive(_queue, &in, 0) == pdTRUE) _arb.accept(in, now_us);
}

void MotionTask::refreshSnapshot(uint64_t now_us) {
    MotionCensus c = _arb.snapshot(now_us);
    c.edges          = ulp_g_edges;
    c.late           = ulp_g_late;
    c.resteers       = ulp_g_resteer;
    c.catchups       = ulp_g_catchup;
    c.step_q8        = ulp_g_step_q8;
    c.emitter_faults = _emitter.faults();
    c.fence_hits     = ulp_g_fence_hits;
    // The planner's: the stack the solve runs on. IDF reports this in BYTES,
    // not the vanilla FreeRTOS words.
    c.stack_free     = _planner ? uxTaskGetStackHighWaterMark(_planner) : 0;

    portENTER_CRITICAL(&_mux);
    _pub = c;
    portEXIT_CRITICAL(&_mux);
}

void MotionTask::planRun() {
    uint64_t prev_us = espNowUs();
    uint64_t next_pub_us = prev_us;
    for (;;) {
        // Wakes on an intent, a request or the home sense, OR on the tick,
        // whichever comes first. dt is MEASURED, so an early wake costs
        // nothing and an intent never waits out the tick to be planned.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kMotionTickUs / 1000));
        const uint64_t now_us = espNowUs();
        drain(now_us);
        const float dt_s = float(now_us - prev_us) * 1e-6f;
        if (dt_s <= 0.0f) continue;
        prev_us = now_us;
        _arb.planTick(now_us, dt_s);
        if (now_us >= next_pub_us) {
            next_pub_us = now_us + kPublishUs;
            _arb.drainAnomalies();
            // The steer's counters and _backstop_on are read as single
            // words, at most one steer old.
            refreshSnapshot(now_us);
        }
    }
}

void MotionTask::steerRun() {
    uint64_t prev_us = espNowUs();
    for (;;) {
        // Nothing wakes it early: one steer per tick, dt measured.
        vTaskDelay(pdMS_TO_TICKS(kMotionTickUs / 1000));
        const uint64_t now_us = espNowUs();
        const float dt_s = float(now_us - prev_us) * 1e-6f;
        if (dt_s <= 0.0f) continue;
        prev_us = now_us;
        _arb.steerTick(now_us, dt_s);
    }
}

MotionCensus MotionTask::census() const {
    portENTER_CRITICAL(&_mux);
    const MotionCensus c = _pub;
    portEXIT_CRITICAL(&_mux);
    return c;
}

}  // namespace

// ---- the public door --------------------------------------------------------

bool motionBegin() { return g_motion.begin(); }
bool motionSubmit(const MotionIntent& in) { return g_motion.submit(in); }
void motionEstop() {
    // Power first: the cut is the stop on this board, the park only keeps the
    // emitter from rendering into a dead drive. The accessory outputs come
    // last: atomic stores only, so they cannot delay either.
    motorSwitchCut();
    g_motion.arbiter().estop(true);
    accessoryIoEstop();
}
void motionEstopClear() { g_motion.arbiter().estop(false); }
void motionSetMotorPowered(bool on) { g_motion.arbiter().setMotorPowered(on); }
void motionSetCommissioned(bool on) { g_motion.arbiter().setCommissioned(on); }
void motionPause(bool on) { g_motion.pause(on); }
bool motionAcquireRail(MotionSource g) { return g_motion.arbiter().acquireRail(g); }
void motionReleaseRail(MotionSource g) { g_motion.arbiter().releaseRail(g); }
void motionSetEstopCutsPower(bool cuts) { g_motion.arbiter().setEstopCutsPower(cuts); }
void motionOverride() { g_motion.override(); }
ReturnStart motionReturn() { return g_motion.returnToPause(); }
HomeStart motionHome() { return g_motion.home(); }
void motionSetFlipped(bool on) { g_motion.arbiter().setFlipped(on); }
void motionSetJogLimits(float v, float a) { g_motion.arbiter().setJogLimits(v, a); }
void motionSetInputLimits(float v, float a, float j) { g_motion.arbiter().setInputLimits(v, a, j); }
void motionSetWindow(float lo, float hi, float rail) { g_motion.arbiter().setWindow(lo, hi, rail); }
void motionNoteStream(uint32_t b, uint32_t s, uint32_t d) { g_motion.arbiter().noteStream(b, s, d); }
float motionForceHome(float stroke_mm) { return g_motion.arbiter().forceHome(stroke_mm); }
MotionCensus motionCensus() { return g_motion.census(); }
bool motionStillFor(uint32_t window_us) { return g_motion.arbiter().stillFor(espNowUs(), window_us); }
uint32_t motionSteerStackFree() { return g_motion.steerStackFree(); }

void motionSetTuning(const MotionTuning& t) { g_motion.setTuning(t); }
void motionSetOscillator(const MotionOsc& o) { g_motion.arbiter().setOscillator(o); }

}  // namespace valence
