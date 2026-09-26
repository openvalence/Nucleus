// ValenceMotion -- the arbiter's board host: the motion task, its queues, the
// census lock, and the LP core's steering words
// Constraints:
// - EVERY GATE LIVES IN MotionArbiter (hardware-free, shared with the host
//   twin). Nothing here decides whether an intent moves the machine; a gate
//   written here is a gate the sim does not run.
// - ONE DOOR OUT: LpEmitter is the only writer of the emitter's shared words,
//   it is private to this file, and only the arbiter calls it. That is the
//   MotionArbiter sole-caller rule in code (architecture.md section 2).
// - The engine (inside the arbiter) and the task stack live in INTERNAL RAM,
//   never PSRAM: the HP side is unreachable while the flash cache is off, and
//   the sampler must not fault during an OTA write. Only the LP core is immune.
// - commit() nests KB-scale Ruckig temporaries on the CALLING stack, so it
//   runs on the motion task and nowhere else (T1, memory-budget.md T21).
// - THE ENGINE IS TOUCHED BY THE MOTION TASK ONLY. Every cross-task reader
//   goes through _pub, a plain POD the tick refreshes under _mux; census()
//   copies it under the same lock and calls nothing.
// See: ValenceMotion.h, MotionArbiter.h, .claude/rules/motion-control.md,
// bd val-091.4

#include "ValenceMotion.h"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "MotionArbiter.h"
#include "geiger/geiger.h"
#include "ulp_main.h"

namespace valence {
namespace {

constexpr const char* kTag = "motion";

// ---- task constants ---------------------------------------------------------

// Sampler period. The S3 product evaluated its plan at 1 kHz and that number
// is kept deliberately: it is the cadence the whole engine was benched at, the
// LP core renders every edge between ticks regardless, and a faster tick buys
// nothing because the emitter re-reads its steering word mid-wait anyway.
constexpr uint32_t kTickUs = 1000;

// How often the cross-task snapshot is refreshed. 50 Hz feeds a 60 Hz 0x1100
// and a 45 Hz 0x1110 with one engine sample per refresh instead of one per
// tick, which is the whole reason it is not simply done at kTickUs: the
// snapshot calls Engine::snapshot(), and an instrument billed at the tick rate
// is the class that manufactures the fault it observes (memory-budget.md T27).
constexpr uint32_t kPublishUs = 20000;

// ---- the emitter's one door -------------------------------------------------

uint64_t espNowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }

// The LP core's shared words. steer() is the only writer of ulp_g_dir and the
// arbiter its only caller; park() is one 32-bit store with one writer either
// way.
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

    uint32_t faults() const { return _faults; }

private:
    uint32_t _faults = 0;   // motion task only: steer() is its one writer
};

// ---- the motion task --------------------------------------------------------
// The arbiter's host: the queues in front of it, the task that owns it, and
// the lock the census crosses tasks under. Every gate lives in MotionArbiter.

class MotionTask {
public:
    bool begin();
    bool submit(const MotionIntent& in);     // any task: enqueue and wake
    void stop();                             // any task: gate, brake request, wake
    void setTuning(const MotionTuning& t);   // any task: overwrite the one slot
    MotionCensus census() const;
    MotionArbiter& arbiter() { return _arb; }

private:
    static void taskTrampoline(void* self) { static_cast<MotionTask*>(self)->run(); }
    void run();
    void drain(uint64_t now_us);
    void refreshSnapshot(uint64_t now_us);

    // Declaration order is construction order: the arbiter binds the emitter.
    // Both are members so they land in this object's storage, which is a
    // file-scope static in internal RAM. Never move it to PSRAM.
    LpEmitter     _emitter;
    MotionArbiter _arb{_emitter, &espNowUs};

    TaskHandle_t  _task = nullptr;
    QueueHandle_t _queue = nullptr;
    // Depth ONE, written with xQueueOverwrite: the newest tuning set is the
    // only one worth applying, and the writer never waits on the motion task.
    QueueHandle_t _tuneQueue = nullptr;

    // THE cross-task snapshot. Written by refreshSnapshot() on the motion
    // task, read by census() on any task, both under _mux.
    mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    MotionCensus _pub{};
};

MotionTask g_motion;

bool MotionTask::begin() {
    _queue = xQueueCreate(kIntentQueueDepth, sizeof(MotionIntent));
    if (_queue == nullptr) return false;
    _tuneQueue = xQueueCreate(1, sizeof(MotionTuning));
    if (_tuneQueue == nullptr) return false;
    // The task does not exist yet, so this caller is the arbiter's one owner.
    _arb.begin(espNowUs());
    refreshSnapshot(espNowUs());
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
    const BaseType_t ok = xTaskCreatePinnedToCore(&MotionTask::taskTrampoline, "Motion",
                                                  kMotionTaskStackBytes, this, 6, &_task, 1);
    if (ok != pdPASS) return false;
    GLOGI(kTag, "motion path up: window %.1f..%.1f mm, rail %.1f mm, %.3f steps/mm, %lu us tick",
          double(_arb.winMin()), double(_arb.winMax()), double(_arb.rail()), double(kStepsPerMm),
          static_cast<unsigned long>(kTickUs));
    return true;
}

bool MotionTask::submit(const MotionIntent& in) {
    if (_queue == nullptr) return false;
    if (xQueueSend(_queue, &in, 0) != pdTRUE) {
        GLOGW_EVERY_MS(1000, kTag, "DROP: intent queue full");
        return false;
    }
    // On arrival, never on a tick: the task is woken now and plans now.
    if (_task != nullptr) xTaskNotifyGive(_task);
    return true;
}

void MotionTask::stop() {
    _arb.stop();
    // Brakes at arrival, not on the tick.
    if (_task != nullptr) xTaskNotifyGive(_task);
}

void MotionTask::setTuning(const MotionTuning& t) {
    if (_tuneQueue == nullptr) return;
    xQueueOverwrite(_tuneQueue, &t);
    if (_task != nullptr) xTaskNotifyGive(_task);
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
    // IDF reports this in BYTES, not the vanilla FreeRTOS words.
    c.stack_free     = _task ? uxTaskGetStackHighWaterMark(_task) : 0;

    portENTER_CRITICAL(&_mux);
    _pub = c;
    portEXIT_CRITICAL(&_mux);
}

void MotionTask::run() {
    uint64_t prev_us = espNowUs();
    uint64_t next_pub_us = prev_us;
    for (;;) {
        // Wakes on an intent OR on the tick, whichever comes first. dt is
        // MEASURED, so an early wake costs nothing and an intent never waits
        // out the tick to be planned.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kTickUs / 1000));
        const uint64_t now_us = espNowUs();
        drain(now_us);
        const float dt_s = float(now_us - prev_us) * 1e-6f;
        if (dt_s <= 0.0f) continue;
        prev_us = now_us;
        _arb.evaluate(now_us, dt_s);
        if (now_us >= next_pub_us) {
            next_pub_us = now_us + kPublishUs;
            _arb.drainAnomalies();
            refreshSnapshot(now_us);
        }
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
void motionEstop() { g_motion.arbiter().estop(true); }
void motionEstopClear() { g_motion.arbiter().estop(false); }
void motionStop() { g_motion.stop(); }
void motionPatternAllow() { g_motion.arbiter().allowPattern(); }
void motionStreamAllow() { g_motion.arbiter().allowStream(); }
void motionPause(bool on) { g_motion.arbiter().pause(on); }
void motionSetUserLimits(float v, float a) { g_motion.arbiter().setUserLimits(v, a); }
void motionSetInputLimits(float v, float a, float j) { g_motion.arbiter().setInputLimits(v, a, j); }
void motionSetWindow(float lo, float hi, float rail) { g_motion.arbiter().setWindow(lo, hi, rail); }
void motionNoteStream(uint32_t b, uint32_t s, uint32_t d) { g_motion.arbiter().noteStream(b, s, d); }
float motionForceHome(float stroke_mm) { return g_motion.arbiter().forceHome(stroke_mm); }
MotionCensus motionCensus() { return g_motion.census(); }

void motionSetTuning(const MotionTuning& t) { g_motion.setTuning(t); }

}  // namespace valence
