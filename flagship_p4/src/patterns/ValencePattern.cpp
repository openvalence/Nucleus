// ValencePattern -- the generators' board host: the pattern task, its settings
// slot, and the census read both gate on
// Constraints:
// - NO GENERATOR LOGIC LIVES HERE. Every scheduling, mapping and gating
//   decision is PatternEngine (hardware-free, shared with the host twin).
// - THE GENERATORS ARE TOUCHED BY THE PATTERN TASK ONLY. Settings cross in
//   through a depth-one queue written with xQueueOverwrite; the one value that
//   crosses out, active(), is a relaxed atomic bool nothing orders against.
// - Both generators tick on this one task, each submitting its own source's
//   intents; the arbiter keeps all but the rail holder's off it (RFC-093).
// - Core 1 at priority 4: below both motion tasks (the steer 6, the planner
//   5), which must never wait on a stroke, and below the hub (5), whose task
//   watchdog buys OTA rollback.
//   The work per wake is a few float operations and one queue send.
// - The generators and the task stack are INTERNAL RAM: the generators live
//   in a file-scope static and the stack is a plain xTaskCreatePinnedToCore
//   allocation.
// See: ValencePattern.h, PatternEngine.h, bd val-091.12

#include "ValencePattern.h"

#include <atomic>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "PatternEngine.h"
#include "geiger/geiger.h"
#include "motion/ValenceMotion.h"

namespace valence {
namespace {

constexpr const char* kTag = "pattern";

// The longest the task sleeps with nothing due. It bounds how late a gate
// change it only sees through the census (homed, a stream ending) is noticed;
// e-stop never waits on it, the arbiter parks on the calling task.
constexpr uint32_t kIdleWaitMs = 20;

class PatternTask {
public:
    bool begin();
    void setSettings(const PatternSettings& s) {
        if (_queue == nullptr) return;
        xQueueOverwrite(_queue, &s);
        if (_task != nullptr) xTaskNotifyGive(_task);
    }
    bool active() const { return _active.load(std::memory_order_relaxed); }
    uint32_t stackFree() const { return _task ? uint32_t(uxTaskGetStackHighWaterMark(_task)) : 0; }

private:
    static void taskTrampoline(void* self) { static_cast<PatternTask*>(self)->run(); }
    void run();
    void tickOne(PatternEngine& gen, uint64_t now_us, const PatternInputs& in);

    ClassicGenerator  _classic{};
    AdvancedGenerator _advanced{};
    TaskHandle_t  _task = nullptr;
    QueueHandle_t _queue = nullptr;
    std::atomic<bool> _active{false};
};

PatternTask g_pattern;

bool PatternTask::begin() {
    _queue = xQueueCreate(1, sizeof(PatternSettings));
    if (_queue == nullptr) return false;
    if (xTaskCreatePinnedToCore(&PatternTask::taskTrampoline, "Pattern", kPatternTaskStackBytes,
                                this, 4, &_task, 1) != pdPASS) {
        return false;
    }
    GLOGI(kTag, "generators up: classic (7 patterns) and advanced, stack %lu B",
          static_cast<unsigned long>(kPatternTaskStackBytes));
    return true;
}

void PatternTask::run() {
    uint32_t wait_ms = kIdleWaitMs;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms));
        PatternSettings s;
        if (xQueueReceive(_queue, &s, 0) == pdTRUE) {
            _classic.apply(s);
            _advanced.apply(s);
        }

        const uint64_t now_us = uint64_t(esp_timer_get_time());
        const MotionCensus c = motionCensus();
        PatternInputs in;
        in.homed         = c.homed;
        in.estop         = c.estop;
        in.paused        = c.paused;
        in.stream_active = c.stream;
        in.position_mm   = c.position_mm;
        in.velocity_mm_s = c.velocity_mm_s;
        in.vmax_mm_s     = c.input_vmax_mm_s;
        in.amax_mm_s2    = c.input_amax_mm_s2;
        in.jmax_mm_s3    = c.input_jmax_mm_s3;
        tickOne(_classic, now_us, in);
        tickOne(_advanced, now_us, in);
        _active.store(_classic.active() || _advanced.active(), std::memory_order_relaxed);

        // Sleep until the sooner generator's next half-stroke is due, woken
        // early by a settings push; at least one tick so a due time already
        // past cannot spin.
        const uint64_t dc = _classic.nextDueUs();
        const uint64_t da = _advanced.nextDueUs();
        const uint64_t due = dc == 0 ? da : (da == 0 ? dc : (dc < da ? dc : da));
        uint32_t ms = kIdleWaitMs;
        if (due != 0) {
            const uint64_t after = uint64_t(esp_timer_get_time());
            ms = due > after ? uint32_t((due - after + 999u) / 1000u) : 1u;
            if (ms > kIdleWaitMs) ms = kIdleWaitMs;
            if (ms == 0) ms = 1;
        }
        wait_ms = ms;
    }
}

void PatternTask::tickOne(PatternEngine& gen, uint64_t now_us, const PatternInputs& in) {
    if (const auto it = gen.tick(now_us, in)) {
        if (!motionSubmit(*it)) GLOGW_EVERY_MS(1000, kTag, "stroke dropped: motion queue full");
    }
}

}  // namespace

bool patternBegin() { return g_pattern.begin(); }
void patternSetSettings(const PatternSettings& s) { g_pattern.setSettings(s); }
bool patternActive() { return g_pattern.active(); }
uint32_t patternStackFree() { return g_pattern.stackFree(); }

}  // namespace valence
