// ValenceBoardIo -- implementation. See ValenceBoardIo.h for the task's
// placement and what it hosts.

#include "system/ValenceBoardIo.h"

#include <cstdint>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "geiger/geiger.h"
#include "system/ValenceAccessoryIo.h"
#include "system/ValenceButtons.h"
#include "system/ValenceEstopInput.h"
#include "system/ValenceFan.h"
#include "system/ValenceGlow.h"
#include "system/ValencePdSource.h"

namespace valence {

namespace {

constexpr const char* kTag = "boardio";

TaskHandle_t g_task = nullptr;

void taskMain(void*) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(kBoardIoPeriodMs));
        const uint32_t nowMs = uint32_t(esp_timer_get_time() / 1000);
        estopInputService(nowMs);
        buttonsService(nowMs);
        glowService(nowMs);
        fanService(nowMs);
        accessoryIoService(nowMs);
        pdSourceService(nowMs);
    }
}

}  // namespace

bool boardIoBegin() {
    glowBegin();
    fanBegin();
    buttonsBegin();
    estopInputBegin();
    accessoryIoBegin();
    pdSourceBegin();
    if (xTaskCreatePinnedToCore(&taskMain, "BoardIo", kBoardIoTaskStackBytes, nullptr, 3, &g_task, 0) !=
        pdPASS) {
        g_task = nullptr;
        GLOGE(kTag, "BoardIo task did not start: e-stop unread (release refused), no status pixel, "
                    "fan off, HOME and PAIR unread");
        return false;
    }
    return true;
}

uint32_t boardIoStackFree() {
    // IDF reports BYTES, not the vanilla FreeRTOS words.
    return g_task != nullptr ? uint32_t(uxTaskGetStackHighWaterMark(g_task)) : 0;
}

}  // namespace valence
