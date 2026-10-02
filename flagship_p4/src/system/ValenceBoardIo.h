#pragma once

// ValenceBoardIo -- the BoardIo task: the slow board I/O that shares one
// 10 ms pass (the status pixel, the fan and THERM policy, the HOME button)
// Constraints:
// - Task "BoardIo": HP core 0, priority 3 (below the motor switch's 5 and
//   the hub's 5), kBoardIoTaskStackBytes of internal RAM, vTaskDelayUntil at
//   kBoardIoPeriodMs. It hosts ValenceGlow, ValenceFan and ValenceButtons and
//   is the only caller of their begin and service functions.
// - Nothing on it is on a motion or safety path: the button parks gestures,
//   the fan is comfort, the pixel is display. A stall here freezes the pixel
//   (logging-leds.md T7) and nothing else.
// See: ValenceGlow.h, ValenceFan.h, ValenceButtons.h, bd val-091.26/.27/.28

#include <cstdint>

namespace valence {

// The task's stack, in bytes, and the one home for that number: the create
// site and main.cpp's high-water watch both read it here.
// TODO(val-091.66): size from a high-water mark with the pixel, the fan loop
// and a gesture all exercised.
inline constexpr uint32_t kBoardIoTaskStackBytes = 4096;
inline constexpr uint32_t kBoardIoPeriodMs = 10;

// Brings up the pixel, the fan and the button, then starts the task.
// app_main, after motorSwitchBegin() (THERM rides its ADC poll). False when
// the task did not start; each module's own failure is logged and non-fatal.
bool boardIoBegin();

// The task's stack high-water headroom, bytes; 0 before boardIoBegin().
uint32_t boardIoStackFree();

}  // namespace valence
