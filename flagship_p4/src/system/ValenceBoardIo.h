#pragma once

// ValenceBoardIo -- the BoardIo task: the slow board I/O that shares one
// 10 ms pass (the e-stop contacts, the status pixel, the fan and THERM
// policy, the HOME and PAIR buttons, the accessory headers, the PD
// daughterboard's PD_INT)
// Constraints:
// - Task "BoardIo": HP core 0, priority 3 (below the motor switch's 5 and
//   the hub's 5), kBoardIoTaskStackBytes of internal RAM, vTaskDelayUntil at
//   kBoardIoPeriodMs. It hosts ValenceEstopInput, ValenceGlow, ValenceFan,
//   ValenceButtons, ValenceAccessoryIo and ValencePdSource and is the only
//   caller of their begin and service functions. The e-stop is sampled first
//   in every pass.
// - Nothing on it is on a motion path: the e-stop reading and the buttons'
//   gestures are published for the hub task to act on, the fan is comfort,
//   the pixel is display. A stall here freezes the pixel (logging-leds.md
//   T7) and the e-stop reading; the hardware stop, the motor switch's EN-node
//   watch and the hub delegate's fault latch do not depend on it. Its direct
//   safety acts are two: the HOME hold's dead-hub fallback (ValenceButtons.h),
//   the ESTOP cut and a restart; and the PD source's verdict
//   (ValencePdSource.h), a motor power cut when a contract cannot carry the
//   input ceilings. A stall delays the second by its own length.
// - The accessory outputs reach their pads only here: a stall also holds
//   them where they are, an ESTOP's zeroing included (ValenceAccessoryIo.h).
// See: ValenceEstopInput.h, ValenceGlow.h, ValenceFan.h, ValenceButtons.h,
// ValenceAccessoryIo.h, ValencePdSource.h, bd val-091.23/.26/.27/.28/.30/.31

#include <cstdint>

namespace valence {

// The task's stack, in bytes, and the one home for that number: the create
// site and main.cpp's high-water watch both read it here.
// TODO(val-091.66): size from a high-water mark with the pixel, the fan loop
// and a gesture all exercised.
inline constexpr uint32_t kBoardIoTaskStackBytes = 4096;
inline constexpr uint32_t kBoardIoPeriodMs = 10;

// Brings up the pixel, the fan, the buttons, the e-stop pads and the
// accessory headers (outputs off), then starts the task.
// app_main, after motorSwitchBegin() (THERM rides its ADC poll). False when
// the task did not start; each module's own failure is logged and non-fatal.
bool boardIoBegin();

// The task's stack high-water headroom, bytes; 0 before boardIoBegin().
uint32_t boardIoStackFree();

}  // namespace valence
