#pragma once

// ValenceGlow -- the board's Flux glue: the status pixel (D5, XL-3528RGBW,
// GRBW at 32 bits per pixel) on LED_DATA, the machine-state table and the
// liveness gate
// Constraints:
// - THE one Flux glue file for this board (logging-leds.md): nothing else
//   drives G26, and nothing else pumps the engine.
// - Owner: the BoardIo task (ValenceBoardIo.cpp) calls glowBegin() once and
//   glowService() on every pass; the engine, the RMT channel and the frame
//   are that task's alone. glowSetSelfCheck() is the one door from another
//   task (app_main), two atomic stores.
// - Pixel 0 is the status pixel. The NEOPIXEL OUT chain (J12) after it is
//   never written: its pixels keep their power-on dark until a channel owns
//   them (bd val-091.27).
// - The hub's tick counter is the liveness gate's heartbeat once the hub is
//   up; a hub that stops ticking freezes the pixel mid-frame (T7: a
//   diagnostic, never a bug to fix here).
// See: StatusLook.h, lib/flux/include/flux/flux_core.hpp,
// .claude/rules/logging-leds.md, BoardPins.h, bd val-091.27

#include <cstdint>

namespace valence {

// Brings up the RMT channel and the engine, boot rainbow showing. False when
// the channel would not start; the engine still runs on a null output, so
// the state table and the gate are exercised either way.
bool glowBegin();

// One pass of the BoardIo task: facts, heartbeat, frame. Self-paced; call
// at least every 20 ms.
void glowService(uint32_t nowMs);

// The boot self-check's verdict, once, from app_main. Ends the boot rainbow.
void glowSetSelfCheck(bool passed);

}  // namespace valence
