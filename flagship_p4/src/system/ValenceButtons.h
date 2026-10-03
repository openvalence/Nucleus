#pragma once

// ValenceButtons -- the HOME and PAIR buttons (SW1 on BTN_HOME, SW2 on
// BTN_PAIR, both active low, J13 in parallel) sampled into ButtonGesture.h's
// press / hold core
// Constraints:
// - Owner: the BoardIo task (ValenceBoardIo.cpp) calls buttonsBegin() once
//   and buttonsService() at least every 10 ms; the gesture cores are that
//   task's alone. homeButtonTake() and pairButtonTake() are any task: one
//   atomic exchange each. The hub delegate takes them in tick() and owns
//   every binding (ValenceDevice.cpp, operator ruling 2026-10-02).
// - THE BUTTONS COMMAND NOTHING HERE, with ONE exception: a HOME hold the hub
//   task has not taken within kHoldFallbackMs means the hub task is not
//   running, which is the crash the hold exists for. The BoardIo task then
//   cuts motor power (motionEstop(), any task by its contract) and restarts
//   the chip itself: no GOODBYE, no NVS flush, nothing that needs the hub.
// - Each gesture is logged (tag "button", Warn, so it reaches the 0x0008 log
//   channel) and parked; a newer one overwrites one not yet taken.
// - Declarations only, no IDF: the hardware-free delegate calls the two take
//   functions, so the host twin and the native suite link their own.
// See: ButtonGesture.h, BoardPins.h, Hardware flagship/SPEC.md 2026-09-23
// two-buttons row, bd val-091.26

#include <cstdint>

#include "system/ButtonGesture.h"

namespace valence {

// A parked HOME hold not taken by the hub task within this long is a hub that
// is not running: the hub ticks every 5 ms.
inline constexpr uint32_t kHoldFallbackMs = 2000;

// Configures G49 and G52 as inputs with their pull-ups. Cannot fail in a way
// that matters: a dead pad reads released forever.
void buttonsBegin();

// One sample of both pads.
void buttonsService(uint32_t nowMs);

// The newest completed gesture not yet taken, then none.
button::Gesture homeButtonTake();
button::Gesture pairButtonTake();

}  // namespace valence
