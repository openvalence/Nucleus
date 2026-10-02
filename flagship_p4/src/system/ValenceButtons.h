#pragma once

// ValenceButtons -- the HOME button (SW1 on BTN_HOME, active low, J13 in
// parallel) sampled into ButtonGesture.h's press / hold core
// Constraints:
// - Owner: the BoardIo task (ValenceBoardIo.cpp) calls buttonsBegin() once
//   and buttonsService() at least every 10 ms; the gesture core is that
//   task's alone. homeButtonTake() is any task: one atomic exchange.
// - THE BUTTON COMMANDS NOTHING HERE. A gesture is logged (tag "button",
//   Warn, so it reaches the 0x0008 log channel) and parked for the hub task
//   to take. Acting on it -- home through the home intent path, reset through
//   the admin reset path, both under the arbiter's gates and never while a
//   source owns the rail without the arbiter's say -- is the hub delegate's,
//   and is owed: TODO(val-091.26). Never a side channel to the arbiter, the
//   motor switch or esp_restart().
// - One parked gesture: a newer one overwrites one not yet taken.
// See: ButtonGesture.h, BoardPins.h, Hardware flagship/SPEC.md 2026-09-23
// two-buttons row, bd val-091.26

#include "system/ButtonGesture.h"

namespace valence {

// Configures G49 as an input with its pull-up. Cannot fail in a way that
// matters: a dead pad reads released forever.
void buttonsBegin();

// One sample of the pad.
void buttonsService(uint32_t nowMs);

// The newest completed HOME gesture not yet taken, then none.
button::Gesture homeButtonTake();

}  // namespace valence
