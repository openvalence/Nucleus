#pragma once

// ValenceEstopInput -- the external E-stop's NC (G39) and NO (G30) contacts,
// sampled on the BoardIo task into EstopInput.h's reading and handed to the
// hub task as one atomic byte
// Constraints:
// - Owner: the BoardIo task (ValenceBoardIo.cpp) calls estopInputBegin() once
//   and estopInputService() every kBoardIoPeriodMs; the reader core is that
//   task's alone. estopInputRead() is any task: one atomic load. This module
//   is the two pads' one configurer and reader.
// - IT COMMANDS NOTHING. The hub delegate latches ESTOP while the reading
//   stops and refuses the release until it reads released
//   (ValenceDevice.cpp); the stop itself is hardware.
// - POLLED, NEVER AN ISR. With motor power on, the motor switch's 5 ms
//   EN-node watch parks the emitter before any debounced read could; an ISR
//   can only set a flag the hub task polls anyway, and a flapping plug would
//   storm core 0 with edges.
// - Declarations only, no IDF: the hardware-free delegate calls
//   estopInputRead(), so the host twin and the native suite link their own.
// See: EstopInput.h, BoardPins.h, docs/board-map.md, bd val-091.23

#include <cstdint>

#include "system/EstopInput.h"

namespace valence {

// Configures G39 and G30 as inputs with their pull-ups.
void estopInputBegin();

// One sample of both pads. Logs every settled change (tag "estop", Warn and
// above, so each reaches the 0x0008 log channel).
void estopInputService(uint32_t nowMs);

// The settled reading; not known until the first state settles after boot.
estop::Reading estopInputRead();

}  // namespace valence
