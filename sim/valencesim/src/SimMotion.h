#pragma once

// SimMotion -- the host twin's drive for motion/ValenceMotion.h
// Constraints:
// - SINGLE-THREADED: simMotionTick() and every ValenceMotion.h function run on
//   the sim's one hub thread. Nothing here is safe to call from an IXWebSocket
//   connection thread.
// See: SimMotion.cpp, flagship_p4/src/motion/ValenceMotion.h

#include <cstdint>

namespace valence {

// Drains queued intents and evaluates the plan at now_us, the same clock
// deviceNowUs() reads. Call every loop pass; dt is measured, not assumed.
void simMotionTick(uint64_t now_us);

// --home-sense-at: a hard stop at `at_mm` in the emitter's boot frame (0.0 mm
// is where the carriage booted). The sense reads HIGH while the carriage is at
// or past it, on the side away from 0, so a negative value puts the stop
// behind the boot position. Without a call the sim has no sense line and home
// op 1 refuses, as a board without one does. Before motionBegin().
void simMotionSetHomeSenseAt(float at_mm);

}  // namespace valence
