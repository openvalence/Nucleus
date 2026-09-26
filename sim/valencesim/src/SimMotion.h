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

}  // namespace valence
