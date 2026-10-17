#pragma once

// SimPattern -- the host twin's drive for patterns/ValencePattern.h
// Constraints:
// - SINGLE-THREADED: simPatternTick() and every ValencePattern.h function run
//   on the sim's one hub thread.
// See: SimPattern.cpp, flagship_p4/src/patterns/ValencePattern.h

#include <cstdint>

namespace valence {

// Applies any pending settings and ticks both generators at now_us, submitting
// their strokes through motionSubmit(). Call every loop pass.
void simPatternTick(uint64_t now_us);

}  // namespace valence
