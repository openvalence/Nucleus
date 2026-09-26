// SimPattern -- patterns/ValencePattern.h on a desktop: the board's own
// PatternEngine, ticked on the sim's one thread
// Constraints:
// - SINGLE-THREADED (SimPattern.h). The P4 hands settings across a depth-one
//   FreeRTOS queue to its pattern task; here one pending slot on one thread is
//   the same thing, and nothing is locked.
// - NO GENERATOR LOGIC LIVES HERE: flagship_p4/src/patterns/PatternEngine.cpp
//   is compiled verbatim. This file is the tick order and nothing else.
// See: SimPattern.h, flagship_p4/src/patterns/PatternEngine.h

#include "SimPattern.h"

#include <optional>

#include "motion/ValenceMotion.h"
#include "patterns/PatternEngine.h"
#include "patterns/ValencePattern.h"

namespace valence {
namespace {

// File scope, not a stack local: seven pattern objects.
PatternEngine g_engine;
std::optional<PatternSettings> g_pending;

}  // namespace

void simPatternTick(uint64_t now_us) {
    if (g_pending) {
        g_engine.apply(*g_pending);
        g_pending.reset();
    }
    const MotionCensus c = motionCensus();
    PatternInputs in;
    in.homed         = c.homed;
    in.estop         = c.estop;
    in.paused        = c.paused;
    in.stream_active = c.stream;
    in.position_mm   = c.position_mm;
    in.velocity_mm_s = c.velocity_mm_s;
    if (const auto it = g_engine.tick(now_us, in)) motionSubmit(*it);
}

// ---- patterns/ValencePattern.h ----------------------------------------------

bool patternBegin() { return true; }
void patternSetSettings(const PatternSettings& s) { g_pending = s; }
bool patternActive() { return g_engine.active(); }
uint32_t patternStackFree() { return 0; }

}  // namespace valence
