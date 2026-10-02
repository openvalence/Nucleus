// SimPattern -- patterns/ValencePattern.h on a desktop: the board's own two
// generators, ticked on the sim's one thread
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

// File scope, not stack locals: seven pattern objects in the classic one.
ClassicGenerator  g_classic;
AdvancedGenerator g_advanced;
std::optional<PatternSettings> g_pending;

}  // namespace

void simPatternTick(uint64_t now_us) {
    if (g_pending) {
        g_classic.apply(*g_pending);
        g_advanced.apply(*g_pending);
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
    // The board's order: classic, then advanced, on one task.
    if (const auto it = g_classic.tick(now_us, in)) motionSubmit(*it);
    if (const auto it = g_advanced.tick(now_us, in)) motionSubmit(*it);
}

// ---- patterns/ValencePattern.h ----------------------------------------------

bool patternBegin() { return true; }
void patternSetSettings(const PatternSettings& s) { g_pending = s; }
bool patternActive() { return g_classic.active() || g_advanced.active(); }
uint32_t patternStackFree() { return 0; }

}  // namespace valence
