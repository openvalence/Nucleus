#pragma once

// ValencePattern -- the generators' one door: hand them settings, ask whether
// either is driving
// Constraints:
// - The generator OWNS NOTHING the hub needs back. The hub delegate owns the
//   live PatternSettings and pushes whole copies; the generator's strokes
//   leave through motionSubmit() like every other input source, so the
//   arbiter stays the sole caller of the emitter (architecture.md section 2).
// - patternSetSettings() may be called from any task and never blocks: a
//   newer set overwrites an unapplied older one, the only one that matters.
// - Each composition links its own implementation: ValencePattern.cpp (the
//   P4's pattern task) or sim/valencesim's SimPattern.cpp (one thread).
// See: PatternEngine.h, PatternSettings.h, bd val-091.12

#include <cstdint>

#include "PatternSettings.h"

namespace valence {

// The pattern task's stack, in bytes, and the ONE home for that number (C-1):
// the create site and main.cpp's high-water watch table both read it here.
// Sized, not yet measured (bd val-def.1). The task never plans -- commit()
// and its KB-scale Ruckig temporaries run on the motion task -- so its
// deepest path is one Geiger line (a 128 B record plus newlib's float
// vsnprintf frame, ~1.5 KB) over a MotionCensus copy (~0.3 KB) and the
// engine's own frames; 4,096 B leaves roughly a third of it spare. The two
// generators tick one after the other, so the deepest path is one of them.
inline constexpr uint32_t kPatternTaskStackBytes = 4096;

// Brings up the generator and, on the board, its task. Must run AFTER
// motionBegin(): every tick reads motionCensus().
bool patternBegin();

void patternSetSettings(const PatternSettings& s);

// True while either generator is driving the machine (PatternEngine::active()).
// Any task.
bool patternActive();

// Pattern task stack high-water headroom in bytes, 0 before the task exists
// or on a host with no task.
uint32_t patternStackFree();

}  // namespace valence
