#pragma once

// ValenceHomeSense -- BOARD_GPIO_HOME_SENSE as the arbiter's HomeSense: a
// level read, a rise confirmed by a second read, and a rising-edge interrupt
// that parks an armed seek and wakes the planner task
// Constraints:
// - IDF-free here; ValenceHomeSense.cpp is the board host and the sim has its
//   own stand-in (sim/valencesim/src/SimMotion.cpp).
// - homeSenseBegin() runs once, from MotionTask::begin(), before either
//   motion task exists. With BOARD_GPIO_HOME_SENSE -1 it touches no pad and the
//   sense it returns is absent: home op 1 refuses, never seeks open-loop.
// - The level is active HIGH and push-pull from its source; the pad holds a
//   pull-down. probe() lifts it to the pull-up for kProbeSettleUs to tell a
//   driven-low line from an undriven one, so it runs on the hub task before a
//   cycle and never during one (MotionArbiter::home()).
// - The interrupt calls `rose` FIRST (MotionArbiter::homeSenseRose(): an armed
//   seek parks there, before anything else runs), then stamps the rise, then
//   calls `wake` (the planner task's notify, so the rise is confirmed now and
//   not on the next tick). The rise is NOT debounced before the park: high()
//   on the planner confirms it (the line HIGH, a rise younger than
//   kHomeSenseDebounceUs read again at that age), and a rise it rejects
//   resumes the seek (MotionArbiter.h, homing).
// See: MotionArbiter.h (homing: the input contract), BoardPins.h,
// docs/board-map.md

#include "motion/MotionArbiter.h"

namespace valence {

// Called from an ISR: no blocking, no log. Null does nothing.
using HomeSenseIsr = void (*)();

HomeSense& homeSenseBegin(HomeSenseIsr rose, HomeSenseIsr wake);

}  // namespace valence
