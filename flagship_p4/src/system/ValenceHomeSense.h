#pragma once

// ValenceHomeSense -- BOARD_GPIO_HOME_SENSE as the arbiter's HomeSense: a
// rising-edge interrupt latched beside a level read
// Constraints:
// - IDF-free here; ValenceHomeSense.cpp is the board host and the sim has its
//   own stand-in (sim/valencesim/src/SimMotion.cpp).
// - homeSenseBegin() runs once, from MotionTask::begin(), before the motion
//   task exists. With BOARD_GPIO_HOME_SENSE -1 it touches no pad and the
//   sense it returns is absent: home op 1 refuses, never seeks open-loop.
// - The level is active HIGH and push-pull from its source; the pad holds a
//   pull-down. probe() lifts it to the pull-up for kProbeSettleUs to tell a
//   driven-low line from an undriven one, so it runs on the hub task before a
//   cycle and never during one (MotionArbiter::home()).
// - The interrupt only latches a rise for the motion task's next high();
//   nothing in it touches motion. Debounce is the arbiter's.
// See: MotionArbiter.h (homing: the input contract), BoardPins.h,
// docs/board-map.md

#include "motion/MotionArbiter.h"

namespace valence {

HomeSense& homeSenseBegin();

}  // namespace valence
