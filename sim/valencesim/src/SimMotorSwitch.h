#pragma once

// SimMotorSwitch -- the host twin's drive for system/ValenceMotorSwitch.h
// Constraints:
// - SINGLE-THREADED: both functions and every ValenceMotorSwitch.h function
//   run on the sim's one hub thread.
// See: SimMotorSwitch.cpp, flagship_p4/src/system/ValenceMotorSwitch.h

#include <cstdint>

namespace valence {

// Turns the switch model on (--motor-switch). Call before motorSwitchBegin();
// without it the twin has no switch and motor power is on from boot.
void simMotorSwitchModel();

// MSW_FLT_N reads low from from_us until until_us, hub clock (--msw-fault).
void simMotorSwitchInjectFault(uint64_t from_us, uint64_t until_us);

// Steps the model at now_us, the clock deviceNowUs() reads. Call every loop
// pass, before simMotionTick(), as the board's switch task runs beside motion.
void simMotorSwitchTick(uint64_t now_us);

}  // namespace valence
