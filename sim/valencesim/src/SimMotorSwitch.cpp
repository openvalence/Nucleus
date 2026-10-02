// SimMotorSwitch -- system/ValenceMotorSwitch.h on a desktop: no motor switch,
// motor power on from boot
// Constraints:
// - SINGLE-THREADED: every function runs on the sim's one hub thread.
// - The twin declares estop_cuts_power false, so there is no power to cut and
//   the arbiter's power gate is opened once at begin and never closed.
// See: flagship_p4/src/system/ValenceMotorSwitch.h, bd val-091.24

#include "system/ValenceMotorSwitch.h"

#include "motion/ValenceMotion.h"

namespace valence {

bool motorSwitchBegin() {
    motionSetMotorPowered(true);
    return true;
}

void motorSwitchSetSelfCheck(bool) {}

motorswitch::Refusal motorSwitchRequestEnable() { return motorswitch::Refusal::none; }

void motorSwitchCut() {}

MotorSwitchStatus motorSwitchStatus() {
    MotorSwitchStatus s;
    s.state = motorswitch::State::on;
    return s;
}

}  // namespace valence
