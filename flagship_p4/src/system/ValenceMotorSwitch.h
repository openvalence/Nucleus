#pragma once

// ValenceMotorSwitch -- the motor switch's one door: begin, the self-check
// verdict, the enable request, the power cut, and its status
// Constraints:
// - Declarations only, no IDF: the board (ValenceMotorSwitch.cpp) and the host
//   twin (sim/valencesim SimMotorSwitch.cpp) each link one implementation,
//   and the hardware-free hub delegate calls through this header.
// - motorSwitchCut(): ANY task, never blocks; MOTOR_EN and PRECHARGE_EN are
//   low when it returns. It is the ESTOP's power cut (SPEC 11.2, H1 path).
// - motorSwitchRequestEnable(): the hub task. Answers what it can at once
//   (MotorSwitch.h precheck()); the EN node is judged by the switch's own
//   task after the INA ALERT re-arm, and a refusal there is logged, not
//   returned.
// - motorSwitchStatus(): any task, one consistent copy.
// - The implementation pushes every entry into and exit from `on` to the
//   arbiter (motionSetMotorPowered), so motion is gated on exactly the state
//   this door reports.
// See: MotorSwitch.h, ValenceMotion.h, bd val-091.24

#include <cstdint>

#include "system/MotorSwitch.h"

namespace valence {

// The board's switch task stack, in bytes, and the one home for that number:
// the create site and main.cpp's high-water watch both read it here.
// TODO(val-091.56): size from a high-water mark under estop, release and a fault.
inline constexpr uint32_t kMotorSwitchTaskStackBytes = 4096;

struct MotorSwitchStatus {
    motorswitch::State state      = motorswitch::State::off;
    motorswitch::Fault last_fault = motorswitch::Fault::none;
    uint16_t           faults     = 0;   // faults latched since boot, wraps
};

// Takes the first readings and starts the switch's host, state off. Board:
// app_main, after selfCheckHoldMotorOff() and powerBegin(), before
// selfCheckRun(). Returns false when the host could not start; the switch
// then stays off for good.
bool motorSwitchBegin();

// The boot self-check's verdict, once, from app_main. A pass requests the
// enable sequence at once (val-091.21: a clean board enables within a bounded
// time); a fail holds motor power off for the life of the boot.
void motorSwitchSetSelfCheck(bool passed);

motorswitch::Refusal motorSwitchRequestEnable();
void motorSwitchCut();
MotorSwitchStatus motorSwitchStatus();

// Board only: MSW_FLT_N read now, true = asserted (low). The self-check's
// switch-fault entry reads it here so the pin has one reader module.
bool motorSwitchFaultLine();
// Board only: the switch task's stack high-water headroom, bytes; 0 before
// motorSwitchBegin().
uint32_t motorSwitchStackFree();

}  // namespace valence
