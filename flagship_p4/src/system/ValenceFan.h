#pragma once

// ValenceFan -- the fan (J6: FAN_PWM G27 into the high-side switch and its LC
// buck, FAN_TACH G38) and the J7 thermistor's temperature, run by Thermal.h's
// policy
// Constraints:
// - Owner: the BoardIo task (ValenceBoardIo.cpp) calls fanBegin() once and
//   fanService() on every pass; the LEDC channel, the PCNT unit and the
//   policy are that task's alone. fanStatus() is any task.
// - THERM is ADC1, and the motor switch task is ADC1's one reader: this
//   module takes the volts from motorSwitchThermVolts() and never touches the
//   ADC (ValenceMotorSwitch.cpp says why a second reader cuts motor power).
// - The fan is off from reset until fanBegin() (R701 100k holds FAN_PWM low);
//   fanBegin() writes duty 0 before anything else can run.
// - NOT ON THE WIRE: no catalog channel carries a board temperature or a fan.
//   TODO(val-091.65): a board-thermal STATE channel published from fanStatus().
// See: Thermal.h, BoardPins.h, docs/board-map.md (regen clamp, thermal, fan),
// bd val-091.28

#include <cstdint>

#include "system/Thermal.h"

namespace valence {

// One snapshot for the console line and, later, the wire. Fields are stored
// one at a time once a second: a reader may see one step's duty beside the
// previous step's RPM, never a torn single field.
struct FanStatus {
    float            celsius = thermal::kNaN;   // J7; NaN = no usable reading
    float            duty    = 0.0f;            // 0..1 on FAN_PWM
    float            rpm     = 0.0f;            // measured over the last step
    thermal::FanMode mode    = thermal::FanMode::off;
    bool             tach    = false;           // closed loop on RPM
    bool             fitted  = true;            // false: no tach after the kick, no fan
};

// Configures LEDC (duty 0) and the PCNT tach counter. False when either
// failed; the fan then stays off for the boot.
bool fanBegin();

// One pass of the BoardIo task. Self-paced at thermal::kStepMs.
void fanService(uint32_t nowMs);

FanStatus fanStatus();

}  // namespace valence
