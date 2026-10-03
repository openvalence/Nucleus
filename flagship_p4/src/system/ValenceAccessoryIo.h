#pragma once

// ValenceAccessoryIo -- the accessory headers on the spare LP pads: pump PWM
// (J10, 12 V through the pump eFuse U14), EXT PWM (J11), AUX1 and AUX2 (J13),
// the Qwiic bus (J14, shared with the PD daughterboard's J16), the header
// UART and IO1-IO5 (J15)
// Constraints:
// - Owner: the BoardIo task (ValenceBoardIo.cpp) calls accessoryIoBegin()
//   once and accessoryIoService() on every pass; the LEDC channels, the AUX
//   gesture cores and every IO pad's drive are that task's alone. The request
//   functions are ANY task (AccessoryIo.h): the owner applies them on its next
//   pass, within kBoardIoPeriodMs.
// - OFF AT BOOT AND AFTER EVERY ESTOP: both duties 0, every IO output
//   released. motionEstop() calls accessoryIoEstop(), and the pads follow on
//   the owner's next pass. The latch is the hub's: refusing a request while it
//   is held is the delegate's job when the channels land (val-091.71).
// - The pump eFuse enables itself (EN/UVLO divider off +12V, about 9 V): no
//   P4 pin enables it, so the duty request is the pump's only enable.
// - The Qwiic bus and the header UART are interfaces, not outputs: an ESTOP
//   leaves them to their consumer.
// - HP peripherals only, set up without gpio_config() (BoardPins.h).
// - Declarations only, no IDF: the hub delegate will call the request
//   functions, so a host twin links its own. The two board-only interfaces
//   name the IDF handle type without the IDF header.
// See: AccessoryIo.h, ButtonGesture.h, BoardPins.h, docs/board-map.md
// (accessory headers), Hardware flagship/SPEC.md 2026-09-23 accessory rows,
// bd val-091.30

#include <cstdint>
#include <optional>

#include "system/AccessoryIo.h"
#include "system/ButtonGesture.h"

// IDF's i2c_master_bus_handle_t is i2c_master_bus_t*.
struct i2c_master_bus_t;

namespace valence {

// The PWM timer at duty 0, the AUX inputs, IO1-IO5 released. The BoardIo
// host calls it from boardIoBegin(). False when the PWM timer would not
// start: both PWMs then stay off for the boot; the AUX buttons and the IO
// pads still run.
bool accessoryIoBegin();

// One pass of the BoardIo task: applies the requests, samples AUX1 and AUX2.
void accessoryIoService(uint32_t nowMs);

// Every request off (AccessoryIo.h estop()). Any task or ISR, never blocks.
void accessoryIoEstop();

// Duty 0..1, clamped, NaN is 0. Any task. False when the PWM timer is not up
// or an ESTOP landed during the call.
// TODO(val-091.71): the pump and EXT PWM duty controls and their STATE.
bool accessoryPwmSet(accessory::Pwm out, float duty);

// IO1..IO5 by the number on the silk. Any task.
// TODO(val-091.71): the direction setting, the level INTENT, the levels STATE.
bool accessoryGpioConfigure(uint8_t io, accessory::Dir dir);   // once per boot
bool accessoryGpioSet(uint8_t io, bool high);                 // outputs only
std::optional<bool> accessoryGpioGet(uint8_t io);              // the pad, read now

// The newest AUX press or hold not yet taken, then none. Any task.
// TODO(val-091.71): AUX gestures as events or delegate bindings.
button::Gesture accessoryAuxTake(accessory::Aux which);

// Board only: the Qwiic bus (SDA G11, SCL G9), opened by the first call on a
// free HP I2C port; nullptr when it would not open. Any task, never an ISR or
// a critical section: the first call blocks for the open. Joiners add and
// remove their own device handles; the bus is never deleted and never gets a
// second master.
i2c_master_bus_t* qwiicI2cBus();

// Board only: the header UART (TX G14, RX G15, the HP UART BOARD_UART_ACC)
// at `baud`, 8N1: the IDF port number, or nullopt when it would not open or
// another consumer already holds it. The caller's task owns the port from
// then on.
std::optional<int> accessoryUartOpen(uint32_t baud);

}  // namespace valence
