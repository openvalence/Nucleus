#pragma once

// ValencePdSource -- the PD daughterboard's controller (TPS26750, 0x21) on the
// Qwiic bus: found at boot, its contract read and logged, PD_INT (LPG0)
// serviced on the BoardIo task, and the contract's verdict pushed to the motor
// switch
// Constraints:
// - Owner: the BoardIo task (ValenceBoardIo.cpp) calls pdSourceBegin() once
//   from boardIoBegin(), on app_main before the task exists, and
//   pdSourceService() every kBoardIoPeriodMs. Every transaction to 0x21 and
//   every write of this module's state happens there. pdSourceAssessNow() is
//   any task: a copy under one spinlock, the motion census pattern.
// - PD_INT IS POLLED, NEVER AN ISR. I2Ct_IRQ is open-drain and LEVEL: it holds
//   LPG0 low while any armed INT_EVENT1 bit is set and lets go only when
//   INT_CLEAR1 clears it (SLVUCR7 4.5, 4.7), so a 10 ms sample cannot miss it.
//   The cause is an I2C read, which cannot run in an ISR, and the source has
//   already switched contracts when the line falls: an ISR would only wake
//   this same task sooner, for nothing.
// - THE QWIIC BUS IS ValenceAccessoryIo's (qwiicI2cBus(), J14 and J16 on one
//   pair): this module joins it with its own device at 0x21 and never opens a
//   master. The IDF driver serializes the bus's devices across tasks.
// - IT COMMANDS ONE THING: the motor switch's source verdict
//   (motorSwitchSetSourceOk), pushed on every change. A refused contract cuts
//   motor power if it is on and refuses every enable until a contract carries
//   the input ceilings again.
// - A board whose 0x21 never answered is a DC-input build: nothing is serviced
//   and the motor switch never hears from this module.
// - pdSourceBegin() runs BEFORE motionBegin(): the census reads zero ceilings
//   there, so the boot judgment refuses only a contract that is unusable at
//   any speed. The first service pass after the motion task publishes judges
//   the real ceilings, and the boot self-check's pd-source row, which gates
//   the first enable, reads them too.
// - A boot probe that hit a bus error is never retried: the self-check row
//   fails with it, and a failed row holds motor power off until reboot.
// - BSS: a device handle, two readings and a few words. A transaction blocks
//   the BoardIo task ~1.5 ms at 100 kHz; nothing here is on a motion path.
// See: PdSource.h, ValenceAccessoryIo.h, ValenceBoardIo.h,
// ValenceMotorSwitch.h, BoardPins.h, docs/board-map.md, bd val-091.31

#include <cstdint>

#include "system/PdSource.h"

namespace valence {

// Configures PD_INT, joins the Qwiic bus, probes 0x21 and reads MODE; in APP,
// arms INT_MASK1, clears INT_EVENT1 and reads the contract. Logs what it
// found. False when no TPS26750 is ready, which a DC-input build is.
bool pdSourceBegin();

// One BoardIo pass: PD_INT sampled; while it is low, INT_EVENT1 read and
// cleared and the contract re-read, logged on every renegotiation; the
// verdict re-judged against the live input ceilings once a second.
void pdSourceService(uint32_t nowMs);

// The latest reading against the input ceilings as the engine plans them
// (motionCensus()). Any task.
// TODO(val-091.75): ch::pd_source publishes this; until then the contract
// reaches a client only as the self-check row and the log lines on 0x0008.
pd::Assessment pdSourceAssessNow();

}  // namespace valence
