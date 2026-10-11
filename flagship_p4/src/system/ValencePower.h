#pragma once

// ValencePower -- the motor current monitor (U11, INA228 or INA237) on its
// private I2C bus: identify at boot, configure, read, re-arm the ALERT latch
// Constraints:
// - Owner: app_main calls powerBegin() before any reader exists; after it
//   returns, the chip and scale are read-only. powerRead(),
//   powerTakeAlerts() and boardMonitorRead() block on I2C for up to a few ms:
//   never call them from the motion task or a timer callback.
// - ONE TASK ON THE BUS. During boot app_main alone calls the three reads
//   (powerBegin(), the self-check). From powerClaimBus() on, the motor-switch
//   task is the bus's one owner (ValenceMotorSwitch.cpp) and a read from any
//   other task is refused without touching the bus. Everyone else takes the
//   owner's 1 Hz reading from motorSwitchPower() (ValenceMotorSwitch.h).
// - The ALERT is LATCHED (PowerMonitor.h) and wired into the motor-switch EN:
//   once it trips, the motor stays cut until powerTakeAlerts() reads
//   DIAG_ALRT. Re-arming is that read plus the MOTOR_EN toggle, never one alone.
// - Absent or unrecognized part is non-fatal: powerChip() reports none and the
//   machine runs without current sensing. Homing and the boot self-check
//   decide what that refuses; this module only reports.
// - The bus (BOARD_GPIO_INA_SDA / _SCL, U11 at 0x40) is also the board
//   monitor's (U12 at SV_I2C_ADDR, val-091.19): its device handle is added
//   here at begin and read through boardMonitorRead(), never a second master
//   or a second handle on these pins.
// - State: a few dozen bytes of BSS plus the IDF driver's small internal-heap
//   allocation at begin; nothing in PSRAM.
// See: flagship_p4/src/system/PowerMonitor.h, Hardware flagship/SPEC.md,
// bd val-091.22, val-9hr

#include <cstdint>
#include <optional>
#include <span>

#include <esp_err.h>

#include "system/PowerMonitor.h"

namespace valence {

// Probes, identifies and configures U11; logs the detected part. Returns
// false when no supported part answered. Call once, from app_main.
bool powerBegin();

PowerChip powerChip();

// One snapshot of the latest conversion. nullopt with no part or on an I2C
// error.
std::optional<PowerReading> powerRead();

// One block from the board monitor: the register id written, `out` read
// (Supervisor.h block lengths). ESP_ERR_INVALID_STATE before the bus opened or
// from a task that does not own it.
esp_err_t boardMonitorRead(uint8_t reg, std::span<uint8_t> out);

// Reads DIAG_ALRT, which clears the latched ALERT (ina2xx::kFlag*). nullopt
// with no part or on an I2C error, in which case the latch is NOT cleared.
std::optional<uint16_t> powerTakeAlerts();

// Hands the bus to the calling task for the rest of the boot. The motor-switch
// task calls it once, after the self-check's verdict. False when another task
// already owns it.
bool powerClaimBus();

}  // namespace valence
