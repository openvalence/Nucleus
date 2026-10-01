#pragma once

// ValencePower -- the motor current monitor (U11, INA228 or INA237) on its
// private I2C bus: identify at boot, configure, read, re-arm the ALERT latch
// Constraints:
// - Owner: app_main calls powerBegin() before any reader exists; after it
//   returns, the chip and scale are read-only. powerRead() and
//   powerTakeAlerts() block on I2C for up to a few ms: never call them from
//   the motion task or a timer callback.
// - The ALERT is LATCHED (PowerMonitor.h) and wired into the motor-switch EN:
//   once it trips, the motor stays cut until powerTakeAlerts() reads
//   DIAG_ALRT. Re-arming is that read plus the MOTOR_EN toggle, never one alone.
// - Absent or unrecognized part is non-fatal: powerChip() reports none and the
//   machine runs without current sensing. Homing and the boot self-check
//   decide what that refuses; this module only reports.
// - The bus (SDA G34, SCL G36, 0x40) is also the board monitor's (val-091.19);
//   that device joins this bus handle, it does not open a second one.
// - State: a few dozen bytes of BSS plus the IDF driver's small internal-heap
//   allocation at begin; nothing in PSRAM.
// See: flagship_p4/src/system/PowerMonitor.h, Hardware flagship/SPEC.md,
// bd val-091.22

#include <cstdint>
#include <optional>

#include "system/PowerMonitor.h"

namespace valence {

struct PowerReading {
    float bus_v;       // VBUS on MOTOR_V+
    float current_a;   // motor current; regen reads negative
    float power_w;     // |V x I| as the part computes it, always >= 0
    float die_c;
};

// Probes, identifies and configures U11; logs the detected part. Returns
// false when no supported part answered. Call once, from app_main.
bool powerBegin();

PowerChip powerChip();

// One snapshot of the latest conversion. nullopt with no part or on an I2C
// error.
std::optional<PowerReading> powerRead();

// Reads DIAG_ALRT, which clears the latched ALERT (ina2xx::kFlag*). nullopt
// with no part or on an I2C error, in which case the latch is NOT cleared.
std::optional<uint16_t> powerTakeAlerts();

}  // namespace valence
