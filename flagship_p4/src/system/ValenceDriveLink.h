#pragma once

// ValenceDriveLink -- the drive's one door: the RS485 Modbus link to the
// 60AIM40 (identity, configuration check, alarm code, encoder readback) and
// its two status lines, DRV_ALM and DRV_RDY
// Constraints:
// - Declarations only, no IDF: the hardware-free hub delegate calls
//   driveAlarmTake(), so the host twin and the native suites link their own.
// - Task "DriveLnk": HP core 0, priority 2 (under BoardIo's 3), a
//   kDriveLinkPeriodMs pass, kDriveLinkTaskStackBytes of internal RAM. It
//   samples both lines every pass and is the only owner of the UART, its RX
//   ring and every AimDrive.h object.
// - THE CROSSINGS, and nothing else is shared: driveAlarmTake() is one atomic
//   exchange; driveLinkStatus() peeks a one-slot mailbox the task overwrites;
//   driveLinkRead() and driveLinkTakeResult() are two bounded queues of
//   kDriveLinkQueueDepth that never block either side.
// - RS485_DE is driven LOW by driveLinkHoldIdle(), app_main's second call, and
//   only the UART's RS485 half-duplex mode raises it, for its own frames
//   (Hardware hw-3jk: DE and /RE tied float near 1.65 V until driven).
// - A DRV_ALM assertion while the drive is settled (aim::StatusLines) is
//   handed to the hub delegate, which latches ESTOP cause fault
//   (ValenceDevice.cpp tick()). This module never cuts power itself.
// - Built without CONFIG_NUCLEUS_DRIVE_LINK (Kconfig.projbuild) the UART is
//   never installed: DE stays a GPIO held low, both lines are still watched,
//   and the status reads built = false.
// See: AimDrive.h, ModbusRtu.h, BoardPins.h, docs/board-map.md (drive comms
// and status), bd val-091.29

#include <cstdint>

#include "system/AimDrive.h"
#include "system/ModbusRtu.h"

namespace valence {

// The task's stack, in bytes, and the one home for that number: the create
// site and main.cpp's high-water watch both read it here.
// TODO(val-091.69): size from a high-water mark with a probe, the poll and a
// drive alarm all exercised.
inline constexpr uint32_t kDriveLinkTaskStackBytes = 4096;
inline constexpr uint32_t kDriveLinkPeriodMs = 5;
inline constexpr uint32_t kDriveLinkQueueDepth = 4;

// One consistent copy of the link and the lines, as of the task's last pass.
// TODO(val-091.73): the drive-status STATE entry in ValenceCatalog.h
// publishes this.
struct DriveLinkStatus {
    bool built = false;      // CONFIG_NUCLEUS_DRIVE_LINK: the UART link exists
    bool uart = false;       // the UART installed in RS485 mode: the link can run
    bool alarm = false;      // DRV_ALM debounced: the drive reports an alarm
    bool ready = false;      // DRV_RDY debounced: ready, following error under 0.5 deg
    bool armed = false;      // DRV_ALM is acted on: motor power settled
    uint32_t alarms = 0;     // alarms handed to the hub since boot, wraps
    aim::Snapshot link{};
};

// RS485_DE as an output, LOW; DRV_ALM and DRV_RDY as inputs. app_main, right
// after selfCheckHoldMotorOff().
void driveLinkHoldIdle();

// The UART (when built), the mailbox, the queues and the task. app_main,
// after motorSwitchBegin(): the task reads the switch's state. False when the
// task did not start: the lines go unwatched and the status stays default.
bool driveLinkBegin();

// Any task.
DriveLinkStatus driveLinkStatus();

// True once per DRV_ALM alarm the drive-link task raised. The hub task only:
// the delegate takes it in tick() and latches ESTOP.
bool driveAlarmTake();

// An FC 0x03 read for any task, answered through driveLinkTakeResult() with
// the same tag. False when the queue is full. A request reaching a link that
// is not up is answered at once, status not_sent.
struct DriveRead {
    uint32_t tag = 0;
    uint16_t reg = 0;
    uint8_t count = 1;
};
struct DriveReadResult {
    uint32_t tag = 0;
    modbus::Result result{};
};
bool driveLinkRead(const DriveRead& r);
// One consumer only. False when no answer is waiting.
bool driveLinkTakeResult(DriveReadResult& out);

// The task's stack high-water headroom, bytes; 0 before driveLinkBegin().
uint32_t driveLinkStackFree();

}  // namespace valence
