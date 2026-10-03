#pragma once

// ValenceSelfCheck -- the boot self-check's IDF host: holds motor power off
// from the first line of app_main, reads the board into the SelfCheck table,
// and names every result over Valence
// Constraints:
// - Owner: app_main, and only app_main. selfCheckHoldMotorOff() is its FIRST
//   call; selfCheckRun() runs once, after hubBegin() and logBridgeBegin();
//   selfCheckSummary(), selfCheckRemind() and selfCheckDriveLink() run from
//   the liveness loop. No other task touches the table (cpp-safety.md
//   concurrency).
// - MOTOR_EN, PRECHARGE_EN and ESTOP_BYP are driven LOW here and never high
//   here. Raising the first two is the motor switch module's alone
//   (ValenceMotorSwitch.h), and app_main hands it this table's verdict: a
//   table that did not pass holds motor power off for the boot.
// - Results ride the log channel 0x0008 through ValenceLogBridge: FAIL is
//   GLOGE, SKIPPED is GLOGW, both on the wire; PASS is GLOGI, console and
//   /diag only. Tag "selfcheck".
// - selfCheckRun() blocks on the private I2C bus and on nothing else: the
//   power monitor's read (ValencePower.h) and two board-monitor blocks at a
//   10 ms timeout each (Supervisor.h). Its state is the ~1 KB table in BSS.
// See: SelfCheck.h, BoardPins.h, bd val-091.21

#include <cstdint>

namespace valence {

// What app_main already knows when the check runs; the host reads the rest.
struct SelfCheckFacts {
    uint32_t lpEdges = 0;    // ulp_g_edges: edges the LP core rendered since boot
    bool hostLink = false;   // esp_hosted connected and the C6 reported its version
};

// Drives MOTOR_EN, PRECHARGE_EN and ESTOP_BYP low and configures MSW_FLT_N as
// an input; the e-stop row reads ValenceEstopInput.h. Call before anything
// else in app_main.
void selfCheckHoldMotorOff();

// Runs every check in table order and logs each result plus a summary.
// Returns motorPowerAllowed() of the finished table.
bool selfCheckRun(const SelfCheckFacts& facts);

struct SelfCheckSummary {
    bool ran = false;
    bool allowed = false;
    uint8_t failed = 0;
    uint8_t skipped = 0;
    const char* first = "";   // name of the first entry that did not pass
};
SelfCheckSummary selfCheckSummary();

// Re-logs the one-line summary at Warn while motor power is held off, so a
// client that connects long after boot still finds it in the replay ring.
void selfCheckRemind();

// The post-enable drive-link row (SelfCheck.h postEnable()): records and logs
// the drive link's probe verdict whenever it changes. Never moves the gate.
void selfCheckDriveLink();

}  // namespace valence
