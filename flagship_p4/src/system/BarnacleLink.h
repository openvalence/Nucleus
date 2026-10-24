#pragma once

// BarnacleLink -- nucleus-p4b's remote board I/O: the WROOM-32D helper on the
// OSSM R+D board, reached over the barnacle link (lib/barnacle_link), and the
// TRIP line beside it
// Constraints:
// - Barnacle only (CONFIG_NUCLEUS_BOARD_BARNACLE). BarnacleLink.cpp is the
//   board's host of THREE existing doors, so nothing above them changes:
//   ValenceDriveLink.h (the link's task, and TRIP while running as the drive
//   alarm the hub latches ESTOP on), ValenceHomeSense.h (TRIP as the stall
//   level) and this file's helper surface. BarnacleBoard.cpp hosts the doors
//   for the parts the R+D board does not have.
// - Task "DriveLnk" (the drive-link door's task, HP core 0, priority 2,
//   kDriveLinkPeriodMs pass, kDriveLinkTaskStackBytes) owns UART
//   BOARD_UART_LINK, the decoder and every frame. Out of it: one one-slot
//   mailbox (helperReading()) and the alarm flag. Into it: one atomic word
//   (helperSetRgb()).
// - THE HELPER IS ADVISORY FOR MOTION, NEVER TRUSTED STALE. A reading older
//   than barnacle::kHelperStaleMs reads stale: no current, no voltage, no
//   button, and the home sense probes undriven, so home op 1 refuses. Every
//   change of freshness, a helper restart and a TRIP line that disagrees with
//   the helper's own report are logged at Warn or above (the 0x0008 log
//   channel). Nothing recovers silently.
// - A helper gone stale during a home cycle is an ESTOP (cause fault, the
//   drive-alarm path): the stall the cycle waits on is the helper's. Stale
//   while running is a health message only: TRIP is its own wire and the
//   drive keeps its own overcurrent protection.
// - Declarations and hardware-free policy only: the native suite compiles
//   this file (test_barnacle_link).
// See: lib/barnacle_link/include/barnacle_link/barnacle_link.hpp, BoardPins.h
// (the barnacle section), openvalence.github.io guides/barnacle, bd val-omh3

#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>

#include "barnacle_link/barnacle_link.hpp"

namespace valence {

// ---- THE KNOBS ------------------------------------------------------------------

// The trip thresholds sent to the helper: the link's defaults
// (barnacle_link.hpp), until a setting exists for them (bd val-omh3.5).
inline constexpr uint16_t kHomeTripMa = barnacle::kDefaultHomeTripMa;
inline constexpr uint16_t kRunTripMa = barnacle::kDefaultRunTripMa;

// The TRIP line and the helper's own trip flag may disagree for this many
// consecutive passes before it is logged as wiring: a trip changes between a
// status frame and the pad read.
inline constexpr uint32_t kTripDisagreePasses = 10;

// ---- the helper's reading -------------------------------------------------------

struct HelperReading {
    bool fresh = false;            // a valid frame inside barnacle::kHelperStaleMs
    bool ever = false;             // any valid frame since boot
    bool have_slow = false;        // a slow frame since the helper's last restart
    uint32_t age_ms = 0;           // since the last valid frame; 0 before any
    barnacle::Status status{};     // the last one, stale or not: read fresh first
    barnacle::Slow slow{};
    barnacle::RxStats rx{};
    uint32_t lost = 0;             // frames the helper's sequence says were lost
    uint32_t restarts = 0;         // helper restarts seen since this boot
};

// ---- the doors ------------------------------------------------------------------

// Any task: the link task's last published reading, aged now. Never blocks.
HelperReading helperReading();

// Any task: the status pixel's color, gamma applied (ValenceGlow.cpp). The
// next command frame carries it.
void helperSetRgb(uint8_t r, uint8_t g, uint8_t b);

// S2 pressed now; nullopt while the helper is stale.
std::optional<bool> helperButtonPressed();

// app_main, after driveLinkBegin(): S2 held at boot (SPEC 13.4.1 config mode),
// read through the helper. Waits up to kPairAtBootWaitMs for a fresh reading.
inline constexpr uint32_t kPairAtBootWaitMs = 600;
bool barnaclePairHeldAtBoot();

// ---- hardware-free policy ---------------------------------------------------------

// The mode the helper applies: homing through a home cycle, running while the
// motion gate is open and no ESTOP holds, else idle.
constexpr barnacle::Mode modeFor(bool homing, bool gateOpen, bool estop) {
    if (homing) return barnacle::Mode::homing;
    if (gateOpen && !estop) return barnacle::Mode::running;
    return barnacle::Mode::idle;
}

// TRIP while running, once per trip: a trip that began in another mode
// raises when running begins under it. SINGLE OWNER: the link task.
class RunTripWatch {
public:
    bool sample(bool tripped, barnacle::Mode m) {
        if (!tripped) {
            _raised = false;
            return false;
        }
        if (m != barnacle::Mode::running || _raised) return false;
        _raised = true;
        return true;
    }

private:
    bool _raised = false;
};

// A helper restart: its boot count moved, or its uptime ran backwards.
constexpr bool helperRestarted(const barnacle::Slow& was, const barnacle::Slow& now) {
    return now.boots != was.boots || now.uptime_ms < was.uptime_ms;
}

// The self-check's helper row: pass only on a fresh reading that has carried
// a slow frame. Writes the reason into `out`.
inline bool describeHelper(const HelperReading& h, std::span<char> out) {
    if (!h.ever) {
        std::snprintf(out.data(), out.size(), "no frame from the WROOM helper on J8 pin 9");
        return false;
    }
    if (!h.fresh) {
        std::snprintf(out.data(), out.size(), "helper stale: last frame %lu ms ago",
                      static_cast<unsigned long>(h.age_ms));
        return false;
    }
    if (!h.have_slow) {
        std::snprintf(out.data(), out.size(), "helper up, no slow frame yet");
        return false;
    }
    const char* scale = (h.status.flags & barnacle::kFlagCalibrated) != 0 ? "mA" : "mA provisional";
    if (h.status.motor_mv == barnacle::kNoMillivolts)
        std::snprintf(out.data(), out.size(), "helper v%u boot %u: no motor V, %d %s",
                      unsigned(h.slow.shim_version), unsigned(h.slow.boots), int(h.status.current_ma), scale);
    else
        std::snprintf(out.data(), out.size(), "helper v%u boot %u: %.1f V, %d %s",
                      unsigned(h.slow.shim_version), unsigned(h.slow.boots),
                      double(h.status.motor_mv) / 1000.0, int(h.status.current_ma), scale);
    return true;
}

}  // namespace valence
