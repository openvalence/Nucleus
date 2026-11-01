#pragma once

// BarnacleLink -- nucleus-p4b's remote board I/O: the WROOM-32D helper on the
// OSSM R+D board, reached over the barnacle link (lib/barnacle_link), and the
// TRIP line beside it
// Constraints:
// - Barnacle only (CONFIG_NUCLEUS_BOARD_BARNACLE). BarnacleLink.cpp is the
//   board's host of THREE existing doors, so nothing above them changes:
//   ValenceDriveLink.h (the link's task; driveAlarmTake() is always false
//   here), ValenceHomeSense.h (TRIP as the stall level) and this file's
//   helper surface. BarnacleBoard.cpp hosts the doors for the parts the R+D
//   board does not have.
// - NOTHING HERE LATCHES ESTOP (operator ruling 2026-10-30, bd val-4fgu: an
//   ESTOP is only the operator's hand or the Flagship board monitor's
//   critical failure). The helper's current trip is armed only for homing:
//   running and idle send kRunTripMa, 0, which the helper's TripDetector
//   reads as off. A TRIP outside a home cycle is logged as an anomaly line
//   and nothing else.
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
// - A helper gone stale during a home cycle fails the home: the stall the
//   cycle waits on is the helper's, so the home sense reports it lost
//   (HomeSense::lost()) and the arbiter unloads a sensed leg, then fails the
//   cycle unhomed with that reason. Stale while running is a health message
//   only: the drive keeps its own overcurrent protection.
// - The helper's firmware update runs on the same task, idle only
//   (BarnacleLink.cpp helperUpdateRefusal(), lib/barnacle_link
//   barnacle_update.hpp): this firmware carries the helper's image.
// - Declarations and hardware-free policy only: the native suite compiles
//   this file (test_barnacle_link).
// See: lib/barnacle_link/include/barnacle_link/barnacle_link.hpp, BoardPins.h
// (the barnacle section), openvalence.github.io guides/barnacle, bd val-omh3

#include <array>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>

#include "barnacle_link/barnacle_link.hpp"

namespace valence {

// ---- THE KNOBS ------------------------------------------------------------------

// The trip thresholds sent to the helper. Homing: the link's default
// (barnacle_link.hpp), until a setting exists for it (bd val-omh3.5).
// Running and idle: 0, the trip off (bd val-4fgu); never a jam detector.
inline constexpr uint16_t kHomeTripMa = barnacle::kDefaultHomeTripMa;
inline constexpr uint16_t kRunTripMa = 0;

// The TRIP line and the helper's own trip flag may disagree for this many
// consecutive passes before it is logged as wiring: a trip changes between a
// status frame and the pad read.
inline constexpr uint32_t kTripDisagreePasses = 10;

// The rest current a home starts from: the mean of the last kRestFrames
// status currents (128 ms at 500 Hz, about 850 helper samples), the stock
// OSSM firmware's 1000-read zero at every home start (barnacle_link.hpp).
inline constexpr uint32_t kRestFrames = 64;

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
    int16_t rest_ma = barnacle::kNoCurrent;   // kRestFrames mean of status.current_ma
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

// Blocks the calling task until the helper reports a current (its 0 A zero
// taken, barnacle::CurrentZero) or kHelperZeroWaitMs passes; returns early
// while no frame has arrived kPairAtBootWaitMs in. Call only while the motion
// gate is still closed: the helper zeroes only in Mode::idle, and the open
// gate is Mode::running for the rest of the boot.
inline constexpr uint32_t kHelperZeroWaitMs = 8000;
void helperAwaitZero();

// ---- hardware-free policy ---------------------------------------------------------

// The mode the helper applies: homing through a home cycle, running while the
// motion gate is open and no ESTOP holds, else idle.
constexpr barnacle::Mode modeFor(bool homing, bool gateOpen, bool estop) {
    if (homing) return barnacle::Mode::homing;
    if (gateOpen && !estop) return barnacle::Mode::running;
    return barnacle::Mode::idle;
}

// The home trip the helper applies: `trip_ma` over the rest current read
// before the home, so the trip is measured from the rest, as the stock OSSM
// firmware's re-zero at every home start makes it. restCurrentSane() has held
// the rest under half the trip before any home starts; never 0, which
// disables the trip.
constexpr uint16_t homeTripFrom(uint16_t trip_ma, int16_t rest_ma) {
    if (rest_ma == barnacle::kNoCurrent) return trip_ma;
    const int32_t t = int32_t(trip_ma) + rest_ma;
    return uint16_t(t < 1 ? 1 : (t > 65535 ? 65535 : t));
}

// TRIP outside a home cycle, once per trip: an anomaly to log, never a stop.
// A trip that began homing reports when the cycle ends under it. SINGLE
// OWNER: the link task.
class StrayTripWatch {
public:
    bool sample(bool tripped, barnacle::Mode m) {
        if (!tripped) {
            _raised = false;
            return false;
        }
        if (m == barnacle::Mode::homing || _raised) return false;
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
// a slow frame and a current (the helper's 0 A zero taken). Writes the reason
// into `out`.
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
    const bool zeroed = h.status.current_ma != barnacle::kNoCurrent;
    std::array<char, 40> current{};
    if (!zeroed)
        std::snprintf(current.data(), current.size(), "no current: no ADC or 0 A zero");
    else
        std::snprintf(current.data(), current.size(), "%d %s", int(h.status.current_ma),
                      (h.status.flags & barnacle::kFlagCalibrated) != 0 ? "mA" : "mA provisional");
    if (h.status.motor_mv == barnacle::kNoMillivolts)
        std::snprintf(out.data(), out.size(), "helper v%u boot %u: no motor V, %s",
                      unsigned(h.slow.shim_version), unsigned(h.slow.boots), current.data());
    else
        std::snprintf(out.data(), out.size(), "helper v%u boot %u: %.1f V, %s",
                      unsigned(h.slow.shim_version), unsigned(h.slow.boots),
                      double(h.status.motor_mv) / 1000.0, current.data());
    return zeroed;
}

}  // namespace valence
