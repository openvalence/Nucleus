#pragma once

// SelfCheck -- the boot self-check table: named checks in a fixed order, each
// with a result and a reason, and the one rule that opens motor power
// Constraints:
// - Hardware-free and allocation-free. ValenceSelfCheck.cpp does the reads
//   and records them here; suite test_self_check drives this file directly.
// - THE GATE IS "EVERY ENTRY PASSED". Skipped is not passed and pending is not
//   passed: a check the firmware cannot run yet holds motor power off and
//   says why (Hardware SPEC.md 2026-09-23 board-monitor ruling, val-091.21).
// - Order is the ruling's order and the report order. The first entry that
//   did not pass is the one a one-line summary names.
// - kNames are log text, not catalog strings: renaming one changes no etag.
// - Reasons truncate at kReasonBytes - 1; a long reason is cut, never lost.
// - sizeof(Table) is ~1 KB. Construct it once, in static storage; never reset
//   it by assigning a temporary (cpp-safety.md T1).
// See: bd val-091.21, docs/board-map.md

#include <array>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>

namespace valence::selfcheck {

// ---- the table's vocabulary -------------------------------------------------

enum class Result : uint8_t { pending, pass, fail, skipped };

// Declaration order IS run and report order.
enum class Check : uint8_t {
    board_monitor,   // U12 answers on the private bus, image current
    rails,           // +12V, +5V, +5V_SYS, accessory rails in window (U12)
    power_monitor,   // U11 identified and its readings sane
    bus_window,      // input stage and +BUS inside the supply profile (U12)
    motor_rail_off,  // MOTOR_V+ near 0 V with the switch commanded off
    switch_fault,    // motor switch controller reports no fault
    regen_clamp,     // clamp test pulse seen on CLAMP_MON (U12 command)
    estop,           // external E-stop present and released
    quadrature,      // LP emitter rendered edges since boot
    drive_link,      // drive answers on Modbus (needs motor power)
    host_link,       // C6 up over SDIO and answering
    nvs,             // NVS partition initialized and readable
    trust_ledger,    // trust ledger loaded from NVS
    catalog,         // catalog built and encoded
    count_,
};

inline constexpr size_t kCheckCount = size_t(Check::count_);
inline constexpr size_t kReasonBytes = 72;

inline constexpr std::array<const char*, kCheckCount> kNames = {
    "board-monitor", "rails", "power-monitor", "bus-window", "motor-rail-off",
    "switch-fault", "regen-clamp", "estop", "quadrature", "drive-link",
    "host-link", "nvs", "trust-ledger", "catalog",
};

constexpr const char* name(Check c) {
    return size_t(c) < kCheckCount ? kNames[size_t(c)] : "?";
}

constexpr const char* resultName(Result r) {
    switch (r) {
        case Result::pending: return "PENDING";
        case Result::pass:    return "PASS";
        case Result::fail:    return "FAIL";
        case Result::skipped: return "SKIPPED";
    }
    return "?";
}

// ---- the table --------------------------------------------------------------

struct Entry {
    Result result = Result::pending;
    std::array<char, kReasonBytes> reason{};
};

class Table {
public:
    // Re-recording a check overwrites it: the last reading is the truth.
    [[gnu::format(printf, 4, 5)]]
    void record(Check c, Result r, const char* fmt, ...) {
        if (size_t(c) >= kCheckCount) return;
        Entry& e = _entries[size_t(c)];
        e.result = r;
        va_list ap;
        va_start(ap, fmt);
        const int n = std::vsnprintf(e.reason.data(), e.reason.size(), fmt, ap);
        va_end(ap);
        if (n < 0) e.reason[0] = '\0';
    }

    const Entry& entry(Check c) const {
        return _entries[size_t(c) < kCheckCount ? size_t(c) : 0];
    }

    std::optional<Check> firstBlocking() const {
        for (size_t i = 0; i < kCheckCount; ++i)
            if (_entries[i].result != Result::pass) return Check(i);
        return std::nullopt;
    }

    bool motorPowerAllowed() const { return !firstBlocking().has_value(); }

    size_t count(Result r) const {
        size_t n = 0;
        for (const Entry& e : _entries) n += (e.result == r) ? 1u : 0u;
        return n;
    }

private:
    std::array<Entry, kCheckCount> _entries{};
};

// ---- pure decisions the host feeds raw readings into ------------------------

// The external E-stop's two contacts as the P4 reads them through the 2.2k
// pull-ups (Hardware SPEC.md 2026-09-23 E-stop row): HIGH means open.
enum class EStop : uint8_t { normal, pressed, unplugged, wiring_fault };

constexpr EStop decodeEStop(bool ncHigh, bool noHigh) {
    if (!ncHigh && noHigh) return EStop::normal;
    if (ncHigh && !noHigh) return EStop::pressed;
    if (ncHigh && noHigh) return EStop::unplugged;
    return EStop::wiring_fault;
}

// THE CALIBRATION KNOBS, unmeasured on a Flagship.
// TODO(val-091.21): set both from the first board's bench readings.
// MOTOR_V+ with the switch off: the drive's input bank bleeds toward 0 V; a
// shorted switch FET or a fitted budget link (JP401) reads the full bus.
inline constexpr float kMotorRailOffMaxV = 2.0f;
// Motor current with the switch off: anything above the INA237's few-LSB
// noise floor means current is flowing where none can.
inline constexpr float kIdleCurrentMaxA = 0.5f;
// Die temperature a working part can report at all (datasheet range).
inline constexpr float kDieMinC = -40.0f;
inline constexpr float kDieMaxC = 125.0f;

}  // namespace valence::selfcheck
