#pragma once

// SelfCheck -- the boot self-check table: named checks in a fixed order, each
// with a result and a reason, and the one rule that opens motor power
// Constraints:
// - Hardware-free and allocation-free. ValenceSelfCheck.cpp does the reads
//   and records them here; suite test_self_check drives this file directly.
// - THE GATE IS "EVERY PRE-ENABLE ENTRY PASSED". Skipped is not passed and
//   pending is not passed: a check the firmware cannot run yet holds motor
//   power off and says why (Hardware SPEC.md 2026-09-23 board-monitor ruling,
//   val-091.21). The one POST-ENABLE entry, drive_link, never gates: the
//   drive's logic rides the motor bus, so it can only be probed after the
//   enable it would otherwise block (postEnable(), val-091.29). Its FAIL is
//   reported, never adjudicated.
// - Order is the ruling's order and the report order. The first entry that
//   did not pass is the one a one-line summary names.
// - kNames are log text, not catalog strings: renaming one changes no etag.
// - Reasons truncate at kReasonBytes - 1: a long reason is cut short, never
//   refused.
// - sizeof(Table) is ~1 KB. Construct it once, in static storage; never reset
//   it by assigning a temporary (cpp-safety.md T1).
// See: bd val-091.21, docs/board-map.md

#include <array>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>

#include "AimDrive.h"
#include "Supervisor.h"

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
    drive_link,      // drive answers its map on Modbus; POST-ENABLE
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

// The one check that runs AFTER motor power, so it never gates it (file
// header).
constexpr bool postEnable(Check c) { return c == Check::drive_link; }

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

    // The first pre-enable entry that did not pass: the gate, and the name a
    // one-line summary gives.
    std::optional<Check> firstBlocking() const {
        for (size_t i = 0; i < kCheckCount; ++i)
            if (!postEnable(Check(i)) && _entries[i].result != Result::pass) return Check(i);
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
// The E-stop contacts' decode is EstopInput.h's: the runtime reader and this
// table's estop row share one reading.

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
// +BUS window for the 24-36 V input ruling (Hardware SPEC.md 2026-09-23), 10 %
// either side. TODO(val-091.31): narrow it to the profile the supply reports.
inline constexpr uint16_t kBusMinMv = 21600;
inline constexpr uint16_t kBusMaxMv = 39600;

// ---- the board monitor's blocks (Supervisor.h) ------------------------------
// The host reads IDENT, then STATUS, and hands the decode results here. Each
// judge records exactly one entry.

inline void judgeMonitorIdent(Table& t, int decodeResult, const SvIdent& id) {
    if (decodeResult != SV_OK) {
        t.record(Check::board_monitor, Result::fail,
                 "IDENT block rejected (sv error %d): corrupt or wrong image", decodeResult);
        return;
    }
    if (id.link_version != SV_LINK_VERSION) {
        t.record(Check::board_monitor, Result::fail, "link v%u, this P4 speaks v%u",
                 unsigned(id.link_version), unsigned(SV_LINK_VERSION));
        return;
    }
    t.record(Check::board_monitor, Result::pass, "fw %u.%u.%u, link v%u, image %08lx",
             unsigned(id.fw_major), unsigned(id.fw_minor), unsigned(id.fw_patch),
             unsigned(id.link_version), static_cast<unsigned long>(id.image_crc32));
}

// SV_F_BOOT and SV_F_VDD make every reading untrusted, so both rails and the
// bus window refuse to pass on them.
inline bool monitorReadingsTrusted(Table& t, Check c, const SvStatus& s) {
    if (s.faults_live & SV_F_BOOT) {
        t.record(c, Result::skipped, "monitor's first full scan not complete");
        return false;
    }
    if (s.faults_live & SV_F_VDD) {
        t.record(c, Result::fail, "monitor supply out of window: readings untrusted");
        return false;
    }
    return true;
}

// +12V and +5V_SYS are motion-critical (monitor faults); +5V and +3V3_ACC
// only warn, so they name themselves in a passing reason.
inline void judgeRails(Table& t, const SvStatus& s) {
    if (!monitorReadingsTrusted(t, Check::rails, s)) return;
    if (s.faults_live & (SV_F_12V | SV_F_5V_SYS)) {
        t.record(Check::rails, Result::fail, "out of window: +12V %u mV, +5V_SYS %u mV",
                 unsigned(s.mv[SV_CH_12V]), unsigned(s.mv[SV_CH_5V_SYS]));
        return;
    }
    const bool warn = (s.warns & (SV_W_5V | SV_W_3V3_ACC)) != 0;
    t.record(Check::rails, Result::pass, "12V %u, 5V %u, 5VSYS %u, 3V3A %u mV%s",
             unsigned(s.mv[SV_CH_12V]), unsigned(s.mv[SV_CH_5V]),
             unsigned(s.mv[SV_CH_5V_SYS]), unsigned(s.mv[SV_CH_3V3_ACC]),
             warn ? " (accessory rail warn)" : "");
}

inline void judgeBusWindow(Table& t, const SvStatus& s) {
    if (!monitorReadingsTrusted(t, Check::bus_window, s)) return;
    const unsigned vin = s.mv[SV_CH_VIN_RAW];
    const unsigned bus = s.mv[SV_CH_BUS];
    if (s.faults_live & SV_F_INPUT_STAGE) {
        t.record(Check::bus_window, Result::fail,
                 "input stage: VIN_RAW %u mV present, +BUS %u mV not following", vin, bus);
        return;
    }
    if (s.faults_live & SV_F_BUS_OV) {
        t.record(Check::bus_window, Result::fail, "+BUS %u mV above the clamp's reach", bus);
        return;
    }
    if (bus < kBusMinMv || bus > kBusMaxMv) {
        t.record(Check::bus_window, Result::fail, "+BUS %u mV outside %u-%u mV", bus,
                 unsigned(kBusMinMv), unsigned(kBusMaxMv));
        return;
    }
    t.record(Check::bus_window, Result::pass, "+BUS %u mV, VIN_RAW %u mV%s", bus, vin,
             (s.flags & SV_FLAG_BUDGET) ? " (budget fuse position)" : "");
}

// The clamp TEST is not sequenced yet; a live clamp fault still fails.
inline void judgeRegenClamp(Table& t, const SvStatus& s) {
    if (s.faults_live & SV_F_SHUNT_HOT) {
        t.record(Check::regen_clamp, Result::fail, "regen load past its trip temperature");
        return;
    }
    if (s.faults_live & SV_F_CLAMP_STUCK) {
        t.record(Check::regen_clamp, Result::fail, "clamp conducting with no regen");
        return;
    }
    t.record(Check::regen_clamp, Result::skipped,
             "clamp test pulse not sequenced yet (val-091.21)");
}

// ---- the trust ledger (TrustStore.h's boot result) --------------------------
// hubUp: the hub built, so its ledger load ran. A stored ledger that did not
// load is a FAIL: NVS holds pairings the hub cannot read, and the hub runs
// with none. No ledger stored at all is a factory-fresh hub and passes.

struct LedgerBoot {
    bool hubUp = false;
    bool loaded = false;     // a stored ledger was read and decoded
    bool rejected = false;   // one was stored and did not load
    unsigned paired = 0;     // ledger entries after the load
};

inline void judgeTrustLedger(Table& t, const LedgerBoot& b) {
    if (!b.hubUp) {
        t.record(Check::trust_ledger, Result::fail, "hub did not start: no ledger loaded");
        return;
    }
    if (b.rejected) {
        t.record(Check::trust_ledger, Result::fail,
                 "stored ledger unreadable or rejected: running with no pairings");
        return;
    }
    if (b.loaded) {
        t.record(Check::trust_ledger, Result::pass, "%u paired", b.paired);
        return;
    }
    t.record(Check::trust_ledger, Result::pass, "factory fresh: no ledger stored");
}

// ---- the drive link (post-enable: AimDrive.h's probe) -------------------------
// One entry per probe outcome, each naming its cause. The host records it
// again whenever the outcome changes.

inline void judgeDriveLink(Table& t, const aim::Probe& p) {
    using aim::ProbeOutcome;
    static_assert(aim::kBauds.size() == 4, "the no-answer reason names four bauds");
    const unsigned long baud = p.baud;
    const unsigned reg = p.reg;
    const char* regName = aim::registerName(p.reg);
    switch (p.outcome) {
        case ProbeOutcome::pending:
            t.record(Check::drive_link, Result::pending,
                     "probed after motor power: the drive rides the motor bus");
            return;
        case ProbeOutcome::no_answer:
            t.record(Check::drive_link, Result::fail, "no answer at %lu, %lu, %lu or %lu baud, address %u",
                     static_cast<unsigned long>(aim::kBauds[0]), static_cast<unsigned long>(aim::kBauds[1]),
                     static_cast<unsigned long>(aim::kBauds[2]), static_cast<unsigned long>(aim::kBauds[3]),
                     unsigned(aim::kAddress));
            return;
        case ProbeOutcome::crc:
            t.record(Check::drive_link, Result::fail,
                     "frames at %lu baud, none valid: noise, A/B swapped, termination", baud);
            return;
        case ProbeOutcome::wrong_model:
            if (p.exception != 0)
                t.record(Check::drive_link, Result::fail, "not the AIM map: exception %u reading 0x%02x %s",
                         unsigned(p.exception), reg, regName);
            else
                t.record(Check::drive_link, Result::fail, "not the AIM map: 0x%02x %s reads %u", reg, regName,
                         unsigned(p.value));
            return;
        case ProbeOutcome::lost:
            t.record(Check::drive_link, Result::fail, "answered at %lu baud, then silent at 0x%02x %s", baud,
                     reg, regName);
            return;
        case ProbeOutcome::modbus_enabled:
            t.record(Check::drive_link, Result::fail, "0x00 modbus-enable = %u: the drive ignores quadrature",
                     unsigned(p.modbusEnable));
            return;
        case ProbeOutcome::output_off:
            t.record(Check::drive_link, Result::fail, "0x01 output-enable = %u: drive output disabled",
                     unsigned(p.outputEnable));
            return;
        case ProbeOutcome::not_quadrature:
            t.record(Check::drive_link, Result::fail,
                     "0x19 special-function = %u, not 2: A/B not read as quadrature",
                     unsigned(p.specialFunction));
            return;
        case ProbeOutcome::alarm_inverted:
            t.record(Check::drive_link, Result::fail,
                     "0x07 position-kp = %u is odd: WR normally closed, DRV_ALM inverted",
                     unsigned(p.positionKp));
            return;
        case ProbeOutcome::ok: {
            const unsigned stall = aim::stallAlarmDigit(p.standstill);
            if (stall == 0)
                t.record(Check::drive_link, Result::pass,
                         "AIM at %lu, %u C, encoder %ld, alarm 0x%02x, stall alarm off", baud,
                         unsigned(p.temperatureC), long(p.encoder), unsigned(p.alarmCode));
            else
                t.record(Check::drive_link, Result::pass,
                         "AIM at %lu, %u C, encoder %ld, alarm 0x%02x, stall alarm %u", baud,
                         unsigned(p.temperatureC), long(p.encoder), unsigned(p.alarmCode), stall);
            return;
        }
    }
}

}  // namespace valence::selfcheck
