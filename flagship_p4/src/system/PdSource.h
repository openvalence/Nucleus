#pragma once

// PdSource -- the USB-C PD daughterboard's controller (TPS26750, I2C 0x21 on
// the Qwiic bus) as bytes and decisions: the registers read, the active
// contract, the bus and budget it sets, and the refusal when the contract
// cannot carry the input ceilings
// Constraints:
// - Hardware-free and allocation-free. ValencePdSource.cpp moves the bytes;
//   suite test_pd_source drives this file.
// - Register facts are TI SLVUCR7 (TPS26750 Technical Reference Manual,
//   September 2024): the map (Table 4-1), the framing (Figures 1-2 and 1-3: a
//   byte count leads every block, both directions) and sections 4.1, 4.5-4.7,
//   4.17 and 4.18. Fields are little-endian: data byte 1 carries bits 7:0
//   (section 1.2.2 puts TX_SOURCE_CAPS bits 2:0 in byte 1). SLVUCR7 defers the
//   PDO and RDO layouts to USB PD; the decode below is USB PD R3.2's source
//   PDO and request tables. Cross-checked against Linux
//   drivers/usb/typec/tipd (the TPS25750 sibling): MODE, the INT register
//   offsets, the 11-byte event block, the event bit numbers and the framing.
// - THE MAIN BOARD NEVER NEGOTIATES (Hardware SPEC.md 2026-09-30 two-builds
//   row). Nothing here builds a request, a sink capability or a 4CC task; the
//   host writes INT_MASK1 and INT_CLEAR1 and nothing else.
// - A contract above 36 V is the 48 V build: its buck holds +BUS at 36 V, and
//   the daughterboard keeps +BUS at or under 36 V on either build (same row).
// - Units: millivolts and milliamps in a contract; watts, mm/s and mm/s^2 in
//   the profile.
// See: Hardware flagship/SPEC.md (2026-09-23 supported-input and input-floor
// rows, 2026-09-30 PD rows), docs/board-map.md, bd val-091.31

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <span>

namespace valence::pd {

// ---- register map (SLVUCR7 Table 4-1) ---------------------------------------

// ADCIN1 = ADCIN2 = GND on the daughterboard (Hardware SPEC.md 2026-09-30 rev A
// row).
inline constexpr uint8_t kAddress = 0x21;

inline constexpr uint8_t kRegMode      = 0x03;   // 4 B, ASCII, the identity read
inline constexpr uint8_t kRegIntEvent1 = 0x14;   // 11 B, I2Ct_IRQ events
inline constexpr uint8_t kRegIntMask1  = 0x16;   // 11 B, R/W
inline constexpr uint8_t kRegIntClear1 = 0x18;   // 11 B, a 1 clears that event
inline constexpr uint8_t kRegActivePdo = 0x34;   // 6 B, the source PDO in 31:0
inline constexpr uint8_t kRegActiveRdo = 0x35;   // 12 B, the request in 31:0

inline constexpr size_t kModeBytes   = 4;
inline constexpr size_t kIntBytes    = 11;   // bits 87:0
inline constexpr size_t kObjectBytes = 4;    // one PDO or RDO

// ---- framing ----------------------------------------------------------------

// A read returns [count N][N data bytes]. N is the register's length; the host
// may stop early (SLVUCR7 1.3.1), so a block holds `need` bytes when N covers
// them and they were read.
constexpr bool blockHolds(std::span<const uint8_t> raw, size_t need) {
    return raw.size() >= need + 1 && raw[0] >= need;
}

constexpr uint32_t le32(std::span<const uint8_t, kObjectBytes> b) {
    return uint32_t(b[0]) | uint32_t(b[1]) << 8 | uint32_t(b[2]) << 16 | uint32_t(b[3]) << 24;
}

// Bits hi..lo of v, hi >= lo.
constexpr uint32_t field(uint32_t v, unsigned hi, unsigned lo) {
    return (v >> lo) & (0xFFFFFFFFu >> (31u - (hi - lo)));
}

// ---- MODE (SLVUCR7 4.1) -----------------------------------------------------

// 'APP ' runs the application firmware with every register live. 'BOOT' is a
// dead-battery boot and 'PTCH' patch mode: only MODE, CMD1, DATA1, the three
// INT registers and BOOT_FLAGS answer there. The bytes arrive in reading
// order, 'A' first (TI's tipd driver compares the raw block to "APP ").
enum class Mode : uint8_t { app, boot, patch, foreign };

constexpr Mode decodeMode(std::span<const uint8_t, kModeBytes> m) {
    const auto is = [&](const char (&s)[5]) {
        return m[0] == uint8_t(s[0]) && m[1] == uint8_t(s[1]) && m[2] == uint8_t(s[2]) &&
               m[3] == uint8_t(s[3]);
    };
    if (is("APP ")) return Mode::app;
    if (is("BOOT")) return Mode::boot;
    if (is("PTCH")) return Mode::patch;
    return Mode::foreign;
}

// The MODE bytes as report text: printable ASCII kept, anything else '?'.
constexpr std::array<char, kModeBytes + 1> modeText(std::span<const uint8_t, kModeBytes> m) {
    std::array<char, kModeBytes + 1> t{};
    for (size_t i = 0; i < kModeBytes; ++i) t[i] = (m[i] >= 0x20 && m[i] < 0x7F) ? char(m[i]) : '?';
    return t;
}

// ---- INT_EVENT1 / INT_MASK1 / INT_CLEAR1 (SLVUCR7 4.5-4.7) ------------------

// I2Ct_IRQ (PD_INT) is pulled low while any event set here is also set in
// INT_MASK1. Bytes 1-10 of INT_MASK1 reset to 0; byte 11 (bits 80-87) is armed
// by default. SLVUCR7 4.6 says bytes 1-10 are "enabled through the
// Application Customization Tool", so a host write is NOT promised to take:
// the host reads the mask back and polls the contract when it did not.
inline constexpr unsigned kIntHardReset     = 1;    // PD Hardreset
inline constexpr unsigned kIntPlug          = 3;    // Plug Insert or Removal
inline constexpr unsigned kIntNewContract   = 12;   // New Contract as Consumer
inline constexpr unsigned kIntPowerStatus   = 24;   // Power Status Updated
inline constexpr unsigned kIntCannotProvide = 33;   // Cannot Provide Voltage or Current Error
inline constexpr unsigned kIntPatchLoaded   = 80;   // Patch Loaded (armed by default)

using IntBits = std::array<uint8_t, kIntBytes>;

// The events the host arms and re-reads the contract on.
inline constexpr std::array<unsigned, 5> kContractEvents = {
    kIntHardReset, kIntPlug, kIntNewContract, kIntPowerStatus, kIntCannotProvide};

constexpr bool bitSet(const IntBits& b, unsigned n) {
    return n < kIntBytes * 8 && ((b[n / 8] >> (n % 8)) & 1u) != 0;
}

constexpr void setBit(IntBits& b, unsigned n) {
    if (n < kIntBytes * 8) b[n / 8] = uint8_t(b[n / 8] | (1u << (n % 8)));
}

constexpr bool anySet(const IntBits& b) {
    for (uint8_t x : b)
        if (x != 0) return true;
    return false;
}

// Read-modify-write (SLVUCR7 chapter 3): every bit already set stays set.
constexpr IntBits withContractEvents(IntBits mask) {
    for (unsigned n : kContractEvents) setBit(mask, n);
    return mask;
}

constexpr bool armsContractEvents(const IntBits& mask) {
    for (unsigned n : kContractEvents)
        if (!bitSet(mask, n)) return false;
    return true;
}

constexpr bool anyContractEvent(const IntBits& ev) {
    for (unsigned n : kContractEvents)
        if (bitSet(ev, n)) return true;
    return false;
}

// ---- the active contract (SLVUCR7 4.17, 4.18; USB PD R3.2) ------------------

enum class Supply : uint8_t { none, fixed, battery, variable, pps, epr_avs, spr_avs, reserved };

constexpr const char* supplyName(Supply s) {
    switch (s) {
        case Supply::none:     return "none";
        case Supply::fixed:    return "fixed";
        case Supply::battery:  return "battery";
        case Supply::variable: return "variable";
        case Supply::pps:      return "PPS";
        case Supply::epr_avs:  return "EPR AVS";
        case Supply::spr_avs:  return "SPR AVS";
        case Supply::reserved: return "reserved";
    }
    return "?";
}

struct Contract {
    Supply   supply   = Supply::none;
    uint8_t  position = 0;       // the request's object position, 1-based
    bool     mismatch = false;   // the request flagged a capability mismatch
    uint32_t mv       = 0;       // the bus the source holds: a range's floor
    uint32_t ma       = 0;       // the operating current the source accepted
    uint32_t pdo      = 0;       // raw, for the report
    uint32_t rdo      = 0;

    constexpr float watts() const { return float(mv) * float(ma) * 1e-6f; }
    constexpr bool operator==(const Contract&) const = default;
};

// pdo all zero is no explicit contract (SLVUCR7 4.17). A range supply is held
// at its floor and a battery request's power is spread over that floor, both
// the conservative reading. An augmented request carries its own voltage.
constexpr Contract decodeContract(uint32_t pdo, uint32_t rdo) {
    Contract c;
    c.pdo = pdo;
    c.rdo = rdo;
    if (pdo == 0) return c;
    c.position = uint8_t(field(rdo, 31, 28));
    c.mismatch = field(rdo, 26, 26) != 0;
    switch (field(pdo, 31, 30)) {
        case 0:   // fixed: 50 mV; the request's operating current, 10 mA
            c.supply = Supply::fixed;
            c.mv = field(pdo, 19, 10) * 50u;
            c.ma = field(rdo, 19, 10) * 10u;
            break;
        case 1:   // battery: floor 50 mV; the request's operating power, 250 mW
            c.supply = Supply::battery;
            c.mv = field(pdo, 19, 10) * 50u;
            c.ma = c.mv != 0 ? field(rdo, 19, 10) * 250u * 1000u / c.mv : 0u;
            break;
        case 2:   // variable: floor 50 mV; the request's operating current, 10 mA
            c.supply = Supply::variable;
            c.mv = field(pdo, 19, 10) * 50u;
            c.ma = field(rdo, 19, 10) * 10u;
            break;
        default:
            switch (field(pdo, 29, 28)) {
                case 0:   // SPR PPS request: 20 mV, 50 mA
                    c.supply = Supply::pps;
                    c.mv = field(rdo, 20, 9) * 20u;
                    c.ma = field(rdo, 6, 0) * 50u;
                    break;
                case 1:   // AVS request, EPR or SPR alike: 25 mV, 50 mA
                case 2:
                    c.supply = field(pdo, 29, 28) == 1 ? Supply::epr_avs : Supply::spr_avs;
                    c.mv = field(rdo, 20, 9) * 25u;
                    c.ma = field(rdo, 6, 0) * 50u;
                    break;
                default:
                    c.supply = Supply::reserved;
                    break;
            }
            break;
    }
    return c;
}

// ---- the bus and budget a contract sets -------------------------------------

// The supported input (Hardware SPEC.md 2026-09-23 rows): never under 24 V,
// never over 36 V at the main board.
inline constexpr uint32_t kFloorMv  = 24000;
inline constexpr uint32_t kBusMaxMv = 36000;
// The 48 V build's buck (LM5148, ~7 W lost at 240 W; SPEC 2026-09-30 two-builds
// row), rounded down.
inline constexpr float kBuckEfficiency = 0.95f;
// The share of the contract a motion peak may draw. PD sources shut off rather
// than sag (SPEC 2026-09-23 input-floor row), so the rest covers the source's
// own tolerance and the cable.
inline constexpr float kContractShare = 0.9f;

struct Profile {
    uint32_t bus_mv   = 0;       // +BUS under this contract
    float    budget_w = 0.0f;    // what a motion peak may draw from the source
    bool     buck     = false;   // the 48 V build: the contract feeds a buck
};

constexpr Profile profileFor(const Contract& c) {
    Profile p;
    p.buck = c.mv > kBusMaxMv;
    p.bus_mv = p.buck ? kBusMaxMv : c.mv;
    p.budget_w = c.watts() * (p.buck ? kBuckEfficiency : 1.0f) * kContractShare;
    return p;
}

// ---- what the input ceilings draw at a peak ---------------------------------
// P = kIdleW + kMovingMassKg * a * v / kDriveEfficiency, v and a the input
// set's ceilings in m/s and m/s^2: the force that accelerates the moving mass
// times the top speed it may still be accelerating at, through the drive's
// losses, plus everything else the source feeds. A CEILING, conservative by
// construction: a plan rarely holds both ceilings at once.
// THE CALIBRATION KNOBS, unmeasured on a Flagship: no rotor inertia or pulley
// figure is on record (design-considerations.md section 13).
// TODO(val-091.69): fit kMovingMassKg from the INA228 POWER peak across a
// chase at known ceilings, and kIdleW from the source's draw at rest.
inline constexpr float kMovingMassKg    = 2.0f;    // carriage, payload, rotor reflected
inline constexpr float kDriveEfficiency = 0.75f;   // drive and motor at a peak
inline constexpr float kIdleW           = 15.0f;   // logic, fan, drive standby, accessories

// Evaluated at the factory speed ceiling (1000 mm/s), the highest input accel
// each contract carries:
//   contract           budget    accel ceiling carried
//   24 V x 3 A         64.8 W    19,600 mm/s^2
//   36 V x 3 A         97.2 W    32,400 mm/s^2
//   24 V x 5 A        108.0 W    36,700 mm/s^2
//   28 V x 5 A        126.0 W    43,800 mm/s^2
//   36 V x 5 A        162.0 W    58,000 mm/s^2
//   48 V x 5 A, buck  205.2 W    75,000 mm/s^2
// The factory ceilings (1000 mm/s, 50,000 mm/s^2) peak at 148.3 W: a 36 V or a
// 48 V source at 5 A carries them, a 28 V one does not.

struct Ceilings {
    float vmax_mm_s  = 0.0f;
    float amax_mm_s2 = 0.0f;
};

// A ceiling that is NaN or negative peaks at infinity: refused, never guessed.
constexpr float peakWatts(const Ceilings& c) {
    if (!(c.vmax_mm_s >= 0.0f) || !(c.amax_mm_s2 >= 0.0f)) return std::numeric_limits<float>::infinity();
    return kIdleW + kMovingMassKg * (c.amax_mm_s2 * 1e-3f) * (c.vmax_mm_s * 1e-3f) / kDriveEfficiency;
}

// ---- the verdict ------------------------------------------------------------

// What the host found at 0x21. absent is a NACK: no daughterboard, a DC-input
// build. bus_error is anything else that kept the probe from an answer.
enum class Presence : uint8_t { absent, bus_error, foreign, not_ready, ready };

struct Reading {
    Presence presence = Presence::absent;
    std::array<char, kModeBytes + 1> mode_text{};   // modeText() of what MODE returned
    uint32_t bus_err  = 0;                          // the host's I2C error code
    Contract contract{};

    constexpr bool operator==(const Reading&) const = default;
};

enum class Verdict : uint8_t {
    absent,        // no daughterboard: a DC-input build, nothing to judge
    carries,       // the contract carries the input ceilings' peak
    over_budget,   // the peak is over the contract's budget
    no_contract,   // the controller is up with no usable explicit contract
    under_floor,   // a contract under 24 V: the input switch holds the bus off
    not_ready,     // BOOT or PTCH: the application configuration is not loaded
    foreign,       // 0x21 answers but MODE is not a TPS26750's
    bus_error,     // the Qwiic bus kept the probe from any answer
};

struct Assessment {
    Verdict  verdict = Verdict::absent;
    Reading  reading{};
    Profile  profile{};
    Ceilings ceilings{};
    float    peak_w = 0.0f;

    // The motor switch's source verdict. absent is a DC-input build and
    // allowed; everything but a carried contract on a fitted board refuses.
    constexpr bool motorAllowed() const {
        return verdict == Verdict::absent || verdict == Verdict::carries;
    }
};

constexpr Assessment assess(const Reading& r, const Ceilings& c) {
    Assessment a;
    a.reading = r;
    a.ceilings = c;
    a.peak_w = peakWatts(c);
    switch (r.presence) {
        case Presence::absent:    a.verdict = Verdict::absent;    return a;
        case Presence::bus_error: a.verdict = Verdict::bus_error; return a;
        case Presence::foreign:   a.verdict = Verdict::foreign;   return a;
        case Presence::not_ready: a.verdict = Verdict::not_ready; return a;
        case Presence::ready:     break;
    }
    const Contract& k = r.contract;
    if (k.supply == Supply::none || k.supply == Supply::reserved || k.ma == 0) {
        a.verdict = Verdict::no_contract;
        return a;
    }
    if (k.mv < kFloorMv) {
        a.verdict = Verdict::under_floor;
        return a;
    }
    a.profile = profileFor(k);
    // Written so a NaN peak reads over: !(peak <= budget).
    a.verdict = a.peak_w <= a.profile.budget_w ? Verdict::carries : Verdict::over_budget;
    return a;
}

// One line naming the contract or its absence. Inside the catalog's ceiling
// bounds every verdict fits the self-check's 72-byte reason column; past them
// the line is cut, never overrun.
inline void describe(std::span<char> out, const Assessment& a) {
    if (out.empty()) return;
    const Contract& k = a.reading.contract;
    const double v = double(k.mv) * 1e-3;
    const double i = double(k.ma) * 1e-3;
    int n = 0;
    switch (a.verdict) {
        case Verdict::absent:
            n = std::snprintf(out.data(), out.size(), "no PD daughterboard (0x21 silent): DC input");
            break;
        case Verdict::bus_error:
            n = std::snprintf(out.data(), out.size(), "Qwiic bus error 0x%lx at 0x21: PD source unknown",
                              static_cast<unsigned long>(a.reading.bus_err));
            break;
        case Verdict::foreign:
            n = std::snprintf(out.data(), out.size(), "0x21 is not a TPS26750 (MODE \"%s\"): address clash?",
                              a.reading.mode_text.data());
            break;
        case Verdict::not_ready:
            n = std::snprintf(out.data(), out.size(), "TPS26750 in %s mode: no application config loaded",
                              a.reading.mode_text.data());
            break;
        case Verdict::no_contract:
            n = std::snprintf(out.data(), out.size(), "TPS26750 up, no usable PD contract (PDO %08lx)",
                              static_cast<unsigned long>(k.pdo));
            break;
        case Verdict::under_floor:
            n = std::snprintf(out.data(), out.size(), "%.1f V %s contract under the 24 V floor: bus off", v,
                              supplyName(k.supply));
            break;
        case Verdict::carries:
        case Verdict::over_budget:
            n = std::snprintf(out.data(), out.size(), "%.1fV %.2fA %s #%u%s: peak %.0f W %s %.0f W budget", v, i,
                              supplyName(k.supply), unsigned(k.position), a.profile.buck ? " via buck" : "",
                              double(a.peak_w), a.verdict == Verdict::carries ? "within" : "over",
                              double(a.profile.budget_w));
            break;
    }
    if (n < 0) out[0] = '\0';
}

}  // namespace valence::pd
