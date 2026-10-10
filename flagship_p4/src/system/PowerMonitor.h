#pragma once

// PowerMonitor -- hardware-free register layer for the motor current monitor
// (U11), which is EITHER an INA228 or an INA237 on the same footprint
// Constraints:
// - Hardware-free: std headers only. The IDF glue (ValencePower.cpp) moves the
//   bytes; everything that decides what the bytes mean lives here, so the
//   native suite test_power_monitor covers both parts without either fitted.
// - ADCRANGE = 1 (+/-40.96 mV) on BOTH parts. The current LSB is the shunt LSB
//   divided by the shunt resistance, so SHUNT_CAL comes out 4096 on both and
//   the CURRENT register counts track VSHUNT counts one for one.
// - The INA237 datasheet (SBOSA20A) documents NO DEVICE_ID register. 0x3F is
//   read on both parts but never decides alone: the CONFIG TEMPCOMP probe
//   (bit 5 is R/W on the INA228, reserved and read as 0 on the INA237) must
//   agree with it, and decides by itself when the id is unrecognized.
// - The limit registers (SOVL, BOVL) and DIAG_ALRT have the same layout and
//   the same LSBs on both parts; only the measurement registers differ.
// - Units at this surface are SI floats; raw counts never leave this file.
// - The boot and read sequences take a register port, never the bus: the
//   IDF glue passes its I2C device, test_power_monitor a fake register file
//   that models each part's widths and its reserved bits.
// - U11 sits AFTER the motor switch (SPEC 2026-09-23 row): its VBUS is
//   MOTOR_V+ and its current the motor's alone. The system voltage is the
//   board monitor's +BUS; pickPower() is the one place that chooses.
// - BusOwner and PowerHandoff are the bus's runtime ownership: one task reads
//   the private bus, every other task reads the hand-off (ValencePower.h).
// See: Hardware flagship/SPEC.md (2026-09-24 U11 row), bd val-091.22, val-9hr

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace valence {

enum class PowerChip : uint8_t { none, ina228, ina237 };

struct PowerReading {
    float bus_v;       // VBUS on MOTOR_V+
    float current_a;   // motor current; regen reads negative
    float power_w;     // |V x I| as the part computes it, always >= 0
    float die_c;
};

constexpr const char* powerChipName(PowerChip c) {
    switch (c) {
        case PowerChip::ina228: return "INA228";
        case PowerChip::ina237: return "INA237";
        default:                return "none";
    }
}

namespace ina2xx {

// ---- register map (addresses shared by both parts) ---------------------------

inline constexpr uint8_t kRegConfig         = 0x00;
inline constexpr uint8_t kRegAdcConfig      = 0x01;
inline constexpr uint8_t kRegShuntCal       = 0x02;
inline constexpr uint8_t kRegVbus           = 0x05;
inline constexpr uint8_t kRegDieTemp        = 0x06;
inline constexpr uint8_t kRegCurrent        = 0x07;
inline constexpr uint8_t kRegPower          = 0x08;
inline constexpr uint8_t kRegDiagAlrt       = 0x0B;
inline constexpr uint8_t kRegSovl           = 0x0C;
inline constexpr uint8_t kRegBovl           = 0x0E;
inline constexpr uint8_t kRegManufacturerId = 0x3E;
inline constexpr uint8_t kRegDeviceId       = 0x3F;

inline constexpr uint16_t kManufacturerTi = 0x5449;   // "TI"
inline constexpr uint16_t kDieIdIna228    = 0x228;    // DEVICE_ID bits 15:4
inline constexpr uint16_t kDieIdIna237    = 0x237;    // undocumented; see header

// ---- register values written at configure time --------------------------------

inline constexpr uint16_t kConfigReset    = 1u << 15;
inline constexpr uint16_t kConfigTempComp = 1u << 5;   // the identification probe only
inline constexpr uint16_t kConfigAdcRange = 1u << 4;   // +/-40.96 mV
inline constexpr uint16_t kConfig         = kConfigAdcRange;

// Continuous shunt + bus + temperature, 540 us per conversion, 4-sample
// average: a ~6.5 ms cycle. Tuning knob for the homing current loop. ALERT
// compares on every raw conversion regardless (SLOWALERT stays 0).
inline constexpr uint16_t kAdcConfig = (0xFu << 12) | (4u << 9) | (4u << 6) | (4u << 3) | 1u;

// Latched, active-low open drain. LATCHED IS REQUIRED: ALERT pulls the motor
// switch EN low, the motor current then collapses, and a transparent alert
// would release EN on the next conversion and re-close the switch into the
// fault. The latch holds the cut until firmware reads DIAG_ALRT.
inline constexpr uint16_t kDiagAlrtLatch = 1u << 15;

// DIAG_ALRT flag bits, same positions on both parts.
inline constexpr uint16_t kFlagMathOverflow = 1u << 9;
inline constexpr uint16_t kFlagShuntOver    = 1u << 6;
inline constexpr uint16_t kFlagBusOver      = 1u << 4;
inline constexpr uint16_t kFlagMemOk        = 1u << 0;   // 0 = trim checksum error

// ---- scaling --------------------------------------------------------------------

inline constexpr float kShuntFullScaleV = 0.04096f;   // ADCRANGE = 1
inline constexpr float kShuntLimitLsbV  = 1.25e-6f;   // SOVL at ADCRANGE = 1, both parts
inline constexpr float kBusLimitLsbV    = 3.125e-3f;  // BOVL, both parts
inline constexpr float kDieTempLsbC     = 7.8125e-3f; // per raw 16-bit count, both parts

struct PowerScale {
    uint8_t  meas_bytes;      // VBUS and CURRENT width on the wire: 3 or 2
    uint16_t shunt_cal;       // SHUNT_CAL register value
    float    current_lsb_a;   // amperes per CURRENT count
    float    power_lsb_w;     // watts per POWER count
    float    vbus_lsb_v;      // volts per VBUS count
};

// SHUNT_CAL = K x CURRENT_LSB x R_SHUNT x 4 (ADCRANGE = 1), with
// CURRENT_LSB = full-scale current / 2^bits. K is 13107.2e6 on the INA228 and
// 819.2e6 on the INA237 (datasheet eq. 2 and eq. 1).
constexpr PowerScale powerScaleFor(PowerChip chip, float shunt_ohms) {
    const float full_scale_a = kShuntFullScaleV / shunt_ohms;
    if (chip == PowerChip::ina228) {
        const float lsb = full_scale_a / 524288.0f;   // 2^19
        return {3, uint16_t(13107.2e6f * lsb * shunt_ohms * 4.0f + 0.5f),
                lsb, 3.2f * lsb, 195.3125e-6f};
    }
    const float lsb = full_scale_a / 32768.0f;        // 2^15
    return {2, uint16_t(819.2e6f * lsb * shunt_ohms * 4.0f + 0.5f),
            lsb, 0.2f * lsb, 3.125e-3f};
}

// ---- identification ---------------------------------------------------------------

// tempcomp_sticks: CONFIG read back with bit 5 set after writing kConfigTempComp.
constexpr PowerChip identify(uint16_t manufacturer_id, uint16_t device_id, bool tempcomp_sticks) {
    if (manufacturer_id != kManufacturerTi) return PowerChip::none;
    const uint16_t die = uint16_t(device_id >> 4);
    if (die == kDieIdIna228) return tempcomp_sticks ? PowerChip::ina228 : PowerChip::none;
    if (die == kDieIdIna237) return tempcomp_sticks ? PowerChip::none : PowerChip::ina237;
    // An unknown id with a working TEMPCOMP bit is some other TI part: refuse it.
    return tempcomp_sticks ? PowerChip::none : PowerChip::ina237;
}

// ---- raw decoding (big-endian register bytes) ---------------------------------------

constexpr uint16_t be16(std::span<const uint8_t> b) {
    return uint16_t((uint16_t(b[0]) << 8) | b[1]);
}

// VBUS / CURRENT: INA228 is 20-bit two's complement in bits 23:4 of 3 bytes;
// INA237 is 16-bit two's complement in 2 bytes.
constexpr int32_t decodeMeasurement(std::span<const uint8_t> b) {
    if (b.size() == 3) {
        const uint32_t raw = (uint32_t(b[0]) << 16) | (uint32_t(b[1]) << 8) | b[2];
        const uint32_t v20 = raw >> 4;
        return (v20 & 0x80000u) ? int32_t(v20) - 0x100000 : int32_t(v20);
    }
    return int16_t(be16(b));
}

// POWER: 24-bit unsigned on both parts.
constexpr uint32_t decodePower(std::span<const uint8_t> b) {
    return (uint32_t(b[0]) << 16) | (uint32_t(b[1]) << 8) | b[2];
}

// DIETEMP: INA228 is 16-bit at 7.8125 mC; INA237 is 12-bit in bits 15:4 at
// 125 mC with bits 3:0 reading 0, which is the same scale per 16-bit count.
constexpr float decodeDieTempC(std::span<const uint8_t> b) {
    return float(int16_t(be16(b))) * kDieTempLsbC;
}

// ---- limit encoding -----------------------------------------------------------------

constexpr uint16_t encodeShuntOverLimit(float amps, float shunt_ohms) {
    const float counts = amps * shunt_ohms / kShuntLimitLsbV;
    if (counts >= 32767.0f) return 0x7FFF;
    if (counts <= 0.0f) return 0;
    return uint16_t(counts + 0.5f);
}

constexpr uint16_t encodeBusOverLimit(float volts) {
    const float counts = volts / kBusLimitLsbV;
    if (counts >= 32767.0f) return 0x7FFF;
    if (counts <= 0.0f) return 0;
    return uint16_t(counts + 0.5f);
}

// ---- sequences over a register port -------------------------------------------------
// Port: bool read(uint8_t reg, std::span<uint8_t> out) and
// bool write16(uint8_t reg, uint16_t value), register bytes big-endian.

template <class Port>
std::optional<uint16_t> read16(Port& port, uint8_t reg) {
    std::array<uint8_t, 2> b{};
    if (!port.read(reg, b)) return std::nullopt;
    return be16(b);
}

struct Identity {
    uint16_t  manufacturer = 0;
    uint16_t  device       = 0;
    PowerChip chip         = PowerChip::none;
};

// Resets the part and leaves TEMPCOMP set on an INA228; configure()
// overwrites CONFIG. An I2C error at any step leaves chip none.
template <class Port>
Identity identifyPart(Port& port) {
    Identity id;
    const auto m = read16(port, kRegManufacturerId);
    const auto d = read16(port, kRegDeviceId);
    if (!m || !d) return id;
    id.manufacturer = *m;
    id.device = *d;
    if (!port.write16(kRegConfig, kConfigReset) || !port.write16(kRegConfig, kConfigTempComp)) return id;
    if (const auto cfg = read16(port, kRegConfig)) id.chip = identify(*m, *d, (*cfg & kConfigTempComp) != 0);
    return id;
}

// sovl and bovl are register counts (encodeShuntOverLimit(),
// encodeBusOverLimit()). True only when SHUNT_CAL reads back as written.
template <class Port>
bool configure(Port& port, const PowerScale& s, uint16_t sovl, uint16_t bovl) {
    return port.write16(kRegConfig, kConfig)
        && port.write16(kRegAdcConfig, kAdcConfig)
        && port.write16(kRegShuntCal, s.shunt_cal)
        && port.write16(kRegSovl, sovl)
        && port.write16(kRegBovl, bovl)
        && port.write16(kRegDiagAlrt, kDiagAlrtLatch)
        && read16(port, kRegShuntCal) == s.shunt_cal;
}

// VBUS and CURRENT are read at the part's width; POWER is 3 bytes and
// DIETEMP 2 on both. nullopt on any I2C error.
template <class Port>
std::optional<PowerReading> readPower(Port& port, const PowerScale& s) {
    std::array<uint8_t, 3> vbus{}, cur{}, pwr{};
    std::array<uint8_t, 2> temp{};
    const std::span<uint8_t> vb(vbus.data(), s.meas_bytes);
    const std::span<uint8_t> cu(cur.data(), s.meas_bytes);
    if (!port.read(kRegVbus, vb) || !port.read(kRegCurrent, cu) || !port.read(kRegPower, pwr)
        || !port.read(kRegDieTemp, temp))
        return std::nullopt;
    return PowerReading{
        .bus_v     = float(decodeMeasurement(vb)) * s.vbus_lsb_v,
        .current_a = float(decodeMeasurement(cu)) * s.current_lsb_a,
        .power_w   = float(decodePower(pwr)) * s.power_lsb_w,
        .die_c     = decodeDieTempC(temp),
    };
}

}  // namespace ina2xx

// ---- the bus's runtime owner ------------------------------------------------------

// Who may touch the private bus. Unclaimed during boot, when app_main is the
// only caller (powerBegin(), the self-check); claim() then hands the bus to
// one task for the rest of the boot, and mayUse() refuses every other.
// Id is the task's handle; a default-constructed Id means unclaimed.
template <class Id>
class BusOwner {
public:
    // True when `self` now owns the bus: the first claim, or a repeat by the owner.
    bool claim(Id self) {
        Id none{};
        return _owner.compare_exchange_strong(none, self) || none == self;
    }
    bool mayUse(Id self) const {
        const Id o = _owner.load();
        return o == Id{} || o == self;
    }

private:
    std::atomic<Id> _owner{};
};

// ---- the power hand-off -------------------------------------------------------------

// The system voltage and the draw at one instant; nullopt = no reading, never 0.
struct PowerNow {
    std::optional<float> bus_v;    // system (+BUS) voltage
    std::optional<float> draw_w;   // what the motor path draws from it
};

// The bus owner's latest PowerNow, for any task, as ONE 32-bit word: bus
// millivolts in the low half, draw in tenths of a watt in the high half,
// 0xFFFF in a half for no reading. A word the CPU stores and loads whole is
// why a reader never sees one half from an older reading. Readers never wait
// and never touch the bus.
class PowerHandoff {
public:
    static constexpr uint16_t kNone = 0xFFFF;

    // The bus owner only.
    void publish(const PowerNow& p) {
        _word.store(uint32_t(half(p.bus_v, 1000.0f)) | (uint32_t(half(p.draw_w, 10.0f)) << 16),
                    std::memory_order_relaxed);
    }
    // Any task. No reading until the owner's first publish.
    PowerNow latest() const {
        const uint32_t w = _word.load(std::memory_order_relaxed);
        return {unhalf(uint16_t(w), 1000.0f), unhalf(uint16_t(w >> 16), 10.0f)};
    }

    // A value as one half: a negative or NaN reading is no reading; a high
    // one saturates one count under kNone, so it can never read as none.
    static uint16_t half(std::optional<float> v, float perUnit) {
        if (!v || !(*v >= 0.0f)) return kNone;
        const float x = *v * perUnit;
        return x >= float(kNone - 1) ? uint16_t(kNone - 1) : uint16_t(x + 0.5f);
    }
    static std::optional<float> unhalf(uint16_t h, float perUnit) {
        if (h == kNone) return std::nullopt;
        return float(h) / perUnit;
    }

private:
    std::atomic<uint32_t> _word{0xFFFFFFFFu};
};

// The owner's pick. Voltage: the board monitor's +BUS when it answered;
// failing that, U11's MOTOR_V+ while the switch is on (it then follows +BUS
// less the switch's drop); otherwise none. Draw: U11's V x I, which is the
// motor path only (the logic rails are not on R2), with regen read as 0,
// since a returning motor draws nothing from the supply.
// ponytail: one sample a second of a stroking load, not its mean; the
// INA228's ENERGY register or a faster owner poll gives the mean.
inline PowerNow pickPower(std::optional<float> monitorBusV, const std::optional<PowerReading>& u11,
                          bool switchOn) {
    PowerNow p;
    if (monitorBusV) p.bus_v = monitorBusV;
    else if (u11 && switchOn) p.bus_v = u11->bus_v;
    if (u11 && std::isfinite(u11->bus_v * u11->current_a))
        p.draw_w = std::fmax(0.0f, u11->bus_v * u11->current_a);
    return p;
}

}  // namespace valence
