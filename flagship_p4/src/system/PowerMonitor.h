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
// See: Hardware flagship/SPEC.md (2026-09-24 U11 row), bd val-091.22

#include <cstddef>
#include <cstdint>
#include <span>

namespace valence {

enum class PowerChip : uint8_t { none, ina228, ina237 };

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

}  // namespace ina2xx
}  // namespace valence
