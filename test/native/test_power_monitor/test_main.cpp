// test_power_monitor -- native doctest suite for the INA228 / INA237 register layer
// Constraints:
// - Hardware-free: register bytes in, SI units out. Expected values come from
//   the datasheets' own worked examples and LSB tables (INA228 SLYS021A,
//   INA237 SBOSA20A), never from the code under test.
// See: flagship_p4/src/system/PowerMonitor.h, bd val-091.22

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cstdint>

#include "../../../flagship_p4/src/system/PowerMonitor.h"

using valence::PowerChip;
namespace ina = valence::ina2xx;

namespace {
constexpr float kShunt = 0.001f;   // R2 on the Flagship
}

TEST_CASE("identify: DEVICE_ID and the TEMPCOMP probe must agree") {
    CHECK(ina::identify(0x5449, 0x2281, true) == PowerChip::ina228);
    CHECK(ina::identify(0x5449, 0x2370, false) == PowerChip::ina237);
    // INA237 has no documented DEVICE_ID: the probe alone decides.
    CHECK(ina::identify(0x5449, 0x0000, false) == PowerChip::ina237);
    CHECK(ina::identify(0x5449, 0xFFFF, false) == PowerChip::ina237);
    // Contradictions and strangers are refused.
    CHECK(ina::identify(0x5449, 0x2281, false) == PowerChip::none);
    CHECK(ina::identify(0x5449, 0x2370, true) == PowerChip::none);
    CHECK(ina::identify(0x5449, 0x0000, true) == PowerChip::none);
    CHECK(ina::identify(0x0000, 0x2281, true) == PowerChip::none);
    CHECK(ina::identify(0xFFFF, 0xFFFF, false) == PowerChip::none);
}

TEST_CASE("scale: ADCRANGE 1 on a 1 mOhm shunt") {
    const auto s228 = ina::powerScaleFor(PowerChip::ina228, kShunt);
    CHECK(s228.meas_bytes == 3);
    CHECK(s228.shunt_cal == 4096);
    CHECK(s228.current_lsb_a == doctest::Approx(78.125e-6).epsilon(1e-6));   // 78.125 nV / 1 mOhm
    CHECK(s228.power_lsb_w == doctest::Approx(3.2 * 78.125e-6).epsilon(1e-6));

    const auto s237 = ina::powerScaleFor(PowerChip::ina237, kShunt);
    CHECK(s237.meas_bytes == 2);
    CHECK(s237.shunt_cal == 4096);
    CHECK(s237.current_lsb_a == doctest::Approx(1.25e-3).epsilon(1e-6));     // 1.25 uV / 1 mOhm
    CHECK(s237.power_lsb_w == doctest::Approx(0.2 * 1.25e-3).epsilon(1e-6));
}

TEST_CASE("scale: SHUNT_CAL follows the datasheet equation off the nominal shunt") {
    // A 2 mOhm shunt: full scale halves, the LSB halves, SHUNT_CAL is unchanged
    // because CURRENT_LSB is pinned to the range.
    CHECK(ina::powerScaleFor(PowerChip::ina228, 0.002f).shunt_cal == 4096);
    CHECK(ina::powerScaleFor(PowerChip::ina228, 0.002f).current_lsb_a
          == doctest::Approx(39.0625e-6).epsilon(1e-6));
}

TEST_CASE("decode: INA228 20-bit measurement in bits 23:4") {
    // +1 count, -1 count, full positive, full negative.
    CHECK(ina::decodeMeasurement(std::array<uint8_t, 3>{0x00, 0x00, 0x10}) == 1);
    CHECK(ina::decodeMeasurement(std::array<uint8_t, 3>{0xFF, 0xFF, 0xF0}) == -1);
    CHECK(ina::decodeMeasurement(std::array<uint8_t, 3>{0x7F, 0xFF, 0xF0}) == 524287);
    CHECK(ina::decodeMeasurement(std::array<uint8_t, 3>{0x80, 0x00, 0x00}) == -524288);
    // Reserved low nibble never leaks into the value.
    CHECK(ina::decodeMeasurement(std::array<uint8_t, 3>{0x00, 0x00, 0x1F}) == 1);
    // Datasheet Table 8-4: VBUS 245760d x 195.3125 uV = 48 V.
    const uint32_t raw = 245760u << 4;
    const std::array<uint8_t, 3> b{uint8_t(raw >> 16), uint8_t(raw >> 8), uint8_t(raw)};
    const auto s = ina::powerScaleFor(PowerChip::ina228, kShunt);
    CHECK(float(ina::decodeMeasurement(b)) * s.vbus_lsb_v == doctest::Approx(48.0));
}

TEST_CASE("decode: INA237 16-bit measurement") {
    CHECK(ina::decodeMeasurement(std::array<uint8_t, 2>{0x00, 0x01}) == 1);
    CHECK(ina::decodeMeasurement(std::array<uint8_t, 2>{0xFF, 0xFF}) == -1);
    CHECK(ina::decodeMeasurement(std::array<uint8_t, 2>{0x80, 0x00}) == -32768);
    // Datasheet Table 8-4: VBUS 15360d x 3.125 mV = 48 V.
    const auto s = ina::powerScaleFor(PowerChip::ina237, kShunt);
    CHECK(float(ina::decodeMeasurement(std::array<uint8_t, 2>{0x3C, 0x00})) * s.vbus_lsb_v
          == doctest::Approx(48.0));
    // Regen: -8 A is -6400 counts at 1.25 mA.
    const int16_t neg = -6400;
    const std::array<uint8_t, 2> nb{uint8_t(uint16_t(neg) >> 8), uint8_t(uint16_t(neg))};
    CHECK(float(ina::decodeMeasurement(nb)) * s.current_lsb_a == doctest::Approx(-8.0));
}

TEST_CASE("decode: power and die temperature share one path") {
    CHECK(ina::decodePower(std::array<uint8_t, 3>{0x48, 0x00, 0x0C}) == 4718604u);
    // INA228: 3200d x 7.8125 mC = 25 C. INA237: 200d in bits 15:4 x 125 mC = 25 C.
    CHECK(ina::decodeDieTempC(std::array<uint8_t, 2>{0x0C, 0x80}) == doctest::Approx(25.0));
    CHECK(ina::decodeDieTempC(std::array<uint8_t, 2>{uint8_t((200 << 4) >> 8), uint8_t(200 << 4)})
          == doctest::Approx(25.0));
    CHECK(ina::decodeDieTempC(std::array<uint8_t, 2>{0xFF, 0x80}) == doctest::Approx(-1.0));
}

TEST_CASE("limits: SOVL and BOVL encode identically on both parts") {
    // 15 A x 1 mOhm = 15 mV / 1.25 uV = 12000 counts.
    CHECK(ina::encodeShuntOverLimit(15.0f, kShunt) == 12000);
    CHECK(ina::encodeShuntOverLimit(100.0f, kShunt) == 0x7FFF);   // past full scale: clamped
    CHECK(ina::encodeShuntOverLimit(-1.0f, kShunt) == 0);
    // 50 V / 3.125 mV = 16000 counts; datasheet example 52 V = 16640 (0x4100).
    CHECK(ina::encodeBusOverLimit(50.0f) == 16000);
    CHECK(ina::encodeBusOverLimit(52.0f) == 0x4100);
    CHECK(ina::encodeBusOverLimit(200.0f) == 0x7FFF);
}

TEST_CASE("register values: ADCRANGE set, alert latched active-low") {
    CHECK((ina::kConfig & ina::kConfigAdcRange) != 0);
    CHECK((ina::kConfig & ina::kConfigTempComp) == 0);
    CHECK(ina::kDiagAlrtLatch == 0x8000);
    CHECK((ina::kAdcConfig >> 12) == 0xF);
}
