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
#include <map>
#include <span>

#include "../../../flagship_p4/src/system/PowerMonitor.h"

using valence::PowerChip;
namespace ina = valence::ina2xx;

namespace {
constexpr float kShunt = 0.001f;   // R2 on the Flagship

// U11 as a register file: each part's register widths and reserved bits.
// A read at the wrong width fails, where silicon would hand back the wrong
// bytes. Reset clears every register to 0; power-on values are not modeled.
struct FakeIna {
    PowerChip part;
    uint16_t device_id;                // what 0x3F answers
    uint16_t manufacturer = ina::kManufacturerTi;
    std::map<uint8_t, uint32_t> regs{};
    bool bus_down = false;
    bool drops_shunt_cal = false;      // a part that does not hold SHUNT_CAL
    int wrong_width = 0;

    size_t width(uint8_t reg) const {
        if (reg == ina::kRegPower) return 3;
        if (reg == 0x04 || reg == ina::kRegVbus || reg == ina::kRegCurrent) return part == PowerChip::ina228 ? 3 : 2;
        return 2;
    }
    bool read(uint8_t reg, std::span<uint8_t> out) {
        if (bus_down) return false;
        if (out.size() != width(reg)) {
            ++wrong_width;
            return false;
        }
        const uint32_t v = reg == ina::kRegManufacturerId ? manufacturer
                         : reg == ina::kRegDeviceId       ? device_id
                                                          : regs[reg];
        for (size_t i = 0; i < out.size(); ++i) out[i] = uint8_t(v >> (8 * (out.size() - 1 - i)));
        return true;
    }
    bool write16(uint8_t reg, uint16_t v) {
        if (bus_down) return false;
        if (reg == ina::kRegConfig && (v & ina::kConfigReset) != 0) {
            regs.clear();
            return true;
        }
        if (reg == ina::kRegConfig && part == PowerChip::ina237) v &= uint16_t(~ina::kConfigTempComp);   // reserved
        if (reg == ina::kRegShuntCal) v = drops_shunt_cal ? 0 : uint16_t(v & 0x7FFF);   // bit 15 reserved
        regs[reg] = v;
        return true;
    }
};

// One boot as ValencePower.cpp runs it: identify, scale, configure.
PowerChip boot(FakeIna& f) {
    const ina::Identity id = ina::identifyPart(f);
    if (id.chip == PowerChip::none) return id.chip;
    const auto s = ina::powerScaleFor(id.chip, kShunt);
    if (!ina::configure(f, s, ina::encodeShuntOverLimit(15.0f, kShunt), ina::encodeBusOverLimit(50.0f)))
        return PowerChip::none;
    return id.chip;
}
}  // namespace

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

TEST_CASE("fake: each part boots, identified by DEVICE_ID and the probe, configured alike") {
    for (FakeIna f : {FakeIna{PowerChip::ina228, 0x2281}, FakeIna{PowerChip::ina237, 0x0000},
                      FakeIna{PowerChip::ina237, 0x2370}}) {
        CAPTURE(f.device_id);
        CHECK(boot(f) == f.part);
        CHECK(f.regs[ina::kRegConfig] == ina::kConfigAdcRange);   // the probe's TEMPCOMP cleared
        CHECK(f.regs[ina::kRegAdcConfig] == ina::kAdcConfig);
        CHECK(f.regs[ina::kRegShuntCal] == 4096);
        CHECK(f.regs[ina::kRegSovl] == 12000);
        CHECK(f.regs[ina::kRegBovl] == 16000);
        CHECK(f.regs[ina::kRegDiagAlrt] == ina::kDiagAlrtLatch);
        CHECK(f.wrong_width == 0);
    }
}

TEST_CASE("fake: a contradiction, a stranger, a dead bus or a lost SHUNT_CAL boots nothing") {
    FakeIna lying{PowerChip::ina237, 0x2281};   // an INA228 id, a dead TEMPCOMP
    CHECK(boot(lying) == PowerChip::none);
    FakeIna stranger{PowerChip::ina228, 0x2281, 0x1234};
    CHECK(boot(stranger) == PowerChip::none);
    FakeIna down{PowerChip::ina228, 0x2281};
    down.bus_down = true;
    CHECK(boot(down) == PowerChip::none);
    FakeIna amnesiac{PowerChip::ina237, 0x0000};
    amnesiac.drops_shunt_cal = true;
    CHECK(ina::identifyPart(amnesiac).chip == PowerChip::ina237);
    CHECK(boot(amnesiac) == PowerChip::none);
}

TEST_CASE("fake: readPower reads each part at its own width into the same SI values") {
    // 36 V, 5 A, 180 W, 30 C. INA228: VBUS 184320 and CURRENT 64000 in bits
    // 23:4, POWER 720000 at 3.2 x 78.125 uW, DIETEMP 3840 at 7.8125 mC.
    FakeIna a{PowerChip::ina228, 0x2281};
    REQUIRE(boot(a) == PowerChip::ina228);
    a.regs[ina::kRegVbus] = 184320u << 4;
    a.regs[ina::kRegCurrent] = 64000u << 4;
    a.regs[ina::kRegPower] = 720000;
    a.regs[ina::kRegDieTemp] = 3840;
    // INA237: VBUS 11520 at 3.125 mV, CURRENT 4000 at 1.25 mA, POWER 720000
    // at 0.2 x 1.25 mW, DIETEMP 240 in bits 15:4 at 125 mC.
    FakeIna b{PowerChip::ina237, 0x0000};
    REQUIRE(boot(b) == PowerChip::ina237);
    b.regs[ina::kRegVbus] = 11520;
    b.regs[ina::kRegCurrent] = 4000;
    b.regs[ina::kRegPower] = 720000;
    b.regs[ina::kRegDieTemp] = 240u << 4;
    for (FakeIna* f : {&a, &b}) {
        const auto r = ina::readPower(*f, ina::powerScaleFor(f->part, kShunt));
        REQUIRE(r.has_value());
        CHECK(r->bus_v == doctest::Approx(36.0));
        CHECK(r->current_a == doctest::Approx(5.0));
        CHECK(r->power_w == doctest::Approx(180.0));
        CHECK(r->die_c == doctest::Approx(30.0));
        CHECK(f->wrong_width == 0);
    }
    // Regen reads negative: -4 A is -3200 counts on the INA237.
    b.regs[ina::kRegCurrent] = uint16_t(int16_t(-3200));
    CHECK(ina::readPower(b, ina::powerScaleFor(PowerChip::ina237, kShunt))->current_a == doctest::Approx(-4.0));
    // The other part's widths never read.
    CHECK_FALSE(ina::readPower(b, ina::powerScaleFor(PowerChip::ina228, kShunt)).has_value());
    CHECK(b.wrong_width == 1);
}

TEST_CASE("fake: the ALERT flags read back through DIAG_ALRT on both parts") {
    for (FakeIna f : {FakeIna{PowerChip::ina228, 0x2281}, FakeIna{PowerChip::ina237, 0x0000}}) {
        REQUIRE(boot(f) == f.part);
        f.regs[ina::kRegDiagAlrt] |= ina::kFlagShuntOver | ina::kFlagMemOk;
        const auto flags = ina::read16(f, ina::kRegDiagAlrt);
        REQUIRE(flags.has_value());
        CHECK((*flags & ina::kFlagShuntOver) != 0);
        CHECK((*flags & ina::kFlagBusOver) == 0);
    }
}
