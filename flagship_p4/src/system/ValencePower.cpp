// ValencePower -- implementation. See ValencePower.h for ownership and the
// ALERT re-arm contract, PowerMonitor.h for what the register bytes mean.

#include "system/ValencePower.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <driver/i2c_master.h>

#include "geiger/geiger.h"

namespace valence {

namespace {

constexpr const char* kTag = "power";

// Pin map and address: Hardware flagship/SPEC.md (2026-09-23 pin-map and
// sensing rows). The board fits 4.7k pull-ups; the internal ones only keep a
// bare stamp's lines from floating.
constexpr gpio_num_t kPinSda     = GPIO_NUM_34;
constexpr gpio_num_t kPinScl     = GPIO_NUM_36;
constexpr uint16_t   kAddress    = 0x40;
constexpr uint32_t   kSclHz      = 400000;
constexpr int        kTimeoutMs  = 10;

// R2, the Kelvin shunt. THE CALIBRATION KNOB: a board measured against a
// reference meter corrects its gain here.
constexpr float kShuntOhms = 0.001f;

// SOVL sits at the motor switch's own over-current level (TPS48111, 15.2 A):
// the INA adds a one-conversion detector, not a second, lower limit.
// TODO(val-091.22): operator ruling on the SOVL trip level.
constexpr float kShuntOverAmps = 15.0f;
// BOVL: below the ~51 V bus-powered trip, above the 44.5 V regen clamp.
constexpr float kBusOverVolts = 50.0f;

i2c_master_bus_handle_t g_bus = nullptr;
i2c_master_dev_handle_t g_dev = nullptr;
PowerChip  g_chip  = PowerChip::none;
ina2xx::PowerScale g_scale = {};

bool readReg(uint8_t reg, std::span<uint8_t> out) {
    return i2c_master_transmit_receive(g_dev, &reg, 1, out.data(), out.size(), kTimeoutMs) == ESP_OK;
}

std::optional<uint16_t> readReg16(uint8_t reg) {
    std::array<uint8_t, 2> b{};
    if (!readReg(reg, b)) return std::nullopt;
    return ina2xx::be16(b);
}

bool writeReg16(uint8_t reg, uint16_t value) {
    const std::array<uint8_t, 3> b{reg, uint8_t(value >> 8), uint8_t(value)};
    return i2c_master_transmit(g_dev, b.data(), b.size(), kTimeoutMs) == ESP_OK;
}

// Decides the part. Leaves the chip freshly reset with TEMPCOMP possibly set;
// configure() overwrites CONFIG.
PowerChip identifyPart(uint16_t& manufacturer, uint16_t& device) {
    const auto m = readReg16(ina2xx::kRegManufacturerId);
    const auto d = readReg16(ina2xx::kRegDeviceId);
    if (!m || !d) return PowerChip::none;
    manufacturer = *m;
    device = *d;
    if (!writeReg16(ina2xx::kRegConfig, ina2xx::kConfigReset)) return PowerChip::none;
    if (!writeReg16(ina2xx::kRegConfig, ina2xx::kConfigTempComp)) return PowerChip::none;
    const auto cfg = readReg16(ina2xx::kRegConfig);
    if (!cfg) return PowerChip::none;
    return ina2xx::identify(*m, *d, (*cfg & ina2xx::kConfigTempComp) != 0);
}

bool configure() {
    using namespace ina2xx;
    return writeReg16(kRegConfig, kConfig)
        && writeReg16(kRegAdcConfig, kAdcConfig)
        && writeReg16(kRegShuntCal, g_scale.shunt_cal)
        && writeReg16(kRegSovl, encodeShuntOverLimit(kShuntOverAmps, kShuntOhms))
        && writeReg16(kRegBovl, encodeBusOverLimit(kBusOverVolts))
        && writeReg16(kRegDiagAlrt, kDiagAlrtLatch)
        && readReg16(kRegShuntCal) == g_scale.shunt_cal;
}

}  // namespace

bool powerBegin() {
    i2c_master_bus_config_t bus{};
    bus.i2c_port = -1;
    bus.sda_io_num = kPinSda;
    bus.scl_io_num = kPinScl;
    bus.clk_source = I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt = 7;
    bus.flags.enable_internal_pullup = 1;
    if (i2c_new_master_bus(&bus, &g_bus) != ESP_OK) {
        GLOGE(kTag, "I2C bus init failed (SDA G%d, SCL G%d)", int(kPinSda), int(kPinScl));
        return false;
    }
    if (i2c_master_probe(g_bus, kAddress, kTimeoutMs) != ESP_OK) {
        GLOGW(kTag, "no power monitor at 0x%02x: current sensing absent", unsigned(kAddress));
        return false;
    }
    i2c_device_config_t dev{};
    dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev.device_address = kAddress;
    dev.scl_speed_hz = kSclHz;
    if (i2c_master_bus_add_device(g_bus, &dev, &g_dev) != ESP_OK) {
        GLOGE(kTag, "I2C device add failed");
        return false;
    }

    uint16_t manufacturer = 0, device = 0;
    const PowerChip chip = identifyPart(manufacturer, device);
    if (chip == PowerChip::none) {
        GLOGE(kTag, "unrecognized part at 0x%02x (mfr 0x%04x, id 0x%04x): current sensing off",
              unsigned(kAddress), unsigned(manufacturer), unsigned(device));
        return false;
    }
    g_scale = ina2xx::powerScaleFor(chip, kShuntOhms);
    if (!configure()) {
        GLOGE(kTag, "%s configure failed: current sensing off", powerChipName(chip));
        return false;
    }
    g_chip = chip;
    powerTakeAlerts();   // start with the latch clear

    GLOGI(kTag, "%s at 0x%02x (id 0x%04x), %.3f mA/LSB, SOVL %.1f A, BOVL %.1f V",
          powerChipName(chip), unsigned(kAddress), unsigned(device),
          double(g_scale.current_lsb_a * 1000.0f), double(kShuntOverAmps), double(kBusOverVolts));
    if (const auto r = powerRead())
        GLOGI(kTag, "bus %.2f V, current %.3f A, die %.1f C", double(r->bus_v),
              double(r->current_a), double(r->die_c));
    return true;
}

PowerChip powerChip() { return g_chip; }

std::optional<PowerReading> powerRead() {
    if (g_chip == PowerChip::none) return std::nullopt;
    std::array<uint8_t, 3> vbus{}, cur{}, pwr{};
    std::array<uint8_t, 2> temp{};
    const std::span<uint8_t> vb(vbus.data(), g_scale.meas_bytes);
    const std::span<uint8_t> cu(cur.data(), g_scale.meas_bytes);
    if (!readReg(ina2xx::kRegVbus, vb) || !readReg(ina2xx::kRegCurrent, cu)
        || !readReg(ina2xx::kRegPower, pwr) || !readReg(ina2xx::kRegDieTemp, temp))
        return std::nullopt;
    return PowerReading{
        .bus_v     = float(ina2xx::decodeMeasurement(vb)) * g_scale.vbus_lsb_v,
        .current_a = float(ina2xx::decodeMeasurement(cu)) * g_scale.current_lsb_a,
        .power_w   = float(ina2xx::decodePower(pwr)) * g_scale.power_lsb_w,
        .die_c     = ina2xx::decodeDieTempC(temp),
    };
}

std::optional<uint16_t> powerTakeAlerts() {
    if (g_chip == PowerChip::none) return std::nullopt;
    return readReg16(ina2xx::kRegDiagAlrt);
}

}  // namespace valence
