// ValenceSelfCheck -- implementation. See ValenceSelfCheck.h for ownership and
// the never-high rule, SelfCheck.h for the gate.

#include "system/ValenceSelfCheck.h"

#include <cmath>
#include <optional>

#include <driver/gpio.h>
#include <esp_err.h>
#include <nvs.h>

#include "geiger/geiger.h"
#include "hub/ValenceHub.h"
#include "system/BoardPins.h"
#include "system/SelfCheck.h"
#include "system/ValencePower.h"

namespace valence {

namespace {

using selfcheck::Check;
using Verdict = selfcheck::Result;

constexpr const char* kTag = "selfcheck";

constexpr gpio_num_t pin(int n) { return static_cast<gpio_num_t>(n); }

// BSS, ~1 KB, app_main's alone.
selfcheck::Table g_table;
bool g_ran = false;

// ---- the reads ---------------------------------------------------------------

void checkPowerMonitor(std::optional<PowerReading>& reading) {
    using selfcheck::kDieMaxC;
    using selfcheck::kDieMinC;
    using selfcheck::kIdleCurrentMaxA;
    const PowerChip chip = powerChip();
    if (chip == PowerChip::none) {
        g_table.record(Check::power_monitor, Verdict::fail,
                       "no INA228/INA237 answered at 0x40 on G%d/G%d",
                       BOARD_GPIO_INA_SDA, BOARD_GPIO_INA_SCL);
        return;
    }
    reading = powerRead();
    if (!reading) {
        g_table.record(Check::power_monitor, Verdict::fail,
                       "%s identified, then its read failed", powerChipName(chip));
        return;
    }
    const PowerReading& r = *reading;
    const bool dieOk = std::isfinite(r.die_c) && r.die_c >= kDieMinC && r.die_c <= kDieMaxC;
    const bool idleOk = std::isfinite(r.current_a) && std::fabs(r.current_a) <= kIdleCurrentMaxA;
    if (!dieOk || !idleOk) {
        g_table.record(Check::power_monitor, Verdict::fail,
                       "%s implausible: die %.1f C, %.3f A with the switch off",
                       powerChipName(chip), double(r.die_c), double(r.current_a));
        return;
    }
    g_table.record(Check::power_monitor, Verdict::pass, "%s: die %.1f C, %.3f A idle",
                   powerChipName(chip), double(r.die_c), double(r.current_a));
}

void checkMotorRailOff(const std::optional<PowerReading>& reading) {
    if (!reading) {
        g_table.record(Check::motor_rail_off, Verdict::skipped,
                       "needs the power monitor, which did not pass");
        return;
    }
    const float v = reading->bus_v;
    if (std::isfinite(v) && v <= selfcheck::kMotorRailOffMaxV) {
        g_table.record(Check::motor_rail_off, Verdict::pass,
                       "MOTOR_V+ %.2f V with the switch off", double(v));
        return;
    }
    g_table.record(Check::motor_rail_off, Verdict::fail,
                   "MOTOR_V+ %.1f V with the switch off: FET short or JP401 fitted",
                   double(v));
}

void checkSwitchFault() {
    if (gpio_get_level(pin(BOARD_GPIO_MSW_FLT_N)) == 0) {
        g_table.record(Check::switch_fault, Verdict::fail,
                       "MSW_FLT_N low: over-current or over-temperature latched");
        return;
    }
    g_table.record(Check::switch_fault, Verdict::pass, "MSW_FLT_N high");
}

void checkEStop() {
    const bool nc = gpio_get_level(pin(BOARD_GPIO_ESTOP_NC)) != 0;
    const bool no = gpio_get_level(pin(BOARD_GPIO_ESTOP_NO)) != 0;
    switch (selfcheck::decodeEStop(nc, no)) {
        case selfcheck::EStop::normal:
            g_table.record(Check::estop, Verdict::pass, "present and released");
            return;
        case selfcheck::EStop::pressed:
            g_table.record(Check::estop, Verdict::fail,
                           "pressed: the hardware stop holds motor power off");
            return;
        case selfcheck::EStop::unplugged:
            g_table.record(Check::estop, Verdict::fail,
                           "no E-stop found (NC and NO open); bypass not built (val-091.23)");
            return;
        case selfcheck::EStop::wiring_fault:
            g_table.record(Check::estop, Verdict::fail,
                           "NC and NO both closed: wiring fault or a non-E-stop plug");
            return;
    }
}

void checkNvs() {
    nvs_stats_t st{};
    const esp_err_t err = nvs_get_stats(nullptr, &st);
    if (err != ESP_OK) {
        g_table.record(Check::nvs, Verdict::fail, "nvs_get_stats: %s", esp_err_to_name(err));
        return;
    }
    g_table.record(Check::nvs, Verdict::pass, "%u of %u entries used",
                   unsigned(st.used_entries), unsigned(st.total_entries));
}

void checkCatalog() {
    // hub() is read here for the boot report only (ValenceHub.h).
    const valence::Hub* h = hub();
    if (h == nullptr || h->catalogEncodedBytes() == 0) {
        g_table.record(Check::catalog, Verdict::fail, "hub or catalog did not build");
        return;
    }
    g_table.record(Check::catalog, Verdict::pass, "%u B encoded",
                   unsigned(h->catalogEncodedBytes()));
}

// ---- the report --------------------------------------------------------------

void logEntry(Check c) {
    const selfcheck::Entry& e = g_table.entry(c);
    const char* n = selfcheck::name(c);
    const char* r = selfcheck::resultName(e.result);
    switch (e.result) {
        case Verdict::fail:    GLOGE(kTag, "%-14s %s: %s", n, r, e.reason.data()); break;
        case Verdict::pass:    GLOGI(kTag, "%-14s %s: %s", n, r, e.reason.data()); break;
        case Verdict::skipped:
        case Verdict::pending: GLOGW(kTag, "%-14s %s: %s", n, r, e.reason.data()); break;
    }
}

void logSummary() {
    const SelfCheckSummary s = selfCheckSummary();
    if (s.allowed) {
        GLOGW(kTag, "motor power stays OFF: all %u passed, enable sequence not built (val-091.24)",
              unsigned(selfcheck::kCheckCount));
    } else if (s.failed > 0) {
        GLOGE(kTag, "motor power held OFF: %u failed, %u skipped; first: %s",
              unsigned(s.failed), unsigned(s.skipped), s.first);
    } else {
        GLOGW(kTag, "motor power held OFF: %u skipped; first: %s",
              unsigned(s.skipped), s.first);
    }
}

}  // namespace

// ---- public surface ------------------------------------------------------------

void selfCheckHoldMotorOff() {
    // Level first, then direction: the pad never drives high, not even for
    // the instant between enabling the output and writing it.
    constexpr int kHeldLow[] = {BOARD_GPIO_MOTOR_EN, BOARD_GPIO_PRECHARGE_EN,
                                BOARD_GPIO_ESTOP_BYP};
    for (int n : kHeldLow) gpio_set_level(pin(n), 0);
    gpio_config_t out{};
    for (int n : kHeldLow) out.pin_bit_mask |= 1ULL << n;
    out.mode = GPIO_MODE_OUTPUT;
    gpio_config(&out);
    for (int n : kHeldLow) gpio_set_level(pin(n), 0);

    // The board fits external pull-ups on all three; the internal ones only
    // give a bare stamp a defined reading.
    gpio_config_t in{};
    in.pin_bit_mask = (1ULL << BOARD_GPIO_ESTOP_NC) | (1ULL << BOARD_GPIO_ESTOP_NO)
                    | (1ULL << BOARD_GPIO_MSW_FLT_N);
    in.mode = GPIO_MODE_INPUT;
    in.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&in);
}

bool selfCheckRun(const SelfCheckFacts& facts) {
    std::optional<PowerReading> reading;

    g_table.record(Check::board_monitor, Verdict::skipped,
                   "monitor firmware and link not landed (val-091.19, val-091.20)");
    g_table.record(Check::rails, Verdict::skipped,
                   "rails are read by the board monitor (val-091.19)");
    checkPowerMonitor(reading);
    g_table.record(Check::bus_window, Verdict::skipped,
                   "+BUS and VIN_RAW are read only by the board monitor (val-091.19)");
    checkMotorRailOff(reading);
    checkSwitchFault();
    g_table.record(Check::regen_clamp, Verdict::skipped,
                   "the clamp test pulse is a board-monitor command (val-091.19)");
    checkEStop();
    if (facts.lpEdges > 0) {
        g_table.record(Check::quadrature, Verdict::pass,
                       "LP core rendered %lu edges; index (ZO) not wired",
                       static_cast<unsigned long>(facts.lpEdges));
    } else {
        g_table.record(Check::quadrature, Verdict::fail, "LP core rendered no edges since boot");
    }
    g_table.record(Check::drive_link, Verdict::skipped,
                   "no Modbus driver; drive unpowered until motor power (val-091.29)");
    if (facts.hostLink) {
        g_table.record(Check::host_link, Verdict::pass, "esp_hosted up, C6 reported its version");
    } else {
        g_table.record(Check::host_link, Verdict::fail, "esp_hosted did not reach the C6");
    }
    checkNvs();
    g_table.record(Check::trust_ledger, Verdict::skipped,
                   "the trust ledger is not persisted yet (val-fvn)");
    checkCatalog();

    g_ran = true;
    for (size_t i = 0; i < selfcheck::kCheckCount; ++i) logEntry(Check(i));
    logSummary();
    return g_table.motorPowerAllowed();
}

SelfCheckSummary selfCheckSummary() {
    SelfCheckSummary s;
    s.ran = g_ran;
    if (!g_ran) return s;
    s.allowed = g_table.motorPowerAllowed();
    s.failed = uint8_t(g_table.count(Verdict::fail));
    s.skipped = uint8_t(g_table.count(Verdict::skipped));
    const std::optional<Check> first = g_table.firstBlocking();
    s.first = first ? selfcheck::name(*first) : "";
    return s;
}

void selfCheckRemind() {
    if (g_ran) logSummary();
}

}  // namespace valence
