// ValenceMotorSwitch -- the board's motor-switch host: the switch task, the
// ADC reads, the two enable pins and the cross-task doors
// Constraints:
// - ONE SPINLOCK, g_mux, guards the machine, its latest readings and the two
//   pins. It exists because the cut lowers the pins on the CALLING task (an
//   ESTOP that waits for a task switch is not one) while the switch task
//   raises them: only a lock makes "no cut since the step, then raise" one
//   step. Inside it: the machine's methods and two gpio_set_level calls. No
//   log, no ADC, no I2C, no allocation, no function-local static (T4).
// - The switch task alone reads G19 and G23 (adc_oneshot_read only try-locks
//   its unit, so a second reader fails rather than waits) and alone calls
//   powerRead() and powerTakeAlerts() here.
// - MOTOR_EN and PRECHARGE_EN are outputs driven low from
//   selfCheckHoldMotorOff(); this file is the only one that drives them high.
// - Task "MotorSw": core 0, priority 5, kMotorSwitchTaskStackBytes of
//   internal RAM, a kPollMs poll. BSS: the machine, its readings, the ADC
//   handles and a few words.
// - No calibrated ADC means NaN readings, and an unread EN node or IMON never
//   passes (MotorSwitch.h): a board whose ADC will not calibrate never enables.
// See: ValenceMotorSwitch.h, MotorSwitch.h, BoardPins.h, ValencePower.h,
// bd val-091.24

#include "system/ValenceMotorSwitch.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <driver/gpio.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "geiger/geiger.h"
#include "motion/ValenceMotion.h"
#include "system/BoardPins.h"
#include "system/ValencePower.h"

namespace valence {

namespace {

using motorswitch::Fault;
using motorswitch::Readings;
using motorswitch::Refusal;
using motorswitch::State;

constexpr const char* kTag = "msw";

// The fault watch's resolution and the window's granularity. The controller
// cuts the FETs itself; this bounds how long the LP emitter can render into a
// drive that already lost power.
constexpr uint32_t kPollMs = 5;
// IMON is averaged: one oneshot sample carries a few mV of noise against a
// 22.5 mV ceiling (kInrushCeilingA x kImonVoltsPerAmp).
constexpr int kImonSamples = 4;
// The INA's ALERT releases the EN node when DIAG_ALRT is read; C404 10 nF
// into the ~28k EN divider settles in ~1.5 ms (5 tau).
constexpr uint32_t kRearmSettleMs = 2;

constexpr gpio_num_t pin(int n) { return static_cast<gpio_num_t>(n); }

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// ---- state under g_mux ---------------------------------------------------------

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
motorswitch::Switch g_sw;
Readings g_last;           // the task's latest, for the hub-side precheck
bool g_allowed = false;    // the self-check's verdict
uint32_t g_cuts = 0;       // every cut, so an enable consumed before one is dropped

// ---- state outside it ----------------------------------------------------------

std::atomic<bool> g_enableReq{false};
// state | last_fault << 8 | faults << 16, written under g_mux, read anywhere.
std::atomic<uint32_t> g_status{0};
TaskHandle_t g_task = nullptr;
// What the arbiter last heard (motionSetMotorPowered), the switch task's alone.
bool g_pushedOn = false;

// The switch task's alone after motorSwitchBegin().
adc_oneshot_unit_handle_t g_adc = nullptr;
adc_cali_handle_t g_caliImon = nullptr;
adc_cali_handle_t g_caliEn = nullptr;
adc_channel_t g_chImon{};
adc_channel_t g_chEn{};

// Under g_mux only. MOTOR_EN is written first both ways: closing, the main
// FETs take the bus before the pre-charge path lets go; opening, the main
// path goes first.
void driveLocked() {
    const motorswitch::Outputs o = g_sw.outputs();
    gpio_set_level(pin(BOARD_GPIO_MOTOR_EN), o.motor_en ? 1u : 0u);
    gpio_set_level(pin(BOARD_GPIO_PRECHARGE_EN), o.precharge_en ? 1u : 0u);
    g_status.store(uint32_t(g_sw.state()) | (uint32_t(g_sw.lastFault()) << 8) |
                       ((g_sw.faults() & 0xFFFFu) << 16),
                   std::memory_order_relaxed);
}

// ---- the reads -----------------------------------------------------------------

bool caliFor(adc_unit_t unit, adc_channel_t ch, adc_cali_handle_t* out) {
    adc_cali_curve_fitting_config_t cfg{};
    cfg.unit_id = unit;
    cfg.chan = ch;
    cfg.atten = ADC_ATTEN_DB_12;
    cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    return adc_cali_create_scheme_curve_fitting(&cfg, out) == ESP_OK;
}

bool adcBegin() {
    adc_unit_t unitImon{};
    adc_unit_t unitEn{};
    if (adc_oneshot_io_to_channel(BOARD_GPIO_MSW_IMON, &unitImon, &g_chImon) != ESP_OK ||
        adc_oneshot_io_to_channel(BOARD_GPIO_EN_NODE, &unitEn, &g_chEn) != ESP_OK || unitImon != unitEn) {
        GLOGE(kTag, "G%d/G%d are not one ADC unit: motor power can never enable",
              BOARD_GPIO_MSW_IMON, BOARD_GPIO_EN_NODE);
        return false;
    }
    adc_oneshot_unit_init_cfg_t unit{};
    unit.unit_id = unitImon;
    if (adc_oneshot_new_unit(&unit, &g_adc) != ESP_OK) {
        GLOGE(kTag, "ADC unit %d init failed: motor power can never enable", int(unitImon) + 1);
        return false;
    }
    adc_oneshot_chan_cfg_t chan{};
    chan.atten = ADC_ATTEN_DB_12;
    chan.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_oneshot_config_channel(g_adc, g_chImon, &chan) != ESP_OK ||
        adc_oneshot_config_channel(g_adc, g_chEn, &chan) != ESP_OK ||
        !caliFor(unitImon, g_chImon, &g_caliImon) || !caliFor(unitImon, g_chEn, &g_caliEn)) {
        GLOGE(kTag, "ADC channel or calibration setup failed: motor power can never enable");
        return false;
    }
    return true;
}

float readVolts(adc_channel_t ch, adc_cali_handle_t cali, int samples) {
    if (g_adc == nullptr || cali == nullptr) return kNaN;
    int sum = 0;
    for (int i = 0; i < samples; ++i) {
        int mv = 0;
        if (adc_oneshot_get_calibrated_result(g_adc, cali, ch, &mv) != ESP_OK) return kNaN;
        sum += mv;
    }
    return float(sum) / float(samples) / 1000.0f;
}

Readings readPins() {
    Readings r;
    r.fault_line = gpio_get_level(pin(BOARD_GPIO_MSW_FLT_N)) == 0;
    r.en_node_v = readVolts(g_chEn, g_caliEn, 1);
    r.imon_a = readVolts(g_chImon, g_caliImon, kImonSamples) / motorswitch::kImonVoltsPerAmp;
    return r;
}

// ---- the task ------------------------------------------------------------------

void logTransition(State is, Fault f, const Readings& r) {
    if (is == State::on) {
        GLOGI(kTag, "motor power ON after %lu ms pre-charge: IMON %.3f A, MOTOR_V+ %.1f V",
              static_cast<unsigned long>(motorswitch::kPrechargeUs / 1000), double(r.imon_a),
              double(r.motor_v));
    } else if (is == State::faulted) {
        GLOGE(kTag, "motor switch FAULTED, power latched off until release: %s "
                    "(FLT %s, EN node %.2f V, IMON %.3f A, MOTOR_V+ %.1f V)",
              motorswitch::faultName(f), r.fault_line ? "low" : "high", double(r.en_node_v),
              double(r.imon_a), double(r.motor_v));
    }
}

// An enable request, on this task: the INA ALERT re-arm, then the EN node
// read fresh, then the machine's own judgment.
void serviceEnable() {
    portENTER_CRITICAL(&g_mux);
    const uint32_t cutsAtRequest = g_cuts;
    portEXIT_CRITICAL(&g_mux);

    // The INA's latched ALERT holds the EN node down until DIAG_ALRT is read
    // (ValencePower.h): that read plus this sequence's MOTOR_EN toggle is the
    // re-arm, never one alone.
    if (const std::optional<uint16_t> flags = powerTakeAlerts()) {
        if (*flags & (ina2xx::kFlagShuntOver | ina2xx::kFlagBusOver)) {
            GLOGW(kTag, "re-arm: power monitor had latched%s%s (DIAG_ALRT 0x%04x)",
                  (*flags & ina2xx::kFlagShuntOver) ? " shunt over-current" : "",
                  (*flags & ina2xx::kFlagBusOver) ? " bus over-voltage" : "", unsigned(*flags));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(kRearmSettleMs));
    const Readings r = readPins();
    const uint64_t now = uint64_t(esp_timer_get_time());

    Refusal why = Refusal::none;
    bool dropped = false;
    bool started = false;
    portENTER_CRITICAL(&g_mux);
    g_last = r;
    if (g_cuts != cutsAtRequest) {
        dropped = true;   // a cut landed after the request: the later command wins
    } else {
        const State was = g_sw.state();
        why = g_sw.requestEnable(g_allowed, r, now);
        started = was != State::precharging && g_sw.state() == State::precharging;
        driveLocked();
    }
    portEXIT_CRITICAL(&g_mux);

    if (dropped) {
        GLOGW(kTag, "enable dropped: a power cut followed the request");
    } else if (why != Refusal::none) {
        GLOGE(kTag, "enable refused: %s (FLT %s, EN node %.2f V)", motorswitch::refusalName(why),
              r.fault_line ? "low" : "high", double(r.en_node_v));
    } else if (started) {
        GLOGI(kTag, "pre-charge: PRECHARGE_EN high for %lu ms (EN node %.2f V)",
              static_cast<unsigned long>(motorswitch::kPrechargeUs / 1000), double(r.en_node_v));
    }
}

void taskMain(void*) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kPollMs));
        if (g_enableReq.exchange(false)) serviceEnable();

        Readings r = readPins();
        const uint64_t now = uint64_t(esp_timer_get_time());
        portENTER_CRITICAL(&g_mux);
        const bool due = g_sw.windowDue(now);
        portEXIT_CRITICAL(&g_mux);
        // I2C, so only at the one moment the machine judges it.
        if (due) {
            if (const std::optional<PowerReading> p = powerRead()) r.motor_v = p->bus_v;
        }

        portENTER_CRITICAL(&g_mux);
        g_last = r;
        const bool changed = g_sw.step(r, now);
        driveLocked();
        const State is = g_sw.state();
        const Fault f = g_sw.lastFault();
        portEXIT_CRITICAL(&g_mux);

        if (changed) logTransition(is, f, r);
        // Every entry into and exit from `on`, a cut from another task
        // included, reaches the arbiter from here and only from here.
        const bool on = is == State::on;
        if (on != g_pushedOn) {
            g_pushedOn = on;
            motionSetMotorPowered(on);
        }
    }
}

}  // namespace

// ---- the door ------------------------------------------------------------------

bool motorSwitchBegin() {
    const bool adcOk = adcBegin();
    const Readings r = readPins();
    portENTER_CRITICAL(&g_mux);
    g_last = r;
    driveLocked();
    portEXIT_CRITICAL(&g_mux);
    if (xTaskCreatePinnedToCore(&taskMain, "MotorSw", kMotorSwitchTaskStackBytes, nullptr, 5, &g_task,
                                0) != pdPASS) {
        g_task = nullptr;
        GLOGE(kTag, "switch task did not start: motor power stays off");
        return false;
    }
    GLOGI(kTag, "motor switch host up: FLT %s, EN node %.2f V, IMON %.3f A%s",
          r.fault_line ? "low" : "high", double(r.en_node_v), double(r.imon_a),
          adcOk ? "" : " (ADC down: enable refused)");
    return true;
}

void motorSwitchSetSelfCheck(bool passed) {
    portENTER_CRITICAL(&g_mux);
    g_allowed = passed;
    portEXIT_CRITICAL(&g_mux);
    if (passed) motorSwitchRequestEnable();
}

motorswitch::Refusal motorSwitchRequestEnable() {
    portENTER_CRITICAL(&g_mux);
    const Refusal why = motorswitch::precheck(g_allowed, g_last.fault_line);
    portEXIT_CRITICAL(&g_mux);
    if (why != Refusal::none) return why;
    if (g_task == nullptr) return Refusal::host_down;
    g_enableReq.store(true);
    xTaskNotifyGive(g_task);
    return Refusal::none;
}

void motorSwitchCut() {
    portENTER_CRITICAL(&g_mux);
    ++g_cuts;
    g_sw.cut();
    driveLocked();
    portEXIT_CRITICAL(&g_mux);
    g_enableReq.store(false);
    if (g_task != nullptr) xTaskNotifyGive(g_task);
}

MotorSwitchStatus motorSwitchStatus() {
    const uint32_t s = g_status.load(std::memory_order_relaxed);
    MotorSwitchStatus out;
    out.state = State(s & 0xFFu);
    out.last_fault = Fault((s >> 8) & 0xFFu);
    out.faults = uint16_t(s >> 16);
    return out;
}

bool motorSwitchFaultLine() { return gpio_get_level(pin(BOARD_GPIO_MSW_FLT_N)) == 0; }

uint32_t motorSwitchStackFree() {
    // IDF reports BYTES, not the vanilla FreeRTOS words.
    return g_task != nullptr ? uint32_t(uxTaskGetStackHighWaterMark(g_task)) : 0;
}

}  // namespace valence
