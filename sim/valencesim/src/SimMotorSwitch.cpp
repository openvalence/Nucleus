// SimMotorSwitch -- system/ValenceMotorSwitch.h on a desktop: by default no
// switch and motor power on from boot; with --motor-switch, the board's own
// MotorSwitch.h machine walking its states on the hub clock
// Constraints:
// - SINGLE-THREADED: every function here, simMotorSwitchTick() included,
//   runs on the sim's one hub thread. No lock: the board's spinlock exists for
//   a cut from another task, which a one-thread twin cannot have.
// - NO DECISION LIVES HERE. Every transition is MotorSwitch.h, compiled
//   verbatim; this file supplies the readings and the clock.
// - The readings are a healthy board: fault line clear, EN node running, the
//   drive's bank charging through R412 as an RC on the hub clock. The window
//   passes, and the twin reaches `on` kPrechargeUs after a request. The one
//   exception is the injected fault window (simMotorSwitchInjectFault()),
//   during which MSW_FLT_N reads low.
// - With the model on, main.cpp declares estop_cuts_power true, so a release
//   lands in PAUSE with home_required: the sequence Phosphor and the probe
//   exercise (estop, release, pre-charge, home, resume).
// See: SimMotorSwitch.h, flagship_p4/src/system/MotorSwitch.h, bd val-091.24

#include "SimMotorSwitch.h"

#include <cmath>

#include "geiger/geiger.h"
#include "hub/ValenceDevice.h"
#include "motion/ValenceMotion.h"
#include "system/ValenceMotorSwitch.h"

namespace valence {
namespace {

using motorswitch::Readings;
using motorswitch::Refusal;
using motorswitch::State;

constexpr const char* kTag = "msw";

// A 36 V supply profile and the EN node's running level (MotorSwitch.h).
constexpr float kBusV = 36.0f;
// motorSwitchPower()'s model: the motor path's draw while powered, a drive
// holding at rest. A modeled figure, not a measurement (the 0x1010 field
// desc says so); motion does not move it.
constexpr float kVirtualHoldW = 4.0f;
constexpr float kEnNodeRunningV = 1.8f;

bool g_model = false;
bool g_allowed = false;
motorswitch::Switch g_sw;
uint64_t g_windowStartUs = 0;   // PRECHARGE_EN rose, for the RC model
uint64_t g_faultFromUs = 0;     // MSW_FLT_N reads low in [from, until)
uint64_t g_faultUntilUs = 0;
bool g_pushedOn = false;        // what the arbiter last heard

Readings readings(uint64_t now_us) {
    Readings r;
    r.fault_line = now_us >= g_faultFromUs && now_us < g_faultUntilUs;
    r.en_node_v = kEnNodeRunningV;
    r.imon_a = 0.0f;
    r.motor_v = 0.0f;
    if (g_sw.state() == State::precharging) {
        const float tau_s = motorswitch::kPrechargeOhms * motorswitch::kDriveInputF;
        const float decay = std::exp(-float(now_us - g_windowStartUs) * 1e-6f / tau_s);
        r.imon_a = kBusV / motorswitch::kPrechargeOhms * decay;
        r.motor_v = kBusV * (1.0f - decay);
    } else if (g_sw.state() == State::on) {
        r.motor_v = kBusV;
    }
    return r;
}

// The board's push, from the one place it happens: every entry into and exit
// from `on` reaches the arbiter. Without the model the power is simply on.
void push() {
    const bool on = !g_model || g_sw.state() == State::on;
    if (on == g_pushedOn) return;
    g_pushedOn = on;
    motionSetMotorPowered(on);
}

}  // namespace

void simMotorSwitchModel() { g_model = true; }

void simMotorSwitchInjectFault(uint64_t from_us, uint64_t until_us) {
    g_faultFromUs = from_us;
    g_faultUntilUs = until_us;
}

void simMotorSwitchTick(uint64_t now_us) {
    if (!g_model) return;
    const Readings r = readings(now_us);
    if (g_sw.step(r, now_us)) {
        if (g_sw.state() == State::on) {
            GLOGI(kTag, "motor power ON after %lu ms pre-charge: IMON %.3f A, MOTOR_V+ %.1f V",
                  static_cast<unsigned long>(motorswitch::kPrechargeUs / 1000), double(r.imon_a),
                  double(r.motor_v));
        } else {
            GLOGE(kTag, "motor switch FAULTED: %s", motorswitch::faultName(g_sw.lastFault()));
        }
    }
    push();
}

// ---- system/ValenceMotorSwitch.h ------------------------------------------------

bool motorSwitchBegin() {
    push();
    return true;
}

void motorSwitchSetSelfCheck(bool passed) {
    g_allowed = passed;
    if (passed) motorSwitchRequestEnable();
}

motorswitch::Refusal motorSwitchRequestEnable() {
    if (!g_model) return Refusal::none;
    const uint64_t now = deviceNowUs();
    const State was = g_sw.state();
    const Refusal why = g_sw.requestEnable(g_allowed, readings(now), now);
    if (why != Refusal::none) {
        GLOGE(kTag, "enable refused: %s", motorswitch::refusalName(why));
    } else if (was != State::precharging && g_sw.state() == State::precharging) {
        g_windowStartUs = now;
        GLOGI(kTag, "pre-charge: PRECHARGE_EN high for %lu ms",
              static_cast<unsigned long>(motorswitch::kPrechargeUs / 1000));
    }
    return why;
}

void motorSwitchCut() {
    if (!g_model) return;
    g_sw.cut();
    push();
}

PowerNow motorSwitchPower() {
    return {kBusV, motorSwitchStatus().state == State::on ? kVirtualHoldW : 0.0f};
}

MotorSwitchStatus motorSwitchStatus() {
    MotorSwitchStatus s;
    if (!g_model) {
        s.state = State::on;
        return s;
    }
    s.state = g_sw.state();
    s.last_fault = g_sw.lastFault();
    s.faults = uint16_t(g_sw.faults());
    return s;
}

}  // namespace valence
