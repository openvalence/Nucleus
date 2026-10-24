// BarnacleBoard -- the barnacle's hosts for the Flagship parts the OSSM R+D
// board does not have: the motor switch, the E-stop input, the fan and the PD
// source
// Constraints:
// - Barnacle only: src/CMakeLists.txt links this file in place of
//   ValenceMotorSwitch.cpp, ValenceEstopInput.cpp, ValenceFan.cpp and
//   ValencePdSource.cpp, so every caller keeps its door.
// - THERE IS NO MOTOR SWITCH. Software cannot cut the servo supply; the
//   latching E-stop in it does (guides/barnacle, step 5). The switch door is
//   the MOTION GATE alone: `on` from the self-check's pass (or a release)
//   until a cut, `off` otherwise, pushed to the arbiter like the Flagship's.
//   A cut parks the emitter and is an ESTOP's whole stop on this board.
// - THE GATE'S ONE RACE: a cut on one task beside an enable on another. Each
//   moves the state under g_mux and bumps g_gen; the enable pushes `on` and
//   re-reads both after, pushing `off` again when a cut came between and no
//   later enable followed it. The later command wins, and the arbiter never
//   stays powered with the state off. Inside g_mux: stores, no log, no call.
// - No E-stop input: the reading is released, known, masked. Nothing reads a
//   contact; the hub's release is never held by one (ValenceDevice.cpp).
// - The power reading is the helper's: J2's supply voltage, and its draw only
//   once the current is calibrated (bd val-omh3.4). Stale reads none, never a
//   0 standing in for one.
// See: BarnacleLink.h, ValenceMotorSwitch.h, ValenceEstopInput.h,
// ValenceFan.h, ValencePdSource.h, bd val-omh3

#include <atomic>
#include <cstdint>
#include <optional>

#include <freertos/FreeRTOS.h>
#include <sdkconfig.h>

#include "geiger/geiger.h"
#include "motion/ValenceMotion.h"
#include "system/BarnacleLink.h"
#include "system/ValenceEstopInput.h"
#include "system/ValenceFan.h"
#include "system/ValenceMotorSwitch.h"
#include "system/ValencePdSource.h"

#ifndef CONFIG_NUCLEUS_BOARD_BARNACLE
#error "BarnacleBoard.cpp is the barnacle's host; src/CMakeLists.txt links it only there"
#endif

namespace valence {

namespace {

constexpr const char* kTag = "msw";

using motorswitch::Refusal;
using motorswitch::State;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
// Under g_mux.
bool g_allowed = false;   // the self-check's verdict
State g_state = State::off;
uint32_t g_gen = 0;       // bumped by every state change
// Written under g_mux, read anywhere.
std::atomic<uint8_t> g_published{uint8_t(State::off)};

void setLocked(State s) {
    g_state = s;
    ++g_gen;
    g_published.store(uint8_t(s), std::memory_order_relaxed);
}

}  // namespace

// ---- the motor switch door: the motion gate ----------------------------------------

bool motorSwitchBegin() {
    GLOGI(kTag, "no motor switch on this board: the E-stop breaks the servo supply, the gate only "
                "parks the emitter");
    return true;
}

void motorSwitchSetSelfCheck(bool passed) {
    portENTER_CRITICAL(&g_mux);
    g_allowed = passed;
    portEXIT_CRITICAL(&g_mux);
    if (passed) motorSwitchRequestEnable();
}

// No PD source on this board.
void motorSwitchSetSourceOk(bool) {}

motorswitch::Refusal motorSwitchRequestEnable() {
    portENTER_CRITICAL(&g_mux);
    if (!g_allowed) {
        portEXIT_CRITICAL(&g_mux);
        return Refusal::self_check;
    }
    const bool was = g_state == State::on;
    setLocked(State::on);
    const uint32_t gen = g_gen;
    portEXIT_CRITICAL(&g_mux);
    motionSetMotorPowered(true);
    portENTER_CRITICAL(&g_mux);
    const bool cutSince = g_gen != gen && g_state != State::on;
    portEXIT_CRITICAL(&g_mux);
    if (cutSince) {
        motionSetMotorPowered(false);
        return Refusal::none;
    }
    if (!was) GLOGI(kTag, "motion gate OPEN: the servo supply is the E-stop's, not switched here");
    return Refusal::none;
}

void motorSwitchCut() {
    portENTER_CRITICAL(&g_mux);
    setLocked(State::off);
    portEXIT_CRITICAL(&g_mux);
    motionSetMotorPowered(false);
}

MotorSwitchStatus motorSwitchStatus() {
    MotorSwitchStatus s;
    s.state = State(g_published.load(std::memory_order_relaxed));
    return s;
}

bool motorSwitchFaultLine() { return false; }

PowerNow motorSwitchPower() {
    const HelperReading h = helperReading();
    PowerNow p;
    if (!h.fresh || h.status.motor_mv == barnacle::kNoMillivolts) return p;
    const float volts = float(h.status.motor_mv) / 1000.0f;
    p.bus_v = volts;
    if ((h.status.flags & barnacle::kFlagCalibrated) != 0 && h.status.current_ma != barnacle::kNoCurrent) {
        const float watts = volts * float(h.status.current_ma) / 1000.0f;
        p.draw_w = watts > 0.0f ? watts : 0.0f;
    }
    return p;
}

std::optional<float> motorSwitchThermVolts() { return std::nullopt; }

uint32_t motorSwitchStackFree() { return 0; }   // no task

// ---- the E-stop input door: none fitted --------------------------------------------

void estopInputBegin() {
    GLOGI("estop", "no E-stop input on this board: the hardware E-stop breaks the servo supply");
}

void estopInputService(uint32_t) {}

estop::Reading estopInputRead() {
    estop::Reading r;
    r.state = estop::Contacts::released;
    r.known = true;
    r.masked = true;
    return r;
}

// ---- the fan door: none fitted -------------------------------------------------------

bool fanBegin() { return false; }
void fanService(uint32_t) {}
FanStatus fanStatus() {
    FanStatus s;
    s.fitted = false;
    return s;
}

// ---- the PD source door: a DC-input board ---------------------------------------------

bool pdSourceBegin() { return false; }
void pdSourceService(uint32_t) {}
pd::Assessment pdSourceAssessNow() { return pd::Assessment{}; }

}  // namespace valence
