// test_motor_switch -- native doctest suite for the motor switch state machine
// Constraints:
// - Hardware-free: MotorSwitch.h alone, readings handed in, a synthetic
//   microsecond clock. The pins, the ADC and the task are
//   ValenceMotorSwitch.cpp and are bench work (val-091.56).
// - The rules under test are the sequence and the latch: MOTOR_EN never high
//   before a full window, a fault drops both lines and stays down until an
//   explicit request finds the cause clear.
// See: flagship_p4/src/system/MotorSwitch.h, bd val-091.24

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>

#include "../../../flagship_p4/src/system/MotorSwitch.h"

namespace ms = valence::motorswitch;
using ms::Fault;
using ms::Readings;
using ms::Refusal;
using ms::State;

namespace {

constexpr uint64_t kT0 = 1'000'000;

// A healthy board: fault line clear, EN node running, the bank charged.
Readings healthy() {
    Readings r;
    r.fault_line = false;
    r.en_node_v = 1.8f;
    r.imon_a = 0.002f;
    r.motor_v = 35.9f;
    return r;
}

void checkOff(const ms::Switch& s) {
    CHECK_FALSE(s.outputs().motor_en);
    CHECK_FALSE(s.outputs().precharge_en);
}

// Requests and runs the whole window on healthy readings.
void enable(ms::Switch& s, uint64_t& now) {
    REQUIRE(s.requestEnable(true, healthy(), now) == Refusal::none);
    now += ms::kPrechargeUs;
    REQUIRE(s.step(healthy(), now));
    REQUIRE(s.state() == State::on);
}

}  // namespace

TEST_CASE("the window is the derivation: 5 tau at twice the measured drive capacitance") {
    CHECK(ms::kPrechargeUs == 150000);
    const float tau_s = ms::kPrechargeOhms * ms::kDriveInputF;
    CHECK(tau_s == doctest::Approx(0.015f));
    // The doubled bank still clears 99 % inside the window.
    CHECK(1.0 - std::exp(-double(ms::kPrechargeUs) * 1e-6 / (2.0 * double(tau_s))) > 0.99);
    // The ceiling sits under a short's pre-charge current at the lowest bus
    // and far over a charged bank's residual.
    CHECK(ms::kInrushCeilingA < 24.0f / ms::kPrechargeOhms);
    CHECK(ms::kInrushCeilingA > 0.36f * std::exp(-5.0f) * 10.0f);
}

TEST_CASE("boots off, both lines low, nothing moves it but a request") {
    ms::Switch s;
    CHECK(s.state() == State::off);
    CHECK(s.lastFault() == Fault::none);
    checkOff(s);
    CHECK_FALSE(s.step(healthy(), kT0));
    CHECK(s.state() == State::off);
}

TEST_CASE("the sequence: PRECHARGE_EN for the whole window, then MOTOR_EN high and PRECHARGE_EN low") {
    ms::Switch s;
    uint64_t now = kT0;
    REQUIRE(s.requestEnable(true, healthy(), now) == Refusal::none);
    CHECK(s.state() == State::precharging);
    CHECK(s.outputs().precharge_en);
    CHECK_FALSE(s.outputs().motor_en);

    // One microsecond short of the window: still pre-charging.
    CHECK_FALSE(s.windowDue(now + ms::kPrechargeUs - 1));
    CHECK_FALSE(s.step(healthy(), now + ms::kPrechargeUs - 1));
    CHECK(s.state() == State::precharging);
    CHECK_FALSE(s.outputs().motor_en);

    CHECK(s.windowDue(now + ms::kPrechargeUs));
    CHECK(s.step(healthy(), now + ms::kPrechargeUs));
    CHECK(s.state() == State::on);
    CHECK(s.outputs().motor_en);
    CHECK_FALSE(s.outputs().precharge_en);
    CHECK_FALSE(s.windowDue(now + ms::kPrechargeUs));
}

TEST_CASE("refusals: self-check, fault line, EN node; nothing changes on one") {
    ms::Switch s;
    Readings r = healthy();
    CHECK(s.requestEnable(false, r, kT0) == Refusal::self_check);
    r.fault_line = true;
    CHECK(s.requestEnable(true, r, kT0) == Refusal::fault_line);
    r = healthy();
    r.en_node_v = 0.05f;
    CHECK(s.requestEnable(true, r, kT0) == Refusal::en_node);
    r.en_node_v = std::nanf("");
    CHECK(s.requestEnable(true, r, kT0) == Refusal::en_node);
    CHECK(s.state() == State::off);
    checkOff(s);
    // The hub-side precheck answers the first two and leaves the EN node to
    // the switch's own task.
    CHECK(ms::precheck(false, false) == Refusal::self_check);
    CHECK(ms::precheck(true, true) == Refusal::fault_line);
    CHECK(ms::precheck(true, false) == Refusal::none);
}

TEST_CASE("a request while pre-charging or on changes nothing, the window included") {
    ms::Switch s;
    uint64_t now = kT0;
    REQUIRE(s.requestEnable(true, healthy(), now) == Refusal::none);
    // A second request mid-window must not restart the clock.
    CHECK(s.requestEnable(true, healthy(), now + 100000) == Refusal::none);
    CHECK(s.step(healthy(), now + ms::kPrechargeUs));
    CHECK(s.state() == State::on);
    CHECK(s.requestEnable(true, healthy(), now + ms::kPrechargeUs) == Refusal::none);
    CHECK(s.state() == State::on);
}

TEST_CASE("fault line at any time latches faulted, both lines low") {
    for (const bool whileOn : {false, true}) {
        CAPTURE(whileOn);
        ms::Switch s;
        uint64_t now = kT0;
        if (whileOn) {
            enable(s, now);
        } else {
            REQUIRE(s.requestEnable(true, healthy(), now) == Refusal::none);
            now += 20000;
        }
        Readings r = healthy();
        r.fault_line = true;
        CHECK(s.step(r, now));
        CHECK(s.state() == State::faulted);
        CHECK(s.lastFault() == Fault::fault_line);
        CHECK(s.faults() == 1);
        checkOff(s);
        // The latch holds once the line clears: no step leaves faulted.
        CHECK_FALSE(s.step(healthy(), now + 1000));
        CHECK(s.state() == State::faulted);
    }
}

TEST_CASE("EN node drop while pre-charging or on is a fault (a hardware kill)") {
    ms::Switch s;
    uint64_t now = kT0;
    enable(s, now);
    Readings r = healthy();
    r.en_node_v = 0.02f;
    CHECK(s.step(r, now + 5000));
    CHECK(s.lastFault() == Fault::en_node);
    checkOff(s);
}

TEST_CASE("end of window: inrush over the ceiling, unread IMON, MOTOR_V+ short all fault; MOTOR_EN never rises") {
    struct Case {
        float imon;
        float motor_v;
        Fault want;
    };
    const Case cases[] = {
        {0.30f, 35.9f, Fault::inrush},             // a short holding the pre-charge current
        {std::nanf(""), 35.9f, Fault::inrush},     // unread: not proven under the ceiling
        {0.001f, 3.0f, Fault::precharge},          // the bank never charged
    };
    for (const Case& c : cases) {
        ms::Switch s;
        REQUIRE(s.requestEnable(true, healthy(), kT0) == Refusal::none);
        Readings r = healthy();
        r.imon_a = c.imon;
        r.motor_v = c.motor_v;
        CHECK(s.step(r, kT0 + ms::kPrechargeUs));
        CHECK(s.state() == State::faulted);
        CHECK(s.lastFault() == c.want);
        CHECK_FALSE(s.outputs().motor_en);
    }
}

TEST_CASE("no power monitor: MOTOR_V+ unread skips its check, IMON still judges") {
    ms::Switch s;
    REQUIRE(s.requestEnable(true, healthy(), kT0) == Refusal::none);
    Readings r = healthy();
    r.motor_v = std::nanf("");
    CHECK(s.step(r, kT0 + ms::kPrechargeUs));
    CHECK(s.state() == State::on);
}

TEST_CASE("re-enable only by an explicit request, and only once the fault reads clear") {
    ms::Switch s;
    uint64_t now = kT0;
    enable(s, now);
    Readings bad = healthy();
    bad.fault_line = true;
    REQUIRE(s.step(bad, now));
    REQUIRE(s.state() == State::faulted);

    CHECK(s.requestEnable(true, bad, now) == Refusal::fault_line);
    CHECK(s.state() == State::faulted);
    CHECK(s.requestEnable(true, healthy(), now) == Refusal::none);
    CHECK(s.state() == State::precharging);
    CHECK_FALSE(s.outputs().motor_en);
    // The last fault stays reported after a clean recovery.
    CHECK(s.step(healthy(), now + ms::kPrechargeUs));
    CHECK(s.state() == State::on);
    CHECK(s.lastFault() == Fault::fault_line);
    CHECK(s.faults() == 1);
}

TEST_CASE("the cut: pre-charging and on drop to off, faulted stays latched, off stays off") {
    ms::Switch s;
    uint64_t now = kT0;
    s.cut();
    CHECK(s.state() == State::off);

    REQUIRE(s.requestEnable(true, healthy(), now) == Refusal::none);
    s.cut();
    CHECK(s.state() == State::off);
    checkOff(s);
    // The window that was open does not complete after the cut.
    CHECK_FALSE(s.step(healthy(), now + ms::kPrechargeUs));
    CHECK(s.state() == State::off);

    enable(s, now);
    s.cut();
    CHECK(s.state() == State::off);
    checkOff(s);

    Readings bad = healthy();
    bad.fault_line = true;
    REQUIRE(s.requestEnable(true, healthy(), now) == Refusal::none);
    REQUIRE(s.step(bad, now + 1000));
    s.cut();
    CHECK(s.state() == State::faulted);
}

TEST_CASE("fault ordinals are wire values: append-only") {
    CHECK(uint8_t(Fault::none) == 0);
    CHECK(uint8_t(Fault::fault_line) == 1);
    CHECK(uint8_t(Fault::en_node) == 2);
    CHECK(uint8_t(Fault::inrush) == 3);
    CHECK(uint8_t(Fault::precharge) == 4);
    CHECK(uint8_t(State::off) == 0);
    CHECK(uint8_t(State::precharging) == 1);
    CHECK(uint8_t(State::on) == 2);
    CHECK(uint8_t(State::faulted) == 3);
}
