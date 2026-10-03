// test_accessory_io -- native doctest suite for the accessory header's request
// rules
// Constraints:
// - Hardware-free: AccessoryIo.h alone, plus ButtonGesture.h for the AUX
//   gesture that feeds a slot. The LEDC channels, the pads and the BoardIo
//   pass are ValenceAccessoryIo.cpp and are bench work (val-091.69).
// See: flagship_p4/src/system/AccessoryIo.h, bd val-091.30

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstdint>
#include <limits>

#include "../../../flagship_p4/src/system/AccessoryIo.h"

namespace acc = valence::accessory;
using acc::Dir;
using acc::Drive;
using acc::Pwm;
using valence::button::Gesture;

// ---- PWM duty clamp ---------------------------------------------------------

TEST_CASE("duty clamps to 0..1 and NaN is off") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    CHECK(acc::dutyCounts(nan) == 0);
    CHECK(acc::dutyCounts(-inf) == 0);
    CHECK(acc::dutyCounts(-0.5f) == 0);
    CHECK(acc::dutyCounts(-0.0f) == 0);
    CHECK(acc::dutyCounts(0.0f) == 0);
    CHECK(acc::dutyCounts(1.0f) == acc::kDutyMax);
    CHECK(acc::dutyCounts(1.5f) == acc::kDutyMax);
    CHECK(acc::dutyCounts(inf) == acc::kDutyMax);
    CHECK(acc::dutyCounts(0.5f) == 512);    // 511.5 rounds away from zero
    CHECK(acc::dutyCounts(0.25f) == 256);   // 255.75
    CHECK(acc::kDutyMax == 1023);
}

TEST_CASE("a duty request stores the clamped counts on its own output only") {
    acc::Requests r;
    CHECK(r.setDuty(Pwm::pump, 0.5f));
    CHECK(r.duty(Pwm::pump) == 512);
    CHECK(r.duty(Pwm::ext) == 0);
    CHECK(r.setDuty(Pwm::ext, 2.0f));
    CHECK(r.duty(Pwm::ext) == acc::kDutyMax);
    CHECK(r.setDuty(Pwm::pump, std::numeric_limits<float>::quiet_NaN()));
    CHECK(r.duty(Pwm::pump) == 0);
    CHECK_FALSE(r.setDuty(static_cast<Pwm>(7), 0.5f));
    CHECK(r.duty(static_cast<Pwm>(7)) == 0);
}

// ---- off on ESTOP -----------------------------------------------------------

TEST_CASE("an ESTOP zeroes every duty and releases every output; directions stay") {
    acc::Requests r;
    REQUIRE(r.setDuty(Pwm::pump, 0.7f));
    REQUIRE(r.setDuty(Pwm::ext, 0.3f));
    REQUIRE(r.configure(0, Dir::output));
    REQUIRE(r.configure(1, Dir::input_pullup));
    REQUIRE(r.configure(4, Dir::output));
    REQUIRE(r.set(0, true));
    REQUIRE(r.set(4, false));
    CHECK(r.estops() == 0);

    r.estop();
    CHECK(r.estops() == 1);
    CHECK(r.duty(Pwm::pump) == 0);
    CHECK(r.duty(Pwm::ext) == 0);
    for (uint8_t i = 0; i < acc::kGpioCount; ++i) CHECK(r.drive(i) == Drive::released);
    CHECK(r.dir(0) == Dir::output);
    CHECK(r.dir(1) == Dir::input_pullup);
    CHECK(r.dir(4) == Dir::output);
}

TEST_CASE("nothing comes back on by itself; a request after the ESTOP applies") {
    acc::Requests r;
    REQUIRE(r.configure(2, Dir::output));
    REQUIRE(r.set(2, true));
    REQUIRE(r.setDuty(Pwm::pump, 1.0f));
    r.estop();
    r.estop();
    CHECK(r.estops() == 2);
    CHECK(r.drive(2) == Drive::released);
    CHECK(r.duty(Pwm::pump) == 0);

    CHECK(r.setDuty(Pwm::pump, 0.25f));
    CHECK(r.duty(Pwm::pump) == 256);
    CHECK(r.set(2, false));
    CHECK(r.drive(2) == Drive::low);
}

// ---- GPIO direction guard ---------------------------------------------------

TEST_CASE("a pad's direction is set once per boot") {
    acc::Requests r;
    for (uint8_t i = 0; i < acc::kGpioCount; ++i) CHECK(r.dir(i) == Dir::unset);
    CHECK(r.configure(3, Dir::input));
    CHECK_FALSE(r.configure(3, Dir::output));
    CHECK_FALSE(r.configure(3, Dir::input));
    CHECK(r.dir(3) == Dir::input);
    CHECK_FALSE(r.configure(0, Dir::unset));
    CHECK(r.dir(0) == Dir::unset);
    CHECK_FALSE(r.configure(acc::kGpioCount, Dir::output));
    CHECK(r.dir(acc::kGpioCount) == Dir::unset);
}

TEST_CASE("only an output pad takes a level, and it starts released") {
    acc::Requests r;
    CHECK_FALSE(r.set(0, true));   // unset
    REQUIRE(r.configure(1, Dir::input));
    REQUIRE(r.configure(2, Dir::input_pullup));
    REQUIRE(r.configure(3, Dir::output));
    CHECK_FALSE(r.set(1, true));
    CHECK_FALSE(r.set(2, true));
    CHECK(r.drive(1) == Drive::released);
    CHECK(r.drive(2) == Drive::released);
    CHECK(r.drive(3) == Drive::released);
    CHECK(r.set(3, true));
    CHECK(r.drive(3) == Drive::high);
    CHECK(r.set(3, false));
    CHECK(r.drive(3) == Drive::low);
    CHECK_FALSE(r.set(acc::kGpioCount, true));
    CHECK(r.drive(acc::kGpioCount) == Drive::released);
}

// ---- AUX gesture parking ----------------------------------------------------

TEST_CASE("a slot parks press and hold, keeps the newest, and is taken once") {
    acc::GestureSlot s;
    CHECK(s.take() == Gesture::none);
    CHECK(s.park(Gesture::press));
    CHECK(s.take() == Gesture::press);
    CHECK(s.take() == Gesture::none);
    CHECK(s.park(Gesture::press));
    CHECK(s.park(Gesture::hold));
    CHECK(s.take() == Gesture::hold);
    CHECK(s.take() == Gesture::none);
}

TEST_CASE("a stuck press is never parked and never displaces a waiting gesture") {
    acc::GestureSlot s;
    CHECK_FALSE(s.park(Gesture::stuck));
    CHECK_FALSE(s.park(Gesture::none));
    CHECK(s.take() == Gesture::none);
    REQUIRE(s.park(Gesture::press));
    CHECK_FALSE(s.park(Gesture::stuck));
    CHECK(s.take() == Gesture::press);
}

TEST_CASE("an AUX tap through the gesture core lands in its slot once") {
    valence::button::Button b;
    acc::GestureSlot s;
    uint32_t t = 1000;
    auto feed = [&](bool pressed, uint32_t ms) {
        for (uint32_t end = t + ms; t < end; t += 10) s.park(b.sample(pressed, t));
    };
    feed(false, 200);
    feed(true, 200);
    CHECK(s.take() == Gesture::none);   // nothing fires on the way down
    feed(false, 100);
    CHECK(s.take() == Gesture::press);
    CHECK(s.take() == Gesture::none);
}
