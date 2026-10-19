// test_ota_arm -- native doctest suite for the update request a PAIR double
// press approves, and the idle gate
// Constraints:
// - Hardware-free: OtaArm.h alone, on a synthetic millisecond clock. The
//   routes, the lock and the flash writes are ValenceOta.cpp and are bench
//   work; which press the request owns is test_valence_device's VD-PAIR-11/12.
// See: flagship_p4/src/system/OtaArm.h, bd val-ktal

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstdint>
#include <string_view>

#include "../../../flagship_p4/src/system/OtaArm.h"

namespace ota = valence::ota;
using ota::ArmState;
using valence::motorswitch::State;

TEST_CASE("OA-1: nothing is approved without a request") {
    ota::ArmGate g;
    CHECK_FALSE(g.press(1000));
    CHECK(g.status(7, 1000) == ArmState::expired);
    CHECK_FALSE(g.take(7, 1000));
}

TEST_CASE("OA-2: request, press, one image: the approval is spent once") {
    ota::ArmGate g;
    REQUIRE(g.arm(7, 1000));
    CHECK(g.status(7, 1000) == ArmState::pending);
    CHECK_FALSE(g.take(7, 1001));   // not before the press
    REQUIRE(g.press(5000));
    CHECK(g.status(7, 5000) == ArmState::approved);
    CHECK_FALSE(g.take(8, 5001));   // not for another id
    CHECK(g.take(7, 5002));
    CHECK_FALSE(g.take(7, 5003));   // once
    CHECK(g.status(7, 5003) == ArmState::expired);
}

TEST_CASE("OA-3: a press after the window approves nothing") {
    ota::ArmGate g;
    REQUIRE(g.arm(7, 1000));
    CHECK(g.status(7, 1000 + ota::kArmWindowMs - 1) == ArmState::pending);
    CHECK(g.status(7, 1000 + ota::kArmWindowMs) == ArmState::expired);
    CHECK_FALSE(g.press(1000 + ota::kArmWindowMs));
    CHECK_FALSE(g.take(7, 1000 + ota::kArmWindowMs));
}

TEST_CASE("OA-4: an approval not used in time lapses") {
    ota::ArmGate g;
    REQUIRE(g.arm(7, 1000));
    REQUIRE(g.press(2000));
    CHECK_FALSE(g.take(7, 2000 + ota::kApprovedHoldMs));
    CHECK(g.status(7, 2000 + ota::kApprovedHoldMs) == ArmState::expired);
}

TEST_CASE("OA-5: one live request: a second is refused until the first ends") {
    ota::ArmGate g;
    REQUIRE(g.arm(7, 1000));
    CHECK_FALSE(g.arm(9, 1500));    // pending
    REQUIRE(g.press(2000));
    CHECK_FALSE(g.arm(9, 2500));    // approved
    CHECK(g.status(7, 2500) == ArmState::approved);
    REQUIRE(g.take(7, 3000));
    CHECK(g.arm(9, 3000));          // spent: the next may ask
    CHECK(g.status(7, 3000) == ArmState::expired);
    CHECK(g.status(9, 3000) == ArmState::pending);
}

TEST_CASE("OA-6: an expired request frees the slot; a second press does nothing") {
    ota::ArmGate g;
    REQUIRE(g.arm(7, 1000));
    CHECK(g.arm(9, 1000 + ota::kArmWindowMs));
    REQUIRE(g.press(1000 + ota::kArmWindowMs + 10));
    CHECK_FALSE(g.press(1000 + ota::kArmWindowMs + 20));   // already approved: the press is not its
    CHECK(g.status(9, 1000 + ota::kArmWindowMs + 20) == ArmState::approved);
}

TEST_CASE("OA-7: id 0 never arms and never matches") {
    ota::ArmGate g;
    CHECK_FALSE(g.arm(0, 1000));
    CHECK_FALSE(g.press(1000));
    CHECK(g.status(0, 1000) == ArmState::expired);
}

TEST_CASE("OA-8: the clock wraps") {
    ota::ArmGate g;
    const uint32_t t = 0xFFFFFFFFu - 100;
    REQUIRE(g.arm(7, t));
    CHECK(g.status(7, t + 200) == ArmState::pending);   // past the wrap
    REQUIRE(g.press(t + 300));
    CHECK(g.take(7, t + 400));
}

TEST_CASE("OA-9: the idle gate: parked and motor power off, nothing else") {
    CHECK(ota::idleRefusal(false, State::off) == nullptr);
    CHECK(ota::idleRefusal(false, State::faulted) == nullptr);
    CHECK(ota::idleRefusal(false, State::on) != nullptr);
    CHECK(ota::idleRefusal(false, State::precharging) != nullptr);
    CHECK(ota::idleRefusal(true, State::off) != nullptr);
    CHECK(ota::idleRefusal(true, State::on) != nullptr);
}

TEST_CASE("OA-10: the reported words") {
    CHECK(std::string_view(ota::armStateName(ArmState::pending)) == "pending");
    CHECK(std::string_view(ota::armStateName(ArmState::approved)) == "approved");
    CHECK(std::string_view(ota::armStateName(ArmState::expired)) == "expired");
}
