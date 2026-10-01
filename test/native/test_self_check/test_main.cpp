// test_self_check -- native doctest suite for the boot self-check table
// Constraints:
// - Hardware-free: the table and the E-stop decode only. The reads live in
//   ValenceSelfCheck.cpp and are bench work (val-091.21).
// - The gate under test is the ruling, not the code: motor power opens only
//   when EVERY entry passed; skipped and pending hold it shut.
// See: flagship_p4/src/system/SelfCheck.h, bd val-091.21

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstring>
#include <set>
#include <string>

#include "../../../flagship_p4/src/system/SelfCheck.h"

using valence::selfcheck::Check;
using valence::selfcheck::EStop;
using valence::selfcheck::Result;
using valence::selfcheck::Table;
namespace sc = valence::selfcheck;

namespace {

void passAll(Table& t) {
    for (size_t i = 0; i < sc::kCheckCount; ++i) t.record(Check(i), Result::pass, "ok %u", unsigned(i));
}

}  // namespace

TEST_CASE("names: one per check, unique, none empty") {
    std::set<std::string> seen;
    for (size_t i = 0; i < sc::kCheckCount; ++i) {
        const char* n = sc::name(Check(i));
        REQUIRE(n != nullptr);
        CHECK(std::strlen(n) > 0);
        CHECK(std::strlen(n) <= 14);   // the report's %-14s column
        seen.insert(n);
    }
    CHECK(seen.size() == sc::kCheckCount);
    CHECK(std::string(sc::name(Check::count_)) == "?");
}

TEST_CASE("order is the ruling's: monitor first, catalog last") {
    CHECK(size_t(Check::board_monitor) == 0);
    CHECK(size_t(Check::power_monitor) < size_t(Check::motor_rail_off));
    CHECK(size_t(Check::motor_rail_off) < size_t(Check::estop));
    CHECK(size_t(Check::catalog) == sc::kCheckCount - 1);
}

TEST_CASE("a fresh table is pending everywhere and holds motor power off") {
    Table t;
    CHECK(t.count(Result::pending) == sc::kCheckCount);
    CHECK_FALSE(t.motorPowerAllowed());
    REQUIRE(t.firstBlocking().has_value());
    CHECK(*t.firstBlocking() == Check::board_monitor);
}

TEST_CASE("every entry passed opens the gate") {
    Table t;
    passAll(t);
    CHECK(t.motorPowerAllowed());
    CHECK_FALSE(t.firstBlocking().has_value());
    CHECK(t.count(Result::pass) == sc::kCheckCount);
}

TEST_CASE("skipped is not passed: one skip holds motor power off") {
    Table t;
    passAll(t);
    t.record(Check::regen_clamp, Result::skipped, "needs the bench");
    CHECK_FALSE(t.motorPowerAllowed());
    REQUIRE(t.firstBlocking().has_value());
    CHECK(*t.firstBlocking() == Check::regen_clamp);
    CHECK(t.count(Result::skipped) == 1);
}

TEST_CASE("one fail holds motor power off and the FIRST non-pass is named") {
    Table t;
    passAll(t);
    t.record(Check::catalog, Result::fail, "late failure");
    t.record(Check::estop, Result::fail, "pressed");
    REQUIRE(t.firstBlocking().has_value());
    CHECK(*t.firstBlocking() == Check::estop);
    CHECK(t.count(Result::fail) == 2);
}

TEST_CASE("one pending entry holds motor power off") {
    Table t;
    for (size_t i = 0; i + 1 < sc::kCheckCount; ++i) t.record(Check(i), Result::pass, "ok");
    CHECK_FALSE(t.motorPowerAllowed());
    CHECK(*t.firstBlocking() == Check::catalog);
}

TEST_CASE("re-recording overwrites: the last reading is the truth") {
    Table t;
    passAll(t);
    t.record(Check::nvs, Result::fail, "first read failed");
    CHECK_FALSE(t.motorPowerAllowed());
    t.record(Check::nvs, Result::pass, "retry %d", 2);
    CHECK(t.motorPowerAllowed());
    CHECK(std::string(t.entry(Check::nvs).reason.data()) == "retry 2");
}

TEST_CASE("reasons format and truncate inside the fixed buffer") {
    Table t;
    t.record(Check::power_monitor, Result::pass, "%s: %.1f C", "INA237", 31.0);
    CHECK(std::string(t.entry(Check::power_monitor).reason.data()) == "INA237: 31.0 C");

    const std::string longText(200, 'x');
    t.record(Check::rails, Result::skipped, "%s", longText.c_str());
    const auto& r = t.entry(Check::rails).reason;
    CHECK(std::strlen(r.data()) == sc::kReasonBytes - 1);
    CHECK(r[sc::kReasonBytes - 1] == '\0');
}

TEST_CASE("an out-of-range check is ignored, never written past the table") {
    Table t;
    passAll(t);
    t.record(Check::count_, Result::fail, "nowhere");
    CHECK(t.motorPowerAllowed());
}

TEST_CASE("E-stop decode: (NC, NO) read HIGH when open") {
    CHECK(sc::decodeEStop(false, true) == EStop::normal);
    CHECK(sc::decodeEStop(true, false) == EStop::pressed);
    CHECK(sc::decodeEStop(true, true) == EStop::unplugged);
    CHECK(sc::decodeEStop(false, false) == EStop::wiring_fault);
}

TEST_CASE("result names") {
    CHECK(std::string(sc::resultName(Result::pass)) == "PASS");
    CHECK(std::string(sc::resultName(Result::fail)) == "FAIL");
    CHECK(std::string(sc::resultName(Result::skipped)) == "SKIPPED");
    CHECK(std::string(sc::resultName(Result::pending)) == "PENDING");
}
