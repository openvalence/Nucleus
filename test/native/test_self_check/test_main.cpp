// test_self_check -- native doctest suite for the boot self-check table
// Constraints:
// - Hardware-free: the table and the board-monitor judges over sealed
//   Supervisor.h blocks. The reads live in ValenceSelfCheck.cpp and are
//   bench work (val-091.21); the E-stop decode is test_estop_input's.
// - The gate under test is the ruling, not the code: motor power opens only
//   when EVERY pre-enable entry passed; skipped and pending hold it shut.
//   The post-enable drive-link row never gates (val-091.29).
// See: flagship_p4/src/system/SelfCheck.h, bd val-091.21

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstring>
#include <set>
#include <string>

#include "../../../flagship_p4/src/system/SelfCheck.h"

using valence::selfcheck::Check;
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

TEST_CASE("result names") {
    CHECK(std::string(sc::resultName(Result::pass)) == "PASS");
    CHECK(std::string(sc::resultName(Result::fail)) == "FAIL");
    CHECK(std::string(sc::resultName(Result::skipped)) == "SKIPPED");
    CHECK(std::string(sc::resultName(Result::pending)) == "PENDING");
}

// ---- board monitor judges (Supervisor.h blocks) -----------------------------

namespace {

SvStatus healthyStatus() {
    SvStatus s{};
    s.mv[SV_CH_VIN_RAW] = 36100;
    s.mv[SV_CH_BUS] = 35900;
    s.mv[SV_CH_12V] = 12070;
    s.mv[SV_CH_5V] = 5050;
    s.mv[SV_CH_5V_SYS] = 5000;
    s.mv[SV_CH_3V3_ACC] = 3300;
    return s;
}

}  // namespace

TEST_CASE("monitor IDENT: a sealed block on this link version passes") {
    SvIdent in{};
    in.link_version = SV_LINK_VERSION;
    in.fw_major = 0;
    in.fw_minor = 1;
    in.fw_patch = 2;
    in.image_crc32 = 0xDEADBEEFu;
    uint8_t wire[SV_IDENT_LEN] = {};
    sv_ident_encode(&in, wire);
    SvIdent out{};
    Table t;
    sc::judgeMonitorIdent(t, sv_ident_decode(wire, SV_IDENT_LEN, &out), out);
    CHECK(t.entry(Check::board_monitor).result == Result::pass);
    CHECK(std::string(t.entry(Check::board_monitor).reason.data()).find("0.1.2") != std::string::npos);

    wire[3] ^= 0xFF;   // magic byte, CRC now wrong too: the block is refused whole
    sc::judgeMonitorIdent(t, sv_ident_decode(wire, SV_IDENT_LEN, &out), out);
    CHECK(t.entry(Check::board_monitor).result == Result::fail);
}

TEST_CASE("monitor IDENT: another link version fails") {
    SvIdent id{};
    id.link_version = uint8_t(SV_LINK_VERSION + 1);
    Table t;
    sc::judgeMonitorIdent(t, SV_OK, id);
    CHECK(t.entry(Check::board_monitor).result == Result::fail);
}

TEST_CASE("rails and bus window: a healthy STATUS passes both") {
    Table t;
    const SvStatus s = healthyStatus();
    sc::judgeRails(t, s);
    sc::judgeBusWindow(t, s);
    sc::judgeRegenClamp(t, s);
    CHECK(t.entry(Check::rails).result == Result::pass);
    CHECK(t.entry(Check::bus_window).result == Result::pass);
    // The clamp TEST is not sequenced, so a quiet clamp is still not a pass.
    CHECK(t.entry(Check::regen_clamp).result == Result::skipped);
}

TEST_CASE("rails: a motion-critical rail fault fails, an accessory warn passes") {
    Table t;
    SvStatus s = healthyStatus();
    s.warns = SV_W_3V3_ACC;
    sc::judgeRails(t, s);
    CHECK(t.entry(Check::rails).result == Result::pass);
    s.faults_live = SV_F_12V;
    sc::judgeRails(t, s);
    CHECK(t.entry(Check::rails).result == Result::fail);
}

TEST_CASE("untrusted readings never pass: first scan pending, monitor VDD out") {
    Table t;
    SvStatus s = healthyStatus();
    s.faults_live = SV_F_BOOT;
    sc::judgeRails(t, s);
    sc::judgeBusWindow(t, s);
    CHECK(t.entry(Check::rails).result == Result::skipped);
    CHECK(t.entry(Check::bus_window).result == Result::skipped);
    s.faults_live = SV_F_VDD;
    sc::judgeRails(t, s);
    sc::judgeBusWindow(t, s);
    CHECK(t.entry(Check::rails).result == Result::fail);
    CHECK(t.entry(Check::bus_window).result == Result::fail);
}

TEST_CASE("bus window: input stage, overvoltage, and the 24-36 V window") {
    Table t;
    SvStatus s = healthyStatus();
    s.faults_live = SV_F_INPUT_STAGE;
    sc::judgeBusWindow(t, s);
    CHECK(t.entry(Check::bus_window).result == Result::fail);
    s.faults_live = SV_F_BUS_OV;
    sc::judgeBusWindow(t, s);
    CHECK(t.entry(Check::bus_window).result == Result::fail);
    s.faults_live = 0;
    s.mv[SV_CH_BUS] = 0;   // USB-only bench: no motor supply
    sc::judgeBusWindow(t, s);
    CHECK(t.entry(Check::bus_window).result == Result::fail);
    s.mv[SV_CH_BUS] = 24000;
    sc::judgeBusWindow(t, s);
    CHECK(t.entry(Check::bus_window).result == Result::pass);
    s.mv[SV_CH_BUS] = 44500;   // at the clamp: outside the supply window
    sc::judgeBusWindow(t, s);
    CHECK(t.entry(Check::bus_window).result == Result::fail);
}

TEST_CASE("regen clamp: a live clamp fault fails") {
    Table t;
    SvStatus s = healthyStatus();
    s.faults_live = SV_F_CLAMP_STUCK;
    sc::judgeRegenClamp(t, s);
    CHECK(t.entry(Check::regen_clamp).result == Result::fail);
    s.faults_live = SV_F_SHUNT_HOT;
    sc::judgeRegenClamp(t, s);
    CHECK(t.entry(Check::regen_clamp).result == Result::fail);
}

TEST_CASE("trust ledger: loaded passes with its count, absent is factory fresh, rejected fails") {
    Table t;
    sc::LedgerBoot b;
    b.hubUp = true;
    b.loaded = true;
    b.paired = 3;
    sc::judgeTrustLedger(t, b);
    CHECK(t.entry(Check::trust_ledger).result == Result::pass);
    CHECK(std::string(t.entry(Check::trust_ledger).reason.data()) == "3 paired");

    b = sc::LedgerBoot{};
    b.hubUp = true;
    sc::judgeTrustLedger(t, b);
    CHECK(t.entry(Check::trust_ledger).result == Result::pass);
    CHECK(std::string(t.entry(Check::trust_ledger).reason.data()) == "factory fresh: no ledger stored");

    b.rejected = true;
    sc::judgeTrustLedger(t, b);
    CHECK(t.entry(Check::trust_ledger).result == Result::fail);

    // Never a skip: the row is a real check now, and a hub that never ran
    // its load is a failure, not an unknown.
    sc::judgeTrustLedger(t, sc::LedgerBoot{});
    CHECK(t.entry(Check::trust_ledger).result == Result::fail);
}

// ---- the drive link: the one post-enable row (val-091.29) ---------------------

TEST_CASE("drive link is the one post-enable row: it never gates, either way") {
    CHECK(sc::postEnable(Check::drive_link));
    for (size_t i = 0; i < sc::kCheckCount; ++i)
        if (Check(i) != Check::drive_link) CHECK_FALSE(sc::postEnable(Check(i)));
    Table t;
    passAll(t);
    t.record(Check::drive_link, Result::pending, "probed after motor power");
    CHECK(t.motorPowerAllowed());   // the drive answers only once powered
    t.record(Check::drive_link, Result::fail, "no answer");
    CHECK(t.motorPowerAllowed());   // reported, never adjudicated
    CHECK(t.count(Result::fail) == 1);
    t.record(Check::estop, Result::fail, "pressed");
    REQUIRE(t.firstBlocking().has_value());
    CHECK(*t.firstBlocking() == Check::estop);
}

namespace {

std::string driveReason(const valence::aim::Probe& p, Result expect) {
    Table t;
    sc::judgeDriveLink(t, p);
    CHECK(t.entry(Check::drive_link).result == expect);
    return t.entry(Check::drive_link).reason.data();
}

bool has(const std::string& s, const char* part) { return s.find(part) != std::string::npos; }

}  // namespace

TEST_CASE("drive link row: every probe outcome names its cause") {
    namespace aim = valence::aim;
    using aim::ProbeOutcome;
    aim::Probe p;
    CHECK(has(driveReason(p, Result::pending), "after motor power"));

    p.outcome = ProbeOutcome::no_answer;
    CHECK(has(driveReason(p, Result::fail), "no answer at 19200, 115200, 38400 or 9600 baud"));

    p = aim::Probe{};
    p.outcome = ProbeOutcome::crc;
    p.baud = 19200;
    CHECK(has(driveReason(p, Result::fail), "frames at 19200 baud, none valid"));

    p = aim::Probe{};
    p.outcome = ProbeOutcome::wrong_model;
    p.reg = aim::reg::device_address;
    p.exception = 2;
    CHECK(has(driveReason(p, Result::fail), "exception 2 reading 0x15 device-address"));
    p.exception = 0;
    p.value = 7;
    CHECK(has(driveReason(p, Result::fail), "0x15 device-address reads 7"));

    p = aim::Probe{};
    p.outcome = ProbeOutcome::lost;
    p.baud = 115200;
    p.reg = aim::reg::standstill;
    CHECK(has(driveReason(p, Result::fail), "answered at 115200 baud, then silent at 0x18 standstill"));

    p = aim::Probe{};
    p.outcome = ProbeOutcome::modbus_enabled;
    p.modbusEnable = 1;
    CHECK(has(driveReason(p, Result::fail), "ignores quadrature"));
    p.outcome = ProbeOutcome::output_off;
    CHECK(has(driveReason(p, Result::fail), "drive output disabled"));
    p.outcome = ProbeOutcome::not_quadrature;
    CHECK(has(driveReason(p, Result::fail), "not read as quadrature"));
    p.outcome = ProbeOutcome::alarm_inverted;
    p.positionKp = 3001;
    CHECK(has(driveReason(p, Result::fail), "3001 is odd: WR normally closed"));
}

TEST_CASE("drive link row: a pass names baud, temperature, encoder, alarm and stall alarm, and fits") {
    namespace aim = valence::aim;
    aim::Probe p;
    p.outcome = aim::ProbeOutcome::ok;
    p.baud = 19200;
    p.temperatureC = 31;
    p.encoder = -123456;
    p.standstill = 600;
    CHECK(driveReason(p, Result::pass) == "AIM at 19200, 31 C, encoder -123456, alarm 0x00, stall alarm off");
    p.standstill = 603;
    CHECK(has(driveReason(p, Result::pass), "stall alarm 3"));
    // The widest values still fit the reason buffer whole.
    p.baud = 115200;
    p.temperatureC = 100;
    p.encoder = INT32_MIN;
    p.alarmCode = aim::kAlarmOverVoltage;
    p.standstill = 600;
    const std::string wide = driveReason(p, Result::pass);
    CHECK(has(wide, "stall alarm off"));
    CHECK(wide.size() < sc::kReasonBytes - 1);
}
