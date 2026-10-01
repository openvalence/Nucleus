// test_supervisor -- native doctest suite for the P4 <-> board monitor link
// (Supervisor.h) and the monitor's hardware-free core (monitor_core.c)
// Constraints:
// - Expected values come from the CRC catalog check values and from the
//   schematic's component values (dividers, the regen comparator network),
//   never from the code under test.
// - monitor_core.c is C compiled here as C++; it is included, not linked.
// See: docs/supervisor.md, bd val-091.19

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include "system/Supervisor.h"
#include "../../../flagship_ch32v003/src/monitor_core.c"

namespace {

// ---- the schematic, independently of the firmware constants ---------------------

constexpr double kVref = 1.24;   // TLV431
constexpr double kR501 = 348e3, kR502 = 12.1e3, kR503 = 3.01e6, kRt = 47e3 + 10e3;

// +BUS threshold in mV with CLAMP_TRIM at duty d (0..1) on a VDD of vdd volts.
double thresholdMv(double d, double vdd) {
    const double vth0 = kVref * (1 + kR501 / kR502 + kR501 / kR503 + kR501 / kRt);
    return 1000.0 * (vth0 - kR501 / kRt * d * vdd);
}

uint16_t rawFor(double rail_mv, double ratio, double vdd_mv = 3300) {
    return (uint16_t)std::lround(rail_mv / ratio * 1023.0 / vdd_mv);
}

struct Bench {
    McState s{};
    McInputs in{};
    double vdd = 3300;

    Bench() {
        mc_init(&s);
        setVdd(3300);
        set(SV_CH_VIN_RAW, 36000);
        set(SV_CH_BUS, 35900);
        set(SV_CH_12V, 12000);
        set(SV_CH_5V, 5000);
        set(SV_CH_5V_SYS, 5000);
        set(SV_CH_3V3_ACC, 3300);
        in.raw[SV_CH_SHUNT_TEMP] = 512;   // NTC at 25 C: half of the pull-up
        set(SV_CH_CLAMP_MON, 0);
        in.pump_flt_low = 0;
    }
    void setVdd(double mv) {
        vdd = mv;
        in.raw_vrefint = (uint16_t)std::lround(1200.0 * 1023.0 / mv);
    }
    void set(int ch, double mv) {
        static const double ratio[SV_CH_COUNT] = { 17, 17, 4.9, 2, 2, 2, 1, 1 };
        in.raw[ch] = rawFor(mv, ratio[ch], vdd);
    }
    void clamp(bool on) { set(SV_CH_CLAMP_MON, on ? 2980 : 0); }
    void run(int ticks) {
        for (int i = 0; i < ticks; i++) mc_tick(&s, &in);
    }
    void settle() { run(MC_BOOT_TICKS + MC_STABLE_TICKS + 5); }
};

}  // namespace

// ---- link -------------------------------------------------------------------------

TEST_CASE("checksums match the catalog check values") {
    const uint8_t msg[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    CHECK(sv_crc8(msg, sizeof msg) == 0xF4);                                   // CRC-8/SMBUS
    CHECK((~sv_crc32_update(0xFFFFFFFFu, msg, sizeof msg)) == 0xCBF43926u);    // CRC-32/ISO-HDLC
}

TEST_CASE("every block round-trips, and a damaged block is refused whole") {
    SvStatus st{};
    st.seq = 7;
    st.flags = SV_FLAG_FAULT_N | SV_FLAG_BUDGET;
    st.faults_live = SV_F_12V;
    st.faults_latched = SV_F_12V | SV_F_HEARTBEAT;
    st.warns = SV_W_PUMP_FLT;
    st.first_fault = 3;
    st.test_result = SV_TEST_PASS;
    for (int i = 0; i < SV_CH_COUNT; i++) st.mv[i] = (uint16_t)(1000 * i + 17);
    st.vdd_mv = 3301;
    st.trim_target_mv = 39000;
    st.trim_permille = 279;
    st.clamp_on_permille = 48;

    uint8_t b[SV_STATUS_LEN];
    sv_status_encode(&st, b);
    CHECK(b[0] == SV_REG_STATUS);
    CHECK(b[1] == SV_STATUS_LEN);
    SvStatus back{};
    REQUIRE(sv_status_decode(b, sizeof b, &back) == SV_OK);
    CHECK(std::memcmp(&st, &back, sizeof st) == 0);

    for (unsigned i = 0; i < sizeof b; i++) {
        uint8_t bad[SV_STATUS_LEN];
        std::memcpy(bad, b, sizeof b);
        bad[i] ^= 0x10;
        CHECK(sv_status_decode(bad, sizeof bad, &back) != SV_OK);
    }
    CHECK(sv_status_decode(b, sizeof b - 1, &back) == SV_ERR_LEN);

    SvIdent id{ SV_LINK_VERSION, 0, 1, 2, 0xDEADBEEFu, 0x0C };
    uint8_t ib[SV_IDENT_LEN];
    sv_ident_encode(&id, ib);
    SvIdent idb{};
    REQUIRE(sv_ident_decode(ib, sizeof ib, &idb) == SV_OK);
    CHECK(idb.image_crc32 == 0xDEADBEEFu);
    CHECK(idb.fw_patch == 2);
    CHECK(sv_ident_decode(b, sizeof b, &idb) != SV_OK);   // a STATUS block is not an IDENT

    uint8_t hb[SV_HEARTBEAT_LEN], counter = 0;
    sv_heartbeat_encode(42, hb);
    CHECK(sv_heartbeat_decode(hb, sizeof hb, &counter) == SV_OK);
    CHECK(counter == 42);

    uint8_t cb[SV_COMMAND_LEN], op = 0;
    uint16_t arg = 0;
    sv_command_encode(SV_CMD_CLAMP_TEST, 40, cb);
    CHECK(sv_command_decode(cb, sizeof cb, &op, &arg) == SV_OK);
    CHECK(op == SV_CMD_CLAMP_TEST);
    CHECK(arg == 40);
    CHECK(sv_heartbeat_decode(cb, sizeof cb, &counter) == SV_ERR_LEN);
}

TEST_CASE("version check: any image difference reflashes, a match does not") {
    SvIdent id{ SV_LINK_VERSION, 0, 1, 0, 0x12345678u, 0 };
    CHECK_FALSE(sv_needs_flash(SV_OK, &id, 0x12345678u));
    CHECK(sv_needs_flash(SV_OK, &id, 0x12345679u));          // newer OR older
    CHECK(sv_needs_flash(SV_ERR_CRC, &id, 0x12345678u));     // no answer / garbage
    id.link_version = SV_LINK_VERSION + 1;
    CHECK(sv_needs_flash(SV_OK, &id, 0x12345678u));
}

TEST_CASE("limits: a request can tighten but never loosen") {
    McState s{};
    mc_init(&s);
    SvLimits req = s.base;
    req.lo_mv[0] = 9000;           // looser low edge: refused
    req.hi_mv[0] = 12500;          // tighter high edge: taken
    req.bus_ov_mv = 60000;         // looser: refused
    req.shunt_trip_mv = 300;       // cooler trip: taken
    req.heartbeat_timeout_ms = 0;  // zero is not "tighter"
    mc_on_limits(&s, &req);
    CHECK(s.active.lo_mv[0] == s.base.lo_mv[0]);
    CHECK(s.active.hi_mv[0] == 12500);
    CHECK(s.active.bus_ov_mv == s.base.bus_ov_mv);
    CHECK(s.active.shunt_trip_mv == 300);
    CHECK(s.active.heartbeat_timeout_ms == s.base.heartbeat_timeout_ms);
}

// ---- monitor core -------------------------------------------------------------------

TEST_CASE("boot: FAULT_N is held through the boot window, then released on good rails") {
    Bench b;
    b.run(1);
    CHECK(mc_fault_n(&b.s));
    CHECK(b.s.trim_mode == MC_TRIM_RELEASED);
    b.run(MC_BOOT_TICKS);
    CHECK_FALSE(mc_fault_n(&b.s));
    CHECK(b.s.faults_latched == 0);
    CHECK(std::abs((int)b.s.mv[SV_CH_BUS] - 35900) < 60);
    CHECK(std::abs((int)b.s.vdd_mv - 3300) <= 2);
}

TEST_CASE("+12V low cuts after the debounce and stays latched until cleared") {
    Bench b;
    b.settle();
    b.set(SV_CH_12V, 9000);
    b.run(MC_RAIL_TICKS - 1);
    CHECK_FALSE(mc_fault_n(&b.s));
    b.run(1);
    CHECK(mc_fault_n(&b.s));
    CHECK(b.s.first_fault == 3);   // bit 2, SV_F_12V
    b.set(SV_CH_12V, 12000);
    b.run(5);
    CHECK(b.s.faults_live == 0);
    CHECK(mc_fault_n(&b.s));       // latched
    mc_on_command(&b.s, SV_CMD_CLEAR_LATCHED, 0);
    CHECK_FALSE(mc_fault_n(&b.s));
    CHECK(b.s.first_fault == 0);
}

TEST_CASE("an accessory rail only warns") {
    Bench b;
    b.settle();
    b.set(SV_CH_3V3_ACC, 2000);
    b.run(MC_RAIL_TICKS + 5);
    CHECK((b.s.warns & SV_W_3V3_ACC) != 0);
    CHECK_FALSE(mc_fault_n(&b.s));
}

TEST_CASE("input stage: VIN_RAW present with no bus is a fault; no VIN_RAW is a budget build") {
    Bench b;
    b.set(SV_CH_BUS, 0);
    b.run(MC_BOOT_TICKS + MC_INPUT_TICKS + 5);
    CHECK((b.s.faults_latched & SV_F_INPUT_STAGE) != 0);

    Bench budget;
    budget.set(SV_CH_VIN_RAW, 0);
    budget.set(SV_CH_BUS, 24000);
    budget.settle();
    CHECK(budget.s.budget);
    CHECK(budget.s.faults_latched == 0);
    REQUIRE(budget.s.trim_latched);
    CHECK(std::abs((int)budget.s.trim_target_mv - (24000 + 1200)) < 60);
}

TEST_CASE("trim: latched from a stable VIN_RAW, threshold never below supply plus margin") {
    for (double vdd : { 3150.0, 3300.0, 3450.0 }) {
        for (bool budget : { false, true }) {
            for (int supply = 24000; supply <= 36000; supply += 2000) {
                Bench b;
                b.setVdd(vdd);
                b.set(SV_CH_12V, 12000);
                b.set(SV_CH_5V, 5000);
                b.set(SV_CH_5V_SYS, 5000);
                b.set(SV_CH_3V3_ACC, 3300);
                b.in.raw[SV_CH_SHUNT_TEMP] = 512;
                b.set(SV_CH_VIN_RAW, budget ? 0 : supply);
                b.set(SV_CH_BUS, budget ? supply : supply - 100);
                b.settle();
                CAPTURE(vdd);
                CAPTURE(budget);
                CAPTURE(supply);
                REQUIRE(b.s.trim_latched);
                REQUIRE(b.s.trim_mode == MC_TRIM_DRIVEN);
                const double margin = budget ? 1200 : 3000;
                const double th = thresholdMv(b.s.trim_permille / 1000.0, vdd / 1000.0);
                // The firmware sees the supply through a 10-bit ADC: one count is
                // ~55 mV of rail, and VDD is read through Vrefint the same way.
                CHECK(th >= supply + margin - 150);
                // A target below the trim's reach (24.5 V at full duty) saturates on the safe side.
                if (b.s.trim_permille < 1000) CHECK(th <= supply + margin + 150);
                CHECK(th < 44700);
            }
        }
    }
}

TEST_CASE("clamp conducting with no regen backs the trim off, then calls it stuck") {
    Bench b;
    b.settle();
    REQUIRE(b.s.trim_permille > 0);
    b.clamp(true);   // bus still at the supply: no regen
    b.run(MC_FEED_TICKS);
    CHECK(b.s.trim_backoff);
    CHECK(b.s.trim_permille == 0);
    CHECK((b.s.warns & SV_W_CLAMP_FEEDING) != 0);
    CHECK_FALSE(mc_fault_n(&b.s));
    b.run(MC_STUCK_TICKS + 1);
    CHECK((b.s.faults_latched & SV_F_CLAMP_STUCK) != 0);
}

TEST_CASE("clamp conducting on real regen is left alone") {
    Bench b;
    b.settle();
    b.set(SV_CH_BUS, 39500);   // regen lifted the bus above the supply
    b.clamp(true);
    b.run(200);
    CHECK_FALSE(b.s.trim_backoff);
    CHECK(b.s.trim_permille > 0);
    CHECK(b.s.faults_latched == 0);
}

TEST_CASE("heartbeat: unarmed never trips; a repeated counter does not feed it") {
    Bench b;
    b.settle();
    b.run(2000);
    CHECK_FALSE(mc_fault_n(&b.s));

    mc_on_heartbeat(&b.s, 1);
    for (int i = 0; i < 10; i++) {
        b.run(100);
        mc_on_heartbeat(&b.s, (uint8_t)(i + 2));
    }
    CHECK_FALSE(mc_fault_n(&b.s));

    for (int i = 0; i < 6; i++) {
        b.run(100);
        mc_on_heartbeat(&b.s, 11);   // same counter as the last real one
    }
    CHECK((b.s.faults_latched & SV_F_HEARTBEAT) != 0);
}

TEST_CASE("regen load temperature: warm releases the trim, hot cuts") {
    Bench b;
    b.settle();
    b.in.raw[SV_CH_SHUNT_TEMP] = rawFor(400, 1);   // just past 85 C
    b.run(MC_SHUNT_TICKS);
    CHECK((b.s.warns & SV_W_SHUNT_WARM) != 0);
    CHECK(b.s.trim_permille == 0);
    CHECK(b.s.trim_mode == MC_TRIM_DRIVEN);
    CHECK_FALSE(mc_fault_n(&b.s));
    b.in.raw[SV_CH_SHUNT_TEMP] = rawFor(200, 1);   // past 110 C
    b.run(MC_SHUNT_TICKS);
    CHECK(mc_fault_n(&b.s));
}

TEST_CASE("clamp test: pass, fail, refusal, and no backoff while the trim settles") {
    Bench b;
    b.settle();
    mc_on_command(&b.s, SV_CMD_CLAMP_TEST, 200);   // clamped to 50 ms
    CHECK(b.s.test_ms_left == MC_TEST_MAX_MS);
    b.run(1);
    CHECK(thresholdMv(b.s.trim_permille / 1000.0, 3.3) < 35900);
    b.run(30);
    b.clamp(true);
    b.run(MC_TEST_MAX_MS - 31);
    CHECK(b.s.test_result == SV_TEST_PASS);
    b.run(10);                  // clamp still on while the filter recovers
    CHECK_FALSE(b.s.trim_backoff);
    b.clamp(false);
    b.run(MC_TEST_GRACE_TICKS);

    mc_on_command(&b.s, SV_CMD_CLAMP_TEST, 30);
    b.run(30);
    CHECK(b.s.test_result == SV_TEST_FAIL);

    Bench cold;
    mc_on_command(&cold.s, SV_CMD_CLAMP_TEST, 40);   // nothing latched yet
    CHECK(cold.s.test_result == SV_TEST_FAIL);
    CHECK(cold.s.test_ms_left == 0);
}

TEST_CASE("TRIM_RELEASE holds the reset level; TRIM_RELATCH takes the trim back") {
    Bench b;
    b.settle();
    mc_on_command(&b.s, SV_CMD_TRIM_RELEASE, 0);
    b.run(500);
    CHECK(b.s.trim_mode == MC_TRIM_RELEASED);
    CHECK(b.s.trim_held);
    mc_on_command(&b.s, SV_CMD_TRIM_RELATCH, 0);
    b.run(MC_STABLE_TICKS + 2);
    CHECK(b.s.trim_mode == MC_TRIM_DRIVEN);
    CHECK(b.s.trim_permille > 0);
}

TEST_CASE("the core's status survives the wire") {
    Bench b;
    b.settle();
    mc_on_heartbeat(&b.s, 5);
    b.run(3);
    SvStatus st{};
    mc_status(&b.s, &st);
    uint8_t wire[SV_STATUS_LEN];
    sv_status_encode(&st, wire);
    SvStatus got{};
    REQUIRE(sv_status_decode(wire, sizeof wire, &got) == SV_OK);
    CHECK((got.flags & SV_FLAG_TRIM_LATCH) != 0);
    CHECK((got.flags & SV_FLAG_HB_ARMED) != 0);
    CHECK_FALSE((got.flags & SV_FLAG_FAULT_N) != 0);
    CHECK(got.trim_permille == b.s.trim_permille);
    CHECK(std::abs((int)got.trim_target_mv - 39000) < 60);
}
