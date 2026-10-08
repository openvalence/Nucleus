// test_pd_source -- native doctest suite for the PD daughterboard's register
// layer, the motor power profile and the self-check's pd-source row
// Constraints:
// - Hardware-free: register bytes in, a contract, a budget and a verdict out.
//   Every PDO and RDO below is assembled by hand from USB PD R3.2's field
//   tables and every register byte from TI SLVUCR7, never from the code under
//   test.
// - The refusal boundary is pinned on both sides of the documented table in
//   PdSource.h, so a knob change that moves it fails here first.
// See: flagship_p4/src/system/PdSource.h, flagship_p4/src/system/SelfCheck.h,
// bd val-091.31

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "../../../flagship_p4/src/system/PdSource.h"
#include "../../../flagship_p4/src/system/SelfCheck.h"

namespace pd = valence::pd;
namespace sc = valence::selfcheck;
using pd::Presence;
using pd::Supply;
using pd::Verdict;

namespace {

// Fixed 28 V 5 A, EPR object 8: B19..10 = 560 (x 50 mV), B9..0 = 500 (x 10 mA).
constexpr uint32_t kPdoFixed28 = 0x0008C1F4;
// Its request: position 8, EPR mode capable (B22), 500 x 10 mA operating and max.
constexpr uint32_t kRdoFixed28 = 0x8047D1F4;
// Fixed 36 V 5 A (720 x 50 mV) and its request at position 9.
constexpr uint32_t kPdoFixed36 = 0x000B41F4;
constexpr uint32_t kRdoFixed36 = 0x9007D1F4;
// Fixed 48 V 5 A (960 x 50 mV) and its request at position 10.
constexpr uint32_t kPdoFixed48 = 0x000F01F4;
constexpr uint32_t kRdoFixed48 = 0xA007D1F4;

pd::Reading ready(uint32_t pdo, uint32_t rdo) {
    pd::Reading r;
    r.presence = Presence::ready;
    r.mode_text = {'A', 'P', 'P', ' ', '\0'};
    r.contract = pd::decodeContract(pdo, rdo);
    return r;
}

pd::Reading absent() { return pd::Reading{}; }

constexpr pd::Ceilings kSet950{950.0f, 50000.0f};   // a fixture, not the factory set (PdSource.h)

// Wider than the reason column, so a line that would be cut is measured whole.
std::string text(const pd::Assessment& a) {
    std::array<char, 160> out{};
    pd::describe(out, a);
    return out.data();
}

}  // namespace

// ---- framing and identity ---------------------------------------------------

TEST_CASE("framing: the byte count leads, a short count or a short read is refused") {
    const std::array<uint8_t, 5> pdo{6, 0xF4, 0xC1, 0x08, 0x00};   // ACTIVE_CONTRACT_PDO is 6 B
    CHECK(pd::blockHolds(pdo, 4));
    CHECK_FALSE(pd::blockHolds(pdo, 5));   // only 4 data bytes were read
    const std::array<uint8_t, 5> shortCount{3, 0, 0, 0, 0};
    CHECK_FALSE(pd::blockHolds(shortCount, 4));
    CHECK_FALSE(pd::blockHolds(std::span<const uint8_t>(), 0));
}

TEST_CASE("framing: fields are little-endian, data byte 1 is bits 7:0") {
    const std::array<uint8_t, 4> b{0xF4, 0xC1, 0x08, 0x00};
    CHECK(pd::le32(b) == kPdoFixed28);
    CHECK(pd::field(0xA5878190u, 31, 30) == 2u);
    CHECK(pd::field(0xFFFFFFFFu, 31, 0) == 0xFFFFFFFFu);
    CHECK(pd::field(0x80000000u, 31, 31) == 1u);
}

TEST_CASE("MODE: the four ASCII characters SLVUCR7 4.1 names, anything else is foreign") {
    const std::array<uint8_t, 4> app{'A', 'P', 'P', ' '};
    const std::array<uint8_t, 4> boot{'B', 'O', 'O', 'T'};
    const std::array<uint8_t, 4> ptch{'P', 'T', 'C', 'H'};
    const std::array<uint8_t, 4> io{0xFF, 0xFF, 0x00, 0x41};   // an I/O expander at 0x21
    const std::array<uint8_t, 4> reversed{' ', 'P', 'P', 'A'};
    CHECK(pd::decodeMode(app) == pd::Mode::app);
    CHECK(pd::decodeMode(boot) == pd::Mode::boot);
    CHECK(pd::decodeMode(ptch) == pd::Mode::patch);
    CHECK(pd::decodeMode(io) == pd::Mode::foreign);
    CHECK(pd::decodeMode(reversed) == pd::Mode::foreign);
    CHECK(std::string(pd::modeText(io).data()) == "???A");
    CHECK(std::string(pd::modeText(app).data()) == "APP ");
}

// ---- INT_EVENT1 / INT_MASK1 bits --------------------------------------------

TEST_CASE("interrupt bits: SLVUCR7 Table 4-7 positions, byte 1 first") {
    pd::IntBits ev{};
    ev[1] = 0x10;   // bit 12, New Contract as Consumer
    CHECK(pd::bitSet(ev, pd::kIntNewContract));
    CHECK(pd::anyContractEvent(ev));
    pd::IntBits patch{};
    patch[10] = 0x01;   // bit 80, Patch Loaded: not a contract event
    CHECK(pd::bitSet(patch, pd::kIntPatchLoaded));
    CHECK_FALSE(pd::anyContractEvent(patch));
    CHECK(pd::anySet(patch));
    CHECK_FALSE(pd::anySet(pd::IntBits{}));
    CHECK_FALSE(pd::bitSet(ev, 88));   // past bit 87: never read, never written
}

TEST_CASE("interrupt mask: read-modify-write keeps the default byte and arms five events") {
    pd::IntBits mask{};
    mask[10] = 0x07;   // bits 80-82, armed by default (SLVUCR7 4.6)
    const pd::IntBits armed = pd::withContractEvents(mask);
    CHECK(armed[0] == 0x0A);    // bits 1 (hard reset) and 3 (plug)
    CHECK(armed[1] == 0x10);    // bit 12 (new contract)
    CHECK(armed[3] == 0x01);    // bit 24 (power status)
    CHECK(armed[4] == 0x02);    // bit 33 (cannot provide)
    CHECK(armed[10] == 0x07);   // untouched
    CHECK(pd::armsContractEvents(armed));
    CHECK_FALSE(pd::armsContractEvents(mask));   // a write that did not take
}

// ---- the contract decode ----------------------------------------------------

TEST_CASE("contract: a fixed EPR 28 V request reads 28 V, 5 A, object 8") {
    const pd::Contract c = pd::decodeContract(kPdoFixed28, kRdoFixed28);
    CHECK(c.supply == Supply::fixed);
    CHECK(c.mv == 28000);
    CHECK(c.ma == 5000);
    CHECK(c.position == 8);
    CHECK_FALSE(c.mismatch);
    CHECK(c.watts() == doctest::Approx(140.0));
}

TEST_CASE("contract: EPR AVS takes its voltage from the request, 25 mV and 50 mA") {
    // APDO 11b/01b, 15-48 V (B15..8 = 150, B25..17 = 480 x 100 mV), PDP 140 W.
    const uint32_t pdo = 0xD3C0968C;
    // Request at object 11: B20..9 = 1440 x 25 mV = 36 V, B6..0 = 70 x 50 mA.
    const uint32_t rdo = 0xB00B4046;
    const pd::Contract c = pd::decodeContract(pdo, rdo);
    CHECK(c.supply == Supply::epr_avs);
    CHECK(c.mv == 36000);
    CHECK(c.ma == 3500);
    CHECK(c.position == 11);
}

TEST_CASE("contract: SPR PPS is 20 mV and 50 mA units") {
    // APDO 11b/00b, 3.3-21 V, 5 A; request object 5: 1000 x 20 mV, 60 x 50 mA.
    const pd::Contract c = pd::decodeContract(0xC1A42164, 0x5007D03C);
    CHECK(c.supply == Supply::pps);
    CHECK(c.mv == 20000);
    CHECK(c.ma == 3000);
    CHECK(c.position == 5);
}

TEST_CASE("contract: a range supply is held at its floor, a battery's power spread over it") {
    // Battery 24-36 V, 150 W; request object 2 for 120 W (480 x 250 mW).
    const pd::Contract b = pd::decodeContract(0x6D078258, 0x200781E0);
    CHECK(b.supply == Supply::battery);
    CHECK(b.mv == 24000);
    CHECK(b.ma == 5000);   // 120 W over 24 V
    // Variable 24-30 V, 4 A; request object 3 at 400 x 10 mA.
    const pd::Contract v = pd::decodeContract(0xA5878190, 0x30064190);
    CHECK(v.supply == Supply::variable);
    CHECK(v.mv == 24000);
    CHECK(v.ma == 4000);
}

TEST_CASE("contract: an all-zero PDO is no explicit contract, the mismatch flag is B26") {
    CHECK(pd::decodeContract(0, 0).supply == Supply::none);
    CHECK(pd::decodeContract(0, 0x1234).mv == 0);
    CHECK(pd::decodeContract(kPdoFixed28, kRdoFixed28 | (1u << 26)).mismatch);
}

// ---- the profile and the refusal boundary -----------------------------------

TEST_CASE("profile: a contract over 36 V is the 48 V build behind its buck") {
    const pd::Profile p28 = pd::profileFor(pd::decodeContract(kPdoFixed28, kRdoFixed28));
    CHECK_FALSE(p28.buck);
    CHECK(p28.bus_mv == 28000);
    CHECK(p28.budget_w == doctest::Approx(126.0));   // 140 W x 0.9
    const pd::Profile p48 = pd::profileFor(pd::decodeContract(kPdoFixed48, kRdoFixed48));
    CHECK(p48.buck);
    CHECK(p48.bus_mv == 36000);
    CHECK(p48.budget_w == doctest::Approx(205.2));   // 240 W x 0.95 x 0.9
}

TEST_CASE("peak: 950 mm/s and 50,000 mm/s^2 draw 141.7 W, idle alone 15 W") {
    CHECK(pd::peakWatts(kSet950) == doctest::Approx(141.667).epsilon(1e-4));
    CHECK(pd::peakWatts(pd::Ceilings{}) == doctest::Approx(15.0));
    CHECK(std::isinf(pd::peakWatts({std::numeric_limits<float>::quiet_NaN(), 50000.0f})));
    CHECK(std::isinf(pd::peakWatts({950.0f, -1.0f})));
}

TEST_CASE("refusal: 950 mm/s and 50,000 mm/s^2 ride 36 V and 48 V at 5 A, never 28 V") {
    CHECK(pd::assess(ready(kPdoFixed36, kRdoFixed36), kSet950).verdict == Verdict::carries);
    CHECK(pd::assess(ready(kPdoFixed48, kRdoFixed48), kSet950).verdict == Verdict::carries);
    const pd::Assessment a28 = pd::assess(ready(kPdoFixed28, kRdoFixed28), kSet950);
    CHECK(a28.verdict == Verdict::over_budget);
    CHECK_FALSE(a28.motorAllowed());
}

TEST_CASE("refusal boundary: 28 V x 5 A at 950 mm/s carries 43,800 mm/s^2, refuses 43,900") {
    const pd::Reading r = ready(kPdoFixed28, kRdoFixed28);
    const pd::Assessment under = pd::assess(r, {950.0f, 43800.0f});
    const pd::Assessment over = pd::assess(r, {950.0f, 43900.0f});
    CHECK(under.verdict == Verdict::carries);
    CHECK(under.motorAllowed());
    CHECK(over.verdict == Verdict::over_budget);
    CHECK_FALSE(over.motorAllowed());
    // The documented table's 36 V x 5 A row: 58,000 mm/s^2 carried, 58,100 not.
    const pd::Reading r36 = ready(kPdoFixed36, kRdoFixed36);
    CHECK(pd::assess(r36, {950.0f, 58000.0f}).verdict == Verdict::carries);
    CHECK(pd::assess(r36, {950.0f, 58100.0f}).verdict == Verdict::over_budget);
}

TEST_CASE("refusal: a NaN ceiling, no contract, or a contract under 24 V never carries") {
    const pd::Reading r36 = ready(kPdoFixed36, kRdoFixed36);
    CHECK(pd::assess(r36, {std::numeric_limits<float>::quiet_NaN(), 1.0f}).verdict == Verdict::over_budget);
    CHECK(pd::assess(ready(0, 0), kSet950).verdict == Verdict::no_contract);
    CHECK(pd::assess(ready(0xC1A42164, 0x5007D03C), pd::Ceilings{}).verdict == Verdict::under_floor);
    // A request for 0 A is no usable contract either.
    CHECK(pd::assess(ready(kPdoFixed36, 0x90000000), pd::Ceilings{}).verdict == Verdict::no_contract);
}

// ---- the absent board and the other non-ready presences ---------------------

TEST_CASE("absent: a silent 0x21 is a DC-input build, motor power allowed, the absence named") {
    const pd::Assessment a = pd::assess(absent(), kSet950);
    CHECK(a.verdict == Verdict::absent);
    CHECK(a.motorAllowed());
    CHECK(text(a) == "no PD daughterboard (0x21 silent): DC input");
}

TEST_CASE("presence: a bus error, a stranger at 0x21 or a controller not in APP refuses") {
    pd::Reading r;
    r.presence = Presence::bus_error;
    r.bus_err = 0x107;   // the IDF timeout code
    CHECK_FALSE(pd::assess(r, pd::Ceilings{}).motorAllowed());
    CHECK(text(pd::assess(r, pd::Ceilings{})) == "Qwiic bus error 0x107 at 0x21: PD source unknown");
    r = pd::Reading{};
    r.presence = Presence::foreign;
    r.mode_text = {'?', '?', '?', 'A', '\0'};
    CHECK_FALSE(pd::assess(r, pd::Ceilings{}).motorAllowed());
    r.presence = Presence::not_ready;
    r.mode_text = {'P', 'T', 'C', 'H', '\0'};
    CHECK_FALSE(pd::assess(r, pd::Ceilings{}).motorAllowed());
    CHECK(text(pd::assess(r, pd::Ceilings{})) == "TPS26750 in PTCH mode: no application config loaded");
}

TEST_CASE("describe: the contract named, every line inside the reason column") {
    CHECK(text(pd::assess(ready(kPdoFixed36, kRdoFixed36), kSet950)) ==
          "36.0V 5.00A fixed #9: peak 142 W within 162 W budget");
    CHECK(text(pd::assess(ready(kPdoFixed28, kRdoFixed28), kSet950)) ==
          "28.0V 5.00A fixed #8: peak 142 W over 126 W budget");
    CHECK(text(pd::assess(ready(kPdoFixed48, kRdoFixed48), kSet950)) ==
          "48.0V 5.00A fixed #10 via buck: peak 142 W within 205 W budget");
    // The widest line at the catalog's ceiling bounds (10,000 mm/s, 100,000
    // mm/s^2: a four-digit peak) still fits the column with its terminator.
    const pd::Assessment wide = pd::assess(ready(0xD3C0968C, 0xD00B47FF), {10000.0f, 100000.0f});
    CHECK(wide.reading.contract.position == 13);
    CHECK(text(wide).size() < sc::kReasonBytes);
}

// ---- the self-check's pd-source row and the narrowed bus window -------------

TEST_CASE("self-check: absent PASSES naming the absence, an over-budget contract FAILS") {
    sc::Table t;
    sc::judgePdSource(t, pd::assess(absent(), kSet950));
    CHECK(t.entry(sc::Check::pd_source).result == sc::Result::pass);
    CHECK(std::string(t.entry(sc::Check::pd_source).reason.data()) ==
          "no PD daughterboard (0x21 silent): DC input");
    sc::judgePdSource(t, pd::assess(ready(kPdoFixed28, kRdoFixed28), kSet950));
    CHECK(t.entry(sc::Check::pd_source).result == sc::Result::fail);
    sc::judgePdSource(t, pd::assess(ready(kPdoFixed36, kRdoFixed36), kSet950));
    CHECK(t.entry(sc::Check::pd_source).result == sc::Result::pass);
    CHECK(std::string(sc::name(sc::Check::pd_source)) == "pd-source");
    CHECK(size_t(sc::Check::pd_source) < size_t(sc::Check::bus_window));
}

TEST_CASE("bus window: no daughterboard keeps 24-36 V, a contract narrows it to its bus") {
    const sc::BusWindowMv dc = sc::busWindowFor(pd::assess(absent(), kSet950));
    CHECK(dc.min_mv == 21600);
    CHECK(dc.max_mv == 39600);
    const sc::BusWindowMv w28 = sc::busWindowFor(pd::assess(ready(kPdoFixed28, kRdoFixed28), kSet950));
    CHECK(w28.min_mv == 25200);
    CHECK(w28.max_mv == 30800);
    const sc::BusWindowMv w48 = sc::busWindowFor(pd::assess(ready(kPdoFixed48, kRdoFixed48), kSet950));
    CHECK(w48.min_mv == 32400);   // the buck's 36 V, not the contract's 48
    CHECK(w48.max_mv == 39600);

    SvStatus s{};
    s.mv[SV_CH_VIN_RAW] = 36100;
    s.mv[SV_CH_BUS] = 35900;   // a 36 V bus under a 28 V contract: the wrong supply
    sc::Table t;
    sc::judgeBusWindow(t, s, w28);
    CHECK(t.entry(sc::Check::bus_window).result == sc::Result::fail);
    s.mv[SV_CH_BUS] = 27800;
    sc::judgeBusWindow(t, s, w28);
    CHECK(t.entry(sc::Check::bus_window).result == sc::Result::pass);
}
