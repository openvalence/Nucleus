// test_estop_input -- native doctest suite for the E-stop contacts' reading:
// the decode table, the debounce, the chatter bound and the bench mask
// Constraints:
// - Hardware-free: EstopInput.h alone, the two pad levels and a synthetic
//   millisecond clock handed in. The pads, the BoardIo poll and the hub's
//   latch are ValenceEstopInput.cpp, bench work, and test_valence_device.
// - Levels are as the pads read them: HIGH means the contact is open.
// See: flagship_p4/src/system/EstopInput.h, bd val-091.23

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstdint>

#include "../../../flagship_p4/src/system/EstopInput.h"

namespace es = valence::estop;
using es::Contacts;
using es::Reading;

namespace {

constexpr uint32_t kPollMs = 10;

// (NC, NO) pad levels for each contact state.
struct Pads {
    bool nc;
    bool no;
};
constexpr Pads kReleased{false, true};
constexpr Pads kPressed{true, false};
constexpr Pads kUnplugged{true, true};
constexpr Pads kWiringFault{false, false};

// Feeds `p` every kPollMs from `t` for `ms`; counts reading changes. Counted
// in samples, so the clock may wrap mid-hold.
struct Rig {
    explicit Rig(bool benchMask = false) : r(benchMask) {}

    es::Reader r;
    uint32_t t = 5000;
    int changes = 0;

    void hold(Pads p, uint32_t ms) {
        for (uint32_t n = 0; n < ms / kPollMs; ++n, t += kPollMs)
            if (r.sample(p.nc, p.no, t)) ++changes;
    }
    Reading now() const { return r.reading(); }
};

}  // namespace

TEST_CASE("decode: NC must read closed and NO open for released") {
    CHECK(es::decode(kReleased.nc, kReleased.no) == Contacts::released);
    CHECK(es::decode(kPressed.nc, kPressed.no) == Contacts::pressed);
    CHECK(es::decode(kUnplugged.nc, kUnplugged.no) == Contacts::unplugged);
    CHECK(es::decode(kWiringFault.nc, kWiringFault.no) == Contacts::wiring_fault);
}

TEST_CASE("the reading is unknown until the first state holds the debounce") {
    Rig g;
    g.hold(kReleased, es::kDebounceMs);
    CHECK_FALSE(g.now().known);
    CHECK_FALSE(g.now().stops());
    g.hold(kReleased, 2 * kPollMs);
    CHECK(g.now().known);
    CHECK(g.now().state == Contacts::released);
    CHECK_FALSE(g.now().stops());
    CHECK(g.changes == 1);
}

TEST_CASE("a press stops after the debounce, never before") {
    Rig g;
    g.hold(kReleased, 100);
    g.hold(kPressed, es::kDebounceMs);
    CHECK(g.now().state == Contacts::released);
    g.hold(kPressed, kPollMs);
    CHECK(g.now().state == Contacts::pressed);
    CHECK(g.now().stops());
}

TEST_CASE("a glitch shorter than the debounce changes nothing") {
    Rig g;
    g.hold(kReleased, 100);
    const int before = g.changes;
    g.hold(kPressed, es::kDebounceMs - kPollMs);
    g.hold(kReleased, 200);
    CHECK(g.changes == before);
    CHECK(g.now().state == Contacts::released);
}

TEST_CASE("a slammed press reads pressed, never the both-open moment between the contacts") {
    Rig g;
    g.hold(kReleased, 100);
    g.hold(kUnplugged, 20);   // NC opened, NO not yet closed
    g.hold(kPressed, 100);
    CHECK(g.now().state == Contacts::pressed);
    CHECK(g.changes == 2);    // the boot settle, then the press
}

TEST_CASE("released again is reported, and only reported") {
    Rig g;
    g.hold(kReleased, 100);
    g.hold(kPressed, 100);
    REQUIRE(g.now().stops());
    g.hold(kReleased, 100);
    CHECK(g.now().state == Contacts::released);
    CHECK_FALSE(g.now().stops());
}

TEST_CASE("both open and both closed are faults that stop (release build)") {
    Rig open;
    open.hold(kUnplugged, 100);
    CHECK(open.now().state == Contacts::unplugged);
    CHECK(open.now().stops());
    CHECK_FALSE(open.now().masked);

    Rig closed;
    closed.hold(kWiringFault, 100);
    CHECK(closed.now().state == Contacts::wiring_fault);
    CHECK(closed.now().stops());
}

TEST_CASE("contacts that never settle read wiring_fault within the chatter bound") {
    Rig g;
    g.hold(kReleased, 100);
    uint32_t elapsed = 0;
    while (g.now().state == Contacts::released && elapsed <= es::kChatterMs + kPollMs) {
        g.hold(kPressed, kPollMs);
        g.hold(kReleased, kPollMs);
        elapsed += 2 * kPollMs;
    }
    CHECK(g.now().state == Contacts::wiring_fault);
    CHECK(g.now().stops());
    CHECK(elapsed <= es::kChatterMs + 2 * kPollMs);
    // A clean state that then holds is accepted again.
    g.hold(kReleased, 100);
    CHECK(g.now().state == Contacts::released);
}

TEST_CASE("bench mask: unplugged reads released and masked; pressed and miswired still stop") {
    Rig bench(true);
    bench.hold(kUnplugged, 100);
    CHECK(bench.now().known);
    CHECK(bench.now().state == Contacts::released);
    CHECK(bench.now().masked);
    CHECK_FALSE(bench.now().stops());

    bench.hold(kPressed, 100);
    CHECK(bench.now().state == Contacts::pressed);
    CHECK_FALSE(bench.now().masked);
    CHECK(bench.now().stops());

    Rig miswired(true);
    miswired.hold(kWiringFault, 100);
    CHECK(miswired.now().stops());
}

TEST_CASE("the mask exists only under the flag") {
    Rig release(false);
    release.hold(kUnplugged, 100);
    CHECK(release.now().stops());
    CHECK_FALSE(release.now().masked);
}

TEST_CASE("one byte carries the whole reading") {
    for (int s = 0; s < 4; ++s) {
        for (int k = 0; k < 2; ++k) {
            for (int m = 0; m < 2; ++m) {
                Reading r;
                r.state = Contacts(s);
                r.known = k != 0;
                r.masked = m != 0;
                CHECK(Reading::unpack(r.pack()) == r);
            }
        }
    }
    CHECK_FALSE(Reading::unpack(0).known);   // the boot value is "not read"
}

TEST_CASE("the clock may wrap") {
    Rig g;
    g.t = UINT32_MAX - 55;
    g.hold(kReleased, 100);
    g.hold(kPressed, 100);
    CHECK(g.now().state == Contacts::pressed);
}
