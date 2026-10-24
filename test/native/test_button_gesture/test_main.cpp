// test_button_gesture -- native doctest suite for the press / hold core
// Constraints:
// - Hardware-free: ButtonGesture.h alone, the raw level and a synthetic
//   millisecond clock handed in. The pad, its pull-up and the poll task are
//   ValenceButtons.cpp and are bench work.
// See: flagship_p4/src/system/ButtonGesture.h, bd val-091.26

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstdint>
#include <vector>

#include "../../../flagship_p4/src/system/ButtonGesture.h"

namespace btn = valence::button;
using btn::Gesture;

namespace {

constexpr uint32_t kPollMs = 10;

// Feeds `level` every kPollMs from `t` for `ms`; collects every gesture.
struct Rig {
    btn::Button b;
    uint32_t t = 1000;
    std::vector<Gesture> seen;

    void hold(bool level, uint32_t ms) {
        for (uint32_t end = t + ms; t < end; t += kPollMs) {
            const Gesture g = b.sample(level, t);
            if (g != Gesture::none) seen.push_back(g);
        }
    }
    // Contact bounce: alternate levels every 1 ms for `ms`.
    void bounce(uint32_t ms) {
        for (uint32_t i = 0; i < ms; ++i, ++t) {
            const Gesture g = b.sample((i & 1u) != 0, t);
            if (g != Gesture::none) seen.push_back(g);
        }
    }
};

}  // namespace

TEST_CASE("a short press fires press on release, nothing on the way down") {
    Rig r;
    r.hold(false, 200);
    r.hold(true, 300);
    CHECK(r.seen.empty());
    CHECK(r.b.down());
    r.hold(false, 100);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == Gesture::press);
}

TEST_CASE("a hold fires hold, never also press") {
    Rig r;
    r.hold(false, 200);
    r.hold(true, btn::kHoldMs - 100);
    CHECK_FALSE(r.b.holdReached(r.t));
    r.hold(true, 200);
    CHECK(r.b.holdReached(r.t));
    CHECK(r.seen.empty());
    r.hold(false, 100);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == Gesture::hold);
}

TEST_CASE("the hold boundary is measured between the raw edges") {
    // Raw press edge at the first pressed sample, raw release edge at the
    // first released one; both land on the poll grid.
    auto gestureFor = [](uint32_t pressMs) {
        Rig r;
        r.hold(false, 200);
        r.hold(true, pressMs);
        r.hold(false, 100);
        return r.seen.size() == 1 ? r.seen[0] : Gesture::none;
    };
    CHECK(gestureFor(btn::kHoldMs) == Gesture::hold);
    CHECK(gestureFor(btn::kHoldMs - kPollMs) == Gesture::press);
    CHECK(gestureFor(btn::kStuckMs - kPollMs) == Gesture::hold);
    CHECK(gestureFor(btn::kStuckMs) == Gesture::stuck);
}

TEST_CASE("contact bounce on both edges is one gesture") {
    Rig r;
    r.hold(false, 200);
    r.bounce(8);
    r.hold(true, 400);
    r.bounce(8);
    r.hold(false, 100);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == Gesture::press);
}

TEST_CASE("a glitch shorter than the debounce window is ignored") {
    Rig r;
    r.hold(false, 200);
    r.hold(true, btn::kDebounceMs - 10);
    r.hold(false, 200);
    CHECK(r.seen.empty());
    CHECK_FALSE(r.b.down());
}

TEST_CASE("held through power-on: its release fires nothing, the next press does") {
    Rig r;
    r.hold(true, 5000);
    r.hold(false, 100);
    CHECK(r.seen.empty());
    r.hold(true, 200);
    r.hold(false, 100);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == Gesture::press);
}

TEST_CASE("a press inside the arming window after boot fires nothing") {
    Rig r;
    r.hold(false, btn::kDebounceMs - 10);
    r.hold(true, 200);
    r.hold(false, 100);
    CHECK(r.seen.empty());
}

TEST_CASE("a press past kStuckMs is stuck, not hold") {
    Rig r;
    r.hold(false, 200);
    r.hold(true, btn::kStuckMs + 100);
    r.hold(false, 100);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == Gesture::stuck);
}

TEST_CASE("the millisecond clock wraps without a phantom gesture") {
    btn::Button b;
    uint32_t t = 0xFFFFFFFFu - 500u;
    for (int i = 0; i < 30; ++i, t += 10) CHECK(b.sample(false, t) == Gesture::none);
    for (int i = 0; i < 30; ++i, t += 10) CHECK(b.sample(true, t) == Gesture::none);
    Gesture g = Gesture::none;
    for (int i = 0; i < 10; ++i, t += 10) {
        const Gesture x = b.sample(false, t);
        if (x != Gesture::none) g = x;
    }
    CHECK(g == Gesture::press);
}

// ---- PAIR: press, double press, demote hold (Valence RFC-114 draft), config
// hold (SPEC 13.4.1) ------------------------------------------------------------

namespace {

using btn::PairGesture;

struct PairRig {
    btn::PairButton b;
    uint32_t t = 1000;
    std::vector<PairGesture> seen;
    std::vector<uint32_t> at;
    bool downAtHold = false;

    void hold(bool level, uint32_t ms) {
        for (uint32_t end = t + ms; t < end; t += kPollMs) {
            const PairGesture g = b.sample(level, t);
            if (g == PairGesture::none) continue;
            if (g == PairGesture::hold || g == PairGesture::config) downAtHold = b.down();
            seen.push_back(g);
            at.push_back(t);
        }
    }
    void tap(uint32_t downMs = 100) { hold(true, downMs); }
};

PairRig armed() {
    PairRig r;
    r.hold(false, 200);
    return r;
}

}  // namespace

TEST_CASE("PAIR: one press is press, after the double-press window and not before") {
    auto r = armed();
    r.tap();
    r.hold(false, btn::kDoublePressMs - 100);
    CHECK(r.seen.empty());
    r.hold(false, 200);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == PairGesture::press);
}

TEST_CASE("PAIR: two presses inside the window are one double press, never a press") {
    auto r = armed();
    r.tap();
    r.hold(false, 300);
    r.tap();
    r.hold(false, btn::kDoublePressMs + 100);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == PairGesture::double_press);
}

TEST_CASE("PAIR: a third press makes it no gesture, and upgrades nothing") {
    auto r = armed();
    for (int i = 0; i < 3; ++i) {
        r.tap();
        r.hold(false, 300);
    }
    r.hold(false, btn::kDoublePressMs);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == PairGesture::extra);
}

TEST_CASE("PAIR: a second press just outside the window is two single presses") {
    auto r = armed();
    r.tap();
    r.hold(false, btn::kDoublePressMs + 50);
    r.tap();
    r.hold(false, btn::kDoublePressMs + 100);
    REQUIRE(r.seen.size() == 2);
    CHECK(r.seen[0] == PairGesture::press);
    CHECK(r.seen[1] == PairGesture::press);
}

TEST_CASE("PAIR: a 5 s hold fires on its release, nothing while held") {
    auto r = armed();
    r.hold(true, btn::kDemoteHoldMs + 100);
    CHECK(r.seen.empty());
    r.hold(false, btn::kDoublePressMs + 100);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == PairGesture::hold);
    CHECK_FALSE(r.downAtHold);
}

TEST_CASE("PAIR: a 10 s hold is the config gesture, fired while held; no demote, and its release fires nothing") {
    auto r = armed();
    r.hold(true, btn::kConfigHoldMs - 100);
    CHECK(r.seen.empty());
    r.hold(true, 200);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == PairGesture::config);
    CHECK(r.downAtHold);
    r.hold(true, 2000);
    r.hold(false, btn::kDoublePressMs + 100);
    CHECK(r.seen.size() == 1);
}

TEST_CASE("PAIR: a 9.9 s hold is only the demote") {
    auto r = armed();
    r.hold(true, btn::kConfigHoldMs - 100);
    r.hold(false, btn::kDoublePressMs + 100);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == PairGesture::hold);
}

TEST_CASE("PAIR: a 4.9 s press is only a single press") {
    auto r = armed();
    r.hold(true, btn::kDemoteHoldMs - 100);
    r.hold(false, btn::kDoublePressMs + 100);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == PairGesture::press);
}

TEST_CASE("PAIR: a press then a hold is only the hold") {
    auto r = armed();
    r.tap();
    r.hold(false, 300);
    r.hold(true, btn::kDemoteHoldMs + 100);
    r.hold(false, btn::kDoublePressMs + 100);
    REQUIRE(r.seen.size() == 1);
    CHECK(r.seen[0] == PairGesture::hold);
}

TEST_CASE("PAIR: a hold then a quick press is the hold, then a press of its own") {
    auto r = armed();
    r.hold(true, btn::kDemoteHoldMs + 100);
    r.hold(false, 200);
    r.tap();
    r.hold(false, btn::kDoublePressMs + 100);
    REQUIRE(r.seen.size() == 2);
    CHECK(r.seen[0] == PairGesture::hold);
    CHECK(r.seen[1] == PairGesture::press);
}

TEST_CASE("PAIR: held through power-on fires no hold") {
    PairRig r;
    r.hold(true, btn::kDemoteHoldMs + 1000);
    r.hold(false, btn::kDoublePressMs + 100);
    CHECK(r.seen.empty());
}

TEST_CASE("PAIR: a jammed button fires its config hold once, then stuck on release") {
    auto r = armed();
    r.hold(true, btn::kStuckMs + 100);
    r.hold(false, 100);
    REQUIRE(r.seen.size() == 2);
    CHECK(r.seen[0] == PairGesture::config);
    CHECK(r.seen[1] == PairGesture::stuck);
}
