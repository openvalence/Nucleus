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
