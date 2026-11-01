// test_ota_arm -- native doctest suite for the update request a PAIR double
// press approves, the idle gate and the board gate
// Constraints:
// - Hardware-free: OtaArm.h alone, on a synthetic millisecond clock. The
//   routes, the lock and the flash writes are ValenceOta.cpp and are bench
//   work; which press the request owns is test_valence_device's VD-PAIR-11/12.
// See: flagship_p4/src/system/OtaArm.h, bd val-ktal, val-3yls

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cstdint>
#include <cstring>
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
    // The motion half is motionRefusal(), the same words.
    CHECK(ota::motionRefusal(false) == nullptr);
    CHECK(std::string_view(ota::idleRefusal(true, State::off)) == ota::motionRefusal(true));
}

TEST_CASE("OA-10: the reported words") {
    CHECK(std::string_view(ota::armStateName(ArmState::pending)) == "pending");
    CHECK(std::string_view(ota::armStateName(ArmState::approved)) == "approved");
    CHECK(std::string_view(ota::armStateName(ArmState::expired)) == "expired");
}

TEST_CASE("OA-11: a live image buys at its first rest, once; motion only defers it, never expires it") {
    ota::BuyGate g;
    CHECK(g.take(false, true) == ota::Buy::wait);   // not live yet: nothing owed
    CHECK_FALSE(g.held());
    CHECK(g.take(true, false) == ota::Buy::held);   // live mid-stroke: said once
    CHECK(g.take(true, false) == ota::Buy::wait);
    CHECK(g.held());
    for (int i = 0; i < 100000; ++i) REQUIRE(g.take(i % 2 == 0, false) == ota::Buy::wait);   // hours of motion
    CHECK(g.held());
    CHECK(g.take(false, true) == ota::Buy::now);    // the first rest, from any caller
    CHECK_FALSE(g.held());
    CHECK(g.take(true, true) == ota::Buy::wait);
    CHECK(g.take(false, true) == ota::Buy::wait);
}

TEST_CASE("OA-12: a live image already at rest buys at once") {
    ota::BuyGate g;
    CHECK(g.take(true, true) == ota::Buy::now);
    CHECK(g.take(true, true) == ota::Buy::wait);
}

// ---- the board gate ---------------------------------------------------------

namespace {

// An image head as IDF lays it out: the header's magic, the first segment's
// esp_app_desc_t at 32 and its version at 48.
std::array<uint8_t, 4096> imageHead(const char* version, uint8_t magic = 0xE9) {
    std::array<uint8_t, 4096> b{};
    b[0] = magic;
    b[12] = 0x12;   // chip id: ESP32-P4
    const uint32_t descMagic = 0xABCD5432;
    std::memcpy(b.data() + 32, &descMagic, 4);
    std::strncpy(reinterpret_cast<char*>(b.data() + 48), version, 32);
    std::strncpy(reinterpret_cast<char*>(b.data() + 80), "flagship_p4", 32);
    return b;
}

constexpr std::string_view kFlagship = "nucleus-p4/ccc6f2f";
constexpr std::string_view kBarnacle = "nucleus-p4b/ccc6f2f-dirty";

struct FitCase {
    const char* name;
    const char* image;           // the image's esp_app_desc version
    std::string_view running;    // the running image's
    size_t n;                    // bytes of the head read
    ota::ImageFit fit;
    const char* why;             // nullptr when it fits
};

}  // namespace

TEST_CASE("OA-13: the board gate, case by case") {
    using F = ota::ImageFit;
    const size_t head = ota::kImageHeadBytes;
    const FitCase cases[] = {
        {"same board, Flagship", "nucleus-p4/abc1234", kFlagship, head, F::fits, nullptr},
        {"same board, barnacle", "nucleus-p4b/abc1234", kBarnacle, head, F::fits, nullptr},
        {"bench build on a Flagship", "nucleus-p4/57716f1-dirty", kFlagship, 4096, F::fits, nullptr},
        {"barnacle image on a Flagship", "nucleus-p4b/abc1234", kFlagship, head, F::otherBoard,
         "image is for the barnacle, this machine is a Flagship"},
        {"Flagship image on the barnacle", "nucleus-p4/abc1234", kBarnacle, head, F::otherBoard,
         "image is for a Flagship, this machine is the barnacle"},
        {"an unknown product names itself", "nucleus-p4c/abc1234", kFlagship, head, F::otherBoard,
         "image is for nucleus-p4c, this machine is a Flagship"},
        {"no board: the git label alone", "ccc6f2f-dirty", kFlagship, head, F::noBoard,
         "image names no board (an older build): flash it over USB"},
        {"no board: a git describe", "v0.1-12-gabc1234", kBarnacle, head, F::noBoard,
         "image names no board (an older build): flash it over USB"},
        {"no board: the family alone", "nucleus-/abc", kFlagship, head, F::noBoard,
         "image names no board (an older build): flash it over USB"},
        {"no board: an empty version", "", kFlagship, head, F::noBoard,
         "image names no board (an older build): flash it over USB"},
        {"short first chunk", "nucleus-p4/abc1234", kFlagship, head - 1, F::notImage,
         "body too short to be an app image"},
        {"nothing read", "nucleus-p4/abc1234", kFlagship, 0, F::notImage, "body too short to be an app image"},
    };
    for (const FitCase& c : cases) {
        CAPTURE(c.name);
        const auto b = imageHead(c.image);
        char why[96] = "untouched";
        CHECK(ota::imageFit(b.data(), c.n, c.running, why, sizeof(why)) == c.fit);
        CHECK(std::string_view(why) == (c.why ? c.why : "untouched"));
    }
}

TEST_CASE("OA-14: garbage never fits") {
    char why[96];
    auto b = imageHead("nucleus-p4/abc1234", 0x7F);   // not the image magic
    CHECK(ota::imageFit(b.data(), b.size(), kFlagship, why, sizeof(why)) == ota::ImageFit::notImage);
    CHECK(std::string_view(why) == "not an app image");
    b = imageHead("nucleus-p4/abc1234");
    b[32] ^= 0xFF;                                     // no app description
    CHECK(ota::imageFit(b.data(), b.size(), kFlagship, why, sizeof(why)) == ota::ImageFit::notImage);
    std::array<uint8_t, 4096> noise{};
    uint32_t x = 0x12345678;
    for (uint8_t& v : noise) v = uint8_t((x = x * 1664525u + 1013904223u) >> 24);
    noise[0] = 0xE9;                                   // the magic alone is not an image
    CHECK(ota::imageFit(noise.data(), noise.size(), kFlagship, why, sizeof(why)) == ota::ImageFit::notImage);
}

TEST_CASE("OA-15: a version that fills its field, unterminated, still reads") {
    // IDF cuts PROJECT_VER to 31 characters; an image whose 32 bytes carry no
    // NUL must neither read past the field nor lose its product.
    auto b = imageHead("");
    std::memcpy(b.data() + 48, "nucleus-p4b/ccc6f2f-dirty-kinet", 31);
    b[48 + 31] = 'x';
    b[80] = 'f';
    char why[96];
    CHECK(ota::imageFit(b.data(), b.size(), kBarnacle, why, sizeof(why)) == ota::ImageFit::fits);
    CHECK(ota::productOf("nucleus-p4b/ccc6f2f-dirty-kinetic-unpinned") == "nucleus-p4b");
}

TEST_CASE("OA-16: a refusal never overruns its buffer") {
    const auto b = imageHead("nucleus-p4b/abc1234");
    char why[16];
    CHECK(ota::imageFit(b.data(), b.size(), kFlagship, why, sizeof(why)) == ota::ImageFit::otherBoard);
    CHECK(std::strlen(why) == sizeof(why) - 1);
}
