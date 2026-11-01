#pragma once

// OtaArm -- the firmware update request a PAIR double press approves: one
// request at a time, its window, its one use; the idle gate every update
// passes; the board gate every image passes; and the new image's buy at rest
// Constraints:
// - Hardware-free and allocation-free; suite test_ota_arm drives it. The
//   board glue (ValenceOta.cpp) owns the one instance, its lock and its clock.
// - SINGLE OWNER: no method is safe against a concurrent call. The glue holds
//   one spinlock across each call (the httpd task arms, reports and takes;
//   the hub task presses).
// - ONE LIVE REQUEST. arm() refuses while one is pending or approved, so a
//   second client cannot slip its request under the operator's press.
// - press() approves only a PENDING request inside its window, and take()
//   spends an approval once, for the id that armed it. Nothing else approves.
// - Milliseconds in the glue's clock; wrap-safe (signed differences only).
// - THE BOARD IS IN THE IMAGE (bd val-3yls): esp_app_desc's version is
//   "<product>/<git label>" (flagship_p4/CMakeLists.txt), nucleus-p4 on the
//   Flagship, nucleus-p4b on the barnacle. imageFit() reads it from the
//   image's first kImageHeadBytes before anything is erased; an image of
//   another product, or naming none, never reaches esp_ota_begin. Bench and
//   release are one product: a bench image fits a Flagship.
// See: ValenceOta.h, bd val-ktal, val-y3da, val-3yls

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "system/MotorSwitch.h"

namespace valence::ota {

// ---- timing: THE KNOBS ------------------------------------------------------

// From the request to the press: the operator's walk to the machine.
inline constexpr uint32_t kArmWindowMs = 60000;
// From the press to the image's first byte; a client polls about once a second.
inline constexpr uint32_t kApprovedHoldMs = 30000;

// Reported as these words by GET /ota/arm. `expired` also covers an unknown,
// spent or superseded id: the client's next step is the same, arm again.
enum class ArmState : uint8_t { expired, pending, approved };

constexpr const char* armStateName(ArmState s) {
    switch (s) {
        case ArmState::pending:  return "pending";
        case ArmState::approved: return "approved";
        default:                 return "expired";
    }
}

class ArmGate {
public:
    // Opens a request under `id` (nonzero). False while another one is live.
    bool arm(uint32_t id, uint32_t nowMs) {
        if (id == 0 || state(nowMs) != ArmState::expired) return false;
        _id = id;
        _state = ArmState::pending;
        _deadlineMs = nowMs + kArmWindowMs;
        return true;
    }

    // The PAIR double press. True when it approved the pending request: the
    // press is then this request's and binds nothing else.
    bool press(uint32_t nowMs) {
        if (state(nowMs) != ArmState::pending) return false;
        _state = ArmState::approved;
        _deadlineMs = nowMs + kApprovedHoldMs;
        return true;
    }

    ArmState status(uint32_t id, uint32_t nowMs) const {
        return (id != 0 && id == _id) ? state(nowMs) : ArmState::expired;
    }

    // Spends the approval for `id`. True once.
    bool take(uint32_t id, uint32_t nowMs) {
        if (status(id, nowMs) != ArmState::approved) return false;
        _state = ArmState::expired;
        return true;
    }

private:
    ArmState state(uint32_t nowMs) const {
        return int32_t(nowMs - _deadlineMs) < 0 ? _state : ArmState::expired;
    }

    uint32_t _id = 0;
    uint32_t _deadlineMs = 0;
    ArmState _state = ArmState::expired;
};

// Why an update may not start while `moving`, or nullptr: the motion half
// of idleRefusal(), and the whole motion rule of the barnacle's helper update
// (BarnacleLink.h helperUpdateRefusal()).
constexpr const char* motionRefusal(bool moving) {
    return moving ? "motion is running: stop the machine first" : nullptr;
}

// Why the machine may not take an update now, or nullptr. Halted means the
// emitter parked and no plan running; motor power off means the switch is
// off or faulted (precharging is on its way to on).
constexpr const char* idleRefusal(bool moving, motorswitch::State power) {
    if (const char* why = motionRefusal(moving)) return why;
    if (power == motorswitch::State::on || power == motorswitch::State::precharging)
        return "motor power is on: latch the E-stop first";
    return nullptr;
}

// ---- the board gate ---------------------------------------------------------

// The image header (24 B), the first segment's header (8 B) and the
// esp_app_desc_t IDF places first in that segment (256 B).
inline constexpr size_t kImageHeadBytes = 24 + 8 + 256;

// The product an esp_app_desc version names, or empty when it names none (an
// image built before the board went into it, or not Nucleus).
constexpr std::string_view productOf(std::string_view version) {
    constexpr std::string_view kFamily = "nucleus-";
    const size_t slash = version.find('/');
    if (slash == std::string_view::npos || slash <= kFamily.size() || !version.starts_with(kFamily)) return {};
    return version.substr(0, slash);
}

// The board a product is, as a refusal says it.
constexpr std::string_view boardName(std::string_view product) {
    if (product == "nucleus-p4") return "a Flagship";
    if (product == "nucleus-p4b") return "the barnacle";
    return product.empty() ? std::string_view("unnamed") : product;
}

enum class ImageFit : uint8_t {
    fits,         // this machine's product: write it
    notImage,     // too short, or no image header or app description: 400
    noBoard,      // names no product: 409, USB is the way
    otherBoard,   // another product: 409
};

// Reads the image's product from its first `n` bytes and holds it against
// `running`, the running image's own esp_app_desc version. Anything but fits
// writes the plain reason into `why`.
inline ImageFit imageFit(const uint8_t* head, size_t n, std::string_view running, char* why, size_t cap) {
    constexpr uint8_t kImageMagic = 0xE9;
    constexpr uint32_t kAppDescMagic = 0xABCD5432;
    constexpr size_t kAppDesc = 24 + 8;
    constexpr size_t kVersion = kAppDesc + 16;
    constexpr size_t kVersionBytes = 32;
    if (n < kImageHeadBytes) {
        snprintf(why, cap, "body too short to be an app image");
        return ImageFit::notImage;
    }
    const uint32_t descMagic = uint32_t(head[kAppDesc]) | uint32_t(head[kAppDesc + 1]) << 8 |
                               uint32_t(head[kAppDesc + 2]) << 16 | uint32_t(head[kAppDesc + 3]) << 24;
    if (head[0] != kImageMagic || descMagic != kAppDescMagic) {
        snprintf(why, cap, "not an app image");
        return ImageFit::notImage;
    }
    const char* v = reinterpret_cast<const char*>(head + kVersion);
    size_t len = 0;
    while (len < kVersionBytes && v[len] != '\0') ++len;
    const std::string_view image = productOf(std::string_view(v, len));
    if (image.empty()) {
        snprintf(why, cap, "image names no board (an older build): flash it over USB");
        return ImageFit::noBoard;
    }
    const std::string_view mine = productOf(running);
    if (image == mine) return ImageFit::fits;
    const std::string_view a = boardName(image), b = boardName(mine);
    snprintf(why, cap, "image is for %.*s, this machine is %.*s", int(a.size()), a.data(), int(b.size()), b.data());
    return ImageFit::otherBoard;
}

// THE BUY AT REST (bd val-y3da, ValenceOta.h): the image's buy writes otadata,
// a flash write that would stop a moving carriage. A live call makes the buy
// owed; it is taken at the first call at rest, once. No deadline: nothing here
// expires an owed buy. Single owner, as ArmGate.
enum class Buy : uint8_t {
    wait,   // nothing to do now
    now,    // commit the image now, the only time
    held,   // first owed while motion runs: say so once
};

class BuyGate {
public:
    Buy take(bool live, bool atRest) {
        const bool first = live && !_owed;
        if (live) _owed = true;
        if (!_owed || _bought) return Buy::wait;
        if (!atRest) return first ? Buy::held : Buy::wait;
        _bought = true;
        return Buy::now;
    }
    bool held() const { return _owed && !_bought; }

private:
    bool _owed = false;
    bool _bought = false;
};

}  // namespace valence::ota
