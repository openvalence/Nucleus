#pragma once

// OtaArm -- the firmware update request a PAIR double press approves: one
// request at a time, its window, its one use; the idle gate every update
// passes; and the new image's buy at rest
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
// See: ValenceOta.h, bd val-ktal, val-y3da

#include <cstdint>

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

// Why the machine may not take an update now, or nullptr. Halted means the
// emitter parked and no plan running; motor power off means the switch is
// off or faulted (precharging is on its way to on).
constexpr const char* idleRefusal(bool moving, motorswitch::State power) {
    if (moving) return "motion is running: stop the machine first";
    if (power == motorswitch::State::on || power == motorswitch::State::precharging)
        return "motor power is on: latch the E-stop first";
    return nullptr;
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
