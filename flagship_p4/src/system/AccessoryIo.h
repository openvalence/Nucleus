#pragma once

// AccessoryIo -- the accessory header's request rules, hardware-free: the PWM
// duty clamp, the ESTOP that zeroes every request, the IO pads' direction set
// once per boot, and the AUX gesture slots
// Constraints:
// - Hardware-free and allocation-free. ValenceAccessoryIo.cpp is the board's
//   host; suite test_accessory_io drives this file.
// - Requests are ANY task and estop() is any task or ISR: every shared field
//   is a lock-free 32-bit std::atomic, and nothing here blocks or locks. The
//   host's one owner task reads the requests and drives the pads.
// - THE ESTOP WINS EVERY RACE. A request re-reads the ESTOP count after its
//   store and withdraws itself when the count moved, so once estop() returns
//   every output stays off until a request that BEGAN after it.
// - Off is duty 0 for a PWM and RELEASED for an IO pad (input, pull-down),
//   never a driven level: an active-low load reads a driven low as on.
// - An IO pad's direction is set once per boot; an output starts released.
// See: ValenceAccessoryIo.h, ButtonGesture.h, BoardPins.h, bd val-091.30

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "ButtonGesture.h"

namespace valence::accessory {

// ---- the PWM outputs --------------------------------------------------------

enum class Pwm : uint8_t { pump, ext };
inline constexpr uint8_t kPwmCount = 2;

// LEDC duty resolution of both PWMs; the host's timer reads it here.
inline constexpr uint32_t kDutyBits = 10;
inline constexpr uint32_t kDutyMax = (1u << kDutyBits) - 1u;

// Duty 0..1 as LEDC counts. NaN and anything at or below 0 is off; 1 and
// above is full on.
inline uint32_t dutyCounts(float duty) {
    if (!(duty > 0.0f)) return 0;
    if (duty >= 1.0f) return kDutyMax;
    return uint32_t(std::lround(duty * float(kDutyMax)));
}

// ---- the IO pads ------------------------------------------------------------

// IO1..IO5 on J15, as index 0..4.
inline constexpr uint8_t kGpioCount = 5;

// input carries a pull-down, so an unwired pad never floats.
enum class Dir : uint8_t { unset, input, input_pullup, output };
enum class Drive : uint8_t { released, low, high };

constexpr const char* dirName(Dir d) {
    switch (d) {
        case Dir::input:        return "input";
        case Dir::input_pullup: return "input, pull-up";
        case Dir::output:       return "output";
        default:                return "unset";
    }
}

// ---- the request block ------------------------------------------------------

class Requests {
public:
    // Stores the clamped duty. False when `p` is out of range, or when an
    // ESTOP landed during the call and the request was withdrawn.
    bool setDuty(Pwm p, float duty) {
        if (size_t(p) >= kPwmCount) return false;
        std::atomic<uint32_t>& slot = _duty[size_t(p)];
        const uint32_t seq = _estops.load();
        const uint32_t counts = dutyCounts(duty);
        slot.store(counts);
        if (_estops.load() == seq) return true;
        uint32_t mine = counts;
        slot.compare_exchange_strong(mine, 0);
        return false;
    }
    uint32_t duty(Pwm p) const { return size_t(p) < kPwmCount ? _duty[size_t(p)].load() : 0; }

    // Once per pad per boot, never back to unset. False: out of range, unset
    // asked, or the direction is already set.
    bool configure(uint8_t i, Dir d) {
        if (i >= kGpioCount || d == Dir::unset) return false;
        uint32_t expected = uint32_t(Dir::unset);
        return _dir[i].compare_exchange_strong(expected, uint32_t(d));
    }
    Dir dir(uint8_t i) const { return i < kGpioCount ? Dir(_dir[i].load()) : Dir::unset; }

    // Drives an OUTPUT pad. False: out of range, not an output, or an ESTOP
    // landed during the call (withdrawn: the pad stays released).
    bool set(uint8_t i, bool high) {
        if (dir(i) != Dir::output) return false;
        const uint32_t seq = _estops.load();
        const uint32_t want = uint32_t(high ? Drive::high : Drive::low);
        _drive[i].store(want);
        if (_estops.load() == seq) return true;
        uint32_t mine = want;
        _drive[i].compare_exchange_strong(mine, uint32_t(Drive::released));
        return false;
    }
    Drive drive(uint8_t i) const { return i < kGpioCount ? Drive(_drive[i].load()) : Drive::released; }

    // Every duty to 0 and every IO pad released; directions are kept. The
    // count moves FIRST: that is what a racing request re-reads.
    void estop() {
        _estops.fetch_add(1);
        for (std::atomic<uint32_t>& d : _duty) d.store(0);
        for (std::atomic<uint32_t>& d : _drive) d.store(uint32_t(Drive::released));
    }
    uint32_t estops() const { return _estops.load(); }

private:
    std::array<std::atomic<uint32_t>, kPwmCount> _duty{};
    std::array<std::atomic<uint32_t>, kGpioCount> _dir{};
    std::array<std::atomic<uint32_t>, kGpioCount> _drive{};
    std::atomic<uint32_t> _estops{0};
};

// ---- the AUX buttons --------------------------------------------------------

enum class Aux : uint8_t { aux1, aux2 };
inline constexpr uint8_t kAuxCount = 2;

// One AUX pad's newest press or hold not yet taken; a newer one overwrites
// it. A stuck press is never parked: it is a fault, not a gesture.
class GestureSlot {
public:
    // True when `g` was parked.
    bool park(button::Gesture g) {
        if (g != button::Gesture::press && g != button::Gesture::hold) return false;
        _g.store(uint32_t(g));
        return true;
    }
    button::Gesture take() { return button::Gesture(_g.exchange(uint32_t(button::Gesture::none))); }

private:
    std::atomic<uint32_t> _g{uint32_t(button::Gesture::none)};
};

}  // namespace valence::accessory
