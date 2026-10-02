#pragma once

// ButtonGesture -- one debounced push button as press / hold gestures, acted
// on RELEASE: a press shorter than the hold time is `press`, one at least as
// long is `hold`
// Constraints:
// - Hardware-free and allocation-free. The host samples the pad, hands in
//   "pressed now" and its millisecond clock, and acts on what sample()
//   returns. ValenceButtons.cpp is the board's host; suite
//   test_button_gesture drives this file.
// - SINGLE OWNER: no method is safe against a concurrent call.
// - Nothing fires on the way DOWN. A gesture exists only at the debounced
//   release, so a hold never also fires its press.
// - Not armed until the button has been seen released for one debounce
//   window: a button already down at the first sample (held through power-on)
//   is ignored until it lets go, and that release fires nothing.
// - A press past kStuckMs is `stuck` on release, never `hold`: a shorted
//   header or a jammed cap must not read as an operator gesture.
// - Milliseconds in the host's clock; wrap-safe (unsigned differences only).
// See: Hardware flagship/SPEC.md 2026-09-23 two-buttons row, bd val-091.26

#include <cstdint>

namespace valence::button {

// ---- timing: THE KNOBS ------------------------------------------------------

// Raw level must hold this long before the debounced state follows it. Tact
// switch bounce is a few ms; 30 ms is also below what a deliberate tap lasts.
inline constexpr uint32_t kDebounceMs = 30;
// A press at least this long is a hold.
inline constexpr uint32_t kHoldMs = 3000;
// A press at least this long is a fault, not a gesture.
inline constexpr uint32_t kStuckMs = 30000;

static_assert(kDebounceMs < kHoldMs && kHoldMs < kStuckMs, "gesture windows out of order");

enum class Gesture : uint8_t { none, press, hold, stuck };

constexpr const char* gestureName(Gesture g) {
    switch (g) {
        case Gesture::press: return "press";
        case Gesture::hold:  return "hold";
        case Gesture::stuck: return "stuck";
        default:             return "none";
    }
}

class Button {
public:
    // One sample. Returns the gesture completed by this sample's debounced
    // release, else none.
    Gesture sample(bool rawPressed, uint32_t nowMs) {
        if (!_seeded) {
            _seeded = true;
            _raw = rawPressed;
            _rawSinceMs = nowMs;
            _down = rawPressed;
            return Gesture::none;
        }
        if (rawPressed != _raw) {
            _raw = rawPressed;
            _rawSinceMs = nowMs;
            return Gesture::none;
        }
        if (_raw == _down || nowMs - _rawSinceMs < kDebounceMs) {
            if (!_down && !_armed && nowMs - _rawSinceMs >= kDebounceMs) _armed = true;
            return Gesture::none;
        }
        // The debounced state follows the raw one, stamped at the raw edge.
        _down = _raw;
        if (_down) {
            _downAtMs = _rawSinceMs;
            return Gesture::none;
        }
        const bool counted = _armed;
        _armed = true;
        if (!counted) return Gesture::none;
        const uint32_t heldMs = _rawSinceMs - _downAtMs;
        if (heldMs >= kStuckMs) return Gesture::stuck;
        return heldMs >= kHoldMs ? Gesture::hold : Gesture::press;
    }

    // Debounced state, for feedback while held.
    bool down() const { return _down; }
    // True once a debounced press has lasted kHoldMs: releasing now is a hold.
    bool holdReached(uint32_t nowMs) const {
        return _down && _armed && nowMs - _downAtMs >= kHoldMs;
    }

private:
    bool _seeded = false;
    bool _armed = false;
    bool _raw = false;
    bool _down = false;
    uint32_t _rawSinceMs = 0;
    uint32_t _downAtMs = 0;
};

}  // namespace valence::button
