#pragma once

// ButtonGesture -- one debounced push button as press / hold gestures, acted
// on RELEASE: a press shorter than the hold time is `press`, one at least as
// long is `hold`; and PAIR's sequence on top of it (PairButton below)
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

// PAIR (Valence RFC-114 draft; both move to the registry's limits at landing).
// A next press beginning within this long of a release continues the sequence.
inline constexpr uint32_t kDoublePressMs = 600;
// A press at least this long, released before kConfigHoldMs, demotes every
// client.
inline constexpr uint32_t kDemoteHoldMs = 5000;
// A press this long is the config-mode gesture (SPEC 13.4.1, device-defined),
// fired while still held.
inline constexpr uint32_t kConfigHoldMs = 10000;

static_assert(kDebounceMs < kDoublePressMs && kDemoteHoldMs < kConfigHoldMs && kConfigHoldMs < kStuckMs,
              "PAIR windows out of order");

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
    bool holdReached(uint32_t nowMs) const { return heldMs(nowMs) >= kHoldMs; }
    // How long a counted press has been down, from its raw edge; 0 when up.
    uint32_t heldMs(uint32_t nowMs) const { return _down && _armed ? nowMs - _downAtMs : 0; }

private:
    bool _seeded = false;
    bool _armed = false;
    bool _raw = false;
    bool _down = false;
    uint32_t _rawSinceMs = 0;
    uint32_t _downAtMs = 0;
};

// ---- PAIR: press, double press, demote hold, config hold ----------------------
// Constraints:
// - A sequence of presses, each beginning within kDoublePressMs of the last
//   release (measured between debounced edges), resolves kDoublePressMs after
//   its last release: one press is `press`, two `double_press`, three or more
//   `extra`, which binds nothing. A single press waits that long to know.
// - `hold` fires ON RELEASE after kDemoteHoldMs and before kConfigHoldMs;
//   `config` fires WHILE HELD at kConfigHoldMs, and its release fires nothing
//   (`stuck` past kStuckMs, for the log). Either cancels the sequence it ends:
//   no press of it, before or after, also counts. Shorter is a press.
// - Inherits Button's arming: held through power-on fires nothing.

enum class PairGesture : uint8_t { none, press, double_press, hold, extra, stuck, config };

constexpr const char* pairGestureName(PairGesture g) {
    switch (g) {
        case PairGesture::press:        return "press";
        case PairGesture::double_press: return "double press";
        case PairGesture::hold:         return "hold";
        case PairGesture::extra:        return "3+ presses";
        case PairGesture::stuck:        return "stuck";
        case PairGesture::config:       return "config hold";
        default:                        return "none";
    }
}

class PairButton {
public:
    PairGesture sample(bool rawPressed, uint32_t nowMs) {
        const Gesture g = _core.sample(rawPressed, nowMs);
        if (_core.down()) {
            _downMs = _core.heldMs(nowMs);
            if (_held || _downMs < kConfigHoldMs) return PairGesture::none;
            _held = true;
            _presses = 0;
            return PairGesture::config;
        }
        if (g != Gesture::none) {   // a counted release
            const uint32_t downMs = _downMs;
            _downMs = 0;
            if (_held) {
                _held = false;
                return g == Gesture::stuck ? PairGesture::stuck : PairGesture::none;
            }
            if (downMs >= kDemoteHoldMs) {
                _presses = 0;
                return PairGesture::hold;
            }
            if (_presses < 3) ++_presses;
            _upAtMs = nowMs;
            return PairGesture::none;
        }
        if (_presses == 0 || nowMs - _upAtMs < kDoublePressMs) return PairGesture::none;
        const uint8_t n = _presses;
        _presses = 0;
        return n == 1 ? PairGesture::press : n == 2 ? PairGesture::double_press : PairGesture::extra;
    }

    bool down() const { return _core.down(); }

private:
    Button _core;
    uint32_t _upAtMs = 0;
    uint32_t _downMs = 0;   // the press under way, as long as the last sample saw it
    uint8_t _presses = 0;
    bool _held = false;
};

}  // namespace valence::button
