#pragma once

// EstopInput -- the external E-stop's NC and NO contacts as one debounced
// reading: released, pressed, unplugged or a wiring fault
// Constraints:
// - Hardware-free and allocation-free. The host samples both pads, hands in
//   "reads HIGH" for each with its millisecond clock, and publishes what
//   reading() returns. ValenceEstopInput.cpp is the board's host; suite
//   test_estop_input drives this file.
// - SINGLE OWNER: no method is safe against a concurrent call. Reading is the
//   cross-task form, packed into ONE byte.
// - A READING, NEVER A COMMAND. The stop is hardware (Q901 on the motor
//   switch EN node, Q904 for NO closed); nothing downstream may restore power
//   or resume motion because the contacts read released again (SPEC 11.2).
// - Every disturbance settles: a decoded state held kDebounceMs is accepted,
//   and contacts that hold none for kChatterMs read wiring_fault.
// - Milliseconds in the host's clock; wrap-safe (unsigned differences only).
// See: Hardware flagship/SPEC.md 2026-09-23 external wired E-stop row and its
// 2026-09-28, 2026-10-01 and 2026-10-02 amendments, bd val-091.23

#include <cstdint>

namespace valence::estop {

// ---- timing: THE KNOBS ------------------------------------------------------

// A decoded state must hold this long to be accepted. Contact bounce and a
// TRS plug's insertion wipe last a few ms; the jack's own filter (2.2k with
// 100 nF) is 0.22 ms.
// TODO(val-091.69): scope a slammed press, NC opening to NO closing.
inline constexpr uint32_t kDebounceMs = 30;
// Contacts that hold no state this long read wiring_fault: a flapping plug
// must stop the machine, never read as its last good state.
inline constexpr uint32_t kChatterMs = 200;

static_assert(kDebounceMs < kChatterMs, "a chattering reading must not preempt a clean one");

// ---- the contacts -----------------------------------------------------------

// HIGH means open: R901/R902 2.2k pull-ups, each contact to COM (GND).
// A 2-wire NC-only button reads unplugged when pressed: still a stop.
enum class Contacts : uint8_t { released, pressed, unplugged, wiring_fault };

constexpr Contacts decode(bool ncHigh, bool noHigh) {
    if (!ncHigh && noHigh) return Contacts::released;
    if (ncHigh && !noHigh) return Contacts::pressed;
    if (ncHigh && noHigh) return Contacts::unplugged;
    return Contacts::wiring_fault;
}

constexpr const char* contactsName(Contacts c) {
    switch (c) {
        case Contacts::released:     return "released";
        case Contacts::pressed:      return "pressed";
        case Contacts::unplugged:    return "unplugged (NC and NO open)";
        case Contacts::wiring_fault: return "wiring fault (NC and NO closed, or chattering)";
    }
    return "?";
}

// ---- the reading ------------------------------------------------------------

struct Reading {
    Contacts state  = Contacts::released;
    bool     known  = false;   // a state has settled since boot
    bool     masked = false;   // the bench profile read unplugged as released

    // Any settled state but released stops the machine.
    constexpr bool stops() const { return known && state != Contacts::released; }

    constexpr uint8_t pack() const {
        return uint8_t(uint8_t(state) | (known ? 0x04u : 0u) | (masked ? 0x08u : 0u));
    }
    static constexpr Reading unpack(uint8_t b) {
        Reading r;
        r.state = Contacts(b & 0x03u);
        r.known = (b & 0x04u) != 0;
        r.masked = (b & 0x08u) != 0;
        return r;
    }

    bool operator==(const Reading&) const = default;
};

// ---- the reader -------------------------------------------------------------

class Reader {
public:
    // benchMask is the NUCLEUS_BENCH_NO_MOTOR policy: a devkit wires no
    // e-stop, so unplugged reads released, flagged masked. Pressed and a
    // wiring fault still stop. The board host passes kBenchNoMotor.
    explicit constexpr Reader(bool benchMask) : _benchMask(benchMask) {}

    // One sample. True when the reading changed.
    bool sample(bool ncHigh, bool noHigh, uint32_t nowMs) {
        const Contacts raw = decode(ncHigh, noHigh);
        if (!_seeded) {
            _seeded = true;
            _raw = raw;
            _rawSinceMs = nowMs;
            _episodeSinceMs = nowMs;
            return false;
        }
        if (raw != _raw) {
            _raw = raw;
            _rawSinceMs = nowMs;
        }
        if (!_episode) {
            if (_raw == _settled) return false;
            _episode = true;
            _episodeSinceMs = nowMs;
        }
        // Whatever holds kDebounceMs ends the episode, the state it left included.
        if (nowMs - _rawSinceMs >= kDebounceMs) return settle(_raw);
        if (nowMs - _episodeSinceMs >= kChatterMs) return settle(Contacts::wiring_fault);
        return false;
    }

    Reading reading() const {
        Reading r;
        r.known = _known;
        r.masked = _benchMask && _known && _settled == Contacts::unplugged;
        r.state = r.masked ? Contacts::released : _settled;
        return r;
    }

private:
    bool settle(Contacts s) {
        const bool changed = !_known || s != _settled;
        _settled = s;
        _known = true;
        _episode = false;
        return changed;
    }

    bool     _benchMask;
    bool     _seeded = false;
    bool     _known = false;
    // An episode runs from the first sample that disagrees with the settled
    // state (from the seed, before the first settle) until a state settles.
    bool     _episode = true;
    Contacts _settled = Contacts::released;
    Contacts _raw = Contacts::released;
    uint32_t _rawSinceMs = 0;
    uint32_t _episodeSinceMs = 0;
};

}  // namespace valence::estop
