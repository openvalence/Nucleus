#pragma once

// MotorSwitch -- the motor switch (U2, TPS48111) as a state machine: off,
// precharging, on, faulted; the enable sequence and the fault latch
// Constraints:
// - Hardware-free and allocation-free. A host reads the pins into Readings,
//   steps the machine, and drives MOTOR_EN and PRECHARGE_EN from outputs().
//   ValenceMotorSwitch.cpp is the board's host, sim/valencesim's
//   SimMotorSwitch.cpp the twin's; suite test_motor_switch drives this file.
// - SINGLE OWNER. No method is safe against a concurrent call; the host
//   serializes them (the board under one spinlock, ValenceMotorSwitch.cpp).
// - MOTOR_EN goes high only in `on`, and only a completed pre-charge window
//   with the fault line clear, the EN node up and the inrush under its
//   ceiling reaches `on`.
// - A FAULT LATCHES. faulted holds both lines low until requestEnable() finds
//   the fault line clear and the EN node up; cut() never leaves faulted.
// - State and Fault ordinals are wire values: hub-status 0x0006
//   `motor_switch` and `motor_fault` label them in ValenceCatalog.h.
//   Append-only, and the labels move with them.
// - Microseconds in the host's clock; amperes and volts.
// See: ../Hardware/flagship/design-considerations.md section 4,
// docs/board-map.md (motion and motor power), bd val-091.24

#include <cmath>
#include <cstdint>
#include <limits>

namespace valence::motorswitch {

// ---- the sequence's numbers -----------------------------------------------------

// The pre-charge window, derived (design-considerations.md section 4): the
// drive presents ~150 uF across its input (hw-x65, a DMM reading on an
// unpowered drive, ballpark only), charged through R412, 100 R. tau = 15 ms,
// ~75 ms to 99 %. The window doubles C for the ballpark reading and waits five
// time constants: 5 x 100 R x 300 uF = 150 ms, which leaves a 150 uF bank at
// 99.995 % and a 300 uF one at 99.3 %.
// TODO(val-091.56): scope MOTOR_V+ across the window on a real drive.
inline constexpr float kDriveInputF       = 150e-6f;
inline constexpr float kPrechargeOhms     = 100.0f;
inline constexpr float kCapacitanceMargin = 2.0f;
inline constexpr float kTimeConstants     = 5.0f;
inline constexpr uint32_t kPrechargeUs = uint32_t(
    kTimeConstants * kPrechargeOhms * kDriveInputF * kCapacitanceMargin * 1e6f + 0.5f);
static_assert(kPrechargeUs == 150000, "pre-charge window drifted from its derivation");

// MSW_IMON: R406 8.25k puts 3.0 V on G19 at 20 A (schematic note on U2).
inline constexpr float kImonVoltsPerAmp = 0.15f;

// Current ceiling at the END of the window. A healthy bank has decayed to
// ~2.4 mA there (0.36 A x e^-5); a short on MOTOR_V+ holds the whole
// pre-charge current, 0.24 A at a 24 V bus and 0.36 A at 36 V.
// TODO(val-091.56): unmeasured, including whether IMON sees the pre-charge
// path at all (R401's place in the topology).
inline constexpr float kInrushCeilingA = 0.15f;

// MOTOR_V+ at the end of the window, when the power monitor answered: the
// controller's own turn-on bus voltage (UVLO on, 19.6 V, R407/R408 470k/30.1k).
// Under it the bank did not charge, or the bus cannot run the switch anyway.
// TODO(val-091.56): judge against +BUS from the board monitor, not a floor.
inline constexpr float kPrechargeMinV = 19.6f;

// The EN/UVLO wired-OR node on G23 (every hardware kill pulls it). 19.6 V on
// +BUS through 470k/30.1k puts the controller's turn-on point at 1.18 V; the
// node reads ~1.8 V running and ~0 V tripped (Hardware SPEC.md 2026-09-23
// bus-OVP row). Under this the switch is held off by hardware.
// TODO(val-091.56): measure running and tripped at 24 V and 36 V.
inline constexpr float kEnNodeMinV = 1.1f;

// ---- vocabulary ---------------------------------------------------------------

enum class State : uint8_t { off, precharging, on, faulted };

// Why the switch last latched faulted. Wire ordinals (file header).
enum class Fault : uint8_t {
    none,         // never faulted since boot
    fault_line,   // MSW_FLT_N low: the controller latched over-current or over-temperature
    en_node,      // EN/UVLO node fell: E-stop, bus OVP, INA ALERT or the board monitor
    inrush,       // MSW_IMON over its ceiling at the end of the window, or unread
    precharge,    // MOTOR_V+ did not reach kPrechargeMinV in the window
};

// Why an enable request was refused. none = accepted, or already under way.
enum class Refusal : uint8_t {
    none,
    self_check,   // the boot self-check did not pass: motor power stays off
    fault_line,   // MSW_FLT_N still low: the cause is not resolved
    en_node,      // a hardware kill still holds the EN node down
    host_down,    // the switch's host never started: nothing can run the window
    source,       // the PD contract cannot carry the input ceilings (PdSource.h)
};

constexpr const char* stateName(State s) {
    switch (s) {
        case State::off:         return "off";
        case State::precharging: return "precharging";
        case State::on:          return "on";
        case State::faulted:     return "faulted";
    }
    return "?";
}

constexpr const char* faultName(Fault f) {
    switch (f) {
        case Fault::none:       return "none";
        case Fault::fault_line: return "MSW_FLT_N low";
        case Fault::en_node:    return "EN node low";
        case Fault::inrush:     return "inrush over ceiling";
        case Fault::precharge:  return "MOTOR_V+ did not rise";
    }
    return "?";
}

constexpr const char* refusalName(Refusal r) {
    switch (r) {
        case Refusal::none:       return "accepted";
        case Refusal::self_check: return "the boot self-check holds motor power off";
        case Refusal::fault_line: return "MSW_FLT_N still low";
        case Refusal::en_node:    return "a hardware kill holds the EN node down";
        case Refusal::host_down:  return "the motor switch host is not running";
        case Refusal::source:     return "the PD contract cannot carry the ceilings";
    }
    return "?";
}

// NaN means "not read". An unread EN node or IMON never passes; an unread
// MOTOR_V+ (no power monitor fitted) skips its check.
struct Readings {
    bool  fault_line = false;   // MSW_FLT_N reads low
    float en_node_v  = std::numeric_limits<float>::quiet_NaN();
    float imon_a     = std::numeric_limits<float>::quiet_NaN();
    float motor_v    = std::numeric_limits<float>::quiet_NaN();
};

struct Outputs {
    bool motor_en     = false;
    bool precharge_en = false;
};

// Written so a NaN reads down: !(v >= min), never (v < min).
constexpr bool enNodeUp(float v) { return v >= kEnNodeMinV; }

// What a host can answer without the EN node, which only a fresh read after
// the INA ALERT re-arm can judge (ValenceMotorSwitch.cpp).
constexpr Refusal precheck(bool allowed, bool faultLine) {
    if (!allowed) return Refusal::self_check;
    if (faultLine) return Refusal::fault_line;
    return Refusal::none;
}

constexpr Refusal refusal(bool allowed, const Readings& r) {
    const Refusal p = precheck(allowed, r.fault_line);
    if (p != Refusal::none) return p;
    return enNodeUp(r.en_node_v) ? Refusal::none : Refusal::en_node;
}

// ---- the machine ----------------------------------------------------------------

class Switch {
public:
    State state() const { return _state; }
    Fault lastFault() const { return _fault; }
    uint32_t faults() const { return _faults; }

    Outputs outputs() const {
        switch (_state) {
            case State::precharging: return {false, true};
            case State::on:          return {true, false};
            default:                 return {};
        }
    }

    // The explicit request. From off or faulted it opens the window at now_us
    // unless refused; precharging and on answer none and change nothing.
    Refusal requestEnable(bool allowed, const Readings& r, uint64_t now_us) {
        if (_state == State::precharging || _state == State::on) return Refusal::none;
        const Refusal why = refusal(allowed, r);
        if (why != Refusal::none) return why;
        _state = State::precharging;
        _since_us = now_us;
        return Refusal::none;
    }

    // True from the end of the window until step() judges it: the one moment
    // a host needs MOTOR_V+, which may cost it an I2C read.
    bool windowDue(uint64_t now_us) const {
        return _state == State::precharging && now_us - _since_us >= kPrechargeUs;
    }

    // The power cut. precharging and on drop to off; faulted stays latched.
    void cut() {
        if (_state == State::precharging || _state == State::on) _state = State::off;
    }

    // Advances on fresh readings. True when the state changed.
    bool step(const Readings& r, uint64_t now_us) {
        switch (_state) {
            case State::off:
            case State::faulted:
                return false;
            case State::precharging:
                if (r.fault_line) return trip(Fault::fault_line);
                if (!enNodeUp(r.en_node_v)) return trip(Fault::en_node);
                if (now_us - _since_us < kPrechargeUs) return false;
                if (!(r.imon_a <= kInrushCeilingA)) return trip(Fault::inrush);
                if (std::isfinite(r.motor_v) && r.motor_v < kPrechargeMinV) return trip(Fault::precharge);
                _state = State::on;
                return true;
            case State::on:
                if (r.fault_line) return trip(Fault::fault_line);
                if (!enNodeUp(r.en_node_v)) return trip(Fault::en_node);
                return false;
        }
        return false;
    }

private:
    bool trip(Fault f) {
        _state = State::faulted;
        _fault = f;
        ++_faults;
        return true;
    }

    State    _state    = State::off;
    Fault    _fault    = Fault::none;
    uint32_t _faults   = 0;
    uint64_t _since_us = 0;   // when the window opened
};

}  // namespace valence::motorswitch
