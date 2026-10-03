#pragma once

// AimDrive -- the YZ AIM integrated servo over RS485 as this firmware uses
// it: the register map by function, the post-enable identity probe, the
// runtime poll, and the WR alarm and RDY lines with the alarm's latch rule
// Constraints:
// - Hardware-free and allocation-free. ValenceDriveLink.cpp is the board's
//   host, and its drive-link task is the only owner of every object here;
//   suite test_drive_link drives this file through a fake port.
// - READS ONLY. Nothing here writes a register: 0x00 = 1 deafens the
//   quadrature input (manual p. 16), and writes to 0x0C/0x0D, 0x16/0x17 or
//   0x19 move the motor. Motion is the LP core's alone (architecture.md
//   section 1).
// - The map is the YZ-AIM manual v2.55 (cover v2.53) pp. 11-13, the alarm
//   codes and the WR and RDY outputs p. 5, quadrature follow p. 7. The
//   60AIM40 is not in that manual's model tables: the series is assumed to
//   share the map (Hardware design-considerations.md section 12) until the
//   bench says so (val-091.69).
// - The drive's logic rides the motor bus: nothing answers, and WR means
//   nothing, until the motor switch has been on for kSettleMs (bd val-091.29).
// - Milliseconds in the host's clock, wrap-safe.
// See: ModbusRtu.h, ValenceDriveLink.h, docs/board-map.md (drive comms and
// status), bd val-091.29

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "ModbusRtu.h"

namespace valence::aim {

// ---- the register map, by function ------------------------------------------
// Units and encodings ride each line; manual pp. 11-13.

namespace reg {
inline constexpr uint16_t modbus_enable     = 0x00;  // 1: Modbus owns the drive, pulse input ignored
inline constexpr uint16_t output_enable     = 0x01;  // 0: drive output off
inline constexpr uint16_t speed_limit       = 0x02;  // r/min, the position-mode ceiling
inline constexpr uint16_t acceleration      = 0x03;  // (r/min)/s; 60000 and up: no internal ramp
inline constexpr uint16_t field_weakening   = 0x04;  // internal
inline constexpr uint16_t speed_kp          = 0x05;  // 0-10000 is 0.0-10.0; ones digit: pulse edge
inline constexpr uint16_t speed_ti          = 0x06;  // ms
inline constexpr uint16_t position_kp       = 0x07;  // 60-30000; ones digit odd: WR normally closed
inline constexpr uint16_t speed_feedforward = 0x08;  // 327 is 1 V/krpm
inline constexpr uint16_t dir_polarity      = 0x09;
inline constexpr uint16_t gear_numerator    = 0x0A;  // 0: Modbus control at power-on (p. 10)
inline constexpr uint16_t gear_denominator  = 0x0B;
inline constexpr uint16_t to_go_low         = 0x0C;  // distance to go, signed 32-bit with 0x0D
inline constexpr uint16_t to_go_high        = 0x0D;
inline constexpr uint16_t alarm_code        = 0x0E;  // see the codes below
inline constexpr uint16_t current           = 0x0F;  // A x 2000
inline constexpr uint16_t speed             = 0x10;  // signed, r/min x 10
inline constexpr uint16_t voltage           = 0x11;  // V x 327
inline constexpr uint16_t temperature       = 0x12;  // deg C, 0-100
inline constexpr uint16_t output_pwm        = 0x13;  // signed, 32767 is 100 %
inline constexpr uint16_t save_flag         = 0x14;
inline constexpr uint16_t device_address    = 0x15;  // the drive's own Modbus address
inline constexpr uint16_t position_low      = 0x16;  // encoder, signed 32-bit with 0x17, 32768 a turn
inline constexpr uint16_t position_high     = 0x17;
inline constexpr uint16_t standstill        = 0x18;  // max output % x 10, plus the stall-alarm digit
inline constexpr uint16_t special_function  = 0x19;  // 2: quadrature (encoder) follow
}  // namespace reg

enum class Access : uint8_t { read_only, read_write };

struct Register {
    uint16_t addr;
    const char* name;   // log text, never wire text
    Access access;
};

inline constexpr std::array<Register, 26> kRegisters{{
    {reg::modbus_enable, "modbus-enable", Access::read_write},
    {reg::output_enable, "output-enable", Access::read_write},
    {reg::speed_limit, "speed-limit", Access::read_write},
    {reg::acceleration, "acceleration", Access::read_write},
    {reg::field_weakening, "field-weakening", Access::read_write},
    {reg::speed_kp, "speed-kp", Access::read_write},
    {reg::speed_ti, "speed-ti", Access::read_write},
    {reg::position_kp, "position-kp", Access::read_write},
    {reg::speed_feedforward, "speed-feedforward", Access::read_write},
    {reg::dir_polarity, "dir-polarity", Access::read_write},
    {reg::gear_numerator, "gear-numerator", Access::read_write},
    {reg::gear_denominator, "gear-denominator", Access::read_write},
    {reg::to_go_low, "to-go-low", Access::read_only},
    {reg::to_go_high, "to-go-high", Access::read_only},
    {reg::alarm_code, "alarm-code", Access::read_only},
    {reg::current, "current", Access::read_only},
    {reg::speed, "speed", Access::read_only},
    {reg::voltage, "voltage", Access::read_only},
    {reg::temperature, "temperature", Access::read_only},
    {reg::output_pwm, "output-pwm", Access::read_only},
    {reg::save_flag, "save-flag", Access::read_write},
    {reg::device_address, "device-address", Access::read_only},
    {reg::position_low, "position-low", Access::read_write},
    {reg::position_high, "position-high", Access::read_write},
    {reg::standstill, "standstill", Access::read_write},
    {reg::special_function, "special-function", Access::read_write},
}};

constexpr const char* registerName(uint16_t addr) {
    for (const Register& r : kRegisters)
        if (r.addr == addr) return r.name;
    return "?";
}

// ---- values -------------------------------------------------------------------

// 0x0E (manual p. 5). The manual has no row for 0; this reads it as none.
inline constexpr uint16_t kAlarmBattery     = 0x10;  // indication only: the motor keeps running
inline constexpr uint16_t kAlarmOverCurrent = 0x12;  // halts
inline constexpr uint16_t kAlarmStall       = 0x14;  // halts; armed by 0x18's ones digit
inline constexpr uint16_t kAlarmOverVoltage = 0x15;  // halts: regen from a large inertia

constexpr const char* alarmName(uint16_t code) {
    switch (code) {
        case 0:                 return "none";
        case kAlarmBattery:     return "battery";
        case kAlarmOverCurrent: return "over-current";
        case kAlarmStall:       return "stall";
        case kAlarmOverVoltage: return "over-voltage";
        default:                return "unknown";
    }
}

// 0x19's value for the A/B the LP core renders (manual p. 7).
inline constexpr uint16_t kQuadratureFollow = 2;
// 0x12's ceiling (manual p. 12). A reading above it is not this map.
inline constexpr uint16_t kMaxTemperatureC = 100;

// Two registers, low word first, as one signed 32-bit value (manual p. 14).
constexpr int32_t joinWords(uint16_t low, uint16_t high) {
    return int32_t(uint32_t(high) << 16 | low);
}
// 0x07's ones digit odd: WR is normally closed, conducting while healthy.
constexpr bool wrNormallyClosed(uint16_t positionKp) { return positionKp % 2u != 0; }
// 0x18's ones digit: 0 is no stall alarm (output halves after a 3 s stall).
constexpr uint8_t stallAlarmDigit(uint16_t standstill) { return uint8_t(standstill % 10u); }

// ---- THE KNOBS ------------------------------------------------------------------

// The drive's factory address (manual p. 13 examples).
inline constexpr uint8_t kAddress = 1;
// Every baud the drive can be set to (manual p. 18), the factory one first.
inline constexpr std::array<uint32_t, 4> kBauds{19200, 115200, 38400, 9600};
// TODO(val-091.69): both from the bench, the first against the drive's boot
// to its first Modbus answer.
// Motor power on this long before the drive is asked anything or WR counts.
inline constexpr uint32_t kSettleMs = 1000;
// A status line holds a level this long before it counts.
inline constexpr uint32_t kLineDebounceMs = 20;
// One runtime poll transaction this often: the alarm code and the encoder
// pair alternate, each every 2 x kPollEveryMs.
inline constexpr uint32_t kPollEveryMs = 50;
// Consecutive unanswered polls that mark the drive lost, and the wait before
// a probe after a failed one.
inline constexpr uint8_t kLostAfterFailures = 8;
inline constexpr uint32_t kReprobeMs = 3000;

// ---- motor power --------------------------------------------------------------

// The motor switch's `on` and how long it has held: the drive's boot clock.
// One tracker, read by both the lines and the link.
class DrivePower {
public:
    void sample(bool on, uint32_t nowMs) {
        if (!on) {
            _on = false;
            return;
        }
        if (!_on) {
            _on = true;
            _sinceMs = nowMs;
        }
    }
    bool on() const { return _on; }
    bool settled(uint32_t nowMs) const { return _on && nowMs - _sinceMs >= kSettleMs; }

private:
    bool _on = false;
    uint32_t _sinceMs = 0;
};

// ---- the WR alarm and RDY lines ---------------------------------------------------
// Each is an opto NPN to COM behind a 10k pull-up and 1k series: the pad reads
// LOW while the opto conducts. With 0x07's ones digit even, the only setting
// the probe passes, WR conducts on an alarm; RDY conducts while the drive is
// ready and its following error is under 0.5 degrees (manual p. 5).

class Line {
public:
    // Returns the debounced level after this sample.
    bool sample(bool raw, uint32_t nowMs) {
        if (!_seeded) {
            _seeded = true;
            _raw = _level = raw;
            _sinceMs = nowMs;
        } else if (raw != _raw) {
            _raw = raw;
            _sinceMs = nowMs;
        } else if (raw != _level && nowMs - _sinceMs >= kLineDebounceMs) {
            _level = raw;
        }
        return _level;
    }
    bool level() const { return _level; }

private:
    bool _seeded = false;
    bool _raw = false;
    bool _level = false;
    uint32_t _sinceMs = 0;
};

class StatusLines {
public:
    // One sample of both pads (true: LOW, the opto conducting). Returns true
    // ONCE per alarm: WR debounced and asserted while the drive is settled,
    // including one already asserted when it settles. A cleared WR, or any
    // loss of motor power, re-arms it.
    bool sample(bool alarmPadLow, bool readyPadLow, const DrivePower& power, uint32_t nowMs) {
        _alarm.sample(alarmPadLow, nowMs);
        _ready.sample(readyPadLow, nowMs);
        if (!power.settled(nowMs)) {
            _armed = false;
            _latched = false;
            return false;
        }
        const bool justArmed = !_armed;
        _armed = true;
        if (!_alarm.level()) {
            _latched = false;
            return false;
        }
        if (_latched) return false;
        _latched = true;
        _raisedAtArming = justArmed;
        ++_raised;
        return true;
    }

    bool alarm() const { return _alarm.level(); }   // debounced, armed or not
    bool ready() const { return _ready.level(); }   // debounced
    bool armed() const { return _armed; }
    uint32_t raised() const { return _raised; }     // since boot, wraps
    // The last alarm was already asserted when the drive settled: a drive in
    // alarm at power-on, or a WR wired normally closed.
    bool raisedAtArming() const { return _raisedAtArming; }

private:
    Line _alarm;
    Line _ready;
    uint32_t _raised = 0;
    bool _armed = false;
    bool _latched = false;
    bool _raisedAtArming = false;
};

// ---- the probe ------------------------------------------------------------------

enum class LinkState : uint8_t {
    unpowered,   // motor switch not on: the drive is dark
    settling,    // on, inside kSettleMs
    probing,     // running the identity probe
    up,          // a probe found the AIM map: the runtime poll runs
    down,        // the last probe failed, or the poll lost the drive
};

constexpr const char* linkStateName(LinkState s) {
    switch (s) {
        case LinkState::unpowered: return "unpowered";
        case LinkState::settling:  return "settling";
        case LinkState::probing:   return "probing";
        case LinkState::up:        return "up";
        case LinkState::down:      return "down";
    }
    return "?";
}

enum class ProbeOutcome : uint8_t {
    pending,          // none completed yet
    no_answer,        // silence at every baud
    crc,              // frames at some baud, none valid
    wrong_model,      // an answer, but not the AIM map: an exception, another
                      // address, a temperature past the map's range
    lost,             // found, then silent or broken before the probe ended
    modbus_enabled,   // 0x00 nonzero: Modbus owns the drive, quadrature ignored
    output_off,       // 0x01 not 1: the drive's output is disabled
    not_quadrature,   // 0x19 not 2: A/B are not read as quadrature
    alarm_inverted,   // 0x07 odd: WR normally closed, DRV_ALM reads inverted
    ok,
};

// The outcomes where the drive answered the whole map, so the poll runs.
constexpr bool mapAnswered(ProbeOutcome o) {
    return o == ProbeOutcome::modbus_enabled || o == ProbeOutcome::output_off ||
           o == ProbeOutcome::not_quadrature || o == ProbeOutcome::alarm_inverted ||
           o == ProbeOutcome::ok;
}

// Read in this order, one register a transaction, the identity first: the
// baud scan rides it. The encoder pair is one read of two, never torn.
inline constexpr std::array<uint16_t, 9> kProbeRegs{
    reg::device_address, reg::modbus_enable, reg::output_enable, reg::special_function,
    reg::position_kp,    reg::standstill,    reg::alarm_code,    reg::temperature,
    reg::position_low,
};

struct Probe {
    ProbeOutcome outcome = ProbeOutcome::pending;
    uint32_t baud = 0;       // the baud that answered; with crc, the one that framed
    uint16_t reg = 0;        // the register a wrong_model or lost verdict names
    uint16_t value = 0;      // what it read, when the verdict is by value
    uint8_t exception = 0;   // the device's code, when the verdict is by exception
    // As read, valid up to the register the probe reached:
    uint16_t address = 0;
    uint16_t modbusEnable = 0;
    uint16_t outputEnable = 0;
    uint16_t specialFunction = 0;
    uint16_t positionKp = 0;
    uint16_t standstill = 0;
    uint16_t alarmCode = 0;
    uint16_t temperatureC = 0;
    int32_t encoder = 0;
};

// The verdict on a probe that read every register.
constexpr ProbeOutcome judgeMap(Probe& p) {
    if (p.temperatureC > kMaxTemperatureC) {
        p.reg = reg::temperature;
        p.value = p.temperatureC;
        return ProbeOutcome::wrong_model;
    }
    if (p.modbusEnable != 0) return ProbeOutcome::modbus_enabled;
    if (p.outputEnable != 1) return ProbeOutcome::output_off;
    if (p.specialFunction != kQuadratureFollow) return ProbeOutcome::not_quadrature;
    if (wrNormallyClosed(p.positionKp)) return ProbeOutcome::alarm_inverted;
    return ProbeOutcome::ok;
}

// ---- the link -------------------------------------------------------------------

struct Snapshot {
    uint32_t seq = 0;              // bumped on every change: a reader's cue
    LinkState state = LinkState::unpowered;
    uint32_t baud = 0;             // the line's baud once found, 0 before
    Probe probe{};                 // the latest COMPLETED probe
    uint32_t probes = 0;           // completed since boot
    bool alarmCodeKnown = false;   // read since this power-on
    uint16_t alarmCode = 0;
    bool encoderKnown = false;     // read since this power-on
    int32_t encoder = 0;           // counts, 32768 a motor turn
    uint32_t encoderAtMs = 0;
    uint32_t polls = 0;            // runtime poll transactions since boot
    uint32_t pollFailures = 0;
};

// One external read's answer, tagged by its requester.
struct External {
    uint32_t tag = 0;
    modbus::Result result{};
};

class Link {
public:
    explicit Link(modbus::IPort& port) : _master(port) {}

    // One pass of the owning task. alarmRaised: StatusLines::sample() returned
    // true this pass, so the alarm code is read next.
    void step(uint32_t nowMs, const DrivePower& power, bool alarmRaised) {
        if (!power.on()) {
            dark(nowMs);
            return;
        }
        if (_snap.state == LinkState::unpowered) setState(LinkState::settling);
        if (alarmRaised) _alarmReadDue = true;
        if (_master.busy()) {
            const std::optional<modbus::Result> r = _master.poll(nowMs);
            if (!r) return;
            finish(*r, nowMs);
        }
        if (_snap.state == LinkState::settling && power.settled(nowMs)) beginProbe();
        if (_snap.state == LinkState::down && nowMs - _downAtMs >= kReprobeMs) beginProbe();
        if (_snap.state == LinkState::probing) startProbeRead(nowMs);
        else if (_snap.state == LinkState::up) startUpRead(nowMs);
    }

    const Snapshot& snapshot() const { return _snap; }

    // An external FC 0x03 read, served ahead of the poll while up. False when
    // not up, when one is pending, or for anything but a read.
    bool canSubmit() const { return _snap.state == LinkState::up && !_extPending && !_extReady; }
    bool submit(const modbus::Request& r, uint32_t tag) {
        if (!canSubmit() || r.fc != modbus::kFcReadHolding || !modbus::valid(r)) return false;
        _extReq = r;
        _extTag = tag;
        _extPending = true;
        _extStarted = false;
        return true;
    }
    // The external read's answer, once.
    std::optional<External> takeResult() {
        if (!_extReady) return std::nullopt;
        _extReady = false;
        return _ext;
    }

    uint32_t glitches() const { return _master.glitches(); }

private:
    enum class Job : uint8_t { none, probe, alarm_code, encoder, external };

    void touch() { ++_snap.seq; }
    void setState(LinkState s) {
        if (s != LinkState::up && _extPending && !_extStarted) completeExternal(modbus::Status::not_sent);
        _snap.state = s;
        touch();
    }

    void completeExternal(modbus::Status s) {
        _ext = External{};
        _ext.tag = _extTag;
        _ext.result.status = s;
        _extPending = false;
        _extReady = true;
    }

    void dark(uint32_t nowMs) {
        if (_snap.state == LinkState::unpowered) return;
        if (_job == Job::external) completeExternal(modbus::Status::timeout);
        _master.abort(nowMs);
        _job = Job::none;
        _alarmReadDue = false;
        _snap.alarmCodeKnown = false;
        _snap.encoderKnown = false;
        setState(LinkState::unpowered);
    }

    // The last baud that answered first, then the rest in kBauds order.
    uint32_t candidateBaud(size_t i) const {
        if (i == 0) return _goodBaud;
        size_t k = 0;
        for (const uint32_t b : kBauds) {
            if (b == _goodBaud) continue;
            if (++k == i) return b;
        }
        return 0;
    }

    void beginProbe() {
        _probe = Probe{};
        _step = 0;
        _baudIndex = 0;
        _framedBaud = 0;
        _master.setBaud(candidateBaud(0));
        setState(LinkState::probing);
    }

    void startProbeRead(uint32_t nowMs) {
        if (_master.busy()) return;
        const uint16_t r = kProbeRegs[_step];
        const uint8_t count = r == reg::position_low ? 2 : 1;
        if (_master.start(modbus::readHolding(kAddress, r, count), nowMs)) _job = Job::probe;
    }

    void startUpRead(uint32_t nowMs) {
        if (_master.busy()) return;
        if (_alarmReadDue) {
            // One attempt: the drive may be losing power as this goes out.
            if (_master.start(modbus::readHolding(kAddress, reg::alarm_code, 1), nowMs, 0)) {
                _alarmReadDue = false;
                _job = Job::alarm_code;
            }
            return;
        }
        if (_extPending && !_extStarted) {
            if (_master.start(_extReq, nowMs)) {
                _extStarted = true;
                _job = Job::external;
            }
            return;
        }
        if (nowMs - _lastPollMs < kPollEveryMs) return;
        const bool encoder = _pollEncoderNext;
        const modbus::Request q = encoder ? modbus::readHolding(kAddress, reg::position_low, 2)
                                          : modbus::readHolding(kAddress, reg::alarm_code, 1);
        if (!_master.start(q, nowMs)) return;
        _lastPollMs = nowMs;
        _pollEncoderNext = !encoder;
        _job = encoder ? Job::encoder : Job::alarm_code;
    }

    void finish(const modbus::Result& r, uint32_t nowMs) {
        const Job job = _job;
        _job = Job::none;
        switch (job) {
            case Job::probe: probeResult(r, nowMs); break;
            case Job::alarm_code:
            case Job::encoder: pollResult(r, job, nowMs); break;
            case Job::external:
                _ext = External{};
                _ext.tag = _extTag;
                _ext.result = r;
                _extPending = false;
                _extReady = true;
                touch();
                break;
            case Job::none: break;
        }
    }

    static void store(Probe& p, uint16_t r, const modbus::Result& res) {
        const uint16_t v = res.regs[0];
        switch (r) {
            case reg::device_address:   p.address = v; break;
            case reg::modbus_enable:    p.modbusEnable = v; break;
            case reg::output_enable:    p.outputEnable = v; break;
            case reg::special_function: p.specialFunction = v; break;
            case reg::position_kp:      p.positionKp = v; break;
            case reg::standstill:       p.standstill = v; break;
            case reg::alarm_code:       p.alarmCode = v; break;
            case reg::temperature:      p.temperatureC = v; break;
            case reg::position_low:     p.encoder = joinWords(res.regs[0], res.regs[1]); break;
            default: break;
        }
    }

    void probeResult(const modbus::Result& r, uint32_t nowMs) {
        using modbus::Status;
        const uint16_t at = kProbeRegs[_step];
        if (_step == 0 && r.status != Status::ok && r.status != Status::exception) {
            if (r.status != Status::timeout) _framedBaud = _master.baud();
            if (++_baudIndex < kBauds.size()) {
                _master.setBaud(candidateBaud(_baudIndex));
                return;
            }
            _probe.baud = _framedBaud;
            completeProbe(_framedBaud != 0 ? ProbeOutcome::crc : ProbeOutcome::no_answer, nowMs);
            return;
        }
        _probe.baud = _master.baud();
        if (r.status == Status::exception) {
            _probe.reg = at;
            _probe.exception = r.exception;
            completeProbe(ProbeOutcome::wrong_model, nowMs);
            return;
        }
        if (r.status != Status::ok) {
            _probe.reg = at;
            completeProbe(ProbeOutcome::lost, nowMs);
            return;
        }
        store(_probe, at, r);
        if (_step == 0 && _probe.address != kAddress) {
            _probe.reg = at;
            _probe.value = _probe.address;
            completeProbe(ProbeOutcome::wrong_model, nowMs);
            return;
        }
        if (++_step < kProbeRegs.size()) return;
        completeProbe(judgeMap(_probe), nowMs);
    }

    void completeProbe(ProbeOutcome o, uint32_t nowMs) {
        _probe.outcome = o;
        _snap.probe = _probe;
        ++_snap.probes;
        if (!mapAnswered(o)) {
            _downAtMs = nowMs;
            setState(LinkState::down);
            return;
        }
        _goodBaud = _probe.baud;
        _snap.baud = _probe.baud;
        _snap.alarmCode = _probe.alarmCode;
        _snap.alarmCodeKnown = true;
        _snap.encoder = _probe.encoder;
        _snap.encoderKnown = true;
        _snap.encoderAtMs = nowMs;
        _failures = 0;
        _lastPollMs = nowMs;
        setState(LinkState::up);
    }

    void pollResult(const modbus::Result& r, Job job, uint32_t nowMs) {
        using modbus::Status;
        ++_snap.polls;
        if (r.status == Status::ok) {
            _failures = 0;
            if (job == Job::alarm_code) {
                _snap.alarmCode = r.regs[0];
                _snap.alarmCodeKnown = true;
            } else {
                _snap.encoder = joinWords(r.regs[0], r.regs[1]);
                _snap.encoderKnown = true;
                _snap.encoderAtMs = nowMs;
            }
            touch();
            return;
        }
        ++_snap.pollFailures;
        if (r.status == Status::exception) {
            _failures = 0;   // an exception is an answer: the drive is there
            touch();
            return;
        }
        if (++_failures < kLostAfterFailures) {
            touch();
            return;
        }
        _snap.alarmCodeKnown = false;
        _snap.encoderKnown = false;
        _downAtMs = nowMs;
        setState(LinkState::down);
    }

    modbus::Master _master;
    Snapshot _snap{};
    Probe _probe{};
    modbus::Request _extReq{};
    External _ext{};
    uint32_t _extTag = 0;
    uint32_t _goodBaud = kBauds[0];
    uint32_t _framedBaud = 0;
    uint32_t _downAtMs = 0;
    uint32_t _lastPollMs = 0;
    size_t _step = 0;
    size_t _baudIndex = 0;
    Job _job = Job::none;
    uint8_t _failures = 0;
    bool _alarmReadDue = false;
    bool _pollEncoderNext = true;
    bool _extPending = false;
    bool _extStarted = false;
    bool _extReady = false;
};

}  // namespace valence::aim
