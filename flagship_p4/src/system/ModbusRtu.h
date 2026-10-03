#pragma once

// ModbusRtu -- a Modbus RTU master: CRC-16, the request and reply frames of
// FC 0x03, 0x06 and 0x10 and the exception reply, and a transaction engine
// that never waits: a reply deadline, retries and the inter-frame gap
// Constraints:
// - Hardware-free and allocation-free. The host hands in an IPort (its UART)
//   and a millisecond clock; ValenceDriveLink.cpp is the board's host, suite
//   test_drive_link drives this file through a fake port.
// - SINGLE OWNER: no method is safe against a concurrent call.
// - poll() takes what the port holds and returns. A reply deadline resolves
//   to the host's polling period, never finer.
// - Registers are big-endian on the wire, the CRC low byte first.
// - At most kMaxRegs registers a request. Address 0 (broadcast) and the
//   reserved 248-255 are refused: a broadcast has no reply to judge.
// - Milliseconds wrap-safe: unsigned differences only.
// See: modbus.org, Modbus over Serial Line V1.02 sections 2.5.1.1 and 6.2.2;
// Modbus Application Protocol V1.1b3 sections 6.3, 6.6, 6.12 and 7

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace valence::modbus {

// ---- frames -------------------------------------------------------------------

inline constexpr uint8_t kFcReadHolding = 0x03;
inline constexpr uint8_t kFcWriteSingle = 0x06;
inline constexpr uint8_t kFcWriteMultiple = 0x10;
inline constexpr uint8_t kExceptionBit = 0x80;

inline constexpr uint8_t kMaxRegs = 8;
// The largest frame either way: a FC 0x10 request of kMaxRegs values.
inline constexpr size_t kMaxFrameBytes = 9 + 2 * kMaxRegs;
// An exception reply, and the shortest reply of any kind.
inline constexpr size_t kExceptionBytes = 5;

// CRC-16/MODBUS: polynomial 0xA001 reflected, seed 0xFFFF, no final XOR.
constexpr uint16_t crc16(std::span<const uint8_t> bytes) {
    uint16_t crc = 0xFFFF;
    for (const uint8_t b : bytes) {
        crc = uint16_t(crc ^ b);
        for (int i = 0; i < 8; ++i)
            crc = (crc & 1u) != 0 ? uint16_t((crc >> 1) ^ 0xA001u) : uint16_t(crc >> 1);
    }
    return crc;
}

struct Request {
    uint8_t addr = 1;
    uint8_t fc = kFcReadHolding;
    uint16_t reg = 0;
    uint8_t count = 1;   // registers read, or values written (FC 0x06: 1)
    std::array<uint16_t, kMaxRegs> values{};   // FC 0x06 and 0x10 only
};

constexpr Request readHolding(uint8_t addr, uint16_t reg, uint8_t count) {
    Request r;
    r.addr = addr;
    r.fc = kFcReadHolding;
    r.reg = reg;
    r.count = count;
    return r;
}

constexpr Request writeSingle(uint8_t addr, uint16_t reg, uint16_t value) {
    Request r;
    r.addr = addr;
    r.fc = kFcWriteSingle;
    r.reg = reg;
    r.count = 1;
    r.values[0] = value;
    return r;
}

// More than kMaxRegs values yields an invalid request: start() refuses it.
constexpr Request writeMultiple(uint8_t addr, uint16_t reg, std::span<const uint16_t> values) {
    Request r;
    r.addr = addr;
    r.fc = kFcWriteMultiple;
    r.reg = reg;
    r.count = values.size() <= kMaxRegs ? uint8_t(values.size()) : 0;
    for (size_t i = 0; i < r.count; ++i) r.values[i] = values[i];
    return r;
}

constexpr bool valid(const Request& r) {
    if (r.addr == 0 || r.addr > 247) return false;
    switch (r.fc) {
        case kFcReadHolding:
        case kFcWriteMultiple: return r.count >= 1 && r.count <= kMaxRegs;
        case kFcWriteSingle:   return r.count == 1;
        default:               return false;
    }
}

constexpr size_t requestBytes(const Request& r) {
    if (!valid(r)) return 0;
    return r.fc == kFcWriteMultiple ? 9u + 2u * r.count : 8u;
}

// The normal (non-exception) reply.
constexpr size_t replyBytes(const Request& r) {
    if (!valid(r)) return 0;
    return r.fc == kFcReadHolding ? 5u + 2u * r.count : 8u;
}

// Writes the request frame; returns its length, 0 for an invalid request.
constexpr size_t encode(const Request& r, std::span<uint8_t, kMaxFrameBytes> out) {
    const size_t n = requestBytes(r);
    if (n == 0) return 0;
    out[0] = r.addr;
    out[1] = r.fc;
    out[2] = uint8_t(r.reg >> 8);
    out[3] = uint8_t(r.reg);
    if (r.fc == kFcWriteSingle) {
        out[4] = uint8_t(r.values[0] >> 8);
        out[5] = uint8_t(r.values[0]);
    } else {
        out[4] = 0;
        out[5] = r.count;
    }
    if (r.fc == kFcWriteMultiple) {
        out[6] = uint8_t(2u * r.count);
        for (size_t i = 0; i < r.count; ++i) {
            out[7 + 2 * i] = uint8_t(r.values[i] >> 8);
            out[8 + 2 * i] = uint8_t(r.values[i]);
        }
    }
    const uint16_t crc = crc16(std::span<const uint8_t>(out.data(), n - 2));
    out[n - 2] = uint8_t(crc);
    out[n - 1] = uint8_t(crc >> 8);
    return n;
}

enum class Status : uint8_t {
    ok,
    timeout,     // nothing arrived inside the deadline
    crc,         // a whole frame arrived and its CRC did not match
    malformed,   // a frame that does not answer the request: wrong address,
                 // function, length or echo, or one cut short at the deadline
    exception,   // the device refused; `exception` holds its code
    not_sent,    // nothing went on the wire: the host refused it first
};

constexpr const char* statusName(Status s) {
    switch (s) {
        case Status::ok:        return "ok";
        case Status::timeout:   return "timeout";
        case Status::crc:       return "crc";
        case Status::malformed: return "malformed";
        case Status::exception: return "exception";
        case Status::not_sent:  return "not sent";
    }
    return "?";
}

struct Result {
    Status status = Status::not_sent;
    uint8_t exception = 0;   // the device's code when status is exception
    uint8_t attempts = 0;    // frames sent for this result
    uint8_t count = 0;       // registers in regs, FC 0x03 only
    std::array<uint16_t, kMaxRegs> regs{};
};

constexpr uint16_t get16(std::span<const uint8_t> f, size_t at) {
    return uint16_t(uint16_t(f[at]) << 8 | f[at + 1]);
}

// Judges one complete frame against the request it answers; sets status,
// exception, count and regs.
constexpr Result decode(const Request& r, std::span<const uint8_t> f) {
    Result out;
    out.status = Status::malformed;
    if (f.size() < kExceptionBytes) return out;
    const size_t body = f.size() - 2;
    const uint16_t crc = crc16(f.first(body));
    if (f[body] != uint8_t(crc) || f[body + 1] != uint8_t(crc >> 8)) {
        out.status = Status::crc;
        return out;
    }
    if (f[0] != r.addr) return out;
    if (f[1] == uint8_t(r.fc | kExceptionBit)) {
        if (f.size() != kExceptionBytes) return out;
        out.status = Status::exception;
        out.exception = f[2];
        return out;
    }
    if (f[1] != r.fc || f.size() != replyBytes(r)) return out;
    switch (r.fc) {
        case kFcReadHolding:
            if (f[2] != uint8_t(2u * r.count)) return out;
            for (size_t i = 0; i < r.count; ++i) out.regs[i] = get16(f, 3 + 2 * i);
            out.count = r.count;
            break;
        case kFcWriteSingle:
            if (get16(f, 2) != r.reg || get16(f, 4) != r.values[0]) return out;
            break;
        case kFcWriteMultiple:
            if (get16(f, 2) != r.reg || get16(f, 4) != r.count) return out;
            break;
        default: return out;
    }
    out.status = Status::ok;
    return out;
}

// ---- line timing --------------------------------------------------------------

// Wire time of n characters, rounded up. 11 bits a character: 8N1 is 10, and
// the margin covers a device answering with two stop bits.
constexpr uint32_t wireMs(size_t chars, uint32_t baud) {
    return baud == 0 ? 0 : uint32_t((uint64_t(chars) * 11u * 1000u + baud - 1) / baud);
}

// The silence before a new request: 3.5 characters, fixed at 1.75 ms above
// 19200 baud (serial line spec 2.5.1.1), plus 1 ms for the clock's grain.
constexpr uint32_t gapMs(uint32_t baud) {
    if (baud == 0) return 0;
    return (baud > 19200 ? 2u : (38500u + baud - 1) / baud) + 1u;
}

// Retries after the first frame, on a timeout, CRC or malformed reply. An
// exception is an answer and is never retried.
inline constexpr uint8_t kRetries = 2;
// The device's turnaround allowance on top of both frames' wire time.
// TODO(val-091.69): measure the drive's turnaround on the bench.
inline constexpr uint32_t kReplyLatencyMs = 30;

// ---- the port seam ------------------------------------------------------------

// The UART as the master sees it. Every method returns at once.
class IPort {
public:
    virtual void setBaud(uint32_t baud) = 0;
    // Drops whatever has arrived and not been read.
    virtual void discardInput() = 0;
    // Queues one frame. The host's transceiver turns itself around: driver
    // on for the frame, receiver back on after it.
    virtual void write(std::span<const uint8_t> frame) = 0;
    // Copies up to into.size() received bytes; returns how many.
    virtual size_t read(std::span<uint8_t> into) = 0;

protected:
    ~IPort() = default;
};

// ---- the master ---------------------------------------------------------------

class Master {
public:
    explicit Master(IPort& port) : _port(port) {}

    // Idle only: the port and the frame-time arithmetic follow it.
    bool setBaud(uint32_t baud) {
        if (_busy || baud == 0) return false;
        _baud = baud;
        _port.setBaud(baud);
        return true;
    }
    uint32_t baud() const { return _baud; }
    bool busy() const { return _busy; }

    // Sends the request's first frame. False while a transaction is in
    // flight, inside the inter-frame gap, or for an invalid request.
    bool start(const Request& r, uint32_t nowMs, uint8_t retries = kRetries) {
        if (_busy || !valid(r) || !quiet(nowMs)) return false;
        _req = r;
        _retries = retries;
        _attempt = 0;
        _worst = Status::timeout;
        _busy = true;
        send(nowMs);
        return true;
    }

    // Advances the transaction in flight; returns its Result once, when it
    // completes.
    std::optional<Result> poll(uint32_t nowMs) {
        if (!_busy) return std::nullopt;
        if (_resendDue) {
            if (quiet(nowMs)) send(nowMs);
            return std::nullopt;
        }
        receive();
        const size_t need = expected();
        if (need != 0 && _rxLen >= need) {
            return conclude(decode(_req, std::span<const uint8_t>(_rx.data(), need)), nowMs);
        }
        if (nowMs - _sentMs < _deadlineMs) return std::nullopt;
        Result r;
        r.status = _rxLen == 0 ? Status::timeout : Status::malformed;
        return conclude(r, nowMs);
    }

    // Drops the transaction in flight with no Result. The gap still applies
    // to the next start(): a reply may still be on the wire.
    void abort(uint32_t nowMs) {
        _busy = false;
        _resendDue = false;
        markQuiet(nowMs);
    }

    // Leading bytes dropped because they were not the device's address: the
    // transceiver's turnaround glitches. Wraps.
    uint32_t glitches() const { return _glitches; }

private:
    bool quiet(uint32_t nowMs) const { return !_quietKnown || nowMs - _quietFromMs >= gapMs(_baud); }
    void markQuiet(uint32_t nowMs) {
        _quietFromMs = nowMs;
        _quietKnown = true;
    }

    void send(uint32_t nowMs) {
        std::array<uint8_t, kMaxFrameBytes> frame{};
        const size_t n = encode(_req, frame);
        _port.discardInput();
        _port.write(std::span<const uint8_t>(frame.data(), n));
        ++_attempt;
        _rxLen = 0;
        _resendDue = false;
        _sentMs = nowMs;
        _deadlineMs = wireMs(n + replyBytes(_req), _baud) + kReplyLatencyMs;
    }

    // Appends what the port holds, dropping leading bytes that cannot start
    // this device's reply. Bounded: every pass either reads or stops.
    void receive() {
        for (;;) {
            if (_rxLen == _rx.size()) return;
            const size_t got = _port.read(std::span<uint8_t>(_rx).subspan(_rxLen));
            if (got == 0) return;
            _rxLen += got;
            size_t skip = 0;
            while (skip < _rxLen && _rx[skip] != _req.addr) ++skip;
            if (skip == 0) continue;
            _glitches += uint32_t(skip);
            for (size_t i = skip; i < _rxLen; ++i) _rx[i - skip] = _rx[i];
            _rxLen -= skip;
        }
    }

    // Bytes the reply in hand needs, 0 until its function byte has arrived.
    size_t expected() const {
        if (_rxLen < 2) return 0;
        return (_rx[1] & kExceptionBit) != 0 ? kExceptionBytes : replyBytes(_req);
    }

    std::optional<Result> conclude(Result r, uint32_t nowMs) {
        markQuiet(nowMs);
        r.attempts = _attempt;
        if (r.status == Status::ok || r.status == Status::exception) {
            _busy = false;
            return r;
        }
        // A frame that arrived broken says more than silence does.
        if (r.status != Status::timeout) _worst = r.status;
        if (_attempt <= _retries) {
            _resendDue = true;
            return std::nullopt;
        }
        _busy = false;
        r.status = _worst;
        return r;
    }

    IPort& _port;
    Request _req{};
    std::array<uint8_t, kMaxFrameBytes> _rx{};
    size_t _rxLen = 0;
    uint32_t _baud = 19200;
    uint32_t _sentMs = 0;
    uint32_t _deadlineMs = 0;
    uint32_t _quietFromMs = 0;
    uint32_t _glitches = 0;
    uint8_t _attempt = 0;
    uint8_t _retries = 0;
    Status _worst = Status::timeout;
    bool _busy = false;
    bool _resendDue = false;
    bool _quietKnown = false;
};

}  // namespace valence::modbus
