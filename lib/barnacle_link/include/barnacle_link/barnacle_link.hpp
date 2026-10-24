// barnacle_link -- the barnacle helper link: COBS frames with a version byte,
// a sequence number and a CRC-16 between the Stamp-P4 (nucleus-p4b) and the
// OSSM R+D board's WROOM-32D (barnacle_shim), and the trip detector the
// helper runs
// Constraints:
// - Header-only, hardware-free, allocation-free, no exceptions. The P4 (IDF),
//   the shim (Arduino ESP32) and the native suite compile this one file.
// - THE ONE HOME of every link number: baud, rates, timeouts, frame layouts
//   and the trip detector's timing. Neither firmware restates one.
// - On the wire a frame is COBS(payload) then one 0x00. The payload is
//   version, type, seq, body, then CRC-16/CCITT-FALSE (poly 0x1021, init
//   0xFFFF) over version..body, low byte first. Every multi-byte field is
//   little-endian, written and read byte by byte.
// - Each sender keeps ONE sequence counter across all of its frame types;
//   the receiver counts the gaps as lost frames.
// - A layout change bumps kVersion: a receiver drops every frame of another
//   version and counts it, never guesses at its fields.
// - Bandwidth: a status frame is 14 bytes on the wire, 70 kbit/s at 500 Hz,
//   with the slow frame's 3 kbit/s beside it, inside 115200 baud. A field
//   added to the status frame costs 5 kbit/s per byte.
// See: Nucleus flagship_p4/src/system/BarnacleLink.h (the P4 side),
// barnacle_shim/src/main.cpp (the helper side), openvalence.github.io
// guides/barnacle (the wiring)

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace barnacle {

// ---- the link's numbers ----------------------------------------------------

inline constexpr uint8_t  kVersion = 1;
inline constexpr uint32_t kBaud = 115200;

// Helper to P4: one status frame every kStatusPeriodUs, one slow frame every
// kSlowEvery status frames (10 Hz).
inline constexpr uint32_t kStatusPeriodUs = 2000;
inline constexpr uint32_t kSlowEvery = 50;
// P4 to helper: one command frame every kCommandPeriodMs. The command is the
// heartbeat: there is no separate heartbeat frame.
inline constexpr uint32_t kCommandPeriodMs = 5;
// The P4 reads the helper stale after this long without a valid frame: 50
// status frames.
inline constexpr uint32_t kHelperStaleMs = 100;
// The helper reads the P4 silent after this long without a valid command,
// holds the last thresholds and mode it was given, and says so in its flags.
inline constexpr uint32_t kP4SilentMs = 250;

static_assert(kHelperStaleMs * 1000 >= 10 * kStatusPeriodUs, "stale must outlast a burst of lost frames");
static_assert(kP4SilentMs >= 10 * kCommandPeriodMs, "silent must outlast a burst of lost commands");

// ---- the trip detector's timing -------------------------------------------
// The helper samples motor current every kTripSampleUs. kTripOnSamples
// consecutive samples at or over the active threshold assert TRIP;
// kTripOffSamples consecutive samples under kTripReleasePercent of it release
// it. kTripDetectUs is the stall-detect half of the P4's contact-to-park
// latency: the P4 static_asserts it against its own figure.
inline constexpr uint32_t kTripSampleUs = 150;
inline constexpr uint32_t kTripOnSamples = 3;
inline constexpr uint32_t kTripOffSamples = 3;
inline constexpr uint32_t kTripReleasePercent = 50;
inline constexpr uint32_t kTripDetectUs = kTripSampleUs * kTripOnSamples;

// The trip thresholds, in the helper's current scale (provisional mA until
// the CH701 is calibrated, bd val-omh3.4): what the P4 sends, and what the
// helper applies from boot until its first command. Homing is the stall
// against the end stop (the S3 home sense's 1.0 A), running a jam.
// TODO(val-omh3.5): both from the bench.
inline constexpr uint16_t kDefaultHomeTripMa = 1000;
inline constexpr uint16_t kDefaultRunTripMa = 8000;

// ---- vocabulary --------------------------------------------------------------

enum class Type : uint8_t {
    status  = 0x01,   // helper to P4, kStatusPeriodUs
    slow    = 0x02,   // helper to P4, every kSlowEvery status frames
    command = 0x81,   // P4 to helper, kCommandPeriodMs
};

// What the P4 is doing, which picks the helper's active threshold. Wire
// ordinals.
enum class Mode : uint8_t {
    idle    = 0,   // motion gate closed: the running threshold, nothing acts on TRIP
    homing  = 1,   // a home cycle: the homing threshold, TRIP is the stall
    running = 2,   // motion gate open: the running threshold, TRIP stops motion
};

constexpr const char* modeName(Mode m) {
    switch (m) {
        case Mode::idle:    return "idle";
        case Mode::homing:  return "homing";
        case Mode::running: return "running";
    }
    return "?";
}

// Status flags.
inline constexpr uint8_t kFlagButton     = 0x01;   // S2 pressed now
inline constexpr uint8_t kFlagTrip       = 0x02;   // IO27 driven: TRIP low on the wire
inline constexpr uint8_t kFlagCalibrated = 0x04;   // current_ma is calibrated, not provisional
inline constexpr uint8_t kFlagP4Silent   = 0x08;   // no valid command for kP4SilentMs
inline constexpr uint8_t kFlagSpareIn    = 0x10;   // J8 pin 4 (IO4) reads high

// No reading, in a field that has one.
inline constexpr int16_t  kNoCurrent = INT16_MIN;
inline constexpr uint16_t kNoMillivolts = 0xFFFF;

// ---- the frames' bodies ------------------------------------------------------

struct Status {
    uint8_t  flags = 0;
    uint16_t current_raw = 0;              // ADC counts on IO36, averaged
    int16_t  current_ma = kNoCurrent;      // provisional until kFlagCalibrated
    uint16_t motor_mv = kNoMillivolts;     // J2 supply

    static constexpr size_t kBytes = 7;
    bool operator==(const Status&) const = default;
};

struct Slow {
    uint32_t uptime_ms = 0;
    uint16_t boots = 0;           // helper boots since power-on, this one included
    uint8_t  reset_reason = 0;    // esp_reset_reason_t of this boot
    Mode     mode = Mode::idle;   // the mode the helper applies
    uint16_t home_trip_ma = 0;    // thresholds in force
    uint16_t run_trip_ma = 0;
    uint16_t ana1_mv = kNoMillivolts;
    uint16_t ana2_mv = kNoMillivolts;
    uint16_t trips = 0;           // TRIP assertions since boot, wraps
    uint16_t bad_frames = 0;      // P4 frames dropped (CRC, version, layout), wraps
    uint16_t late_samples = 0;    // current samples later than kTripSampleUs allows, wraps
    uint8_t  shim_version = 0;

    static constexpr size_t kBytes = 23;
    bool operator==(const Slow&) const = default;
};

struct Command {
    Mode     mode = Mode::idle;
    uint8_t  r = 0, g = 0, b = 0;   // the status pixel, gamma applied
    uint16_t home_trip_ma = 0;
    uint16_t run_trip_ma = 0;

    static constexpr size_t kBytes = 8;
    bool operator==(const Command&) const = default;
};

// ---- sizes -------------------------------------------------------------------

inline constexpr size_t kHeaderBytes = 3;   // version, type, seq
inline constexpr size_t kCrcBytes = 2;
inline constexpr size_t kMaxBody = 24;
inline constexpr size_t kMaxPayload = kHeaderBytes + kMaxBody + kCrcBytes;
// COBS adds one byte per 254 and the delimiter one more.
inline constexpr size_t kMaxWire = kMaxPayload + kMaxPayload / 254 + 2;

static_assert(Status::kBytes <= kMaxBody && Slow::kBytes <= kMaxBody && Command::kBytes <= kMaxBody,
              "a body outgrew kMaxBody");

// ---- CRC ---------------------------------------------------------------------

// CRC-16/CCITT-FALSE. Check value: "123456789" -> 0x29B1.
constexpr uint16_t crc16(std::span<const uint8_t> data, uint16_t crc = 0xFFFF) {
    for (const uint8_t byte : data) {
        crc = uint16_t(crc ^ uint16_t(byte << 8));
        for (int i = 0; i < 8; ++i) crc = (crc & 0x8000u) ? uint16_t((crc << 1) ^ 0x1021u) : uint16_t(crc << 1);
    }
    return crc;
}

// ---- COBS --------------------------------------------------------------------

// Encodes `in` into `out` with no delimiter. Returns the encoded length, 0
// when `out` is too small.
constexpr size_t cobsEncode(std::span<const uint8_t> in, std::span<uint8_t> out) {
    if (out.size() < in.size() + in.size() / 254 + 1) return 0;
    size_t code_at = 0;
    size_t n = 1;
    uint8_t code = 1;
    for (const uint8_t byte : in) {
        if (byte == 0) {
            out[code_at] = code;
            code_at = n++;
            code = 1;
            continue;
        }
        out[n++] = byte;
        if (++code == 0xFF) {
            out[code_at] = code;
            code_at = n++;
            code = 1;
        }
    }
    out[code_at] = code;
    return n;
}

// Decodes one delimiter-free block. Returns the decoded length, nullopt for a
// block that is not COBS (a zero inside, a code past the end) or that does not
// fit `out`.
constexpr std::optional<size_t> cobsDecode(std::span<const uint8_t> in, std::span<uint8_t> out) {
    size_t i = 0;
    size_t n = 0;
    while (i < in.size()) {
        const uint8_t code = in[i++];
        if (code == 0 || i + code - 1 > in.size()) return std::nullopt;
        for (uint8_t k = 1; k < code; ++k) {
            if (in[i] == 0 || n >= out.size()) return std::nullopt;
            out[n++] = in[i++];
        }
        if (code != 0xFF && i < in.size()) {
            if (n >= out.size()) return std::nullopt;
            out[n++] = 0;
        }
    }
    return n;
}

// ---- field packing -------------------------------------------------------------

inline void putU16(std::span<uint8_t> o, size_t at, uint16_t v) {
    o[at] = uint8_t(v);
    o[at + 1] = uint8_t(v >> 8);
}
inline void putU32(std::span<uint8_t> o, size_t at, uint32_t v) {
    for (size_t k = 0; k < 4; ++k) o[at + k] = uint8_t(v >> (8 * k));
}
inline uint16_t getU16(std::span<const uint8_t> b, size_t at) {
    return uint16_t(b[at] | (uint16_t(b[at + 1]) << 8));
}
inline uint32_t getU32(std::span<const uint8_t> b, size_t at) {
    uint32_t v = 0;
    for (size_t k = 0; k < 4; ++k) v |= uint32_t(b[at + k]) << (8 * k);
    return v;
}

inline void packBody(const Status& s, std::span<uint8_t> o) {
    o[0] = s.flags;
    putU16(o, 1, s.current_raw);
    putU16(o, 3, uint16_t(s.current_ma));
    putU16(o, 5, s.motor_mv);
}
inline void packBody(const Slow& s, std::span<uint8_t> o) {
    putU32(o, 0, s.uptime_ms);
    putU16(o, 4, s.boots);
    o[6] = s.reset_reason;
    o[7] = uint8_t(s.mode);
    putU16(o, 8, s.home_trip_ma);
    putU16(o, 10, s.run_trip_ma);
    putU16(o, 12, s.ana1_mv);
    putU16(o, 14, s.ana2_mv);
    putU16(o, 16, s.trips);
    putU16(o, 18, s.bad_frames);
    putU16(o, 20, s.late_samples);
    o[22] = s.shim_version;
}
inline void packBody(const Command& c, std::span<uint8_t> o) {
    o[0] = uint8_t(c.mode);
    o[1] = c.r;
    o[2] = c.g;
    o[3] = c.b;
    putU16(o, 4, c.home_trip_ma);
    putU16(o, 6, c.run_trip_ma);
}

constexpr bool validMode(uint8_t m) { return m <= uint8_t(Mode::running); }

// ---- encoding ----------------------------------------------------------------

// One frame onto the wire form, delimiter included. Returns its length, 0 when
// `out` is too small or the body too long.
inline size_t encodeFrame(Type type, uint8_t seq, std::span<const uint8_t> body, std::span<uint8_t> out) {
    if (body.size() > kMaxBody) return 0;
    std::array<uint8_t, kMaxPayload> p{};
    p[0] = kVersion;
    p[1] = uint8_t(type);
    p[2] = seq;
    for (size_t k = 0; k < body.size(); ++k) p[kHeaderBytes + k] = body[k];
    const size_t covered = kHeaderBytes + body.size();
    putU16(p, covered, crc16(std::span<const uint8_t>(p.data(), covered)));
    const size_t n = cobsEncode(std::span<const uint8_t>(p.data(), covered + kCrcBytes), out);
    if (n == 0 || n >= out.size()) return 0;
    out[n] = 0;
    return n + 1;
}

inline size_t encode(const Status& s, uint8_t seq, std::span<uint8_t> out) {
    std::array<uint8_t, Status::kBytes> b{};
    packBody(s, b);
    return encodeFrame(Type::status, seq, b, out);
}
inline size_t encode(const Slow& s, uint8_t seq, std::span<uint8_t> out) {
    std::array<uint8_t, Slow::kBytes> b{};
    packBody(s, b);
    return encodeFrame(Type::slow, seq, b, out);
}
inline size_t encode(const Command& c, uint8_t seq, std::span<uint8_t> out) {
    std::array<uint8_t, Command::kBytes> b{};
    packBody(c, b);
    return encodeFrame(Type::command, seq, b, out);
}

// ---- decoding ----------------------------------------------------------------

// One verified frame: the version matched, the CRC held.
struct Frame {
    Type type = Type::status;
    uint8_t seq = 0;
    std::array<uint8_t, kMaxBody> body{};
    uint8_t len = 0;

    std::span<const uint8_t> bytes() const { return std::span<const uint8_t>(body.data(), len); }
};

inline std::optional<Status> asStatus(const Frame& f) {
    if (f.type != Type::status || f.len != Status::kBytes) return std::nullopt;
    const std::span<const uint8_t> b = f.bytes();
    Status s;
    s.flags = b[0];
    s.current_raw = getU16(b, 1);
    s.current_ma = int16_t(getU16(b, 3));
    s.motor_mv = getU16(b, 5);
    return s;
}
inline std::optional<Slow> asSlow(const Frame& f) {
    if (f.type != Type::slow || f.len != Slow::kBytes || !validMode(f.body[7])) return std::nullopt;
    const std::span<const uint8_t> b = f.bytes();
    Slow s;
    s.uptime_ms = getU32(b, 0);
    s.boots = getU16(b, 4);
    s.reset_reason = b[6];
    s.mode = Mode(b[7]);
    s.home_trip_ma = getU16(b, 8);
    s.run_trip_ma = getU16(b, 10);
    s.ana1_mv = getU16(b, 12);
    s.ana2_mv = getU16(b, 14);
    s.trips = getU16(b, 16);
    s.bad_frames = getU16(b, 18);
    s.late_samples = getU16(b, 20);
    s.shim_version = b[22];
    return s;
}
inline std::optional<Command> asCommand(const Frame& f) {
    if (f.type != Type::command || f.len != Command::kBytes || !validMode(f.body[0])) return std::nullopt;
    const std::span<const uint8_t> b = f.bytes();
    Command c;
    c.mode = Mode(b[0]);
    c.r = b[1];
    c.g = b[2];
    c.b = b[3];
    c.home_trip_ma = getU16(b, 4);
    c.run_trip_ma = getU16(b, 6);
    return c;
}

// What the receiver threw away, by cause. Each wraps.
struct RxStats {
    uint32_t frames = 0;      // verified frames handed out
    uint32_t crc = 0;         // decoded, CRC wrong: noise, or a frame joined mid-way
    uint32_t version = 0;     // CRC held, another kVersion
    uint32_t malformed = 0;   // not COBS, or shorter than a header and a CRC
    uint32_t overrun = 0;     // longer than kMaxWire before a delimiter
};

// The byte-stream side: feed every received byte; a delimiter ends a block,
// and a verified block comes back as a Frame. Resync is the delimiter: a lost
// or corrupted byte costs the frame it was in and nothing after it.
// SINGLE OWNER: the task that reads the UART.
class Decoder {
public:
    std::optional<Frame> feed(uint8_t byte) {
        if (byte != 0) {
            if (_discarding) return std::nullopt;
            if (_n == _buf.size()) {
                ++_stats.overrun;
                _discarding = true;
                _n = 0;
                return std::nullopt;
            }
            _buf[_n++] = byte;
            return std::nullopt;
        }
        const size_t n = _n;
        _n = 0;
        if (_discarding) {
            _discarding = false;
            return std::nullopt;
        }
        if (n == 0) return std::nullopt;   // back-to-back delimiters
        return finish(std::span<const uint8_t>(_buf.data(), n));
    }

    const RxStats& stats() const { return _stats; }

private:
    std::optional<Frame> finish(std::span<const uint8_t> block) {
        std::array<uint8_t, kMaxPayload> p{};
        const std::optional<size_t> got = cobsDecode(block, p);
        if (!got || *got < kHeaderBytes + kCrcBytes) {
            ++_stats.malformed;
            return std::nullopt;
        }
        const size_t covered = *got - kCrcBytes;
        if (crc16(std::span<const uint8_t>(p.data(), covered)) != getU16(p, covered)) {
            ++_stats.crc;
            return std::nullopt;
        }
        if (p[0] != kVersion) {
            ++_stats.version;
            return std::nullopt;
        }
        Frame f;
        f.type = Type(p[1]);
        f.seq = p[2];
        f.len = uint8_t(covered - kHeaderBytes);
        for (size_t k = 0; k < f.len; ++k) f.body[k] = p[kHeaderBytes + k];
        ++_stats.frames;
        return f;
    }

    std::array<uint8_t, kMaxWire> _buf{};
    size_t _n = 0;
    bool _discarding = false;
    RxStats _stats{};
};

// Lost frames from the sender's one sequence counter. A gap of 128 or more is
// read as an outage, never as a count: reset() after one.
class SeqTracker {
public:
    // Returns the frames this one says were lost before it.
    uint32_t note(uint8_t seq) {
        uint32_t gap = 0;
        if (_have) {
            const uint8_t d = uint8_t(seq - _last);
            gap = (d == 0 || d >= 128) ? 0u : uint32_t(d - 1);
        }
        _have = true;
        _last = seq;
        _lost += gap;
        return gap;
    }
    void reset() { _have = false; }
    uint32_t lost() const { return _lost; }

private:
    bool _have = false;
    uint8_t _last = 0;
    uint32_t _lost = 0;
};

// ---- the trip detector (the helper runs it; the suite drives it) -----------------

// Consecutive-sample hysteresis on the signed current, in the same mA scale
// the thresholds come in. Regeneration reads negative and never trips. A
// threshold of 0 disables the trip: it never asserts and releases at once.
// SINGLE OWNER: the helper's sampling loop.
class TripDetector {
public:
    // One sample. Returns the asserted state after it.
    bool sample(int32_t ma, uint16_t threshold_ma) {
        if (threshold_ma == 0) {
            _asserted = false;
            _run = 0;
            return false;
        }
        const int32_t release = int32_t(threshold_ma) * int32_t(kTripReleasePercent) / 100;
        const bool toward = _asserted ? (ma < release) : (ma >= int32_t(threshold_ma));
        _run = toward ? _run + 1 : 0;
        if (!_asserted && _run >= kTripOnSamples) {
            _asserted = true;
            _run = 0;
            ++_assertions;
        } else if (_asserted && _run >= kTripOffSamples) {
            _asserted = false;
            _run = 0;
        }
        return _asserted;
    }
    bool asserted() const { return _asserted; }
    uint16_t assertions() const { return _assertions; }

private:
    bool _asserted = false;
    uint32_t _run = 0;
    uint16_t _assertions = 0;
};

// The threshold a mode applies: homing's in a home cycle, running's otherwise.
constexpr uint16_t activeThreshold(Mode m, uint16_t home_ma, uint16_t run_ma) {
    return m == Mode::homing ? home_ma : run_ma;
}

}  // namespace barnacle
