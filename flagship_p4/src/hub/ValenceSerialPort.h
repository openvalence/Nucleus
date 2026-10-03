#pragma once

// ValenceSerialPort -- the Valence serial binding (SPEC §13.5) over a byte
// pipe: COBS framing, one link, attached to the hub by its first frame
// Constraints:
// - HUB TASK ONLY. loop(), read(), write(), close() and every counter run on
//   the hub task; the pipe's ISR and rings are its driver's (T5). Nothing here
//   is shared with another task, so nothing here is atomic.
// - SPEC §13.5 FRAMING: COBS, delimiter 0x00. Every frame leaves as
//   0x00 + COBS(frame) + 0x00. The leading delimiter ends any console text the
//   pipe carried before it, so a reader drops that text as one chunk that is
//   not a frame.
// - THE PIPE IS SHARED: on the board it is the USB-Serial/JTAG console too.
//   Text lands between frames, never inside one, because a frame is one
//   all-or-nothing pipe write. Inbound, a chunk that is not exactly one frame
//   is dropped and counted, after the raw ESTOP scan §13.5 requires.
// - WRITE NEVER BLOCKS (ITransport). A frame the pipe refuses is shed
//   (STATE, STREAM), held for the hub to resend (BLOB_CHUNK), or queued
//   (control) and flushed next tick. A control queue stuck for kCtrlStallMs
//   detaches the link.
// - DETACH IS ONE FUNNEL (T3): idle reap, control stall and the hub's own
//   close() all end in detach(), on the hub task, before the next update().
// - LIVE IN EVERY MODE: the binding never switches off with the mode; config
//   mode (bd val-9u0.14) adds BLE GATT beside it, it does not enable this.
// - wifi_join (0x000F) frames go to the provisioning desk, never the hub
//   (ValenceProvisioning.h); every other frame passes through untouched.
// See: Valence SPEC.md §5.5, §6.3, §13.1, §13.5, §13.9; transport.md T3, T5

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "ValenceProvisioning.h"
#include "geiger/geiger.h"
#include "valence/hub/hub.hpp"
#include "valence/transport/transport.hpp"
#include "valence/wire/cbor/cbor_reader.hpp"
#include "valence/wire/estop_frame.hpp"
#include "valence/wire/frame_buffer.hpp"
#include "valence/wire/frame_header.hpp"
#include "valence/wire/messages/goodbye.hpp"
#include "valence/wire/serial_cobs.hpp"

namespace valence {

// ---- the byte pipe -------------------------------------------------------------

// What the link rides. Hub task only; both calls return at once.
class ISerialPipe {
public:
    virtual ~ISerialPipe() = default;
    // Up to out.size() received bytes; 0 when none are waiting.
    virtual size_t read(std::span<std::byte> out) = 0;
    // All of `bytes`, or none of them (false).
    virtual bool write(std::span<const std::byte> bytes) = 0;
};

// The board's pipe: the USB-Serial/JTAG driver, installed with the console
// moved onto it (ValenceSerialPort.cpp). nullptr when the install failed.
ISerialPipe* usbSerialPipeBegin();

// ---- framing -------------------------------------------------------------------

// The largest COBS body one kFrameBufferCapacity frame encodes to
// (serial_cobs.hpp's sizing rule), and a delimited frame on the wire.
inline constexpr size_t kSerialMaxChunk = kFrameBufferCapacity + kFrameBufferCapacity / 254 + 1;
inline constexpr size_t kSerialMaxEncoded = kSerialMaxChunk + 2;

// True when `f` is one whole frame: a CRC-valid 12-byte ESTOP, or a header
// whose `len` accounts for every byte after it.
inline bool serialFrameShaped(std::span<const std::byte> f) {
    if (f.size() == kEstopFrameBytes && f[0] == kEstopMagicByte && f[1] == kEstopMagicByte &&
        f[2] == kEstopMagicByte && f[3] == kEstopMagicByte)
        return bool(decodeEstop(f));
    const std::optional<FrameHeader> h = decodeFrameHeader(f);
    return h && size_t(h->len) == f.size() - kHeaderBytes;
}

// 0x00 + COBS(frame) + 0x00 into `out`; the byte count, or 0 when `out` is too
// small or `frame` is empty.
inline size_t encodeSerialFrame(std::span<const std::byte> frame, std::span<std::byte> out) {
    if (frame.empty() || out.size() < 3) return 0;
    out[0] = std::byte{0};
    const size_t n = cobsEncode(frame, out.subspan(1, out.size() - 2));
    if (n == 0) return 0;
    out[1 + n] = std::byte{0};
    return n + 2;
}

// SPEC §13.5's receiver: splits a byte stream on 0x00, COBS-decodes each chunk
// and keeps what is frame-shaped. Hardware-free; the native suite runs the
// client end of a link on it too.
class CobsDeframer {
public:
    // onFrame(const FrameBuffer&) for every whole frame in `bytes`, in order.
    // The buffer it receives is reused by the next call.
    template <class OnFrame>
    void feed(std::span<const std::byte> bytes, OnFrame&& onFrame) {
        for (const std::byte b : bytes) {
            if (b == std::byte{0}) {
                endChunk(onFrame);
                continue;
            }
            if (_len == _chunk.size()) overflow(onFrame);
            _chunk[_len++] = b;
        }
    }

    // Chunks that were not one frame: console text, a torn frame, junk.
    uint32_t dropped() const { return _dropped; }

private:
    // §13.5: a receiver in an unsynced or corrupt state still runs the
    // four-0xE5 scanner over the raw bytes; the CRC vets any candidate.
    template <class OnFrame>
    void scanRaw(std::span<const std::byte> raw, OnFrame& onFrame) {
        const EstopScanResult s = scanForEstop(raw);
        if (!s.found) return;
        std::memcpy(_frame.writable().data(), raw.data() + s.offset, kEstopFrameBytes);
        _frame.setSize(kEstopFrameBytes);
        onFrame(static_cast<const FrameBuffer&>(_frame));
    }

    template <class OnFrame>
    void endChunk(OnFrame& onFrame) {
        const std::span<const std::byte> raw(_chunk.data(), _len);
        const bool unsynced = _overflow;
        _len = 0;
        _overflow = false;
        // Back-to-back delimiters: the leading 0x00 of a frame after the
        // trailing 0x00 of the last. A boundary, not a chunk.
        if (raw.empty() && !unsynced) return;
        if (!unsynced) {
            const auto n = cobsDecode(raw, _frame.writable());
            if (n) {
                _frame.setSize(n.value());
                if (serialFrameShaped(_frame.bytes())) {
                    onFrame(static_cast<const FrameBuffer&>(_frame));
                    return;
                }
            }
        }
        ++_dropped;
        scanRaw(raw, onFrame);
    }

    // Too long to be a frame: scan what is here, keep the tail a magic run
    // could straddle, and drop the rest of the chunk at its delimiter.
    template <class OnFrame>
    void overflow(OnFrame& onFrame) {
        scanRaw(std::span<const std::byte>(_chunk), onFrame);
        constexpr size_t kKeep = kEstopFrameBytes - 1;
        std::memmove(_chunk.data(), _chunk.data() + _chunk.size() - kKeep, kKeep);
        _len = kKeep;
        _overflow = true;
    }

    std::array<std::byte, kSerialMaxChunk> _chunk{};
    size_t _len = 0;
    bool _overflow = false;
    FrameBuffer _frame{};
    uint32_t _dropped = 0;
};

// ---- the port ------------------------------------------------------------------

class ValenceSerialPort final : public ITransport {
public:
    // Decoded inbound frames the hub has not read yet. A burst past this is
    // dropped and counted; the pipe's own ring absorbs bursts first.
    static constexpr uint8_t kRxDepth = 8;
    // Control frames the pipe refused, held in order for the next tick.
    static constexpr uint8_t kTxDepth = 4;
    // The WS port's numbers, for the same reasons (ValenceWsPort.h): two
    // seconds of control that cannot leave is a client that stopped reading,
    // and ten missed idle pings is a client that is gone.
    static constexpr uint32_t kCtrlStallMs = 2000;
    static constexpr uint32_t kIdleReapMs = 10 * limits::ping_interval_idle_ms;
    // Pipe reads per tick, each up to kReadBytes: 2 KB per 5 ms tick, far over
    // what a client sends and a bound on the tick's work if a host floods.
    static constexpr size_t kReadBytes = 256;
    static constexpr uint8_t kReadsPerTick = 8;

    // `desk` may be nullptr: then wifi_join reaches the hub like any intent.
    void begin(Hub& hub, ISerialPipe& pipe, Provisioning* desk) {
        _hub = &hub;
        _pipe = &pipe;
        _desk = desk;
    }

    // Hub task, before Hub::update(): the deferred detach, the pipe's bytes,
    // the attach, the reaps and the congestion level.
    void loop(uint32_t nowMs);

    // Hub task, last act before a planned restart: one GOODBYE `code`,
    // best-effort, written past the queue.
    void goodbye(NackCode code);

    // ---- ITransport (hub task) ----------------------------------------------
    bool open() override { return _pipe != nullptr; }
    // The hub ends this link: detached at the top of the next loop(), never
    // inside update(), whose slot walk is running (T5).
    void close() override { _wantDetach = true; }
    bool write(std::span<const std::byte> frame) override;
    std::optional<FrameBuffer> read() override;
    TransportProperties properties() const override {
        TransportProperties p;
        p.mtu = uint16_t(kFrameBufferCapacity);   // §13.1 serial: max_frame 512
        p.ordered = true;
        p.reliable = true;                        // §13.1 †: USB CDC
        p.congestion = CongestionSignal::QueueWatermark;
        return p;
    }

    // ---- diagnostics (hub task) ----------------------------------------------
    bool attached() const { return _attached; }
    uint32_t sessionId() const { return _sessionId; }
    uint32_t framesRx() const { return _framesRx; }
    uint32_t framesTx() const { return _framesTx; }
    uint32_t junkChunks() const { return _rxFramer.dropped(); }
    uint32_t rxDrops() const { return _rxDrops; }
    uint32_t txDataDrops() const { return _txDataDrops; }
    uint32_t txCtrlFails() const { return _txCtrlFails; }

private:
    void detach();
    void push(const FrameBuffer& f);
    bool send(std::span<const std::byte> frame);
    bool enqueue(std::span<const std::byte> frame);
    void flushTx();
    void noteOutbound(std::span<const std::byte> frame);
    void latchUnattachedEstops();
    static uint32_t welcomeSessionId(std::span<const std::byte> payload);

    Hub* _hub = nullptr;
    ISerialPipe* _pipe = nullptr;
    Provisioning* _desk = nullptr;

    CobsDeframer _rxFramer{};
    std::array<std::byte, kReadBytes> _readBuf{};
    std::array<FrameBuffer, kRxDepth> _rx{};
    uint8_t _rxHead = 0;
    uint8_t _rxCount = 0;
    std::array<FrameBuffer, kTxDepth> _tx{};
    uint8_t _txHead = 0;
    uint8_t _txCount = 0;
    std::array<std::byte, kSerialMaxEncoded> _encoded{};

    bool _attached = false;
    bool _wantDetach = false;
    // From the WELCOME this link carried; 0 before one and after GOODBYE.
    uint32_t _sessionId = 0;
    uint32_t _nowMs = 0;
    uint32_t _lastRxMs = 0;
    uint32_t _txStallSinceMs = 0;
    uint8_t _blobThisTick = 0;

    uint32_t _framesRx = 0;
    uint32_t _framesTx = 0;
    uint32_t _rxDrops = 0;
    uint32_t _txDataDrops = 0;
    uint32_t _txCtrlFails = 0;
};

// ---- port implementation -----------------------------------------------------------

namespace detail {
inline constexpr const char* kSerialTag = "serial";
}  // namespace detail

inline void ValenceSerialPort::push(const FrameBuffer& f) {
    if (_rxCount == kRxDepth) {
        ++_rxDrops;
        return;
    }
    _rx[(_rxHead + _rxCount) % kRxDepth] = f;
    ++_rxCount;
    ++_framesRx;
}

inline void ValenceSerialPort::loop(uint32_t nowMs) {
    _nowMs = nowMs;
    _blobThisTick = 0;
    if (_pipe == nullptr || _hub == nullptr) return;
    if (_wantDetach) detach();

    for (uint8_t i = 0; i < kReadsPerTick; ++i) {
        const size_t n = _pipe->read(_readBuf);
        if (n == 0) break;
        const uint32_t before = _framesRx;
        _rxFramer.feed(std::span<const std::byte>(_readBuf.data(), n), [this](const FrameBuffer& f) { push(f); });
        // A frame is proof of life; console noise typed into the port is not.
        if (_framesRx != before) _lastRxMs = nowMs;
        if (n < _readBuf.size()) break;
    }

    if (!_attached && _rxCount > 0) {
        if (_hub->attachTransport(*this)) {
            _attached = true;
            _lastRxMs = nowMs;
            GLOGI(detail::kSerialTag, "link attached");
        } else {
            // Every hub slot is taken: the frames are refused, but SPEC §6.3
            // keeps ESTOP reachable at capacity.
            latchUnattachedEstops();
            GLOGW_EVERY_MS(5000, detail::kSerialTag, "no hub slot free: frames dropped");
        }
    }
    if (!_attached) return;

    flushTx();
    if (uint32_t(nowMs - _lastRxMs) > kIdleReapMs) {
        GLOGW(detail::kSerialTag, "link silent >%lu ms: detached", static_cast<unsigned long>(kIdleReapMs));
        detach();
        return;
    }
    if (_txCount > 0 && uint32_t(nowMs - _txStallSinceMs) > kCtrlStallMs) {
        GLOGW(detail::kSerialTag, "control stalled >%lu ms: detached", static_cast<unsigned long>(kCtrlStallMs));
        detach();
        return;
    }
    // §10.4: severe while never-shed frames cannot leave. Level 1 is never
    // reported: data the pipe refuses is shed at write(), and a USB host
    // either drains at bus speed or not at all.
    _hub->setCongestionLevel(*this, _txCount > 0 ? 2 : 0);
}

inline void ValenceSerialPort::detach() {
    // The hub's own detach path calls close() on this link; the flag clears
    // after it, or the next tick would detach again.
    if (_attached) _hub->detachTransport(*this);
    _wantDetach = false;
    _attached = false;
    _sessionId = 0;
    _rxHead = _rxCount = 0;
    _txHead = _txCount = 0;
    _txStallSinceMs = 0;
    if (_desk != nullptr) _desk->forget(*this);
}

inline std::optional<FrameBuffer> ValenceSerialPort::read() {
    while (_rxCount > 0) {
        FrameBuffer& slot = _rx[_rxHead];
        _rxHead = uint8_t((_rxHead + 1) % kRxDepth);
        --_rxCount;
        if (_desk != nullptr && _desk->offer(slot.bytes(), *this, _sessionId, _nowMs)) {
            // A wifi_join carries the passphrase: the slot does not keep it.
            std::memset(slot.writable().data(), 0, kFrameBufferCapacity);
            slot.setSize(0);
            continue;
        }
        return slot;
    }
    return std::nullopt;
}

inline bool ValenceSerialPort::send(std::span<const std::byte> frame) {
    const size_t n = encodeSerialFrame(frame, _encoded);
    return n != 0 && _pipe->write(std::span<const std::byte>(_encoded.data(), n));
}

inline bool ValenceSerialPort::enqueue(std::span<const std::byte> frame) {
    if (_txCount == kTxDepth) {
        ++_txCtrlFails;
        return false;
    }
    FrameBuffer& slot = _tx[(_txHead + _txCount) % kTxDepth];
    std::memcpy(slot.writable().data(), frame.data(), frame.size());
    slot.setSize(frame.size());
    if (_txCount++ == 0) _txStallSinceMs = _nowMs;
    return true;
}

inline void ValenceSerialPort::flushTx() {
    while (_txCount > 0 && send(_tx[_txHead].bytes())) {
        _txHead = uint8_t((_txHead + 1) % kTxDepth);
        --_txCount;
        ++_framesTx;
    }
    if (_txCount == 0) _txStallSinceMs = 0;
}

inline bool ValenceSerialPort::write(std::span<const std::byte> frame) {
    if (!_attached || frame.empty() || frame.size() > kFrameBufferCapacity) return false;
    noteOutbound(frame);
    const uint8_t type = uint8_t(frame[0]);
    const bool data = type == uint8_t(FrameType::STATE) || type == uint8_t(FrameType::STREAM);
    const bool blob = type == uint8_t(FrameType::BLOB_CHUNK);

    // Behind a queued control frame nothing may overtake it.
    if (_txCount > 0) {
        if (data) {
            ++_txDataDrops;
            return false;
        }
        if (blob) return false;   // held: the hub resends this index next tick
        return enqueue(frame);
    }
    // T16: bulk chunks are paced against the registry's own sender budget.
    if (blob && _blobThisTick >= limits::blob_chunks_in_flight) return false;
    if (send(frame)) {
        ++_framesTx;
        if (blob) ++_blobThisTick;
        return true;
    }
    if (data) {
        ++_txDataDrops;
        return false;
    }
    if (blob) return false;
    return enqueue(frame);
}

inline uint32_t ValenceSerialPort::welcomeSessionId(std::span<const std::byte> payload) {
    CborReader r(payload);
    const auto n = r.readMapHeader();
    if (!n) return 0;
    for (uint32_t i = 0; i < n.value(); ++i) {
        const auto k = r.readKey();
        if (!k) return 0;
        if (k.value() == uint64_t(CborKey::session_id)) {
            const auto v = r.readUint();
            return v ? uint32_t(v.value()) : 0;
        }
        if (!r.skipValue()) return 0;
    }
    return 0;
}

// The session this link carries is learned from what the hub tells it: the
// WELCOME names it, a GOODBYE ends it.
inline void ValenceSerialPort::noteOutbound(std::span<const std::byte> frame) {
    const std::optional<FrameHeader> h = decodeFrameHeader(frame);
    if (!h || size_t(h->len) != frame.size() - kHeaderBytes) return;
    if (h->type == uint8_t(FrameType::WELCOME)) {
        const uint32_t id = welcomeSessionId(frame.subspan(kHeaderBytes));
        if (id != _sessionId && _desk != nullptr) _desk->forget(*this);
        _sessionId = id;
    } else if (h->type == uint8_t(FrameType::GOODBYE)) {
        _sessionId = 0;
        if (_desk != nullptr) _desk->forget(*this);
    }
}

inline void ValenceSerialPort::latchUnattachedEstops() {
    for (; _rxCount > 0; --_rxCount, _rxHead = uint8_t((_rxHead + 1) % kRxDepth)) {
        const std::span<const std::byte> f = _rx[_rxHead].bytes();
        if (f.size() != kEstopFrameBytes || f[0] != kEstopMagicByte) continue;
        const auto e = decodeEstop(f);
        if (e) _hub->latchEstop(e.value().cause, e.value().origin, e.value().seq);
    }
    _rxHead = 0;
}

inline void ValenceSerialPort::goodbye(NackCode code) {
    if (!_attached) return;
    GoodbyeMsg gb;
    gb.code = code;
    std::array<std::byte, kHeaderBytes + 32> frame{};
    const size_t n = encodeGoodbye(gb, std::span<std::byte>(frame).subspan(kHeaderBytes));
    if (n == 0) return;
    FrameHeader h;
    h.type = uint8_t(FrameType::GOODBYE);
    h.len = uint16_t(n);
    if (encodeFrameHeader(h, frame) != kHeaderBytes) return;
    send(std::span<const std::byte>(frame.data(), kHeaderBytes + n));
}

}  // namespace valence
