#pragma once

// ValenceSerialPort -- the Valence serial binding (SPEC §13.5) over a byte
// pipe: COBS framing under a ValenceLinkPort
// Constraints:
// - HUB TASK ONLY, as ValenceLinkPort.h; the pipe's ISR and rings are its
//   driver's (T5).
// - SPEC §13.5 FRAMING: COBS, delimiter 0x00. Every frame leaves as
//   0x00 + COBS(frame) + 0x00. The leading delimiter ends any console text the
//   pipe carried before it, so a reader drops that text as one chunk that is
//   not a frame.
// - THE PIPE IS SHARED: on the board it is the USB-Serial/JTAG console too.
//   Text lands between frames, never inside one, because a frame is one
//   all-or-nothing pipe write. Inbound, a chunk that is not exactly one frame
//   is dropped and counted, after the raw ESTOP scan §13.5 requires.
// - LIVE IN EVERY MODE: the binding never switches off with the mode; config
//   mode adds BLE GATT beside it (ValenceBle.h), it does not enable this.
// See: Valence SPEC.md §5.5, §13.1, §13.5, §13.9; ValenceLinkPort.h

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "ValenceLinkPort.h"
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

// ---- the pipe and the port -------------------------------------------------------

// SPEC §13.5 over a byte pipe: COBS frames out, the deframer in.
class CobsFramePipe final : public IFramePipe {
public:
    // Pipe reads per drain, each up to kReadBytes: 2 KB per 5 ms tick, far
    // over what a client sends and a bound on the tick's work if a host floods.
    static constexpr size_t kReadBytes = 256;
    static constexpr uint8_t kReadsPerTick = 8;

    void begin(ISerialPipe& pipe) { _pipe = &pipe; }

    void drain(FrameSink& sink) override {
        for (uint8_t i = 0; i < kReadsPerTick; ++i) {
            const size_t n = _pipe->read(_readBuf);
            if (n == 0) break;
            _rxFramer.feed(std::span<const std::byte>(_readBuf.data(), n),
                           [&sink](const FrameBuffer& f) { sink.push(f); });
            if (n < _readBuf.size()) break;
        }
    }
    bool send(std::span<const std::byte> frame) override {
        const size_t n = encodeSerialFrame(frame, _encoded);
        return n != 0 && _pipe->write(std::span<const std::byte>(_encoded.data(), n));
    }
    TransportProperties properties() const override {
        TransportProperties p;
        p.mtu = uint16_t(kFrameBufferCapacity);   // §13.1 serial: max_frame 512
        p.ordered = true;
        p.reliable = true;                        // §13.1 †: USB CDC
        p.congestion = CongestionSignal::QueueWatermark;
        return p;
    }
    uint32_t junkChunks() const { return _rxFramer.dropped(); }

private:
    ISerialPipe* _pipe = nullptr;
    CobsDeframer _rxFramer{};
    std::array<std::byte, kReadBytes> _readBuf{};
    std::array<std::byte, kSerialMaxEncoded> _encoded{};
};

namespace detail {
inline constexpr const char* kSerialTag = "serial";
}  // namespace detail

// The serial binding: a link port on a COBS pipe.
class ValenceSerialPort final : public ValenceLinkPort {
public:
    // `desk` may be nullptr: then wifi_join reaches the hub like any intent.
    void begin(Hub& hub, ISerialPipe& pipe, Provisioning* desk) {
        _cobs.begin(pipe);
        ValenceLinkPort::begin(hub, _cobs, desk, detail::kSerialTag, true);
    }
    // §12.2: the hub tiers the serial binding configure. Declared here, never
    // on the COBS pipe, which other links may ride.
    TransportProperties properties() const override {
        TransportProperties p = ValenceLinkPort::properties();
        p.binding = BindingKind::serial;
        return p;
    }
    uint32_t junkChunks() const { return _cobs.junkChunks(); }

private:
    CobsFramePipe _cobs{};
};

}  // namespace valence
