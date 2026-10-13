#pragma once

// ValenceLinkPort -- one point-to-point Valence binding over a frame pipe,
// attached to the hub by its first frame. The serial binding (SPEC §13.5,
// ValenceSerialPort.h) and BLE GATT (§13.4, ValenceBle.h) are its two pipes.
// Constraints:
// - HUB TASK ONLY. loop(), read(), write(), close() and every counter run on
//   the hub task. A pipe whose link lives on another task (BLE's host task)
//   hands frames across inside the pipe (T5); nothing here is atomic.
// - THE PIPE OWNS FRAMING AND THE LINK: drain() yields whole frames, send()
//   writes one whole frame or none of it, linkEpoch() changes whenever the
//   link under the pipe ends or a new one begins (0 = no link).
// - WRITE NEVER BLOCKS (ITransport). A frame the pipe refuses is shed
//   (STATE, STREAM), held for the hub to resend (BLOB_CHUNK), or queued
//   (control) and flushed next tick. A control queue stuck for kCtrlStallMs
//   detaches the link.
// - DETACH IS ONE FUNNEL (T3): idle reap, control stall, a changed link epoch
//   and the hub's own close() all end in detach(), on the hub task, before the
//   next update().
// - wifi_join (0x000F) frames go to the provisioning desk, never the hub
//   (ValenceProvisioning.h); every other frame passes through untouched. Only
//   a binding that may carry credentials (SPEC §13.9) is built on this port.
// See: Valence SPEC.md §5.5, §6.3, §13.1, §13.4, §13.5, §13.9; transport.md T3, T5

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

namespace valence {

// Where a pipe delivers inbound frames. The buffer is the pipe's: copy it.
class FrameSink {
public:
    virtual void push(const FrameBuffer& f) = 0;

protected:
    ~FrameSink() = default;
};

// What a link port rides. Hub task only; every call returns at once.
class IFramePipe {
public:
    virtual ~IFramePipe() = default;
    // Every whole inbound frame waiting, oldest first.
    virtual void drain(FrameSink& sink) = 0;
    // One whole frame out, all or nothing.
    virtual bool send(std::span<const std::byte> frame) = 0;
    virtual TransportProperties properties() const = 0;
    // Changes whenever the link under the pipe ends or a new one begins;
    // 0 = no link. A byte pipe that is always there returns a constant.
    virtual uint32_t linkEpoch() const { return 1; }
};

class ValenceLinkPort : public ITransport, private FrameSink {
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
    // How long one refused data frame keeps the link at congestion level 1.
    static constexpr uint32_t kShedHoldMs = 1000;

    // `desk` may be nullptr: then wifi_join reaches the hub like any intent.
    // `tag` is a string literal naming the binding in the log.
    void begin(Hub& hub, IFramePipe& pipe, Provisioning* desk, const char* tag) {
        _hub = &hub;
        _pipe = &pipe;
        _desk = desk;
        _tag = tag;
    }

    // Hub task, before Hub::update(): the deferred detach, the pipe's frames,
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
    TransportProperties properties() const override { return _pipe->properties(); }

    // ---- diagnostics (hub task) ----------------------------------------------
    bool attached() const { return _attached; }
    uint32_t sessionId() const { return _sessionId; }
    uint32_t framesRx() const { return _framesRx; }
    uint32_t framesTx() const { return _framesTx; }
    uint32_t rxDrops() const { return _rxDrops; }
    uint32_t txDataDrops() const { return _txDataDrops; }
    uint32_t txCtrlFails() const { return _txCtrlFails; }

private:
    void push(const FrameBuffer& f) override;
    void detach();
    bool enqueue(std::span<const std::byte> frame);
    void flushTx();
    void noteOutbound(std::span<const std::byte> frame);
    void latchUnattachedEstops();
    static uint32_t welcomeSessionId(std::span<const std::byte> payload);

    Hub* _hub = nullptr;
    IFramePipe* _pipe = nullptr;
    Provisioning* _desk = nullptr;
    const char* _tag = "link";

    std::array<FrameBuffer, kRxDepth> _rx{};
    uint8_t _rxHead = 0;
    uint8_t _rxCount = 0;
    std::array<FrameBuffer, kTxDepth> _tx{};
    uint8_t _txHead = 0;
    uint8_t _txCount = 0;

    bool _attached = false;
    bool _wantDetach = false;
    // The pipe's link epoch at attach; a different one is a different link.
    uint32_t _epoch = 0;
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
    uint32_t _dropsSeen = 0;
    uint32_t _shedAtMs = 0;
};

// ---- port implementation -----------------------------------------------------------

inline void ValenceLinkPort::push(const FrameBuffer& f) {
    if (_rxCount == kRxDepth) {
        ++_rxDrops;
        return;
    }
    _rx[(_rxHead + _rxCount) % kRxDepth] = f;
    ++_rxCount;
    ++_framesRx;
}

inline void ValenceLinkPort::loop(uint32_t nowMs) {
    _nowMs = nowMs;
    _blobThisTick = 0;
    if (_pipe == nullptr || _hub == nullptr) return;
    if (_wantDetach) detach();
    if (_attached && _pipe->linkEpoch() != _epoch) {
        GLOGW(_tag, "link ended: detached");
        detach();
    }

    const uint32_t before = _framesRx;
    _pipe->drain(*this);
    // A frame is proof of life; console noise typed into a byte pipe is not.
    if (_framesRx != before) _lastRxMs = nowMs;

    if (!_attached && _rxCount > 0) {
        if (_pipe->linkEpoch() == 0) {
            _rxHead = _rxCount = 0;   // frames of a link that is already gone
        } else if (_hub->attachTransport(*this)) {
            _attached = true;
            _epoch = _pipe->linkEpoch();
            _lastRxMs = nowMs;
            GLOGI(_tag, "link attached");
        } else {
            // Every hub slot is taken: the frames are refused, but SPEC §6.3
            // keeps ESTOP reachable at capacity.
            latchUnattachedEstops();
            GLOGW_EVERY_MS(5000, _tag, "no hub slot free: frames dropped");
        }
    }
    if (!_attached) return;

    flushTx();
    if (uint32_t(nowMs - _lastRxMs) > kIdleReapMs) {
        GLOGW(_tag, "link silent >%lu ms: detached", static_cast<unsigned long>(kIdleReapMs));
        detach();
        return;
    }
    if (_txCount > 0 && uint32_t(nowMs - _txStallSinceMs) > kCtrlStallMs) {
        GLOGW(_tag, "control stalled >%lu ms: detached", static_cast<unsigned long>(kCtrlStallMs));
        detach();
        return;
    }
    // §10.4: severe while never-shed frames cannot leave; moderate while the
    // pipe has refused data within kShedHoldMs, so the hub sheds at the
    // source instead of every frame meeting a full pipe.
    const bool shedding = _txDataDrops != _dropsSeen;
    _dropsSeen = _txDataDrops;
    if (shedding) _shedAtMs = nowMs;
    const bool recent = _shedAtMs != 0 && uint32_t(nowMs - _shedAtMs) < kShedHoldMs;
    _hub->setCongestionLevel(*this, _txCount > 0 ? 2 : recent ? 1 : 0);
}

inline void ValenceLinkPort::detach() {
    // The hub's own detach path calls close() on this link; the flag clears
    // after it, or the next tick would detach again.
    if (_attached) _hub->detachTransport(*this);
    _wantDetach = false;
    _attached = false;
    _epoch = 0;
    _sessionId = 0;
    _rxHead = _rxCount = 0;
    _txHead = _txCount = 0;
    _txStallSinceMs = 0;
    if (_desk != nullptr) _desk->forget(*this);
}

inline std::optional<FrameBuffer> ValenceLinkPort::read() {
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

inline bool ValenceLinkPort::enqueue(std::span<const std::byte> frame) {
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

inline void ValenceLinkPort::flushTx() {
    while (_txCount > 0 && _pipe->send(_tx[_txHead].bytes())) {
        _txHead = uint8_t((_txHead + 1) % kTxDepth);
        --_txCount;
        ++_framesTx;
    }
    if (_txCount == 0) _txStallSinceMs = 0;
}

inline bool ValenceLinkPort::write(std::span<const std::byte> frame) {
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
    if (_pipe->send(frame)) {
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

inline uint32_t ValenceLinkPort::welcomeSessionId(std::span<const std::byte> payload) {
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
inline void ValenceLinkPort::noteOutbound(std::span<const std::byte> frame) {
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

inline void ValenceLinkPort::latchUnattachedEstops() {
    for (; _rxCount > 0; --_rxCount, _rxHead = uint8_t((_rxHead + 1) % kRxDepth)) {
        const std::span<const std::byte> f = _rx[_rxHead].bytes();
        if (f.size() != kEstopFrameBytes || f[0] != kEstopMagicByte) continue;
        const auto e = decodeEstop(f);
        if (e) _hub->latchEstop(e.value().cause, e.value().origin, e.value().seq);
    }
    _rxHead = 0;
}

inline void ValenceLinkPort::goodbye(NackCode code) {
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
    _pipe->send(std::span<const std::byte>(frame.data(), kHeaderBytes + n));
}

}  // namespace valence
