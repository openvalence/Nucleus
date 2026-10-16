#pragma once

// NeutrinoLink -- one client's link as a network delivers it: latency,
// jitter, TCP-style loss and a Wi-Fi drop between the hub and that client's
// WebSocket transport, plus the link figures the dev tool shows
// Constraints:
// - HUB THREAD ONLY. The hub calls read() and write(); the port calls tick(),
//   setFaults(), setHeld(), bind() and stats() from the same thread. Nothing
//   here locks, so nothing here may be reached from a connection thread.
// - ORDER IS KEPT BOTH WAYS. A WebSocket rides TCP, so no frame overtakes the
//   one before it. A "lost" frame is a retransmit: it lands kRetransmitMs late
//   and holds every frame queued behind it (head-of-line blocking); it is
//   never missing, because the transport declares itself reliable.
// - With no fault set and nothing queued, write() and read() pass straight
//   through: a run without the dev tool is the bare transport.
// - BOUNDED. A full outbound queue refuses the write, the back-pressure a full
//   TCP send buffer gives the board's hub. A full inbound queue stops pulling,
//   so the transport's own RX ring overflows and counts the drop.
// - RTT is measured at the hub: a client PING pulled from the transport to
//   the hub's PONG handed back to it, so it holds both directions' injected
//   delay and no socket time.
// See: sim/valencesim/README.md (the dev tool)

#include <algorithm>
#include <cstdint>
#include <deque>
#include <optional>
#include <random>
#include <span>

#include "valence/generated/registry_constants.hpp"
#include "valence/transport/transport.hpp"
#include "valence/wire/frame_buffer.hpp"
#include "valence/wire/frame_header.hpp"
#include "valence/wire/messages/welcome.hpp"

namespace valence {

struct LinkFaults {
    uint32_t latency_ms = 0;   // one way
    uint32_t jitter_ms = 0;    // one way, uniform in [0, jitter] on top
    float loss_pct = 0.0f;     // share of frames retransmitted, 0..100
    bool operator==(const LinkFaults&) const = default;
};

// The global knobs and a client's own: delays add, losses compound.
inline LinkFaults combineFaults(const LinkFaults& a, const LinkFaults& b) {
    const float keep = (1.0f - std::clamp(a.loss_pct, 0.0f, 100.0f) / 100.0f) *
                       (1.0f - std::clamp(b.loss_pct, 0.0f, 100.0f) / 100.0f);
    return {a.latency_ms + b.latency_ms, a.jitter_ms + b.jitter_ms, (1.0f - keep) * 100.0f};
}

struct LinkStats {
    int32_t rtt_ms = -1;        // last PING in to its PONG out; -1 none yet
    uint32_t queued = 0;        // frames held, both directions
    uint32_t retransmits = 0;   // frames the loss knob delayed
    uint32_t refused = 0;       // writes refused on a full outbound queue
    uint32_t write_fails = 0;   // frames the transport refused (muted or closed)
    uint32_t session_id = 0;    // from the last WELCOME sent; 0 none
};

class NeutrinoLink final : public ITransport {
public:
    static constexpr size_t kQueueDepth = 256;
    static constexpr uint32_t kRetransmitMs = 200;   // TCP's minimum RTO
    static constexpr size_t kPingsTracked = 8;

    // A new connection behind this link: every queue and figure starts over.
    void bind(ITransport* inner) {
        _inner = inner;
        _rx.clear();
        _tx.clear();
        _pings.clear();
        _rxDue = _txDue = 0;
        _stats = {};
    }
    void setFaults(const LinkFaults& f) { _faults = f; }
    const LinkFaults& faults() const { return _faults; }
    // A Wi-Fi drop: nothing moves either way until released.
    void setHeld(bool held) { _held = held; }
    // Every pass: advance the clock and hand the transport what is due.
    void tick(uint64_t nowUs) {
        _nowUs = nowUs;
        while (!_held && !_tx.empty() && _tx.front().dueUs <= _nowUs) {
            const FrameBuffer fb = _tx.front().fb;
            _tx.pop_front();
            send(fb.bytes());
        }
    }
    LinkStats stats() const {
        LinkStats s = _stats;
        s.queued = uint32_t(_rx.size() + _tx.size());
        return s;
    }

    // ---- ITransport ------------------------------------------------------
    bool open() override { return _inner != nullptr && _inner->open(); }
    void close() override {
        if (_inner) _inner->close();
    }
    TransportProperties properties() const override { return _inner ? _inner->properties() : TransportProperties{}; }

    bool write(std::span<const std::byte> frame) override {
        if (!_inner) return false;
        noteWelcome(frame);
        if (passThrough() && _tx.empty()) return send(frame);
        if (_tx.size() >= kQueueDepth) {
            ++_stats.refused;
            return false;
        }
        _tx.push_back({schedule(_txDue), FrameBuffer::from(frame)});
        return true;
    }

    std::optional<FrameBuffer> read() override {
        if (!_inner) return std::nullopt;
        while (_rx.size() < kQueueDepth) {
            std::optional<FrameBuffer> fb = _inner->read();
            if (!fb) break;
            if (isType(fb->bytes(), FrameType::PING)) {
                if (_pings.size() == kPingsTracked) _pings.pop_front();
                _pings.push_back(_nowUs);
            }
            if (passThrough() && _rx.empty()) return fb;
            _rx.push_back({schedule(_rxDue), *fb});
        }
        if (_held || _rx.empty() || _rx.front().dueUs > _nowUs) return std::nullopt;
        FrameBuffer out = _rx.front().fb;
        _rx.pop_front();
        return out;
    }

private:
    struct Held {
        uint64_t dueUs = 0;
        FrameBuffer fb;
    };

    bool passThrough() const { return !_held && _faults == LinkFaults{}; }

    static bool isType(std::span<const std::byte> f, FrameType t) {
        const auto h = decodeFrameHeader(f);
        return h && h->type == uint8_t(t);
    }

    // The arrival of a frame entering now, in order behind the last one that
    // went this way.
    uint64_t schedule(uint64_t& lastDueUs) {
        uint64_t due = _nowUs + uint64_t(_faults.latency_ms) * 1000u;
        if (_faults.jitter_ms != 0)
            due += std::uniform_int_distribution<uint64_t>(0, uint64_t(_faults.jitter_ms) * 1000u)(_rng);
        if (_faults.loss_pct > 0.0f && std::uniform_real_distribution<float>(0.0f, 100.0f)(_rng) < _faults.loss_pct) {
            due += uint64_t(kRetransmitMs) * 1000u;
            ++_stats.retransmits;
        }
        lastDueUs = std::max(due, lastDueUs);
        return lastDueUs;
    }

    bool send(std::span<const std::byte> frame) {
        if (!_inner->write(frame)) {
            ++_stats.write_fails;
            return false;
        }
        if (!_pings.empty() && isType(frame, FrameType::PONG)) {
            _stats.rtt_ms = int32_t((_nowUs - _pings.front()) / 1000u);
            _pings.pop_front();
        }
        return true;
    }

    void noteWelcome(std::span<const std::byte> frame) {
        const auto h = decodeFrameHeader(frame);
        if (!h || h->type != uint8_t(FrameType::WELCOME) || h->fragMore() || frame.size() < kHeaderBytes) return;
        const auto w = decodeWelcome(frame.subspan(kHeaderBytes));
        if (w) _stats.session_id = w.value().session_id;
    }

    ITransport* _inner = nullptr;
    LinkFaults _faults{};
    bool _held = false;
    uint64_t _nowUs = 0;
    uint64_t _rxDue = 0;
    uint64_t _txDue = 0;
    std::deque<Held> _rx;
    std::deque<Held> _tx;
    std::deque<uint64_t> _pings;
    LinkStats _stats{};
    std::minstd_rand _rng{0x4e75u};
};

}  // namespace valence
