#pragma once

// ValenceBle -- the Valence BLE GATT binding (SPEC §13.4): the frame pipe a
// ValenceLinkPort rides, and the board's NimBLE host behind it
// Constraints:
// - TWO TASKS, ONE HAND-OFF (T5). NimBLE's host task writes inbound frames
//   into BleRxRing (single producer) and the link's MTU and epoch into
//   atomics; the hub task alone drains, sends, fragments and reassembles.
// - ONE WRITE = ONE FRAME, ONE NOTIFICATION = ONE FRAME (SPEC §13.4), each at
//   most ATT_MTU - 3. A control frame larger than that leaves as SPEC §5.6
//   fragments; STATE and STREAM never fragment (§9.1, §5.4): the pipe refuses
//   them and the port sheds them. Inbound fragments reassemble here.
// - ONE LINK. The host allows one connection; a new one is a new epoch, and
//   the port detaches the old session on the change (ValenceLinkPort.h).
// - The board half (ValenceBle.cpp) brings the controller up over esp_hosted
//   VHCI on its own task: those are RPCs to the C6, never run on the hub
//   task. It is started only when LinkMode wants BLE (Valence RFC-117 draft).
// See: Valence SPEC.md §5.6, §13.1, §13.4, §13.4.1; ValenceLinkPort.h, LinkMode.h

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "ValenceLinkPort.h"
#include "valence/wire/fragmentation.hpp"
#include "valence/wire/frame_buffer.hpp"
#include "valence/wire/frame_header.hpp"

namespace valence {

// ---- the cross-task ring -------------------------------------------------------

// Single producer (the BLE host task), single consumer (the hub task). Each
// slot carries the link epoch it arrived on, so frames of a dead link never
// reach the next one.
class BleRxRing {
public:
    static constexpr uint8_t kDepth = 8;

    // Producer. False (dropped) when full or the frame is oversized.
    bool put(uint32_t epoch, std::span<const std::byte> frame) {
        if (frame.empty() || frame.size() > kFrameBufferCapacity) return false;
        const uint32_t head = _head.load(std::memory_order_relaxed);
        if (head - _tail.load(std::memory_order_acquire) == kDepth) return false;
        Slot& s = _slots[head % kDepth];
        s.epoch = epoch;
        std::memcpy(s.frame.writable().data(), frame.data(), frame.size());
        s.frame.setSize(frame.size());
        _head.store(head + 1, std::memory_order_release);
        return true;
    }

    // Consumer. fn(epoch, const FrameBuffer&) per frame, oldest first.
    template <class Fn>
    void take(Fn&& fn) {
        uint32_t tail = _tail.load(std::memory_order_relaxed);
        const uint32_t head = _head.load(std::memory_order_acquire);
        for (; tail != head; ++tail) {
            Slot& s = _slots[tail % kDepth];
            fn(s.epoch, static_cast<const FrameBuffer&>(s.frame));
            // A wifi_join carries the passphrase: the slot does not keep it.
            std::memset(s.frame.writable().data(), 0, s.frame.size());
            _tail.store(tail + 1, std::memory_order_release);
        }
    }

private:
    struct Slot {
        uint32_t epoch = 0;
        FrameBuffer frame{};
    };
    std::array<Slot, kDepth> _slots{};
    std::atomic<uint32_t> _head{0};
    std::atomic<uint32_t> _tail{0};
};

// ---- the notify seam -------------------------------------------------------------

// The board's notification on the h2c characteristic; the native suite fakes
// it. Hub task. False when the host has no buffer for it now.
class IBleNotifier {
public:
    virtual ~IBleNotifier() = default;
    virtual bool notify(std::span<const std::byte> value) = 0;
};

// ---- the pipe ------------------------------------------------------------------

class BleFramePipe final : public IFramePipe {
public:
    // SPEC §5.6 fragments of one control frame still waiting for a host
    // buffer. 512 B at the 247 B ATT MTU floor §13.4 asks for is 3; a client
    // stuck below that gets control frames up to 8 fragments long.
    static constexpr uint8_t kFragDepth = 8;
    // ATT_MTU before an exchange (Core Spec Vol 3 Part F 3.2.8).
    static constexpr uint16_t kAttMtuDefault = 23;

    void begin(IBleNotifier& notifier) { _notifier = &notifier; }

    // ---- BLE host task ---------------------------------------------------------
    // The MTU is NOT reset here: the host can report the exchange before the
    // connection (measured on the P4, esp_hosted VHCI); it resets on the
    // disconnect instead.
    void onConnect() { _epoch.store(++_lastEpoch, std::memory_order_release); }
    void onDisconnect() {
        _epoch.store(0, std::memory_order_release);
        _mtu.store(kAttMtuDefault, std::memory_order_relaxed);
    }
    void onMtu(uint16_t attMtu) { _mtu.store(attMtu, std::memory_order_relaxed); }
    // One characteristic write = one frame (or one §5.6 fragment).
    bool onWrite(std::span<const std::byte> value) {
        const uint32_t e = _epoch.load(std::memory_order_acquire);
        if (e == 0) return false;
        if (_rx.put(e, value)) return true;
        _rxDrops.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    // ---- hub task --------------------------------------------------------------
    void drain(FrameSink& sink) override {
        const uint32_t now = _epoch.load(std::memory_order_acquire);
        if (now != _linkEpoch) {
            _linkEpoch = now;
            _reasm = Reassembler{};
            _open = {};
        }
        _rx.take([&](uint32_t e, const FrameBuffer& f) {
            if (e != now) return;
            const auto h = decodeFrameHeader(f.bytes());
            if (!h || f.size() != kHeaderBytes + size_t(h->len) || !fragment(h.value())) {
                sink.push(f);   // a whole frame, the raw ESTOP included
                return;
            }
            const auto r = _reasm.accept(h.value(), f.bytes().subspan(kHeaderBytes), _nowMs);
            if (r && r.value()) sink.push(*r.value());
        });
        _reasm.expireStale(_nowMs);
        flushFrags();
    }

    bool send(std::span<const std::byte> frame) override {
        if (_notifier == nullptr || _epoch.load(std::memory_order_acquire) == 0) return false;
        flushFrags();
        if (_fragCount > 0) return false;   // nothing overtakes a fragmented frame
        const uint16_t max = maxFrame();
        if (frame.size() <= max) return _notifier->notify(frame);
        const uint8_t type = uint8_t(frame[0]);
        if (type == uint8_t(FrameType::STATE) || type == uint8_t(FrameType::STREAM)) return false;
        const size_t payload = max > kHeaderBytes + kFragIndexBytes ? max - kHeaderBytes - kFragIndexBytes : 0;
        if (payload == 0 || (frame.size() - kHeaderBytes + payload - 1) / payload > kFragDepth) return false;
        _fragEpoch = _epoch.load(std::memory_order_acquire);
        const bool ok = fragmentFrame(frame, max, [this](std::span<const std::byte> frag) {
            FrameBuffer& slot = _frags[(_fragHead + _fragCount) % kFragDepth];
            std::memcpy(slot.writable().data(), frag.data(), frag.size());
            slot.setSize(frag.size());
            ++_fragCount;
        });
        if (!ok) {
            _fragCount = 0;
            return false;
        }
        flushFrags();
        return true;
    }

    TransportProperties properties() const override {
        TransportProperties p;
        p.mtu = maxFrame();
        p.ordered = true;    // §13.1 BLE GATT: notifications ordered
        p.reliable = false;  // §13.1: notify is unacknowledged
        p.congestion = CongestionSignal::NotifyQueueDepth;
        return p;
    }

    uint32_t linkEpoch() const override { return _epoch.load(std::memory_order_acquire); }

    // Hub task, before the port's loop: the reassembler's clock.
    void setNow(uint32_t nowMs) { _nowMs = nowMs; }
    // The largest whole frame one value carries: ATT_MTU - 3, capped at the
    // frame buffer (SPEC §13.1 BLE GATT: 244 at an MTU of 247).
    uint16_t maxFrame() const {
        const uint16_t att = _mtu.load(std::memory_order_relaxed);
        return uint16_t(std::min<size_t>(att > 3 ? att - 3 : 0, kFrameBufferCapacity));
    }
    uint32_t rxDrops() const { return _rxDrops.load(std::memory_order_relaxed); }

private:
    // SPEC §5.6: START and MORE mark a fragment, and the LAST fragment carries
    // neither, so it is told from a whole frame only by the (type, seq) of a
    // reassembly still open. Opened by a fragment with MORE, closed by the
    // last, expired with the reassembler's own timeout.
    struct OpenKey {
        uint8_t type = 0;
        uint16_t seq = 0;
        uint32_t sinceMs = 0;
        bool used = false;
    };
    bool fragment(const FrameHeader& h) {
        for (OpenKey& k : _open)
            if (k.used && _nowMs - k.sinceMs >= limits::frag_reassembly_timeout_ms) k.used = false;
        OpenKey* match = nullptr;
        for (OpenKey& k : _open)
            if (k.used && k.type == h.type && k.seq == h.seq) match = &k;
        if (h.fragMore()) {
            if (match == nullptr) {
                match = &_open[0];
                for (OpenKey& k : _open)
                    if (!k.used || k.sinceMs < match->sinceMs) match = &k;
                *match = OpenKey{h.type, h.seq, _nowMs, true};
            }
            return true;
        }
        if (h.fragStart()) return true;   // a single-fragment message
        if (match == nullptr) return false;
        match->used = false;
        return true;
    }

    void flushFrags() {
        if (_fragCount > 0 && _fragEpoch != _epoch.load(std::memory_order_acquire)) _fragCount = 0;
        while (_fragCount > 0 && _notifier->notify(_frags[_fragHead].bytes())) {
            _fragHead = uint8_t((_fragHead + 1) % kFragDepth);
            --_fragCount;
        }
        if (_fragCount == 0) _fragHead = 0;
    }

    IBleNotifier* _notifier = nullptr;
    BleRxRing _rx{};
    std::atomic<uint32_t> _epoch{0};
    std::atomic<uint16_t> _mtu{kAttMtuDefault};
    std::atomic<uint32_t> _rxDrops{0};
    uint32_t _lastEpoch = 0;   // BLE host task only

    // Hub task only.
    uint32_t _linkEpoch = 0;
    uint32_t _nowMs = 0;
    Reassembler _reasm{};
    std::array<OpenKey, limits::frag_max_concurrent_per_session> _open{};
    std::array<FrameBuffer, kFragDepth> _frags{};
    uint8_t _fragHead = 0;
    uint8_t _fragCount = 0;
    uint32_t _fragEpoch = 0;   // the link the queued fragments belong to
};

// ---- board functions (ValenceBle.cpp; not built natively) -----------------------

namespace ble {

// The pipe the BLE link port rides. Hub task, from hubBegin().
BleFramePipe& pipe();

// Starts the bring-up task once: the C6's controller over esp_hosted, the
// NimBLE host, the service. Any task; returns at once. Later calls are no-ops.
void bringUp(const char* hubName);
// The controller and the host are up and the service is registered.
bool ready();
// Advertising on or off, and the scan response's flags byte (SPEC §13.4).
// Hub task; applied by the host task. Off also ends a connected link.
void setAdvertising(bool on, uint8_t advFlags);

}  // namespace ble

}  // namespace valence
