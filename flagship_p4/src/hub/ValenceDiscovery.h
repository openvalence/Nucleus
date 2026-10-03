#pragma once

// ValenceDiscovery -- the SPEC §13.8 UDP discovery responder: a DISCOVER_PROBE
// on registry udp_discovery.port gets one unicast DISCOVER_REPLY built from the
// hub's live state
// Constraints:
// - HUB TASK ONLY (T5). poll() reads the Hub (identity, catalog etag, pairing
//   window) on the task that owns it, from a NON-BLOCKING socket, once per
//   tick: no task, no callback, nothing shared across tasks. The one blocking
//   call is a reply's sendto, the same bounded class as the WS port's writes.
// - A probe storm buys a bounded tick: poll() drains at most
//   kDiscoveryDrainPerPoll datagrams; DiscoveryLimiter answers a source once
//   per udp_discovery.reply_rate_limit_per_source_s and never evicts a source
//   still inside its window, so at most DiscoveryLimiter::kSources replies
//   leave per window in all.
// - A datagram that is not a probe (length, magic) is dropped silently and
//   counted: §13.8 gives a probe no error path. A probe of any proto_ver is
//   answered; the reply's own proto_ver is how a client judges the hub.
// - ONE LISTENER PER PORT: a unicast datagram reaches one socket only, and
//   RFC-053 puts the ESTOP datagram on this port too. Every datagram goes to
//   the datagram hook first; one it consumes is never answered.
// - ONE FILE, TWO HOSTS: the board and sim/valencesim compile
//   ValenceDiscovery.cpp verbatim (lwIP sockets on the board, Winsock in the
//   sim). Everything in this header is hardware-free; suite
//   test_valence_discovery runs it natively.
// See: Valence SPEC.md §13.7, §13.8; valence/wire/messages/discover.hpp

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "valence/wire/messages/discover.hpp"

namespace valence {

class Hub;

// Datagrams one poll() reads at most. The rest wait for the next tick, and
// past the stack's own receive queue they are dropped by the stack.
inline constexpr size_t kDiscoveryDrainPerPoll = 4;

// ---- per-source limiter ------------------------------------------------------

class DiscoveryLimiter {
public:
    static constexpr size_t kSources = 8;
    static constexpr uint32_t kWindowMs = udp_discovery::reply_rate_limit_per_source_s * 1000u;

    // True when `ipv4` may be answered at `nowMs`, and records that it was.
    // Wrap-safe for gaps under 2^31 ms.
    bool admit(uint32_t ipv4, uint32_t nowMs) {
        Slot* open = nullptr;
        for (Slot& s : _slots) {
            const bool expired = uint32_t(nowMs - s.lastMs) >= kWindowMs;
            if (s.used && s.ipv4 == ipv4) {
                if (!expired) return false;
                s.lastMs = nowMs;
                return true;
            }
            if (open == nullptr && (!s.used || expired)) open = &s;
        }
        if (open == nullptr) return false;
        open->ipv4 = ipv4;
        open->lastMs = nowMs;
        open->used = true;
        return true;
    }

private:
    struct Slot {
        uint32_t ipv4 = 0;
        uint32_t lastMs = 0;
        bool used = false;
    };
    std::array<Slot, kSources> _slots{};
};

// ---- the responder core --------------------------------------------------------

class DiscoveryResponder {
public:
    // Offered every datagram before the probe parse, on the hub task; true =
    // consumed (RFC-053: an ESTOP frame), so no reply. `srcIpv4` is host order.
    using DatagramHook = bool (*)(std::span<const std::byte> datagram, uint32_t srcIpv4, uint32_t nowMs);
    void setDatagramHook(DatagramHook hook) { _hook = hook; }

    // What only the composition root knows. Copied into the reply template,
    // truncated byte-wise to str32 and str16.
    void setIdentity(std::string_view hubName, std::string_view fwVersion, uint16_t wsPort) {
        setDiscoverString(_reply.hub_name, hubName);
        setDiscoverString(_reply.fw_version, fwVersion);
        _reply.ws_port = wsPort;
    }

    // What the Hub owns, read on the hub task before answer().
    void setLive(uint64_t hubInstanceId, std::span<const std::byte> catalogEtag, bool pairingWindowOpen) {
        _reply.hub_instance_id = hubInstanceId;
        const size_t n = catalogEtag.size() < _reply.catalog_etag.size() ? catalogEtag.size()
                                                                          : _reply.catalog_etag.size();
        _reply.catalog_etag = {};
        for (size_t i = 0; i < n; ++i) _reply.catalog_etag[i] = catalogEtag[i];
        _reply.flags = pairingWindowOpen ? kDiscoverFlagPairingWindowOpen : 0;
    }

    // One datagram from `srcIpv4` (host order): the reply written to `out`
    // and its length, or 0 when nothing is sent.
    size_t answer(std::span<const std::byte> datagram, uint32_t srcIpv4, uint32_t nowMs,
                  std::span<std::byte> out) {
        if (_hook != nullptr && _hook(datagram, srcIpv4, nowMs)) return 0;
        const auto probe = decodeDiscoverProbe(datagram);
        if (!probe.isOk()) {
            ++_malformed;
            return 0;
        }
        if (!_limiter.admit(srcIpv4, nowMs)) {
            ++_throttled;
            return 0;
        }
        _reply.nonce = probe.value().nonce;
        const size_t n = encodeDiscoverReply(_reply, out);
        if (n != 0) ++_replies;
        return n;
    }

    uint32_t replies() const { return _replies; }
    uint32_t malformed() const { return _malformed; }
    uint32_t throttled() const { return _throttled; }

private:
    DatagramHook _hook = nullptr;
    DiscoverReply _reply{};
    DiscoveryLimiter _limiter{};
    uint32_t _replies = 0;
    uint32_t _malformed = 0;
    uint32_t _throttled = 0;
};

// ---- the socket host -------------------------------------------------------------

class ValenceDiscoveryPort {
public:
    ValenceDiscoveryPort() = default;
    ValenceDiscoveryPort(const ValenceDiscoveryPort&) = delete;
    ValenceDiscoveryPort& operator=(const ValenceDiscoveryPort&) = delete;
    ~ValenceDiscoveryPort() { end(); }

    // Binds a non-blocking UDP socket on 0.0.0.0:`port`. false = no discovery
    // this run (port taken, no socket); the hub runs regardless, and a typed
    // address still reaches it (§13.7). The views are copied before return.
    bool begin(uint16_t port, std::string_view hubName, std::string_view fwVersion, uint16_t wsPort);
    // Composition root, before the first poll().
    void setDatagramHook(DiscoveryResponder::DatagramHook hook) { _responder.setDatagramHook(hook); }
    // Hub task, once per tick.
    void poll(const Hub& hub, uint32_t nowMs);
    void end();

    bool up() const { return _sock != kNoSocket; }
    const DiscoveryResponder& responder() const { return _responder; }

private:
    static constexpr intptr_t kNoSocket = -1;
    intptr_t _sock = kNoSocket;  // an lwIP fd or a Winsock SOCKET
    DiscoveryResponder _responder{};
};

}  // namespace valence
