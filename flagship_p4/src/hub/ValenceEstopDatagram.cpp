// ValenceEstopDatagram -- the hub-task half of RFC-053's UDP ESTOP. See
// ValenceEstopDatagram.h for the task, listener and latch rules.
// Constraints:
// - Static state is three words and one 8-slot table (EstopDatagramGate,
//   kSources x 16 B), internal RAM, touched by the hub task only.
// - A replay is the normal traffic of every honest initiation (19 repeats per
//   press) and is never logged; the other drops log at most once per 10 s.

#include "ValenceEstopDatagram.h"

#include "geiger/geiger.h"
#include "valence/hub/hub.hpp"

namespace valence {

namespace {

constexpr const char* kTag = "estop-udp";

Hub* g_hub = nullptr;
EstopDatagramGate g_gate{};
bool g_enabled = true;

// Octet `n` (0 = first) of a host-order IPv4, for the log line only.
[[maybe_unused]] unsigned octet(uint32_t ipv4, int n) { return unsigned((ipv4 >> (24 - 8 * n)) & 0xFF); }

}  // namespace

const char* estopAdmitName(EstopAdmit v) {
    switch (v) {
    case EstopAdmit::not_estop: return "not an ESTOP";
    case EstopAdmit::admitted: return "admitted";
    case EstopAdmit::malformed: return "malformed";
    case EstopAdmit::replay: return "replay";
    case EstopAdmit::rate_limited: return "rate limited";
    case EstopAdmit::disabled: return "disabled";
    }
    return "?";
}

void estopDatagramBind(Hub* hub) {
    g_hub = hub;
    g_gate = EstopDatagramGate{};
}

void estopDatagramSetEnabled(bool on) { g_enabled = on; }

bool estopDatagramEnabled() { return g_enabled; }

uint8_t estopDatagramReplyFlags() {
    return NUCLEUS_ESTOP_DATAGRAM && g_enabled && g_hub != nullptr ? discover_reply_flags::datagram_estop : 0;
}

EstopAdmit estopDatagramReceive(std::span<const std::byte> datagram, uint32_t srcIpv4, uint32_t nowMs) {
#if NUCLEUS_ESTOP_DATAGRAM
    if (!isEstopCandidate(datagram)) return EstopAdmit::not_estop;
    EstopAdmit v = EstopAdmit::disabled;
    EstopFrame f{};
    if (g_enabled && g_hub != nullptr) v = g_gate.admit(datagram, srcIpv4, nowMs, f);
    if (v == EstopAdmit::admitted) {
        if (!g_hub->estopLatched()) {
            g_hub->latchEstop(f.cause, f.origin, f.seq);
            GLOGW(kTag, "ESTOP latched by a datagram from %u.%u.%u.%u: cause %u, origin %u, seq %u",
                  octet(srcIpv4, 0), octet(srcIpv4, 1), octet(srcIpv4, 2), octet(srcIpv4, 3),
                  unsigned(f.cause), unsigned(f.origin), unsigned(f.seq));
        }
    } else if (v != EstopAdmit::replay) {
        GLOGW_EVERY_MS(10000, kTag, "datagram from %u.%u.%u.%u dropped: %s", octet(srcIpv4, 0),
                       octet(srcIpv4, 1), octet(srcIpv4, 2), octet(srcIpv4, 3), estopAdmitName(v));
    }
    return v;
#else
    (void)datagram;
    (void)srcIpv4;
    (void)nowMs;
    return EstopAdmit::not_estop;
#endif
}

bool estopDatagramHook(std::span<const std::byte> datagram, uint32_t srcIpv4, uint32_t nowMs) {
    return estopDatagramReceive(datagram, srcIpv4, nowMs) != EstopAdmit::not_estop;
}

}  // namespace valence
