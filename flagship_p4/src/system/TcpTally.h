#pragma once

// TcpTally -- classifies one outgoing TCP segment of the hub's WebSocket
// binding as a first send or a retransmission: hub-status tcp_sent and
// tcp_resent
// Constraints:
// - Hardware-free, header-only, allocation-free. ValenceTcpTally.cpp feeds it
//   from lwIP's segment output hook on the tcpip thread; suite test_tcp_tally
//   drives it on the host.
// - A segment counts only when it occupies sequence space (payload, SYN or
//   FIN). Pure ACKs, RSTs and keepalives never do, so they are never counted:
//   their loss is invisible to the sender.
// - Retransmission = the segment starts before snd_nxt. lwIP advances snd_nxt
//   only forward and neither the RTO nor the fast path rewinds it, so this
//   catches every retransmit path, the RTO one lwIP's MIB2 tcpretranssegs
//   misses included.
// - A pcb in the persist state (zero window) sends probes, never counted.
//   lwIP advances snd_nxt past the probed byte, so the first full segment
//   after the window opens starts below it and counts as one resend: at most
//   one per zero-window episode.
// - Sequence comparison is modulo 2^32 (RFC 9293 section 3.4).
// See: Valence RFC-109 (draft, revised 2026-10-10), ValenceDevice.cpp
// publishHubStatus

#include <cstdint>

namespace valence::tcptally {

enum class Kind : uint8_t { none, sent, resent };

struct Segment {
    bool listening = false;      // the pcb is a listen pcb (no send state)
    bool persisting = false;     // the pcb is probing a zero window
    uint16_t localPort = 0;
    uint32_t seqno = 0;          // host order
    uint32_t sndNxt = 0;         // the pcb's snd_nxt before this output
    uint32_t seqLen = 0;         // payload bytes, +1 for SYN, +1 for FIN
};

// port 0 = no binding watched: nothing counts.
constexpr Kind classify(const Segment& s, uint16_t port) {
    if (port == 0 || s.listening || s.persisting || s.localPort != port || s.seqLen == 0) return Kind::none;
    return int32_t(s.seqno - s.sndNxt) < 0 ? Kind::resent : Kind::sent;
}

}  // namespace valence::tcptally
