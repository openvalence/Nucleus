// ValenceTcpTally -- the board's TCP segment tally on the WebSocket binding's
// port: lwIP's output hook feeds TcpTally, the hub task reads the totals
// Constraints:
// - valence_tcp_out_hook runs on the tcpip thread (ValenceTcpHook.h says what
//   it must never do); deviceLinkTcp() runs on the hub task. One writer: a
//   resend bumps sent, then resent with release; the reader loads resent with
//   acquire, then sent, so a snapshot never shows resent above sent.
// - BSS: two u32 counters and one u16 port. No task, no stack of its own.
// - Counts every connection on the watched port, all sessions together, and
//   nothing before tcpTallyWatch() names the port.
// See: TcpTally.h, ValenceDevice.h deviceLinkTcp(), Valence RFC-109 (draft)

#include "system/ValenceTcpTally.h"

#include <atomic>

#include <lwip/priv/tcp_priv.h>
#include <lwip/tcp.h>

#include "hub/ValenceDevice.h"
#include "system/TcpTally.h"
#include "system/ValenceTcpHook.h"

namespace {

std::atomic<uint16_t> g_port{0};
std::atomic<uint32_t> g_sent{0};
std::atomic<uint32_t> g_resent{0};

}  // namespace

extern "C" u32_t* valence_tcp_out_hook(struct pbuf* p, struct tcp_hdr* hdr, const struct tcp_pcb* pcb, u32_t* opts) {
    if (pcb == nullptr || p == nullptr || hdr == nullptr) return opts;
    using namespace valence::tcptally;
    const uint8_t flags = TCPH_FLAGS(hdr);
    Segment s;
    s.listening = pcb->state == LISTEN;
    s.persisting = !s.listening && pcb->persist_backoff != 0;
    s.localPort = pcb->local_port;
    s.seqno = lwip_ntohl(hdr->seqno);
    s.sndNxt = s.listening ? 0 : pcb->snd_nxt;
    s.seqLen = uint32_t(p->tot_len) - TCPH_HDRLEN_BYTES(hdr) + ((flags & TCP_SYN) ? 1u : 0u) +
               ((flags & TCP_FIN) ? 1u : 0u);
    switch (classify(s, g_port.load(std::memory_order_relaxed))) {
        case Kind::sent: g_sent.fetch_add(1, std::memory_order_relaxed); break;
        case Kind::resent:
            g_sent.fetch_add(1, std::memory_order_relaxed);
            g_resent.fetch_add(1, std::memory_order_release);
            break;
        case Kind::none: break;
    }
    return opts;
}

namespace valence {

void tcpTallyWatch(uint16_t port) { g_port.store(port, std::memory_order_relaxed); }

LinkTcp deviceLinkTcp() {
    const uint32_t resent = g_resent.load(std::memory_order_acquire);
    return {g_sent.load(std::memory_order_relaxed), resent};
}

}  // namespace valence
