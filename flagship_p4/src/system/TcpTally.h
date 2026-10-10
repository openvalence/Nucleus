#pragma once

// TcpTally -- classifies one outgoing TCP segment of the hub's WebSocket
// binding as a first send or a retransmission, and turns the running totals
// into hub-status `resent`, the resent share over a trailing window
// Constraints:
// - Hardware-free, header-only, allocation-free. ValenceTcpTally.cpp feeds
//   classify() from lwIP's segment output hook on the tcpip thread;
//   ResentWindow is the hub task's (ValenceDevice::publishHubStatus). Suite
//   test_tcp_tally drives both on the host.
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
// - Sequence and counter arithmetic is modulo 2^32 (RFC 9293 section 3.4).
// See: Valence RFC-109 (draft, revised 2026-10-10), ValenceDevice.cpp
// publishHubStatus

#include <array>
#include <cstddef>
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

// The wire value when nothing was sent in the window: no reading.
inline constexpr uint16_t kNoReading = 0xFFFF;

// Resent over sent across the last kSlots-1 seconds of totals, in hundredths
// of a percent (hub-status `resent`, scale 100). A sample less than a second
// after the newest replaces it (never the oldest), so a burst of publishes
// cannot shrink the window below kSlots-1 seconds.
class ResentWindow {
public:
    static constexpr size_t kSlots = 11;   // 10 s at the 1 Hz publish

    uint16_t push(uint32_t nowMs, uint32_t sent, uint32_t resent) {
        if (_count > 1 && uint32_t(nowMs - _slots[_newest].atMs) < 1000u) {
            _slots[_newest] = {nowMs, sent, resent};
        } else {
            _newest = (_newest + 1) % kSlots;
            _slots[_newest] = {nowMs, sent, resent};
            if (_count < kSlots) ++_count;
        }
        const Slot& a = _slots[(_newest + kSlots - (_count - 1)) % kSlots];
        const Slot& b = _slots[_newest];
        const uint32_t dSent = b.sent - a.sent;
        const uint32_t dResent = b.resent - a.resent;
        if (dSent == 0) return kNoReading;
        const uint64_t q = (uint64_t(dResent) * 10000u + dSent / 2) / dSent;
        return q > 10000u ? 10000u : uint16_t(q);
    }

private:
    struct Slot {
        uint32_t atMs = 0;
        uint32_t sent = 0;
        uint32_t resent = 0;
    };
    std::array<Slot, kSlots> _slots{};
    size_t _newest = 0;
    size_t _count = 0;
};

}  // namespace valence::tcptally
