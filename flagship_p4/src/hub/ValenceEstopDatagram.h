#pragma once

// ValenceEstopDatagram -- RFC-053's UDP ESTOP: a §5.5 frame that arrives as a
// datagram on the §13.8 port latches the hub exactly as a raw 0xE5 over WS does
// Constraints:
// - HUB TASK ONLY (T5). The §13.8 listener (ValenceDiscoveryPort) offers
//   every datagram through estopDatagramHook() from its poll on the hub task;
//   the gate, the bound Hub and the switch are hub-task state and take no lock.
// - ONE LISTENER PER PORT. RFC-053 puts ESTOP on udp_discovery.port, and a
//   unicast datagram reaches one socket only, so this file opens no socket:
//   the composition root sets the hook on the discovery port, which parses a
//   datagram itself only when the hook declines it. The listener's receive
//   buffer must hold kEstopFrameBytes plus one, so an oversize datagram shows
//   as one.
// - THE HUB NEVER ANSWERS A DATAGRAM, admitted or not: no reply, no ack, no
//   amplification (RFC-053 item 6, option i).
// - A latch goes through Hub::latchEstop(), the function the raw frame and the
//   `estop` op share (SPEC 11.2): the frame's seq becomes estop_seq on the
//   transition, and the snapshot and the edge are the WS path's. A datagram
//   that finds ESTOP latched changes nothing and re-broadcasts nothing.
// - Hardware-free: the board, sim/valencesim and suite test_estop_datagram
//   compile the .cpp verbatim.
// - NUCLEUS_ESTOP_DATAGRAM=0 compiles the path out (RFC-053 item 3's build
//   flag): every datagram reads not_estop.
// See: Valence spec/RFC-QUEUE.md RFC-053; SPEC.md §5.5, §11.2, §13.8; bd rfc-kk2

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "valence/generated/registry_constants.hpp"
#include "valence/wire/estop_frame.hpp"

#ifndef NUCLEUS_ESTOP_DATAGRAM
#define NUCLEUS_ESTOP_DATAGRAM 1
#endif

namespace valence {

class Hub;

enum class EstopAdmit : uint8_t {
    not_estop,     // no E5 magic: the listener's own datagram
    admitted,      // a new initiation from this source
    malformed,     // E5 magic but not 12 bytes, or a bad CRC: dropped silently
    replay,        // this source's last admitted seq, inside the replay window
    rate_limited,  // a new seq from this source inside one repeat interval
    disabled,      // the switch is off or no hub is bound: consumed, never latched
};

const char* estopAdmitName(EstopAdmit v);

// Four 0xE5 bytes at offset 0: the §5.5 magic, so the datagram is ours.
inline bool isEstopCandidate(std::span<const std::byte> datagram) {
    return datagram.size() >= 4 && datagram[0] == kEstopMagicByte && datagram[1] == kEstopMagicByte &&
           datagram[2] == kEstopMagicByte && datagram[3] == kEstopMagicByte;
}

// ---- per-source admission ------------------------------------------------------
// One gate per binding, owned by the task that receives on it. `source` is
// the binding's own sender address: an IPv4 here, a MAC on the ESP-NOW spoke.

class EstopDatagramGate {
public:
    static constexpr size_t kSources = 8;
    // One initiation's whole §11.2 repeat budget: every repeat of a seq lands
    // inside it, so a release is never undone by its initiation's stragglers.
    static constexpr uint32_t kReplayWindowMs = limits::estop_repeat_interval_ms * limits::estop_repeat_max;
    // A conforming source starts at most one initiation per repeat interval
    // (the latching fob, RFC-053 item 5); anything faster is a spray (item 4).
    static constexpr uint32_t kMinIntervalMs = limits::estop_repeat_interval_ms;

    // `out` is written only on admitted. A full table evicts the source
    // admitted longest ago. Wrap-safe for gaps under 2^31 ms.
    EstopAdmit admit(std::span<const std::byte> datagram, uint64_t source, uint32_t nowMs, EstopFrame& out) {
        if (!isEstopCandidate(datagram)) return EstopAdmit::not_estop;
        if (datagram.size() != kEstopFrameBytes) return EstopAdmit::malformed;
        const auto frame = decodeEstop(datagram);
        if (!frame) return EstopAdmit::malformed;
        const uint16_t seq = frame.value().seq;

        Slot* mine = nullptr;
        Slot* victim = nullptr;
        for (Slot& s : _slots) {
            if (s.used && s.source == source) {
                mine = &s;
                break;
            }
            if (victim == nullptr || (victim->used && (!s.used || age(s, nowMs) > age(*victim, nowMs)))) victim = &s;
        }
        if (mine != nullptr) {
            const uint32_t sinceMs = age(*mine, nowMs);
            if (seq == mine->seq && sinceMs < kReplayWindowMs) return EstopAdmit::replay;
            if (sinceMs < kMinIntervalMs) return EstopAdmit::rate_limited;
        } else {
            mine = victim;
        }
        mine->source = source;
        mine->admittedMs = nowMs;
        mine->seq = seq;
        mine->used = true;
        out = frame.value();
        return EstopAdmit::admitted;
    }

private:
    struct Slot {
        uint64_t source = 0;
        uint32_t admittedMs = 0;
        uint16_t seq = 0;
        bool used = false;
    };
    static uint32_t age(const Slot& s, uint32_t nowMs) { return uint32_t(nowMs - s.admittedMs); }

    std::array<Slot, kSources> _slots{};
};

// ---- the door ---------------------------------------------------------------------

// Composition root, before the §13.8 listener's first poll: the hub a datagram
// latches. Binding resets the gate; nullptr unbinds (datagrams read disabled).
void estopDatagramBind(Hub* hub);

// RFC-053 item 3's runtime switch, on from boot. machine-modes
// datagram_estop drives it (ValenceDevice: the stored value at adoption,
// before the hub task starts, then every modes-set write). Hub task.
void estopDatagramSetEnabled(bool on);
bool estopDatagramEnabled();

// RFC-053 item 2b: the DISCOVER_REPLY flags this file owns, datagram_estop
// while a datagram would latch right now (compiled in, switch on, hub bound).
// Hub task; the §13.8 port's reply-flags hook. TODO(rfc-kk2): the ESP-NOW
// BEACON's bit1 mirrors it when that binding lands.
uint8_t estopDatagramReplyFlags();

// One datagram from the §13.8 port, hub task. `srcIpv4` in host order is a
// key and a log field only; nothing is sent back to it. not_estop is the only
// verdict that leaves the datagram to the listener.
EstopAdmit estopDatagramReceive(std::span<const std::byte> datagram, uint32_t srcIpv4, uint32_t nowMs);

// ValenceDiscoveryPort's datagram hook (DiscoveryResponder::DatagramHook):
// true = ours, consumed, never answered.
bool estopDatagramHook(std::span<const std::byte> datagram, uint32_t srcIpv4, uint32_t nowMs);

}  // namespace valence
