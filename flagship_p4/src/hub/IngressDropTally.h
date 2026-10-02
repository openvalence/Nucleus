#pragma once

// IngressDropTally -- folds the hub's per-session STREAM ingress drop counters
// into one running total, the hub half of 0x1111 sync_dropped
// Constraints:
// - HARDWARE-FREE and header-only; single-task (the hub task, T5).
// - The library drops a bundle WHOLE before the delegate sees it (ungranted,
//   malformed, over rate, source conflict, pre-READY, and PAUSE under SPEC
//   §11.1) and counts it per session in Hub::streamIngressCounters(). That
//   counter dies with its session, so the total is folded here from per-slot
//   deltas, one observe() per slot per hub tick. Drops a session makes in the
//   tick it leaves are lost: an undercount of at most one tick.
// - A new session id in a slot restarts that slot's baseline at 0.
// - kIngressSlots mirrors Hub::kSlotCapacity (private, kHubMaxSessions + 1).
//   A slot index at or past it is ignored, never written out of bounds.
// See: Valence SPEC.md §9.2, §11.1; RFC-074 clause 4; ValenceDevice.cpp
// (publishMotionDiag), bd val-091.53

#include <array>
#include <cstddef>
#include <cstdint>

#include "valence/hub/hub.hpp"

namespace valence {

inline constexpr size_t kIngressSlots = kHubMaxSessions + 1;

class IngressDropTally {
public:
    // sessionId 0 = the slot is unoccupied. `dropped` is the slot's counter
    // as read now; it only grows within one session (u32 modular).
    void observe(size_t slot, uint32_t sessionId, uint32_t dropped) {
        if (slot >= _slots.size()) return;
        Slot& s = _slots[slot];
        if (sessionId != s.sessionId) {
            s.sessionId = sessionId;
            s.seen = 0;
        }
        if (sessionId == 0) return;
        _total += dropped - s.seen;
        s.seen = dropped;
    }

    // Bundles the hub dropped at ingress since boot (u32, wraps).
    uint32_t total() const { return _total; }

private:
    struct Slot {
        uint32_t sessionId = 0;
        uint32_t seen = 0;
    };
    std::array<Slot, kIngressSlots> _slots{};
    uint32_t _total = 0;
};

}  // namespace valence
