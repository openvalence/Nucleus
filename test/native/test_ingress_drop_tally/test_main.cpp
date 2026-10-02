// test_ingress_drop_tally -- native doctest suite for the hub half of 0x1111
// sync_dropped
// Constraints:
// - Hardware-free: per-slot readings are fed as literals, the shape
//   ValenceDevice::tick() reads from Hub::sessionBySlot() and
//   Hub::streamIngressCounters(). That the library counts a PAUSE drop in
//   that counter is Valence's own suite (test_valence_streamingress, SI-85).
// See: flagship_p4/src/hub/IngressDropTally.h, Valence SPEC.md §11.1,
// bd val-091.53

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstdint>
#include <limits>

// Named directly so the library finder adds lib/valence; it does not follow
// the relative include below.
#include "valence/hub/hub.hpp"

#include "../../../flagship_p4/src/hub/IngressDropTally.h"

using valence::IngressDropTally;

TEST_CASE("ID-01: deltas accumulate per session") {
    IngressDropTally t;
    t.observe(0, 11, 0);
    t.observe(1, 22, 3);
    CHECK(t.total() == 3);
    t.observe(0, 11, 5);
    t.observe(1, 22, 3);
    CHECK(t.total() == 8);
}

TEST_CASE("ID-02: a session's drops survive its departure") {
    IngressDropTally t;
    t.observe(0, 11, 7);
    t.observe(0, 0, 0);   // left: the library cleared the slot
    CHECK(t.total() == 7);
    t.observe(0, 33, 2);  // a new session reuses the slot from 0
    CHECK(t.total() == 9);
}

TEST_CASE("ID-03: a new session id in a still-occupied slot restarts its baseline") {
    IngressDropTally t;
    t.observe(2, 11, 40);
    t.observe(2, 44, 1);
    CHECK(t.total() == 41);
}

TEST_CASE("ID-04: an unoccupied slot reading adds nothing") {
    IngressDropTally t;
    t.observe(0, 0, 0);
    t.observe(0, 0, 0);
    CHECK(t.total() == 0);
}

TEST_CASE("ID-05: a counter that wraps u32 still yields its true delta") {
    IngressDropTally t;
    t.observe(0, 11, std::numeric_limits<uint32_t>::max() - 1);
    const uint32_t before = t.total();
    t.observe(0, 11, 2);   // max-1 -> 2 is 4 drops
    CHECK(uint32_t(t.total() - before) == 4);
}

TEST_CASE("ID-06: a slot past the mirrored capacity is ignored, never written") {
    IngressDropTally t;
    t.observe(valence::kIngressSlots, 11, 99);
    CHECK(t.total() == 0);
}
