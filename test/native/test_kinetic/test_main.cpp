// test_kinetic: Kinetic's own suite, compiled from the sibling checkout
// kinetic.pin names. Its cases live there and nowhere else; never copy them.
// The one case added here is the tie Kinetic cannot make, because it stays
// protocol-free: its pinned dwell span must be the Valence registry's.
#include "../../../../Kinetic/tests/test_kinetic.cpp"

#include "valence/generated/registry_constants.hpp"

TEST_CASE("Kinetic's pinned dwell span is the registry's segment_dwell_span") {
    CHECK(kSegmentDwellSpan == valence::limits::segment_dwell_span);
}
