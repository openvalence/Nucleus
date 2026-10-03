// test_valence_discovery -- native doctest suite for the §13.8 responder core
// Constraints:
// - Hardware-free and deterministic: every time is a literal u32 ms and no
//   socket exists. Compiles flagship_p4/src/hub/ValenceDiscovery.h itself, the
//   one copy the board and the sim both build.
// - The reply is checked byte-for-byte against discover.hpp's U-02 vector, so
//   the responder cannot drift from the codec's golden bytes.
// See: flagship_p4/src/hub/ValenceDiscovery.h, bd val-qxs

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

// Named directly so the library finder adds lib/valence; it does not follow
// the relative include below.
#include "valence/wire/messages/discover.hpp"

#include "../../../flagship_p4/src/hub/ValenceDiscovery.h"

using valence::DiscoverReply;
using valence::DiscoveryLimiter;
using valence::DiscoveryResponder;
namespace vectors = valence::discover_vectors;

namespace {

constexpr uint32_t kHostA = 0xC0A80105;  // 192.168.1.5
constexpr uint32_t kHostB = 0xC0A80106;
constexpr std::array<std::byte, 8> kEtag = {std::byte{0x8C}, std::byte{0x5D}, std::byte{0x68}, std::byte{0xF4},
                                            std::byte{0x1A}, std::byte{0xD0}, std::byte{0x32}, std::byte{0x5E}};

template <size_t N>
std::vector<std::byte> bytesOf(const std::array<uint8_t, N>& a) {
    std::vector<std::byte> v(N);
    for (size_t i = 0; i < N; ++i) v[i] = std::byte(a[i]);
    return v;
}

// A responder holding exactly U-02's inputs.
DiscoveryResponder u02Responder() {
    DiscoveryResponder r;
    r.setIdentity("valence-fixture", "1.0.0", 82);
    r.setLive(0x0102030405060708ull, kEtag, true);
    return r;
}

size_t ask(DiscoveryResponder& r, std::span<const std::byte> datagram, uint32_t src, uint32_t nowMs) {
    std::array<std::byte, valence::kDiscoverReplyBytes> out{};
    return r.answer(datagram, src, nowMs, out);
}

}  // namespace

TEST_CASE("probe parse: a valid probe gets the 76-byte U-02 reply, nonce echoed") {
    DiscoveryResponder r = u02Responder();
    std::array<std::byte, valence::kDiscoverReplyBytes> out{};
    const auto probe = bytesOf(vectors::kU01Probe);
    REQUIRE(r.answer(probe, kHostA, 5000, out) == valence::kDiscoverReplyBytes);
    CHECK(std::vector<std::byte>(out.begin(), out.end()) == bytesOf(vectors::kU02Reply));
    CHECK(r.replies() == 1);
}

TEST_CASE("reply encode: identity, live state and the pairing bit") {
    DiscoveryResponder r;
    r.setIdentity("nucleus-p4", "0.1.7-p4hub", 82);
    r.setLive(0xFEEDC0DE00000001ull, kEtag, false);
    std::array<std::byte, valence::kDiscoverProbeBytes> probe{};
    REQUIRE(valence::encodeDiscoverProbe({1, 0xDEADBEEFu}, probe) == probe.size());
    std::array<std::byte, valence::kDiscoverReplyBytes> out{};
    REQUIRE(r.answer(probe, kHostA, 1000, out) == out.size());
    const auto reply = valence::decodeDiscoverReply(out);
    REQUIRE(reply.isOk());
    const DiscoverReply& d = reply.value();
    CHECK(d.nonce == 0xDEADBEEFu);
    CHECK(valence::discoverString(d.hub_name) == "nucleus-p4");
    CHECK(valence::discoverString(d.fw_version) == "0.1.7-p4hub");
    CHECK(d.hub_instance_id == 0xFEEDC0DE00000001ull);
    CHECK(d.proto_ver == valence::kProtocolVersion);
    CHECK(d.ws_port == 82);
    CHECK(d.catalog_etag == kEtag);
    CHECK(d.flags == 0);

    r.setLive(0xFEEDC0DE00000001ull, kEtag, true);
    REQUIRE(r.answer(probe, kHostB, 1000, out) == out.size());
    CHECK(valence::decodeDiscoverReply(out).value().flags == valence::kDiscoverFlagPairingWindowOpen);
}

TEST_CASE("malformed ignored: short, long and foreign datagrams get nothing and cost no slot") {
    DiscoveryResponder r = u02Responder();
    auto probe = bytesOf(vectors::kU01Probe);
    CHECK(ask(r, std::span(probe).first(8), kHostA, 1000) == 0);
    auto longer = probe;
    longer.push_back(std::byte{0});
    CHECK(ask(r, longer, kHostA, 1000) == 0);
    auto foreign = probe;
    foreign[0] = std::byte{'X'};
    CHECK(ask(r, foreign, kHostA, 1000) == 0);
    CHECK(ask(r, bytesOf(vectors::kU02Reply), kHostA, 1000) == 0);
    CHECK(r.malformed() == 4);
    CHECK(r.replies() == 0);
    // None of the four took the source's slot: its first real probe is answered.
    CHECK(ask(r, probe, kHostA, 1000) == valence::kDiscoverReplyBytes);
}

namespace {
uint32_t g_hookCalls = 0;
uint32_t g_hookSource = 0;
// Consumes what starts with the four-0xE5 ESTOP magic, as RFC-053's gate does.
bool estopLikeHook(std::span<const std::byte> d, uint32_t src, uint32_t) {
    ++g_hookCalls;
    g_hookSource = src;
    return d.size() >= 4 && d[0] == std::byte{0xE5} && d[1] == std::byte{0xE5} && d[2] == std::byte{0xE5} &&
           d[3] == std::byte{0xE5};
}
}  // namespace

TEST_CASE("datagram hook: offered every datagram first; what it consumes is never answered") {
    DiscoveryResponder r = u02Responder();
    r.setDatagramHook(&estopLikeHook);
    std::vector<std::byte> estop(12, std::byte{0xE5});
    CHECK(ask(r, estop, kHostA, 1000) == 0);
    CHECK(g_hookCalls == 1);
    CHECK(g_hookSource == kHostA);
    CHECK(r.malformed() == 0);  // consumed, not counted as a bad probe
    // The consumed datagram took no limiter slot: the source's probe is answered.
    CHECK(ask(r, bytesOf(vectors::kU01Probe), kHostA, 1000) == valence::kDiscoverReplyBytes);
    CHECK(g_hookCalls == 2);
}

TEST_CASE("rate limit: one reply per source per window, sources independent") {
    DiscoveryResponder r = u02Responder();
    const auto probe = bytesOf(vectors::kU01Probe);
    CHECK(ask(r, probe, kHostA, 10000) != 0);
    CHECK(ask(r, probe, kHostA, 10000 + DiscoveryLimiter::kWindowMs - 1) == 0);
    CHECK(ask(r, probe, kHostB, 10500) != 0);
    CHECK(ask(r, probe, kHostA, 10000 + DiscoveryLimiter::kWindowMs) != 0);
    CHECK(r.throttled() == 1);
    CHECK(r.replies() == 3);
    CHECK(DiscoveryLimiter::kWindowMs == 1000);
}

TEST_CASE("rate limit: a many-source storm is capped at kSources replies per window") {
    DiscoveryLimiter lim;
    for (uint32_t i = 0; i < DiscoveryLimiter::kSources; ++i) CHECK(lim.admit(0x0A000001 + i, 2000));
    CHECK_FALSE(lim.admit(0x0A0000FF, 2500));   // every slot still inside its window
    CHECK(lim.admit(0x0A0000FF, 3000));         // the oldest slot expired and is reused
    CHECK_FALSE(lim.admit(0x0A0000FF, 3001));   // and the new source is now limited itself
}

TEST_CASE("rate limit: the u32 millisecond clock wraps without a stuck source") {
    DiscoveryLimiter lim;
    CHECK(lim.admit(kHostA, 0xFFFFFF00u));
    CHECK_FALSE(lim.admit(kHostA, 0x00000010u));  // 272 ms later, across the wrap
    CHECK(lim.admit(kHostA, 0x00000300u));        // 1,024 ms later
}
