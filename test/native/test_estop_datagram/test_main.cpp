// test_estop_datagram -- native doctest suite for RFC-053's UDP ESTOP on the hub
// Constraints:
// - Compiles flagship_p4/src/hub/ValenceEstopDatagram.cpp itself, the one copy
//   the board and the sim build, against a real Hub with a real Client over
//   InProcessLink. Every time is a ManualClock reading; no socket exists, so
//   estopDatagramReceive() is called exactly as the §13.8 listener's poll does.
// - The delegate does what ValenceDevice does first on the board: onEstop
//   latches the REAL MotionArbiter (arbiter_rig.cpp) and canClearEstop drops
//   it, so a datagram's latch is proven to reach the emitter.
// See: flagship_p4/src/hub/ValenceEstopDatagram.h, Valence SPEC.md §5.5,
// §11.2, RFC-053; bd val-yvt

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

// Named directly so the library finder adds lib/valence and lib/geiger; it
// does not follow the relative include below.
#include "geiger/geiger.h"
#include "valence/channel/safety_events_channel.hpp"
#include "valence/client/client.hpp"
#include "valence/core/clock.hpp"
#include "valence/core/rng.hpp"
#include "valence/hub/hub.hpp"
#include "valence/transport/inprocess_binding.hpp"
#include "valence/util/byte_io.hpp"
#include "valence/wire/estop_frame.hpp"
#include "valence/wire/messages/discover.hpp"

#include "../../../flagship_p4/src/hub/ValenceDiscovery.h"
#include "../../../flagship_p4/src/hub/ValenceEstopDatagram.cpp"

#include "arbiter_rig.h"

using namespace valence;

namespace {

constexpr uint32_t kHostA = 0xC0A80105;  // 192.168.1.5
constexpr uint32_t kHostB = 0xC0A80106;

ManualClock g_clock{};

using Datagram = std::array<std::byte, kEstopFrameBytes>;

Datagram frameOf(uint8_t cause, uint8_t origin, uint16_t seq) {
    EstopFrame f;
    f.cause = cause;
    f.origin = origin;
    f.seq = seq;
    Datagram d{};
    REQUIRE(encodeEstop(f, std::span<std::byte>(d)) == kEstopFrameBytes);
    return d;
}

std::vector<std::byte> bytesOf(std::initializer_list<uint8_t> v) {
    std::vector<std::byte> out;
    for (uint8_t b : v) out.push_back(std::byte(b));
    return out;
}

// The safety STATE twin and its edge channel, nothing else.
void buildCatalog(Catalog32& c) {
    c.clear();
    c.addEntry({.id = channels::safety, .name = "safety",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::critical});
    c.addBitfieldField({.name = "word", .type = PackedFieldType::bitfield8, .unit = "flag", .scale = 1.0f},
                       {"estop", "stop", "hold", "pause"});
    c.addLayoutField({.name = "cause", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "owner_session", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "estop_seq", .type = PackedFieldType::u16, .unit = "count", .scale = 1.0f});
    c.addBitfieldField({.name = "modes", .type = PackedFieldType::bitfield8, .unit = "flag", .scale = 1.0f},
                       {"override", "home_required"});
    REQUIRE(addSafetyEventsChannel(c));
}

class ArbiterDelegate final : public HubDelegate {
public:
    int estops = 0;
    uint8_t lastCause = 0xFF;
    uint8_t lastOrigin = 0xFF;

    Result<IntentValueMap, NackCode> applyIntent(uint16_t, const IntentValueMap&, AccessLevel, bool&) override {
        return Result<IntentValueMap, NackCode>::err(NackCode::UNKNOWN_CHANNEL);
    }
    // ValenceDevice::onEstop's first act on the board is motionEstop(), the
    // arbiter's latch, which parks the emitter before it returns.
    void onEstop(uint8_t cause, uint8_t origin) override {
        ++estops;
        lastCause = cause;
        lastOrigin = origin;
        rigArbiterEstop(true);
    }
    // ValenceDevice::canClearEstop() drops the arbiter's latch with the hub's.
    bool canClearEstop() override {
        rigArbiterEstop(false);
        return true;
    }
};

class Recorder final : public ClientDelegate {
public:
    std::vector<std::byte> safety;  // the last 0x0003 snapshot
    int safetyFrames = 0;
    int safetyEdges = 0;

    void onStateChange(ClientSessionState) override {}
    void onState(uint16_t channel_id, uint16_t, std::span<const std::byte> payload) override {
        if (channel_id != channels::safety) return;
        safety.assign(payload.begin(), payload.end());
        ++safetyFrames;
    }
    void onEvent(uint16_t channel_id, std::span<const std::byte>) override {
        if (channel_id == channels::safety_events) ++safetyEdges;
    }
    void onEcho(uint16_t, const IntentValueMap&, uint16_t) override {}
    void onNack(const NackMsg&) override {}
    void onPendingDropped(uint16_t) override {}
};

// The client's end of the link, counting the hub's 0x0003 STATE frames on the
// wire: the client drops a same-seq re-broadcast before onState, so only the
// wire can show one.
class WireCounter final : public ITransport {
public:
    explicit WireCounter(ITransport& inner) : _inner(inner) {}
    bool open() override { return _inner.open(); }
    void close() override { _inner.close(); }
    bool write(std::span<const std::byte> frame) override { return _inner.write(frame); }
    std::optional<FrameBuffer> read() override {
        auto fb = _inner.read();
        if (fb) {
            const auto h = decodeFrameHeader(fb->bytes());
            if (h && h->type == uint8_t(FrameType::STATE) && h->channel == channels::safety) ++safetyOnWire;
        }
        return fb;
    }
    TransportProperties properties() const override { return _inner.properties(); }

    int safetyOnWire = 0;

private:
    ITransport& _inner;
};

// Heap, not stack: the catalog and the hub are tens of KB each.
struct Rig {
    Catalog32 catalog{};
    XorShift32 hubRng{0x5EED};
    ArbiterDelegate del{};
    std::optional<Hub> hub{};
    std::optional<InProcessLink> link{};
    std::optional<WireCounter> wire{};
    XorShift32 clientRng{0x5EEF};
    Recorder rec{};
    std::optional<Client> client{};

    Rig() {
        rigArbiterReset();
        buildCatalog(catalog);
        hub.emplace(catalog, g_clock, hubRng, del);
        estopDatagramBind(&*hub);
        estopDatagramSetEnabled(true);
        link.emplace(g_clock, hubRng);
        REQUIRE(hub->attachTransport(link->endpointA()));
        ClientIdentity id;
        id.instance_id[0] = std::byte{9};
        id.client_kind = "sim";
        id.client_name = "estop-udp";
        wire.emplace(link->endpointB());
        client.emplace(id, *wire, g_clock, clientRng, rec);
        client->addSubscriptionWish(channels::safety, 0.0f, Priority::critical);
        client->addSubscriptionWish(channels::safety_events, 0.0f, Priority::critical);
        REQUIRE(client->connect());
        step(200);
        REQUIRE(client->state() == ClientSessionState::LIVE);
        REQUIRE(rec.safetyFrames >= 1);
    }
    ~Rig() { estopDatagramBind(nullptr); }

    void step(int rounds = 16) {
        for (int i = 0; i < rounds; ++i) {
            g_clock.advanceUs(1000);
            hub->update(g_clock.nowUs());
            client->update(g_clock.nowUs());
        }
    }
    uint32_t nowMs() const { return uint32_t(g_clock.nowUs() / 1000); }
    EstopAdmit offer(std::span<const std::byte> d, uint32_t src = kHostA) {
        return estopDatagramReceive(d, src, nowMs());
    }
    bool latched() const { return !rec.safety.empty() && (uint8_t(rec.safety[0]) & 0x01) != 0; }
    uint16_t snapshotSeq() const {
        REQUIRE(rec.safety.size() >= 8);
        return getU16(std::span<const std::byte>(rec.safety).subspan(6, 2));
    }
};

}  // namespace

// ---- parse and CRC ---------------------------------------------------------------

TEST_CASE("gate: the E5 magic claims a datagram; only 12 bytes with a good CRC are admitted") {
    EstopDatagramGate gate;
    EstopFrame out{};
    const auto probe = bytesOf({'V', 'L', 'N', 'C', 1, 0x11, 0x22, 0x33, 0x44});
    CHECK(gate.admit(probe, kHostA, 0, out) == EstopAdmit::not_estop);
    CHECK(gate.admit({}, kHostA, 0, out) == EstopAdmit::not_estop);
    CHECK(gate.admit(bytesOf({0xE5, 0xE5, 0xE5}), kHostA, 0, out) == EstopAdmit::not_estop);
    CHECK(gate.admit(bytesOf({0xE5, 0xE5, 0xE5, 0x00, 0, 0, 1, 0, 0, 0, 0, 0}), kHostA, 0, out) ==
          EstopAdmit::not_estop);

    const Datagram good = frameOf(safety_causes::user, uint8_t(AccessLevel::watch), 0x1234);
    CHECK(gate.admit(std::span<const std::byte>(good).first(11), kHostA, 0, out) == EstopAdmit::malformed);
    std::vector<std::byte> longer(good.begin(), good.end());
    longer.push_back(std::byte{0});
    CHECK(gate.admit(longer, kHostA, 0, out) == EstopAdmit::malformed);
    Datagram badSeq = good;
    badSeq[6] ^= std::byte{0x01};
    CHECK(gate.admit(badSeq, kHostA, 0, out) == EstopAdmit::malformed);
    Datagram badCrc = good;
    badCrc[11] ^= std::byte{0x80};
    CHECK(gate.admit(badCrc, kHostA, 0, out) == EstopAdmit::malformed);

    REQUIRE(gate.admit(good, kHostA, 0, out) == EstopAdmit::admitted);
    CHECK(out.cause == safety_causes::user);
    CHECK(out.origin == uint8_t(AccessLevel::watch));
    CHECK(out.seq == 0x1234);
}

TEST_CASE("gate: valence-js's pinned datagram is admitted as the C++ codec reads it") {
    // clients/js/test/valence-estop-datagram.test.mjs: user, watch, seq 0xBEEF.
    const auto js = bytesOf({0xE5, 0xE5, 0xE5, 0xE5, 0x00, 0x00, 0xEF, 0xBE, 0xEA, 0xCE, 0x08, 0xAC});
    EstopDatagramGate gate;
    EstopFrame out{};
    REQUIRE(gate.admit(js, kHostA, 0, out) == EstopAdmit::admitted);
    CHECK(out.seq == 0xBEEF);
    const Datagram cpp = frameOf(safety_causes::user, uint8_t(AccessLevel::watch), 0xBEEF);
    CHECK(std::equal(cpp.begin(), cpp.end(), js.begin(), js.end()));
}

// ---- replay window and rate limit ---------------------------------------------

TEST_CASE("gate: the admitted seq is a replay for one whole repeat budget, then a new initiation") {
    EstopDatagramGate gate;
    EstopFrame out{};
    const Datagram d = frameOf(safety_causes::user, 0, 7);
    REQUIRE(gate.admit(d, kHostA, 1000, out) == EstopAdmit::admitted);
    CHECK(EstopDatagramGate::kReplayWindowMs == limits::estop_repeat_interval_ms * limits::estop_repeat_max);
    CHECK(gate.admit(d, kHostA, 1000, out) == EstopAdmit::replay);
    CHECK(gate.admit(d, kHostA, 1050, out) == EstopAdmit::replay);
    CHECK(gate.admit(d, kHostA, 1000 + EstopDatagramGate::kReplayWindowMs - 1, out) == EstopAdmit::replay);
    CHECK(gate.admit(d, kHostA, 1000 + EstopDatagramGate::kReplayWindowMs, out) == EstopAdmit::admitted);
    // The same seq from another source is that source's first initiation.
    CHECK(gate.admit(d, kHostB, 1000 + EstopDatagramGate::kReplayWindowMs, out) == EstopAdmit::admitted);
}

TEST_CASE("gate: a fresh seq from one source waits one repeat interval; another source never waits") {
    EstopDatagramGate gate;
    EstopFrame out{};
    REQUIRE(gate.admit(frameOf(0, 0, 7), kHostA, 5000, out) == EstopAdmit::admitted);
    CHECK(gate.admit(frameOf(0, 0, 8), kHostA, 5000 + EstopDatagramGate::kMinIntervalMs - 1, out) ==
          EstopAdmit::rate_limited);
    CHECK(gate.admit(frameOf(0, 0, 8), kHostB, 5001, out) == EstopAdmit::admitted);
    // The latching fob: a fresh seq every interval is admitted every interval.
    for (uint16_t k = 0; k < 5; ++k) {
        CHECK(gate.admit(frameOf(0, 0, uint16_t(8 + k)), kHostA,
                         5000 + EstopDatagramGate::kMinIntervalMs * (k + 1u), out) == EstopAdmit::admitted);
    }
}

TEST_CASE("gate: a full table evicts the source admitted longest ago") {
    EstopDatagramGate gate;
    EstopFrame out{};
    const Datagram d = frameOf(0, 0, 42);
    for (uint32_t i = 0; i < EstopDatagramGate::kSources; ++i)
        REQUIRE(gate.admit(d, 100 + i, 10 + i, out) == EstopAdmit::admitted);
    REQUIRE(gate.admit(d, 999, 30, out) == EstopAdmit::admitted);   // evicts source 100
    CHECK(gate.admit(d, 100, 31, out) == EstopAdmit::admitted);      // forgotten: new again
    CHECK(gate.admit(d, 102, 31, out) == EstopAdmit::replay);        // still remembered
}

// ---- through the hub and the arbiter ---------------------------------------------

TEST_CASE("hub: a datagram latches the hub and the arbiter; the snapshot carries its seq") {
    auto rig = std::make_unique<Rig>();
    REQUIRE_FALSE(rig->latched());
    REQUIRE_FALSE(rigArbiterLatched());

    const Datagram d = frameOf(safety_causes::user, uint8_t(AccessLevel::watch), 0x1234);
    CHECK(rig->offer(d) == EstopAdmit::admitted);
    // Synchronous on the hub task: the arbiter is latched and the emitter
    // parked before estopDatagramReceive() returns.
    CHECK(rigArbiterLatched());
    CHECK(rigArbiterParks() >= 1);
    CHECK(rig->hub->estopLatched());
    CHECK(rig->hub->estopSeq() == 0x1234);
    CHECK(rig->del.estops == 1);
    CHECK(rig->del.lastCause == safety_causes::user);
    CHECK(rig->del.lastOrigin == uint8_t(AccessLevel::watch));

    rig->step();
    CHECK(rig->latched());
    CHECK(rig->snapshotSeq() == 0x1234);
    CHECK(uint8_t(rig->rec.safety[1]) == safety_causes::user);
    CHECK(rig->rec.safetyEdges == 1);
}

TEST_CASE("hub: the same frame over WS and as a datagram leaves the same client-visible snapshot") {
    const Datagram d = frameOf(safety_causes::fault, uint8_t(AccessLevel::control), 0x0BAD);
    std::vector<std::byte> overWs;
    std::vector<std::byte> asDatagram;
    int wsEdges = 0;
    {
        auto rig = std::make_unique<Rig>();
        REQUIRE(rig->link->endpointB().write(std::span<const std::byte>(d)));
        rig->step();
        REQUIRE(rig->latched());
        overWs = rig->rec.safety;
        wsEdges = rig->rec.safetyEdges;
    }
    {
        auto rig = std::make_unique<Rig>();
        REQUIRE(rig->offer(d, kHostB) == EstopAdmit::admitted);
        rig->step();
        REQUIRE(rig->latched());
        asDatagram = rig->rec.safety;
        CHECK(rig->rec.safetyEdges == wsEdges);
    }
    CHECK(overWs == asDatagram);
}

TEST_CASE("hub: a malformed datagram is dropped and touches nothing") {
    auto rig = std::make_unique<Rig>();
    const int frames = rig->rec.safetyFrames;
    Datagram bad = frameOf(safety_causes::user, 0, 5);
    bad[8] ^= std::byte{0xFF};
    CHECK(rig->offer(bad) == EstopAdmit::malformed);
    rig->step();
    CHECK_FALSE(rig->hub->estopLatched());
    CHECK_FALSE(rigArbiterLatched());
    CHECK(rig->del.estops == 0);
    CHECK(rig->rec.safetyFrames == frames);
}

TEST_CASE("hub: after a release, the latched initiation's repeats do not re-latch; a new seq does") {
    auto rig = std::make_unique<Rig>();
    const Datagram first = frameOf(safety_causes::user, 0, 5);
    REQUIRE(rig->offer(first) == EstopAdmit::admitted);
    rig->step(100);
    REQUIRE(rig->hub->releaseEstop());
    CHECK_FALSE(rigArbiterLatched());
    rig->step(100);
    REQUIRE_FALSE(rig->latched());

    // A sender that cannot see the latch spends its whole budget (RFC-053 item 6).
    CHECK(rig->offer(first) == EstopAdmit::replay);
    rig->step();
    CHECK_FALSE(rig->hub->estopLatched());
    CHECK_FALSE(rigArbiterLatched());

    CHECK(rig->offer(frameOf(safety_causes::user, 0, 6)) == EstopAdmit::admitted);
    rig->step();
    CHECK(rig->latched());
    CHECK(rigArbiterLatched());
    CHECK(rig->snapshotSeq() == 6);
}

TEST_CASE("hub: a datagram while latched changes nothing and re-broadcasts nothing") {
    auto rig = std::make_unique<Rig>();
    REQUIRE(rig->offer(frameOf(safety_causes::user, 0, 3)) == EstopAdmit::admitted);
    rig->step();
    const int onWire = rig->wire->safetyOnWire;
    const int edges = rig->rec.safetyEdges;

    CHECK(rig->offer(frameOf(safety_causes::user, 0, 9), kHostB) == EstopAdmit::admitted);
    rig->step();
    CHECK(rig->hub->estopSeq() == 3);
    CHECK(rig->snapshotSeq() == 3);
    CHECK(rig->wire->safetyOnWire == onWire);
    CHECK(rig->rec.safetyEdges == edges);
    CHECK(rig->del.estops == 1);

    // The WS path re-broadcasts on a repeat (SPEC 11.2 loss recovery for a
    // subscribed initiator); a datagram's sender subscribes to nothing.
    const Datagram again = frameOf(safety_causes::user, 0, 3);
    REQUIRE(rig->link->endpointB().write(std::span<const std::byte>(again)));
    rig->step();
    CHECK(rig->wire->safetyOnWire == onWire + 1);
    CHECK(rig->rec.safetyEdges == edges);
}

TEST_CASE("listener: the §13.8 port's hook takes an ESTOP and answers nothing; a probe is still answered") {
    auto rig = std::make_unique<Rig>();
    DiscoveryResponder port;
    port.setIdentity("estop-test", "0.0.0", 82);
    port.setLive(0x0102030405060708ull, rig->hub->catalogEtag(), false);
    port.setDatagramHook(&estopDatagramHook);
    std::array<std::byte, kDiscoverReplyBytes> out{};

    const Datagram d = frameOf(safety_causes::user, 0, 77);
    CHECK(port.answer(d, kHostA, rig->nowMs(), out) == 0);
    CHECK(rig->hub->estopLatched());
    CHECK(rig->hub->estopSeq() == 77);
    CHECK(rigArbiterLatched());
    Datagram bad = d;
    bad[9] ^= std::byte{0x01};
    CHECK(port.answer(bad, kHostB, rig->nowMs(), out) == 0);

    DiscoverProbe p;
    p.nonce = 0xABCD;
    std::array<std::byte, kDiscoverProbeBytes> probe{};
    REQUIRE(encodeDiscoverProbe(p, probe) == kDiscoverProbeBytes);
    CHECK(port.answer(probe, kHostA, rig->nowMs(), out) == kDiscoverReplyBytes);
    // The hook's datagrams never reached the probe parse.
    CHECK(port.malformed() == 0);
}

TEST_CASE("listener: the reply's bit1 datagram_estop is the switch's live value; bit0 stays the hub's") {
    auto rig = std::make_unique<Rig>();
    DiscoveryResponder port;
    port.setIdentity("estop-test", "0.0.0", 82);
    port.setReplyFlagsHook(&estopDatagramReplyFlags);
    std::array<std::byte, kDiscoverReplyBytes> out{};
    std::array<std::byte, kDiscoverProbeBytes> probe{};
    REQUIRE(encodeDiscoverProbe(DiscoverProbe{}, probe) == kDiscoverProbeBytes);
    const auto flagsAt = [&](uint32_t src, bool pairing) {
        port.setLive(1, rig->hub->catalogEtag(), pairing);
        REQUIRE(port.answer(probe, src, rig->nowMs(), out) == kDiscoverReplyBytes);
        return decodeDiscoverReply(out).value().flags;
    };
    CHECK(flagsAt(kHostA, false) == discover_reply_flags::datagram_estop);
    CHECK(flagsAt(kHostB, true) == (discover_reply_flags::datagram_estop | discover_reply_flags::pairing_window_open));
    estopDatagramSetEnabled(false);
    CHECK(flagsAt(kHostA + 10, true) == discover_reply_flags::pairing_window_open);
    estopDatagramSetEnabled(true);
}

TEST_CASE("switch: off, or no hub bound, a valid datagram reads disabled and latches nothing") {
    auto rig = std::make_unique<Rig>();
    estopDatagramSetEnabled(false);
    CHECK_FALSE(estopDatagramEnabled());
    CHECK(rig->offer(frameOf(safety_causes::user, 0, 1)) == EstopAdmit::disabled);
    // Ours or not is decided before the switch: the listener keeps its own.
    CHECK(rig->offer(bytesOf({'V', 'L', 'N', 'C', 1, 0, 0, 0, 0})) == EstopAdmit::not_estop);
    rig->step();
    CHECK_FALSE(rig->hub->estopLatched());
    CHECK_FALSE(rigArbiterLatched());

    CHECK(estopDatagramReplyFlags() == 0);
    estopDatagramSetEnabled(true);
    CHECK(estopDatagramReplyFlags() == discover_reply_flags::datagram_estop);
    estopDatagramBind(nullptr);
    CHECK(estopDatagramReplyFlags() == 0);
    CHECK(rig->offer(frameOf(safety_causes::user, 0, 2)) == EstopAdmit::disabled);
    CHECK_FALSE(rig->hub->estopLatched());

    estopDatagramBind(&*rig->hub);
    CHECK(rig->offer(frameOf(safety_causes::user, 0, 2)) == EstopAdmit::admitted);
    CHECK(rig->hub->estopLatched());
}
