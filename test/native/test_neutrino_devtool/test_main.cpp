// test_neutrino_devtool -- native doctest suite for the dev tool channel's
// crossings, its text-in / JSON-out codec, and the per-client fault link
// Constraints:
// - Host-only: NeutrinoDevtool.h and NeutrinoLink.h alone, no IXWebSocket and
//   no SimCore. The concurrency cases run real threads against the real
//   queue and mailbox; the link runs on a synthetic microsecond clock.
// See: sim/neutrino/src/NeutrinoDevtool.h, sim/neutrino/src/NeutrinoLink.h,
// bd val-sf7.7

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <atomic>
#include <cstdint>
#include <deque>
#include <string>
#include <thread>
#include <vector>

// Named here so the library finder resolves lib/valence for the sim headers.
#include "valence/transport/transport.hpp"

#include "../../../sim/neutrino/src/NeutrinoDevtool.h"
#include "../../../sim/neutrino/src/NeutrinoLink.h"

using namespace valence;
using devtool::Input;

namespace {

// ---- a transport double: frames in, frames out, nothing else -----------------

class FakeTransport final : public ITransport {
public:
    bool open() override { return true; }
    void close() override { closed = true; }
    bool write(std::span<const std::byte> f) override {
        if (refuse) return false;
        sent.push_back(FrameBuffer::from(f));
        return true;
    }
    std::optional<FrameBuffer> read() override {
        if (inbox.empty()) return std::nullopt;
        FrameBuffer f = inbox.front();
        inbox.pop_front();
        return f;
    }
    TransportProperties properties() const override { return {}; }

    std::deque<FrameBuffer> inbox;
    std::vector<FrameBuffer> sent;
    bool refuse = false;
    bool closed = false;
};

// A header-framed frame of `type` whose seq field carries `tag`.
FrameBuffer frame(FrameType type, uint16_t tag) {
    std::array<std::byte, kHeaderBytes> b{};
    FrameHeader h;
    h.type = uint8_t(type);
    h.seq = tag;
    encodeFrameHeader(h, b);
    return FrameBuffer::from(b);
}

uint16_t tagOf(const FrameBuffer& f) { return f.header()->seq; }

// Drains what the link hands the hub at `nowUs`.
std::vector<uint16_t> readAll(NeutrinoLink& l, uint64_t nowUs) {
    l.tick(nowUs);
    std::vector<uint16_t> out;
    while (auto f = l.read()) out.push_back(tagOf(*f));
    return out;
}

}  // namespace

// ---- the input queue ------------------------------------------------------------

TEST_CASE("input queue: FIFO, bounded, a full queue drops and counts") {
    devtool::InputQueue<int, 4> q;
    for (int i = 0; i < 4; ++i) CHECK(q.push(i));
    CHECK_FALSE(q.push(99));
    CHECK(q.dropped() == 1);
    std::array<int, 4> out{};
    REQUIRE(q.drain(out) == 4);
    CHECK(out == std::array<int, 4>{0, 1, 2, 3});
    CHECK(q.drain(out) == 0);
    CHECK(q.push(5));
    REQUIRE(q.drain(out) == 1);
    CHECK(out[0] == 5);
}

TEST_CASE("input queue: two pushers and a draining hub lose nothing and keep each pusher's order") {
    devtool::InputQueue<uint32_t, 64> q;
    constexpr uint32_t kEach = 20000;
    auto pusher = [&q](uint32_t tag) {
        for (uint32_t i = 0; i < kEach;) {
            if (q.push((tag << 24) | i)) ++i;
            else std::this_thread::yield();
        }
    };
    std::thread a(pusher, 1u), b(pusher, 2u);
    std::array<uint32_t, 64> out{};
    std::array<uint32_t, 3> next{};
    uint32_t got = 0;
    bool ordered = true;
    while (got < 2 * kEach) {
        const size_t n = q.drain(out);   // never waits: 0 when a pusher holds the lock
        for (size_t i = 0; i < n; ++i) {
            const uint32_t tag = out[i] >> 24, seq = out[i] & 0xFFFFFF;
            ordered = ordered && seq == next[tag];
            next[tag] = seq + 1;
        }
        got += uint32_t(n);
    }
    a.join();
    b.join();
    CHECK(ordered);
    CHECK(next[1] == kEach);
    CHECK(next[2] == kEach);
}

// ---- the snapshot mailbox -----------------------------------------------------------

TEST_CASE("mailbox: newest wins, an untaken snapshot's samples and log carry forward") {
    devtool::SnapshotMailbox box;
    devtool::Snapshot hub, reader;
    CHECK_FALSE(box.take(reader));
    hub.seq = 1;
    hub.samples = {{1, 1.0f, 0.0f}, {2, 2.0f, 0.0f}};
    hub.log = {{'I', "one"}};
    REQUIRE(box.publish(hub));
    hub.samples.clear();
    hub.log.clear();
    hub.seq = 2;
    hub.samples = {{3, 3.0f, 0.0f}};
    hub.log = {{'W', "two"}};
    REQUIRE(box.publish(hub));
    REQUIRE(box.take(reader));
    CHECK(reader.seq == 2);
    REQUIRE(reader.samples.size() == 3);
    CHECK(reader.samples[0].us == 1);
    CHECK(reader.samples[2].us == 3);
    REQUIRE(reader.log.size() == 2);
    CHECK(reader.log[0].text == "one");
    CHECK(reader.log[1].text == "two");
    CHECK_FALSE(box.take(reader));
}

TEST_CASE("mailbox: a hub publishing every pass and a slower reader see every sample once, in order") {
    devtool::SnapshotMailbox box;
    constexpr uint64_t kPasses = 50000;
    std::atomic<bool> done{false};
    std::vector<uint64_t> seen;
    std::thread reader([&] {
        devtool::Snapshot s;
        for (;;) {
            const bool last = done.load();
            while (box.take(s))
                for (const auto& p : s.samples) seen.push_back(p.us);
            if (last) break;
            std::this_thread::yield();
        }
    });
    devtool::Snapshot hub;
    for (uint64_t us = 1; us <= kPasses; ++us) {
        hub.samples.push_back({us, 0.0f, 0.0f});
        if (box.publish(hub)) {   // never waits: false keeps accumulating
            hub.samples.clear();
            hub.log.clear();
        }
    }
    while (!box.publish(hub)) std::this_thread::yield();
    done = true;
    reader.join();
    REQUIRE(seen.size() == kPasses);
    bool inOrder = true;
    for (uint64_t i = 0; i < kPasses; ++i) inOrder = inOrder && seen[i] == i + 1;
    CHECK(inOrder);
}

TEST_CASE("mailbox: a waiting take wakes on the publish, and times out without one") {
    devtool::SnapshotMailbox box;
    devtool::Snapshot s;
    CHECK_FALSE(box.take(s, std::chrono::milliseconds(5)));
    const auto t0 = std::chrono::steady_clock::now();
    std::thread hub([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        devtool::Snapshot h;
        h.seq = 7;
        while (!box.publish(h)) std::this_thread::yield();
    });
    REQUIRE(box.take(s, std::chrono::milliseconds(5000)));
    const auto waited = std::chrono::steady_clock::now() - t0;
    hub.join();
    CHECK(s.seq == 7);
    CHECK(waited < std::chrono::milliseconds(2000));
}

// ---- the command line --------------------------------------------------------------

TEST_CASE("parseInput: every command, and the malformed ones refused") {
    Input in;
    REQUIRE(devtool::parseInput("pair down", in));
    CHECK(in.kind == Input::Kind::pair);
    CHECK(in.down);
    REQUIRE(devtool::parseInput("estop up", in));
    CHECK(in.kind == Input::Kind::estop);
    CHECK_FALSE(in.down);
    REQUIRE(devtool::parseInput("home down", in));
    CHECK(in.kind == Input::Kind::home);
    REQUIRE(devtool::parseInput("drive-alarm", in));
    CHECK(in.kind == Input::Kind::drive_alarm);
    REQUIRE(devtool::parseInput("switch-fault", in));
    REQUIRE(devtool::parseInput("faults all 40 10 2.5", in));
    CHECK(in.kind == Input::Kind::faults);
    CHECK(in.slot == -1);
    CHECK(in.faults == LinkFaults{40, 10, 2.5f});
    REQUIRE(devtool::parseInput("faults 3 0 0 0", in));
    CHECK(in.slot == 3);
    REQUIRE(devtool::parseInput("wifi-drop 2.5", in));
    CHECK(in.ms == 2500);
    REQUIRE(devtool::parseInput("kick 1", in));
    CHECK(in.slot == 1);

    for (const char* bad : {"", "pair", "pair sideways", "faults all 1 2", "faults x 1 2 3", "faults all 1 2 101",
                            "wifi-drop", "wifi-drop -1", "kick", "kick x", "drive-alarm now", "reboot"})
        CHECK_MESSAGE(!devtool::parseInput(bad, in), bad);
}

// ---- the JSON ----------------------------------------------------------------------

TEST_CASE("encodeSnapshot: one object, strings escaped, samples flat") {
    devtool::Snapshot s;
    s.seq = 7;
    s.m.position_mm = 12.5f;
    s.stripN = 2;
    s.strip[0] = 1.0f;
    s.strip[1] = 2.0f;
    devtool::Client c;
    c.name = "say \"hi\"";
    c.link.rtt_ms = 81;
    s.clients.push_back(c);
    s.samples = {{1000, 1.5f, -2.0f}};
    s.log = {{'W', "tab\there"}};
    std::string j;
    devtool::encodeSnapshot(s, j);
    CHECK(j.front() == '{');
    CHECK(j.back() == '}');
    CHECK(j.find("\"seq\":7") != std::string::npos);
    CHECK(j.find("\"pos\":12.500") != std::string::npos);
    CHECK(j.find("\"p\":[1.00,2.00]") != std::string::npos);
    CHECK(j.find("say \\\"hi\\\"") != std::string::npos);
    CHECK(j.find("\"rtt\":81") != std::string::npos);
    CHECK(j.find("\"s\":[1000,1.500,-2.00]") != std::string::npos);
    CHECK(j.find("tab\\u0009here") != std::string::npos);
}

// ---- the fault link ------------------------------------------------------------------

TEST_CASE("link: no fault set is a straight pass-through both ways") {
    FakeTransport t;
    NeutrinoLink l;
    l.bind(&t);
    t.inbox.push_back(frame(FrameType::PING, 1));
    CHECK(readAll(l, 0) == std::vector<uint16_t>{1});
    CHECK(l.write(frame(FrameType::PONG, 2).bytes()));
    REQUIRE(t.sent.size() == 1);
    CHECK(l.stats().queued == 0);
}

TEST_CASE("link: latency holds each direction, and the RTT is both ways") {
    FakeTransport t;
    NeutrinoLink l;
    l.bind(&t);
    l.setFaults({30, 0, 0.0f});
    l.tick(1000000);
    t.inbox.push_back(frame(FrameType::PING, 1));
    CHECK(readAll(l, 1000000).empty());
    CHECK(readAll(l, 1029000).empty());
    CHECK(readAll(l, 1030000) == std::vector<uint16_t>{1});
    CHECK(l.write(frame(FrameType::PONG, 1).bytes()));
    l.tick(1059000);
    CHECK(t.sent.empty());
    l.tick(1060000);
    REQUIRE(t.sent.size() == 1);
    CHECK(l.stats().rtt_ms == 60);
}

TEST_CASE("link: jitter never reorders, loss is a late frame never a missing one") {
    FakeTransport t;
    NeutrinoLink l;
    l.bind(&t);
    l.setFaults({5, 40, 30.0f});
    std::vector<uint16_t> got;
    uint64_t now = 0;
    for (uint16_t i = 0; i < 400; ++i) {
        t.inbox.push_back(frame(FrameType::STREAM, i));
        for (uint16_t f : readAll(l, now)) got.push_back(f);
        CHECK(l.write(frame(FrameType::STATE, i).bytes()));
        now += 1000;
    }
    for (int k = 0; k < 2000; ++k, now += 1000)
        for (uint16_t f : readAll(l, now)) got.push_back(f);
    REQUIRE(got.size() == 400);
    REQUIRE(t.sent.size() == 400);
    bool inOrder = true;
    for (uint16_t i = 0; i < 400; ++i) inOrder = inOrder && got[i] == i && tagOf(t.sent[i]) == i;
    CHECK(inOrder);
    CHECK(l.stats().retransmits > 100);   // 30 % of 800 frames
    CHECK(l.stats().retransmits < 400);
}

TEST_CASE("link: a Wi-Fi drop holds both ways, a full queue refuses, release delivers what waited") {
    FakeTransport t;
    NeutrinoLink l;
    l.bind(&t);
    l.setHeld(true);
    for (uint16_t i = 0; i < 300; ++i) t.inbox.push_back(frame(FrameType::STREAM, i));
    CHECK(readAll(l, 1000).empty());
    CHECK(t.inbox.size() == 300 - NeutrinoLink::kQueueDepth);   // the rest stays in the transport's ring
    size_t accepted = 0;
    for (uint16_t i = 0; i < 300; ++i) accepted += l.write(frame(FrameType::STATE, i).bytes()) ? 1 : 0;
    CHECK(accepted == NeutrinoLink::kQueueDepth);
    CHECK(l.stats().refused == 300 - NeutrinoLink::kQueueDepth);
    l.tick(500000);
    CHECK(t.sent.empty());
    l.setHeld(false);
    CHECK(readAll(l, 501000).size() == 300);
    CHECK(t.sent.size() == NeutrinoLink::kQueueDepth);
}

TEST_CASE("link: the session id comes from the WELCOME the hub sends") {
    FakeTransport t;
    NeutrinoLink l;
    l.bind(&t);
    WelcomeMsg w;
    w.session_id = 0xC0FFEE;
    w.boot_id = 9;
    std::array<std::byte, kFrameBufferCapacity> buf{};
    const size_t body = encodeWelcome(w, std::span(buf).subspan(kHeaderBytes));
    REQUIRE(body > 0);
    FrameHeader h;
    h.type = uint8_t(FrameType::WELCOME);
    h.len = uint16_t(body);
    encodeFrameHeader(h, buf);
    CHECK(l.write(std::span(buf).first(kHeaderBytes + body)));
    CHECK(l.stats().session_id == 0xC0FFEE);
    l.bind(&t);
    CHECK(l.stats().session_id == 0);
}

TEST_CASE("faults combine: delays add, losses compound") {
    const LinkFaults c = combineFaults({10, 5, 50.0f}, {20, 1, 50.0f});
    CHECK(c.latency_ms == 30);
    CHECK(c.jitter_ms == 6);
    CHECK(c.loss_pct == doctest::Approx(75.0f));
}

// ---- the Wi-Fi drop on the discovery port (bd val-6sa) ----------------------

namespace {
int g_liveOffered = 0;
bool liveHook(std::span<const std::byte>, uint32_t, uint32_t) {
    ++g_liveOffered;
    return false;
}
}  // namespace

TEST_CASE("discovery: a Wi-Fi drop consumes every datagram unanswered, the restore answers again") {
    DiscoveryResponder r;
    r.setIdentity("Neutrino", "0.1", 82);
    std::array<std::byte, kDiscoverProbeBytes> probe{};
    REQUIRE(encodeDiscoverProbe(DiscoverProbe{kProtocolVersion, 7}, probe) == probe.size());
    std::array<std::byte, kDiscoverReplyBytes> out{};
    g_liveOffered = 0;

    r.setDatagramHook(devtool::discoveryHook(true, &liveHook));
    CHECK(r.answer(probe, 0x0A000001u, 0, out) == 0);
    CHECK(r.answer(probe, 0x0A000002u, 1, out) == 0);
    CHECK(g_liveOffered == 0);
    CHECK(r.replies() == 0);

    r.setDatagramHook(devtool::discoveryHook(false, &liveHook));
    CHECK(r.answer(probe, 0x0A000001u, 2, out) == out.size());
    CHECK(g_liveOffered == 1);
    CHECK(r.replies() == 1);
}
