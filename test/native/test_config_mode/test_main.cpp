// test_config_mode -- native doctest suite for config mode, the offline BLE
// fallback, the single press, and the BLE GATT pipe
// Constraints:
// - Hardware-free: LinkMode.h and ValenceBle.h's pipe, with a synthetic clock
//   and a fake notifier. NimBLE, esp_hosted and the C6 are bench work.
// See: flagship_p4/src/hub/LinkMode.h, ValenceBle.h; Valence SPEC §5.6,
// §13.4, §13.4.1; RFC-114, RFC-117 drafts

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstdint>
#include <vector>

#include "geiger/geiger.h"
#include "valence/wire/frame_header.hpp"

#include "../../../flagship_p4/src/hub/LinkMode.h"
#include "../../../flagship_p4/src/hub/ValenceBle.h"

using namespace valence;
using linkmode::Policy;
using linkmode::PressAct;

namespace {

constexpr uint8_t kWindow = ble_adv_flags::pairing_window_open;
constexpr uint8_t kWs = ble_adv_flags::ws_available;
constexpr uint8_t kConfig = ble_adv_flags::config_mode;

// Steps the policy every 5 ms for `ms`.
void run(Policy& p, uint32_t& t, uint32_t ms, bool online, bool bleAttached) {
    for (uint32_t end = t + ms; t < end; t += 5) p.tick(online, bleAttached, t);
}

class FakeNotifier final : public IBleNotifier {
public:
    bool full = false;
    std::vector<std::vector<std::byte>> sent;
    bool notify(std::span<const std::byte> v) override {
        if (full) return false;
        sent.emplace_back(v.begin(), v.end());
        return true;
    }
};

class Sink final : public FrameSink {
public:
    std::vector<std::vector<std::byte>> got;
    void push(const FrameBuffer& f) override { got.emplace_back(f.bytes().begin(), f.bytes().end()); }
};

std::vector<std::byte> frame(FrameType type, size_t payload, uint8_t fill = 0x5A) {
    std::vector<std::byte> f(kHeaderBytes + payload, std::byte{fill});
    FrameHeader h;
    h.type = uint8_t(type);
    h.len = uint16_t(payload);
    h.seq = 7;
    encodeFrameHeader(h, f);
    return f;
}

}  // namespace

// ---- the policy -------------------------------------------------------------

TEST_CASE("config mode: BLE from boot for the whole mode, config_mode set, never ws_available") {
    Policy p;
    uint32_t t = 0;
    p.boot(true, true, t);
    CHECK(p.configMode());
    CHECK(p.bleWanted());
    CHECK(p.offline());
    CHECK(p.advFlags(true) == (kWindow | kConfig));
    run(p, t, 600000, true, false);   // even an address does not end the mode
    CHECK(p.bleWanted());
    CHECK(p.advFlags(false) == kConfig);
}

TEST_CASE("a healthy hub never starts BLE") {
    Policy p;
    uint32_t t = 0;
    p.boot(false, true, t);
    run(p, t, 10000, false, false);   // joining
    CHECK_FALSE(p.bleWanted());
    run(p, t, 3600000, true, false);
    CHECK_FALSE(p.bleWanted());
    CHECK_FALSE(p.offline());
    CHECK(p.advFlags(false) == kWs);
}

TEST_CASE("no credentials: BLE at once") {
    Policy p;
    p.boot(false, false, 0);
    CHECK(p.bleWanted());
    CHECK(p.advFlags(false) == 0);
}

TEST_CASE("offline 30 s continuously starts BLE; a short drop does not") {
    Policy p;
    uint32_t t = 0;
    p.boot(false, true, t);
    run(p, t, 5000, true, false);
    run(p, t, 20000, false, false);   // a 20 s drop
    run(p, t, 1000, true, false);
    run(p, t, linkmode::kOfflineBleAfterMs - 100, false, false);
    CHECK_FALSE(p.bleWanted());
    run(p, t, 200, false, false);
    CHECK(p.bleWanted());
    CHECK(p.offline());
}

TEST_CASE("BLE stops online only with no BLE session, after the dwell") {
    Policy p;
    uint32_t t = 0;
    p.boot(false, false, t);
    REQUIRE(p.bleWanted());
    run(p, t, 10000, true, false);   // back online at once: the dwell holds BLE up
    CHECK(p.bleWanted());
    run(p, t, linkmode::kBleDwellMs, true, true);   // a BLE session is attached
    CHECK(p.bleWanted());
    run(p, t, 5, true, false);   // it migrated to WS
    CHECK_FALSE(p.bleWanted());
    CHECK(p.advFlags(false) == kWs);
}

TEST_CASE("a hub that comes back by itself stops BLE after the dwell") {
    Policy p;
    uint32_t t = 0;
    p.boot(false, true, t);
    run(p, t, linkmode::kOfflineBleAfterMs + 10, false, false);
    REQUIRE(p.bleWanted());
    run(p, t, linkmode::kBleDwellMs + 10, true, false);
    CHECK_FALSE(p.bleWanted());
}

// ---- the single press -------------------------------------------------------

TEST_CASE("online, a press opens the window (RFC-114 item 4), asks or not") {
    CHECK(linkmode::singlePress(false, false, true, 0, 0) == PressAct::open_window);
    CHECK(linkmode::singlePress(false, false, true, 1, 3) == PressAct::open_window);
}

TEST_CASE("offline, a press approves the configure ask; several are the hub's refusal") {
    CHECK(linkmode::singlePress(true, false, true, 1, 0) == PressAct::approve);
    CHECK(linkmode::singlePress(true, false, true, 2, 0) == PressAct::approve);   // hub answers several_pending
    CHECK(linkmode::singlePress(true, true, true, 1, 0) == PressAct::approve);
}

TEST_CASE("offline, a LAN session left (parked included) refuses the press") {
    CHECK(linkmode::singlePress(true, false, true, 1, 1) == PressAct::refuse_lan);
}

TEST_CASE("offline with no ask: config mode or unclaimed opens the window; claimed opens nothing") {
    CHECK(linkmode::singlePress(true, true, true, 0, 0) == PressAct::open_window);
    CHECK(linkmode::singlePress(true, false, false, 0, 0) == PressAct::open_window);
    CHECK(linkmode::singlePress(true, false, true, 0, 0) == PressAct::nothing);
}

// ---- the BLE pipe -----------------------------------------------------------

TEST_CASE("a write reaches the sink on its own link only") {
    BleFramePipe pipe;
    FakeNotifier n;
    pipe.begin(n);
    Sink s;
    const auto f = frame(FrameType::HELLO, 20);
    CHECK_FALSE(pipe.onWrite(f));   // no link
    pipe.onConnect();
    CHECK(pipe.linkEpoch() != 0);
    CHECK(pipe.onWrite(f));
    pipe.onDisconnect();
    pipe.onConnect();   // a new link before the hub task drained
    pipe.drain(s);
    CHECK(s.got.empty());   // the old link's frame never reaches the new one
    CHECK(pipe.onWrite(f));
    pipe.drain(s);
    REQUIRE(s.got.size() == 1);
    CHECK(s.got[0] == f);
}

TEST_CASE("the ring drops past its depth and counts it") {
    BleFramePipe pipe;
    FakeNotifier n;
    pipe.begin(n);
    pipe.onConnect();
    const auto f = frame(FrameType::INTENT, 10);
    for (int i = 0; i < BleRxRing::kDepth; ++i) CHECK(pipe.onWrite(f));
    CHECK_FALSE(pipe.onWrite(f));
    CHECK(pipe.rxDrops() == 1);
}

TEST_CASE("MTU sets max_frame: ATT_MTU - 3, capped at the frame buffer") {
    BleFramePipe pipe;
    FakeNotifier n;
    pipe.begin(n);
    pipe.onConnect();
    CHECK(pipe.properties().mtu == 20);
    pipe.onMtu(247);
    CHECK(pipe.properties().mtu == 244);
    pipe.onMtu(517);
    CHECK(pipe.properties().mtu == kFrameBufferCapacity);
    CHECK_FALSE(pipe.properties().reliable);
}

TEST_CASE("a frame that fits is one notification") {
    BleFramePipe pipe;
    FakeNotifier n;
    pipe.begin(n);
    pipe.onConnect();
    pipe.onMtu(247);
    const auto f = frame(FrameType::STATE, 200);
    CHECK(pipe.send(f));
    REQUIRE(n.sent.size() == 1);
    CHECK(n.sent[0] == f);
}

TEST_CASE("an oversized control frame leaves as SPEC 5.6 fragments that reassemble") {
    BleFramePipe pipe;
    FakeNotifier n;
    pipe.begin(n);
    pipe.onConnect();
    pipe.onMtu(247);
    const auto f = frame(FrameType::WELCOME, 450, 0x33);
    CHECK(pipe.send(f));
    REQUIRE(n.sent.size() == 2);
    for (const auto& frag : n.sent) CHECK(frag.size() <= 244);

    // The client end: the same fragments written back reassemble whole.
    BleFramePipe back;
    FakeNotifier n2;
    back.begin(n2);
    back.onConnect();
    for (const auto& frag : n.sent) CHECK(back.onWrite(frag));
    Sink s;
    back.drain(s);
    REQUIRE(s.got.size() == 1);
    CHECK(s.got[0].size() == f.size());
    CHECK(s.got[0] == f);
}

TEST_CASE("STATE and STREAM never fragment: refused, so the port sheds them") {
    BleFramePipe pipe;
    FakeNotifier n;
    pipe.begin(n);
    pipe.onConnect();
    pipe.onMtu(185);
    CHECK_FALSE(pipe.send(frame(FrameType::STATE, 240)));
    CHECK_FALSE(pipe.send(frame(FrameType::STREAM, 240)));
    CHECK(n.sent.empty());
}

TEST_CASE("fragments the host cannot take wait, and nothing overtakes them") {
    BleFramePipe pipe;
    FakeNotifier n;
    pipe.begin(n);
    pipe.onConnect();
    pipe.onMtu(247);
    n.full = true;
    CHECK(pipe.send(frame(FrameType::WELCOME, 450)));   // queued whole
    CHECK_FALSE(pipe.send(frame(FrameType::ECHO, 10)));   // behind it
    n.full = false;
    Sink s;
    pipe.drain(s);   // the hub tick flushes
    CHECK(n.sent.size() == 2);
    CHECK(pipe.send(frame(FrameType::ECHO, 10)));
    CHECK(n.sent.size() == 3);
}

TEST_CASE("a send with no link is refused") {
    BleFramePipe pipe;
    FakeNotifier n;
    pipe.begin(n);
    CHECK_FALSE(pipe.send(frame(FrameType::ECHO, 10)));
    CHECK(pipe.linkEpoch() == 0);
}
