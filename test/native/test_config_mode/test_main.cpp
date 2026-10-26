// test_config_mode -- native doctest suite for the boot routes (SPEC 13.1),
// config mode (13.4.1), the advertisement and its endpoint tail (13.4), the
// policy reboots, the single press, and the BLE GATT pipe
// Constraints:
// - Hardware-free: LinkMode.h and ValenceBle.h's pipe and record, with a
//   synthetic clock and a fake notifier. NimBLE, esp_hosted and the C6 are
//   bench work.
// See: flagship_p4/src/hub/LinkMode.h, ValenceBle.h; Valence SPEC §5.6,
// §12.3, §13.1, §13.4, §13.4.1

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "geiger/geiger.h"
#include "valence/wire/frame_header.hpp"

#include "../../../flagship_p4/src/hub/LinkMode.h"
#include "../../../flagship_p4/src/hub/ValenceBle.h"

using namespace valence;
using linkmode::Boot;
using linkmode::BootFacts;
using linkmode::NetLook;
using linkmode::Next;
using linkmode::Policy;
using linkmode::PressAct;

namespace {

constexpr uint8_t kWindow = ble_adv_flags::pairing_window_open;
constexpr uint8_t kWs = ble_adv_flags::ws_available;
constexpr uint8_t kConfig = ble_adv_flags::config_mode;

// Steps the policy every 5 ms for `ms`.
void run(Policy& p, uint32_t& t, uint32_t ms, bool online) {
    for (uint32_t end = t + ms; t < end; t += 5) p.tick(online, t);
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

// ---- the route ---------------------------------------------------------------

Boot decide(Next next, bool held, bool radiosOff, bool creds) {
    BootFacts f;
    f.next = next;
    f.pairHeld = held;
    f.radiosOff = radiosOff;
    f.haveCredentials = creds;
    return linkmode::decideBoot(f);
}

TEST_CASE("route: credentials and nothing asked boot normal") {
    CHECK(decide(Next::none, false, false, true) == Boot::normal);
}

TEST_CASE("route: the gesture, held at boot or made while running, enters config mode, radios off or not") {
    CHECK(decide(Next::none, true, false, true) == Boot::config_gesture);
    CHECK(decide(Next::config_gesture, false, false, true) == Boot::config_gesture);
    CHECK(decide(Next::config_gesture, false, true, false) == Boot::config_gesture);
    CHECK(decide(Next::skip_auto, true, true, true) == Boot::config_gesture);
}

TEST_CASE("route: the radios-off setting is the serial route, with no automatic config mode") {
    CHECK(decide(Next::none, false, true, true) == Boot::radios_off);
    CHECK(decide(Next::none, false, true, false) == Boot::radios_off);
    CHECK(decide(Next::config_join_failed, false, true, true) == Boot::radios_off);
}

TEST_CASE("route: no credentials, or a missed boot-time join, enter config mode by policy") {
    CHECK(decide(Next::none, false, false, false) == Boot::config_policy);
    CHECK(decide(Next::config_join_failed, false, false, true) == Boot::config_policy);
}

TEST_CASE("route: the boot after a reboot out of policy config mode skips the automatic entry") {
    CHECK(decide(Next::skip_auto, false, false, true) == Boot::normal);
    CHECK(decide(Next::skip_auto, false, false, false) == Boot::normal);
}

// ---- the advertisement and the look ----------------------------------------------

TEST_CASE("config mode: config_mode set for the whole mode, never ws_available") {
    for (const Boot b : {Boot::config_gesture, Boot::config_policy}) {
        Policy p;
        uint32_t t = 0;
        p.boot(b, true, t);
        CHECK(p.configMode());
        CHECK(p.advertising());
        CHECK(p.advFlags(true) == (kWindow | kConfig));
        run(p, t, 600000, true);   // even an address does not end the mode
        CHECK(p.advFlags(false) == kConfig);
        CHECK(p.look() == NetLook::config);
    }
}

TEST_CASE("normal mode: advertises, ws_available only while online, never config_mode") {
    Policy p;
    uint32_t t = 0;
    p.boot(Boot::normal, true, t);
    CHECK_FALSE(p.configMode());
    CHECK(p.advertising());
    CHECK(p.advFlags(false) == 0);
    CHECK(p.look() == NetLook::joining);
    run(p, t, 1000, true);
    CHECK(p.advFlags(false) == kWs);
    CHECK(p.advFlags(true) == (kWs | kWindow));
    CHECK(p.look() == NetLook::online);
    run(p, t, 1000, false);
    CHECK(p.advFlags(false) == 0);
    CHECK(p.look() == NetLook::lost);
}

TEST_CASE("normal mode: the advertisement pauses while motion runs and kAdvQuietAfterMotionMs after") {
    Policy p;
    uint32_t t = 0;
    p.boot(Boot::normal, true, t);
    run(p, t, 1000, true);
    CHECK(p.advertisingNow());
    p.noteMotion(true);
    CHECK_FALSE(p.advertisingNow());
    run(p, t, linkmode::kAdvQuietAfterMotionMs - 10, true);
    p.noteMotion(false);
    CHECK_FALSE(p.advertisingNow());   // a brake to rest inside the stream keeps the radio
    run(p, t, 20, true);
    CHECK(p.advertisingNow());
    CHECK(p.advertising());
}

TEST_CASE("config mode never pauses its advertisement; radios off never advertises") {
    for (Boot b : {Boot::config_gesture, Boot::config_policy}) {
        Policy p;
        uint32_t t = 0;
        p.boot(b, true, t);
        p.noteMotion(true);
        CHECK(p.advertisingNow());
    }
    Policy off;
    off.boot(Boot::radios_off, true, 0);
    CHECK_FALSE(off.advertisingNow());
}

TEST_CASE("radios off: nothing advertised, nothing online") {
    Policy p;
    uint32_t t = 0;
    p.boot(Boot::radios_off, true, t);
    CHECK_FALSE(p.advertising());
    CHECK_FALSE(p.configMode());
    run(p, t, 1000, true);
    CHECK_FALSE(p.wsAvailable());
    CHECK(p.look() == NetLook::radios_off);
}

// ---- the policy reboots (SPEC 13.4.1, device-defined) ------------------------------

TEST_CASE("normal: a boot-time join that misses the window wants config mode; no sooner") {
    Policy p;
    uint32_t t = 0;
    p.boot(Boot::normal, true, t);
    run(p, t, linkmode::kJoinWindowMs - 10, false);
    CHECK(p.rebootWanted(false, false, false) == Next::none);
    run(p, t, 20, false);
    CHECK(p.rebootWanted(false, false, false) == Next::config_join_failed);
}

TEST_CASE("normal: an attached session holds the boot; once online, never") {
    Policy p;
    uint32_t t = 0;
    p.boot(Boot::normal, true, t);
    run(p, t, linkmode::kJoinWindowMs + 10, false);
    CHECK(p.rebootWanted(true, false, false) == Next::none);

    Policy q;
    t = 0;
    q.boot(Boot::normal, true, t);
    run(q, t, 1000, true);
    run(q, t, linkmode::kJoinWindowMs * 2, false);   // lost after it was up
    CHECK(q.rebootWanted(false, false, false) == Next::none);
}

TEST_CASE("normal: the boot after a reboot out (no automatic entry) never reboots for a slow join") {
    Policy p;
    uint32_t t = 0;
    p.boot(Boot::normal, false, t);
    run(p, t, linkmode::kJoinWindowMs * 3, false);
    CHECK(p.rebootWanted(false, false, false) == Next::none);
}

TEST_CASE("policy config mode: an address with no configure session reboots out, skipping the next entry") {
    Policy p;
    uint32_t t = 0;
    p.boot(Boot::config_policy, true, t);
    run(p, t, 1000, false);
    CHECK(p.rebootWanted(false, false, false) == Next::none);
    CHECK(p.rebootWanted(false, true, true) == Next::none);        // a configure session holds it
    CHECK(p.rebootWanted(true, false, true) == Next::skip_auto);   // a lesser one does not
}

TEST_CASE("gesture config mode and radios off: no policy reboot") {
    for (const Boot b : {Boot::config_gesture, Boot::radios_off}) {
        Policy p;
        uint32_t t = 0;
        p.boot(b, true, t);
        run(p, t, linkmode::kJoinWindowMs * 2, false);
        CHECK(p.rebootWanted(false, false, true) == Next::none);
        CHECK(p.rebootWanted(false, false, false) == Next::none);
    }
}

// ---- the single press ----------------------------------------------------------

TEST_CASE("outside policy config mode a press opens the window (RFC-114 item 4), asks or not") {
    for (const Boot b : {Boot::normal, Boot::radios_off, Boot::config_gesture}) {
        CHECK(linkmode::singlePress(b, true, 0) == PressAct::open_window);
        CHECK(linkmode::singlePress(b, true, 1) == PressAct::open_window);
    }
}

TEST_CASE("policy config mode: a press approves the configure ask; several are the hub's refusal") {
    CHECK(linkmode::singlePress(Boot::config_policy, true, 1) == PressAct::approve);
    CHECK(linkmode::singlePress(Boot::config_policy, true, 2) == PressAct::approve);   // hub answers several_pending
}

TEST_CASE("policy config mode with no ask: unclaimed opens the window; claimed opens nothing") {
    CHECK(linkmode::singlePress(Boot::config_policy, false, 0) == PressAct::open_window);
    CHECK(linkmode::singlePress(Boot::config_policy, true, 0) == PressAct::nothing);
}

// ---- the scan response record (SPEC 13.4, RFC-072, RFC-121) --------------------------

TEST_CASE("record: company id, flags, and the endpoint tail only while ws_available") {
    std::array<uint8_t, kScanRecordMax> r{};
    REQUIRE(scanRecord(kWs | kWindow, 0xC0A801E5, 80, r) == 9);   // the SPEC's own example
    const std::array<uint8_t, 9> want{0xFF, 0xFF, uint8_t(kWs | kWindow), 0xC0, 0xA8, 0x01, 0xE5, 0x50, 0x00};
    CHECK(r == want);
    REQUIRE(scanRecord(kWs, 0xC0A80132, 82, r) == 9);
    CHECK(r[7] == 82);
    CHECK(r[8] == 0);

    std::array<uint8_t, kScanRecordMax> c{};
    CHECK(scanRecord(kConfig, 0xC0A801E5, 80, c) == 3);   // config mode: never the endpoint
    CHECK(c[2] == kConfig);
    CHECK(scanRecord(kWs, 0, 80, c) == 3);   // nothing to advertise
    CHECK(scanRecord(kWs, 0xC0A801E5, 0, c) == 3);
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
