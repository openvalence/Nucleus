// test_barnacle_update -- native doctest suite for the helper's firmware
// update over the barnacle link: the frozen update frames, the three
// transition tables cell by cell, and the sender and receiver run against
// each other through the real encoder and decoder
// Constraints:
// - Hardware-free: barnacle_update.hpp with a fake sink in place of esp_ota_*
//   and a synthetic 5 ms pass clock. The UART, the helper's flash, its
//   bootloader's rollback and esp_ota_end()'s SHA-256 check are bench work
//   (bd val-omh3.7).
// - Frames cross through bl::Decoder in both directions, one batch per pass:
//   what a pass sends arrives at the next, as on the wire.
// See: lib/barnacle_link/include/barnacle_link/barnacle_update.hpp

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <vector>

#include "barnacle_link/barnacle_link.hpp"
#include "barnacle_link/barnacle_update.hpp"

namespace bl = barnacle;

namespace {

using Bytes = std::vector<uint8_t>;
using bl::RecvAct;
using bl::RecvEvent;
using bl::RecvState;
using bl::SendAct;
using bl::SendEvent;
using bl::SendState;
using bl::TrialAct;
using bl::TrialEvent;
using bl::TrialState;

// An image of `size` bytes: a byte pattern with zeros in it, its last 32
// bytes standing in for the digest esptool appends.
Bytes makeImage(size_t size, uint8_t salt = 0) {
    Bytes b(size);
    for (size_t i = 0; i < size; ++i) b[i] = uint8_t((i * 7 + salt) % 251);
    for (size_t k = 0; k < bl::kDigestBytes; ++k) b[size - bl::kDigestBytes + k] = uint8_t(0xA0 + k + salt);
    return b;
}

bl::IdentDigest identOf(const Bytes& image) {
    bl::IdentDigest d{};
    for (size_t k = 0; k < bl::kIdentDigestBytes; ++k) d[k] = image[image.size() - bl::kDigestBytes + k];
    return d;
}

class FakeSink final : public bl::UpdateSink {
public:
    uint32_t capacity() const override { return cap; }
    bool begin(uint32_t size) override {
        ++begins;
        open = beginOk;
        wanted = size;
        written.clear();
        return beginOk;
    }
    bool write(std::span<const uint8_t> bytes) override {
        if (!open) return false;
        if (failWriteAt && written.size() >= *failWriteAt) return false;
        written.insert(written.end(), bytes.begin(), bytes.end());
        return true;
    }
    bool finish() override {
        const bool ok = open && finishOk;
        open = false;
        if (ok) ++finished;
        return ok;
    }
    void abort() override {
        open = false;
        ++aborts;
    }

    uint32_t cap = 0x140000;
    bool beginOk = true;
    bool finishOk = true;
    std::optional<size_t> failWriteAt;
    bool open = false;
    uint32_t wanted = 0;
    Bytes written;
    int begins = 0;
    int aborts = 0;
    int finished = 0;
};

template <typename T>
Bytes wireOf(const T& body, uint8_t seq) {
    std::array<uint8_t, bl::kMaxWire> out{};
    const size_t n = bl::encode(body, seq, out);
    REQUIRE(n > 0);
    return Bytes(out.begin(), out.begin() + long(n));
}

std::vector<bl::Frame> feedAll(bl::Decoder& d, const Bytes& w) {
    std::vector<bl::Frame> got;
    for (const uint8_t b : w)
        if (const std::optional<bl::Frame> f = d.feed(b)) got.push_back(*f);
    return got;
}

// The P4 and the helper joined through the decoders. One pass: the helper
// takes what the last pass sent and answers, the P4 takes the answers, then
// the P4's pass tick runs.
struct Rig {
    explicit Rig(Bytes img) : image(std::move(img)), tx(image) { rx.emplace(sink); }

    Bytes image;
    bl::UpdateSender tx;
    FakeSink sink;
    std::optional<bl::UpdateReceiver> rx;   // re-made when the helper restarts
    bl::Decoder atHelper;
    bl::Decoder atP4;
    std::vector<Bytes> toHelper;
    std::vector<Bytes> toP4;
    uint32_t now = 1000;
    uint8_t p4Seq = 0;
    uint8_t helperSeq = 0;
    bool onTrial = false;
    uint32_t p4Frames = 0;       // update frames the P4 sent
    uint32_t helperFrames = 0;   // acks the helper sent
    std::function<bool(uint32_t)> dropToHelper = [](uint32_t) { return false; };
    std::function<bool(uint32_t)> dropToP4 = [](uint32_t) { return false; };
    std::vector<bl::SendStep> steps;
    std::optional<bl::TrialGate> trial;   // set while the helper runs a new image
    bool helperUp = true;                 // false: it hears and says nothing
    // The helper restarts on trial once it has verified an image, and from
    // then on sends its ident every 100 ms, as the shim does.
    bool autoRestart = false;
    uint32_t sinceRestart = 0;

    void act(const bl::SendStep& s) {
        steps.push_back(s);
        std::array<uint8_t, bl::kMaxWire> out{};
        const size_t n = tx.frameFor(s.act, p4Seq++, out);
        if (n == 0) return;
        if (!dropToHelper(p4Frames++)) toHelper.emplace_back(out.begin(), out.begin() + long(n));
    }

    void pass(bool idle = true) {
        now += 5;
        std::vector<Bytes> in;
        in.swap(toHelper);
        for (const Bytes& w : in) {
            if (!helperUp) continue;
            for (const bl::Frame& f : feedAll(atHelper, w)) {
                if (trial) {
                    if (const std::optional<bl::UpdateKeep> k = bl::asUpdateKeep(f)) trial->onKeep(*k);
                }
                if (const std::optional<bl::UpdateAck> a = rx->onFrame(f, now, onTrial)) {
                    const Bytes ack = wireOf(*a, helperSeq++);
                    if (!dropToP4(helperFrames++)) toP4.push_back(ack);
                }
            }
        }
        rx->tick(now);
        std::vector<Bytes> back;
        back.swap(toP4);
        for (const Bytes& w : back)
            for (const bl::Frame& f : feedAll(atP4, w))
                if (const std::optional<bl::UpdateAck> a = bl::asUpdateAck(f)) act(tx.onAck(*a, now));
        if (const std::optional<bl::SendStep> s = tx.tick(now, idle, helperUp)) act(*s);
        if (!autoRestart) return;
        if (rx->state() == RecvState::done && !trial) restartOnTrial();
        if (trial && ++sinceRestart % 20 == 0) ident(identOf(image), trial->onTrial());
    }

    // The helper's ident, as the P4 decodes it.
    void ident(const bl::IdentDigest& d, bool trialFlag, uint8_t version = 5) {
        bl::Ident id;
        id.shim_version = version;
        id.digest = d;
        id.flags = trialFlag ? bl::kIdentTrial : 0;
        for (const bl::Frame& f : feedAll(atP4, wireOf(id, helperSeq++))) {
            const std::optional<bl::Ident> got = bl::asIdent(f);
            REQUIRE(got.has_value());
            act(tx.onIdent(*got, now));
        }
    }

    bool sawTransition(SendState from, SendState to) const {
        for (const bl::SendStep& s : steps)
            if (s.from == from && s.to == to) return true;
        return false;
    }

    // Passes until `done` or the budget runs out; the passes taken.
    int runUntil(const std::function<bool()>& done, int budget, bool idle = true) {
        for (int i = 0; i < budget; ++i) {
            if (done()) return i;
            pass(idle);
        }
        return budget;
    }

    // The helper restarting on what it just verified: a new boot on trial.
    void restartOnTrial() {
        trial.emplace(true, identOf(image));
        onTrial = true;
        rx.emplace(sink);
    }
};

}  // namespace

// ---- the frozen frames ------------------------------------------------------

TEST_CASE("the update frames round-trip under kUpdateVersion") {
    bl::Decoder d;
    bl::Ident id;
    id.shim_version = 5;
    id.digest = {0x00, 0x11, 0x00, 0x22, 0x33, 0x44, 0x55, 0x66};
    id.flags = bl::kIdentTrial;
    std::vector<bl::Frame> got = feedAll(d, wireOf(id, 9));
    REQUIRE(got.size() == 1);
    CHECK(got[0].version == bl::kUpdateVersion);
    CHECK(got[0].seq == 9);
    REQUIRE(bl::asIdent(got[0]).has_value());
    CHECK(*bl::asIdent(got[0]) == id);

    bl::UpdateBegin b;
    b.size = 321616;
    for (size_t k = 0; k < bl::kDigestBytes; ++k) b.digest[k] = uint8_t(k * 9);
    got = feedAll(d, wireOf(b, 1));
    REQUIRE(got.size() == 1);
    CHECK(*bl::asUpdateBegin(got[0]) == b);

    const Bytes image = makeImage(1000);
    bl::UpdateData data;
    data.index = 3;
    data.bytes = std::span<const uint8_t>(image.data() + 720, bl::kChunkBytes);
    const Bytes w = wireOf(data, 2);
    CHECK(w.size() <= bl::kMaxWire);
    got = feedAll(d, w);
    REQUIRE(got.size() == 1);
    const std::optional<bl::UpdateData> back = bl::asUpdateData(got[0]);
    REQUIRE(back.has_value());
    CHECK(back->index == 3);
    CHECK(Bytes(back->bytes.begin(), back->bytes.end()) == Bytes(image.begin() + 720, image.begin() + 960));

    bl::UpdateAck a;
    a.code = bl::AckCode::digest;
    a.next = 0x1234;
    got = feedAll(d, wireOf(a, 3));
    CHECK(*bl::asUpdateAck(got[0]) == a);

    std::array<uint8_t, bl::kMaxWire> out{};
    for (const bl::Type t : {bl::Type::update_end, bl::Type::update_abort}) {
        const size_t n = bl::encodeBare(t, 4, out);
        REQUIRE(n > 0);
        got = feedAll(d, Bytes(out.begin(), out.begin() + long(n)));
        REQUIRE(got.size() == 1);
        CHECK(got[0].type == t);
        CHECK(got[0].len == 0);
    }
    CHECK(bl::encodeBare(bl::Type::status, 4, out) == 0);
    CHECK(d.stats().version == 0);
}

TEST_CASE("the link's own parsers refuse an update frame, the update parsers a link frame") {
    bl::Frame f;
    f.version = bl::kUpdateVersion;
    f.type = bl::Type::status;
    f.len = bl::Status::kBytes;
    CHECK_FALSE(bl::asStatus(f).has_value());
    f.type = bl::Type::command;
    f.len = bl::Command::kBytes;
    CHECK_FALSE(bl::asCommand(f).has_value());
    f.version = bl::kVersion;
    f.type = bl::Type::ident;
    f.len = bl::Ident::kBytes;
    CHECK_FALSE(bl::asIdent(f).has_value());
    f.version = bl::kUpdateVersion;
    CHECK(bl::asIdent(f).has_value());
    f.type = bl::Type::update_ack;
    f.len = bl::UpdateAck::kBytes;
    f.body[0] = bl::kAckCodes;
    CHECK_FALSE(bl::asUpdateAck(f).has_value());
}

TEST_CASE("a frame neither kVersion nor kUpdateVersion is dropped and counted") {
    std::array<uint8_t, bl::kMaxPayload> p{};
    p[0] = bl::kUpdateVersion + 1;
    p[1] = uint8_t(bl::Type::ident);
    const size_t covered = bl::kHeaderBytes + bl::Ident::kBytes;
    bl::putU16(p, covered, bl::crc16(std::span<const uint8_t>(p.data(), covered)));
    std::array<uint8_t, bl::kMaxWire> w{};
    const size_t n = bl::cobsEncode(std::span<const uint8_t>(p.data(), covered + bl::kCrcBytes), w);
    bl::Decoder d;
    for (size_t i = 0; i < n; ++i) CHECK_FALSE(d.feed(w[i]).has_value());
    CHECK_FALSE(d.feed(0).has_value());
    CHECK(d.stats().version == 1);
}

TEST_CASE("the update layer's sizes hold") {
    CHECK(bl::chunkCount(1) == 1);
    CHECK(bl::chunkCount(uint32_t(bl::kChunkBytes)) == 1);
    CHECK(bl::chunkCount(uint32_t(bl::kChunkBytes) + 1) == 2);
    CHECK(bl::chunkCount(321616) == 1341);
    CHECK(bl::chunkCount(bl::kMaxImageBytes) == 65535);
    // The esp32dev OTA slot (0x140000) fits under the index's reach.
    CHECK(0x140000u <= bl::kMaxImageBytes);
}

// ---- the tables, cell by cell -----------------------------------------------

TEST_CASE("sender table: each action leads only where it belongs") {
    for (size_t s = 0; s < bl::kSendStates; ++s)
        for (size_t e = 0; e < bl::kSendEvents; ++e) {
            const auto c = bl::sendCell(SendState(s), SendEvent(e));
            if (!c.legal) continue;
            CAPTURE(bl::sendStateName(SendState(s)));
            CAPTURE(bl::sendEventName(SendEvent(e)));
            switch (c.act) {
                case SendAct::start:      CHECK(c.to == SendState::beginning); break;
                case SendAct::begin:      CHECK(c.to == SendState::beginning); break;
                case SendAct::chunk:      CHECK(c.to == SendState::sending); break;
                case SendAct::end:        CHECK(c.to == SendState::ending); break;
                case SendAct::abort:      CHECK(c.to == SendState::waiting); break;
                case SendAct::fail:
                case SendAct::fail_abort: CHECK(c.to == SendState::failed); break;
                case SendAct::await:      CHECK(c.to == SendState::restarting); break;
                case SendAct::keep:       CHECK(c.to == SendState::keeping); break;
                case SendAct::done:       CHECK(c.to == SendState::current); break;
                case SendAct::none:       break;
            }
        }
}

TEST_CASE("sender table: only waiting starts a transfer, only on idle, and only start (the unhome) does") {
    for (size_t s = 0; s < bl::kSendStates; ++s)
        for (size_t e = 0; e < bl::kSendEvents; ++e) {
            const auto c = bl::sendCell(SendState(s), SendEvent(e));
            if (!c.legal) continue;
            CAPTURE(bl::sendStateName(SendState(s)));
            CAPTURE(bl::sendEventName(SendEvent(e)));
            const bool entersTransfer = c.to == SendState::beginning && SendState(s) != SendState::beginning;
            CHECK(entersTransfer == (c.act == SendAct::start));
            if (c.act == SendAct::begin) CHECK(SendState(s) == SendState::beginning);
            if (!entersTransfer) continue;
            CHECK(SendState(s) == SendState::waiting);
            CHECK(SendEvent(e) == SendEvent::idle);
        }
}

TEST_CASE("sender table: busy mid-transfer always stops it with ABORT, and idle or busy is a rule everywhere") {
    for (const SendState s : {SendState::beginning, SendState::sending, SendState::ending}) {
        const auto c = bl::sendCell(s, SendEvent::busy);
        CHECK(c.legal);
        CHECK(c.act == SendAct::abort);
        CHECK(c.to == SendState::waiting);
    }
    for (size_t s = 0; s < bl::kSendStates; ++s) {
        CAPTURE(bl::sendStateName(SendState(s)));
        CHECK(bl::sendCell(SendState(s), SendEvent::idle).legal);
        CHECK(bl::sendCell(SendState(s), SendEvent::busy).legal);
    }
}

TEST_CASE("sender table: every ident has a rule in every state, so the 10 Hz idents never count as ignored") {
    for (size_t s = 0; s < bl::kSendStates; ++s)
        for (const SendEvent e : {SendEvent::ident_ours, SendEvent::ident_ours_trial, SendEvent::ident_other,
                                  SendEvent::ident_other_trial}) {
            CAPTURE(bl::sendStateName(SendState(s)));
            CAPTURE(bl::sendEventName(e));
            CHECK(bl::sendCell(SendState(s), e).legal);
        }
}

TEST_CASE("sender table: the cells with no rule are the impossible ones") {
    // An ack outside a transfer, a deadline in a state that arms none, a
    // legacy shim mid-transfer: each is ignored and counted.
    std::vector<std::pair<SendState, SendEvent>> ignored;
    for (size_t s = 0; s < bl::kSendStates; ++s)
        for (size_t e = 0; e < bl::kSendEvents; ++e)
            if (!bl::sendCell(SendState(s), SendEvent(e)).legal) ignored.emplace_back(SendState(s), SendEvent(e));
    for (const auto& [s, e] : ignored) {
        CAPTURE(bl::sendStateName(s));
        CAPTURE(bl::sendEventName(e));
        const bool ack = e == SendEvent::ack_ok || e == SendEvent::ack_all || e == SendEvent::ack_stale ||
                         e == SendEvent::ack_done || e == SendEvent::ack_error;
        const bool deadline = e == SendEvent::retry || e == SendEvent::expire || e == SendEvent::backoff_over ||
                              e == SendEvent::give_up;
        const bool legacyMidTransfer = e == SendEvent::legacy_seen;
        CHECK((ack || deadline || legacyMidTransfer));
    }
    CHECK(ignored.size() == 91);
}

TEST_CASE("receiver table: done takes nothing, idle opens only on begin, a session closes on every refusal") {
    for (size_t e = 0; e < bl::kRecvEvents; ++e) {
        if (RecvEvent(e) == RecvEvent::timeout) continue;   // never posted outside receiving
        const auto c = bl::recvCell(RecvState::done, RecvEvent(e));
        CHECK(c.legal);
        CHECK(c.to == RecvState::done);
        CHECK(c.act == RecvAct::none);
    }
    for (size_t e = 0; e < bl::kRecvEvents; ++e) {
        const auto c = bl::recvCell(RecvState::idle, RecvEvent(e));
        if (!c.legal) {
            CHECK(RecvEvent(e) == RecvEvent::timeout);
            continue;
        }
        CHECK((c.to == RecvState::idle || RecvEvent(e) == RecvEvent::begin));
    }
    for (const RecvEvent e : {RecvEvent::begin_refused, RecvEvent::chunk_bad, RecvEvent::end_short,
                              RecvEvent::end_digest}) {
        const auto c = bl::recvCell(RecvState::receiving, e);
        CHECK(c.to == RecvState::idle);
        CHECK(c.act == RecvAct::close_refuse);
    }
    for (size_t e = 0; e < bl::kRecvEvents; ++e) CHECK(bl::recvCell(RecvState::receiving, RecvEvent(e)).legal);
}

TEST_CASE("trial table: only a KEEP for the running image keeps it, only the deadline reverts") {
    for (size_t s = 0; s < bl::kTrialStates; ++s)
        for (size_t e = 0; e < bl::kTrialEvents; ++e) {
            const auto c = bl::trialCell(TrialState(s), TrialEvent(e));
            if (!c.legal) {
                CHECK(TrialEvent(e) == TrialEvent::deadline);
                CHECK(TrialState(s) != TrialState::trial);
                continue;
            }
            if (c.act == TrialAct::mark_valid) {
                CHECK(TrialState(s) == TrialState::trial);
                CHECK(TrialEvent(e) == TrialEvent::keep_ours);
            }
            if (c.act == TrialAct::revert) {
                CHECK(TrialState(s) == TrialState::trial);
                CHECK(TrialEvent(e) == TrialEvent::deadline);
            }
        }
}

// ---- the sender and the receiver together -----------------------------------

TEST_CASE("a whole image goes across, verifies, restarts on trial and is kept") {
    Rig r(makeImage(321616));
    REQUIRE(r.tx.valid());
    r.ident(bl::IdentDigest{1, 2, 3, 4, 5, 6, 7, 8}, false, 4);
    CHECK(r.tx.state() == SendState::waiting);
    const int passes = r.runUntil([&] { return r.tx.state() == SendState::restarting; }, 5000);
    REQUIRE(r.tx.state() == SendState::restarting);
    CHECK(r.rx->state() == RecvState::done);
    CHECK(r.sink.written == r.image);
    CHECK(r.sink.finished == 1);
    CHECK(r.sink.aborts == 0);
    CHECK(r.tx.attempts() == 0);
    // One chunk per pass when nothing is lost: chunks, plus BEGIN and END.
    CHECK(passes <= int(bl::chunkCount(321616)) + 4);

    r.restartOnTrial();
    r.ident(identOf(r.image), true);
    CHECK(r.tx.state() == SendState::keeping);
    r.pass();   // the KEEP crosses
    REQUIRE(r.trial.has_value());
    CHECK(r.trial->state() == TrialState::kept);
    r.ident(identOf(r.image), false);
    CHECK(r.tx.state() == SendState::current);
    CHECK(r.steps.back().act == SendAct::done);
    CHECK(r.tx.startedMs().has_value());
    CHECK(r.tx.ignored() == 0);
}

namespace {
size_t countAct(const std::vector<bl::SendStep>& steps, SendAct a) {
    return size_t(std::count_if(steps.begin(), steps.end(), [=](const bl::SendStep& s) { return s.act == a; }));
}
}  // namespace

TEST_CASE("an attempt's first BEGIN is start, the caller's unhome; a resent BEGIN is begin and unhomes nothing") {
    Rig r(makeImage(4000, 2));
    r.dropToHelper = [](uint32_t n) { return n < 2; };   // the first BEGIN and its first resend lost
    r.ident(bl::IdentDigest{}, false);
    r.runUntil([&] { return r.tx.state() == SendState::restarting; }, 20000);
    REQUIRE(r.tx.state() == SendState::restarting);
    CHECK(countAct(r.steps, SendAct::start) == 1);
    CHECK(countAct(r.steps, SendAct::begin) == 2);
    CHECK(r.sink.written == r.image);
    CHECK(r.tx.tries() >= 1);
}

TEST_CASE("start and begin send the same BEGIN frame") {
    Rig r(makeImage(4000, 2));
    std::array<uint8_t, bl::kMaxWire> a{};
    std::array<uint8_t, bl::kMaxWire> b{};
    const size_t na = r.tx.frameFor(SendAct::start, 7, a);
    const size_t nb = r.tx.frameFor(SendAct::begin, 7, b);
    REQUIRE(na > 0);
    CHECK(na == nb);
    CHECK(a == b);
}

TEST_CASE("lost frames in both directions are resent until the image is whole") {
    Rig r(makeImage(20000, 3));
    r.dropToHelper = [](uint32_t n) { return n % 7 == 3; };
    r.dropToP4 = [](uint32_t n) { return n % 5 == 1; };
    r.autoRestart = true;
    r.ident(bl::IdentDigest{}, false);
    r.runUntil([&] { return r.tx.state() == SendState::current; }, 200000);
    INFO(bl::sendStateName(r.tx.state()), " attempts ", r.tx.attempts(), " code ", int(r.tx.lastCode()));
    REQUIRE(r.tx.state() == SendState::current);
    CHECK(r.sink.written == r.image);
    CHECK(r.tx.attempts() == 0);
    CHECK(r.trial->state() == TrialState::kept);
}

TEST_CASE("the done ack lost: the new image's ident on trial still leads to KEEP, no attempt counted") {
    Rig r(makeImage(2000, 5));
    r.autoRestart = true;
    const uint32_t lastAck = bl::chunkCount(2000) + 1;   // BEGIN's ack, one per chunk, then END's
    r.dropToP4 = [=](uint32_t n) { return n == lastAck; };
    r.ident(bl::IdentDigest{}, false);
    r.runUntil([&] { return r.tx.state() == SendState::current; }, 20000);
    CHECK(r.tx.state() == SendState::current);
    CHECK(r.tx.attempts() == 0);
    CHECK(r.sawTransition(SendState::ending, SendState::keeping));
}

TEST_CASE("a corrupted chunk never lands: the decoder drops it, the resend replaces it") {
    Rig r(makeImage(5000, 9));
    r.ident(bl::IdentDigest{}, false);
    r.pass();   // BEGIN out
    r.pass();   // ack back, chunk 0 out
    REQUIRE(r.toHelper.size() == 1);
    r.toHelper[0][40] ^= 0x10;
    r.runUntil([&] { return r.tx.state() == SendState::restarting; }, 20000);
    REQUIRE(r.tx.state() == SendState::restarting);
    CHECK(r.sink.written == r.image);
    CHECK(r.atHelper.stats().crc >= 1);
}

TEST_CASE("the machine leaving idle mid-transfer aborts it; the helper keeps its image, the next idle restarts it") {
    Rig r(makeImage(30000, 1));
    r.ident(bl::IdentDigest{}, false);
    r.runUntil([&] { return r.tx.next() >= 40; }, 1000);
    REQUIRE(r.tx.state() == SendState::sending);
    r.pass(false);
    CHECK(r.tx.state() == SendState::waiting);
    CHECK(r.steps.back().act == SendAct::abort);
    r.pass(false);
    CHECK(r.rx->state() == RecvState::idle);
    CHECK(r.sink.aborts == 1);
    CHECK(r.sink.finished == 0);
    for (int i = 0; i < 50; ++i) r.pass(false);
    CHECK(r.tx.state() == SendState::waiting);
    r.runUntil([&] { return r.tx.state() == SendState::restarting; }, 5000);
    REQUIRE(r.tx.state() == SendState::restarting);
    CHECK(r.sink.written == r.image);
    CHECK(r.sink.begins == 2);
    CHECK(r.tx.attempts() == 0);
    // Each attempt unhomes again as it starts.
    CHECK(countAct(r.steps, SendAct::start) == 2);
}

TEST_CASE("a silent helper costs kMaxTries sends, then the attempt fails with ABORT; three attempts and it gives up") {
    Rig r(makeImage(8000));
    r.ident(bl::IdentDigest{}, false);
    r.pass();
    CHECK(r.tx.state() == SendState::beginning);
    r.helperUp = false;
    r.runUntil([&] { return r.tx.state() == SendState::failed; }, 10000);
    REQUIRE(r.tx.state() == SendState::failed);
    CHECK(r.steps.back().act == SendAct::fail_abort);
    CHECK(r.tx.attempts() == 1);
    CHECK(r.tx.tries() == bl::kMaxTries);
    // With the helper unheard, the backoff ends but no new BEGIN goes out.
    r.runUntil([&] { return r.tx.state() == SendState::waiting; }, 10000);
    REQUIRE(r.tx.state() == SendState::waiting);
    for (int i = 0; i < 100; ++i) r.pass();
    CHECK(r.tx.state() == SendState::waiting);
    // Heard again but still answering nothing (pass() reports it fresh).
    r.helperUp = true;
    r.dropToHelper = [](uint32_t) { return true; };
    r.runUntil([&] { return r.tx.state() == SendState::gave_up; }, 100000);
    CHECK(r.tx.state() == SendState::gave_up);
    CHECK(r.tx.attempts() == bl::kMaxAttempts);
    // A helper later found running this copy leaves gave_up.
    r.ident(identOf(r.image), false);
    CHECK(r.tx.state() == SendState::current);
}

TEST_CASE("a refusal fails the attempt with the helper's reason") {
    SUBCASE("on trial") {
        Rig r(makeImage(4000));
        r.onTrial = true;
        r.ident(bl::IdentDigest{}, false);   // a stale ident: the helper refuses anyway
        r.runUntil([&] { return r.tx.state() == SendState::failed; }, 100);
        CHECK(r.tx.lastCode() == bl::AckCode::on_trial);
    }
    SUBCASE("too big for the slot") {
        Rig r(makeImage(4000));
        r.sink.cap = 3000;
        r.ident(bl::IdentDigest{}, false);
        r.runUntil([&] { return r.tx.state() == SendState::failed; }, 100);
        CHECK(r.tx.lastCode() == bl::AckCode::too_big);
        CHECK(r.sink.begins == 0);
    }
    SUBCASE("the slot will not open") {
        Rig r(makeImage(4000));
        r.sink.beginOk = false;
        r.ident(bl::IdentDigest{}, false);
        r.runUntil([&] { return r.tx.state() == SendState::failed; }, 100);
        CHECK(r.tx.lastCode() == bl::AckCode::flash);
        CHECK(r.rx->state() == RecvState::idle);
    }
    SUBCASE("a write fails") {
        Rig r(makeImage(4000));
        r.sink.failWriteAt = 1000;
        r.ident(bl::IdentDigest{}, false);
        r.runUntil([&] { return r.tx.state() == SendState::failed; }, 100);
        CHECK(r.tx.lastCode() == bl::AckCode::flash);
        CHECK(r.rx->state() == RecvState::idle);
        CHECK(r.sink.aborts == 1);
    }
    SUBCASE("the image check refuses it") {
        Rig r(makeImage(4000));
        r.sink.finishOk = false;
        r.ident(bl::IdentDigest{}, false);
        r.runUntil([&] { return r.tx.state() == SendState::failed; }, 100);
        CHECK(r.tx.lastCode() == bl::AckCode::image);
        CHECK(r.rx->state() == RecvState::idle);
    }
}

TEST_CASE("the helper restarting mid-transfer answers no_session and the attempt fails") {
    Rig r(makeImage(30000));
    r.ident(bl::IdentDigest{}, false);
    r.runUntil([&] { return r.tx.next() >= 10; }, 1000);
    r.rx.emplace(r.sink);
    r.runUntil([&] { return r.tx.state() == SendState::failed; }, 100);
    CHECK(r.tx.lastCode() == bl::AckCode::no_session);
}

TEST_CASE("back on another image after the restart is the helper's bootloader refusing the new one") {
    Rig r(makeImage(3000));
    r.ident(bl::IdentDigest{9}, false);
    r.runUntil([&] { return r.tx.state() == SendState::restarting; }, 1000);
    r.ident(bl::IdentDigest{9}, false);
    CHECK(r.tx.state() == SendState::failed);
    CHECK(r.sawTransition(SendState::restarting, SendState::failed));
}

TEST_CASE("a helper that never comes back after the restart fails the attempt at kRestartMs") {
    Rig r(makeImage(3000));
    r.ident(bl::IdentDigest{9}, false);
    r.runUntil([&] { return r.tx.state() == SendState::restarting; }, 1000);
    const uint32_t at = r.now;
    r.runUntil([&] { return r.tx.state() == SendState::failed; }, 10000);
    CHECK(r.tx.state() == SendState::failed);
    CHECK(r.now - at >= bl::kRestartMs);
}

TEST_CASE("a trial that reverts fails the attempt, and the reverted helper is updated again") {
    Rig r(makeImage(3000));
    r.ident(bl::IdentDigest{9}, false);
    r.runUntil([&] { return r.tx.state() == SendState::restarting; }, 1000);
    r.ident(identOf(r.image), true);
    CHECK(r.tx.state() == SendState::keeping);
    r.ident(bl::IdentDigest{9}, false);
    CHECK(r.tx.state() == SendState::failed);
    CHECK(r.tx.attempts() == 1);
    r.onTrial = false;
    r.runUntil([&] { return r.tx.state() == SendState::beginning; }, 10000);
    CHECK(r.tx.state() == SendState::beginning);
}

TEST_CASE("a helper already on trial with this copy at the P4's boot is kept, with no transfer") {
    Rig r(makeImage(3000));
    r.ident(identOf(r.image), true);
    CHECK(r.tx.state() == SendState::keeping);
    CHECK(r.steps.back().act == SendAct::keep);
    r.ident(identOf(r.image), false);
    CHECK(r.tx.state() == SendState::current);
    CHECK_FALSE(r.tx.startedMs().has_value());
    CHECK(r.sink.begins == 0);
}

TEST_CASE("a helper on trial with another image is held, never begun, until it settles") {
    Rig r(makeImage(3000));
    r.ident(bl::IdentDigest{7}, true);
    CHECK(r.tx.state() == SendState::held);
    for (int i = 0; i < 100; ++i) r.pass();
    CHECK(r.tx.state() == SendState::held);
    CHECK(r.sink.begins == 0);
    r.ident(bl::IdentDigest{7}, false);
    CHECK(r.tx.state() == SendState::waiting);
}

TEST_CASE("a shim before v5: slow frames and no ident read as legacy; an ident later is believed") {
    Rig r(makeImage(3000));
    for (uint32_t i = 1; i < bl::kLegacySlowFrames; ++i) CHECK_FALSE(r.tx.onSlow(r.now).has_value());
    const std::optional<bl::SendStep> s = r.tx.onSlow(r.now);
    REQUIRE(s.has_value());
    CHECK(s->to == SendState::legacy);
    for (int i = 0; i < 100; ++i) r.pass();
    CHECK(r.tx.state() == SendState::legacy);
    CHECK(r.sink.begins == 0);
    r.ident(identOf(r.image), false);
    CHECK(r.tx.state() == SendState::current);
    // Slow frames with idents between them never read as legacy.
    for (int i = 0; i < 20; ++i) {
        r.tx.onSlow(r.now);
        r.ident(identOf(r.image), false);
    }
    CHECK(r.tx.state() == SendState::current);
}

TEST_CASE("an ack outside a transfer is ignored and counted") {
    Rig r(makeImage(3000));
    bl::UpdateAck a;
    const bl::SendStep s = r.tx.onAck(a, r.now);
    CHECK(s.ignored);
    CHECK(r.tx.state() == SendState::listen);
    CHECK(r.tx.ignored() == 1);
}

TEST_CASE("an image too small for its digest or too large for the index is invalid") {
    CHECK_FALSE(bl::UpdateSender(std::span<const uint8_t>()).valid());
    const Bytes tiny(bl::kDigestBytes, 0);
    CHECK_FALSE(bl::UpdateSender(tiny).valid());
    const Bytes small(bl::kDigestBytes + 1, 0);
    CHECK(bl::UpdateSender(small).valid());
}

// ---- the receiver alone -----------------------------------------------------

namespace {

bl::Frame frameOf(const Bytes& w) {
    bl::Decoder d;
    const std::vector<bl::Frame> got = feedAll(d, w);
    REQUIRE(got.size() == 1);
    return got[0];
}

bl::Frame beginFrame(const Bytes& image, std::optional<bl::Digest> digest = std::nullopt) {
    bl::UpdateBegin b;
    b.size = uint32_t(image.size());
    for (size_t k = 0; k < bl::kDigestBytes; ++k) b.digest[k] = image[image.size() - bl::kDigestBytes + k];
    if (digest) b.digest = *digest;
    return frameOf(wireOf(b, 0));
}

bl::Frame chunkFrame(const Bytes& image, uint16_t index, std::optional<size_t> len = std::nullopt) {
    const size_t at = size_t(index) * bl::kChunkBytes;
    const size_t n = len ? *len : std::min(bl::kChunkBytes, image.size() - at);
    bl::UpdateData d;
    d.index = index;
    d.bytes = std::span<const uint8_t>(image.data() + at, n);
    return frameOf(wireOf(d, 0));
}

bl::Frame bareFrame(bl::Type t) {
    std::array<uint8_t, bl::kMaxWire> out{};
    const size_t n = bl::encodeBare(t, 0, out);
    return frameOf(Bytes(out.begin(), out.begin() + long(n)));
}

}  // namespace

TEST_CASE("receiver: a chunk that is not the one wanted is re-acked, never written") {
    const Bytes image = makeImage(1000);
    FakeSink sink;
    bl::UpdateReceiver rx(sink);
    CHECK(rx.onFrame(beginFrame(image), 0, false)->next == 0);
    CHECK(rx.onFrame(chunkFrame(image, 0), 1, false)->next == 1);
    const std::optional<bl::UpdateAck> again = rx.onFrame(chunkFrame(image, 0), 2, false);
    REQUIRE(again.has_value());
    CHECK(again->code == bl::AckCode::ok);
    CHECK(again->next == 1);
    CHECK(rx.last().ev == RecvEvent::chunk_again);
    CHECK(sink.written.size() == bl::kChunkBytes);
    const std::optional<bl::UpdateAck> ahead = rx.onFrame(chunkFrame(image, 3), 3, false);
    CHECK(ahead->next == 1);
    CHECK(sink.written.size() == bl::kChunkBytes);
}

TEST_CASE("receiver: a chunk of the wrong length or past the end closes the update as malformed") {
    const Bytes image = makeImage(1000);
    FakeSink sink;
    bl::UpdateReceiver rx(sink);
    rx.onFrame(beginFrame(image), 0, false);
    std::optional<bl::UpdateAck> a = rx.onFrame(chunkFrame(image, 0, 100), 1, false);
    CHECK(a->code == bl::AckCode::malformed);
    CHECK(rx.state() == RecvState::idle);
    CHECK(sink.aborts == 1);
    rx.onFrame(beginFrame(image), 2, false);
    const Bytes big = makeImage(5000);
    a = rx.onFrame(chunkFrame(big, 5), 3, false);   // index 5 of a 5-chunk image
    CHECK(a->code == bl::AckCode::malformed);
}

TEST_CASE("receiver: END before every byte, or with a tail not the digest begun, keeps nothing") {
    const Bytes image = makeImage(1000);
    FakeSink sink;
    bl::UpdateReceiver rx(sink);
    rx.onFrame(beginFrame(image), 0, false);
    rx.onFrame(chunkFrame(image, 0), 1, false);
    std::optional<bl::UpdateAck> a = rx.onFrame(bareFrame(bl::Type::update_end), 2, false);
    CHECK(a->code == bl::AckCode::short_image);
    CHECK(rx.state() == RecvState::idle);
    CHECK(sink.finished == 0);

    bl::Digest wrong{};
    wrong[0] = 0x55;
    rx.onFrame(beginFrame(image, wrong), 3, false);
    for (uint16_t i = 0; i < bl::chunkCount(uint32_t(image.size())); ++i) rx.onFrame(chunkFrame(image, i), 4, false);
    a = rx.onFrame(bareFrame(bl::Type::update_end), 5, false);
    CHECK(a->code == bl::AckCode::digest);
    CHECK(sink.finished == 0);
    CHECK(rx.state() == RecvState::idle);
}

TEST_CASE("receiver: data, END or ABORT with nothing open") {
    const Bytes image = makeImage(1000);
    FakeSink sink;
    bl::UpdateReceiver rx(sink);
    CHECK(rx.onFrame(chunkFrame(image, 0), 0, false)->code == bl::AckCode::no_session);
    CHECK(rx.onFrame(bareFrame(bl::Type::update_end), 0, false)->code == bl::AckCode::no_session);
    CHECK_FALSE(rx.onFrame(bareFrame(bl::Type::update_abort), 0, false).has_value());
    CHECK(sink.written.empty());
}

TEST_CASE("receiver: a BEGIN mid-update starts over; a malformed size is refused") {
    const Bytes image = makeImage(1000);
    FakeSink sink;
    bl::UpdateReceiver rx(sink);
    rx.onFrame(beginFrame(image), 0, false);
    rx.onFrame(chunkFrame(image, 0), 1, false);
    CHECK(rx.onFrame(beginFrame(image), 2, false)->next == 0);
    CHECK(sink.aborts == 1);
    CHECK(sink.begins == 2);
    CHECK(rx.last().act == RecvAct::reopen);
    const Bytes tiny(bl::kDigestBytes, 1);
    bl::UpdateBegin b;
    b.size = uint32_t(tiny.size());
    const std::optional<bl::UpdateAck> a = rx.onFrame(frameOf(wireOf(b, 0)), 3, false);
    CHECK(a->code == bl::AckCode::malformed);
    CHECK(rx.state() == RecvState::idle);
}

TEST_CASE("receiver: an update left silent kReceiverIdleMs is dropped") {
    const Bytes image = makeImage(1000);
    FakeSink sink;
    bl::UpdateReceiver rx(sink);
    rx.onFrame(beginFrame(image), 100, false);
    rx.tick(100 + bl::kReceiverIdleMs - 1);
    CHECK(rx.state() == RecvState::receiving);
    rx.tick(100 + bl::kReceiverIdleMs);
    CHECK(rx.state() == RecvState::idle);
    CHECK(sink.aborts == 1);
}

TEST_CASE("receiver: done takes nothing more; non-update frames and KEEP are not its own") {
    const Bytes image = makeImage(300);
    FakeSink sink;
    bl::UpdateReceiver rx(sink);
    rx.onFrame(beginFrame(image), 0, false);
    rx.onFrame(chunkFrame(image, 0), 0, false);
    rx.onFrame(chunkFrame(image, 1), 0, false);
    CHECK(rx.onFrame(bareFrame(bl::Type::update_end), 0, false)->code == bl::AckCode::done);
    CHECK(rx.state() == RecvState::done);
    CHECK_FALSE(rx.onFrame(beginFrame(image), 0, false).has_value());
    CHECK(sink.begins == 1);
    bl::Command c;
    CHECK_FALSE(rx.onFrame(frameOf(wireOf(c, 0)), 0, false).has_value());
    bl::UpdateKeep k;
    CHECK_FALSE(rx.onFrame(frameOf(wireOf(k, 0)), 0, false).has_value());
}

// ---- the trial --------------------------------------------------------------

TEST_CASE("trial: kept by a KEEP for itself, deaf to another's, reverted at kTrialMs") {
    const bl::IdentDigest mine{1, 2, 3, 4, 5, 6, 7, 8};
    bl::UpdateKeep other;
    other.digest = {8, 7, 6, 5, 4, 3, 2, 1};
    bl::UpdateKeep ours;
    ours.digest = mine;

    bl::TrialGate g(true, mine);
    CHECK(g.onTrial());
    CHECK(g.onKeep(other).act == TrialAct::none);
    CHECK(g.state() == TrialState::trial);
    CHECK_FALSE(g.tick(bl::kTrialMs - 1).has_value());
    const bl::TrialStep k = g.onKeep(ours);
    CHECK(k.act == TrialAct::mark_valid);
    CHECK(g.state() == TrialState::kept);
    CHECK_FALSE(g.onTrial());
    CHECK_FALSE(g.tick(bl::kTrialMs * 2).has_value());
    CHECK(g.onKeep(ours).act == TrialAct::none);

    bl::TrialGate late(true, mine);
    const std::optional<bl::TrialStep> d = late.tick(bl::kTrialMs);
    REQUIRE(d.has_value());
    CHECK(d->act == TrialAct::revert);
    CHECK(late.state() == TrialState::reverting);
    CHECK(late.onKeep(ours).act == TrialAct::none);
    CHECK(late.onTrial());

    bl::TrialGate usb(false, mine);
    CHECK_FALSE(usb.onTrial());
    CHECK_FALSE(usb.tick(bl::kTrialMs).has_value());
    CHECK(usb.onKeep(ours).act == TrialAct::none);
}
