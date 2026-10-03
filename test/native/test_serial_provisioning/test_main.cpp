// test_serial_provisioning -- native doctest suite for the USB serial binding
// (SPEC §13.5) and the provisioning desk (SPEC §13.9) on a real Hub
// Constraints:
// - ValenceSerialPort.h and ValenceProvisioning.h are compiled verbatim, the
//   hub and client are the library's; the byte pipe, the station and the hub
//   delegate are the fakes below, never the board.
// - The catalog here declares core 0x000F as the registry describes it
//   (registry.yaml core_channels 0x000F). The machine's own catalog does not
//   declare it yet (bd val-9u0.21); until it does, the board's desk declines.
// - Geiger runs its host branch here so every line the desk and the port
//   log is captured and searched for the credentials.
// See: flagship_p4/src/hub/ValenceSerialPort.h, ValenceProvisioning.h

#define GEIGER_HOST_PLATFORM 1

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <algorithm>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// Named directly so the library finder adds lib/valence and lib/geiger; it
// does not follow the relative includes below.
#include "geiger/geiger.h"
#include "valence/client/client.hpp"
#include "valence/core/clock.hpp"
#include "valence/core/rng.hpp"
#include "valence/hub/hub.hpp"
#include "valence/transport/inprocess_binding.hpp"
#include "valence/wire/estop_frame.hpp"

#include "../../../flagship_p4/src/hub/ValenceProvisioning.h"
#include "../../../flagship_p4/src/hub/ValenceSerialPort.h"

using namespace valence;

namespace {

ManualClock g_clock{1000000};

// Every Geiger line the code under test emits, as text.
class CaptureSink final : public geiger::ISink {
public:
    std::vector<std::string> lines;
    void write(const geiger::Record& r) override { lines.emplace_back(std::string(r.tag) + " " + r.msg); }
};
CaptureSink g_logs;
const bool g_sinkAdded = geiger::logger().addSink(&g_logs);

bool contains(const std::vector<std::byte>& hay, std::string_view needle) {
    if (needle.empty() || hay.size() < needle.size()) return false;
    const auto* first = reinterpret_cast<const char*>(hay.data());
    return std::search(first, first + hay.size(), needle.begin(), needle.end()) != first + hay.size();
}

bool logged(std::string_view needle) {
    return std::any_of(g_logs.lines.begin(), g_logs.lines.end(),
                       [&](const std::string& l) { return l.find(needle) != std::string::npos; });
}

std::vector<std::byte> bytesOf(std::string_view s) {
    std::vector<std::byte> out;
    for (char c : s) out.push_back(std::byte(uint8_t(c)));
    return out;
}

// A frame with `len` payload bytes, header len set, payload `fill`.
std::vector<std::byte> frameOf(uint8_t type, size_t len, uint8_t fill) {
    std::vector<std::byte> f(kHeaderBytes + len, std::byte{fill});
    FrameHeader h;
    h.type = type;
    h.channel = 0x1234;
    h.len = uint16_t(len);
    encodeFrameHeader(h, f);
    return f;
}

std::vector<std::byte> serialOf(std::span<const std::byte> frame) {
    std::vector<std::byte> out(kSerialMaxEncoded);
    const size_t n = encodeSerialFrame(frame, out);
    out.resize(n);
    return out;
}

std::vector<std::vector<std::byte>> deframe(CobsDeframer& d, std::span<const std::byte> stream) {
    std::vector<std::vector<std::byte>> out;
    d.feed(stream, [&](const FrameBuffer& f) { out.emplace_back(f.bytes().begin(), f.bytes().end()); });
    return out;
}

// ---- the byte pipe, both ends -----------------------------------------------------

struct Wire {
    std::deque<std::byte> toHub;
    std::deque<std::byte> toClient;
    std::vector<std::byte> sentByHub;   // everything the hub end ever wrote
    bool refuseHubWrites = false;
};

class HubEnd final : public ISerialPipe {
public:
    explicit HubEnd(Wire& w) : _w(w) {}
    size_t read(std::span<std::byte> out) override {
        size_t n = 0;
        while (n < out.size() && !_w.toHub.empty()) {
            out[n++] = _w.toHub.front();
            _w.toHub.pop_front();
        }
        return n;
    }
    bool write(std::span<const std::byte> b) override {
        if (_w.refuseHubWrites) return false;
        _w.toClient.insert(_w.toClient.end(), b.begin(), b.end());
        _w.sentByHub.insert(_w.sentByHub.end(), b.begin(), b.end());
        return true;
    }

private:
    Wire& _w;
};

// The client's end of a serial link: the same framing, the other direction.
class ClientLink final : public ITransport {
public:
    explicit ClientLink(Wire& w) : _w(w) {}
    bool open() override { return true; }
    void close() override {}
    bool write(std::span<const std::byte> frame) override {
        const std::vector<std::byte> enc = serialOf(frame);
        if (enc.empty()) return false;
        _w.toHub.insert(_w.toHub.end(), enc.begin(), enc.end());
        return true;
    }
    std::optional<FrameBuffer> read() override {
        while (_q.empty() && !_w.toClient.empty()) {
            const std::byte b = _w.toClient.front();
            _w.toClient.pop_front();
            _d.feed(std::span<const std::byte>(&b, 1), [this](const FrameBuffer& f) { _q.push_back(f); });
        }
        if (_q.empty()) return std::nullopt;
        FrameBuffer f = _q.front();
        _q.pop_front();
        return f;
    }
    TransportProperties properties() const override {
        TransportProperties p;
        p.mtu = uint16_t(kFrameBufferCapacity);
        p.ordered = true;
        p.reliable = true;
        return p;
    }

private:
    Wire& _w;
    CobsDeframer _d{};
    std::deque<FrameBuffer> _q;
};

// ---- the station, the delegate, the client's ears -----------------------------------

class FakeStation final : public IStation {
public:
    int joins = 0;
    int persists = 0;
    bool startable = true;
    bool persistNow = true;
    std::string lastSsid, lastPass, savedSsid, savedPass;
    std::optional<JoinOutcome> outcome;   // the test finishes a join by setting it

    bool beginJoin(std::string_view s, std::string_view p) override {
        if (!startable) return false;
        ++joins;
        lastSsid = s;
        lastPass = p;
        return true;
    }
    std::optional<JoinOutcome> pollJoin() override { return std::exchange(outcome, std::nullopt); }
    bool persist(std::string_view s, std::string_view p) override {
        if (!persistNow) return false;
        ++persists;
        savedSsid = s;
        savedPass = p;
        return true;
    }
};

// What ValenceDevice answers a 0x000F INTENT that reaches the hub: refused.
// It can only arrive on a binding that is not a provisioning path.
class TestDelegate final : public HubDelegate {
public:
    int estops = 0;
    Result<IntentValueMap, NackCode> applyIntent(uint16_t ch, const IntentValueMap&, AccessLevel, bool&) override {
        return Result<IntentValueMap, NackCode>::err(ch == channels::provisioning ? NackCode::ACCESS_DENIED
                                                                                   : NackCode::UNSUPPORTED_OP);
    }
    void onEstop(uint8_t, uint8_t) override { ++estops; }
};

struct Heard {
    NackCode code = NackCode::MALFORMED;
    uint16_t intentId = 0;
    bool hasIntentId = false;
    std::string detail;
    uint32_t retryAfterMs = 0;
};

class Ears final : public ClientDelegate {
public:
    std::vector<Heard> nacks;
    std::vector<std::pair<uint16_t, IntentValueMap>> echoes;
    // The hub upgrades the session in place on PAIR_GRANT; the library client
    // reports it here and nowhere else.
    AccessLevel granted = AccessLevel::watch;
    void onPairGrant(std::span<const std::byte>, AccessLevel roles) override { granted = roles; }
    void onStateChange(ClientSessionState) override {}
    void onState(uint16_t, uint16_t, std::span<const std::byte>) override {}
    void onEcho(uint16_t id, const IntentValueMap& applied, uint16_t) override { echoes.emplace_back(id, applied); }
    void onNack(const NackMsg& n) override {
        nacks.push_back({n.code, n.intent_id, n.has_intent_id, n.has_detail ? std::string(n.detail) : std::string(),
                         n.has_retry_after_ms ? n.retry_after_ms : 0});
    }
    void onPendingDropped(uint16_t) override {}
};

// ---- the catalog ---------------------------------------------------------------------

bool buildTestCatalog(Catalog32& c, bool declareProvisioning) {
    c.clear();
    if (declareProvisioning) {
        c.addEntry({.id = channels::provisioning, .name = "provisioning", .cls = ChannelClass::INTENT,
                    .dir = Direction::c2h, .access = AccessLevel::configure, .maxRateHz = 1.0f,
                    .defaultPriority = Priority::normal, .hasCategory = true, .category = ui_categories::setup});
        c.addSelectSchemaField({.key = 1, .name = "op", .type = CborFieldType::uint_t, .unit = "",
                                .role = "action.provision"},
                               {"reserved", "wifi_join"});
        c.addSchemaField({.key = 2, .name = "ssid", .type = CborFieldType::tstr_t, .unit = "",
                          .flags = setting_flags::secret});
        c.addSchemaField({.key = 3, .name = "passphrase", .type = CborFieldType::tstr_t, .unit = "",
                          .flags = setting_flags::secret});
        c.addSchemaField({.key = 4, .name = "ipv4", .type = CborFieldType::uint_t, .unit = ""});
        c.addSchemaField({.key = 5, .name = "ws_port", .type = CborFieldType::uint_t, .unit = ""});
    }
    // A second entry, so the undeclared catalog is not empty.
    c.addEntry({.id = 0x3000, .name = "config-set", .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 10.0f, .defaultPriority = Priority::normal});
    c.addSchemaField({.key = 1, .name = "max_rail", .type = CborFieldType::f32_t, .unit = "mm"});
    return c.ok();
}

ClientIdentity clientId(uint8_t who) {
    ClientIdentity id;
    id.instance_id.fill(std::byte{0});
    id.instance_id[0] = std::byte{who};
    id.client_kind = "provision";
    id.client_name = "native-test";
    return id;
}

IntentValueMap wifiJoin(std::string_view ssid, std::string_view pass, uint64_t op = provisioning_ops::wifi_join) {
    IntentValueMap m{};
    m.count = 3;
    m.fields[0] = IntentValueField{1, IntentValue::ofU64(op)};
    m.fields[1] = IntentValueField{2, IntentValue::ofTstr(ssid)};
    m.fields[2] = IntentValueField{3, IntentValue::ofTstr(pass)};
    return m;
}

// A raw INTENT on 0x000F with a chosen intent_id, past the library client.
std::vector<std::byte> rawJoin(uint16_t intentId, std::string_view ssid, std::string_view pass) {
    IntentMsg m{};
    m.channel_id = channels::provisioning;
    m.intent_id = intentId;
    const IntentValueMap v = wifiJoin(ssid, pass);
    m.value_count = v.count;
    m.value = v.fields;
    std::vector<std::byte> f(kHeaderBytes + 300);
    const size_t n = encodeIntent(m, std::span<std::byte>(f).subspan(kHeaderBytes));
    FrameHeader h;
    h.type = uint8_t(FrameType::INTENT);
    h.channel = channels::provisioning;
    h.seq = intentId;
    h.len = uint16_t(n);
    encodeFrameHeader(h, f);
    f.resize(kHeaderBytes + n);
    return f;
}

constexpr std::string_view kSsid = "HomeNet-5G";
constexpr std::string_view kPass = "correct horse battery";
constexpr uint32_t kIp = 0xC0A80132;   // 192.168.1.50

// One serial binding on the hub plus its client. Two of these share a hub in
// the both-bindings case.
struct Link {
    Wire wire{};
    HubEnd hubEnd{wire};
    ValenceSerialPort port{};
    ClientLink clientEnd{wire};
    XorShift32 rng;
    Ears ears{};
    std::optional<Client> client{};
    explicit Link(uint32_t seed) : rng(seed) {}
};

// Heap, not stack: the catalog and the hub are tens of KB each.
struct Rig {
    Catalog32 catalog{};
    XorShift32 hubRng{4242};
    TestDelegate del{};
    std::optional<Hub> hub{};
    FakeStation station{};
    std::optional<Provisioning> desk{};
    std::vector<std::unique_ptr<Link>> links;

    explicit Rig(bool declare = true, int bindings = 1) {
        REQUIRE(buildTestCatalog(catalog, declare));
        hub.emplace(catalog, g_clock, hubRng, del);
        desk.emplace(*hub, catalog.find(channels::provisioning), station, uint16_t(82));
        for (int i = 0; i < bindings; ++i) {
            links.push_back(std::make_unique<Link>(uint32_t(100 + i)));
            Link& l = *links.back();
            l.port.begin(*hub, l.hubEnd, &*desk);
            l.client.emplace(clientId(uint8_t(7 + i)), l.clientEnd, g_clock, l.rng, l.ears);
        }
    }

    Link& at(size_t i = 0) { return *links[i]; }
    Client& client(size_t i = 0) { return *links[i]->client; }

    void step(int rounds = 20, bool clientsRun = true) {
        for (int r = 0; r < rounds; ++r) {
            g_clock.advanceUs(5000);
            const uint32_t ms = g_clock.nowUs() / 1000;
            for (auto& l : links) l->port.loop(ms);
            hub->update(g_clock.nowUs());
            desk->tick();
            if (clientsRun)
                for (auto& l : links) l->client->update(g_clock.nowUs());
            geiger::drainToSinks();
        }
    }

    void live(size_t i = 0) {
        REQUIRE(client(i).connect());
        step(120);
        REQUIRE(client(i).state() == ClientSessionState::LIVE);
    }

    // The §12.3 single-grant window: the knock takes it, and with it the
    // window itself (hub_impl.hpp handleKnock).
    void configure(size_t i = 0) {
        hub->openPresenceWindow();
        REQUIRE(client(i).sendPairKnock());
        step();
        REQUIRE(at(i).ears.granted == AccessLevel::configure);
    }

    void finish(JoinOutcome o) {
        station.outcome = o;
        step();
    }
};

}  // namespace

namespace geiger {
uint32_t hostNowMs() { return g_clock.nowUs() / 1000; }
}  // namespace geiger

// ---- framing ----------------------------------------------------------------------

TEST_CASE("framing: COBS bytes match SPEC 13.5, delimiters outside") {
    const std::vector<std::byte> f = {std::byte{0x11}, std::byte{0x22}, std::byte{0x00}, std::byte{0x33}};
    const std::vector<std::byte> want = {std::byte{0x00}, std::byte{0x03}, std::byte{0x11}, std::byte{0x22},
                                         std::byte{0x02}, std::byte{0x33}, std::byte{0x00}};
    CHECK(serialOf(f) == want);
}

TEST_CASE("framing: round trip at the sizes COBS treats specially") {
    for (const size_t len : {size_t(0), size_t(1), size_t(245), size_t(246), size_t(247), size_t(504)}) {
        for (const uint8_t fill : {uint8_t(0x00), uint8_t(0x5A)}) {
            const auto f = frameOf(uint8_t(FrameType::STATE), len, fill);
            CobsDeframer d;
            const auto got = deframe(d, serialOf(f));
            REQUIRE(got.size() == 1);
            CHECK(got[0] == f);
            CHECK(d.dropped() == 0);
        }
    }
}

TEST_CASE("framing: console text between frames is dropped and the frames survive") {
    const auto a = frameOf(uint8_t(FrameType::WELCOME), 20, 0x41);
    const auto b = frameOf(uint8_t(FrameType::ECHO), 7, 0x00);
    std::vector<std::byte> stream = bytesOf("I (1234) wifi: connected\r\n");
    for (auto x : serialOf(a)) stream.push_back(x);
    for (auto x : bytesOf("[    12.345 I1 hub       ] persisted cfg, 86 B, 812 us\n")) stream.push_back(x);
    for (auto x : serialOf(b)) stream.push_back(x);
    CobsDeframer d;
    const auto got = deframe(d, stream);
    REQUIRE(got.size() == 2);
    CHECK(got[0] == a);
    CHECK(got[1] == b);
    CHECK(d.dropped() == 2);
}

TEST_CASE("framing: resync after garbage, a torn frame and a byte-at-a-time feed") {
    const auto good = frameOf(uint8_t(FrameType::PING), 4, 0x07);
    CobsDeframer d;
    // Two kilobytes with no delimiter: longer than any frame, so unsynced.
    std::vector<std::byte> junk(2048, std::byte{0x55});
    CHECK(deframe(d, junk).empty());
    CHECK(deframe(d, serialOf(good)).size() == 1);

    // A frame whose header claims more than it carries is not a frame.
    auto torn = serialOf(good);
    torn.erase(torn.end() - 3);
    CHECK(deframe(d, torn).empty());
    const uint32_t dropped = d.dropped();
    CHECK(dropped >= 2);

    const auto enc = serialOf(good);
    std::vector<std::vector<std::byte>> got;
    for (const std::byte b : enc) {
        auto one = deframe(d, std::span<const std::byte>(&b, 1));
        got.insert(got.end(), one.begin(), one.end());
    }
    REQUIRE(got.size() == 1);
    CHECK(got[0] == good);
}

TEST_CASE("framing: a raw ESTOP inside junk is still found (SPEC 13.5)") {
    // A seq whose 12 bytes hold no 0x00, so the raw frame survives as one run.
    std::array<std::byte, kEstopFrameBytes> raw{};
    for (uint16_t seq = 0x0101;; ++seq) {
        encodeEstop(EstopFrame{1, 1, seq}, raw);
        if (std::none_of(raw.begin(), raw.end(), [](std::byte b) { return b == std::byte{0}; })) break;
    }
    std::vector<std::byte> stream = bytesOf("garbage before ");
    stream.insert(stream.end(), raw.begin(), raw.end());
    for (auto x : bytesOf(" and after")) stream.push_back(x);
    stream.push_back(std::byte{0});
    CobsDeframer d;
    const auto got = deframe(d, stream);
    REQUIRE(got.size() == 1);
    CHECK(std::equal(got[0].begin(), got[0].end(), raw.begin(), raw.end()));
}

// ---- the port on a hub ----------------------------------------------------------------

TEST_CASE("port: a session runs over the link and shrugs off console text") {
    auto rig = std::make_unique<Rig>();
    rig->live();
    CHECK(rig->at().port.attached());
    CHECK(rig->at().port.sessionId() == rig->client().sessionId());
    for (auto x : bytesOf("I (99) boot: the console shares this pipe\r\n")) rig->at().wire.toClient.push_back(x);
    rig->step(40);
    CHECK(rig->client().state() == ClientSessionState::LIVE);
    CHECK(rig->at().port.junkChunks() == 0);   // the hub never sees the client's console
}

TEST_CASE("port: two sessions back to back without a reboot (T3)") {
    auto rig = std::make_unique<Rig>();
    rig->live();
    const uint32_t first = rig->client().sessionId();
    rig->client().disconnect();
    rig->step(40);
    Link& l = rig->at();
    l.client.reset();
    l.client.emplace(clientId(9), l.clientEnd, g_clock, l.rng, l.ears);
    rig->live();
    CHECK(rig->client().sessionId() != first);
    CHECK(l.port.sessionId() == rig->client().sessionId());
}

TEST_CASE("port: a silent link is reaped, and the next frame attaches it again") {
    auto rig = std::make_unique<Rig>();
    rig->live();
    rig->step(int(ValenceSerialPort::kIdleReapMs / 5) + 20, /*clientsRun=*/false);
    CHECK_FALSE(rig->at().port.attached());
    CHECK(rig->at().port.sessionId() == 0);
    rig->step(20);
    CHECK(rig->at().port.attached());
}

TEST_CASE("port: an ESTOP latches even when every hub slot is taken (SPEC 6.3)") {
    auto rig = std::make_unique<Rig>();
    XorShift32 rng{9};
    // Transport capacity is sessions + 1 (SPEC 6.3); fill all of it.
    std::vector<std::unique_ptr<InProcessLink>> others;
    for (int i = 0; i < 16; ++i) {
        others.push_back(std::make_unique<InProcessLink>(g_clock, rng));
        if (!rig->hub->attachTransport(others.back()->endpointA())) break;
    }
    REQUIRE(others.size() == kHubMaxSessions + 2);
    std::array<std::byte, kEstopFrameBytes> estop{};
    encodeEstop(EstopFrame{1, 0, 1}, estop);
    for (auto x : serialOf(estop)) rig->at().wire.toHub.push_back(x);
    rig->step(2, /*clientsRun=*/false);
    CHECK_FALSE(rig->at().port.attached());
    CHECK(rig->del.estops == 1);
}

TEST_CASE("both provisioning bindings live at once on one hub, one join at a time") {
    // Link 1 stands in for BLE GATT, the board's second provisioning binding
    // (bd val-9u0.3): same desk, same hub, its own session.
    auto rig = std::make_unique<Rig>(true, 2);
    rig->live(0);
    rig->live(1);
    CHECK(rig->client(0).state() == ClientSessionState::LIVE);
    CHECK(rig->client(1).state() == ClientSessionState::LIVE);
    rig->configure(0);
    // A configure token exists now, so this window grants `control`; the
    // operator approving link 1 as configure stands in for a second admin.
    rig->hub->setPresenceDefaultRole(AccessLevel::configure);
    rig->configure(1);

    rig->hub->openPresenceWindow();
    REQUIRE(rig->client(0).sendIntent(channels::provisioning, wifiJoin(kSsid, kPass)));
    rig->step();
    CHECK(rig->station.joins == 1);
    REQUIRE(rig->client(1).sendIntent(channels::provisioning, wifiJoin("Other", "another passphrase")));
    rig->step();
    REQUIRE(rig->at(1).ears.nacks.size() == 1);
    CHECK(rig->at(1).ears.nacks[0].code == NackCode::BUSY);
    CHECK(rig->at(1).ears.nacks[0].retryAfterMs > 0);
    CHECK(rig->station.joins == 1);

    rig->finish({true, kIp, JoinFailure::timeout});
    CHECK(rig->at(0).ears.echoes.size() == 1);
    CHECK(rig->at(1).ears.echoes.empty());
}

// ---- the desk ----------------------------------------------------------------------------

TEST_CASE("desk: with 0x000F undeclared the hub answers, and no join starts") {
    auto rig = std::make_unique<Rig>(/*declare=*/false);
    rig->live();
    rig->configure();
    REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin(kSsid, kPass)));
    rig->step();
    REQUIRE(rig->at().ears.nacks.size() == 1);
    CHECK(rig->at().ears.nacks[0].code == NackCode::UNKNOWN_CHANNEL);
    CHECK(rig->station.joins == 0);
}

TEST_CASE("desk: a session that has not declared its catalog is refused NOT_READY") {
    auto rig = std::make_unique<Rig>();
    REQUIRE(rig->client().connect());
    // WELCOME lands; the catalog transfer and CATALOG_READY have not.
    for (int i = 0; i < 40 && rig->at().port.sessionId() == 0; ++i) rig->step(1, true);
    REQUIRE(rig->at().port.sessionId() != 0);
    rig->at().clientEnd.write(rawJoin(40, kSsid, kPass));
    rig->step(2, /*clientsRun=*/false);
    rig->step(1);
    REQUIRE_FALSE(rig->at().ears.nacks.empty());
    CHECK(rig->at().ears.nacks[0].code == NackCode::NOT_READY);
    CHECK(rig->station.joins == 0);
}

TEST_CASE("desk: a watch session is refused ACCESS_DENIED") {
    auto rig = std::make_unique<Rig>();
    rig->live();
    rig->hub->openPresenceWindow();
    REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin(kSsid, kPass)));
    rig->step();
    REQUIRE(rig->at().ears.nacks.size() == 1);
    CHECK(rig->at().ears.nacks[0].code == NackCode::ACCESS_DENIED);
    CHECK(rig->station.joins == 0);
}

TEST_CASE("desk: the knock that granted configure closed the window, so the join waits for PAIR") {
    // SPEC 13.9's gate read literally: no window open, and a configure token
    // now exists. A second PAIR press is what lets this session provision.
    auto rig = std::make_unique<Rig>();
    rig->live();
    rig->configure();
    REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin(kSsid, kPass)));
    rig->step();
    REQUIRE(rig->at().ears.nacks.size() == 1);
    CHECK(rig->at().ears.nacks[0].code == NackCode::ACCESS_DENIED);
    CHECK(rig->at().ears.nacks[0].detail == "pairing window closed");
    CHECK(rig->station.joins == 0);

    rig->hub->openPresenceWindow();
    REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin(kSsid, kPass)));
    rig->step();
    CHECK(rig->station.joins == 1);
}

TEST_CASE("desk: a bad SSID or passphrase is refused INVALID_VALUE and starts nothing") {
    auto rig = std::make_unique<Rig>();
    rig->live();
    rig->configure();
    rig->hub->openPresenceWindow();
    struct Case {
        std::string ssid, pass, detail;
    };
    const std::vector<Case> cases = {
        {"", std::string(kPass), "ssid length"},
        {std::string(33, 'x'), std::string(kPass), "ssid length"},
        {std::string(kSsid), "short12", "passphrase length"},
        {std::string(kSsid), std::string(65, 'p'), "passphrase length"},
        {std::string(kSsid), std::string(63, 'p') + "z", "passphrase"},   // 64 but not hex
    };
    for (const Case& c : cases) {
        rig->at().ears.nacks.clear();
        REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin(c.ssid, c.pass)));
        rig->step();
        REQUIRE(rig->at().ears.nacks.size() == 1);
        CHECK(rig->at().ears.nacks[0].code == NackCode::INVALID_VALUE);
        CHECK(rig->at().ears.nacks[0].detail == c.detail);
    }
    rig->at().ears.nacks.clear();
    REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin(kSsid, kPass, 2)));
    rig->step();
    REQUIRE(rig->at().ears.nacks.size() == 1);
    CHECK(rig->at().ears.nacks[0].code == NackCode::UNSUPPORTED_OP);
    CHECK(rig->station.joins == 0);
}

TEST_CASE("desk: a join that worked echoes true for both secrets, the address and the port; nothing discloses them") {
    g_logs.lines.clear();
    auto rig = std::make_unique<Rig>();
    rig->live();
    rig->configure();
    rig->hub->openPresenceWindow();
    REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin(kSsid, kPass)));
    rig->step();
    REQUIRE(rig->station.joins == 1);
    CHECK(rig->station.lastSsid == kSsid);
    CHECK(rig->station.lastPass == kPass);
    CHECK(rig->at().ears.echoes.empty());   // deferred until the join concludes

    rig->finish({true, kIp, JoinFailure::timeout});
    REQUIRE(rig->at().ears.echoes.size() == 1);
    const IntentValueMap& a = rig->at().ears.echoes[0].second;
    REQUIRE(a.count == 5);
    CHECK(a.fields[0].key == 1);
    CHECK(a.fields[0].value.u64_val == provisioning_ops::wifi_join);
    for (const size_t i : {size_t(1), size_t(2)}) {
        CHECK(a.fields[i].key == i + 1);
        CHECK(a.fields[i].value.kind == IntentValue::Kind::Bool);
        CHECK(a.fields[i].value.bool_val);
    }
    CHECK(a.fields[3].key == 4);
    CHECK(a.fields[3].value.u64_val == kIp);
    CHECK(a.fields[4].key == 5);
    CHECK(a.fields[4].value.u64_val == 82);

    CHECK(rig->station.persists == 1);
    CHECK(rig->station.savedSsid == kSsid);
    CHECK(rig->station.savedPass == kPass);
    CHECK_FALSE(rig->desk->busy());

    CHECK_FALSE(contains(rig->at().wire.sentByHub, kSsid));
    CHECK_FALSE(contains(rig->at().wire.sentByHub, kPass));
    CHECK(logged("joined: 192.168.1.50"));
    CHECK_FALSE(logged(kSsid));
    CHECK_FALSE(logged(kPass));
}

TEST_CASE("desk: a join that failed answers NETWORK_JOIN_FAILED without the credentials, and persists nothing") {
    g_logs.lines.clear();
    auto rig = std::make_unique<Rig>();
    rig->live();
    rig->configure();
    rig->hub->openPresenceWindow();
    REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin(kSsid, kPass)));
    rig->step();
    rig->finish({false, 0, JoinFailure::auth});
    REQUIRE(rig->at().ears.nacks.size() == 1);
    CHECK(rig->at().ears.nacks[0].code == NackCode::NETWORK_JOIN_FAILED);
    CHECK(rig->at().ears.nacks[0].detail == "authentication failed");
    CHECK(rig->station.persists == 0);
    CHECK_FALSE(contains(rig->at().wire.sentByHub, kSsid));
    CHECK_FALSE(contains(rig->at().wire.sentByHub, kPass));
    CHECK_FALSE(logged(kSsid));
    CHECK_FALSE(logged(kPass));
}

TEST_CASE("desk: a duplicate intent_id joins the attempt, and after it gets the same ECHO") {
    auto rig = std::make_unique<Rig>();
    rig->live();
    rig->configure();
    rig->hub->openPresenceWindow();
    rig->at().clientEnd.write(rawJoin(77, kSsid, kPass));
    rig->step();
    rig->at().clientEnd.write(rawJoin(77, kSsid, kPass));
    rig->step();
    CHECK(rig->station.joins == 1);
    CHECK(rig->at().ears.nacks.empty());

    rig->finish({true, kIp, JoinFailure::timeout});
    rig->at().clientEnd.write(rawJoin(77, kSsid, kPass));
    rig->step();
    CHECK(rig->station.joins == 1);
    // The library client correlates only its own pending ids; count the ECHO
    // frames on the wire instead.
    size_t echoes = 0;
    CobsDeframer d;
    d.feed(rig->at().wire.sentByHub, [&](const FrameBuffer& f) {
        if (f.size() >= kHeaderBytes && uint8_t(f.bytes()[0]) == uint8_t(FrameType::ECHO)) ++echoes;
    });
    CHECK(echoes == 2);
}

TEST_CASE("desk: a store that cannot write yet keeps the desk busy, then persists once") {
    auto rig = std::make_unique<Rig>();
    rig->live();
    rig->configure();
    rig->hub->openPresenceWindow();
    rig->station.persistNow = false;
    REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin(kSsid, kPass)));
    rig->step();
    rig->finish({true, kIp, JoinFailure::timeout});
    CHECK(rig->desk->busy());
    REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin("Other", "another passphrase")));
    rig->step();
    REQUIRE_FALSE(rig->at().ears.nacks.empty());
    CHECK(rig->at().ears.nacks.back().code == NackCode::BUSY);
    rig->station.persistNow = true;
    rig->step();
    CHECK(rig->station.persists == 1);
    CHECK(rig->station.savedSsid == kSsid);
    CHECK_FALSE(rig->desk->busy());
}

TEST_CASE("desk: a station that cannot start a join answers station down") {
    auto rig = std::make_unique<Rig>();
    rig->live();
    rig->configure();
    rig->hub->openPresenceWindow();
    rig->station.startable = false;
    REQUIRE(rig->client().sendIntent(channels::provisioning, wifiJoin(kSsid, kPass)));
    rig->step();
    REQUIRE(rig->at().ears.nacks.size() == 1);
    CHECK(rig->at().ears.nacks[0].code == NackCode::NETWORK_JOIN_FAILED);
    CHECK(rig->at().ears.nacks[0].detail == "station down");
    CHECK_FALSE(rig->desk->busy());
}
