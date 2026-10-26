// test_valence_device -- native doctest suite for the delegate's safety seams with the hub
// Constraints:
// - ValenceDevice.cpp is compiled verbatim into this one translation unit,
//   driven by a real Hub over InProcessLink. The motion, pattern, motor
//   switch, button and e-stop doors are fakes below: the composition's seam,
//   never the board. The datagram switch is one of them; what it does when
//   off is test_estop_datagram's.
// - The capacity macros are defined here, before any Valence include, and this
//   binary has no other translation unit that sees Catalog32. They only need
//   to hold the machine's own catalog; the board's values live in
//   flagship_p4/valence_capacity.cmake and are not restated here.
// See: Valence SPEC.md §5.5, §11.2, §16.1; bd val-091.23, val-091.56,
// rfc-1ek, rfc-2w2, Valence RFC-095

#define VALENCE_CATALOG_ENTRIES 64
#define VALENCE_CATALOG_LAYOUT_FIELDS 320
#define VALENCE_CATALOG_SCHEMA_FIELDS 192
#define VALENCE_CATALOG_LABELS 256
#define NUCLEUS_ACCESSORIES 0
#define NUCLEUS_ACCESSORY_ENTRIES 1
#define NUCLEUS_ACCESSORY_LAYOUT_FIELDS 1
#define NUCLEUS_ACCESSORY_SCHEMA_FIELDS 1
#define NUCLEUS_ACCESSORY_SAFE_FIELDS 1
#define NUCLEUS_ACCESSORY_CATALOG_BYTES 1

// Geiger's host layer, so the GLOG lines ValenceDevice.cpp writes reach a
// sink here (VD-HOME-4..6); hostNowMs() is defined on the rig's clock below.
#define GEIGER_HOST_PLATFORM

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// Named directly so the library finder adds lib/valence and lib/geiger; it
// does not follow the relative include below.
#include "geiger/geiger.h"
#include "valence/client/client.hpp"
#include "valence/conformance/catalog_check.hpp"
#include "valence/core/clock.hpp"
#include "valence/core/rng.hpp"
#include "valence/hub/hub.hpp"
#include "valence/transport/inprocess_binding.hpp"
#include "valence/util/byte_io.hpp"
#include "valence/wire/catalog_codec.hpp"
#include "valence/wire/catalog_etag.hpp"
#include "valence/wire/estop_frame.hpp"

#include "../../../flagship_p4/src/hub/ValenceDevice.cpp"
#include "../../../flagship_p4/src/patterns/advanced/AdvancedPattern.cpp"

using namespace valence;

// ---- the composition's doors, faked ------------------------------------------

namespace {

ManualClock g_clock{};
MotionCensus g_census{};
MotorSwitchStatus g_switch{};
// What the fake arbiter answers a `return` with; an arrival counts at once,
// as MotionArbiter::returnToPause() does.
ReturnStart g_returnAnswer = ReturnStart::queued;
// The fake gesture source: what the BoardIo task would have parked.
button::Gesture g_homeGesture = button::Gesture::none;
button::PairGesture g_pairGesture = button::PairGesture::none;
// The fake update door (ValenceOta.h): a pending request takes the next
// double press; in flight holds the release.
bool g_otaPending = false;
bool g_otaInFlight = false;
int g_forceHomes = 0;
// What the fake arbiter answers home op 1 with, and how often it was asked.
HomeStart g_homeAnswer = HomeStart::started;
int g_homeCalls = 0;
// The fake DRV_ALM handoff: what the drive-link task would have raised.
bool g_driveAlarm = false;
// Calls through the motion and generator doors: what a refused write must
// never reach.
int g_submits = 0;
// The intents the motion door took, in order (VD-SEG-1).
std::vector<MotionIntent> g_intents;
// The oscillator the motion door last took (osc-set).
MotionOsc g_osc{};
int g_patPushes = 0;
// The fake e-stop reading: what the BoardIo task would have published.
estop::Reading g_estop{};
// The planner's handoff: anomalies waiting for the hub task, oldest first.
std::vector<MotionAnomaly> g_anomalies;

}  // namespace

namespace geiger {
uint32_t hostNowMs() { return uint32_t(g_clock.nowUs() / 1000); }
}  // namespace geiger

namespace valence {

uint64_t deviceNowUs() { return g_clock.nowUs(); }
uint32_t deviceFreeHeapBytes() { return 0; }
LinkTcp g_linkTcp{};
LinkTcp deviceLinkTcp() { return g_linkTcp; }

bool motionBegin() { return true; }
// False: the motion door refuses the intent, as a full queue does.
bool g_submitOk = true;
bool motionSubmit(const MotionIntent& in) {
    if (!g_submitOk) return false;
    ++g_submits;
    g_intents.push_back(in);
    return true;
}
void motionEstop() { g_census.estop = true; }
void motionEstopClear() { g_census.estop = false; }
void motionSetMotorPowered(bool on) { g_census.motor_on = g_census.power_gate = on; }
void motionSetCommissioned(bool) {}
void motionPause(bool on) { g_census.paused = on; }
bool motionAcquireRail(MotionSource) { return true; }
void motionReleaseRail(MotionSource) {}
void motionSetEstopCutsPower(bool) {}
void motionOverride() {}
ReturnStart motionReturn() {
    if (g_returnAnswer == ReturnStart::arrived) {
        ++g_census.returns;
        g_census.override_mode = false;
    }
    return g_returnAnswer;
}
bool g_flipped = false;
void motionSetFlipped(bool on) { g_flipped = on; }
void motionSetJogLimits(float, float) {}
void motionSetInputLimits(float, float, float) {}
std::vector<WindowPost> g_windowPosts;
void motionSetWindow(float, float, float, WindowPost post) { g_windowPosts.push_back(post); }
void motionNoteStream(uint32_t, uint32_t, uint32_t) {}
float motionForceHome(float stroke_mm) {
    ++g_forceHomes;
    return stroke_mm;
}
HomeStart motionHome() {
    ++g_homeCalls;
    return g_homeAnswer;
}
MotionCensus motionCensus() { return g_census; }
bool motionTakeAnomaly(MotionAnomaly& out) {
    if (g_anomalies.empty()) return false;
    out = g_anomalies.front();
    g_anomalies.erase(g_anomalies.begin());
    return true;
}
MotionTuning motionDefaultTuning() { return MotionTuning{}; }
void motionSetTuning(const MotionTuning&) {}
void motionSetOscillator(const MotionOsc& o) { g_osc = o; }
// The osc-drive posts, newest last.
struct OscDrivePost {
    float amplitude, frequency;
    uint64_t at_us;
};
std::vector<OscDrivePost> g_oscDrives;
void motionOscDrive(float a, float f, uint64_t at_us) { g_oscDrives.push_back({a, f, at_us}); }

bool patternBegin() { return true; }
void patternSetSettings(const PatternSettings&) { ++g_patPushes; }
bool patternActive() { return false; }
uint32_t patternStackFree() { return 0; }

bool motorSwitchBegin() { return true; }
void motorSwitchSetSelfCheck(bool) {}
motorswitch::Refusal motorSwitchRequestEnable() { return motorswitch::Refusal::none; }
void motorSwitchCut() {}
MotorSwitchStatus motorSwitchStatus() { return g_switch; }
bool motorSwitchFaultLine() { return false; }
uint32_t motorSwitchStackFree() { return 0; }
std::optional<float> motorSwitchThermVolts() { return std::nullopt; }
PowerNow g_powerNow{};
PowerNow motorSwitchPower() { return g_powerNow; }

button::Gesture homeButtonTake() { return std::exchange(g_homeGesture, button::Gesture::none); }
button::PairGesture pairButtonTake() { return std::exchange(g_pairGesture, button::PairGesture::none); }
bool driveAlarmTake() { return std::exchange(g_driveAlarm, false); }
bool otaPressTake() { return std::exchange(g_otaPending, false); }
bool otaInFlight() { return g_otaInFlight; }

estop::Reading estopInputRead() { return g_estop; }

bool g_datagramOn = true;
void estopDatagramSetEnabled(bool on) { g_datagramOn = on; }
bool estopDatagramEnabled() { return g_datagramOn; }

}  // namespace valence

// ---- rig ------------------------------------------------------------------------

namespace {

struct RecordedNack {
    NackCode code = NackCode::MALFORMED;
    std::string detail;   // copied: the decoded view dies with the frame
};

class RecordingClient final : public ClientDelegate {
public:
    std::map<uint16_t, std::vector<std::byte>> lastState;
    std::map<uint16_t, int> stateCount;
    std::vector<RecordedNack> nacks;
    int echoes = 0;
    IntentValueMap lastEcho{};
    std::vector<EventMsg> anomalyEvents;   // 0x4100, decoded

    void onStateChange(ClientSessionState) override {}
    void onState(uint16_t channel_id, uint16_t, std::span<const std::byte> payload) override {
        lastState[channel_id] = std::vector<std::byte>(payload.begin(), payload.end());
        ++stateCount[channel_id];
    }
    void onEcho(uint16_t, const IntentValueMap& applied, uint16_t) override {
        ++echoes;
        lastEcho = applied;
    }
    void onNack(const NackMsg& n) override {
        nacks.push_back({n.code, n.has_detail ? std::string(n.detail) : std::string()});
    }
    void onPendingDropped(uint16_t) override {}
    std::optional<AccessLevel> granted;   // the last PAIR_GRANT's tier
    void onPairGrant(std::span<const std::byte>, AccessLevel roles) override { granted = roles; }
    void onEvent(uint16_t channel_id, std::span<const std::byte> payload) override {
        if (channel_id != ch::motion_anomaly) return;
        const auto ev = decodeEvent(payload);
        REQUIRE(ev.isOk());
        anomalyEvents.push_back(ev.value());
    }
};

// The saved list's station half (RFC-116): live entry chosen by the test. A
// scan (RFC-118) finishes on the tick it starts and hears `air`, unless
// `hold` keeps it running until stopScan().
class FakeNetStation final : public INetStation {
public:
    uint8_t live = 0;
    bool associating = false;
    int persists = 0;
    int scans = 0;
    int stops = 0;
    bool scanning = false;
    bool hold = false;
    std::vector<net::Sighting> air{{"HomeNet", -51}, {"Office", -70}, {"Cafe", -60, false}, {"HomeNet", -80}};
    bool offerList(const net::SavedNetworks&) override { return true; }
    uint8_t liveId() const override { return live; }
    int8_t rssiOf(uint8_t id) const override { return id == live ? int8_t(-51) : int8_t(-67); }
    bool persist(std::span<const uint8_t>) override {
        ++persists;
        return true;
    }
    bool linked() const override { return live != 0 || associating; }
    bool beginScan() override {
        if (scanning) return false;
        scanning = true;
        ++scans;
        return true;
    }
    bool scanRunning() const override { return scanning; }
    void stopScan() override {
        ++stops;
        scanning = false;
    }
    bool takeScan(net::VisibleNetworks& out) override {
        if (!scanning || hold) return false;
        scanning = false;
        out.take(air);
        return true;
    }
};

constexpr std::string_view kHomeSsid = "HomeNet";
constexpr std::string_view kHomePass = "home passphrase 1";

// Heap, not stack: the catalog and the hub are tens of KB each.
struct Rig {
    Catalog32 catalog{};
    XorShift32 hubRng{4242};
    ValenceDevice device{};
    std::optional<Hub> hub{};
    std::optional<InProcessLink> link{};
    XorShift32 clientRng{4243};
    RecordingClient del{};
    std::optional<Client> client{};
    FakeNetStation netStation{};
    std::optional<NetworkDesk> nets{};

    // motionHz > 0 also subscribes 0x1100 at that rate; hubStatus, 0x0006 and
    // 0x1010; networks, 0x0017 and 0x0018 (RFC-116).
    explicit Rig(AccessLevel role = AccessLevel::control, float motionHz = 0.0f, bool hubStatus = false,
                 bool networks = false) {
        g_census = MotionCensus{};
        g_census.homed = true;
        g_census.motor_on = true;
        g_census.power_gate = true;
        g_returnAnswer = ReturnStart::queued;
        g_homeGesture = button::Gesture::none;
        g_pairGesture = button::PairGesture::none;
        g_otaPending = g_otaInFlight = false;
        g_forceHomes = 0;
        g_flipped = false;
        g_homeAnswer = HomeStart::started;
        g_homeCalls = 0;
        g_driveAlarm = false;
        g_submits = g_patPushes = 0;
        g_submitOk = true;
        g_intents.clear();
        g_switch = MotorSwitchStatus{};
        g_switch.state = motorswitch::State::on;
        g_estop = estop::Reading{};
        g_estop.known = true;
        g_datagramOn = true;
        g_anomalies.clear();
        g_linkTcp = LinkTcp{};
        g_powerNow = PowerNow{};
        REQUIRE(buildValenceCatalog(catalog, boardFeatures()));
        device.setUnvouchedRole(role);
        device.setSetupWritten(kSetupRequiredMask);
        hub.emplace(catalog, g_clock, hubRng, device);
        device.attach(*hub, catalog);
        nets.emplace(netStation);
        {
            net::SavedNetworks boot;
            boot.save(std::nullopt, kHomeSsid, kHomePass);
            nets->adopt(boot);
            netStation.live = nets->list().id(0);
        }
        nets->attach(*hub);
        device.bindNetworks(&*nets);
        link.emplace(g_clock, hubRng);
        // A 13th wish takes WELCOME past the link's default 250 B (the
        // library client adds 0x0001's to the 12 below), and a 15th the HELLO;
        // a WS frame has room for both.
        link->profileA().mtu = link->profileB().mtu = 1024;
        REQUIRE(hub->attachTransport(link->endpointA()));
        ClientIdentity id;
        id.instance_id.fill(std::byte{0});
        id.instance_id[0] = std::byte{7};
        id.hasToken = false;
        id.client_kind = "sim";
        id.client_name = "device-test";
        client.emplace(id, link->endpointB(), g_clock, clientRng, del);
        // The library client reassembles at most 64 chunks and the machine
        // catalog is larger, so it connects as one that already holds it.
        const auto etag = hub->catalogEtag();
        client->setCachedEtag(std::span<const std::byte, limits::etag_bytes>(etag.data(), limits::etag_bytes));
        client->addSubscriptionWish(0x0003, 0.0f, Priority::critical);
        client->addSubscriptionWish(ch::pattern_advanced, 0.0f, Priority::normal);
        client->addSubscriptionWish(ch::pattern_adv_mod_crest, 0.0f, Priority::normal);
        client->addSubscriptionWish(ch::motion_anomaly, 0.0f, Priority::normal);
        // The STATE every writer moves, so a refused write is seen to move none.
        for (const uint16_t id : {ch::machine_config, ch::machine_modes, ch::kinetic_limits,
                                  ch::kinetic_planner, ch::pattern_state, ch::pattern_presets_roster,
                                  ch::oscillator, ch::plan_strip})
            REQUIRE(client->addSubscriptionWish(id, 0.0f, Priority::normal));
        if (motionHz > 0.0f) REQUIRE(client->addSubscriptionWish(ch::motion, motionHz, Priority::elevated));
        if (hubStatus) {
            REQUIRE(client->addSubscriptionWish(channels::hub_status, 1.0f, Priority::background));
            REQUIRE(client->addSubscriptionWish(ch::power, 1.0f, Priority::background));
        }
        if (networks) {
            REQUIRE(client->addSubscriptionWish(net::draft::ch_networks, 0.0f, Priority::background));
            REQUIRE(client->addSubscriptionWish(net::draft::ch_network_names, 0.0f, Priority::background));
            // HELLO carries 15 wishes at most here; a suite subscribes 0x001A live.
            if (role == AccessLevel::configure)
                REQUIRE(client->addSubscriptionWish(net::draft::ch_visible_networks, 0.0f, Priority::background));
        }
        REQUIRE(client->connect());
        step(200);
        REQUIRE(client->state() == ClientSessionState::LIVE);
    }

    void step(int rounds = 16) {
        for (int i = 0; i < rounds; ++i) {
            g_clock.advanceUs(1000);
            hub->update(g_clock.nowUs());
            nets->tick(uint32_t(g_clock.nowUs() / 1000), motionRuns(g_census));
            const uint8_t due = device.tick(g_clock.nowUs() / 1000);
            persisted |= due;
            if (due != 0) ++persistWrites;
            client->update(g_clock.nowUs());
        }
    }

    // The kPersist* bits tick() has returned, and the ticks that returned any.
    uint8_t persisted = 0;
    int persistWrites = 0;

    uint16_t snapshotSeq() {
        const auto it = del.lastState.find(0x0003);
        REQUIRE(it != del.lastState.end());
        REQUIRE(it->second.size() >= 8);
        return getU16(std::span<const std::byte>(it->second).subspan(6, 2));
    }

    uint8_t snapshotCause() {
        const auto it = del.lastState.find(0x0003);
        REQUIRE(it != del.lastState.end());
        REQUIRE(it->second.size() >= 2);
        return uint8_t(it->second[1]);
    }
};

IntentValueMap moveTo(float mm) {
    IntentValueMap m{};
    m.count = 1;
    m.fields[0] = IntentValueField{1, IntentValue::ofF32(mm)};
    return m;
}

IntentValueMap safetyOp(uint8_t op) {
    IntentValueMap m{};
    m.count = 1;
    m.fields[0] = IntentValueField{1, IntentValue::ofU64(op)};
    return m;
}

}  // namespace

// ---- SPEC 5.5: the fault latch continues the hub's one estop_seq ---------------

TEST_CASE("VD-01: a motor switch fault after a client estop latches seq N+1") {
    auto rig = std::make_unique<Rig>();

    // A raw 0xE5 initiation at a seq no count of initiations could guess.
    constexpr uint16_t kN = 41;
    EstopFrame f;
    f.cause = safety_causes::user;
    f.origin = uint8_t(AccessLevel::control);
    f.seq = kN;
    std::array<std::byte, kEstopFrameBytes> raw{};
    REQUIRE(encodeEstop(f, std::span<std::byte>(raw)) == kEstopFrameBytes);
    REQUIRE(rig->link->endpointB().write(std::span<const std::byte>(raw)));
    rig->step();
    REQUIRE(rig->hub->estopLatched());
    const uint16_t n = rig->hub->estopSeq();
    CHECK(n == kN);
    CHECK(rig->snapshotSeq() == n);

    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::release)).has_value());
    rig->step();
    REQUIRE_FALSE(rig->hub->estopLatched());

    g_switch.state = motorswitch::State::faulted;
    g_switch.last_fault = motorswitch::Fault::fault_line;
    ++g_switch.faults;
    rig->step();
    REQUIRE(rig->hub->estopLatched());
    CHECK(rig->hub->estopSeq() == uint16_t(n + 1));
    CHECK(rig->snapshotSeq() == uint16_t(n + 1));
}

// ---- SPEC 16.1: the refusal's reason rides the NACK ----------------------------

TEST_CASE("VD-02: a move refused for motor power carries the reason on the NACK") {
    auto rig = std::make_unique<Rig>();

    g_census.motor_on = g_census.power_gate = false;
    g_switch.state = motorswitch::State::off;
    REQUIRE(rig->client->sendIntent(ch::move, moveTo(10.0f)).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INTERLOCK);
    CHECK(rig->del.nacks[0].detail == "motor power off (switch off)");

    g_switch.state = motorswitch::State::faulted;
    g_switch.last_fault = motorswitch::Fault::precharge;
    REQUIRE(rig->client->sendIntent(ch::move, moveTo(10.0f)).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 2);
    CHECK(rig->del.nacks[1].code == NackCode::INTERLOCK);
    CHECK(rig->del.nacks[1].detail == "motor power off: MOTOR_V+ did not rise");
    CHECK(rig->del.nacks[1].detail.size() <= limits::nack_detail_max_bytes);

    // A later refusal that gives no reason must not carry the stale one.
    g_census.motor_on = g_census.power_gate = true;
    g_census.homed = false;
    REQUIRE(rig->client->sendIntent(ch::move, moveTo(10.0f)).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 3);
    CHECK(rig->del.nacks[2].code == NackCode::NOT_HOMED);
    CHECK(rig->del.nacks[2].detail.empty());
}

// ---- RFC-095: the dwells on the wire --------------------------------------------

namespace {

const IntentValue* echoed(const IntentValueMap& m, uint8_t key) {
    for (uint32_t i = 0; i < m.count; ++i)
        if (m.fields[i].key == key) return &m.fields[i].value;
    return nullptr;
}

}  // namespace

TEST_CASE("VD-03: the dwells write on keys 46 and 47 and publish at the tail of 0x1210") {
    auto rig = std::make_unique<Rig>();

    IntentValueMap m{};
    m.count = 3;
    m.fields[0] = IntentValueField{46, IntentValue::ofF32(0.5f)};
    m.fields[1] = IntentValueField{47, IntentValue::ofF32(1.25f)};
    m.fields[2] = IntentValueField{48, IntentValue::ofU64(70)};   // the crest modulator's amount
    REQUIRE(rig->client->sendIntent(ch::pattern_advanced_cmd, m).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(rig->del.echoes == 1);
    const IntentValue* crest = echoed(rig->del.lastEcho, 46);
    const IntentValue* trough = echoed(rig->del.lastEcho, 47);
    const IntentValue* amount = echoed(rig->del.lastEcho, 48);
    REQUIRE(crest != nullptr);
    REQUIRE(trough != nullptr);
    REQUIRE(amount != nullptr);
    CHECK(crest->f32_val == doctest::Approx(0.5f));
    CHECK(trough->f32_val == doctest::Approx(1.25f));
    CHECK(amount->u64_val == 70);

    // 0x1210: 10 bytes as before, then the dwells in hundredths and the
    // second mask, both dwell bits up.
    const auto adv = rig->del.lastState.find(ch::pattern_advanced);
    REQUIRE(adv != rig->del.lastState.end());
    const std::span<const std::byte> b(adv->second);
    REQUIRE(b.size() == 15);
    CHECK(getU16(b.subspan(10, 2)) == 50);
    CHECK(getU16(b.subspan(12, 2)) == 125);
    CHECK(b[14] == std::byte{0x03});

    const auto mod = rig->del.lastState.find(ch::pattern_adv_mod_crest);
    REQUIRE(mod != rig->del.lastState.end());
    REQUIRE(mod->second.size() == 7);
    CHECK(mod->second[0] == std::byte{70});
}

// ---- bd val-96m: a return with the power gate shut ------------------------------

namespace {

bool overrideLatched(const Rig& rig) {
    return (rig.hub->safetyModes() & safety_mode_bits::OVERRIDE) != 0;
}

}  // namespace

TEST_CASE("VD-04: unpowered, a return with nothing to travel clears override and resume runs") {
    auto rig = std::make_unique<Rig>();
    g_census.motor_on = g_census.power_gate = false;
    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::override)).has_value());
    rig->step();
    REQUIRE(overrideLatched(*rig));

    g_returnAnswer = ReturnStart::arrived;
    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::return_op)).has_value());
    rig->step();
    CHECK(rig->del.nacks.empty());
    CHECK_FALSE(overrideLatched(*rig));
    REQUIRE(rig->hub->pauseLatched());

    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::resume)).has_value());
    rig->step();
    CHECK(rig->del.nacks.empty());
    CHECK_FALSE(rig->hub->pauseLatched());
}

TEST_CASE("VD-05: unpowered, a return with travel left is refused with the motor-power detail") {
    auto rig = std::make_unique<Rig>();
    g_census.motor_on = g_census.power_gate = false;
    g_switch.state = motorswitch::State::off;
    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::override)).has_value());
    rig->step();
    REQUIRE(overrideLatched(*rig));

    g_returnAnswer = ReturnStart::unpowered;
    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::return_op)).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INTERLOCK);
    CHECK(rig->del.nacks[0].detail == "motor power off (switch off)");
    // Not accepted into limbo: override is still the operator's to resolve,
    // and nothing waits on an arrival that cannot happen.
    CHECK(overrideLatched(*rig));
    g_returnAnswer = ReturnStart::arrived;
    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::return_op)).has_value());
    rig->step();
    CHECK(rig->del.nacks.size() == 1);
    CHECK_FALSE(overrideLatched(*rig));
}

// RFC-102 (SPEC 4.5, 6.3): an op the hub does not implement is refused, never
// ignored. Safety op 2 is a retired number.
TEST_CASE("VD-UNSUP: an op the hub does not implement is NACKed UNSUPPORTED_OP and latches nothing") {
    auto rig = std::make_unique<Rig>();
    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(2)).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::UNSUPPORTED_OP);
    CHECK_FALSE(rig->hub->pauseLatched());
    CHECK_FALSE(rig->hub->estopLatched());
    CHECK_FALSE(overrideLatched(*rig));
}

// ---- bd val-urd: every refusal names its reason ---------------------------------

TEST_CASE("VD-06: a move under a latched e-stop carries 'e-stop latched'") {
    auto rig = std::make_unique<Rig>();
    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::estop)).has_value());
    rig->step();
    REQUIRE(rig->hub->estopLatched());
    rig->del.nacks.clear();
    REQUIRE(rig->client->sendIntent(ch::move, moveTo(10.0f)).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::ESTOP_ACTIVE);
    CHECK(rig->del.nacks[0].detail == "e-stop latched");
}

TEST_CASE("VD-07: a PAUSE refusal names itself for intentNackDetail; an admission clears it") {
    auto rig = std::make_unique<Rig>();
    ValenceDevice& d = rig->device;
    CHECK_FALSE(d.admitsUnderPause(ch::move, moveTo(10.0f), false));
    CHECK(d.intentNackDetail(ch::move, NackCode::INTERLOCK) == "paused: override to jog");

    IntentValueMap start{};
    start.count = 1;
    start.fields[0] = IntentValueField{1, IntentValue::ofBool(true)};
    CHECK_FALSE(d.admitsUnderPause(ch::pattern_cmd, start, false));
    CHECK(d.intentNackDetail(ch::pattern_cmd, NackCode::INTERLOCK) == "paused: resume to start");

    CHECK(d.admitsUnderPause(ch::home, IntentValueMap{}, false));
    CHECK(d.intentNackDetail(ch::home, NackCode::INTERLOCK).empty());
    CHECK(d.admitsUnderPause(ch::move, moveTo(10.0f), true));
    CHECK(d.intentNackDetail(ch::move, NackCode::INTERLOCK).empty());
}

TEST_CASE("VD-08: a refused release names why for intentNackDetail") {
    auto rig = std::make_unique<Rig>();
    g_census.busy = true;
    CHECK_FALSE(rig->device.canClearEstop());
    CHECK(rig->device.intentNackDetail(channels::safety_intents, NackCode::CLEAR_REFUSED) ==
          "motion not at rest");
    g_census.busy = false;
    CHECK(rig->device.canClearEstop());
    CHECK(rig->device.intentNackDetail(channels::safety_intents, NackCode::CLEAR_REFUSED).empty());
}

TEST_CASE("VD-08b: no release while a firmware update is in flight: motor power stays off") {
    auto rig = std::make_unique<Rig>();
    g_otaInFlight = true;
    CHECK_FALSE(rig->device.canClearEstop());
    CHECK(rig->device.intentNackDetail(channels::safety_intents, NackCode::CLEAR_REFUSED) ==
          "firmware update in flight");
    g_otaInFlight = false;
    CHECK(rig->device.canClearEstop());
}

// ---- bd val-091.26: the HOME and PAIR buttons ---------------------------------

TEST_CASE("VD-09: HOME press runs home op 1 through applyHome; force_home is never reached") {
    auto rig = std::make_unique<Rig>();
    g_homeGesture = button::Gesture::press;
    rig->step();
    CHECK(g_homeGesture == button::Gesture::none);   // taken
    CHECK(g_homeCalls == 1);
    CHECK(g_forceHomes == 0);
    CHECK_FALSE(rig->hub->estopLatched());
    CHECK_FALSE(rig->device.rebootDue());
}

TEST_CASE("VD-10: HOME hold brakes first, then latches ESTOP and reports the reboot due") {
    auto rig = std::make_unique<Rig>();
    g_census.busy = true;
    g_homeGesture = button::Gesture::hold;
    rig->step(4);
    CHECK(g_census.paused);                 // the arbiter's brake was asked for
    CHECK_FALSE(rig->hub->estopLatched());  // still braking
    CHECK_FALSE(rig->device.rebootDue());
    g_census.busy = false;
    rig->step(2);
    CHECK(rig->hub->estopLatched());
    CHECK(rig->device.rebootDue());
    // Gestures during the reboot are ignored.
    g_pairGesture = button::PairGesture::press;
    rig->step(2);
    CHECK_FALSE(rig->hub->presenceWindowOpen());
}

TEST_CASE("VD-11: a brake that never reaches rest still reboots, after the bound") {
    auto rig = std::make_unique<Rig>();
    g_census.busy = true;
    g_homeGesture = button::Gesture::hold;
    rig->step(1000);
    CHECK_FALSE(rig->device.rebootDue());
    rig->step(2500);
    CHECK(rig->hub->estopLatched());
    CHECK(rig->device.rebootDue());
}

TEST_CASE("VD-12: PAIR press opens the presence window; 3+ presses do nothing") {
    auto rig = std::make_unique<Rig>();
    g_pairGesture = button::PairGesture::extra;
    rig->step();
    CHECK_FALSE(rig->hub->presenceWindowOpen());
    g_pairGesture = button::PairGesture::press;
    rig->step();
    CHECK(rig->hub->presenceWindowOpen());
    CHECK(rig->device.pairLook() == PairLook::window);
    CHECK_FALSE(rig->device.rebootDue());
}

TEST_CASE("VD-13: a planned reboot takes the armed persists at once, then none") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    IntentValueMap m{};
    m.count = 1;
    m.fields[0] = IntentValueField{3, IntentValue::ofF32(42.0f)};   // jog_speed
    REQUIRE(rig->client->sendIntent(ch::config_set, m).has_value());
    rig->step(20);   // applied and armed, well inside the debounce
    CHECK(rig->device.takePendingPersist() == kPersistConfig);
    CHECK(rig->device.takePendingPersist() == 0);
}

// ---- bd val-091.23: the e-stop at the machine ------------------------------------

namespace {

void setEstop(estop::Contacts c) {
    g_estop.state = c;
    g_estop.known = true;
}

}  // namespace

TEST_CASE("VD-14: a pressed e-stop latches ESTOP through onEstop, cause user") {
    auto rig = std::make_unique<Rig>();
    setEstop(estop::Contacts::pressed);
    rig->step();
    REQUIRE(rig->hub->estopLatched());
    CHECK(g_census.estop);   // onEstop() ran motionEstop(): the board's cut and park
    CHECK(rig->snapshotCause() == safety_causes::user);
}

TEST_CASE("VD-15: releasing the button resumes nothing; only release clears, into PAUSE") {
    auto rig = std::make_unique<Rig>();
    setEstop(estop::Contacts::pressed);
    rig->step();
    REQUIRE(rig->hub->estopLatched());
    const uint16_t seq = rig->hub->estopSeq();

    setEstop(estop::Contacts::released);
    rig->step(500);
    CHECK(rig->hub->estopLatched());
    CHECK(g_census.estop);
    CHECK(rig->hub->estopSeq() == seq);

    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::release)).has_value());
    rig->step();
    CHECK(rig->del.nacks.empty());
    CHECK_FALSE(rig->hub->estopLatched());
    CHECK(rig->hub->pauseLatched());
    CHECK_FALSE(g_census.estop);
}

TEST_CASE("VD-16: a release while the button is pressed is refused and names the button") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    setEstop(estop::Contacts::pressed);
    rig->step();
    REQUIRE(rig->hub->estopLatched());

    CHECK_FALSE(rig->device.canClearEstop());
    CHECK(rig->device.intentNackDetail(channels::safety_intents, NackCode::CLEAR_REFUSED) ==
          "e-stop pressed at the machine");
    CHECK(g_census.estop);   // the arbiter's latch was not dropped

    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::release)).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::CLEAR_REFUSED);
    CHECK(rig->hub->estopLatched());

    // force_home's release goes through the same door.
    IntentValueMap home{};
    home.count = 2;
    home.fields[0] = IntentValueField{1, IntentValue::ofU64(2)};
    home.fields[1] = IntentValueField{2, IntentValue::ofF32(200.0f)};
    REQUIRE(rig->client->sendIntent(ch::home, home).has_value());
    rig->step();
    REQUIRE(g_forceHomes == 1);
    CHECK(rig->hub->estopLatched());
}

TEST_CASE("VD-17: both-open and both-closed contacts latch cause fault and name themselves") {
    struct Row {
        estop::Contacts contacts;
        const char* detail;
    };
    for (const Row row : {Row{estop::Contacts::unplugged, "no e-stop found at the machine"},
                          Row{estop::Contacts::wiring_fault, "e-stop wiring fault at the machine"}}) {
        auto rig = std::make_unique<Rig>();
        setEstop(row.contacts);
        rig->step();
        REQUIRE(rig->hub->estopLatched());
        CHECK(rig->snapshotCause() == safety_causes::fault);
        CHECK_FALSE(rig->device.canClearEstop());
        CHECK(rig->device.intentNackDetail(channels::safety_intents, NackCode::CLEAR_REFUSED) == row.detail);
    }
}

TEST_CASE("VD-18: an unread e-stop latches nothing and refuses the release") {
    auto rig = std::make_unique<Rig>();
    g_estop = estop::Reading{};
    rig->step();
    CHECK_FALSE(rig->hub->estopLatched());
    CHECK_FALSE(rig->device.canClearEstop());
    CHECK(rig->device.intentNackDetail(channels::safety_intents, NackCode::CLEAR_REFUSED) ==
          "e-stop input not read");
}

TEST_CASE("VD-19: an EN-node switch fault waits for the e-stop to name it: a press is cause user") {
    auto rig = std::make_unique<Rig>();
    g_switch.state = motorswitch::State::faulted;
    g_switch.last_fault = motorswitch::Fault::en_node;
    ++g_switch.faults;
    rig->step(40);
    CHECK_FALSE(rig->hub->estopLatched());

    setEstop(estop::Contacts::pressed);
    rig->step(2);
    REQUIRE(rig->hub->estopLatched());
    CHECK(rig->snapshotCause() == safety_causes::user);
    const uint16_t seq = rig->hub->estopSeq();
    rig->step(200);   // the switch fault is spent on the e-stop's latch, never a second one
    CHECK(rig->hub->estopSeq() == seq);
    CHECK(rig->snapshotCause() == safety_causes::user);
}

TEST_CASE("VD-20: an EN-node fault the e-stop does not name latches fault after the window") {
    auto rig = std::make_unique<Rig>();
    g_switch.state = motorswitch::State::faulted;
    g_switch.last_fault = motorswitch::Fault::en_node;
    ++g_switch.faults;
    rig->step(int(kEstopNameMs) - 10);
    CHECK_FALSE(rig->hub->estopLatched());
    rig->step(20);
    REQUIRE(rig->hub->estopLatched());
    CHECK(rig->snapshotCause() == safety_causes::fault);
}

TEST_CASE("VD-21: any other switch fault latches at once") {
    auto rig = std::make_unique<Rig>();
    g_switch.state = motorswitch::State::faulted;
    g_switch.last_fault = motorswitch::Fault::inrush;
    ++g_switch.faults;
    rig->step(2);
    CHECK(rig->hub->estopLatched());
    CHECK(rig->snapshotCause() == safety_causes::fault);
}

// ---- bd val-gnw: a value that is not a finite number is refused whole ---------

namespace {

// What a refused write must leave as it was: the config blob (0x1000, the
// modes and the tuning), cfg_gen, the preset store, every STATE the client
// holds, the persist timers, and the motion, home and generator doors.
struct Snapshot {
    std::vector<std::byte> cfgBlob;
    std::vector<std::byte> presetsBlob;
    uint16_t cfgGen = 0;
    std::map<uint16_t, std::vector<std::byte>> state;
    uint8_t persist = 0;
    int submits = 0;
    int forceHomes = 0;
    int patPushes = 0;
    bool operator==(const Snapshot&) const = default;
};

// Disarms the persist timers it reads.
Snapshot snapshot(Rig& rig) {
    Snapshot s;
    s.cfgBlob.resize(stored::kConfigBlobBytes);
    REQUIRE(rig.device.encodeConfigBlob(s.cfgBlob, 0) == s.cfgBlob.size());
    s.presetsBlob.resize(PatternPresetStore::kBlobBytes);
    REQUIRE(rig.device.encodePresetsBlob(s.presetsBlob) == s.presetsBlob.size());
    s.cfgGen = rig.hub->cfgGen();
    s.state = rig.del.lastState;
    s.persist = rig.device.takePendingPersist();
    s.submits = g_submits;
    s.forceHomes = g_forceHomes;
    s.patPushes = g_patPushes;
    return s;
}

// `m` with {key, v} added in ascending key order, which the CBOR writer
// requires.
IntentValueMap plus(IntentValueMap m, uint8_t key, IntentValue v) {
    uint32_t i = m.count++;
    for (; i > 0 && m.fields[i - 1].key > key; --i) m.fields[i] = m.fields[i - 1];
    m.fields[i] = IntentValueField{key, v};
    return m;
}

// NaN and both infinities on `key`, each over `base`: refused INVALID_VALUE
// with `detail`, and nothing moves. Then `finite` on the same key applies, and
// the snapshot sees it move.
void expectNotANumberRefused(Rig& rig, uint16_t channel, const IntentValueMap& base, uint8_t key,
                             IntentValue finite, const std::string& detail) {
    rig.step();
    const Snapshot before = snapshot(rig);
    for (const uint16_t id : {ch::machine_config, ch::machine_modes, ch::kinetic_planner, ch::pattern_state,
                              ch::pattern_advanced, ch::pattern_presets_roster})
        REQUIRE(before.state.count(id) == 1);
    const int echoes = rig.del.echoes;
    const size_t nacks = rig.del.nacks.size();
    for (const float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity()}) {
        CAPTURE(bad);
        const size_t was = rig.del.nacks.size();
        REQUIRE(rig.client->sendIntent(channel, plus(base, key, IntentValue::ofF32(bad))).has_value());
        rig.step();
        REQUIRE(rig.del.nacks.size() == was + 1);
        CHECK(rig.del.nacks.back().code == NackCode::INVALID_VALUE);
        CHECK(rig.del.nacks.back().detail == detail);
        CHECK(rig.del.echoes == echoes);
        CHECK(snapshot(rig) == before);
    }
    REQUIRE(rig.client->sendIntent(channel, plus(base, key, finite)).has_value());
    rig.step();
    CHECK(rig.del.nacks.size() == nacks + 3);
    REQUIRE(rig.del.echoes == echoes + 1);
    CHECK(echoed(rig.del.lastEcho, key) != nullptr);
    CHECK_FALSE(snapshot(rig) == before);
}

}  // namespace

TEST_CASE("VD-22: NaN and infinity are refused INVALID_VALUE naming the field, and nothing moves") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    IntentValueMap base{};
    SUBCASE("config-set") {
        expectNotANumberRefused(*rig, ch::config_set, base, 1, IntentValue::ofF32(12.5f),
                                "window_min: not a number");
    }
    SUBCASE("modes-set") {
        expectNotANumberRefused(*rig, ch::modes_set, base, 7, IntentValue::ofU64(1),
                                "schedule_horizon: not a number");
    }
    SUBCASE("modes-set home_speed") {
        expectNotANumberRefused(*rig, ch::modes_set, base, 9, IntentValue::ofF32(30.0f),
                                "home_speed: not a number");
    }
    SUBCASE("kinetic-set") {
        expectNotANumberRefused(*rig, ch::kinetic_set, base, 6, IntentValue::ofF32(0.37f),
                                "trim_max: not a number");
    }
    SUBCASE("pattern-cmd number") {
        expectNotANumberRefused(*rig, ch::pattern_cmd, base, 3, IntentValue::ofF32(37.0f), "speed: not a number");
    }
    SUBCASE("pattern-cmd bool") {
        expectNotANumberRefused(*rig, ch::pattern_cmd, base, 1, IntentValue::ofBool(false),
                                "running: not a number");
    }
    SUBCASE("pattern-advanced-cmd") {
        expectNotANumberRefused(*rig, ch::pattern_advanced_cmd, base, 46, IntentValue::ofF32(0.75f),
                                "dwell_crest: not a number");
    }
    SUBCASE("pattern-presets-cmd") {
        base = plus(plus(base, 1, IntentValue::ofU64(store_ops::save)), 3, IntentValue::ofTstr("p"));
        expectNotANumberRefused(*rig, ch::pattern_presets_cmd, base, 2, IntentValue::ofU64(3),
                                "slot: not a number");
    }
    SUBCASE("move") {
        expectNotANumberRefused(*rig, ch::move, base, 1, IntentValue::ofF32(10.0f), "position: not a number");
    }
    SUBCASE("home force_home stroke") {
        base = plus(base, 1, IntentValue::ofU64(2));
        expectNotANumberRefused(*rig, ch::home, base, 2, IntentValue::ofF32(300.0f), "stroke: not a number");
    }
}

// ---- RFC-089: the preset writer's per-verb set (SPEC 8.7) ------------------------

namespace {

IntentValueMap presetOp(uint8_t op) {
    IntentValueMap m{};
    m.count = 1;
    m.fields[0] = IntentValueField{1, IntentValue::ofU64(op)};
    return m;
}

std::array<uint8_t, PatternPresetStore::kPayloadBytes> testPayload() {
    std::array<uint8_t, PatternPresetStore::kPayloadBytes> p{};
    for (size_t i = 0; i < p.size(); ++i) p[i] = uint8_t(3 * i + 1);
    return p;
}

// A store-item document over testPayload() in `out`; returns its bytes.
std::span<const std::byte> presetItem(std::span<std::byte> out, uint8_t slot, std::string_view name,
                                      std::string_view kind, size_t payloadBytes, bool digest) {
    static const auto payload = testPayload();
    StoreItem it;
    it.slot = slot;
    it.name = name;
    it.kind = kind;
    it.payload = std::as_bytes(std::span(payload)).first(payloadBytes);
    it.has_digest = digest;
    const size_t n = encodeStoreItem(it, out);
    REQUIRE(n > 0);
    return out.first(n);
}

}  // namespace

TEST_CASE("VD-STORE-1: save without a slot picks the lowest free one and echoes it; a full store names itself") {
    auto rig = std::make_unique<Rig>();
    for (uint8_t i = 0; i < PatternPresetStore::kCapacity; ++i) {
        REQUIRE(rig->client->sendIntent(ch::pattern_presets_cmd,
                                        plus(presetOp(store_ops::save), 3, IntentValue::ofTstr("p"))).has_value());
        rig->step();
        REQUIRE(rig->del.nacks.empty());
        REQUIRE(echoed(rig->del.lastEcho, 2) != nullptr);
        CHECK(echoed(rig->del.lastEcho, 2)->u64_val == i);
    }
    REQUIRE(rig->client->sendIntent(ch::pattern_presets_cmd,
                                    plus(presetOp(store_ops::save), 3, IntentValue::ofTstr("p"))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
    CHECK(rig->del.nacks[0].detail == "pattern presets full");
}

TEST_CASE("VD-STORE-2: a verb missing its slot or name is refused; a field the verb does not use is not echoed") {
    auto rig = std::make_unique<Rig>();
    for (const uint8_t op : {store_ops::load, store_ops::delete_item, store_ops::rename}) {
        rig->del.nacks.clear();
        REQUIRE(rig->client->sendIntent(ch::pattern_presets_cmd,
                                        plus(presetOp(op), 3, IntentValue::ofTstr("p"))).has_value());
        rig->step();
        REQUIRE(rig->del.nacks.size() == 1);
        CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
        CHECK(rig->del.nacks[0].detail == "slot required");
    }
    for (const uint8_t op : {store_ops::save, store_ops::rename}) {
        rig->del.nacks.clear();
        REQUIRE(rig->client->sendIntent(ch::pattern_presets_cmd,
                                        plus(presetOp(op), 2, IntentValue::ofU64(3))).has_value());
        rig->step();
        REQUIRE(rig->del.nacks.size() == 1);
        CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
        CHECK(rig->del.nacks[0].detail == "name required");
    }

    rig->del.nacks.clear();
    REQUIRE(rig->client->sendIntent(ch::pattern_presets_cmd,
                                    plus(plus(presetOp(store_ops::save), 2, IntentValue::ofU64(3)), 3,
                                         IntentValue::ofTstr("p"))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    std::array<std::byte, 160> buf{};
    const auto item = presetItem(buf, 3, "p", kPresetKind, PatternPresetStore::kPayloadBytes, true);
    const int echoes = rig->del.echoes;
    REQUIRE(rig->client->sendIntent(ch::pattern_presets_cmd,
                                    plus(plus(plus(presetOp(store_ops::delete_item), 2, IntentValue::ofU64(3)), 3,
                                              IntentValue::ofTstr("p")), 4, IntentValue::ofBstr(item)))
                .has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(rig->del.echoes == echoes + 1);
    CHECK(rig->del.lastEcho.count == 2);
    CHECK(echoed(rig->del.lastEcho, 3) == nullptr);
    CHECK(echoed(rig->del.lastEcho, 4) == nullptr);
}

TEST_CASE("VD-STORE-3: an import stores the document's payload unread; its address, kind, size and digest are checked") {
    auto rig = std::make_unique<Rig>();
    std::array<std::byte, 160> buf{};
    auto import = [&](uint8_t slot, std::string_view name, std::span<const std::byte> item) {
        rig->del.nacks.clear();
        REQUIRE(rig->client->sendIntent(ch::pattern_presets_cmd,
                                        plus(plus(plus(presetOp(store_ops::save), 2, IntentValue::ofU64(slot)), 3,
                                                  IntentValue::ofTstr(name)), 4, IntentValue::ofBstr(item)))
                    .has_value());
        rig->step();
    };
    auto refused = [&](const char* detail) {
        REQUIRE(rig->del.nacks.size() == 1);
        CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
        CHECK(rig->del.nacks[0].detail == detail);
        CHECK(rig->device.readBlob(blob_ns::store, kPresetStoreId, 5) == std::nullopt);
    };

    import(5, "warm", presetItem(buf, 6, "warm", kPresetKind, PatternPresetStore::kPayloadBytes, true));
    refused("item slot or name differs");
    import(5, "warm", presetItem(buf, 5, "cold", kPresetKind, PatternPresetStore::kPayloadBytes, true));
    refused("item slot or name differs");
    import(5, "warm", presetItem(buf, 5, "warm", "pattern.other", PatternPresetStore::kPayloadBytes, true));
    refused("item kind");
    import(5, "warm", presetItem(buf, 5, "warm", kPresetKind, PatternPresetStore::kPayloadBytes - 1, false));
    refused("item size");
    {
        auto item = presetItem(buf, 5, "warm", kPresetKind, PatternPresetStore::kPayloadBytes, true);
        buf[item.size() - 1] ^= std::byte{0x01};   // the digest is the document's last member
        import(5, "warm", item);
        refused("item digest");
    }

    const auto item = presetItem(buf, 5, "warm", kPresetKind, PatternPresetStore::kPayloadBytes, false);
    import(5, "warm", item);
    REQUIRE(rig->del.nacks.empty());
    CHECK(echoed(rig->del.lastEcho, 2)->u64_val == 5);
    REQUIRE(echoed(rig->del.lastEcho, 4) != nullptr);
    CHECK(echoed(rig->del.lastEcho, 4)->kind == IntentValue::Kind::Bstr);
    const auto blob = rig->device.readBlob(blob_ns::store, kPresetStoreId, 5);
    REQUIRE(blob.has_value());
    const auto doc = decodeStoreItem(blob->bytes);
    REQUIRE(doc.isOk());
    CHECK(doc.value().name == "warm");
    const auto want = testPayload();
    REQUIRE(doc.value().payload.size() == want.size());
    CHECK(std::memcmp(doc.value().payload.data(), want.data(), want.size()) == 0);
}

TEST_CASE("VD-23: a move with no position is refused, never read as position 0") {
    auto rig = std::make_unique<Rig>();
    REQUIRE(rig->client->sendIntent(ch::move, IntentValueMap{}).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
    CHECK(g_submits == 0);
}

// ---- SPEC 11.4: a refused move takes nothing (bd val-u8a) ------------------------

namespace {

// A second control session on its own link, subscribed to control-owner. The
// rig's step() updates only its own client, so stepping goes through here.
struct Peer {
    XorShift32 rng;
    std::optional<InProcessLink> link{};
    RecordingClient del{};
    std::optional<Client> client{};

    Peer(Rig& rig, uint8_t instance) : rng(5000u + instance) {
        link.emplace(g_clock, rng);
        link->profileA().mtu = 1024;   // control-owner is 216 B
        REQUIRE(rig.hub->attachTransport(link->endpointA()));
        ClientIdentity id;
        id.instance_id.fill(std::byte{0});
        id.instance_id[0] = std::byte{instance};
        id.hasToken = false;
        id.client_kind = "sim";
        id.client_name = "peer";
        client.emplace(id, link->endpointB(), g_clock, rng, del);
        const auto etag = rig.hub->catalogEtag();
        client->setCachedEtag(std::span<const std::byte, limits::etag_bytes>(etag.data(), limits::etag_bytes));
        REQUIRE(client->addSubscriptionWish(channels::control_owner, 0.0f, Priority::normal));
        REQUIRE(client->connect());
        step(rig, 200);
        REQUIRE(client->state() == ClientSessionState::LIVE);
    }

    void step(Rig& rig, int rounds = 16) {
        for (int i = 0; i < rounds; ++i) {
            rig.step(1);
            client->update(g_clock.nowUs());
        }
    }

    // control-owner's owner of the jog source (slot 0, Manual), 0 = unowned.
    uint32_t jogOwner() const {
        const auto it = del.lastState.find(channels::control_owner);
        REQUIRE(it != del.lastState.end());
        REQUIRE(it->second.size() >= 5);
        return getU32(std::span<const std::byte>(it->second).subspan(1, 4));
    }
};

}  // namespace

TEST_CASE("VD-OWN-1: a move refused after it took the jog source releases it; another session's move is accepted") {
    auto rig = std::make_unique<Rig>();
    Peer b(*rig, 9);
    REQUIRE(rig->client->sessionId() != 0);
    REQUIRE(b.jogOwner() == 0);

    g_submitOk = false;
    REQUIRE(rig->client->sendIntent(ch::move, moveTo(100.0f)).has_value());
    b.step(*rig);
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INTERLOCK);
    CHECK(b.jogOwner() == 0);

    g_submitOk = true;
    REQUIRE(b.client->sendIntent(ch::move, moveTo(200.0f)).has_value());
    b.step(*rig);
    CHECK(b.del.nacks.empty());
    CHECK(b.del.echoes == 1);
    CHECK(b.jogOwner() == b.client->sessionId());
    REQUIRE(g_intents.size() == 1);
    CHECK(g_intents[0].target_mm == 200.0f);
}

// The source a session already owns stays owned through its refused move (the
// hub's rule), so the refusal must not pin it either: once the move it did
// submit settles, the source goes quiet and another session's move lands.
TEST_CASE("VD-OWN-2: a refused move by the jog's owner leaves the source to go quiet; another session's move is accepted") {
    auto rig = std::make_unique<Rig>();
    Peer b(*rig, 9);
    const uint32_t a = rig->client->sessionId();

    REQUIRE(rig->client->sendIntent(ch::move, moveTo(100.0f)).has_value());
    b.step(*rig);
    REQUIRE(rig->del.echoes == 1);
    REQUIRE(b.jogOwner() == a);
    // The planner takes the move and drives it.
    g_census.intents = 1;
    g_census.busy = true;
    b.step(*rig);
    REQUIRE(b.jogOwner() == a);

    g_submitOk = false;
    REQUIRE(rig->client->sendIntent(ch::move, moveTo(150.0f)).has_value());
    b.step(*rig);
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INTERLOCK);
    CHECK(b.jogOwner() == a);

    // The submitted move settles: the jog has nothing left to execute.
    g_census.busy = false;
    b.step(*rig);
    CHECK(b.jogOwner() == 0);

    g_submitOk = true;
    REQUIRE(b.client->sendIntent(ch::move, moveTo(200.0f)).has_value());
    b.step(*rig);
    CHECK(b.del.nacks.empty());
    CHECK(b.del.echoes == 1);
    CHECK(b.jogOwner() == b.client->sessionId());
}

// ---- DRV_ALM: the drive's own alarm reaches the hub as a fault latch (val-091.29) --

TEST_CASE("VD-DRV-1: a drive alarm latches ESTOP, cause fault, through onEstop") {
    auto rig = std::make_unique<Rig>();
    REQUIRE_FALSE(rig->hub->estopLatched());
    g_driveAlarm = true;
    rig->step();
    REQUIRE(rig->hub->estopLatched());
    CHECK(g_census.estop);   // onEstop() reached the motion path first
    CHECK_FALSE(g_driveAlarm);
    const auto it = rig->del.lastState.find(0x0003);
    REQUIRE(it != rig->del.lastState.end());
    REQUIRE(it->second.size() >= 2);
    CHECK(std::to_integer<uint8_t>(it->second[1]) == safety_causes::fault);
}

TEST_CASE("VD-DRV-2: a drive alarm under a held ESTOP keeps that latch's cause and seq") {
    auto rig = std::make_unique<Rig>();
    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::estop)).has_value());
    rig->step();
    REQUIRE(rig->hub->estopLatched());
    const uint16_t seq = rig->hub->estopSeq();
    g_driveAlarm = true;
    rig->step();
    CHECK(rig->hub->estopSeq() == seq);
    CHECK_FALSE(g_driveAlarm);   // taken: it cannot latch later, after a release
    const auto it = rig->del.lastState.find(0x0003);
    REQUIRE(it != rig->del.lastState.end());
    REQUIRE(it->second.size() >= 2);
    CHECK(std::to_integer<uint8_t>(it->second[1]) == safety_causes::user);
}

// ---- RFC-099: trial writes on the delegate ------------------------------------

namespace {

IntentValueMap oneKey(uint8_t key, IntentValue v) {
    IntentValueMap m{};
    m.count = 1;
    m.fields[0] = IntentValueField{key, v};
    return m;
}

// One f32 of the config blob the persist would write now. Read raw: the fake
// engine's tuning is all zeros, which decodeConfig() rightly refuses.
// Offsets per StoredState.h: header 7 B, then the eight 0x1000 values; the
// v8 tail is smoothness, handle_floor, trim_max.
constexpr size_t kBlobJogSpeed = 7 + 2 * 4;
constexpr size_t kBlobTrimMax = stored::kConfigV7Bytes + 2 * 4;
float storedF32(Rig& rig, size_t offset) {
    std::array<std::byte, stored::kConfigBlobBytes> blob{};
    REQUIRE(rig.device.encodeConfigBlob(blob, rig.hub->cfgGen()) == blob.size());
    return getF32(std::span<const std::byte>(blob).subspan(offset, 4));
}

// Steps past the persist debounce, returning every kPersist* bit tick() raised.
uint8_t persistBitsOver(Rig& rig, int ms) {
    uint8_t due = 0;
    for (int i = 0; i < ms; ++i) {
        g_clock.advanceUs(1000);
        rig.hub->update(g_clock.nowUs());
        due |= rig.device.tick(g_clock.nowUs() / 1000);
        rig.client->update(g_clock.nowUs());
    }
    return due;
}

float stateF32(Rig& rig, uint16_t channel, size_t offset) {
    const auto it = rig.del.lastState.find(channel);
    REQUIRE(it != rig.del.lastState.end());
    REQUIRE(it->second.size() >= offset + 4);
    return getF32(std::span<const std::byte>(it->second).subspan(offset, 4));
}

uint8_t stateU8(Rig& rig, uint16_t channel, size_t offset) {
    const auto it = rig.del.lastState.find(channel);
    REQUIRE(it != rig.del.lastState.end());
    REQUIRE(it->second.size() > offset);
    return std::to_integer<uint8_t>(it->second[offset]);
}

}  // namespace

TEST_CASE("VD-TR-1: a trial jog speed is live, marked, never stored, and commit stores it") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    persistBitsOver(*rig, 2500);   // drain the boot's own writes
    const float before = rig->device.config().jog_speed;
    const float trial = before + 7.0f;
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(3, IntentValue::ofF32(trial)), std::nullopt, false,
                                    true).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    CHECK(rig->device.config().jog_speed == doctest::Approx(trial));
    CHECK(stateF32(*rig, ch::machine_config, 8) == doctest::Approx(trial));
    CHECK(stateU8(*rig, ch::machine_config, 37) == 0x04);   // bit 2: jog_speed
    CHECK(storedF32(*rig, kBlobJogSpeed) == doctest::Approx(before));
    CHECK((persistBitsOver(*rig, 2500) & kPersistConfig) == 0);

    REQUIRE(rig->client->sendIntent(channels::settings_trial, oneKey(1, IntentValue::ofU64(trial_ops::commit)))
                .has_value());
    rig->step();
    CHECK(rig->hub->trialCount() == 0);
    CHECK(stateU8(*rig, ch::machine_config, 37) == 0x00);
    CHECK(storedF32(*rig, kBlobJogSpeed) == doctest::Approx(trial));
    CHECK((persistBitsOver(*rig, 2500) & kPersistConfig) != 0);
}

TEST_CASE("VD-TR-2: revert and a session's end restore the kinetic baseline") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    const float trim0 = storedF32(*rig, kBlobTrimMax);
    REQUIRE(rig->client->sendIntent(ch::kinetic_set, oneKey(6, IntentValue::ofF32(0.6f)), std::nullopt, false,
                                    true).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    CHECK(rig->hub->trialCount() == 1);
    CHECK(stateU8(*rig, ch::kinetic_planner, 21) == 0x04);   // bit 2: trim_max
    CHECK(storedF32(*rig, kBlobTrimMax) == doctest::Approx(trim0));
    CHECK(stateF32(*rig, ch::kinetic_planner, 8) == doctest::Approx(0.6f));

    REQUIRE(rig->client->sendIntent(channels::settings_trial, oneKey(1, IntentValue::ofU64(trial_ops::revert)))
                .has_value());
    rig->step();
    CHECK(rig->hub->trialCount() == 0);
    CHECK(stateF32(*rig, ch::kinetic_planner, 8) == doctest::Approx(trim0));
    CHECK(stateU8(*rig, ch::kinetic_planner, 21) == 0x00);

    // A session that goes away takes its trial with it.
    REQUIRE(rig->client->sendIntent(ch::kinetic_set, oneKey(6, IntentValue::ofF32(0.5f)), std::nullopt, false,
                                    true).has_value());
    rig->step();
    CHECK(rig->hub->trialCount() == 1);
    rig->client->disconnect();
    rig->step(50);
    CHECK(rig->hub->trialCount() == 0);
    // The client is gone, so the live value is read where a baseline is.
    CHECK(rig->device.trialBaseline(ch::kinetic_set, 6)->f32_val == doctest::Approx(trim0));
}

TEST_CASE("VD-WIN-POST: only a session's config write is a live window edit; a revert and the measured rail adopt") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    rig->step(50);
    g_windowPosts.clear();
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(2, IntentValue::ofF32(200.0f))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(g_windowPosts.size() == 1);
    CHECK(g_windowPosts.back() == WindowPost::edit);
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(2, IntentValue::ofF32(150.0f)), std::nullopt, false,
                                    true).has_value());
    rig->step();
    REQUIRE(g_windowPosts.size() == 2);
    CHECK(g_windowPosts.back() == WindowPost::edit);   // a trial is the session's live edit
    REQUIRE(rig->client->sendIntent(channels::settings_trial, oneKey(1, IntentValue::ofU64(trial_ops::revert)))
                .has_value());
    rig->step();
    REQUIRE(g_windowPosts.size() == 3);
    CHECK(g_windowPosts.back() == WindowPost::adopt);
    g_census.home_rail_mm = 180.0f;
    ++g_census.homes;
    rig->step();
    REQUIRE(g_windowPosts.size() == 4);
    CHECK(g_windowPosts.back() == WindowPost::adopt);
}

TEST_CASE("VD-TR-3: keys gated on live state refuse a trial; the flip waits for a window trial") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    REQUIRE(rig->client->sendIntent(ch::kinetic_set, oneKey(7, IntentValue::ofF32(40.0f)), std::nullopt, false,
                                    true).has_value());
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(8, IntentValue::ofU64(1)), std::nullopt, false, true)
                .has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 2);
    CHECK(rig->del.nacks[0].code == NackCode::UNSUPPORTED_OP);
    CHECK(rig->del.nacks[1].code == NackCode::UNSUPPORTED_OP);
    CHECK(rig->hub->trialCount() == 0);

    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(2, IntentValue::ofF32(200.0f)), std::nullopt, false,
                                    true).has_value());
    rig->step();
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(8, IntentValue::ofU64(1))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 3);
    CHECK(rig->del.nacks[2].code == NackCode::INTERLOCK);
    CHECK(rig->del.nacks[2].detail == "window on trial: commit or revert first");
}

// ---- bd val-dbo: home op 1, the real cycle ------------------------------------

namespace {

IntentValueMap homeOp1() {
    IntentValueMap m{};
    m.count = 1;
    m.fields[0] = IntentValueField{1, IntentValue::ofU64(1)};
    return m;
}

}  // namespace

TEST_CASE("VD-HOME-1: home op 1 refusals carry the arbiter's reason; a sense-less board is UNSUPPORTED_OP") {
    struct Row {
        HomeStart answer;
        NackCode code;
        const char* detail;
    };
    for (const Row row : {Row{HomeStart::no_sense, NackCode::UNSUPPORTED_OP, "no home sense on this board"},
                          Row{HomeStart::undriven, NackCode::INTERLOCK, "home sense not driven: sensor unwired or down"},
                          Row{HomeStart::sense_high, NackCode::INTERLOCK, "home sense already reads a stall"}}) {
        auto rig = std::make_unique<Rig>();
        g_homeAnswer = row.answer;
        REQUIRE(rig->client->sendIntent(ch::home, homeOp1()).has_value());
        rig->step();
        REQUIRE(rig->del.nacks.size() == 1);
        CHECK(rig->del.nacks[0].code == row.code);
        CHECK(rig->del.nacks[0].detail == row.detail);
        CHECK(g_homeCalls == 1);
    }
}

TEST_CASE("VD-HOME-2: a moving machine is refused before the arbiter is asked") {
    auto rig = std::make_unique<Rig>();
    g_census.busy = true;
    REQUIRE(rig->client->sendIntent(ch::home, homeOp1()).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INTERLOCK);
    CHECK(g_homeCalls == 0);
}

TEST_CASE("VD-HOME-3: a started cycle ECHOes; its completion clears home_required, force_home waits") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    rig->hub->setHomeRequired(true);
    rig->step();
    const int echoes = rig->del.echoes;
    REQUIRE(rig->client->sendIntent(ch::home, homeOp1()).has_value());
    rig->step();
    CHECK(rig->del.nacks.empty());
    CHECK(rig->del.echoes == echoes + 1);
    CHECK(g_homeCalls == 1);

    // While the cycle runs, force_home is refused and nothing clears.
    g_census.homing = true;
    g_census.homed = false;
    IntentValueMap force{};
    force.count = 1;
    force.fields[0] = IntentValueField{1, IntentValue::ofU64(2)};
    REQUIRE(rig->client->sendIntent(ch::home, force).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INTERLOCK);
    CHECK(g_forceHomes == 0);
    CHECK((rig->hub->safetyModes() & safety_mode_bits::HOME_REQUIRED) != 0);

    g_census.homing = false;
    g_census.homed = true;
    ++g_census.homes;
    rig->step();
    CHECK((rig->hub->safetyModes() & safety_mode_bits::HOME_REQUIRED) == 0);
}

TEST_CASE("VD-HOME-COLD: after a cold start home_required reads true until a home or force_home completes (val-mvw4)") {
    auto rig = std::make_unique<Rig>();
    // The arbiter boots unhomed: the published bit follows it, never a flag of its own.
    g_census.homed = false;
    rig->step();
    CHECK((rig->hub->safetyModes() & safety_mode_bits::HOME_REQUIRED) != 0);
    rig->step(50);
    CHECK((rig->hub->safetyModes() & safety_mode_bits::HOME_REQUIRED) != 0);
    // A force_home asserts homed without a cycle (no homes count): the bit clears.
    g_census.homed = true;
    rig->step();
    CHECK((rig->hub->safetyModes() & safety_mode_bits::HOME_REQUIRED) == 0);
    // Unhomed again (an e-stop that cuts power): the bit returns.
    g_census.homed = false;
    rig->step();
    CHECK((rig->hub->safetyModes() & safety_mode_bits::HOME_REQUIRED) != 0);
}

// ---- bd val-zsr: the two-sided cycle's hub half --------------------------------

namespace {

// Every record at Warn and above, the floor the board's log channel bridge
// takes (system/ValenceLogBridge.cpp).
struct LogCapture final : geiger::ISink {
    std::vector<std::string> lines;
    void write(const geiger::Record& r) override {
        if (r.level >= geiger::Level::Warn) lines.emplace_back(r.msg);
    }
};

LogCapture& logCapture() {
    static LogCapture c;
    static const bool added = geiger::logger().addSink(&c);
    REQUIRE(added);
    return c;
}

void clearLog() {
    geiger::drainToSinks();
    logCapture().lines.clear();
}

bool logged(const std::string& line) {
    geiger::drainToSinks();
    for (const std::string& l : logCapture().lines)
        if (l == line) return true;
    return false;
}

// One completed cycle as the arbiter's census reports it.
void completeHome(Rig& rig, float rail_mm) {
    g_census.homing = false;
    g_census.homed = true;
    g_census.home_rail_mm = rail_mm;
    ++g_census.homes;
    rig.step();
}

// Offsets per StoredState.h: header 7 B, max_rail the eighth 0x1000 value;
// home_speed the v6 layout's last four bytes.
constexpr size_t kBlobMaxRail = 7 + 7 * 4;
constexpr size_t kBlobHomeSpeed = stored::kConfigV6Bytes - 4;

}  // namespace

TEST_CASE("VD-HOME-4: a completed cycle's rail lands in max_rail through config-set's writer, logged") {
    auto rig = std::make_unique<Rig>();
    logCapture();
    persistBitsOver(*rig, 2500);   // drain the boot's own writes
    clearLog();
    const uint16_t gen = rig->hub->cfgGen();
    completeHome(*rig, 612.4f);
    CHECK(rig->device.config().max_rail == doctest::Approx(612.4f));
    CHECK(stateF32(*rig, ch::machine_config, 24) == doctest::Approx(612.4f));   // max_rail
    CHECK(stateF32(*rig, ch::machine_config, 33) == doctest::Approx(612.4f));   // measured_stroke
    CHECK(rig->hub->cfgGen() == uint16_t(gen + 1));                             // RFC-011, once
    CHECK(storedF32(*rig, kBlobMaxRail) == doctest::Approx(612.4f));
    CHECK((persistBitsOver(*rig, 2500) & kPersistConfig) != 0);
    CHECK(logged("HOME: homed. Usable rail 612.4 mm between the safety margins, max_rail 612.4 mm"));

    // The same rail again: the measurement republishes, the setting and
    // cfg_gen hold.
    completeHome(*rig, 612.4f);
    CHECK(rig->hub->cfgGen() == uint16_t(gen + 1));
    CHECK(stateF32(*rig, ch::machine_config, 33) == doctest::Approx(612.4f));
}

TEST_CASE("VD-HOME-5: a rail past max_rail's bounds is stored clamped; measured_stroke keeps the measurement") {
    auto rig = std::make_unique<Rig>();
    clearLog();
    completeHome(*rig, 2600.0f);
    CHECK(rig->device.config().max_rail == ceiling::rail_mm);
    CHECK(stateF32(*rig, ch::machine_config, 24) == ceiling::rail_mm);
    CHECK(stateF32(*rig, ch::machine_config, 33) == doctest::Approx(2600.0f));
    CHECK(logged("HOME: homed. Usable rail 2600.0 mm between the safety margins, max_rail 2000.0 mm"));
}

TEST_CASE("VD-HOME-6: a failed cycle is logged in words with its leg; nothing is stored") {
    auto rig = std::make_unique<Rig>();
    clearLog();
    const uint16_t gen = rig->hub->cfgGen();
    const float rail = rig->device.config().max_rail;
    g_census.homing = false;
    g_census.homed = false;
    g_census.home_fail_leg = 1;
    g_census.home_fail_why = "no stall across max_rail";
    ++g_census.home_fails;
    rig->step();
    CHECK(logged("HOME failed at the far end: no stall across max_rail, unhomed"));
    CHECK(rig->device.config().max_rail == rail);
    CHECK(rig->hub->cfgGen() == gen);
    CHECK(stateF32(*rig, ch::machine_config, 33) == 0.0f);   // never measured
}

// bd val-3kd: while a real cycle's measurement stands, max_rail and the
// window never reach past the stop it found.
TEST_CASE("VD-HOME-7: after a real home, max_rail and the window clamp to the measured stroke, echoed") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    REQUIRE(rig->device.config().window_max == factory::window_max);   // 500, set before any home
    completeHome(*rig, 267.69f);
    CHECK(rig->device.config().max_rail == doctest::Approx(267.69f));
    CHECK(rig->device.config().window_max == doctest::Approx(267.69f));   // the adoption cut it
    CHECK(stateF32(*rig, ch::machine_config, 4) == doctest::Approx(267.69f));
    CHECK(stateF32(*rig, ch::machine_config, 33) == doctest::Approx(267.69f));   // measured_stroke

    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(8, IntentValue::ofF32(500.0f))).has_value());
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(2, IntentValue::ofF32(400.0f))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(echoed(rig->del.lastEcho, 2) != nullptr);
    CHECK(echoed(rig->del.lastEcho, 2)->f32_val == doctest::Approx(267.69f));
    CHECK(rig->device.config().max_rail == doctest::Approx(267.69f));
    CHECK(stateF32(*rig, ch::machine_config, 24) == doctest::Approx(267.69f));

    // Shorter than the measurement is the owner's to choose.
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(8, IntentValue::ofF32(200.0f))).has_value());
    rig->step();
    REQUIRE(echoed(rig->del.lastEcho, 8) != nullptr);
    CHECK(echoed(rig->del.lastEcho, 8)->f32_val == 200.0f);

    // A window bound past the stop has no legal nearest value.
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(1, IntentValue::ofF32(300.0f))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
    CHECK(rig->del.nacks[0].detail == "window past the measured stroke");

    // Unhomed, a longer rail's search can be asked for again.
    g_census.homed = false;
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(8, IntentValue::ofF32(500.0f))).has_value());
    rig->step();
    REQUIRE(echoed(rig->del.lastEcho, 8) != nullptr);
    CHECK(echoed(rig->del.lastEcho, 8)->f32_val == 500.0f);
}

TEST_CASE("VD-HOME-8: force_home measures nothing; a window wholly past the stop never blocks the adoption, it collapses") {
    // Homed by assertion only: the catalog ceiling, as before any home.
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(8, IntentValue::ofF32(800.0f))).has_value());
    rig->step();
    REQUIRE(echoed(rig->del.lastEcho, 8) != nullptr);
    CHECK(echoed(rig->del.lastEcho, 8)->f32_val == 800.0f);

    IntentValueMap win{};
    win.count = 2;
    win.fields[0] = IntentValueField{1, IntentValue::ofF32(300.0f)};
    win.fields[1] = IntentValueField{2, IntentValue::ofF32(400.0f)};
    REQUIRE(rig->client->sendIntent(ch::config_set, win).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    completeHome(*rig, 267.69f);
    CHECK(rig->device.config().max_rail == doctest::Approx(267.69f));
    // Onto the rail's far end, where the arbiter's clamp already held it (val-8yv).
    CHECK(rig->device.config().window_min == doctest::Approx(266.69f));
    CHECK(rig->device.config().window_max == doctest::Approx(267.69f));
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(3, IntentValue::ofF32(60.0f))).has_value());
    rig->step();
    CHECK(rig->del.nacks.empty());
    CHECK(rig->device.config().jog_speed == 60.0f);
}

// bd val-8yv: SPEC 9.6 holds the travel window inside max_rail on every write
// of it, and 0x1000 never publishes a window past it.
TEST_CASE("VD-HOME-9: a max_rail write holds the window inside it; a window write wholly past it is refused") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    logCapture();
    clearLog();
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(2, IntentValue::ofF32(400.0f))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(stateF32(*rig, ch::machine_config, 4) == 400.0f);

    // The soak's repro: the window's top edge follows max_rail down.
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(8, IntentValue::ofF32(100.0f))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    CHECK(rig->device.config().window_max == 100.0f);
    CHECK(stateF32(*rig, ch::machine_config, 0) == 0.0f);      // window_min
    CHECK(stateF32(*rig, ch::machine_config, 4) == 100.0f);    // window_max
    CHECK(stateF32(*rig, ch::machine_config, 24) == 100.0f);   // max_rail

    // A window wholly past it has no nearest legal value, also with max_rail
    // in the same write.
    IntentValueMap win{};
    win.count = 2;
    win.fields[0] = IntentValueField{1, IntentValue::ofF32(200.0f)};
    win.fields[1] = IntentValueField{2, IntentValue::ofF32(300.0f)};
    REQUIRE(rig->client->sendIntent(ch::config_set, win).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
    CHECK(rig->del.nacks[0].detail == "window past max_rail");
    win.count = 3;
    win.fields[2] = IntentValueField{8, IntentValue::ofF32(80.9f)};
    REQUIRE(rig->client->sendIntent(ch::config_set, win).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 2);
    CHECK(rig->del.nacks[1].code == NackCode::INVALID_VALUE);
    CHECK(rig->device.config().window_min == 0.0f);
    CHECK(rig->device.config().window_max == 100.0f);
    CHECK(rig->device.config().max_rail == 100.0f);

    // A window straddling a max_rail written with it is cut, echoed post-clamp.
    win.fields[0] = IntentValueField{1, IntentValue::ofF32(50.0f)};
    win.fields[1] = IntentValueField{2, IntentValue::ofF32(300.0f)};
    win.fields[2] = IntentValueField{8, IntentValue::ofF32(250.0f)};
    REQUIRE(rig->client->sendIntent(ch::config_set, win).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 2);
    REQUIRE(echoed(rig->del.lastEcho, 2) != nullptr);
    CHECK(echoed(rig->del.lastEcho, 2)->f32_val == 250.0f);
    CHECK(stateF32(*rig, ch::machine_config, 4) == 250.0f);

    // max_rail written under a window wholly inside the old rail collapses it
    // onto the new rail's end, logged.
    win.count = 2;
    win.fields[0] = IntentValueField{1, IntentValue::ofF32(200.0f)};
    win.fields[1] = IntentValueField{2, IntentValue::ofF32(240.0f)};
    REQUIRE(rig->client->sendIntent(ch::config_set, win).has_value());
    rig->step();
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(8, IntentValue::ofF32(150.0f))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 2);
    CHECK(stateF32(*rig, ch::machine_config, 0) == 149.0f);
    CHECK(stateF32(*rig, ch::machine_config, 4) == 150.0f);
    CHECK(logged("window 200.0..240.0 mm lies past max_rail 150.0 mm: held at 149.0..150.0 mm, set the window again"));
}

// bd val-ku3: the catalog declares 0x4100 motion-anomaly; every anomaly the
// planner hands over goes out as one EVENT, under the hub's cap.
TEST_CASE("VD-ANOM-1: each planner anomaly is one 0x4100 event, kind and body per the schema, capped") {
    auto rig = std::make_unique<Rig>();
    g_anomalies.push_back({3, 10, 0x100001234ull, 62.5f, 0.5f});   // knot_trimmed
    g_anomalies.push_back({6, 11, 0x100001240ull, 187.5f, 1.25f});  // piece_over_ceiling
    g_anomalies.push_back({1, 12, 0x100001250ull, 125.0f, 12.0f});   // settle
    rig->step();
    CHECK(g_anomalies.empty());
    REQUIRE(rig->del.anomalyEvents.size() == 3);
    const EventMsg& e = rig->del.anomalyEvents[1];
    CHECK(e.channel_id == ch::motion_anomaly);
    CHECK(e.event_kind == 6);
    REQUIRE(e.has_body);
    REQUIRE(e.body_count == 5);
    std::map<uint8_t, IntentValue> body;
    for (uint32_t i = 0; i < e.body_count; ++i) body[e.body[i].key] = e.body[i].value;
    CHECK(body[anom_body::kind].u64_val == 6);
    CHECK(body[anom_body::seq].u64_val == 11);
    CHECK(body[anom_body::target].f32_val == 187.5f);   // mm, as the arbiter hands it over
    CHECK(body[anom_body::ceiling_ratio].f32_val == 1.25f);
    CHECK(body[anom_body::t_ms].u64_val == 0x100001240ull / 1000);   // ms since boot, past 2^32 us
    CHECK(rig->del.anomalyEvents[0].event_kind == 3);
    CHECK(rig->del.anomalyEvents[2].event_kind == 1);

    // A planner recording at its plan rate: a burst goes out, the rest is
    // taken and dropped (0x1111 counts it), never queued behind the cap.
    rig->step(1000);
    rig->del.anomalyEvents.clear();
    for (uint16_t s = 0; s < 50; ++s) g_anomalies.push_back({3, uint16_t(100 + s), 0, 0.5f, 0.1f});
    rig->step(1);
    CHECK(g_anomalies.empty());
    CHECK(rig->del.anomalyEvents.size() == kAnomalyEventBurst);
    rig->step(1000);
    CHECK(rig->del.anomalyEvents.size() == kAnomalyEventBurst);
    g_anomalies.push_back({5, 200, 0, 0.5f, -95.0f});
    rig->step(1);
    REQUIRE(rig->del.anomalyEvents.size() == kAnomalyEventBurst + 1);
    CHECK(rig->del.anomalyEvents.back().event_kind == 5);
}

// Each kind sends the one body field its detail means, declared in the
// schema with its unit; a kind with no detail, or an unnamed refusal, sends none.
TEST_CASE("VD-ANOM-2: each kind's detail rides its own declared field, nothing else") {
    auto rig = std::make_unique<Rig>();
    const CatalogEntry* entry = rig->catalog.find(ch::motion_anomaly);
    REQUIRE(entry != nullptr);
    std::map<uint8_t, SchemaField> schema;
    for (const SchemaField& f : rig->catalog.schemaFields(*entry)) schema[f.key] = f;
    CHECK(schema.count(4) == 0);   // the retired unitless detail
    CHECK(schema[anom_body::target].role == valence::field_roles::telemetry_target);
    CHECK(schema[anom_body::target].unitId == valence::unit_ids::mm);   // RFC-122: mm, not window units
    CHECK(schema[anom_body::t_ms].unitId == valence::unit_ids::ms);
    CHECK(schema[anom_body::kept].unitId == valence::unit_ids::percent);
    CHECK(schema[anom_body::ceiling_ratio].unit == "x");
    CHECK(schema[anom_body::v_end].unitId == valence::unit_ids::mm_s);
    CHECK(rig->catalog.labels(schema[anom_body::refusal].options).size() == 3);

    struct Case { uint8_t kind; float detail; int key; double want; };
    const Case cases[] = {
        {1, 42.5f, anom_body::v_end, 42.5},           // settle: speed at the brake
        {2, -80.0f, anom_body::v_end, -80.0},         // endvel_clamped: the speed held
        {3, 0.72f, anom_body::kept, 72.0},            // knot_trimmed: share kept, %
        {4, 0.0f, -1, 0.0},                           // dwell_zeroed: no detail
        {5, -99.0f, anom_body::refusal, 0.0},         // non-finite
        {5, -95.0f, anom_body::refusal, 1.0},         // in the past
        {5, -96.0f, anom_body::refusal, 2.0},         // timeline full
        {5, -1.0f, -1, 0.0},                          // a sentinel the table does not name
        {6, 12.64f, anom_body::ceiling_ratio, 12.64}, // piece_over_ceiling
    };
    for (size_t i = 0; i < std::size(cases); ++i) {
        const Case& c = cases[i];
        CAPTURE(i);
        g_anomalies.push_back({c.kind, 0, 5'000'000, 140.25f, c.detail});
        rig->step(1000);   // one at a time, under the cap
        REQUIRE(rig->del.anomalyEvents.size() == i + 1);
        const EventMsg& e = rig->del.anomalyEvents[i];
        CHECK(e.event_kind == c.kind);
        std::map<uint8_t, IntentValue> body;
        for (uint32_t k = 0; k < e.body_count; ++k) body[e.body[k].key] = e.body[k].value;
        CHECK(body.count(anom_body::kind) == 1);
        CHECK(body.count(anom_body::seq) == 1);
        CHECK(body[anom_body::target].f32_val == 140.25f);
        CHECK(body[anom_body::t_ms].u64_val == 5000);
        CHECK(e.body_count == (c.key < 0 ? 4u : 5u));
        for (const auto& [k, f] : schema) CHECK(body.count(k) <= 1);
        if (c.key < 0) continue;
        REQUIRE(body.count(uint8_t(c.key)) == 1);
        const IntentValue& v = body[uint8_t(c.key)];
        if (c.key == anom_body::refusal) CHECK(v.u64_val == uint64_t(c.want));
        else CHECK(v.f32_val == doctest::Approx(c.want));
    }
}

// bd val-vik: a plan outside the window is a share below 0 or past 6.5535,
// which the old u16 positions saturated or could not carry.
TEST_CASE("VD-PLAN-1: plan-strip positions are i32 window shares, declared so, negative and past 6.5535") {
    auto rig = std::make_unique<Rig>();
    const CatalogEntry* e = rig->catalog.find(ch::plan_strip);
    REQUIRE(e != nullptr);
    const auto fields = rig->catalog.layoutFields(*e);
    int i32s = 0;
    for (const LayoutField& f : fields) {
        if (f.name == "start_norm" || f.name == "end_norm" || f.name == "cur_norm") {
            CHECK(f.type == PackedFieldType::i32);
            CHECK(f.scale == 10000.0f);
            ++i32s;
        }
    }
    CHECK(i32s == 3);
    REQUIRE(layoutWireSize(fields) == 25);

    g_census.busy = true;
    g_census.plan_start = -0.25f;   // below the window
    g_census.plan_end = 7.5f;       // the soak's carriage, far past it
    g_census.plan_cur = 6.6f;
    g_census.plan_vel = -1.5f;
    g_census.plan_duration_us = 400000;
    g_census.plan_elapsed_us = 1000;
    rig->step(100);
    const auto it = rig->del.lastState.find(ch::plan_strip);
    REQUIRE(it != rig->del.lastState.end());
    REQUIRE(it->second.size() == 25);
    const std::span<const std::byte> s(it->second);
    CHECK(int32_t(getU32(s.subspan(2, 4))) == -2500);
    CHECK(int32_t(getU32(s.subspan(6, 4))) == 75000);
    CHECK(int32_t(getU32(s.subspan(10, 4))) == 66000);
    CHECK(int16_t(getU16(s.subspan(14, 2))) == -1500);
    CHECK(getU32(s.subspan(16, 4)) == 400000u);
}

// The P4's and the twin's hub tick is 5 ms (bd val-7ur). 0x1100 at a 60 Hz
// grant must arrive at 60 Hz, and 0x1110 under a rate-0 grant at no more
// than its 45 Hz ceiling.
TEST_CASE("VD-RATE: on the 5 ms hub tick 0x1100 meets its 60 Hz grant and 0x1110 holds its 45 Hz ceiling") {
    auto rig = std::make_unique<Rig>(AccessLevel::control, 60.0f);
    rig->del.stateCount.clear();
    for (int i = 0; i < 1200; ++i) {   // 6 s
        g_clock.advanceUs(5000);
        g_census.position_mm = float(i % 100);   // moving, so no push repeats a value
        rig->hub->update(g_clock.nowUs());
        (void)rig->device.tick(g_clock.nowUs() / 1000);
        rig->client->update(g_clock.nowUs());
    }
    CHECK(std::abs(rig->del.stateCount[ch::motion] - 360) <= 2);
    CHECK(std::abs(rig->del.stateCount[ch::plan_strip] - 270) <= 2);
}

// A carriage at constant speed must publish constant steps: each 0x1100 push
// carries the census of the tick that pushed it, stamped with its instant in
// t_us. A second schedule in the delegate made every fifth step at 50 Hz span
// two (bd val-7f5).
TEST_CASE("VD-EVEN: at constant speed every 0x1100 push steps the same distance and t_us the same time") {
    auto rig = std::make_unique<Rig>(AccessLevel::control, 50.0f);
    rig->del.stateCount.clear();
    std::vector<int> steps, gaps;
    int seen = 0, lastPos = -1;
    uint32_t lastT = 0;
    for (int i = 0; i < 400; ++i) {   // 2 s at 0.5 mm per 5 ms tick
        g_clock.advanceUs(5000);
        g_census.position_mm = 0.5f * float(i);
        g_census.sample_us = g_clock.nowUs();
        rig->hub->update(g_clock.nowUs());
        (void)rig->device.tick(g_clock.nowUs() / 1000);
        rig->client->update(g_clock.nowUs());
        if (rig->del.stateCount[ch::motion] == seen) continue;
        seen = rig->del.stateCount[ch::motion];
        const std::span<const std::byte> s(rig->del.lastState[ch::motion]);
        REQUIRE(s.size() == 13);
        const int pos = getU16(s.subspan(0, 2));
        const uint32_t t = getU32(s.subspan(9, 4));
        if (lastPos >= 0 && pos > 0) {
            steps.push_back(pos - lastPos);
            gaps.push_back(int(t - lastT));
        }
        lastPos = pos;
        lastT = t;
    }
    // The first step spans the rig's own settling before the ramp.
    REQUIRE(steps.size() > 90);
    CHECK(std::all_of(steps.begin() + 1, steps.end(), [](int d) { return d == 200; }));   // 2 mm in 10 um
    CHECK(std::all_of(gaps.begin() + 1, gaps.end(), [](int g) { return g == 20000; }));
}

TEST_CASE("VD-MODES-9: home_speed writes on modes-set key 9, clamped, published at 0x1030's tail, stored") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    persistBitsOver(*rig, 2500);
    const uint16_t gen = rig->hub->cfgGen();
    CHECK(stateF32(*rig, ch::machine_modes, 8) == doctest::Approx(factory::home_speed));
    CHECK((stateU8(*rig, ch::machine_modes, 3) & 0x04) != 0);   // bit 2: home_speed, always open

    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(9, IntentValue::ofF32(25.0f))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    const IntentValue* v = echoed(rig->del.lastEcho, 9);
    REQUIRE(v != nullptr);
    CHECK(v->f32_val == 25.0f);
    CHECK(stateF32(*rig, ch::machine_modes, 8) == 25.0f);
    CHECK(rig->hub->cfgGen() == uint16_t(gen + 1));
    CHECK(storedF32(*rig, kBlobHomeSpeed) == 25.0f);
    CHECK((persistBitsOver(*rig, 2500) & kPersistConfig) != 0);

    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(9, IntentValue::ofF32(2.0f))).has_value());
    rig->step();
    v = echoed(rig->del.lastEcho, 9);
    REQUIRE(v != nullptr);
    CHECK(v->f32_val == ceiling::home_speed_min);
    CHECK(stateF32(*rig, ch::machine_modes, 8) == ceiling::home_speed_min);
}

// bd val-39e: both are accepted at all times, so a trial of either restores
// unconditionally (SPEC 9.3 "Trial writes" item 5). machine-modes without a
// drive: trial_mask at 7 (bit 2 home_speed, bit 3 datagram_estop), home_speed
// at 8, datagram_estop at 12.
TEST_CASE("VD-TR-4: home_speed and datagram_estop take a trial: live, marked, never stored, reverted") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    persistBitsOver(*rig, 2500);
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(9, IntentValue::ofF32(25.0f)), std::nullopt, false,
                                    true).has_value());
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(10, IntentValue::ofU64(0)), std::nullopt, false, true)
                .has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    CHECK(rig->hub->trialCount() == 2);
    CHECK(stateF32(*rig, ch::machine_modes, 8) == 25.0f);
    CHECK(stateU8(*rig, ch::machine_modes, 12) == 0);
    CHECK_FALSE(estopDatagramEnabled());
    CHECK(stateU8(*rig, ch::machine_modes, 7) == 0x0C);
    CHECK(storedF32(*rig, kBlobHomeSpeed) == doctest::Approx(factory::home_speed));
    std::array<std::byte, stored::kConfigBlobBytes> blob{};
    REQUIRE(rig->device.encodeConfigBlob(blob, rig->hub->cfgGen()) == blob.size());
    CHECK(blob[stored::kConfigV8Bytes] == std::byte{1});
    CHECK((persistBitsOver(*rig, 2500) & kPersistConfig) == 0);

    REQUIRE(rig->client->sendIntent(channels::settings_trial, oneKey(1, IntentValue::ofU64(trial_ops::revert)))
                .has_value());
    rig->step();
    CHECK(rig->hub->trialCount() == 0);
    CHECK(stateF32(*rig, ch::machine_modes, 8) == doctest::Approx(factory::home_speed));
    CHECK(stateU8(*rig, ch::machine_modes, 12) == 1);
    CHECK(estopDatagramEnabled());
    CHECK(stateU8(*rig, ch::machine_modes, 7) == 0x00);
}

// bd val-cem: the push of any config write hands the arbiter max_rail as its
// rail (setWindow()), so a flipped window is mirrored about the max_rail the
// same write leaves, never the rail of before it. The rig's motion layer is a
// census: each step below sets the rail the arbiter's takeWindow() would.
TEST_CASE("VD-FLIP-2: flipped, a write of max_rail and the window together lands the window where it was asked") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    g_census.rail_mm = rig->device.config().max_rail;   // 500, the factory rail
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(8, IntentValue::ofU64(1))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());

    const auto write = [&](float lo, float hi, float rail) {
        IntentValueMap m{};
        m.count = 3;
        m.fields[0] = IntentValueField{1, IntentValue::ofF32(lo)};
        m.fields[1] = IntentValueField{2, IntentValue::ofF32(hi)};
        m.fields[2] = IntentValueField{8, IntentValue::ofF32(rail)};
        REQUIRE(rig->client->sendIntent(ch::config_set, m).has_value());
        rig->step();
        g_census.rail_mm = rig->device.config().max_rail;
        rig->step();
    };
    // A longer rail: before the fix the window moved by the rail's change.
    write(50.0f, 150.0f, 800.0f);
    REQUIRE(rig->del.nacks.empty());
    CHECK(echoed(rig->del.lastEcho, 1)->f32_val == doctest::Approx(50.0f));
    CHECK(echoed(rig->del.lastEcho, 2)->f32_val == doctest::Approx(150.0f));
    CHECK(rig->device.config().window_min == doctest::Approx(650.0f));
    CHECK(rig->device.config().window_max == doctest::Approx(750.0f));
    CHECK(stateF32(*rig, ch::machine_config, 0) == doctest::Approx(50.0f));
    CHECK(stateF32(*rig, ch::machine_config, 4) == doctest::Approx(150.0f));

    // A shorter one: before the fix the window was mirrored past it and refused.
    write(20.0f, 120.0f, 300.0f);
    REQUIRE(rig->del.nacks.empty());
    CHECK(rig->device.config().window_min == doctest::Approx(180.0f));
    CHECK(rig->device.config().window_max == doctest::Approx(280.0f));
    CHECK(stateF32(*rig, ch::machine_config, 0) == doctest::Approx(20.0f));
    CHECK(stateF32(*rig, ch::machine_config, 4) == doctest::Approx(120.0f));
    CHECK(stateF32(*rig, ch::machine_config, 24) == doctest::Approx(300.0f));
}

// ---- RFC-087 supersede (bd val-dz9) ---------------------------------------------

namespace {

// A c2h segments bundle at t_base: each sample (pos e4, duration ms), 25 ms
// apart, end velocity unspecified, parsed the way the hub parses it.
BundleView segmentsBundle(std::vector<std::byte>& bytes, uint32_t t_base,
                          std::initializer_list<std::pair<uint16_t, uint16_t>> samples) {
    const size_t n = samples.size();
    bytes.assign(6 + 2 * n + 6 * n, std::byte{0});
    std::span<std::byte> out(bytes);
    putU32(out.subspan(0, 4), t_base);
    out[4] = std::byte(uint8_t(n));
    size_t i = 0;
    for (const auto& s : samples) {
        putU16(out.subspan(6 + 2 * i, 2), uint16_t(i * 250));   // 100 us units: 25 ms apart
        const size_t at = 6 + 2 * n + 6 * i;
        putU16(out.subspan(at, 2), s.first);
        putU16(out.subspan(at + 2, 2), s.second);
        putU16(out.subspan(at + 4, 2), uint16_t(limits::segment_end_vel_unspecified));
        ++i;
    }
    const auto parsed = BundleView::parse(std::span<const std::byte>(bytes), 6, limits::segment_t_off_unit_us,
                                          250'000u);
    REQUIRE(parsed);
    return parsed.value();
}

}  // namespace

TEST_CASE("VD-SEG-1: a segments bundle flushes on the first segment the motion path takes; a samples bundle never") {
    auto rig = std::make_unique<Rig>();
    const uint32_t now32 = uint32_t(g_clock.nowUs());
    std::vector<std::byte> bytes;

    rig->device.onStreamBundle(ch::motion_segment, 1, segmentsBundle(bytes, now32 + 20'000, {{5000, 25}, {6000, 25}, {7000, 25}}));
    REQUIRE(g_intents.size() == 3);
    CHECK(g_intents[0].supersede);
    CHECK_FALSE(g_intents[1].supersede);
    CHECK_FALSE(g_intents[2].supersede);

    // A zero-duration first sample never reaches the motion path: the flush
    // rides the next one.
    g_intents.clear();
    rig->device.onStreamBundle(ch::motion_segment, 1, segmentsBundle(bytes, now32 + 40'000, {{5000, 0}, {6000, 25}}));
    REQUIRE(g_intents.size() == 1);
    CHECK(g_intents[0].supersede);

    // 0x2100 samples: 4-byte points, never a flush.
    g_intents.clear();
    std::vector<std::byte> pts(6 + 2 * 2 + 4 * 2, std::byte{0});
    std::span<std::byte> out(pts);
    putU32(out.subspan(0, 4), now32 + 10'000);
    out[4] = std::byte{2};
    putU16(out.subspan(8, 2), 5000);   // t_off[1] = 5 ms
    putU16(out.subspan(10, 2), 5000);
    putU16(out.subspan(14, 2), 6000);
    const auto parsed = BundleView::parse(std::span<const std::byte>(pts), 4);
    REQUIRE(parsed);
    rig->device.onStreamBundle(ch::motion_input, 1, parsed.value());
    REQUIRE(g_intents.size() == 2);
    CHECK_FALSE(g_intents[0].supersede);
    CHECK_FALSE(g_intents[1].supersede);
}

TEST_CASE("VD-SEG-2: every sample past its time at ingress counts late; only segments bundles count as such") {
    auto rig = std::make_unique<Rig>();
    const uint32_t now32 = uint32_t(g_clock.nowUs());
    std::vector<std::byte> bytes;

    // Starts 30 ms and 5 ms ago, and 20 ms ahead: two late, one bundle.
    rig->device.onStreamBundle(ch::motion_segment, 1, segmentsBundle(bytes, now32 - 30'000, {{5000, 25}, {6000, 25}, {7000, 25}}));
    CHECK(rig->device.lateSamples() == 2);
    CHECK(rig->device.segBundles() == 1);

    // One point stamped 30 ms ago: late, and not a segments bundle.
    std::vector<std::byte> pts(6 + 2 + 4, std::byte{0});
    std::span<std::byte> out(pts);
    putU32(out.subspan(0, 4), now32 - 30'000);
    out[4] = std::byte{1};
    putU16(out.subspan(8, 2), 5000);   // t_off[0] = 0, then the point
    const auto parsed = BundleView::parse(std::span<const std::byte>(pts), 4);
    REQUIRE(parsed);
    rig->device.onStreamBundle(ch::motion_input, 1, parsed.value());
    CHECK(rig->device.lateSamples() == 3);
    CHECK(rig->device.segBundles() == 1);
}

// ---- bd val-68v / val-88t: the kinetic cards carry what Kinetic² reads ----------

TEST_CASE("VD-K2-1: the catalog advertises only settings the planner reads") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    CHECK(rig->catalog.find(0x1121) == nullptr);   // kinetic-chase, retired

    // Every setting on the two kinetic cards is a 0x3120 key, and every 0x3120
    // key a setting on exactly one of them.
    std::vector<uint8_t> settings;
    for (const uint16_t id : {ch::kinetic_limits, ch::kinetic_planner}) {
        const CatalogEntry* e = rig->catalog.find(id);
        REQUIRE(e != nullptr);
        for (const LayoutField& f : rig->catalog.layoutFields(*e))
            if (f.hasSettingKey) settings.push_back(f.settingKey);
    }
    std::vector<uint8_t> schema;
    const CatalogEntry* w = rig->catalog.find(ch::kinetic_set);
    REQUIRE(w != nullptr);
    for (const SchemaField& f : rig->catalog.schemaFields(*w)) schema.push_back(f.key);
    std::sort(settings.begin(), settings.end());
    std::sort(schema.begin(), schema.end());
    CHECK(settings == std::vector<uint8_t>{1, 2, 3, 4, 5, 6, 7, 8});
    CHECK(schema == settings);

    // kinetic-planner is Kinetic²'s set in mask order, every field a setting
    // but the mask pair; the factory defaults are kinetic2::Config's.
    const CatalogEntry* k = rig->catalog.find(ch::kinetic_planner);
    REQUIRE(k != nullptr);
    CHECK(k->name == "kinetic-planner");
    std::vector<std::string> names;
    for (const LayoutField& f : rig->catalog.layoutFields(*k)) {
        names.emplace_back(f.name);
        if (f.name == "smoothness") CHECK(f.dflt.asFloat() == 0.0f);
        if (f.name == "handle_floor") CHECK(f.dflt.asFloat() == doctest::Approx(0.15f));
        if (f.name == "trim_max") CHECK(f.dflt.asFloat() == 1.0f);
    }
    CHECK(names == std::vector<std::string>{"smoothness", "handle_floor", "trim_max", "chase_dense_ms", "react_ms",
                                            "enabled_mask", "trial_mask"});
    const size_t bytes = layoutWireSize(rig->catalog.layoutFields(*k));
    CHECK(bytes == 22);
    REQUIRE(rig->del.lastState.count(ch::kinetic_planner) == 1);
    CHECK(rig->del.lastState[ch::kinetic_planner].size() == bytes);
    CHECK(stateU8(*rig, ch::kinetic_planner, 20) == 0x1F);   // five settings, no samples grant
    CHECK(stateF32(*rig, ch::kinetic_planner, 4) == doctest::Approx(0.15f));
    CHECK(stateF32(*rig, ch::kinetic_planner, 8) == 1.0f);
    // kinetic-diag: one counter per kinetic2::AnomalyKind but none (RFC-108).
    const CatalogEntry* diag = rig->catalog.find(ch::motion_diag);
    REQUIRE(diag != nullptr);
    CHECK(layoutWireSize(rig->catalog.layoutFields(*diag)) == 72);

    // overshoot_clamp's byte stays on 0x1030, its key 4 on 0x3030 is a gap.
    const CatalogEntry* modes = rig->catalog.find(ch::modes_set);
    REQUIRE(modes != nullptr);
    for (const SchemaField& f : rig->catalog.schemaFields(*modes)) CHECK(f.key != 4);
    CHECK(stateU8(*rig, ch::machine_modes, 2) == 0);
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(4, IntentValue::ofU64(1))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
}

TEST_CASE("VD-K2-2: smoothness, handle_floor, trim_max and react_ms write on kinetic-set keys 4, 5, 6 and 8, clamped, published, stored") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    persistBitsOver(*rig, 2500);
    const uint16_t gen = rig->hub->cfgGen();
    IntentValueMap m{};
    m = plus(m, 4, IntentValue::ofF32(0.5f));
    m = plus(m, 5, IntentValue::ofF32(0.25f));
    m = plus(m, 6, IntentValue::ofF32(0.75f));
    m = plus(m, 8, IntentValue::ofF32(12.5f));
    REQUIRE(rig->client->sendIntent(ch::kinetic_set, m).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    for (const auto& [key, want] : {std::pair{4, 0.5f}, std::pair{5, 0.25f}, std::pair{6, 0.75f}, std::pair{8, 12.5f}}) {
        CAPTURE(key);
        REQUIRE(echoed(rig->del.lastEcho, uint8_t(key)) != nullptr);
        CHECK(echoed(rig->del.lastEcho, uint8_t(key))->f32_val == want);
    }
    CHECK(rig->hub->cfgGen() == uint16_t(gen + 1));
    // 0x1122: the three f32 at 0, 4, 8, react_ms as u32 microseconds at 16.
    CHECK(stateF32(*rig, ch::kinetic_planner, 0) == 0.5f);
    CHECK(stateF32(*rig, ch::kinetic_planner, 4) == 0.25f);
    CHECK(stateF32(*rig, ch::kinetic_planner, 8) == 0.75f);
    uint32_t react = 0;
    std::memcpy(&react, rig->del.lastState[ch::kinetic_planner].data() + 16, 4);
    CHECK(react == 12500);
    // The blob: react_ms in the v7 tail, the three in the v8 tail.
    std::array<std::byte, stored::kConfigBlobBytes> blob{};
    REQUIRE(rig->device.encodeConfigBlob(blob, rig->hub->cfgGen()) == blob.size());
    std::memcpy(&react, blob.data() + stored::kConfigV6Bytes + 1, 4);
    CHECK(react == 12500);
    CHECK(storedF32(*rig, stored::kConfigV7Bytes) == 0.5f);
    CHECK(storedF32(*rig, stored::kConfigV7Bytes + 4) == 0.25f);
    CHECK(storedF32(*rig, kBlobTrimMax) == 0.75f);
    CHECK((persistBitsOver(*rig, 2500) & kPersistConfig) != 0);

    // Clamped into the engine-mirroring bounds, echoed as taken.
    m = IntentValueMap{};
    m = plus(m, 4, IntentValue::ofF32(3.0f));
    m = plus(m, 5, IntentValue::ofF32(0.0f));
    m = plus(m, 6, IntentValue::ofF32(0.0f));
    m = plus(m, 8, IntentValue::ofF32(500.0f));
    REQUIRE(rig->client->sendIntent(ch::kinetic_set, m).has_value());
    rig->step();
    CHECK(echoed(rig->del.lastEcho, 4)->f32_val == 1.0f);
    CHECK(echoed(rig->del.lastEcho, 5)->f32_val == 0.05f);
    CHECK(echoed(rig->del.lastEcho, 6)->f32_val == 0.1f);
    CHECK(echoed(rig->del.lastEcho, 8)->f32_val == 100.0f);

    // A key outside the schema alone applies nothing.
    REQUIRE(rig->client->sendIntent(ch::kinetic_set, oneKey(9, IntentValue::ofF32(0.5f))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
}

// ---- RFC-103: the oscillator (bd val-dzf) -------------------------------------------

TEST_CASE("VD-OSC-1: osc-set clamps and echoes, reaches the motion door, publishes 0x1140; its session's end clears it") {
    auto rig = std::make_unique<Rig>();
    // Published at attach: off, at the factory values.
    {
        const auto it = rig->del.lastState.find(ch::oscillator);
        REQUIRE(it != rig->del.lastState.end());
        REQUIRE(it->second.size() == 56);
        CHECK(it->second[0] == std::byte{0});
        CHECK(getF32(std::span<const std::byte>(it->second).subspan(1, 4)) == MotionOsc{}.frequency_hz);
    }
    // The fixed drives: the fields are the parameters (the factory drive is axis).
    {
        IntentValueMap d{};
        d.count = 2;
        d.fields[0] = IntentValueField{7, IntentValue::ofU64(osc_drives::fixed)};
        d.fields[1] = IntentValueField{12, IntentValue::ofU64(osc_drives::fixed)};
        REQUIRE(rig->client->sendIntent(ch::osc_set, d).has_value());
        rig->step();
        REQUIRE(rig->del.nacks.empty());
        rig->del.echoes = 0;
    }
    IntentValueMap m{};
    m.count = 6;
    m.fields[0] = IntentValueField{1, IntentValue::ofBool(true)};
    m.fields[1] = IntentValueField{2, IntentValue::ofF32(150.0f)};   // past osc_max_hz
    m.fields[2] = IntentValueField{3, IntentValue::ofF32(5000.0f)};  // mm, past the ceiling
    m.fields[3] = IntentValueField{4, IntentValue::ofU64(1)};        // key 4 is unused: ignored
    m.fields[4] = IntentValueField{5, IntentValue::ofF32(0.333f)};   // two decimals
    m.fields[5] = IntentValueField{6, IntentValue::ofF32(9.0f)};     // past the longest hold
    REQUIRE(rig->client->sendIntent(ch::osc_set, m).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(rig->del.echoes == 1);
    CHECK(echoed(rig->del.lastEcho, 1)->bool_val);
    CHECK(echoed(rig->del.lastEcho, 2)->f32_val == OSC_MAX_HZ);
    CHECK(echoed(rig->del.lastEcho, 3)->f32_val == OSC_MAX_AMPLITUDE_MM);
    CHECK(echoed(rig->del.lastEcho, 4) == nullptr);   // absent: not applied (SPEC 9.3)
    CHECK(echoed(rig->del.lastEcho, 5)->f32_val == doctest::Approx(0.33f));
    CHECK(echoed(rig->del.lastEcho, 6)->f32_val == doctest::Approx(4.0f));
    CHECK(g_osc.enabled);
    CHECK(g_osc.frequency_hz == OSC_MAX_HZ);
    CHECK(g_osc.amplitude == OSC_MAX_AMPLITUDE_MM);
    // 0x1140: the applied parameters, then what renders, the census's.
    g_census.osc_active = true;
    g_census.osc_amplitude = 4.0f;
    rig->step(60);
    {
        const auto it = rig->del.lastState.find(ch::oscillator);
        REQUIRE(it != rig->del.lastState.end());
        const std::span<const std::byte> b(it->second);
        CHECK(b[0] == std::byte{1});
        CHECK(getF32(b.subspan(1, 4)) == OSC_MAX_HZ);
        CHECK(getF32(b.subspan(5, 4)) == OSC_MAX_AMPLITUDE_MM);
        CHECK(getF32(b.subspan(9, 4)) == doctest::Approx(0.33f));
        CHECK(getF32(b.subspan(13, 4)) == doctest::Approx(4.0f));
        CHECK(b[17] == std::byte{1});
        CHECK(getF32(b.subspan(18, 4)) == doctest::Approx(4.0f));
    }
    // Valence RFC-112: the four amplitude fields are mm to the ceiling, on
    // the intent and on the twin.
    {
        auto mm = [](std::string_view name, std::string_view unit, bool hasUnitId, uint8_t unitId, bool hasMax,
                     float max) {
            CAPTURE(name);
            CHECK(unit == "mm");
            CHECK(hasUnitId);
            CHECK(unitId == valence::unit_ids::mm);
            if (name != "amplitude_effective") CHECK((hasMax && max == OSC_MAX_AMPLITUDE_MM));
        };
        auto amplitude = [](std::string_view n) {
            return n == "amplitude" || n == "amplitude_out_min" || n == "amplitude_out_max" ||
                   n == "amplitude_effective";
        };
        size_t seen = 0;
        for (const LayoutField& f : rig->catalog.layoutFields(*rig->catalog.find(ch::oscillator)))
            if (amplitude(f.name)) mm(f.name, f.unit, f.hasUnitId, f.unitId, f.hasMax, f.max), ++seen;
        for (const SchemaField& f : rig->catalog.schemaFields(*rig->catalog.find(ch::osc_set)))
            if (amplitude(f.name)) mm(f.name, f.unit, f.hasUnitId, f.unitId, f.hasMax, f.max), ++seen;
        CHECK(seen == 7);
    }
    // A NaN refuses the whole write: nothing moves.
    IntentValueMap bad{};
    bad.count = 2;
    bad.fields[0] = IntentValueField{1, IntentValue::ofBool(false)};
    bad.fields[1] = IntentValueField{3, IntentValue::ofF32(std::numeric_limits<float>::quiet_NaN())};
    REQUIRE(rig->client->sendIntent(ch::osc_set, bad).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
    CHECK(g_osc.enabled);
    // Its session's end clears enabled (SPEC 9.7), publishes it, moves cfg_gen.
    const uint16_t gen = rig->hub->cfgGen();
    rig->device.onSessionLeft(rig->client->sessionId());
    rig->step();
    CHECK_FALSE(g_osc.enabled);
    CHECK(rig->hub->cfgGen() != gen);
    CHECK(rig->del.lastState[ch::oscillator][0] == std::byte{0});
}

// ---- RFC-053 item 3: the datagram_estop setting -----------------------------------

TEST_CASE("VD-DGRAM: datagram_estop is configure tier, drives the datagram switch and survives a reboot") {
    {
        auto rig = std::make_unique<Rig>();
        REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(10, IntentValue::ofU64(0))).has_value());
        rig->step();
        REQUIRE(rig->del.nacks.size() == 1);
        CHECK(rig->del.nacks[0].code == NackCode::ACCESS_DENIED);
        CHECK(estopDatagramEnabled());
    }
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    const auto modesByte = [&] {
        const auto it = rig->del.lastState.find(ch::machine_modes);
        REQUIRE(it != rig->del.lastState.end());
        return it->second[it->second.size() - 2];   // radios follows it
    };
    CHECK(modesByte() == std::byte{1});

    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(10, IntentValue::ofU64(0))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(rig->del.lastEcho.count == 1);
    CHECK(rig->del.lastEcho.fields[0].key == 10);
    CHECK(rig->del.lastEcho.fields[0].value.u64_val == 0);
    CHECK_FALSE(estopDatagramEnabled());
    CHECK(modesByte() == std::byte{0});

    // Stored, and a reboot adopts it off. The rig's tuning is the struct's
    // zeros, which no stored blob passes, so the adopted blob carries a valid set.
    std::array<std::byte, stored::kConfigBlobBytes> blob{};
    REQUIRE(rig->device.encodeConfigBlob(blob, 3) == blob.size());
    CHECK(blob[stored::kConfigV8Bytes] == std::byte{0});
    MotionTuning tune;
    tune.chase_dense_us = 20000;
    StoredModes modes;
    modes.datagram_estop = false;
    REQUIRE(stored::encodeConfig(blob, StoredConfig{}, tune, modes, 3) == blob.size());
    estopDatagramSetEnabled(true);
    {
        auto fresh = std::make_unique<ValenceDevice>();
        uint16_t gen = 0;
        REQUIRE(fresh->adoptConfigBlob(blob, gen));
        CHECK_FALSE(estopDatagramEnabled());
    }

    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(10, IntentValue::ofU64(1))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    CHECK(estopDatagramEnabled());
    CHECK(modesByte() == std::byte{1});
}

// ---- RFC-096: the section separator is the registry's -----------------------------

TEST_CASE("VD-GROUP: every group string in the catalog that has a section spells the registry's separator") {
    auto rig = std::make_unique<Rig>();
    const Catalog32& c = rig->catalog;
    int sectioned = 0;
    auto check = [&](std::string_view g) {
        CAPTURE(std::string(g));
        CHECK(sectionedByRegistry(g));
        if (g.find('/') != std::string_view::npos) ++sectioned;
    };
    for (uint16_t i = 0; i < c.count; ++i) {
        for (const LayoutField& f : c.layoutFields(c.entries[i])) check(f.group);
        for (const SchemaField& f : c.schemaFields(c.entries[i])) check(f.group);
    }
    CHECK(sectioned > 0);
    CHECK_FALSE(sectionedByRegistry("Tuning/Planner"));
    CHECK_FALSE(sectionedByRegistry("Tuning /Planner"));
}

// bd val-3xd: catalog.cddl caps a group at 24 bytes and a C++ client rejects the
// whole catalog over one that is longer.
TEST_CASE("VD-GROUP-24: no group string in the catalog is over the spec's 24-byte cap") {
    auto rig = std::make_unique<Rig>();
    const Catalog32& c = rig->catalog;
    auto check = [&](std::string_view g) {
        CAPTURE(std::string(g));
        CHECK(g.size() <= 24);
    };
    for (uint16_t i = 0; i < c.count; ++i) {
        for (const LayoutField& f : c.layoutFields(c.entries[i])) check(f.group);
        for (const SchemaField& f : c.schemaFields(c.entries[i])) check(f.group);
    }
}

// Valence rfc-tcmf: the library's own authoring check, every size cap the
// decoder enforces and the per-entry byte cap, over the board's catalog.
TEST_CASE("VD-CONFORM: the board catalog passes conformance::checkCatalog") {
    auto rig = std::make_unique<Rig>();
    static std::array<std::byte, limits::catalog_max_entry_bytes + 64> scratch{};
    const auto r = conformance::checkCatalog(rig->catalog, scratch);
    for (size_t i = 0; i < r.count; ++i) {
        CAPTURE(unsigned(r.violations[i].kind));
        CAPTURE(r.violations[i].channel_id);
        CHECK(false);
    }
    CHECK(r.ok());
}

// ---- bd val-4rr, val-y3da: a persist waits for rest ---------------------------------

// A stream in flight, as the census shows it in Run.
void streamRuns(bool on) {
    g_census.busy = on;
    g_census.stream = on;
    g_census.step_q8 = on ? 4000u : 0u;
}

TEST_CASE("VD-PERSIST-1: a config-set during a stream applies at once and is written once, at rest") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    rig->step(3000);
    rig->persisted = 0;
    rig->persistWrites = 0;
    streamRuns(true);
    const float before = rig->device.config().jog_speed;
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(3, IntentValue::ofF32(before + 7.0f))).has_value());
    rig->step(3000);   // past the debounce
    REQUIRE(rig->del.nacks.empty());
    CHECK(stateF32(*rig, ch::machine_config, 8) == doctest::Approx(before + 7.0f));   // live now
    CHECK(rig->persisted == 0);
    CHECK(rig->device.pendingPersist() == kPersistConfig);
    // A second change while moving coalesces into the same write; nothing
    // stops the stream for it.
    REQUIRE(rig->client->sendIntent(ch::config_set, oneKey(3, IntentValue::ofF32(before + 9.0f))).has_value());
    rig->step(10000);
    CHECK(rig->persisted == 0);
    CHECK_FALSE(g_census.estop);
    CHECK_FALSE(g_census.paused);

    // An oscillation alone also holds it.
    streamRuns(false);
    g_census.osc_active = true;
    rig->step(100);
    CHECK(rig->persisted == 0);
    g_census.osc_active = false;
    rig->step(1);
    CHECK(rig->persisted == kPersistConfig);
    CHECK(rig->persistWrites == 1);
    CHECK(rig->device.pendingPersist() == 0);
    CHECK(storedF32(*rig, kBlobJogSpeed) == doctest::Approx(before + 9.0f));
    rig->step(3000);
    CHECK(rig->persistWrites == 1);
}

TEST_CASE("VD-PERSIST-2: the mode's rule decides: ESTOP writes whatever moves, a cycle holds") {
    {
        auto rig = std::make_unique<Rig>(AccessLevel::configure);
        rig->step(3000);
        rig->persisted = 0;
        streamRuns(true);
        g_census.flash = FlashRule::open;   // Estop
        REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(9, IntentValue::ofF32(30.0f))).has_value());
        rig->step(3000);
        CHECK(rig->persisted == kPersistConfig);
        streamRuns(false);
    }
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    rig->step(3000);
    rig->persisted = 0;
    g_census.flash = FlashRule::hold;   // a home cycle, at rest between legs
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(9, IntentValue::ofF32(30.0f))).has_value());
    rig->step(3000);
    CHECK(rig->persisted == 0);
    g_census.flash = FlashRule::rest;
    rig->step(1);
    CHECK(rig->persisted == kPersistConfig);
}

// ---- SPEC 9.7 driven parameters and the osc-drive stream (bd val-o9r) -------------

TEST_CASE("VD-OSC-DRIVE: drives default to axis, clamp and echo; osc-drive samples reach the motion task at their stamps") {
    auto rig = std::make_unique<Rig>();
    // attach pushed the factory set.
    CHECK(g_osc.frequency_drive.drive == osc_drives::axis);
    CHECK(g_osc.amplitude_drive.drive == osc_drives::axis);
    CHECK(g_osc.frequency_drive.out_max == OSC_MAX_HZ);
    CHECK(g_osc.amplitude_drive.out_max == 20.0f);   // mm: a full V8
    CHECK(g_osc.amplitude == 5.0f);                  // mm by hand
    {
        const auto it = rig->del.lastState.find(ch::oscillator);
        REQUIRE(it != rig->del.lastState.end());
        REQUIRE(it->second.size() == 56);
        CHECK(it->second[22] == std::byte{osc_drives::axis});
        CHECK(it->second[39] == std::byte{osc_drives::axis});
    }
    CHECK_FALSE(rig->device.sourceForChannel(ch::osc_drive).has_value());

    IntentValueMap m{};
    m.count = 4;
    m.fields[0] = IntentValueField{7, IntentValue::ofU64(osc_drives::speed)};
    m.fields[1] = IntentValueField{11, IntentValue::ofF32(500.0f)};   // past osc_max_hz
    m.fields[2] = IntentValueField{13, IntentValue::ofF32(0.2f)};
    m.fields[3] = IntentValueField{14, IntentValue::ofF32(0.8f)};
    REQUIRE(rig->client->sendIntent(ch::osc_set, m).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    CHECK(echoed(rig->del.lastEcho, 7)->u64_val == osc_drives::speed);
    CHECK(echoed(rig->del.lastEcho, 11)->f32_val == OSC_MAX_HZ);
    CHECK(echoed(rig->del.lastEcho, 13)->f32_val == doctest::Approx(0.2f));
    CHECK(g_osc.frequency_drive.drive == osc_drives::speed);
    CHECK(g_osc.amplitude_drive.in_min == doctest::Approx(0.2f));
    CHECK(g_osc.amplitude_drive.in_max == doctest::Approx(0.8f));

    // Equal input bounds have no map: refused, nothing moves.
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(13, IntentValue::ofF32(0.8f))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
    CHECK(rig->del.nacks[0].detail == "drive in_min equals in_max");
    CHECK(g_osc.amplitude_drive.in_min == doctest::Approx(0.2f));

    // Driven by the plan, the twin and the echo report the sine that plays, no
    // dwells; the written values are kept.
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(5, IntentValue::ofF32(0.3f))).has_value());
    rig->step();
    CHECK(echoed(rig->del.lastEcho, 5)->f32_val == 0.0f);
    {
        const std::span<const std::byte> tw(rig->del.lastState.find(ch::oscillator)->second);
        CHECK(getF32(tw.subspan(9, 4)) == 0.0f);
    }
    CHECK(g_osc.dwell_crest == doctest::Approx(0.3f));
    CHECK(rig->device.scheduleLatencyUs(ch::osc_drive) == kOscDriveLeadUs);

    // Two samples 10 ms and 30 ms ahead: each handed over at arrival with its
    // stamp, the frequency clamped into 0 .. 1.
    g_oscDrives.clear();
    const uint32_t now32 = uint32_t(g_clock.nowUs());
    std::vector<std::byte> b(6 + 2 * 2 + 8 * 2, std::byte{0});
    std::span<std::byte> out(b);
    putU32(out.subspan(0, 4), now32 + 10'000);
    out[4] = std::byte{2};
    putU16(out.subspan(8, 2), 20'000);   // t_off[1] = 20 ms
    putF32(out.subspan(10, 4), 0.5f);
    putF32(out.subspan(14, 4), 0.25f);
    putF32(out.subspan(18, 4), 0.75f);
    putF32(out.subspan(22, 4), 1.5f);
    const auto parsed = BundleView::parse(std::span<const std::byte>(b), 8);
    REQUIRE(parsed.isOk());
    rig->device.onStreamBundle(ch::osc_drive, 1, parsed.value());
    REQUIRE(g_oscDrives.size() == 2);
    CHECK(g_oscDrives[0].amplitude == 0.5f);
    CHECK(g_oscDrives[0].frequency == 0.25f);
    CHECK(g_oscDrives[0].at_us == g_clock.nowUs() + 10'000);
    CHECK(g_oscDrives[1].amplitude == 0.75f);
    CHECK(g_oscDrives[1].frequency == 1.0f);
    CHECK(g_oscDrives[1].at_us == g_clock.nowUs() + 30'000);
}

// ---- the session behind osc.enabled (Valence rfc-ns5c) ----------------------

TEST_CASE("VD-OSC-SESSION: only the end of the session that set osc.enabled clears it, its silence included") {
    auto rig = std::make_unique<Rig>();
    const uint32_t mine = rig->client->sessionId();
    REQUIRE(mine != 0);
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(1, IntentValue::ofBool(true))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(g_osc.enabled);
    // Another session's end, by either door: it stays on, nothing published.
    const uint16_t gen = rig->hub->cfgGen();
    rig->device.onSessionStale(mine + 1);
    rig->device.onSessionLeft(mine + 1);
    rig->step();
    CHECK(g_osc.enabled);
    CHECK(rig->hub->cfgGen() == gen);
    CHECK(rig->del.lastState[ch::oscillator][0] == std::byte{1});
    // A write that leaves osc.enabled out keeps its writer.
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(2, IntentValue::ofF32(12.0f))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    // The writer falls silent: the hub's own STALE door clears it.
    bool cleared = false;
    for (int i = 0; i < 6000 && !cleared; ++i) {
        g_clock.advanceUs(10'000);
        rig->hub->update(g_clock.nowUs());
        rig->device.tick(g_clock.nowUs() / 1000);
        cleared = !g_osc.enabled;
    }
    CHECK(cleared);
    CHECK(rig->hub->cfgGen() != gen);
}

TEST_CASE("VD-OSC-SESSION-2: the stale hook clears the writer's setting and publishes it") {
    auto rig = std::make_unique<Rig>();
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(1, IntentValue::ofBool(true))).has_value());
    rig->step();
    REQUIRE(g_osc.enabled);
    const uint16_t gen = rig->hub->cfgGen();
    rig->device.onSessionStale(rig->client->sessionId());
    rig->step();
    CHECK_FALSE(g_osc.enabled);
    CHECK(rig->hub->cfgGen() != gen);
    CHECK(rig->del.lastState[ch::oscillator][0] == std::byte{0});
    // WELCOME limits key 7 is the frequency bound.
    CHECK(rig->device.oscMaxHz() == OSC_MAX_HZ);
}

TEST_CASE("VD-HOME-OSC: home op 1 is refused INTERLOCK while the oscillator is on or renders; off, it starts") {
    auto rig = std::make_unique<Rig>();
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(1, IntentValue::ofBool(true))).has_value());
    rig->step();
    REQUIRE(g_osc.enabled);
    REQUIRE(rig->client->sendIntent(ch::home, homeOp1()).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INTERLOCK);
    CHECK(rig->del.nacks[0].detail == "oscillating: disable the oscillator first");
    CHECK(g_homeCalls == 0);
    // Disabled, but a stream still renders it.
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(1, IntentValue::ofBool(false))).has_value());
    rig->step();
    g_census.osc_active = true;
    REQUIRE(rig->client->sendIntent(ch::home, homeOp1()).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 2);
    CHECK(rig->del.nacks[1].code == NackCode::INTERLOCK);
    CHECK(g_homeCalls == 0);
    // Neither, but a live stream with the factory axis drives renders it.
    g_census.osc_active = false;
    g_census.osc_stream_live = true;
    REQUIRE(rig->client->sendIntent(ch::home, homeOp1()).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 3);
    CHECK(rig->del.nacks[2].code == NackCode::INTERLOCK);
    g_census.osc_stream_live = false;
    REQUIRE(rig->client->sendIntent(ch::home, homeOp1()).has_value());
    rig->step();
    CHECK(rig->del.nacks.size() == 3);
    CHECK(g_homeCalls == 1);
}

// ---- dwells as they play (Valence RFC-110 item 3) ---------------------------

TEST_CASE("VD-OSC-SHOWN: the factory card reports its dwells by hand, a live stream none, each change a hub-side cfg_gen") {
    auto rig = std::make_unique<Rig>();
    IntentValueMap m{};
    m.count = 2;
    m.fields[0] = IntentValueField{5, IntentValue::ofF32(0.25f)};
    m.fields[1] = IntentValueField{6, IntentValue::ofF32(0.5f)};
    REQUIRE(rig->client->sendIntent(ch::osc_set, m).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    CHECK(echoed(rig->del.lastEcho, 5)->f32_val == doctest::Approx(0.25f));
    auto twin = [&] { return std::span<const std::byte>(rig->del.lastState[ch::oscillator]); };
    CHECK(getF32(twin().subspan(9, 4)) == doctest::Approx(0.25f));
    CHECK(getF32(twin().subspan(13, 4)) == doctest::Approx(0.5f));
    // A stream goes live: no dwells, published with cfg_gen moved.
    uint16_t gen = rig->hub->cfgGen();
    g_census.osc_stream_live = true;
    rig->step(60);
    CHECK(getF32(twin().subspan(9, 4)) == 0.0f);
    CHECK(getF32(twin().subspan(13, 4)) == 0.0f);
    CHECK(rig->hub->cfgGen() != gen);
    // A dwell written while it is live is kept and echoed 0.
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(5, IntentValue::ofF32(0.4f))).has_value());
    rig->step();
    CHECK(echoed(rig->del.lastEcho, 5)->f32_val == 0.0f);
    CHECK(g_osc.dwell_crest == doctest::Approx(0.4f));
    // Quiet: the kept values again, a hub-side change.
    gen = rig->hub->cfgGen();
    g_census.osc_stream_live = false;
    rig->step(60);
    CHECK(getF32(twin().subspan(9, 4)) == doctest::Approx(0.4f));
    CHECK(getF32(twin().subspan(13, 4)) == doctest::Approx(0.5f));
    CHECK(rig->hub->cfgGen() != gen);
    // No dwells report nothing new: no cfg_gen.
    m.fields[0] = IntentValueField{5, IntentValue::ofF32(0.0f)};
    m.fields[1] = IntentValueField{6, IntentValue::ofF32(0.0f)};
    REQUIRE(rig->client->sendIntent(ch::osc_set, m).has_value());
    rig->step();
    gen = rig->hub->cfgGen();
    g_census.osc_stream_live = true;
    rig->step(60);
    g_census.osc_stream_live = false;
    rig->step(60);
    CHECK(rig->hub->cfgGen() == gen);
    // Both drives fixed: a live stream drives nothing, the dwells play.
    m.count = 3;
    m.fields[0] = IntentValueField{5, IntentValue::ofF32(0.25f)};
    m.fields[1] = IntentValueField{7, IntentValue::ofU64(osc_drives::fixed)};
    m.fields[2] = IntentValueField{12, IntentValue::ofU64(osc_drives::fixed)};
    REQUIRE(rig->client->sendIntent(ch::osc_set, m).has_value());
    rig->step();
    g_census.osc_stream_live = true;
    rig->step(60);
    CHECK(getF32(twin().subspan(9, 4)) == doctest::Approx(0.25f));
}

// ---- SPEC 5.4 (RFC-090): an over-cap bundle moves earlier whole -------------

TEST_CASE("VD-LEADCAP: a bundle stamped past the lead cap moves earlier whole, its spacing kept, on osc-drive and motion-input") {
    auto rig = std::make_unique<Rig>();
    const uint64_t now = g_clock.nowUs();
    const uint32_t now32 = uint32_t(now);
    const uint64_t cap = uint64_t(limits::max_future_schedule_ms) * 1000u;
    // osc-drive: two samples at now + 300 and now + 320 ms, past the 250 ms cap.
    g_oscDrives.clear();
    std::vector<std::byte> b(6 + 2 * 2 + 8 * 2, std::byte{0});
    std::span<std::byte> out(b);
    putU32(out.subspan(0, 4), now32 + 300'000);
    out[4] = std::byte{2};
    putU16(out.subspan(8, 2), 20'000);
    putF32(out.subspan(10, 4), 0.5f);
    putF32(out.subspan(14, 4), 0.25f);
    putF32(out.subspan(18, 4), 0.75f);
    putF32(out.subspan(22, 4), 0.5f);
    const auto parsed = BundleView::parse(std::span<const std::byte>(b), 8);
    REQUIRE(parsed.isOk());
    rig->device.onStreamBundle(ch::osc_drive, 1, parsed.value());
    REQUIRE(g_oscDrives.size() == 2);
    CHECK(g_oscDrives[1].at_us == now + cap);
    CHECK(g_oscDrives[0].at_us == now + cap - 20'000);
    // motion-input: two points 5 ms apart at now + 400 ms.
    g_intents.clear();
    std::vector<std::byte> pts(6 + 2 * 2 + 4 * 2, std::byte{0});
    std::span<std::byte> po(pts);
    putU32(po.subspan(0, 4), now32 + 400'000);
    po[4] = std::byte{2};
    putU16(po.subspan(8, 2), 5000);
    putU16(po.subspan(10, 2), 5000);
    putU16(po.subspan(14, 2), 6000);
    const auto pp = BundleView::parse(std::span<const std::byte>(pts), 4);
    REQUIRE(pp);
    rig->device.onStreamBundle(ch::motion_input, 1, pp.value());
    REQUIRE(g_intents.size() == 2);
    CHECK(g_intents[1].anchor_us == now + cap);
    CHECK(g_intents[0].anchor_us == now + cap - 5000);
}

TEST_CASE("VD-TCP: hub-status carries the binding's resent share at bytes 16-17, hundredths of a percent") {
    auto rig = std::make_unique<Rig>(AccessLevel::control, 0.0f, true);
    g_linkTcp = LinkTcp{1000u, 10u};
    rig->step(2500);
    const auto it = rig->del.lastState.find(channels::hub_status);
    REQUIRE(it != rig->del.lastState.end());
    REQUIRE(it->second.size() == 18);
    const std::span<const std::byte> s(it->second);
    // 10 of 1000 since the attach-time snapshot of zeros: 1 %.
    CHECK(getU16(s.subspan(16, 2)) == 100u);
    const CatalogEntry* e = rig->catalog.find(channels::hub_status);
    REQUIRE(e != nullptr);
    const auto fields = rig->catalog.layoutFields(*e);
    REQUIRE(fields.size() == 8);
    CHECK(fields[7].name == "resent");
    CHECK(fields[7].role == "link.resent");
    CHECK(fields[7].scale == 100.0f);
    CHECK(fields[2].role == "link.rssi");
}

TEST_CASE("VD-PWR: the power layout tags bus voltage and power draw as two roles, in volts and watts") {
    auto cat = std::make_unique<Catalog32>();
    DeviceFeatures feat = boardFeatures();
    feat.has_current_sensor = true;
    feat.has_power_monitor = true;
    REQUIRE(buildValenceCatalog(*cat, feat));
    const CatalogEntry* e = cat->find(ch::power);
    REQUIRE(e != nullptr);
    const auto f = cat->layoutFields(*e);
    REQUIRE(f.size() == 3);
    CHECK((f[0].role == "telemetry.power.bus" && f[0].unitId == unit_ids::v));
    CHECK((f[1].name == "draw_w10" && f[1].role == "telemetry.power.draw" && f[1].unitId == unit_ids::w));
    CHECK(layoutWireSize(f) == 6);
    CHECK(e->maxRateHz == 1.0f);
}

TEST_CASE("VD-PWR-PACK: 0x1010 carries the switch task's reading in mV and tenths of a watt, 65535 for none") {
    auto rig = std::make_unique<Rig>(AccessLevel::control, 0.0f, true);
    const CatalogEntry* e = rig->catalog.find(ch::power);
    REQUIRE(e != nullptr);   // the board advertises it (boardFeatures, val-9hr)
    REQUIRE(layoutWireSize(rig->catalog.layoutFields(*e)) == 4);
    auto packed = [&]() {
        const auto it = rig->del.lastState.find(ch::power);
        REQUIRE(it != rig->del.lastState.end());
        REQUIRE(it->second.size() == 4);
        const std::span<const std::byte> s(it->second);
        return std::pair{getU16(s.subspan(0, 2)), getU16(s.subspan(2, 2))};
    };

    g_powerNow = {36.0f, 12.34f};
    rig->step(2500);
    CHECK(packed() == std::pair<uint16_t, uint16_t>{36000, 123});

    // A failed read: both fields go no reading, never 0.
    g_powerNow = {};
    rig->step(2500);
    CHECK(packed() == std::pair<uint16_t, uint16_t>{65535, 65535});

    // Voltage without a draw, a real 0 W, and a reading past the field saturating under none.
    g_powerNow = {70.0f, 0.0f};
    rig->step(2500);
    CHECK(packed() == std::pair<uint16_t, uint16_t>{65534, 0});
    g_powerNow = {};
}

// ---- PAIR gestures (Valence RFC-114 draft) -----------------------------------

namespace {

// A configure owner in the ledger, as an earlier ceremony would have left one.
void seedConfigureOwner(Rig& rig) {
    std::array<std::byte, limits::instance_id_bytes> inst{};
    inst[0] = std::byte{0x55};
    XorShift32 rng(99);
    std::array<std::byte, limits::token_bytes> tok{};
    REQUIRE(rig.hub->pairing().grant(inst, AccessLevel::configure, pairing_modes::push_to_pair, rng, tok) != nullptr);
    REQUIRE(rig.hub->pairing().hasConfigureToken());
}

void pair(Rig& rig, button::PairGesture g) {
    g_pairGesture = g;
    rig.step();
}

}  // namespace

TEST_CASE("VD-PAIR-1: with configure owners, a single press grants a knock control, not configure") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    pair(*rig, button::PairGesture::press);
    REQUIRE(rig->client->sendPairKnock());
    rig->step();
    REQUIRE(rig->del.granted.has_value());
    CHECK(*rig->del.granted == AccessLevel::control);
}

TEST_CASE("VD-PAIR-2: a double press approves the one configure request, on the session that asked") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    Peer other(*rig, 9);
    REQUIRE(rig->client->sendPairKnock(AccessLevel::configure));
    REQUIRE(other.client->sendPairKnock());   // a bare knock beside it
    other.step(*rig);
    CHECK(rig->device.pairLook() == PairLook::request);
    pair(*rig, button::PairGesture::double_press);
    other.step(*rig);
    REQUIRE(rig->del.granted.has_value());
    CHECK(*rig->del.granted == AccessLevel::configure);
    CHECK_FALSE(other.del.granted.has_value());
    CHECK(rig->device.pairLook() == PairLook::quiet);
}

TEST_CASE("VD-PAIR-3: a double press with no configure request approves nothing and says so") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    pair(*rig, button::PairGesture::double_press);
    CHECK(rig->device.pairLook() == PairLook::refused);
    CHECK_FALSE(rig->hub->presenceWindowOpen());   // not a window either
    rig->step(int(kPairFlashMs));
    CHECK(rig->device.pairLook() == PairLook::quiet);
}

TEST_CASE("VD-PAIR-4: two configure requests: a double press approves neither") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    Peer rival(*rig, 9);
    REQUIRE(rig->client->sendPairKnock(AccessLevel::configure));
    REQUIRE(rival.client->sendPairKnock(AccessLevel::configure));
    rival.step(*rig);
    CHECK(rig->device.pairLook() == PairLook::quiet);   // nothing a double press could approve
    pair(*rig, button::PairGesture::double_press);
    rival.step(*rig);
    CHECK_FALSE(rig->del.granted.has_value());
    CHECK_FALSE(rival.del.granted.has_value());
    CHECK(rig->device.pairLook() == PairLook::refused);
}

TEST_CASE("VD-PAIR-5: a configure request older than the window approves nothing") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    REQUIRE(rig->client->sendPairKnock(AccessLevel::configure));
    rig->step();
    REQUIRE(rig->device.pairLook() == PairLook::request);
    for (int i = 0; i < 1220; ++i) rig->step(100);   // 122 s, the client pinging throughout
    REQUIRE(rig->client->state() == ClientSessionState::LIVE);
    pair(*rig, button::PairGesture::double_press);
    rig->step();
    CHECK_FALSE(rig->del.granted.has_value());
    CHECK(rig->device.pairLook() == PairLook::refused);
}

TEST_CASE("VD-PAIR-6: a 5 s hold demotes every client, stops its motion, and Halt still works after") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    // The control session drives: the oscillator is its motion.
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(1, IntentValue::ofBool(true))).has_value());
    rig->step();
    REQUIRE(g_osc.enabled);

    pair(*rig, button::PairGesture::hold);
    CHECK(rig->client->state() == ClientSessionState::CLOSED);   // evicted
    CHECK_FALSE(g_osc.enabled);                                   // its session's end stopped it
    CHECK_FALSE(rig->hub->pairing().hasConfigureToken());
    CHECK(rig->hub->pairing().entryCount() == 1);                 // kept, at watch
    CHECK(rig->device.pairLook() == PairLook::demoted);

    // Back at watch, though this rig vouches control for anyone unpaired.
    REQUIRE(rig->client->connect());
    rig->step(200);
    REQUIRE(rig->client->state() == ClientSessionState::LIVE);
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(1, IntentValue::ofBool(true))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::NOT_CONTROLLER);
    // Pause and e-stop are role-exempt: a demoted client can still stop the machine.
    rig->del.nacks.clear();
    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::pause)).has_value());
    rig->step();
    CHECK(rig->del.nacks.empty());
    CHECK(rig->hub->pauseLatched());
    REQUIRE(rig->client->sendIntent(channels::safety_intents, safetyOp(safety_ops::estop)).has_value());
    rig->step();
    CHECK(rig->del.nacks.empty());
    CHECK(rig->hub->estopLatched());
}

TEST_CASE("VD-PAIR-7: after a demote, a configure request and a double press restore configure") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    pair(*rig, button::PairGesture::hold);
    REQUIRE(rig->client->connect());
    rig->step(200);
    REQUIRE(rig->client->sendPairKnock(AccessLevel::configure));
    rig->step();
    pair(*rig, button::PairGesture::double_press);
    rig->step();
    REQUIRE(rig->del.granted.has_value());
    CHECK(*rig->del.granted == AccessLevel::configure);
    CHECK(rig->hub->pairing().hasConfigureToken());
}

TEST_CASE("VD-PAIR-8: a single press demotes nothing") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    pair(*rig, button::PairGesture::press);
    CHECK(rig->client->state() == ClientSessionState::LIVE);
    CHECK(rig->hub->pairing().hasConfigureToken());
}

TEST_CASE("VD-PAIR-9: after a demote the hub stays claimed: a press grants control, and nothing is vouched above watch") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    pair(*rig, button::PairGesture::hold);
    pair(*rig, button::PairGesture::press);
    REQUIRE(rig->client->connect());
    rig->step(200);
    REQUIRE(rig->client->sendPairKnock());
    rig->step();
    REQUIRE(rig->del.granted.has_value());
    CHECK(*rig->del.granted == AccessLevel::control);
}

// ---- PAIR in policy config mode (SPEC 12.3, 13.4.1) ------------------------
// The hub glue sets the route once (ValenceHub.cpp hubBegin); so does the rig.

using linkmode::Boot;

TEST_CASE("VD-ROUTE-1: in policy config mode a single press approves the one configure ask, on its session, and names it for the gate") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    rig->device.setRoute(Boot::config_policy);
    REQUIRE(rig->client->sendPairKnock(AccessLevel::configure));
    rig->step();
    REQUIRE(rig->device.pairLook() == PairLook::request);
    pair(*rig, button::PairGesture::press);
    rig->step();
    REQUIRE(rig->del.granted.has_value());
    CHECK(*rig->del.granted == AccessLevel::configure);
    CHECK(rig->device.takePresenceApproval() == rig->client->sessionId());
    CHECK(rig->device.takePresenceApproval() == 0);   // once
    CHECK_FALSE(rig->hub->presenceWindowOpen());       // approving opens no window
}

TEST_CASE("VD-ROUTE-2: in policy config mode, two configure asks: a single press approves neither") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    rig->device.setRoute(Boot::config_policy);
    Peer rival(*rig, 9);
    REQUIRE(rig->client->sendPairKnock(AccessLevel::configure));
    REQUIRE(rival.client->sendPairKnock(AccessLevel::configure));
    rival.step(*rig);
    pair(*rig, button::PairGesture::press);
    rival.step(*rig);
    CHECK_FALSE(rig->del.granted.has_value());
    CHECK_FALSE(rival.del.granted.has_value());
    CHECK(rig->device.pairLook() == PairLook::refused);
    CHECK(rig->device.takePresenceApproval() == 0);
}

TEST_CASE("VD-ROUTE-3: in policy config mode, claimed, no ask: a press opens no window; in gesture config mode it does, granting configure") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    rig->device.setRoute(Boot::config_policy);
    pair(*rig, button::PairGesture::press);
    CHECK_FALSE(rig->hub->presenceWindowOpen());
    CHECK(rig->device.pairLook() == PairLook::refused);

    rig->hub->setPresenceDefaultRole(AccessLevel::configure);   // the gesture entry's window (ValenceHub.cpp)
    rig->device.setRoute(Boot::config_gesture);
    pair(*rig, button::PairGesture::press);
    CHECK(rig->hub->presenceWindowOpen());
    REQUIRE(rig->client->sendPairKnock());
    rig->step();
    REQUIRE(rig->del.granted.has_value());
    CHECK(*rig->del.granted == AccessLevel::configure);
}

TEST_CASE("VD-ROUTE-4: outside policy config mode the single press is RFC-114's: a window granting control, the ask untouched") {
    for (const Boot b : {Boot::normal, Boot::radios_off}) {
        auto rig = std::make_unique<Rig>();
        seedConfigureOwner(*rig);
        rig->device.setRoute(b);
        REQUIRE(rig->client->sendPairKnock(AccessLevel::configure));
        rig->step();
        pair(*rig, button::PairGesture::press);
        rig->step();
        CHECK_FALSE(rig->del.granted.has_value());   // a configure ask outranks a control window
        CHECK(rig->hub->presenceWindowOpen());
        CHECK(rig->device.takePresenceApproval() == 0);
    }
}

TEST_CASE("VD-PAIR-10: a ledger with entries but no configure owner (as after a reboot) vouches nobody above watch") {
    auto rig = std::make_unique<Rig>();   // this rig vouches control for anyone unpaired
    std::array<std::byte, limits::instance_id_bytes> inst{};
    inst[0] = std::byte{0x56};
    XorShift32 rng(98);
    std::array<std::byte, limits::token_bytes> tok{};
    REQUIRE(rig->hub->pairing().grant(inst, AccessLevel::watch, pairing_modes::push_to_pair, rng, tok) != nullptr);
    REQUIRE(rig->client->connect());
    rig->step(200);
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(1, IntentValue::ofBool(true))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::NOT_CONTROLLER);
}

TEST_CASE("VD-PAIR-11: a pending update request owns the double press; the configure request waits for the next") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    REQUIRE(rig->client->sendPairKnock(AccessLevel::configure));
    rig->step();
    REQUIRE(rig->device.pairLook() == PairLook::request);
    g_otaPending = true;
    pair(*rig, button::PairGesture::double_press);
    rig->step();
    CHECK_FALSE(g_otaPending);                           // the update took it
    CHECK_FALSE(rig->del.granted.has_value());           // and nothing else
    CHECK(rig->device.pairLook() == PairLook::request);  // still asking
    pair(*rig, button::PairGesture::double_press);       // no update pending now
    rig->step();
    REQUIRE(rig->del.granted.has_value());
    CHECK(*rig->del.granted == AccessLevel::configure);
}

TEST_CASE("VD-PAIR-12: a pending update request leaves the press and the hold their meanings") {
    auto rig = std::make_unique<Rig>();
    seedConfigureOwner(*rig);
    g_otaPending = true;
    pair(*rig, button::PairGesture::press);
    CHECK(rig->hub->presenceWindowOpen());
    CHECK(g_otaPending);
    pair(*rig, button::PairGesture::hold);
    CHECK(rig->device.pairLook() == PairLook::demoted);
    CHECK(g_otaPending);
}

// ---- RFC-116: the saved-network list over a network binding --------------------

namespace {

bool bytesContain(const std::vector<std::byte>& hay, std::string_view needle) {
    if (needle.empty() || hay.size() < needle.size()) return false;
    for (size_t i = 0; i + needle.size() <= hay.size(); ++i)
        if (std::memcmp(hay.data() + i, needle.data(), needle.size()) == 0) return true;
    return false;
}

bool anyStateContains(const RecordingClient& d, std::string_view needle) {
    for (const auto& [ch, bytes] : d.lastState)
        if (bytesContain(bytes, needle)) return true;
    return false;
}

IntentValueMap netOp(uint8_t op) {
    IntentValueMap m{};
    m.count = 1;
    m.fields[0] = IntentValueField{1, IntentValue::ofU64(op)};
    return m;
}

IntentValueMap netSave(std::string_view ssid, std::string_view pass) {
    return plus(plus(netOp(net::draft::op_net_save), 2, IntentValue::ofTstr(ssid)), 3, IntentValue::ofTstr(pass));
}

std::string ssidAt(const RecordingClient& d, size_t i) {
    const auto& b = d.lastState.at(net::draft::ch_network_names);
    const char* p = reinterpret_cast<const char*>(b.data()) + 3 + i * 32;
    return std::string(p, strnlen(p, 32));
}

}  // namespace

TEST_CASE("VD-NET-1: below configure the list is occupancy only: no SSID, no passphrase, names refused") {
    for (const AccessLevel role : {AccessLevel::watch, AccessLevel::control}) {
        auto rig = std::make_unique<Rig>(role, 0.0f, false, true);
        rig->step(1200);
        REQUIRE(rig->del.lastState.count(net::draft::ch_networks) == 1);
        const auto& s = rig->del.lastState.at(net::draft::ch_networks);
        REQUIRE(s.size() == NetworkDesk::kNetworksBytes);
        CHECK(uint8_t(s[2]) == 4);     // capacity
        CHECK(uint8_t(s[3]) == 1);     // count
        CHECK(uint8_t(s[4]) == 0);     // live: slot 0
        CHECK(int8_t(s[5]) == -51);
        CHECK(rig->del.lastState.count(net::draft::ch_network_names) == 0);
        CHECK_FALSE(anyStateContains(rig->del, kHomeSsid));
        CHECK_FALSE(anyStateContains(rig->del, kHomePass));

        rig->del.nacks.clear();
        REQUIRE(rig->client->sendIntent(channels::provisioning, netSave("Cafe", "cafe passphrase")).has_value());
        rig->step();
        REQUIRE(rig->del.nacks.size() == 1);
        CHECK(rig->del.nacks[0].code == NackCode::ACCESS_DENIED);
        CHECK(rig->nets->list().count() == 1);
    }
}

TEST_CASE("VD-NET-2: at configure the names show, passphrases never; save, move, and the live refusal") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure, 0.0f, false, true);
    rig->step(1200);
    REQUIRE(rig->del.lastState.count(net::draft::ch_network_names) == 1);
    CHECK(rig->del.lastState.at(net::draft::ch_network_names).size() == NetworkDesk::kNamesBytes);
    CHECK(ssidAt(rig->del, 0) == kHomeSsid);

    REQUIRE(rig->client->sendIntent(channels::provisioning, netSave("Cafe", "cafe passphrase")).has_value());
    rig->step(1200);
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(echoed(rig->del.lastEcho, 2) != nullptr);
    CHECK(echoed(rig->del.lastEcho, 2)->kind == IntentValue::Kind::Tstr);   // RFC-116 item 7: not secret
    CHECK(echoed(rig->del.lastEcho, 3)->kind == IntentValue::Kind::Bool);
    CHECK(echoed(rig->del.lastEcho, 6)->u64_val == 1);
    const uint64_t gen = echoed(rig->del.lastEcho, 8)->u64_val;
    CHECK(uint8_t(rig->del.lastState.at(net::draft::ch_networks)[3]) == 2);
    CHECK(ssidAt(rig->del, 1) == "Cafe");
    // Names and occupancy carry the same generation (RFC-116 S9).
    const auto& names = rig->del.lastState.at(net::draft::ch_network_names);
    const auto& occ = rig->del.lastState.at(net::draft::ch_networks);
    CHECK(names[0] == occ[0]);
    CHECK(names[1] == occ[1]);
    CHECK(rig->netStation.persists == 1);

    // Move Cafe first: the live entry follows its id to slot 1.
    REQUIRE(rig->client->sendIntent(channels::provisioning,
                                    plus(plus(netOp(net::draft::op_net_move), 6, IntentValue::ofU64(1)), 7,
                                         IntentValue::ofU64(0))).has_value());
    rig->step(1200);
    REQUIRE(rig->del.nacks.empty());
    CHECK(ssidAt(rig->del, 0) == "Cafe");
    CHECK(uint8_t(rig->del.lastState.at(net::draft::ch_networks)[4]) == 1);

    // The network the hub rides cannot go, nor be overwritten, over WS.
    REQUIRE(rig->client->sendIntent(channels::provisioning,
                                    plus(netOp(net::draft::op_net_remove), 6, IntentValue::ofU64(1))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(uint16_t(rig->del.nacks[0].code) == net::draft::nack_live_network);
    CHECK(rig->del.nacks[0].detail == "in use: the hub is joined through it");
    REQUIRE(rig->client->sendIntent(channels::provisioning, netSave(kHomeSsid, "a new passphrase")).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 2);
    CHECK(uint16_t(rig->del.nacks[1].code) == net::draft::nack_live_network);
    CHECK(rig->nets->list().passphrase(1) == kHomePass);

    // wifi_join never rides the network it would change.
    REQUIRE(rig->client->sendIntent(channels::provisioning,
                                    plus(plus(netOp(provisioning_ops::wifi_join), 2, IntentValue::ofTstr("X")), 3,
                                         IntentValue::ofTstr("passphrase x"))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 3);
    CHECK(rig->del.nacks[2].code == NackCode::ACCESS_DENIED);

    // A stale generation is a conflict; the current one removes Cafe.
    REQUIRE(rig->client->sendIntent(channels::provisioning,
                                    plus(plus(netOp(net::draft::op_net_remove), 6, IntentValue::ofU64(0)), 8,
                                         IntentValue::ofU64(gen))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 4);
    CHECK(rig->del.nacks[3].code == NackCode::CONFLICT);
    REQUIRE(rig->client->sendIntent(channels::provisioning,
                                    plus(plus(netOp(net::draft::op_net_remove), 6, IntentValue::ofU64(0)), 8,
                                         IntentValue::ofU64(rig->nets->generation()))).has_value());
    rig->step(1200);
    CHECK(rig->del.nacks.size() == 4);
    CHECK(rig->nets->list().count() == 1);
    CHECK(ssidAt(rig->del, 0) == kHomeSsid);
    CHECK(ssidAt(rig->del, 1).empty());

    CHECK_FALSE(anyStateContains(rig->del, kHomePass));
    CHECK_FALSE(anyStateContains(rig->del, "cafe passphrase"));
}

namespace {
std::string visibleSsid(const RecordingClient& d, uint16_t ch, size_t i) {
    const auto& b = d.lastState.at(ch);
    const char* p = reinterpret_cast<const char*>(b.data()) + 11 + i * 32;
    return std::string(p, strnlen(p, 32));
}
}  // namespace

TEST_CASE("VD-NET-4: scan at configure: ECHO at once, the rows on 0x0019 strongest first, the saved bit") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure, 0.0f, false, true);
    REQUIRE(rig->client->subscribe(net::draft::ch_visible_networks_more, 0.0f, Priority::background));
    rig->step(1200);
    REQUIRE(rig->del.lastState.count(net::draft::ch_visible_networks) == 1);
    CHECK(rig->del.lastState.at(net::draft::ch_visible_networks).size() == NetworkDesk::kVisibleBytes);
    CHECK(uint8_t(rig->del.lastState.at(net::draft::ch_visible_networks)[0]) == 0);   // no scan yet
    CHECK(rig->netStation.scans == 0);   // joined: no scan unless asked

    REQUIRE(rig->client->sendIntent(channels::provisioning, netOp(net::draft::op_net_scan)).has_value());
    rig->step(1200);
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(echoed(rig->del.lastEcho, 1) != nullptr);
    CHECK(echoed(rig->del.lastEcho, 1)->u64_val == net::draft::op_net_scan);
    CHECK(rig->netStation.scans == 1);
    const auto& a = rig->del.lastState.at(net::draft::ch_visible_networks);
    CHECK(uint8_t(a[0]) == 1);   // scan
    CHECK(uint8_t(a[1]) == 3);   // HomeNet once, at its strongest
    CHECK(uint8_t(a[2]) == 0b101);   // Cafe (row 1) is open
    CHECK(uint8_t(a[3]) == 0b001);   // HomeNet is saved
    CHECK(visibleSsid(rig->del, net::draft::ch_visible_networks, 0) == "HomeNet");
    CHECK(visibleSsid(rig->del, net::draft::ch_visible_networks, 1) == "Cafe");
    CHECK(int8_t(a[4]) == -51);
    const auto& b = rig->del.lastState.at(net::draft::ch_visible_networks_more);
    CHECK(uint8_t(b[0]) == 1);
    CHECK(uint8_t(b[1]) == 0);

    // Saving a heard network moves its saved bit with no new scan.
    REQUIRE(rig->client->sendIntent(channels::provisioning, netSave("Office", "office passphrase")).has_value());
    rig->step(1200);
    CHECK(uint8_t(rig->del.lastState.at(net::draft::ch_visible_networks)[3]) == 0b101);
    CHECK(rig->netStation.scans == 1);
    CHECK_FALSE(anyStateContains(rig->del, "office passphrase"));
}

TEST_CASE("VD-NET-5: joined, scans coalesce inside 10 s and never run during motion; below configure, none") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure, 0.0f, false, true);
    rig->step(200);
    REQUIRE(rig->client->sendIntent(channels::provisioning, netOp(net::draft::op_net_scan)).has_value());
    rig->step(100);
    CHECK(rig->netStation.scans == 1);
    for (int i = 0; i < 3; ++i) {
        REQUIRE(rig->client->sendIntent(channels::provisioning, netOp(net::draft::op_net_scan)).has_value());
        rig->step(100);
    }
    CHECK(rig->del.nacks.empty());
    CHECK(rig->netStation.scans == 1);   // asked three times inside the interval: one scan, later
    rig->step(int(net::draft::scan_min_interval_ms));
    CHECK(rig->netStation.scans == 2);
    rig->step(int(net::draft::scan_min_interval_ms) + 100);
    CHECK(rig->netStation.scans == 2);   // nobody asked again

    g_census.busy = true;
    rig->step(1);   // the desk reads motion once a tick
    REQUIRE(rig->client->sendIntent(channels::provisioning, netOp(net::draft::op_net_scan)).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INTERLOCK);
    CHECK(rig->del.nacks[0].detail == "motion running");
    CHECK(rig->netStation.scans == 2);
    g_census.busy = false;

    for (const AccessLevel role : {AccessLevel::watch, AccessLevel::control}) {
        auto low = std::make_unique<Rig>(role, 0.0f, false, true);
        low->step(200);
        REQUIRE(low->client->sendIntent(channels::provisioning, netOp(net::draft::op_net_scan)).has_value());
        low->step();
        REQUIRE(low->del.nacks.size() == 1);
        CHECK(low->del.nacks[0].code == NackCode::ACCESS_DENIED);
        CHECK(low->netStation.scans == 0);
        CHECK(low->del.lastState.count(net::draft::ch_visible_networks) == 0);
    }
}

TEST_CASE("VD-NET-6: not joined (config mode, offline): a scan runs at once, and the hub rescans every 3 s") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure, 0.0f, false, true);
    rig->netStation.live = 0;
    rig->step(50);
    CHECK(rig->netStation.scans == 1);   // a configure session is present: no ask needed
    rig->step(int(net::draft::scan_offline_period_ms));
    CHECK(rig->netStation.scans == 2);
    REQUIRE(rig->client->sendIntent(channels::provisioning, netOp(net::draft::op_net_scan)).has_value());
    rig->step(20);
    CHECK(rig->netStation.scans == 3);   // no 10 s interval offline
    rig->step(1100);                      // 0x0019 publishes at most once a second
    CHECK(uint8_t(rig->del.lastState.at(net::draft::ch_visible_networks)[0]) == 3);
    g_census.busy = true;
    rig->step(int(net::draft::scan_offline_period_ms) * 2);
    CHECK(rig->netStation.scans == 3);   // never during motion
    g_census.busy = false;
    rig->step(20);
    CHECK(rig->netStation.scans == 4);

    auto watch = std::make_unique<Rig>(AccessLevel::watch, 0.0f, false, true);
    watch->netStation.live = 0;
    watch->step(int(net::draft::scan_offline_period_ms) * 2);
    CHECK(watch->netStation.scans == 0);   // no configure session: no self-driven scans
}

TEST_CASE("VD-NET-7: a scan running when motion starts stops within one channel dwell and runs again after") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure, 0.0f, false, true);
    rig->step(200);
    rig->netStation.hold = true;
    REQUIRE(rig->client->sendIntent(channels::provisioning, netOp(net::draft::op_net_scan)).has_value());
    rig->step(20);
    REQUIRE(rig->netStation.scans == 1);
    REQUIRE(rig->netStation.scanning);
    g_census.busy = true;
    // One hub tick (1 ms here, 5 ms on the board) is far inside a 60 ms dwell.
    rig->step(1);
    CHECK(rig->netStation.stops == 1);
    CHECK_FALSE(rig->netStation.scanning);
    rig->netStation.hold = false;
    rig->step(int(net::draft::scan_min_interval_ms) + 100);
    CHECK(rig->netStation.scans == 1);   // not resumed while motion runs
    CHECK(rig->netStation.stops == 1);
    g_census.busy = false;
    rig->step(20);
    CHECK(rig->netStation.scans == 2);   // the ask stands: it runs once motion stops
    rig->step(1100);
    CHECK(uint8_t(rig->del.lastState.at(net::draft::ch_visible_networks)[0]) == 1);
}

TEST_CASE("VD-NET-8: associating counts as joined; a parked configure session is not present and drops the ask") {
    auto rig = std::make_unique<Rig>(AccessLevel::configure, 0.0f, false, true);
    rig->netStation.live = 0;
    rig->netStation.associating = true;
    rig->step(int(net::draft::scan_offline_period_ms) * 2);
    CHECK(rig->netStation.scans == 0);   // joined policy: no self-driven scans
    rig->netStation.associating = false;
    rig->step(20);
    CHECK(rig->netStation.scans == 1);   // not joined: at once

    // Joined: an ask waits out motion; the session parks before it runs.
    rig->netStation.live = 1;
    REQUIRE(rig->client->sendIntent(channels::provisioning, netOp(net::draft::op_net_scan)).has_value());
    rig->step(20);
    REQUIRE(rig->nets->scanPending());
    bool parked = false;
    for (int i = 0; i < 60000 && !parked; ++i) {   // the client falls silent
        g_clock.advanceUs(10'000);
        rig->hub->update(g_clock.nowUs());
        rig->nets->tick(uint32_t(g_clock.nowUs() / 1000), true);   // motion holds the ask
        const HubSession* s = rig->hub->sessionBySlot(0);
        parked = s != nullptr && s->state == HubSessionState::STALE;
    }
    REQUIRE(parked);
    CHECK_FALSE(rig->nets->scanPending());   // nobody waits for it: dropped
    rig->netStation.live = 0;
    for (int i = 0; i < 1000; ++i) {
        g_clock.advanceUs(10'000);
        rig->nets->tick(uint32_t(g_clock.nowUs() / 1000), false);
    }
    CHECK(rig->netStation.scans == 1);   // offline, but no session present: no self-driven scans
}

TEST_CASE("VD-NET-3: the join rule and first-boot migration") {
    std::array<uint8_t, 4> order{};
    std::array<int8_t, 4> rssi{};
    net::SavedNetworks l;
    for (const char* s : {"one", "two", "hidden"}) l.save(std::nullopt, s, "passphrase");
    const std::vector<net::Sighting> heard{{"one", -62}, {"two", -60}, {"other", -30}};
    REQUIRE(net::joinOrder(l, heard, order, rssi) == 3);
    CHECK(order[0] == 0);   // within 3 dB of two: the earlier entry
    CHECK(order[1] == 1);
    CHECK(order[2] == 2);   // not heard: after the visible ones
    const std::vector<net::Sighting> clear{{"one", -70}, {"two", -60}};
    net::joinOrder(l, clear, order, rssi);
    CHECK(order[0] == 1);

    const net::SavedNetworks fromNvs =
        firstBootList(std::pair{std::string_view("prov"), std::string_view("prov passphrase")}, "compiled",
                      "compiled passphrase");
    REQUIRE(fromNvs.count() == 1);
    CHECK(fromNvs.ssid(0) == "prov");
    const net::SavedNetworks fromSecrets = firstBootList(std::nullopt, "compiled", "compiled passphrase");
    REQUIRE(fromSecrets.count() == 1);
    CHECK(fromSecrets.ssid(0) == "compiled");
    CHECK(fromSecrets.passphrase(0) == "compiled passphrase");
    CHECK(firstBootList(std::nullopt, "your-ssid", "short").count() == 0);
}

// ---- write tiers (SPEC 12.2, operator ruling 2026-10-15) ----------------------

namespace {

constexpr AccessLevel kWatch = AccessLevel::watch;
constexpr AccessLevel kControl = AccessLevel::control;
constexpr AccessLevel kConfigure = AccessLevel::configure;

// The ruling as a table: every writable entry (INTENT, c2h STREAM) and the tier
// each of its keys and ops needs. key 0 is the entry's own row; option -1 is
// every option of the key. A writable entry missing here fails VD-TIER-MATRIX.
struct TierRow {
    uint16_t channel;
    uint8_t key;
    int option;
    AccessLevel need;
};
constexpr TierRow kTierRows[] = {
    // Any tier: Halt and pause. Every other safety op drives.
    {channels::safety_intents, 0, -1, kControl},
    {channels::safety_intents, 1, safety_ops::pause, kWatch},
    {channels::safety_intents, 1, safety_ops::estop, kWatch},
    // Configure: machine settings and limits, trust, network.
    {channels::session_admin, 0, -1, kConfigure},
    {channels::provisioning, 0, -1, kConfigure},
    {ch::config_set, 0, -1, kConfigure},
    {ch::modes_set, 0, -1, kConfigure},
    {ch::kinetic_set, 0, -1, kConfigure},
    {ch::drive_set, 0, -1, kConfigure},
    {ch::machine_admin, 0, -1, kConfigure},
    {ch::machine_admin, 1, 1, kControl},   // clear_fault
    {ch::home, 1, 0, kConfigure},          // the placeholder, strict side
    {ch::home, 1, 2, kConfigure},          // force_home asserts the stroke
    {ch::home, 2, -1, kConfigure},         // that stroke
    // Control: driving inside those limits.
    {ch::config_set, 1, -1, kControl},     // window_min (operator 2026-10-17)
    {ch::config_set, 2, -1, kControl},     // window_max
    {ch::modes_set, 8, -1, kControl},      // flipped
    {channels::settings_trial, 0, -1, kControl},
    {ch::home, 0, -1, kControl},
    {ch::move, 0, -1, kControl},
    {ch::osc_set, 0, -1, kControl},
    {ch::pattern_cmd, 0, -1, kControl},
    {ch::pattern_advanced_cmd, 0, -1, kControl},
    {ch::pattern_presets_cmd, 0, -1, kControl},
    {ch::motion_input, 0, -1, kControl},
    {ch::motion_segment, 0, -1, kControl},
    {ch::osc_drive, 0, -1, kControl},
};

std::optional<AccessLevel> expectedTier(uint16_t channel, uint8_t key, int option) {
    std::optional<AccessLevel> entry, field, exact;
    for (const TierRow& r : kTierRows) {
        if (r.channel != channel) continue;
        if (r.key == 0) entry = r.need;
        else if (r.key == key && r.option == -1) field = r.need;
        else if (r.key == key && r.option == option) exact = r.need;
    }
    return exact ? exact : field ? field : entry;
}

AccessLevel raised(AccessLevel a, AccessLevel b) { return uint8_t(b) > uint8_t(a) ? b : a; }

// One probe per key, and per option of a key that gates by option, with the
// tier the catalog declares for it: the entry's floor raised by the field's
// access and the option's (Hub::requiredAccessFor). A STREAM is key 0.
struct TierProbe {
    uint16_t channel;
    uint8_t key;
    int option;
    AccessLevel declared;
    CborFieldType type;
    float min;
};

std::vector<TierProbe> writableProbes(const Catalog32& c) {
    std::vector<TierProbe> out;
    for (uint16_t i = 0; i < c.count; ++i) {
        const CatalogEntry& e = c.entries[i];
        if (e.cls != ChannelClass::INTENT && !(e.cls == ChannelClass::STREAM && e.dir == Direction::c2h)) continue;
        if (!e.usesSchema()) {
            out.push_back({e.id, 0, -1, e.access, CborFieldType::uint_t, 0.0f});
            continue;
        }
        for (const SchemaField& f : c.schemaFields(e)) {
            const AccessLevel base = f.hasAccess ? raised(e.access, f.access) : e.access;
            const auto oa = c.optionAccess(f);
            if (oa.empty()) {
                out.push_back({e.id, f.key, -1, base, f.type, f.hasMin ? f.min : 0.0f});
                continue;
            }
            for (size_t o = 0; o < oa.size(); ++o)
                out.push_back({e.id, f.key, int(o), raised(base, oa[o]), f.type, 0.0f});
        }
    }
    return out;
}

const std::array<std::byte, 8> kProbeBytes{};

IntentValue probeValue(const TierProbe& p) {
    switch (p.type) {
        case CborFieldType::uint_t:
            return IntentValue::ofU64(p.option >= 0 ? uint64_t(p.option) : uint64_t(std::max(0.0f, p.min)));
        case CborFieldType::int_t: return IntentValue::ofI64(int64_t(p.min));
        case CborFieldType::f32_t: return IntentValue::ofF32(p.min);
        case CborFieldType::bool_t: return IntentValue::ofBool(false);
        case CborFieldType::tstr_t: return IntentValue::ofTstr("probe");
        case CborFieldType::bstr_t: return IntentValue::ofBstr(kProbeBytes);
    }
    return IntentValue::ofU64(0);
}

// The one answer to an intent, sent now: NACK code and detail, or an ECHO.
struct Answer {
    std::optional<NackCode> nack;
    std::string detail;
    bool echoed = false;
};

Answer answerTo(Rig& rig, uint16_t channel, const IntentValueMap& m) {
    const size_t nacks = rig.del.nacks.size();
    const int echoes = rig.del.echoes;
    REQUIRE(rig.client->sendIntent(channel, m).has_value());
    rig.step(40);
    Answer a;
    if (rig.del.nacks.size() > nacks) {
        a.nack = rig.del.nacks.back().code;
        a.detail = rig.del.nacks.back().detail;
    }
    a.echoed = rig.del.echoes > echoes;
    return a;
}

IntentValueMap forceHome(float stroke) {
    IntentValueMap m{};
    m.count = 2;
    m.fields[0] = IntentValueField{1, IntentValue::ofU64(2)};
    m.fields[1] = IntentValueField{2, IntentValue::ofF32(stroke)};
    return m;
}

// The settings a control session asks to change and must not: the rail,
// ceilings, Kinetic tuning, homing, and force_home's asserted stroke.
struct SettingWrite {
    uint16_t channel;
    IntentValueMap value;
};
std::vector<SettingWrite> settingWrites() {
    return {{ch::config_set, oneKey(5, IntentValue::ofF32(100.0f))},   // input_speed ceiling
            {ch::config_set, oneKey(8, IntentValue::ofF32(400.0f))},   // max_rail
            {ch::kinetic_set, oneKey(2, IntentValue::ofF32(5.0f))},    // vmax override
            {ch::kinetic_set, oneKey(4, IntentValue::ofF32(0.5f))},    // smoothness
            {ch::modes_set, oneKey(9, IntentValue::ofF32(20.0f))},     // home_speed
            {ch::home, forceHome(250.0f)}};
}

}  // namespace

TEST_CASE("VD-TIER-MATRIX: every writable entry is classified, and the catalog declares the ruling's tier") {
    auto cat = std::make_unique<Catalog32>();
    DeviceFeatures all{};
    all.has_current_sensor = all.has_power_monitor = all.has_motion = all.has_drive = all.has_pattern = true;
    REQUIRE(buildValenceCatalog(*cat, all));
    const auto probes = writableProbes(*cat);
    REQUIRE(probes.size() > 50);
    for (const TierProbe& p : probes) {
        CAPTURE(p.channel);
        CAPTURE(int(p.key));
        CAPTURE(p.option);
        const auto want = expectedTier(p.channel, p.key, p.option);
        REQUIRE_MESSAGE(want.has_value(), "unclassified writable entry: give it a row in kTierRows");
        CHECK(int(p.declared) == int(*want));
    }
    for (const TierRow& r : kTierRows) {
        CAPTURE(r.channel);
        CHECK(cat->find(r.channel) != nullptr);   // no stale row
    }
}

TEST_CASE("VD-TIER-LIVE: the hub refuses every writable key below its tier, at watch, control and configure") {
    for (const AccessLevel role : {kWatch, kControl, kConfigure}) {
        auto rig = std::make_unique<Rig>(role);
        for (const TierProbe& p : writableProbes(rig->catalog)) {
            if (p.key == 0) continue;   // a STREAM: its tier gates the PUBLISH grant (SPEC 6.2)
            CAPTURE(int(role));
            CAPTURE(p.channel);
            CAPTURE(int(p.key));
            CAPTURE(p.option);
            const Answer a = answerTo(*rig, p.channel, oneKey(p.key, probeValue(p)));
            const bool below = uint8_t(role) < uint8_t(p.declared);
            const bool denied = a.nack == NackCode::ACCESS_DENIED || a.nack == NackCode::NOT_CONTROLLER;
            CHECK(denied == below);
            if (below && denied) {
                CHECK(*a.nack == (p.declared == kControl ? NackCode::NOT_CONTROLLER : NackCode::ACCESS_DENIED));
                CHECK(a.detail == (p.declared == kControl ? "needs control" : "needs configure"));
            }
        }
    }
}

TEST_CASE("VD-TIER-1: control jogs, runs patterns and the oscillator; settings and limits need configure") {
    auto rig = std::make_unique<Rig>(kControl);
    rig->step(50);
    const auto cfg = rig->del.lastState.at(ch::machine_config);
    const auto limits = rig->del.lastState.at(ch::kinetic_limits);
    const auto planner = rig->del.lastState.at(ch::kinetic_planner);
    const auto modes = rig->del.lastState.at(ch::machine_modes);
    const uint16_t gen = rig->hub->cfgGen();
    for (const SettingWrite& w : settingWrites()) {
        CAPTURE(w.channel);
        const Answer a = answerTo(*rig, w.channel, w.value);
        CHECK(a.nack == NackCode::ACCESS_DENIED);
        CHECK(a.detail == "needs configure");
        CHECK_FALSE(a.echoed);
    }
    rig->step(50);
    CHECK(rig->del.lastState.at(ch::machine_config) == cfg);
    CHECK(rig->del.lastState.at(ch::kinetic_limits) == limits);
    CHECK(rig->del.lastState.at(ch::kinetic_planner) == planner);
    CHECK(rig->del.lastState.at(ch::machine_modes) == modes);
    CHECK(rig->hub->cfgGen() == gen);
    CHECK(g_forceHomes == 0);

    // Driving: a jog, the pattern's share-of-window knobs, the oscillator.
    CHECK(answerTo(*rig, ch::move, moveTo(100.0f)).echoed);
    CHECK(answerTo(*rig, ch::pattern_cmd, oneKey(3, IntentValue::ofF32(40.0f))).echoed);   // speed
    CHECK(answerTo(*rig, ch::pattern_cmd, oneKey(5, IntentValue::ofF32(60.0f))).echoed);   // stroke
    CHECK(answerTo(*rig, ch::osc_set, oneKey(2, IntentValue::ofF32(2.0f))).echoed);        // frequency
    // Streams publish at control: the hub's PUBLISH grant reads this floor.
    for (const uint16_t s : {ch::motion_input, ch::motion_segment, ch::osc_drive}) {
        CAPTURE(s);
        CHECK(rig->catalog.find(s)->access == kControl);
    }

    auto run = std::make_unique<Rig>(kControl);
    const Answer started = answerTo(*run, ch::pattern_cmd, oneKey(1, IntentValue::ofBool(true)));
    CHECK(started.echoed);
    CHECK_FALSE(started.nack.has_value());
    CHECK(answerTo(*run, ch::pattern_cmd, oneKey(1, IntentValue::ofBool(false))).echoed);
}

TEST_CASE("VD-TIER-1b: control moves the stroke window and flips; a write carrying a limit beside it is refused whole") {
    auto rig = std::make_unique<Rig>(kControl);
    g_census.rail_mm = rig->device.config().max_rail;
    Answer a = answerTo(*rig, ch::config_set, oneKey(1, IntentValue::ofF32(40.0f)));
    CHECK_FALSE(a.nack.has_value());
    CHECK(a.echoed);
    a = answerTo(*rig, ch::config_set, oneKey(2, IntentValue::ofF32(300.0f)));
    CHECK_FALSE(a.nack.has_value());
    CHECK(a.echoed);
    CHECK(rig->device.config().window_min == doctest::Approx(40.0f));
    CHECK(rig->device.config().window_max == doctest::Approx(300.0f));
    a = answerTo(*rig, ch::modes_set, oneKey(8, IntentValue::ofU64(1)));
    CHECK_FALSE(a.nack.has_value());
    CHECK(a.echoed);
    CHECK(g_flipped);

    IntentValueMap both{};
    both.count = 2;
    both.fields[0] = IntentValueField{2, IntentValue::ofF32(250.0f)};
    both.fields[1] = IntentValueField{8, IntentValue::ofF32(400.0f)};   // max_rail
    const uint16_t gen = rig->hub->cfgGen();
    a = answerTo(*rig, ch::config_set, both);
    CHECK(a.nack == NackCode::ACCESS_DENIED);
    CHECK(a.detail == "needs configure");
    CHECK(rig->hub->cfgGen() == gen);
}

TEST_CASE("VD-TIER-2: configure changes every setting control was refused") {
    auto rig = std::make_unique<Rig>(kConfigure);
    for (const SettingWrite& w : settingWrites()) {
        CAPTURE(w.channel);
        const Answer a = answerTo(*rig, w.channel, w.value);
        CHECK_FALSE(a.nack.has_value());
        CHECK(a.echoed);
    }
    CHECK(g_forceHomes == 1);
}

TEST_CASE("VD-TIER-3: watch halts and pauses; it neither drives nor configures") {
    auto rig = std::make_unique<Rig>(kWatch, 0.0f, false, true);
    Answer a = answerTo(*rig, ch::move, moveTo(100.0f));
    CHECK(a.nack == NackCode::NOT_CONTROLLER);
    CHECK(a.detail == "needs control");
    for (const SettingWrite& w : {SettingWrite{ch::config_set, oneKey(2, IntentValue::ofF32(300.0f))},
                                  SettingWrite{ch::modes_set, oneKey(8, IntentValue::ofU64(1))}}) {
        CAPTURE(w.channel);
        a = answerTo(*rig, w.channel, w.value);   // the window and the flip
        CHECK(a.nack == NackCode::NOT_CONTROLLER);
        CHECK(a.detail == "needs control");
    }
    a = answerTo(*rig, ch::config_set, oneKey(8, IntentValue::ofF32(400.0f)));   // max_rail
    CHECK(a.nack == NackCode::ACCESS_DENIED);
    CHECK(a.detail == "needs configure");
    a = answerTo(*rig, channels::safety_intents, safetyOp(safety_ops::pause));
    CHECK_FALSE(a.nack.has_value());
    CHECK(rig->hub->pauseLatched());
    a = answerTo(*rig, channels::safety_intents, safetyOp(safety_ops::estop));
    CHECK_FALSE(a.nack.has_value());
    CHECK(rig->hub->estopLatched());
}

TEST_CASE("VD-TIER-4: a control session asks configure, a PAIR double press grants it in place, its setting lands") {
    auto rig = std::make_unique<Rig>(kControl);
    REQUIRE(answerTo(*rig, ch::config_set, oneKey(3, IntentValue::ofF32(60.0f))).nack == NackCode::ACCESS_DENIED);
    REQUIRE(rig->client->sendPairKnock(AccessLevel::configure));
    rig->step();
    CHECK_FALSE(rig->del.granted.has_value());
    CHECK(rig->device.pairLook() == PairLook::request);
    pair(*rig, button::PairGesture::double_press);
    rig->step();
    REQUIRE(rig->del.granted.has_value());
    CHECK(*rig->del.granted == kConfigure);
    CHECK(rig->hub->pairing().hasConfigureToken());
    const Answer a = answerTo(*rig, ch::config_set, oneKey(3, IntentValue::ofF32(60.0f)));
    CHECK_FALSE(a.nack.has_value());
    CHECK(a.echoed);
    CHECK(rig->device.config().jog_speed == doctest::Approx(60.0f));
}

// ---- the provisioning catalog (SPEC 13.4.1, RFC-121) ---------------------------------

TEST_CASE("VD-PROV-CAT: config mode's catalog is exactly the members, byte for byte the full catalog's, with its own etag") {
    auto full = std::make_unique<Catalog32>();
    auto prov = std::make_unique<Catalog32>();
    REQUIRE(buildValenceCatalog(*full, boardFeatures()));
    REQUIRE(full->count > 0);
    CHECK(full->entries[0].id == channels::catalog);   // declared, first by id
    REQUIRE(buildProvisioningCatalog(*prov, *full));

    const std::vector<uint16_t> want = {channels::catalog,          channels::safety,
                                        channels::safety_intents,   channels::safety_events,
                                        channels::provisioning,     net::draft::ch_networks,
                                        net::draft::ch_network_names, net::draft::ch_visible_networks,
                                        net::draft::ch_visible_networks_more};
    REQUIRE(prov->count == want.size());
    std::vector<std::byte> a(limits::catalog_max_entry_bytes), b(limits::catalog_max_entry_bytes);
    for (size_t i = 0; i < want.size(); ++i) {
        CAPTURE(want[i]);
        const CatalogEntry& p = prov->entries[i];
        CHECK(p.id == want[i]);
        const CatalogEntry* f = full->find(p.id);
        REQUIRE(f != nullptr);
        CborWriter wf(a), wp(b);
        detail::encodeEntry(wf, *full, *f);
        detail::encodeEntry(wp, *prov, p);
        REQUIRE(wf.size() != 0);
        CHECK(wp.size() == wf.size());
        CHECK(std::equal(a.begin(), a.begin() + wf.size(), b.begin()));
    }
    // No motion source, so no client starts motion in config mode.
    for (uint16_t i = 0; i < prov->count; ++i) CHECK(prov->entries[i].cls != ChannelClass::STREAM);

    std::vector<std::byte> scratch(Hub::catalogScratchCapacity());
    const auto fullTag = catalogEtag(*full, scratch);
    const auto provTag = catalogEtag(*prov, scratch);
    CHECK(fullTag != provTag);
    const size_t provBytes = encodeCatalog(*prov, scratch);
    CHECK(provBytes > 0);
    CHECK(provBytes < 4500);   // SPEC 13.4.1: about 2.8 KB, a short BLE download
    MESSAGE("provisioning catalog: " << provBytes << " B");
}

// Heap, not stack: the hub and the client are hundreds of KB together.
struct ProvRig {
    Catalog32 full{};
    Catalog32 prov{};
    XorShift32 hubRng{17};
    ValenceDevice device{};
    std::optional<Hub> hub{};
    std::optional<InProcessLink> link{};
    XorShift32 clientRng{18};
    RecordingClient del{};
    std::optional<Client> client{};
};

TEST_CASE("VD-PROV-CAT-2: a hub over the provisioning catalog serves a session that downloads and adopts it") {
    auto r = std::make_unique<ProvRig>();
    REQUIRE(buildValenceCatalog(r->full, boardFeatures()));
    REQUIRE(buildProvisioningCatalog(r->prov, r->full));
    r->hub.emplace(r->prov, g_clock, r->hubRng, r->device);
    r->device.attach(*r->hub, r->prov);
    CHECK(r->hub->catalogEncodedBytes() > 0);
    r->link.emplace(g_clock, r->hubRng);
    REQUIRE(r->hub->attachTransport(r->link->endpointA()));
    ClientIdentity id;
    id.instance_id.fill(std::byte{0x33});
    id.hasToken = false;
    id.client_kind = "native-test";
    id.client_name = "prov";
    r->client.emplace(id, r->link->endpointB(), g_clock, r->clientRng, r->del);
    REQUIRE(r->client->connect());
    for (int i = 0; i < 2000 && r->client->state() != ClientSessionState::LIVE; ++i) {
        g_clock.advanceUs(5000);
        r->hub->update(g_clock.nowUs());
        r->device.tick(uint32_t(g_clock.nowUs() / 1000));
        r->client->update(g_clock.nowUs());
    }
    CHECK(r->client->state() == ClientSessionState::LIVE);
}

// ---- the config-mode gesture (SPEC 13.4.1, device-defined) --------------------------

TEST_CASE("VD-CONFIG-1: PAIR held 10 s with motion idle and motor power off reboots into config mode") {
    auto rig = std::make_unique<Rig>();
    g_census.motor_on = g_census.power_gate = false;
    pair(*rig, button::PairGesture::config);
    CHECK(rig->device.rebootDue());
    CHECK(rig->device.rebootIntoConfig());
}

TEST_CASE("VD-CONFIG-2: the config hold is refused with motor power on or motion running, and reboots nothing") {
    auto rig = std::make_unique<Rig>();
    g_census.motor_on = g_census.power_gate = true;
    pair(*rig, button::PairGesture::config);
    CHECK_FALSE(rig->device.rebootDue());
    CHECK(rig->device.pairLook() == PairLook::refused);

    g_census.motor_on = g_census.power_gate = false;
    g_census.busy = true;
    pair(*rig, button::PairGesture::config);
    CHECK_FALSE(rig->device.rebootDue());
    CHECK_FALSE(rig->device.rebootIntoConfig());
    g_census.busy = false;
}

// ---- the radios setting (RFC-121, SPEC 13.1) -------------------------------------

TEST_CASE("VD-RADIOS: radios is configure tier, the tail byte of 0x1030, stored, and a config-mode join turns it back on") {
    {
        auto rig = std::make_unique<Rig>();
        REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(11, IntentValue::ofU64(0))).has_value());
        rig->step();
        REQUIRE(rig->del.nacks.size() == 1);
        CHECK(rig->del.nacks[0].code == NackCode::ACCESS_DENIED);
    }
    auto rig = std::make_unique<Rig>(AccessLevel::configure);
    const auto radiosByte = [&] {
        const auto it = rig->del.lastState.find(ch::machine_modes);
        REQUIRE(it != rig->del.lastState.end());
        return it->second.back();
    };
    CHECK(radiosByte() == std::byte{1});
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(11, IntentValue::ofU64(0))).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(rig->del.lastEcho.count == 1);
    CHECK(rig->del.lastEcho.fields[0].key == 11);
    CHECK(rig->del.lastEcho.fields[0].value.u64_val == 0);
    rig->step(200);
    CHECK(radiosByte() == std::byte{0});

    std::array<std::byte, stored::kConfigBlobBytes> blob{};
    REQUIRE(rig->device.encodeConfigBlob(blob, 3) == blob.size());
    CHECK(blob[stored::kConfigV9Bytes] == std::byte{0});

    const uint16_t gen = rig->hub->cfgGen();
    rig->device.enableRadios();
    rig->step(200);
    CHECK(radiosByte() == std::byte{1});
    CHECK(rig->hub->cfgGen() != gen);   // a hub-side change moves cfg_gen
    REQUIRE(rig->device.encodeConfigBlob(blob, 3) == blob.size());
    CHECK(blob[stored::kConfigV9Bytes] == std::byte{1});
}

TEST_CASE("VD-HEADROOM: accessory room never passes catalog_max_entries or catalog_max_bytes (RFC-121)") {
    auto cat = std::make_unique<Catalog32>();
    REQUIRE(buildValenceCatalog(*cat, boardFeatures()));
    const CatalogHeadroom h = catalogHeadroom(*cat, 31000);
    CHECK(size_t(cat->count) + h.entries <= limits::catalog_max_entries);
    CHECK(31000u + h.bytes <= limits::catalog_max_bytes);
    CHECK(catalogHeadroom(*cat, limits::catalog_max_bytes).bytes == 0);
}
