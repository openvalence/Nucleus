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
#define VALENCE_CATALOG_LAYOUT_FIELDS 256
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
#include "valence/core/clock.hpp"
#include "valence/core/rng.hpp"
#include "valence/hub/hub.hpp"
#include "valence/transport/inprocess_binding.hpp"
#include "valence/util/byte_io.hpp"
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
button::Gesture g_pairGesture = button::Gesture::none;
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
void motionSetFlipped(bool) {}
void motionSetJogLimits(float, float) {}
void motionSetInputLimits(float, float, float) {}
void motionSetWindow(float, float, float) {}
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
// The plan strip's answer (MotionArbiter::stillFor()), and the window asked.
bool g_still = true;
uint32_t g_stillWindowUs = 0;
bool motionStillFor(uint32_t window_us) {
    g_stillWindowUs = window_us;
    return g_still;
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

button::Gesture homeButtonTake() { return std::exchange(g_homeGesture, button::Gesture::none); }
button::Gesture pairButtonTake() { return std::exchange(g_pairGesture, button::Gesture::none); }
bool driveAlarmTake() { return std::exchange(g_driveAlarm, false); }

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
    void onEvent(uint16_t channel_id, std::span<const std::byte> payload) override {
        if (channel_id != ch::motion_anomaly) return;
        const auto ev = decodeEvent(payload);
        REQUIRE(ev.isOk());
        anomalyEvents.push_back(ev.value());
    }
};

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

    // motionHz > 0 also subscribes 0x1100 at that rate.
    explicit Rig(AccessLevel role = AccessLevel::control, float motionHz = 0.0f) {
        g_census = MotionCensus{};
        g_census.homed = true;
        g_census.motor_on = true;
        g_census.power_gate = true;
        g_returnAnswer = ReturnStart::queued;
        g_homeGesture = g_pairGesture = button::Gesture::none;
        g_forceHomes = 0;
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
        g_still = true;
        g_stillWindowUs = 0;
        g_anomalies.clear();
        REQUIRE(buildValenceCatalog(catalog, boardFeatures()));
        device.setUnvouchedRole(role);
        device.setSetupWritten(kSetupRequiredMask);
        hub.emplace(catalog, g_clock, hubRng, device);
        device.attach(*hub, catalog);
        link.emplace(g_clock, hubRng);
        // A 13th wish takes WELCOME past the link's default 250 B; a WS frame has room.
        if (motionHz > 0.0f) link->profileA().mtu = 1024;
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
        REQUIRE(client->connect());
        step(200);
        REQUIRE(client->state() == ClientSessionState::LIVE);
    }

    void step(int rounds = 16) {
        for (int i = 0; i < rounds; ++i) {
            g_clock.advanceUs(1000);
            hub->update(g_clock.nowUs());
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
    g_pairGesture = button::Gesture::press;
    rig->step(2);
    CHECK_FALSE(rig->hub->presenceWindowOpen());
}

TEST_CASE("VD-11: a brake that never reaches rest still reboots, after the bound") {
    auto rig = std::make_unique<Rig>();
    g_census.busy = true;
    g_homeGesture = button::Gesture::hold;
    rig->step(1000);
    CHECK_FALSE(rig->device.rebootDue());
    rig->step(1100);
    CHECK(rig->hub->estopLatched());
    CHECK(rig->device.rebootDue());
}

TEST_CASE("VD-12: PAIR press opens the presence window; PAIR hold does nothing") {
    auto rig = std::make_unique<Rig>();
    g_pairGesture = button::Gesture::hold;
    rig->step();
    CHECK_FALSE(rig->hub->presenceWindowOpen());
    g_pairGesture = button::Gesture::press;
    rig->step();
    CHECK(rig->hub->presenceWindowOpen());
    CHECK_FALSE(rig->device.rebootDue());
}

TEST_CASE("VD-13: a planned reboot takes the armed persists at once, then none") {
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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

TEST_CASE("VD-TR-3: keys gated on live state refuse a trial; the flip waits for a window trial") {
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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
    g_anomalies.push_back({3, 10, 0x100001234ull, 0.25f, 0.5f});   // knot_trimmed
    g_anomalies.push_back({6, 11, 0x100001240ull, 0.75f, 1.25f});  // piece_over_ceiling
    g_anomalies.push_back({1, 12, 0x100001250ull, 0.5f, 12.0f});   // settle
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
    CHECK(body[anom_body::target].f32_val == 0.75f);
    CHECK(body[anom_body::detail].f32_val == 1.25f);
    CHECK(body[anom_body::t_us].u64_val == 0x1240u);   // low 32 bits
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

TEST_CASE("VD-MODES-9: home_speed writes on modes-set key 9, clamped, published at 0x1030's tail, stored") {
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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
    auto rig = std::make_unique<Rig>();
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
        REQUIRE(it->second.size() == 57);
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
    m.fields[2] = IntentValueField{3, IntentValue::ofF32(0.05f)};
    m.fields[3] = IntentValueField{4, IntentValue::ofU64(1)};        // square
    m.fields[4] = IntentValueField{5, IntentValue::ofF32(0.333f)};   // two decimals
    m.fields[5] = IntentValueField{6, IntentValue::ofF32(9.0f)};     // past the longest hold
    REQUIRE(rig->client->sendIntent(ch::osc_set, m).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    REQUIRE(rig->del.echoes == 1);
    CHECK(echoed(rig->del.lastEcho, 1)->bool_val);
    CHECK(echoed(rig->del.lastEcho, 2)->f32_val == OSC_MAX_HZ);
    CHECK(echoed(rig->del.lastEcho, 5)->f32_val == doctest::Approx(0.33f));
    CHECK(echoed(rig->del.lastEcho, 6)->f32_val == doctest::Approx(4.0f));
    CHECK(g_osc.enabled);
    CHECK(g_osc.frequency_hz == OSC_MAX_HZ);
    CHECK(g_osc.amplitude == doctest::Approx(0.05f));
    CHECK(g_osc.shape == 1);
    // 0x1140: the applied parameters, then what renders, the census's.
    g_census.osc_active = true;
    g_census.osc_amplitude = 0.04f;
    rig->step(60);
    {
        const auto it = rig->del.lastState.find(ch::oscillator);
        REQUIRE(it != rig->del.lastState.end());
        const std::span<const std::byte> b(it->second);
        CHECK(b[0] == std::byte{1});
        CHECK(getF32(b.subspan(1, 4)) == OSC_MAX_HZ);
        CHECK(getF32(b.subspan(5, 4)) == doctest::Approx(0.05f));
        CHECK(b[9] == std::byte{1});
        CHECK(getF32(b.subspan(10, 4)) == doctest::Approx(0.33f));
        CHECK(getF32(b.subspan(14, 4)) == doctest::Approx(4.0f));
        CHECK(b[18] == std::byte{1});
        CHECK(getF32(b.subspan(19, 4)) == doctest::Approx(0.04f));
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
        return it->second.back();
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

// ---- bd val-4rr: a persist waits for a still window -------------------------------

TEST_CASE("VD-PERSIST-1: a save requested mid-stroke lands in the next still window, once, never during motion") {
    auto rig = std::make_unique<Rig>();
    rig->step(3000);
    rig->persisted = 0;
    rig->persistWrites = 0;
    g_still = false;   // a stroke in flight
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(9, IntentValue::ofF32(30.0f))).has_value());
    rig->step(3000);   // past the debounce
    REQUIRE(rig->del.nacks.empty());
    CHECK(rig->persisted == 0);
    CHECK(g_stillWindowUs >= 60000u);
    // A second change while moving coalesces into the same write.
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(9, IntentValue::ofF32(35.0f))).has_value());
    rig->step(3000);
    CHECK(rig->persisted == 0);

    g_still = true;
    rig->step(1);
    CHECK(rig->persisted == kPersistConfig);
    CHECK(rig->persistWrites == 1);
    rig->step(3000);
    CHECK(rig->persistWrites == 1);
}

TEST_CASE("VD-PERSIST-2: under ESTOP a due write goes whatever the strip shows; a home cycle holds it") {
    {
        auto rig = std::make_unique<Rig>();
        rig->step(3000);
        rig->persisted = 0;
        g_still = false;
        g_census.estop = true;
        REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(9, IntentValue::ofF32(30.0f))).has_value());
        rig->step(3000);
        CHECK(rig->persisted == kPersistConfig);
    }
    auto rig = std::make_unique<Rig>();
    rig->step(3000);
    rig->persisted = 0;
    g_census.homing = true;
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(9, IntentValue::ofF32(30.0f))).has_value());
    rig->step(3000);
    CHECK(rig->persisted == 0);
    g_census.homing = false;
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
    CHECK(g_osc.amplitude_drive.out_max == 1.0f);
    {
        const auto it = rig->del.lastState.find(ch::oscillator);
        REQUIRE(it != rig->del.lastState.end());
        REQUIRE(it->second.size() == 57);
        CHECK(it->second[23] == std::byte{osc_drives::axis});
        CHECK(it->second[40] == std::byte{osc_drives::axis});
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

    // Driven by the plan, the twin and the echo name the sine that plays, no
    // dwells; the written values are kept.
    {
        IntentValueMap s{};
        s.count = 2;
        s.fields[0] = IntentValueField{4, IntentValue::ofU64(1)};
        s.fields[1] = IntentValueField{5, IntentValue::ofF32(0.3f)};
        REQUIRE(rig->client->sendIntent(ch::osc_set, s).has_value());
    }
    rig->step();
    CHECK(echoed(rig->del.lastEcho, 4)->u64_val == 0);
    CHECK(echoed(rig->del.lastEcho, 5)->f32_val == 0.0f);
    {
        const std::span<const std::byte> tw(rig->del.lastState.find(ch::oscillator)->second);
        CHECK(tw[9] == std::byte{0});
        CHECK(getF32(tw.subspan(10, 4)) == 0.0f);
    }
    CHECK(g_osc.shape == 1);
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

// ---- shape and dwells as they play (Valence RFC-110 item 3) -----------------

TEST_CASE("VD-OSC-SHOWN: the factory card reports its shape and dwells by hand, a live stream the sine with no dwells, each change a hub-side cfg_gen") {
    auto rig = std::make_unique<Rig>();
    IntentValueMap m{};
    m.count = 3;
    m.fields[0] = IntentValueField{4, IntentValue::ofU64(1)};        // square
    m.fields[1] = IntentValueField{5, IntentValue::ofF32(0.25f)};
    m.fields[2] = IntentValueField{6, IntentValue::ofF32(0.5f)};
    REQUIRE(rig->client->sendIntent(ch::osc_set, m).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    CHECK(echoed(rig->del.lastEcho, 4)->u64_val == 1);
    CHECK(echoed(rig->del.lastEcho, 5)->f32_val == doctest::Approx(0.25f));
    auto twin = [&] { return std::span<const std::byte>(rig->del.lastState[ch::oscillator]); };
    CHECK(twin()[9] == std::byte{1});
    CHECK(getF32(twin().subspan(10, 4)) == doctest::Approx(0.25f));
    CHECK(getF32(twin().subspan(14, 4)) == doctest::Approx(0.5f));
    // A stream goes live: the sine, no dwells, published with cfg_gen moved.
    uint16_t gen = rig->hub->cfgGen();
    g_census.osc_stream_live = true;
    rig->step(60);
    CHECK(twin()[9] == std::byte{0});
    CHECK(getF32(twin().subspan(10, 4)) == 0.0f);
    CHECK(getF32(twin().subspan(14, 4)) == 0.0f);
    CHECK(rig->hub->cfgGen() != gen);
    CHECK(g_osc.shape == 1);
    // A write of shape while it is live keeps it and echoes the sine.
    REQUIRE(rig->client->sendIntent(ch::osc_set, oneKey(4, IntentValue::ofU64(2))).has_value());
    rig->step();
    CHECK(echoed(rig->del.lastEcho, 4)->u64_val == 0);
    CHECK(g_osc.shape == 2);
    // Quiet: the kept values again, a hub-side change.
    gen = rig->hub->cfgGen();
    g_census.osc_stream_live = false;
    rig->step(60);
    CHECK(twin()[9] == std::byte{2});
    CHECK(getF32(twin().subspan(10, 4)) == doctest::Approx(0.25f));
    CHECK(rig->hub->cfgGen() != gen);
    // A kept sine with no dwells reports nothing new: no cfg_gen.
    m.fields[0] = IntentValueField{4, IntentValue::ofU64(0)};
    m.fields[1] = IntentValueField{5, IntentValue::ofF32(0.0f)};
    m.fields[2] = IntentValueField{6, IntentValue::ofF32(0.0f)};
    REQUIRE(rig->client->sendIntent(ch::osc_set, m).has_value());
    rig->step();
    gen = rig->hub->cfgGen();
    g_census.osc_stream_live = true;
    rig->step(60);
    g_census.osc_stream_live = false;
    rig->step(60);
    CHECK(rig->hub->cfgGen() == gen);
    // Both drives fixed: a live stream drives nothing, the square plays.
    m.count = 3;
    m.fields[0] = IntentValueField{4, IntentValue::ofU64(1)};
    m.fields[1] = IntentValueField{7, IntentValue::ofU64(osc_drives::fixed)};
    m.fields[2] = IntentValueField{12, IntentValue::ofU64(osc_drives::fixed)};
    REQUIRE(rig->client->sendIntent(ch::osc_set, m).has_value());
    rig->step();
    g_census.osc_stream_live = true;
    rig->step(60);
    CHECK(twin()[9] == std::byte{1});
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
