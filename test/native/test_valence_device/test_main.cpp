// test_valence_device -- native doctest suite for the delegate's safety seams with the hub
// Constraints:
// - ValenceDevice.cpp is compiled verbatim into this one translation unit,
//   driven by a real Hub over InProcessLink. The motion, pattern, motor
//   switch, button and e-stop doors are fakes below: the composition's seam,
//   never the board.
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
int g_patPushes = 0;
// The fake e-stop reading: what the BoardIo task would have published.
estop::Reading g_estop{};

}  // namespace

namespace geiger {
uint32_t hostNowMs() { return uint32_t(g_clock.nowUs() / 1000); }
}  // namespace geiger

namespace valence {

uint64_t deviceNowUs() { return g_clock.nowUs(); }
uint32_t deviceFreeHeapBytes() { return 0; }

bool motionBegin() { return true; }
bool motionSubmit(const MotionIntent&) {
    ++g_submits;
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
MotionTuning motionDefaultTuning() { return MotionTuning{}; }
void motionSetTuning(const MotionTuning&) {}

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
    std::vector<RecordedNack> nacks;
    int echoes = 0;
    IntentValueMap lastEcho{};

    void onStateChange(ClientSessionState) override {}
    void onState(uint16_t channel_id, uint16_t, std::span<const std::byte> payload) override {
        lastState[channel_id] = std::vector<std::byte>(payload.begin(), payload.end());
    }
    void onEcho(uint16_t, const IntentValueMap& applied, uint16_t) override {
        ++echoes;
        lastEcho = applied;
    }
    void onNack(const NackMsg& n) override {
        nacks.push_back({n.code, n.has_detail ? std::string(n.detail) : std::string()});
    }
    void onPendingDropped(uint16_t) override {}
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

    Rig() {
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
        g_switch = MotorSwitchStatus{};
        g_switch.state = motorswitch::State::on;
        g_estop = estop::Reading{};
        g_estop.known = true;
        REQUIRE(buildValenceCatalog(catalog, boardFeatures()));
        device.setUnvouchedRole(AccessLevel::control);
        device.setSetupWritten(kSetupRequiredMask);
        hub.emplace(catalog, g_clock, hubRng, device);
        device.attach(*hub, catalog);
        link.emplace(g_clock, hubRng);
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
        // The STATE every writer moves, so a refused write is seen to move none.
        for (const uint16_t id : {ch::machine_config, ch::machine_modes, ch::kinetic_limits, ch::kinetic_chase,
                                  ch::kinetic_waveform, ch::pattern_state, ch::pattern_presets_roster})
            REQUIRE(client->addSubscriptionWish(id, 0.0f, Priority::normal));
        REQUIRE(client->connect());
        step(200);
        REQUIRE(client->state() == ClientSessionState::LIVE);
    }

    void step(int rounds = 16) {
        for (int i = 0; i < rounds; ++i) {
            g_clock.advanceUs(1000);
            hub->update(g_clock.nowUs());
            device.tick(g_clock.nowUs() / 1000);
            client->update(g_clock.nowUs());
        }
    }

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
    for (const uint16_t id : {ch::machine_config, ch::machine_modes, ch::kinetic_chase, ch::pattern_state,
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
        expectNotANumberRefused(*rig, ch::kinetic_set, base, 8, IntentValue::ofF32(0.37f),
                                "chase_gain: not a number");
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

TEST_CASE("VD-23: a move with no position is refused, never read as position 0") {
    auto rig = std::make_unique<Rig>();
    REQUIRE(rig->client->sendIntent(ch::move, IntentValueMap{}).has_value());
    rig->step();
    REQUIRE(rig->del.nacks.size() == 1);
    CHECK(rig->del.nacks[0].code == NackCode::INVALID_VALUE);
    CHECK(g_submits == 0);
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
// Offsets per StoredState.h: header 7 B, then the eight 0x1000 values, then
// jmax, vmax, amax, chase_gain.
constexpr size_t kBlobJogSpeed = 7 + 2 * 4;
constexpr size_t kBlobChaseGain = 7 + 8 * 4 + 3 * 4;
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

TEST_CASE("VD-TR-2: revert and a session's end restore the kinetic and modes baselines") {
    auto rig = std::make_unique<Rig>();
    const float gain0 = storedF32(*rig, kBlobChaseGain);
    REQUIRE(rig->client->sendIntent(ch::kinetic_set, oneKey(8, IntentValue::ofF32(0.25f)), std::nullopt, false,
                                    true).has_value());
    REQUIRE(rig->client->sendIntent(ch::modes_set, oneKey(4, IntentValue::ofU64(0)), std::nullopt, false, true)
                .has_value());
    rig->step();
    REQUIRE(rig->del.nacks.empty());
    CHECK(rig->hub->trialCount() == 2);
    CHECK(stateU8(*rig, ch::kinetic_chase, 20) == 0x04);   // bit 2: chase_gain
    CHECK(stateU8(*rig, ch::machine_modes, 7) == 0x01);    // bit 0: overshoot_clamp
    CHECK(storedF32(*rig, kBlobChaseGain) == doctest::Approx(gain0));
    CHECK(getF32(std::span<const std::byte>(rig->del.lastState[ch::kinetic_chase]).subspan(2, 4)) ==
          doctest::Approx(0.25f));

    REQUIRE(rig->client->sendIntent(channels::settings_trial, oneKey(1, IntentValue::ofU64(trial_ops::revert)))
                .has_value());
    rig->step();
    CHECK(rig->hub->trialCount() == 0);
    CHECK(getF32(std::span<const std::byte>(rig->del.lastState[ch::kinetic_chase]).subspan(2, 4)) ==
          doctest::Approx(gain0));
    CHECK(stateU8(*rig, ch::kinetic_chase, 20) == 0x00);

    // A session that goes away takes its trial with it.
    REQUIRE(rig->client->sendIntent(ch::kinetic_set, oneKey(8, IntentValue::ofF32(0.5f)), std::nullopt, false,
                                    true).has_value());
    rig->step();
    CHECK(rig->hub->trialCount() == 1);
    rig->client->disconnect();
    rig->step(50);
    CHECK(rig->hub->trialCount() == 0);
    // The client is gone, so the live value is read where a baseline is.
    CHECK(rig->device.trialBaseline(ch::kinetic_set, 8)->f32_val == doctest::Approx(gain0));
}

TEST_CASE("VD-TR-3: keys gated on live state refuse a trial; the flip waits for a window trial") {
    auto rig = std::make_unique<Rig>();
    REQUIRE(rig->client->sendIntent(ch::kinetic_set, oneKey(10, IntentValue::ofF32(40.0f)), std::nullopt, false,
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
// home_speed the blob's last four bytes.
constexpr size_t kBlobMaxRail = 7 + 7 * 4;
constexpr size_t kBlobHomeSpeed = stored::kConfigBlobBytes - 4;

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
    CHECK(logged("HOME: homed. Home datum 0.0 mm, far datum 612.4 mm: rail 612.4 mm, max_rail 612.4 mm"));

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
    CHECK(logged("HOME: homed. Home datum 0.0 mm, far datum 2600.0 mm: rail 2600.0 mm, max_rail 2000.0 mm"));
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

TEST_CASE("VD-MODES-9: home_speed writes on modes-set key 9, clamped, published at 0x1030's tail, stored") {
    auto rig = std::make_unique<Rig>();
    persistBitsOver(*rig, 2500);
    const uint16_t gen = rig->hub->cfgGen();
    CHECK(stateF32(*rig, ch::machine_modes, 8) == doctest::Approx(factory::home_speed));
    CHECK((stateU8(*rig, ch::machine_modes, 3) & 0x08) != 0);   // bit 3: home_speed, always open

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
