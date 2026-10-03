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

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

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
#include "../../../flagship_p4/src/patterns/AdvancedPattern.cpp"

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
// The fake e-stop reading: what the BoardIo task would have published.
estop::Reading g_estop{};

}  // namespace

namespace valence {

uint64_t deviceNowUs() { return g_clock.nowUs(); }
uint32_t deviceFreeHeapBytes() { return 0; }

bool motionBegin() { return true; }
bool motionSubmit(const MotionIntent&) { return true; }
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
MotionCensus motionCensus() { return g_census; }
MotionTuning motionDefaultTuning() { return MotionTuning{}; }
void motionSetTuning(const MotionTuning&) {}

bool patternBegin() { return true; }
void patternSetSettings(const PatternSettings&) {}
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
        g_switch = MotorSwitchStatus{};
        g_switch.state = motorswitch::State::on;
        g_estop = estop::Reading{};
        g_estop.known = true;
        REQUIRE(buildValenceCatalog(catalog, boardFeatures()));
        device.setUnvouchedRole(AccessLevel::control);
        device.setSetupWritten(kSetupRequiredMask);
        hub.emplace(catalog, g_clock, hubRng, device);
        device.attach(*hub);
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
