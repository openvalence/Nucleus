// test_arbiter_table -- the MotionArbiter ownership table, cell by cell
// Constraints:
// - Hardware-free and deterministic: a synthetic microsecond clock, the host
//   twin's ideal emitter (its fence and lease as the LP core applies them) and
//   a home sense polled on the planner tick. No IDF, no FreeRTOS.
// - Compiles flagship_p4/src/motion/MotionArbiter.cpp itself with
//   MOTION_ARB_OBSERVE defined here, so every cell the arbiter is asked for is
//   recorded: a case that reaches a cell with no rule fails unless it asks to.
// - The last case is the coverage gate: every legal cell but the race-only
//   ones (kRaceOnly) must have been reached by a case above it.
// See: flagship_p4/src/motion/MotionArbiter.h (the ownership table), bd val-5ozv.1

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// Named here so the dependency finder builds them; the .cpp below needs all three.
#include "geiger/geiger.h"
#include "kinetic2/types.hpp"
#include "valence/generated/registry_constants.hpp"

#include "../../../flagship_p4/src/motion/MotionArbiter.h"

namespace {
using valence::ArbEvent;
using valence::ArbMode;
using valence::kArbEvents;
using valence::kArbModes;

std::array<std::array<int, kArbEvents>, kArbModes> g_hits{};
std::vector<std::pair<ArbMode, ArbEvent>> g_illegal;

void observeCell(ArbMode m, ArbEvent e, bool legal) {
    ++g_hits[size_t(m)][size_t(e)];
    if (!legal) g_illegal.emplace_back(m, e);
}
}  // namespace

#define MOTION_ARB_OBSERVE(from, ev, legal) observeCell(from, ev, legal)
#include "../../../flagship_p4/src/motion/MotionArbiter.cpp"
#include "../../../sim/neutrino/src/IdealEmitter.h"

using valence::ArbAct;
using valence::ArbCell;
using valence::ArbFeed;
using valence::HomeSense;
using valence::HomeStart;
using valence::IdealEmitter;
using valence::kMmPerStep;
using valence::MotionArbiter;
using valence::MotionCensus;
using valence::MotionIntent;
using valence::MotionSource;
using valence::ReturnStart;
using valence::WindowPost;

namespace {

uint64_t g_now_us = 1'000'000;
uint64_t nowUs() { return g_now_us; }

// Hard stops in the emitter's boot frame: HIGH at or below lo, at or above hi.
// sticky_lo holds the line once the home stop has read HIGH; hi_sticky_from
// holds it from that many rises of the far stop on (2: the far re-touch).
// on_probe runs once inside home(), with the request word already set: a
// planner tick there is the planner preempting the hub between the word and
// the request.
class StopSense final : public HomeSense {
public:
    explicit StopSense(const IdealEmitter& e) : emitter(e) {}
    bool present() const override { return true; }
    Probe probe() override {
        if (on_probe) {
            auto f = std::move(on_probe);
            on_probe = nullptr;
            f();
        }
        if (probe_as) return *probe_as;
        return level() ? Probe::high : Probe::low;
    }
    bool high() override {
        const float p = float(emitter.count()) * kMmPerStep;
        const bool l = p <= lo_mm, h = p >= hi_mm;
        if (l) lo_seen = true;
        if (h && !was_hi) ++hi_rises;
        was_hi = h;
        return l || h || (sticky_lo && lo_seen) || (hi_sticky_from > 0 && hi_rises >= hi_sticky_from);
    }
    bool level() const {
        const float p = float(emitter.count()) * kMmPerStep;
        return p <= lo_mm || p >= hi_mm;
    }

    const IdealEmitter& emitter;
    float lo_mm = -10.0f;
    float hi_mm = 60.0f;
    bool sticky_lo = false;
    int hi_sticky_from = 0;
    std::optional<Probe> probe_as;
    std::function<void()> on_probe;
    bool lo_seen = false;
    bool was_hi = false;
    int hi_rises = 0;
};

// Built on the heap (the arbiter holds a KB-scale engine): powered and
// commissioned unless a case says otherwise, with the stop sense wired.
struct Rig {
    IdealEmitter emitter;
    MotionArbiter arb{emitter, &nowUs};
    StopSense sense{emitter};

    explicit Rig(bool powered = true) {
        arb.setHomeSense(sense);
        emitter.advance(g_now_us);
        arb.begin(g_now_us);
        arb.setMotorPowered(powered);
        arb.setCommissioned(true);
    }
    void run(uint64_t us) {
        for (uint64_t t = 0; t < us; t += 1000) {
            g_now_us += 1000;
            emitter.advance(g_now_us);
            arb.evaluate(g_now_us, 1e-3f);
        }
    }
    bool runUntil(ArbMode m, uint64_t max_us) {
        for (uint64_t t = 0; t < max_us && arb.arbMode() != m; t += 1000) run(1000);
        return arb.arbMode() == m;
    }
    bool submit(MotionSource src, float mm) {
        MotionIntent in;
        in.source = src;
        in.target_mm = mm;
        return arb.accept(in, g_now_us);
    }
    ArbMode mode() const { return arb.arbMode(); }
    MotionCensus census() { return arb.snapshot(g_now_us); }
};

// Every case starts with no illegal cell recorded and must end with none,
// unless it reads and clears them itself.
struct NoIllegal {
    NoIllegal() { g_illegal.clear(); }
    ~NoIllegal() { CHECK(g_illegal.empty()); }
};

std::unique_ptr<Rig> homedRig() {
    auto r = std::make_unique<Rig>();
    r->arb.forceHome(400.0f);
    r->run(1000);
    return r;
}

// Jog under override after a jog to 50 mm, at rest: a return has 50 mm to go.
std::unique_ptr<Rig> jogRig() {
    auto r = homedRig();
    r->arb.override();
    r->run(1000);
    REQUIRE(r->mode() == ArbMode::Jog);
    REQUIRE(r->submit(MotionSource::Manual, 50.0f));
    r->run(4'000'000);
    REQUIRE_FALSE(r->census().busy);
    return r;
}

std::unique_ptr<Rig> returningRig() {
    auto r = jogRig();
    REQUIRE(r->arb.returnToPause() == ReturnStart::queued);
    r->run(1000);
    REQUIRE(r->mode() == ArbMode::Returning);
    return r;
}

// A cycle in flight, stopped in `phase`.
std::unique_ptr<Rig> seekRig(ArbMode phase) {
    auto r = std::make_unique<Rig>();
    REQUIRE(r->arb.home() == HomeStart::started);
    REQUIRE(r->runUntil(phase, 20'000'000));
    return r;
}

// Holds the planner in HomeQueued for `body`: one tick inside home(), between
// the request word and the request.
void inQueued(Rig& r, const std::function<void()>& body) {
    r.sense.on_probe = [&r, body] {
        r.run(1000);
        REQUIRE(r.mode() == ArbMode::HomeQueued);
        body();
    };
    (void)r.arb.home();
}

// A session's live edit and the stored window's adoption: the same cells but
// the act, so every window case runs both.
constexpr std::array<WindowPost, 2> kPosts{WindowPost::edit, WindowPost::adopt};
ArbEvent windowEvent(WindowPost post) {
    return post == WindowPost::edit ? ArbEvent::WindowEdit : ArbEvent::WindowAdopt;
}

constexpr std::array<ArbMode, 4> kSeekModes{ArbMode::HomeApproach, ArbMode::HomeClear, ArbMode::HomeTouch,
                                            ArbMode::HomeFinish};

}  // namespace

// ---- the table itself ---------------------------------------------------------

namespace {
// The audited table, written apart from the rules it checks. One row per mode,
// one column per event in ArbEvent order; "." is illegal, else NEXT:act.
// NEXT: R Run, P Paused, J Jog, T Returning, Q HomeQueued, A HomeApproach,
// C HomeClear, H HomeTouch, F HomeFinish, U Unpowered, E Estop, L the layer.
// act: no none, st settle, br brake, rc record, qu queue, rf refuse, sa start,
// lg leg, ec endCycle, hm homed, fr reframe, rk recheck, df defer, ro reorigin,
// pr planReturn, ar arrive, ad adopt.
// Columns: estop release poweroff poweron pause resume override ovroff brake
// homereq homecancel homeabort homestart homemoving contact cleared datum
// measured done fail window adopt reorigin return returnfailed arrived.
const std::array<const char*, kArbModes> kGrid{
    /* Run       */ "E:st . U:st . P:no . J:no . P:br Q:qu . . . . . . . . . . R:fr R:ad R:ro . . .",
    /* Paused    */ "E:st . U:st . . R:no J:no . P:br Q:qu . . . . . . . . . . P:rk P:ad P:ro . . .",
    /* Jog       */ "E:st . U:st . . . . P:no J:br Q:qu . . . . . . . . . . J:rk J:ad J:ro T:pr J:pr .",
    /* Returning */ "E:st . U:st . . . . . T:br . . T:rf . . . . . . . . T:rk T:ad T:ro T:pr J:pr P:ar",
    /* Queued    */ "E:st . U:st . . . . . Q:br . L:no L:rf A:sa L:rf . . . . . . Q:df Q:df L:rf T:pr Q:pr .",
    /* Approach  */ "E:st . U:st . . . . . A:br . . L:ec . . C:lg . . . . L:ec A:df A:df L:ec . . .",
    /* Clear     */ "E:st . U:st . . . . . C:br . . L:ec . . . H:lg . . . L:ec C:df C:df L:ec . . .",
    /* Touch     */ "E:st . U:st . . . . . H:br . . L:ec . . . . A:lg F:lg . L:ec H:df H:df L:ec . . .",
    /* Finish    */ "E:st . U:st . . . . . F:br . . L:ec . . . . . . L:hm L:ec F:df F:df L:ec . . .",
    /* Unpowered */ "E:st . U:rf L:no . . . . U:rc . . . . . . . . . . . U:df U:df U:df . . .",
    /* Estop     */ "E:rf P:no . . . . . . . . . . . . . . . . . . E:df E:df E:df . . .",
};

std::string token(const ArbCell& c) {
    if (c.act == ArbAct::illegal) return ".";
    static constexpr const char* kNext = "RPJTQACHFUE";
    static constexpr std::array<const char*, 18> kAct{"",   "no", "st", "br", "rc", "qu", "rf", "sa", "lg",
                                                      "ec", "hm", "fr", "rk", "df", "ro", "pr", "ar", "ad"};
    std::string t(1, c.layer ? 'L' : kNext[size_t(c.next)]);
    return t + ":" + kAct[size_t(c.act)];
}

// Reachable only by a post landing between two of the planner's reads of one
// tick, or by no public path at all: the table checks them, no case reaches
// them. A legal cell missing here and never reached fails the last case.
const std::vector<std::pair<ArbMode, ArbEvent>> kRaceOnly{
    {ArbMode::Run, ArbEvent::Brake},           {ArbMode::HomeQueued, ArbEvent::Brake},
    {ArbMode::HomeApproach, ArbEvent::Brake},  {ArbMode::HomeClear, ArbEvent::Brake},
    {ArbMode::HomeTouch, ArbEvent::Brake},     {ArbMode::HomeFinish, ArbEvent::Brake},
    {ArbMode::Returning, ArbEvent::HomeAbort}, {ArbMode::HomeQueued, ArbEvent::Return},
    {ArbMode::HomeQueued, ArbEvent::ReturnFailed},
    // No public path refuses a return's plan: its target is the clamped
    // paused position and a jog-set knot after a truncate.
    {ArbMode::Jog, ArbEvent::ReturnFailed},    {ArbMode::Returning, ArbEvent::ReturnFailed},
    // home() reads the latch and the gate before it sets the request word.
    {ArbMode::Estop, ArbEvent::EstopOn},       {ArbMode::Unpowered, ArbEvent::PowerOff},
};
}  // namespace

TEST_CASE("table: every cell is the audited one") {
    for (size_t m = 0; m < kArbModes; ++m) {
        std::istringstream row(kGrid[m]);
        for (size_t e = 0; e < kArbEvents; ++e) {
            std::string want;
            REQUIRE(static_cast<bool>(row >> want));
            INFO(valence::kArbModeNames[m] << " x " << valence::kArbEventNames[e]);
            CHECK(token(valence::arbCell(ArbMode(m), ArbEvent(e))) == want);
        }
        std::string extra;
        CHECK_FALSE(static_cast<bool>(row >> extra));
    }
}

TEST_CASE("table: the feed column, by mode and source") {
    // Manual, Stream, Pattern, Advanced: f feeds, j the jog, anything else
    // the refusal's initial (e-stop, unpowered, homing, paused, returning).
    const std::array<const char*, kArbModes> want{
        "ffff", "pppp", "jppp", "rppp", "hhhh", "hhhh", "hhhh", "hhhh", "hhhh", "uuuu", "eeee"};
    static constexpr const char* kCode = "fjeuhpr";
    for (size_t m = 0; m < kArbModes; ++m)
        for (size_t s = 0; s < 4; ++s) {
            INFO(valence::kArbModeNames[m] << " source " << s);
            CHECK(kCode[size_t(valence::kArbFeed[m][s])] == want[m][s]);
        }
}

// ---- the layer, the latch and the gate ----------------------------------------

TEST_CASE("cells: boot, power on, pause, brake, resume, override from Run") {
    NoIllegal guard;
    auto r = std::make_unique<Rig>(false);
    r->run(1000);
    CHECK(r->mode() == ArbMode::Unpowered);   // boots off
    r->arb.setMotorPowered(true);
    r->run(1000);
    CHECK(r->mode() == ArbMode::Run);
    r->arb.forceHome(400.0f);
    r->run(1000);
    r->arb.pause(true);
    r->run(1000);
    CHECK(r->mode() == ArbMode::Paused);
    CHECK(g_hits[size_t(ArbMode::Paused)][size_t(ArbEvent::Brake)] > 0);
    r->arb.pause(false);
    r->run(1000);
    CHECK(r->mode() == ArbMode::Run);
    r->arb.override();
    r->run(1000);
    CHECK(r->mode() == ArbMode::Jog);
    CHECK(r->census().arb_mode == uint8_t(ArbMode::Jog));
    CHECK(r->census().arb_transitions == 4);
}

TEST_CASE("cells: override from Paused; the return, a second request, its arrival") {
    NoIllegal guard;
    auto r = homedRig();
    r->arb.pause(true);
    r->run(1000);
    r->arb.override();
    r->run(1000);
    CHECK(r->mode() == ArbMode::Jog);
    REQUIRE(r->submit(MotionSource::Manual, 50.0f));
    r->run(4'000'000);
    REQUIRE(r->arb.returnToPause() == ReturnStart::queued);
    r->run(1000);
    CHECK(r->mode() == ArbMode::Returning);
    CHECK(r->census().returning);
    REQUIRE(r->arb.returnToPause() == ReturnStart::queued);
    r->run(1000);
    CHECK(r->mode() == ArbMode::Returning);
    CHECK(g_hits[size_t(ArbMode::Returning)][size_t(ArbEvent::Return)] > 0);
    r->run(5'000'000);
    CHECK(r->mode() == ArbMode::Paused);
    const MotionCensus c = r->census();
    CHECK_FALSE(c.returning);
    CHECK_FALSE(c.override_mode);
    CHECK(c.returns == 1);
    CHECK(std::fabs(c.position_mm) <= kMmPerStep);
}

TEST_CASE("cells: an unpowered RETURN arriving while the planner sat in Jog is override off") {
    NoIllegal guard;
    auto r = homedRig();
    r->arb.override();
    r->run(1000);
    REQUIRE(r->mode() == ArbMode::Jog);
    // Power lost, the return answered on the spot, power back: no tick between.
    r->arb.setMotorPowered(false);
    REQUIRE(r->arb.returnToPause() == ReturnStart::arrived);
    r->arb.setMotorPowered(true);
    r->run(1000);
    CHECK(r->mode() == ArbMode::Paused);
    CHECK(g_hits[size_t(ArbMode::Jog)][size_t(ArbEvent::OverrideOff)] > 0);
}

TEST_CASE("cells: a forceHome under a RETURN brakes it, and the return arrives") {
    NoIllegal guard;
    auto r = returningRig();
    r->arb.forceHome(400.0f);
    r->run(2000);
    CHECK(g_hits[size_t(ArbMode::Returning)][size_t(ArbEvent::Brake)] > 0);
    CHECK(g_hits[size_t(ArbMode::Returning)][size_t(ArbEvent::Reorigin)] > 0);
    CHECK(r->mode() == ArbMode::Paused);
}

TEST_CASE("cells: ESTOP from every mode, released into PAUSE") {
    auto check = [](std::unique_ptr<Rig> r, ArbMode from) {
        NoIllegal guard;
        INFO(valence::kArbModeNames[size_t(from)]);
        REQUIRE(r->mode() == from);
        r->arb.estop(true);
        r->run(1000);
        CHECK(r->mode() == ArbMode::Estop);
        CHECK(g_hits[size_t(from)][size_t(ArbEvent::EstopOn)] > 0);
        CHECK_FALSE(r->census().homing);
        CHECK_FALSE(r->census().returning);
        r->arb.estop(false);
        r->run(1000);
        CHECK(r->mode() == (r->census().power_gate ? ArbMode::Paused : ArbMode::Unpowered));
    };
    check(homedRig(), ArbMode::Run);
    {
        auto r = homedRig();
        r->arb.pause(true);
        r->run(1000);
        check(std::move(r), ArbMode::Paused);
    }
    check(jogRig(), ArbMode::Jog);
    check(returningRig(), ArbMode::Returning);
    for (ArbMode m : kSeekModes) check(seekRig(m), m);
    {
        auto r = std::make_unique<Rig>(false);
        r->run(1000);
        check(std::move(r), ArbMode::Unpowered);
    }
    {
        NoIllegal guard;
        auto r = std::make_unique<Rig>();
        inQueued(*r, [&r] {
            r->arb.estop(true);
            r->run(1000);
            CHECK(r->mode() == ArbMode::Estop);
        });
        CHECK(g_hits[size_t(ArbMode::HomeQueued)][size_t(ArbEvent::EstopOn)] > 0);
        r->arb.estop(false);
        r->run(2000);
        CHECK(r->mode() == ArbMode::Paused);
        CHECK_FALSE(r->census().homing);
    }
}

TEST_CASE("cells: a power loss from every powered mode, and power back into the layer") {
    auto check = [](std::unique_ptr<Rig> r, ArbMode from, ArbMode back) {
        NoIllegal guard;
        INFO(valence::kArbModeNames[size_t(from)]);
        REQUIRE(r->mode() == from);
        r->arb.setMotorPowered(false);
        r->run(1000);
        CHECK(r->mode() == ArbMode::Unpowered);
        CHECK(g_hits[size_t(from)][size_t(ArbEvent::PowerOff)] > 0);
        CHECK_FALSE(r->census().homing);
        r->arb.setMotorPowered(true);
        r->run(1000);
        CHECK(r->mode() == back);
    };
    check(homedRig(), ArbMode::Run, ArbMode::Run);
    {
        auto r = homedRig();
        r->arb.pause(true);
        r->run(1000);
        check(std::move(r), ArbMode::Paused, ArbMode::Paused);
    }
    check(jogRig(), ArbMode::Jog, ArbMode::Jog);
    // A RETURN cut by the loss leaves override latched: the jog's layer.
    check(returningRig(), ArbMode::Returning, ArbMode::Jog);
    for (ArbMode m : kSeekModes) check(seekRig(m), m, ArbMode::Run);
    {
        NoIllegal guard;
        auto r = std::make_unique<Rig>();
        inQueued(*r, [&r] {
            r->arb.setMotorPowered(false);
            r->run(1000);
            CHECK(r->mode() == ArbMode::Unpowered);
        });
        CHECK(g_hits[size_t(ArbMode::HomeQueued)][size_t(ArbEvent::PowerOff)] > 0);
    }
}

TEST_CASE("cells: a PAUSE while unpowered records the rest, brakes nothing") {
    NoIllegal guard;
    auto r = std::make_unique<Rig>(false);
    r->arb.forceHome(400.0f);
    r->run(1000);
    r->arb.pause(true);
    r->run(1000);
    CHECK(r->mode() == ArbMode::Unpowered);
    CHECK(g_hits[size_t(ArbMode::Unpowered)][size_t(ArbEvent::Brake)] > 0);
    r->arb.setMotorPowered(true);
    r->run(1000);
    CHECK(r->mode() == ArbMode::Paused);
}

TEST_CASE("cells: a window edit or adoption in every mode outside a cycle is the motion layer's") {
    for (const WindowPost post : kPosts) {
        NoIllegal guard;
        auto edit = [post](Rig& r, ArbMode m) {
            INFO(valence::kArbModeNames[size_t(m)] << " post " << int(post));
            REQUIRE(r.mode() == m);
            r.arb.setWindow(10.0f, 300.0f, 400.0f, post);
            r.run(1000);
            CHECK(r.mode() == m);
            CHECK(g_hits[size_t(m)][size_t(windowEvent(post))] > 0);
        };
        {
            auto r = homedRig();
            edit(*r, ArbMode::Run);
            r->arb.pause(true);
            r->run(1000);
            edit(*r, ArbMode::Paused);
        }
        edit(*jogRig(), ArbMode::Jog);
        edit(*returningRig(), ArbMode::Returning);
    }
}

TEST_CASE("cells: a window edit or adoption under ESTOP or with the power off is held, and applied on the release or the power") {
    for (const WindowPost post : kPosts) {
        NoIllegal guard;
        INFO("post " << int(post));
        const ArbEvent ev = windowEvent(post);
        {
            auto r = homedRig();
            r->arb.estop(true);
            r->run(1000);
            r->arb.setWindow(10.0f, 300.0f, 400.0f, post);
            r->run(3000);
            CHECK(r->mode() == ArbMode::Estop);
            CHECK(g_hits[size_t(ArbMode::Estop)][size_t(ev)] > 0);
            CHECK(r->census().window_held == 1);
            CHECK(r->census().window_edits == 0);
            r->arb.estop(false);
            r->run(1000);
            CHECK(r->mode() == ArbMode::Paused);
            CHECK(r->census().window_edits == 1);
            CHECK(g_hits[size_t(ArbMode::Paused)][size_t(ev)] > 0);
        }
        {
            auto r = homedRig();
            r->arb.setMotorPowered(false);
            r->run(1000);
            r->arb.setWindow(10.0f, 300.0f, 400.0f, post);
            r->run(3000);
            CHECK(r->mode() == ArbMode::Unpowered);
            CHECK(g_hits[size_t(ArbMode::Unpowered)][size_t(ev)] > 0);
            CHECK(r->census().window_held == 1);
            CHECK(r->census().window_edits == 0);
            r->arb.setMotorPowered(true);
            r->run(1000);
            CHECK(r->mode() == ArbMode::Run);
            CHECK(r->census().window_edits == 1);
        }
    }
}

TEST_CASE("cells: a force_home in every mode: a reset outside a cycle, the end of one inside, held by the latches") {
    NoIllegal guard;
    auto reorigin = [](Rig& r, ArbMode m, ArbMode after) {
        INFO(valence::kArbModeNames[size_t(m)]);
        REQUIRE(r.mode() == m);
        r.arb.forceHome(400.0f);
        r.run(1000);
        CHECK(g_hits[size_t(m)][size_t(ArbEvent::Reorigin)] > 0);
        CHECK(r.mode() == after);
    };
    {
        auto r = homedRig();
        reorigin(*r, ArbMode::Run, ArbMode::Run);
        r->arb.pause(true);
        r->run(1000);
        reorigin(*r, ArbMode::Paused, ArbMode::Paused);
    }
    reorigin(*jogRig(), ArbMode::Jog, ArbMode::Jog);
    for (ArbMode m : kSeekModes) {
        auto r = seekRig(m);
        reorigin(*r, m, ArbMode::Run);
        CHECK_FALSE(r->census().homing);
    }
    {
        auto r = std::make_unique<Rig>();
        inQueued(*r, [&r] {
            r->arb.forceHome(400.0f);
            r->run(1000);
            CHECK(r->mode() == ArbMode::Run);
        });
        CHECK(g_hits[size_t(ArbMode::HomeQueued)][size_t(ArbEvent::Reorigin)] > 0);
    }
    {
        auto r = std::make_unique<Rig>(false);
        r->run(1000);
        reorigin(*r, ArbMode::Unpowered, ArbMode::Unpowered);
        r->arb.setMotorPowered(true);
        r->run(1000);
        CHECK(g_hits[size_t(ArbMode::Run)][size_t(ArbEvent::Reorigin)] > 0);
        CHECK(r->census().homed);
    }
    {
        auto r = homedRig();
        r->arb.estop(true);
        r->run(1000);
        reorigin(*r, ArbMode::Estop, ArbMode::Estop);
        r->arb.estop(false);
        r->run(1000);
        CHECK(r->mode() == ArbMode::Paused);
        CHECK(g_hits[size_t(ArbMode::Paused)][size_t(ArbEvent::Reorigin)] > 0);
    }
}

// ---- home op 1 ----------------------------------------------------------------

TEST_CASE("cells: a home cycle walks every phase and lands in the layer it left") {
    NoIllegal guard;
    auto r = std::make_unique<Rig>();
    REQUIRE(r->arb.home() == HomeStart::started);
    std::vector<ArbMode> seen;
    for (int t = 0; t < 20000 && (seen.empty() || seen.back() != ArbMode::Run); ++t) {
        r->run(1000);
        if (seen.empty() || seen.back() != r->mode()) seen.push_back(r->mode());
    }
    const std::vector<ArbMode> want{ArbMode::HomeApproach, ArbMode::HomeClear, ArbMode::HomeTouch,
                                    ArbMode::HomeApproach, ArbMode::HomeClear, ArbMode::HomeTouch,
                                    ArbMode::HomeFinish,   ArbMode::Run};
    CHECK(seen == want);
    const MotionCensus c = r->census();
    CHECK(c.homed);
    CHECK(c.homes == 1);
    CHECK_FALSE(c.homing);
    CHECK(g_hits[size_t(ArbMode::Run)][size_t(ArbEvent::HomeRequest)] > 0);
    CHECK(g_hits[size_t(ArbMode::HomeQueued)][size_t(ArbEvent::HomeStart)] > 0);
}

TEST_CASE("cells: a cycle under PAUSE, and a request queued from the jog") {
    NoIllegal guard;
    {
        auto r = std::make_unique<Rig>();
        r->arb.pause(true);
        r->run(1000);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(1000);
        CHECK(g_hits[size_t(ArbMode::Paused)][size_t(ArbEvent::HomeRequest)] > 0);
        CHECK(arbSeeking(r->mode()));
        r->run(20'000'000);
        CHECK(r->mode() == ArbMode::Paused);   // the layer it left
        CHECK(r->census().homed);
    }
    {
        // The hub refuses home under override; the arbiter queues it.
        auto r = std::make_unique<Rig>();
        r->arb.override();
        r->run(1000);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(1000);
        CHECK(g_hits[size_t(ArbMode::Jog)][size_t(ArbEvent::HomeRequest)] > 0);
        r->arb.estop(true);
        r->run(1000);
    }
}

TEST_CASE("cells: a queued request withdrawn, aborted, refused in motion, a window edit held through its cycle") {
    NoIllegal guard;
    {
        auto r = std::make_unique<Rig>();
        r->sense.probe_as = HomeSense::Probe::high;
        inQueued(*r, [] {});
        REQUIRE_FALSE(r->census().homing);
        r->run(1000);
        CHECK(r->mode() == ArbMode::Run);
        CHECK(g_hits[size_t(ArbMode::HomeQueued)][size_t(ArbEvent::HomeCancel)] > 0);
    }
    {
        auto r = std::make_unique<Rig>();
        inQueued(*r, [&r] {
            r->arb.pause(true);
            r->run(1000);
            CHECK(r->mode() == ArbMode::Paused);
        });
        CHECK(g_hits[size_t(ArbMode::HomeQueued)][size_t(ArbEvent::HomeAbort)] > 0);
    }
    for (const WindowPost post : kPosts) {
        INFO("post " << int(post));
        auto r = std::make_unique<Rig>();
        inQueued(*r, [&r, post] {
            r->arb.setWindow(0.0f, 300.0f, 300.0f, post);
            r->run(1000);
            CHECK(r->mode() == ArbMode::HomeQueued);
        });
        CHECK(g_hits[size_t(ArbMode::HomeQueued)][size_t(windowEvent(post))] > 0);
        r->run(20'000'000);
        CHECK(r->mode() == ArbMode::Run);
        CHECK(r->census().homed);
        CHECK(r->census().window_held == 1);
        CHECK(r->census().window_edits == 1);
        CHECK(r->arb.winMax() == doctest::Approx(300.0f));
    }
    {
        auto r = homedRig();
        REQUIRE(r->submit(MotionSource::Stream, 200.0f));
        r->run(50'000);
        REQUIRE(r->census().busy);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(1000);
        CHECK(r->mode() == ArbMode::Run);
        CHECK(g_hits[size_t(ArbMode::HomeQueued)][size_t(ArbEvent::HomeMoving)] > 0);
        CHECK(r->census().home_fails == 1);
    }
}

TEST_CASE("cells: a pause ends a cycle in every phase; a window edit is held to its end, the measured rail kept") {
    for (ArbMode m : kSeekModes) {
        NoIllegal guard;
        INFO(valence::kArbModeNames[size_t(m)]);
        auto a = seekRig(m);
        a->arb.pause(true);
        a->run(1000);
        CHECK(a->mode() == ArbMode::Paused);
        CHECK(g_hits[size_t(m)][size_t(ArbEvent::HomeAbort)] > 0);
        for (const WindowPost post : kPosts) {
            INFO("post " << int(post));
            auto w = seekRig(m);
            const float lo = w->arb.winMin(), hi = w->arb.winMax();
            w->arb.setWindow(5.0f, 300.0f, 300.0f, post);
            w->run(1000);
            CHECK(g_hits[size_t(m)][size_t(windowEvent(post))] > 0);
            CHECK(w->census().homing);
            CHECK(w->arb.winMin() == lo);   // not taken while the cycle runs
            CHECK(w->arb.winMax() == hi);
            w->run(20'000'000);
            const MotionCensus c = w->census();
            CHECK(w->mode() == ArbMode::Run);
            CHECK(c.homed);
            CHECK(c.window_held == 1);
            CHECK(c.window_edits == 1);
            CHECK(w->arb.winMin() == doctest::Approx(5.0f));
            // The post predates the cycle's end: its rail is the measured one.
            CHECK(w->arb.rail() == doctest::Approx(c.home_rail_mm));
        }
    }
}

TEST_CASE("cells: a cycle fails in every phase and lands unhomed in the layer") {
    auto fail = [](const std::function<void(Rig&)>& setup, ArbMode at, const char* why) {
        NoIllegal guard;
        INFO(why);
        auto r = std::make_unique<Rig>();
        setup(*r);
        REQUIRE(r->arb.home() == HomeStart::started);
        r->run(20'000'000);
        const MotionCensus c = r->census();
        CHECK(r->mode() == ArbMode::Run);
        CHECK_FALSE(c.homed);
        CHECK(c.home_fails == 1);
        REQUIRE(c.home_fail_why != nullptr);
        CHECK(std::string(c.home_fail_why) == why);
        CHECK(g_hits[size_t(at)][size_t(ArbEvent::HomeFail)] > 0);
    };
    fail([](Rig& r) {
        r.sense.lo_mm = -1e6f;
        r.arb.setWindow(0.0f, 40.0f, 40.0f);
        r.run(1000);   // taken before the request, which a frame move would end
    }, ArbMode::HomeApproach, "no stall across max_rail");
    fail([](Rig& r) { r.sense.sticky_lo = true; }, ArbMode::HomeClear,
         "still on the stop after the backoff (lower home_speed)");
    fail([](Rig& r) { r.sense.hi_mm = 5.0f; }, ArbMode::HomeTouch,
         "the usable rail between the safety margins is under the rail floor");
    fail([](Rig& r) { r.sense.hi_sticky_from = 2; }, ArbMode::HomeFinish,
         "still on the far stop after the final backoff (lower home_speed)");
}

// ---- who may feed ---------------------------------------------------------------

TEST_CASE("feed: every source in every mode is refused or fed as the feed column says") {
    NoIllegal guard;
    constexpr std::array<MotionSource, 4> kSources{MotionSource::Manual, MotionSource::Stream,
                                                   MotionSource::Pattern, MotionSource::Advanced};
    auto expect = [](ArbMode m, size_t s) {
        const ArbFeed f = valence::kArbFeed[size_t(m)][s];
        return f == ArbFeed::feed || f == ArbFeed::jog;
    };
    auto probe = [&](const std::function<std::unique_ptr<Rig>()>& make, ArbMode m) {
        for (size_t s = 0; s < kSources.size(); ++s) {
            INFO(valence::kArbModeNames[size_t(m)] << " source " << s);
            auto r = make();
            REQUIRE(r->mode() == m);
            CHECK(r->submit(kSources[s], 120.0f) == expect(m, s));
        }
    };
    probe([] { return homedRig(); }, ArbMode::Run);
    probe([] {
        auto r = homedRig();
        r->arb.pause(true);
        r->run(1000);
        return r;
    }, ArbMode::Paused);
    probe([] { return jogRig(); }, ArbMode::Jog);
    probe([] { return returningRig(); }, ArbMode::Returning);
    probe([] { return seekRig(ArbMode::HomeApproach); }, ArbMode::HomeApproach);
    probe([] {
        auto r = std::make_unique<Rig>(false);
        r->arb.forceHome(400.0f);
        r->run(1000);
        return r;
    }, ArbMode::Unpowered);
    probe([] {
        auto r = homedRig();
        r->arb.estop(true);
        r->run(1000);
        return r;
    }, ArbMode::Estop);
    for (size_t s = 0; s < kSources.size(); ++s) {
        auto r = homedRig();
        inQueued(*r, [&] { CHECK(r->submit(kSources[s], 120.0f) == expect(ArbMode::HomeQueued, s)); });
    }
    // A latch posted after the planner's last tick gates the next intent at once.
    auto r = homedRig();
    r->arb.pause(true);
    CHECK_FALSE(r->submit(MotionSource::Stream, 120.0f));
    r->arb.pause(false);
    CHECK(r->submit(MotionSource::Stream, 120.0f));
}

TEST_CASE("rail word: acquire, release and the e-stop's close, cell by cell") {
    NoIllegal guard;
    auto r = homedRig();
    MotionArbiter& a = r->arb;
    a.setEstopCutsPower(false);   // a halt keeps home, so the generators gate on the rail alone
    CHECK_FALSE(a.acquireRail(MotionSource::Stream));   // not a generator
    CHECK(a.acquireRail(MotionSource::Pattern));        // free -> Pattern
    CHECK(a.acquireRail(MotionSource::Pattern));        // its own
    CHECK_FALSE(a.acquireRail(MotionSource::Advanced)); // the other holds it
    CHECK_FALSE(r->submit(MotionSource::Advanced, 100.0f));
    a.releaseRail(MotionSource::Advanced);              // not the holder: unchanged
    CHECK_FALSE(a.acquireRail(MotionSource::Advanced));
    a.releaseRail(MotionSource::Pattern);               // the holder: free
    CHECK(a.acquireRail(MotionSource::Advanced));
    a.estop(true);                                      // any -> closed
    a.estop(false);
    a.pause(false);
    r->run(1000);
    CHECK(r->mode() == ArbMode::Run);
    CHECK_FALSE(r->submit(MotionSource::Advanced, 100.0f));   // closed refuses both
    CHECK_FALSE(r->submit(MotionSource::Pattern, 100.0f));
    CHECK(r->submit(MotionSource::Stream, 100.0f));           // and nothing else
    CHECK(a.acquireRail(MotionSource::Pattern));              // closed -> Pattern
    CHECK(r->submit(MotionSource::Pattern, 120.0f));
}

// ---- what the table forbids -------------------------------------------------------

TEST_CASE("illegal: a resume under override or under a RETURN is held, logged and counted") {
    {
        g_illegal.clear();
        auto r = jogRig();
        r->arb.pause(false);   // the hub refuses resume while override is latched
        r->run(1000);
        CHECK(r->mode() == ArbMode::Jog);
        REQUIRE(g_illegal.size() == 1);
        CHECK(g_illegal[0] == std::make_pair(ArbMode::Jog, ArbEvent::Resume));
        CHECK(r->census().arb_illegal >= 1);
        CHECK_FALSE(r->submit(MotionSource::Stream, 100.0f));   // held: the jog's feed
        g_illegal.clear();
    }
    {
        g_illegal.clear();
        auto r = returningRig();
        r->arb.pause(false);
        r->run(1000);
        CHECK(r->mode() == ArbMode::Returning);
        REQUIRE_FALSE(g_illegal.empty());
        CHECK(g_illegal[0] == std::make_pair(ArbMode::Returning, ArbEvent::Resume));
        g_illegal.clear();
    }
}

TEST_CASE("illegal: a home request during a RETURN is refused and reported, the return runs on") {
    g_illegal.clear();
    auto r = returningRig();
    REQUIRE(r->arb.home() == HomeStart::started);   // the hub refuses it while a return is pending
    r->run(1000);
    CHECK(r->mode() == ArbMode::Returning);
    REQUIRE(g_illegal.size() == 1);
    CHECK(g_illegal[0] == std::make_pair(ArbMode::Returning, ArbEvent::HomeMoving));
    CHECK_FALSE(r->census().homing);   // the request dropped, nothing steered
    g_illegal.clear();
    r->run(5'000'000);
    CHECK(r->mode() == ArbMode::Paused);
    CHECK(r->census().returns == 1);
    CHECK(g_illegal.empty());
}

// ---- the log and the census -----------------------------------------------------

TEST_CASE("census: a storm of transitions is counted, every one") {
    NoIllegal guard;
    auto r = homedRig();
    const uint32_t before = r->census().arb_transitions;
    for (int i = 0; i < 100; ++i) {
        r->arb.pause(true);
        r->run(1000);
        r->arb.pause(false);
        r->run(1000);
    }
    CHECK(r->census().arb_transitions - before == 200);
    CHECK(r->census().arb_illegal == 0);
}

// ---- coverage: keep last ------------------------------------------------------------

TEST_CASE("coverage: every legal cell but the race-only ones was reached above") {
    auto raceOnly = [](size_t m, size_t e) {
        for (const auto& c : kRaceOnly)
            if (size_t(c.first) == m && size_t(c.second) == e) return true;
        return false;
    };
    int legal = 0, reached = 0;
    for (size_t m = 0; m < kArbModes; ++m)
        for (size_t e = 0; e < kArbEvents; ++e) {
            const bool is_legal = valence::arbCell(ArbMode(m), ArbEvent(e)).act != ArbAct::illegal;
            if (raceOnly(m, e)) {
                CHECK(is_legal);
                continue;
            }
            if (!is_legal) continue;
            ++legal;
            INFO(valence::kArbModeNames[m] << " x " << valence::kArbEventNames[e]);
            CHECK(g_hits[m][e] > 0);
            if (g_hits[m][e] > 0) ++reached;
        }
    CHECK(reached == legal);
    MESSAGE("legal cells reached at runtime: " << reached << " of " << legal << ", race-only " << kRaceOnly.size());
}
