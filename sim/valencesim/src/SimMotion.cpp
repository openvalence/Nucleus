// SimMotion -- motion/ValenceMotion.h on a desktop: the board's own
// MotionArbiter and kinetic::Engine, hosted on the sim's one thread, steering
// an ideal emitter
// Constraints:
// - SINGLE-THREADED (SimMotion.h). The P4 hands intents across a FreeRTOS
//   queue to its motion task; here submit() and simMotionTick() share one
//   thread, so the queue is a plain ring and nothing is locked.
// - NO GATE LIVES HERE. Every accept/evaluate decision is
//   flagship_p4/src/motion/MotionArbiter.cpp, compiled verbatim. This file is
//   plumbing: the ring, the tick order, and the emitter.
// - THE EMITTER IS IDEAL (IdealEmitter.h).
// See: SimMotion.h, flagship_p4/src/motion/MotionArbiter.h, bd val-sf7.2

#include "SimMotion.h"

#include <array>
#include <optional>

#include "IdealEmitter.h"
#include "hub/ValenceDevice.h"
#include "motion/MotionArbiter.h"
#include "motion/ValenceMotion.h"
#include "system/ValenceMotorSwitch.h"

namespace valence {
namespace {

class SimHost {
public:
    void begin(uint64_t now_us) {
        _emitter.advance(now_us);
        _arb.begin(now_us);
        _prev_us = now_us;
        refreshSnapshot(now_us);
    }

    bool submit(const MotionIntent& in) {
        if (_count == _queue.size()) return false;
        _queue[(_head + _count) % _queue.size()] = in;
        ++_count;
        return true;
    }

    // The P4 overwrites a depth-one queue; one pending slot is the same thing
    // on one thread.
    void setTuning(const MotionTuning& t) { _tune_pending = t; }

    // Same order as the P4's motion task: tuning, then intents, then the
    // evaluation. The emitter renders first because on the board it never
    // stops rendering between ticks.
    void tick(uint64_t now_us) {
        _emitter.advance(now_us);
        if (_tune_pending) {
            _arb.applyTuning(*_tune_pending);
            _tune_pending.reset();
        }
        while (_count > 0) {
            const MotionIntent in = _queue[_head];
            _head = (_head + 1) % _queue.size();
            --_count;
            _arb.accept(in, now_us);
        }
        const float dt_s = float(now_us - _prev_us) * 1e-6f;
        if (dt_s > 0.0f) {
            _prev_us = now_us;
            _arb.evaluate(now_us, dt_s);
        }
        _arb.drainAnomalies();
        refreshSnapshot(now_us);
    }

    MotionArbiter& arbiter() { return _arb; }
    MotionCensus census() const { return _pub; }

private:
    void refreshSnapshot(uint64_t now_us) {
        MotionCensus c = _arb.snapshot(now_us);
        c.edges          = _emitter.edges();
        c.step_q8        = _emitter.stepQ8();
        c.emitter_faults = _emitter.faults();
        _pub = c;
    }

    // Declaration order is construction order: the arbiter binds the emitter.
    IdealEmitter  _emitter;
    MotionArbiter _arb{_emitter, &deviceNowUs};

    std::array<MotionIntent, kIntentQueueDepth> _queue{};
    size_t _head = 0;
    size_t _count = 0;
    std::optional<MotionTuning> _tune_pending;
    uint64_t _prev_us = 0;
    MotionCensus _pub{};
};

// File scope, not a stack local: the arbiter holds a KB-scale engine.
SimHost g_sim;

}  // namespace

void simMotionTick(uint64_t now_us) { g_sim.tick(now_us); }

// ---- motion/ValenceMotion.h -------------------------------------------------

bool motionBegin() {
    g_sim.begin(deviceNowUs());
    return true;
}
bool motionSubmit(const MotionIntent& in) { return g_sim.submit(in); }
// The board's order: the power cut first (SimMotorSwitch.cpp), then the halt.
void motionEstop() {
    motorSwitchCut();
    g_sim.arbiter().estop(true);
}
void motionEstopClear() { g_sim.arbiter().estop(false); }
void motionSetMotorPowered(bool on) { g_sim.arbiter().setMotorPowered(on); }
void motionSetCommissioned(bool on) { g_sim.arbiter().setCommissioned(on); }
void motionPause(bool on) { g_sim.arbiter().pause(on); }
bool motionAcquireRail(MotionSource g) { return g_sim.arbiter().acquireRail(g); }
void motionReleaseRail(MotionSource g) { g_sim.arbiter().releaseRail(g); }
void motionSetEstopCutsPower(bool cuts) { g_sim.arbiter().setEstopCutsPower(cuts); }
void motionOverride() { g_sim.arbiter().override(); }
ReturnStart motionReturn() { return g_sim.arbiter().returnToPause(); }
void motionSetFlipped(bool on) { g_sim.arbiter().setFlipped(on); }
void motionSetJogLimits(float v, float a) { g_sim.arbiter().setJogLimits(v, a); }
void motionSetInputLimits(float v, float a, float j) { g_sim.arbiter().setInputLimits(v, a, j); }
void motionSetWindow(float lo, float hi, float rail) { g_sim.arbiter().setWindow(lo, hi, rail); }
void motionNoteStream(uint32_t b, uint32_t s, uint32_t d) { g_sim.arbiter().noteStream(b, s, d); }
float motionForceHome(float stroke_mm) { return g_sim.arbiter().forceHome(stroke_mm); }
MotionCensus motionCensus() { return g_sim.census(); }

void motionSetTuning(const MotionTuning& t) { g_sim.setTuning(t); }

}  // namespace valence
