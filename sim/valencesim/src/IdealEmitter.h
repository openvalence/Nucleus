#pragma once

// IdealEmitter -- the LP core's contract with nothing between the word and the
// edge: the steering word's period, rendered from the moment it was written
// Constraints:
// - HOST ONLY. The board's emitter is LpEmitter (ValenceMotion.cpp); this one
//   stands in for it wherever there is no LP core: the sim (SimMotion.cpp) and
//   the offline planner (tools/kinetic-wasm).
// - THE EMITTER IS IDEAL. Every edge on time, so late edges, re-steers and
//   catch-ups read 0. That is the honest answer for a machine with no LP core,
//   not a measurement of one.
// - The LP core's FENCE is modeled as lp_quad.c applies it: an edge past it is
//   withheld and counted. Its LEASE NEVER LAPSES here: the host's tick runs at
//   its timer's resolution (bd val-x2o), far past kLeaseUs, so renew() is
//   accepted and lapses() reads 0.
// - Single-threaded: the caller's one thread steers and advances it.
// See: flagship_p4/src/motion/MotionArbiter.h (the emitter seam)

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "motion/MotionArbiter.h"

namespace valence {

class IdealEmitter final : public MotionEmitter {
public:
    int32_t count() const override { return _count; }

    void steer(float v_mm_s) override {
        const SteerWord w = steerWord(v_mm_s);
        if (w.step_q8 == 0) {
            _step_q8 = 0;
            return;
        }
        if (w.floored) ++_faults;
        _forward = w.forward;
        _step_q8 = w.step_q8;
    }

    void park() override { _step_q8 = 0; }

    void fence(int32_t lo, int32_t hi) override {
        _fence_lo = lo;
        _fence_hi = hi;
    }

    void renew() override {}

    uint32_t lapses() const override { return 0; }

    // Renders the live word over the interval since the previous call.
    void advance(uint64_t now_us) {
        const double dt_s = double(now_us - _last_us) * 1e-6;
        _last_us = now_us;
        if (_step_q8 == 0 || !(dt_s > 0.0)) return;
        const double edges_per_s = double(kLpClockHz) * 256.0 / double(_step_q8);
        _phase += (_forward ? edges_per_s : -edges_per_s) * dt_s;
        const double whole = std::trunc(_phase);
        _phase -= whole;
        // Forward up to the fence's high word, reverse down to its low one; a
        // count outside it moves back freely.
        const int64_t want = int64_t(whole);
        int64_t got = want;
        if (want > 0) got = std::min<int64_t>(want, std::max<int64_t>(int64_t(_fence_hi) - _count, 0));
        if (want < 0) got = std::max<int64_t>(want, std::min<int64_t>(int64_t(_fence_lo) - _count, 0));
        _count += int32_t(got);
        _edges += uint32_t(got < 0 ? -got : got);
        _fence_hits += uint32_t(want > got ? want - got : got - want);
    }

    uint32_t edges() const { return _edges; }
    uint32_t fenceHits() const { return _fence_hits; }
    uint32_t stepQ8() const { return _step_q8; }
    uint32_t faults() const { return _faults; }

private:
    int32_t  _count = 0;
    double   _phase = 0.0;   // fractional edge carried between ticks
    uint64_t _last_us = 0;
    uint32_t _step_q8 = 0;
    bool     _forward = true;
    uint32_t _edges = 0;
    uint32_t _faults = 0;
    int32_t  _fence_lo = INT32_MIN;   // open until the arbiter's first write, as the LP core loads it
    int32_t  _fence_hi = INT32_MAX;
    uint32_t _fence_hits = 0;
};

}  // namespace valence
