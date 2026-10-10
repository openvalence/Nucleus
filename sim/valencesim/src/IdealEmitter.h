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
// - The LP core's FENCE and LEASE are modeled as lp_quad.c applies them: an
//   edge past the fence is withheld and counted; nothing renders before the
//   first renew(), and kLeaseUs after the last one the word is stored 0 and a
//   lapse counted. A host that advances it in steps longer than kLeaseUs
//   without a renew between sees the board's stall, not an ideal render.
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

    // At the clock of the last advance(): the caller advances before it steers.
    void renew() override {
        _renewed_us = _last_us;
        _live = true;
    }

    uint32_t lapses() const override { return _lapses; }

    // Renders the live word over the interval since the previous call, up to
    // the lease's end.
    void advance(uint64_t now_us) {
        const uint64_t from_us = _last_us;
        _last_us = now_us;
        if (!_live || now_us <= from_us) return;
        const uint64_t lease_end_us = _renewed_us + kLeaseUs;
        const bool lapse = now_us > lease_end_us;
        render(double((lapse ? std::max(lease_end_us, from_us) : now_us) - from_us) * 1e-6);
        if (lapse) {
            _live = false;
            _step_q8 = 0;
            _phase = 0.0;
            ++_lapses;
        }
    }

    uint32_t edges() const { return _edges; }
    uint32_t fenceHits() const { return _fence_hits; }
    uint32_t stepQ8() const { return _step_q8; }
    uint32_t faults() const { return _faults; }

private:
    void render(double dt_s) {
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
    uint64_t _renewed_us = 0;
    bool     _live = false;   // unleased until the first renew(), as the LP core starts
    uint32_t _lapses = 0;
};

}  // namespace valence
