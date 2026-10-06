// arbiter_rig -- the REAL MotionArbiter behind test_estop_datagram's delegate
// Constraints:
// - Its own translation unit: MotionArbiter.cpp and ValenceEstopDatagram.cpp
//   each own an anonymous-namespace kTag, so one TU cannot hold both.
// - The arbiter holds a KB-scale engine: file scope, never a stack local
//   (MotionArbiter.h). One test thread, so the owning-task methods are safe.
// See: test_main.cpp, flagship_p4/src/motion/MotionArbiter.h

// Named here so the dependency finder builds them; the .cpp below needs all three.
#include "geiger/geiger.h"
#include "kinetic2/engine.hpp"
#include "valence/generated/registry_constants.hpp"

#include "../../../flagship_p4/src/motion/MotionArbiter.cpp"

#include "arbiter_rig.h"

namespace {

uint64_t g_nowUs = 1'000'000;
uint64_t rigNowUs() { return g_nowUs; }

// Renders nothing; counts the parks the arbiter commands.
class ParkCounter final : public valence::MotionEmitter {
public:
    int32_t count() const override { return 0; }
    void steer(float) override {}
    void park() override { ++parks; }
    int parks = 0;
};

ParkCounter g_emitter;
valence::MotionArbiter g_arb{g_emitter, &rigNowUs};
bool g_begun = false;

}  // namespace

void rigArbiterReset() {
    if (!g_begun) {
        g_arb.begin(g_nowUs);
        g_arb.setMotorPowered(true);
        g_arb.setCommissioned(true);
        g_begun = true;
    }
    g_arb.estop(false);
    g_emitter.parks = 0;
}

void rigArbiterEstop(bool on) { g_arb.estop(on); }

bool rigArbiterLatched() { return g_arb.snapshot(g_nowUs).estop; }

int rigArbiterParks() { return g_emitter.parks; }
