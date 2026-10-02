// test_status_look -- native doctest suite for the status pixel's state table
// Constraints:
// - Hardware-free: StatusLook.h and the Flux core. What the pixel renders for
//   a pair is the Flux suite's job (test_flux); this suite pins which pair
//   each machine state picks and which state the facts pick.
// See: flagship_p4/src/system/StatusLook.h, bd val-091.27

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstddef>

// Named here so the library finder resolves lib/flux; StatusLook.h needs it.
#include "flux/flux_core.hpp"

#include "../../../flagship_p4/src/system/StatusLook.h"

namespace look = valence::look;
using look::Facts;
using look::MachineState;

namespace {

Facts readyHomed() {
    Facts f;
    f.checked = true;
    f.check_passed = true;
    f.homed = true;
    return f;
}

}  // namespace

TEST_CASE("the table has one row per state, indexed by the state") {
    for (size_t i = 0; i < look::kLookTable.size(); ++i) {
        CAPTURE(i);
        CHECK(size_t(look::kLookTable[i].state) == i);
    }
}

TEST_CASE("the pairs follow the two-axis grammar") {
    using flux::Status;
    using flux::System;
    // Unhomed is Motion/Latched, never a fault (logging-leds.md, T15).
    CHECK(look::lookFor(MachineState::idle).system == System::Motion);
    CHECK(look::lookFor(MachineState::idle).status == Status::Latched);
    // Homed at rest is the quiet floor: every owned system Nominal.
    CHECK(look::lookFor(MachineState::homed).status == Status::Nominal);
    // Red is Safety and only Safety.
    for (const auto& row : look::kLookTable) {
        const bool safety = row.state == MachineState::estop || row.state == MachineState::fault;
        CHECK((row.system == System::Safety) == safety);
    }
    // The e-stop outranks every other state in Flux's own arbiter.
    for (const auto& row : look::kLookTable)
        if (row.state != MachineState::estop)
            CHECK(uint8_t(row.status) < uint8_t(look::lookFor(MachineState::estop).status));
}

TEST_CASE("the facts pick the state, first match wins") {
    Facts f;
    CHECK(look::stateFor(f) == MachineState::boot);
    f.estop = true;
    CHECK(look::stateFor(f) == MachineState::boot);   // nothing outranks boot

    f = readyHomed();
    CHECK(look::stateFor(f) == MachineState::homed);
    f.homed = false;
    CHECK(look::stateFor(f) == MachineState::idle);
    f.busy = true;
    CHECK(look::stateFor(f) == MachineState::running);
    f.paused = true;
    CHECK(look::stateFor(f) == MachineState::paused);
    f.switch_faulted = true;
    CHECK(look::stateFor(f) == MachineState::fault);
    f.flashing = true;
    CHECK(look::stateFor(f) == MachineState::flashing);
    f.estop = true;
    CHECK(look::stateFor(f) == MachineState::estop);
}

TEST_CASE("a self-check that held motor power off is a fault, homed or not") {
    Facts f = readyHomed();
    f.check_passed = false;
    CHECK(look::stateFor(f) == MachineState::fault);
}

TEST_CASE("the pixel shows the table's pair through the Flux engine") {
    flux::NullGlowOutput out;
    flux::GlowEngine e(out);
    const auto& row = look::lookFor(MachineState::estop);
    e.set(row.system, row.status);
    e.update(0);
    CHECK(e.currentSystem() == flux::System::Safety);
    CHECK(e.currentStatus() == flux::Status::Urgent);
}
