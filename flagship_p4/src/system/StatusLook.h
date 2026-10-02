#pragma once

// StatusLook -- what the status pixel says about the machine: the machine's
// state from its facts, and the Flux pair (system color, status effect) each
// state shows
// Constraints:
// - Hardware-free. ValenceGlow.cpp gathers the facts and feeds the pair to
//   the Flux engine; suite test_status_look drives this file.
// - The table names a Flux (System, Status) PAIR, never a color: color and
//   effect belong to the fleet-wide two-axis grammar (logging-leds.md), so
//   amber is Motion and red is Safety on every board. Edit a row to change
//   what a state shows; edit flux_core.hpp only to change the grammar.
// - The glue owns the Motion, Safety and Flash systems; every other system
//   belongs to the module that speaks it.
// See: .claude/rules/logging-leds.md, lib/flux/include/flux/flux_core.hpp,
// bd val-091.27

#include <array>
#include <cstddef>
#include <cstdint>

#include "flux/flux_core.hpp"

namespace valence::look {

enum class MachineState : uint8_t {
    boot,      // the self-check has not run: the boot rainbow
    idle,      // up, not homed
    homed,     // homed, at rest
    running,   // a plan is executing
    paused,    // SPEC 11.1 PAUSE latched
    estop,     // the e-stop latch is set
    fault,     // the self-check held motor power off, or the motor switch faulted
    flashing,  // an OTA image is being written: do not power off
    kCount_,
};

// ---- THE TABLE: what each state shows ---------------------------------------

struct LookRow {
    MachineState state;
    flux::System system;
    flux::Status status;
};

using flux::Status;
using flux::System;

inline constexpr std::array<LookRow, size_t(MachineState::kCount_)> kLookTable{{
    //  state                    system           status
    {MachineState::boot,      System::Motion,  Status::Nominal},   // rainbow; the pair is unused
    {MachineState::idle,      System::Motion,  Status::Latched},   // solid amber: home me
    {MachineState::homed,     System::Motion,  Status::Nominal},   // green slow breathe: all quiet
    {MachineState::running,   System::Motion,  Status::Working},   // amber fast breathe
    {MachineState::paused,    System::Motion,  Status::Degraded},  // amber slow blink
    {MachineState::estop,     System::Safety,  Status::Urgent},    // red fast blink
    {MachineState::fault,     System::Safety,  Status::Latched},   // solid red
    {MachineState::flashing,  System::Flash,   Status::Working},   // white, twice the tempo
}};

constexpr const LookRow& lookFor(MachineState s) { return kLookTable[size_t(s)]; }

// ---- the state from the facts -----------------------------------------------

struct Facts {
    bool checked        = false;  // the boot self-check has run
    bool check_passed   = false;  // ...and allowed motor power
    bool switch_faulted = false;  // the motor switch is latched faulted
    bool estop          = false;
    bool paused         = false;
    bool busy           = false;  // a plan is executing
    bool homed          = false;
    bool flashing       = false;  // OTA write in flight
};

// First match wins, in this order. E-stop outranks everything after boot;
// a fault outranks the motion states because motion cannot run under one.
constexpr MachineState stateFor(const Facts& f) {
    if (!f.checked) return MachineState::boot;
    if (f.estop) return MachineState::estop;
    if (f.flashing) return MachineState::flashing;
    if (!f.check_passed || f.switch_faulted) return MachineState::fault;
    if (f.paused) return MachineState::paused;
    if (f.busy) return MachineState::running;
    return f.homed ? MachineState::homed : MachineState::idle;
}

}  // namespace valence::look
