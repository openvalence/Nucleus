---
paths:
  - "**"
---

# Motion control constraints

Architecture-level motion doctrine (one command one plan, the MotionArbiter
sole-caller rule, the two-chip split, HP-evaluates / LP-renders) lives in
`.claude/rules/architecture.md` §2. This file is the mechanism layer.

## Kinetic (`../Kinetic`, pinned by `kinetic.pin`)

**The planner is Kinetic² (operator ruling 2026-10-06, bd val-z1k).**
`kinetic2::Engine<1, 64>` inside MotionArbiter is the one planner of every
Nucleus build: the P4 image, valencesim, kinetic-wasm and the native suites.
The Kinetic repo's `include/kinetic/` engine and its `third_party/` are that
repo's own test oracle and are never compiled here.

Every intent becomes knots at one boundary (MotionArbiter.cpp, "the Kinetic²
boundary"): a segment is a knot at its start plus its duration, a gap before
its start a rest knot; a sample a knot at its arrival plus the grant's
`schedule_latency_us` (`sampleLatencyUs()`); a jog, a return or a homing
backoff a knot at the park time from the newest knot (RFC-105 (k) and (n)),
at an authored rest (a free last knot keeps its secant and is braked past,
RFC-105 (dd)), stretched, never trimmed; a homing seek two knots at the leg's
speed, a ramp-up at the jog accel and a knot the brake past lands on the
search end, so it cruises into contact; a generator's stop and PAUSE are the
arbiter's brake, which refuses a knot due before its end; a timeline that runs
dry still moving is the engine's own brake, which a new knot re-plans from
(RFC-105 (dd)). Kinetic² has no chase heuristics and no dwell rule; a point
move is a quintic, up to 1.875 times a cruise (`kPointMoveSlowdown`, which the
home deadline carries for the backoffs).

Event-driven, never clocked: an intent becomes knots at arrival, the pending
window is solved at the next sample, and the sampler evaluates it.

- **Map:** header-only, hardware-free `kinetic2::Engine`: a knot timeline (64
  knots per axis here), a window solver that re-plans every pending knot
  together, and a brake. The library is the sibling Kinetic checkout,
  consumed in place (PlatformIO `symlink://`, CMake by path), never copied
  here; canon_lint fails when its HEAD is not `kinetic.pin` or its consumed
  paths are dirty. An engine change lands in Kinetic first, then the operator
  moves the pin. A namespace rename is gated on the native suite, never a
  blind sed pass.
- **Deadlines and amplitude.** A segment's knot holds its commanded duration
  under Blend by trimming the stroke, never below `amplitude_floor` (the
  0x1122 `amplitude_budget`); Stretch keeps the stroke and moves the knot
  later. Amplitude is the one quantity a ceiling may shape; this is the
  operator-ratified exception (2026-09-02) to "ceilings are clamps, never
  targets". A sample is never trimmed, only stretched. A Manual move always
  plans under Stretch: a jog that lands short is a wrong answer.
- **One activity clock (operator ruling 2026-09-02).** Every "is the stream
  alive" question keys on ONE reference in the engine's own clock: under
  Kinetic² the timeline itself, alive while a knot is pending. Two mechanisms
  answering the same question from different references is the defect class
  that produced every field hitch of 2026-09-02.
- **Safety:** the arbiter clamps every target to the window before it becomes
  a knot; the solver's referee scores every extremum of a piece against the
  ceilings and the window and never accepts an excursion; authored velocities
  are bounded (EndVelClamped); the census output is window-clamped.
  Exceptions are never instantiated; a non-finite knot is refused at
  `submit()`.
- **Whichever task samples the engine first after a submit needs a deep
  stack**: the window solve copies the pending knots and runs the solver's
  fixed arrays on the calling stack, KB-scale. That is the motion task only.
  Its size is not yet measured under Kinetic² (bd val-4q1); never size it
  down without a measured high-water mark under a real motion workload (T1
  class, `memory-budget.md` T21). That stack is HP-side and internal RAM only
  (`governance.md` §6).

## The LP-core emitter (`flagship_p4/ulp/`)

The LP core renders edges and does nothing else. Its signed edge count IS the
machine's position. All numbers below are measured on this stamp
[verified 2026-09-20 -- Rigol DHO4204, single-shot captures at 8 ns and 100 ns
per sample, decoded in numpy; board `val-091.3`].

- **Phase accumulator in exact fixed-point cycles, and the next deadline is
  computed from the PREVIOUS DEADLINE, never from the instant an edge was
  actually emitted.** Period-in-poll-ticks accumulates phase error and renders
  as a low-frequency staircase. This is a design rule, not a tuning knob, and
  it is now a hardware result: at a deliberately hostile rate (4003
  cycles/edge, prime) intervals take exactly TWO values, 100.000 and
  100.128 us, and the mean is +30 ppm -- the same crystal offset as a friendly
  rate. **The quantization does not accumulate.**
- **The floor is 125 ns and it is the poll loop, not the clock.** The spin
  loop (`csrr mcycle` / `sub` / `bltz`) is 5 CYCLES = 125 ns at 40 MHz, and
  every edge lands on that grid: peak-to-peak spread 128 ns, 16 scope samples.
  A rate that is a multiple of the loop period hides this by landing every
  edge at the same phase; 4000 cycles/edge did exactly that. **Benchmark the
  emitter at a prime rate or the number is a lie.**
- **Clock source is the XTAL, never the RC oscillator.** `RTC_FAST` defaults
  to the internal RC: MEASURED 16.60 MHz on this stamp, part-to-part variable
  and temperature-dependent. The XTAL measured 40.00 MHz exactly (50,000 edges
  per 5 s at 4000 cycles/edge, `late=0`). An emitter scheduling in `mcycle`
  needs a crystal-derived clock. The config home is
  `flagship_p4/sdkconfig.defaults`.
- **One store per edge, with nothing between the load and the store.** DIG-694
  is a silicon erratum requiring at least two instructions between a load and
  the store of that register. The emitter's inner shape (`lw` then `sw`, zero
  instructions between) ran 2,156 scoped edges with no corruption at 10 kHz.
  That is EVIDENCE, NOT PROOF: if the inner loop changes shape, re-scope it.
  The whole-build half of this fact is the `-O2` requirement in
  `build-test-deploy.md`.
- **An HP flash write does not touch it.** The LP core runs from LP SRAM with
  its GPIO in the LP domain [verified 2026-09-20 -- HP-side hammer erasing and
  writing 4 KB every 100 ms, ~1.0 s of cache-off in every 5 s window: `late=0`,
  `f_LP` unchanged, 0 illegal transitions, 0 outliers beyond the poll grid].
  Never move edge rendering back to an HP-resident renderer.
- **Budget, so a rate change is checked and not guessed:** at 40 MHz and
  52.152 steps/mm, 950 mm/s is 807 cycles per edge against a 5-cycle poll
  loop. The LP core has 32 KB of LP SRAM and reaches HP SRAM and peripherals
  while the system is active.
- **A reversal must land while the output line is IDLE.** Quadrature has no
  separate direction line, so the class of bug where a direction flip rewrites
  the widest pulse of a move is gone BY CONSTRUCTION rather than by tuning.
  Any future renderer that adds a direction line brings the whole class back.

## ISR / IRAM / core discipline

- Motion code on the HP side uses microcritical sections, not ISRs: short
  float math only, no heap allocation, no ISR context.
- The HP-to-LP channel is the LP shared memory window. It carries a velocity
  and flags, single-writer per field, never a rendered buffer. A renderer that
  buffers ahead turns every late refill into dead air on the output, which is
  the reason edge rendering lives on a core with nothing else to do.
- Task and core assignment is in `.claude/rules/subsystem-map.md` (C-1).
