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

Every intent becomes knots at one boundary (MotionArbiter.cpp, "the Kinetic²
boundary"): a segment is a knot at its start plus its duration, a gap before
its start a rest knot, and a segments bundle's first segment first replaces
every knot queued at or after its start, the motion in flight handing off
there (RFC-087 supersede, `Engine::truncateAfter`); a sample a knot at its arrival plus the grant's
`schedule_latency_us` (`sampleLatencyUs()`); a jog or a return a knot at the
park time from the newest knot (RFC-105 (k) and (n)), at an authored rest (a
free last knot keeps its secant and is braked past, RFC-105 (dd)), stretched,
never trimmed; a generator's stop and PAUSE are the arbiter's brake, which
refuses a knot due before its end; a timeline that runs dry still moving is
the engine's own brake, which a new knot re-plans from (RFC-105 (dd)).
A Stream segment arms the engine's expectation (`Engine::expect`) until its
arrival plus the hub's quiet window, the larger of `stream_quiet_release_ms`
and the grant's schedule horizon (`MotionIntent::expect_us`), so its free knot
renders through toward a provisional successor instead of at rest; every
other intent, a brake and a reset end it, and the hub's quiet release
(`releaseRail(Stream)`) ends it and re-solves the knots still pending, so the
newest lands at rest. A stream that stops before its release passes its last
free knot moving and the engine's brake stops past it, by the brake distance
from the chord speed (bd val-g62) [verified 2026-10-08 -- test_motion_arbiter,
free knots 125 ms ahead pass at 0.99..1.02 of the chord, 0.90..1.22 without;
a release with the last knot pending rests on it within 0.05 mm].
Kinetic² has no chase heuristics and no dwell rule; a point move is a
quintic, up to 1.875 times a cruise.

**Homing never reaches the planner (operator ruling 2026-10-06, bd val-cp5).**
Home op 1 is the arbiter's own second producer, the seek producer
(MotionArbiter.cpp, homing): every leg steers the emitter directly at a
constant velocity, no accel, decel or jerk profile (`kHomeRampMs`, a build
constant, 0 by default, gives a drive that faults on a step-rate step a
linear ramp without a wire change). An approach or a re-touch ends on the
home sense's rise, which parks the emitter from the interrupt
(`MotionArbiter::homeSenseRose()`); a backoff ends on a distance counted on
the emitter's step count, its last tick rendering the remainder so it lands
on the count. The plan-tracking feedforward, its velocity cap and its
residual kick never steer during a cycle; the engine is reset at the count
when the cycle ends, as after an e-stop. Exclusivity is the rail: a cycle
owns it from the request to its end, `accept()` refuses every intent
meanwhile, and one producer steers per tick: the seek producer runs on the
planner and renews the lease there, and `steerTick()` stands aside while a
leg is in flight. **The safety margin
(`kHomeSafetyMarginMm`, 5 mm) is never a commandable position**: 0.0 mm is a
margin off the low stop's datum, the rail ends a margin short of the high
one, so a 300 mm stop-to-stop rail is a 290 mm usable rail, and the usable
rail is what max_rail stores.

Event-driven, never clocked: an intent becomes knots at arrival, the pending
window is solved at the next sample, and the sampler evaluates it.

- **The planner and the steer (bd val-8rt).** Two tasks on HP core 1
  (`subsystem-map.md`). The planner (priority 5) owns the engine: it drains
  the intents, samples the engine once per tick (`MotionArbiter::planTick()`)
  and publishes THE STRIP, the plan's position in mm every `kMotionTickUs`
  for `kStripLen` (128) entries, read with `kinetic2::Engine::peek()`, which
  returns bit for bit what the sampler returns later. A plan change (a
  submit, a brake, a reset) refills it whole; a tick without one shifts it
  and appends the new tail. The steer (priority 6, `MotionArbiter::steerTick()`)
  copies the strip under the host's lock every tick, interpolates it at now
  and steers, so a window solve of up to 128 ms on the planner never delays
  a steer. Past the strip's end the last entry holds, the carriage stops
  there, and the steer counts a planner stall (`plannerStalls()`, logged
  once a second). An engine reset carries a new generation: the steer steers
  nothing from an older strip and re-anchors its feedforward at the reset's
  position. A new plan landing off the strip the steer was rendering (a
  solve longer than `react_us`) moves the previous command by the gap, so
  the gap closes at the bounded kick, never as a burst (`latePlans()`,
  counted past one step, logged past ten). The first steer after a tick
  the steer held (an e-stop, power, a frame move, no strip) starts its
  feedforward from the plan itself.
  `evaluate()` is both halves on one task: the sim, kinetic-wasm and the
  native suites [verified 2026-10-08 -- test_motion_arbiter, the strip cases
  driving planTick() and steerTick() apart; test_kinetic_wasm_trace
  re-recorded its 60 s fixture byte-identical through evaluate()].

- **The oscillator (RFC-103, bd val-dzf).** Kinetic²'s stage
  (`kinetic2/oscillator.hpp`) is summed into the strip by `fillStrip()`, over
  the plan read `kOscLook` ticks past the strip's end while it renders, so the
  steer follows it as it follows any plan. Its amplitude yields first under
  the input set and never leaves the window, by an envelope smoothed over one
  fade and fitted to the plan a fade ahead; a plan changed with less notice
  than that can exceed a ceiling until the fade ends. It renders nothing
  unhomed or uncommissioned; PAUSE cuts it at once, the strip carrying `cut`
  so the kick closes the gap and no late plan is counted; an engine reset
  stops it; a window move rescales it, never a step. Its parameters cross from
  the hub task as a seqlocked post (`setOscillator()`), and it costs 5,176 B of
  the arbiter's internal RAM [verified 2026-10-09 -- test_motion_arbiter, the
  oscillator cases: the strip under 300 mm strokes at the input set keeps
  every ceiling and the window, sheds to under a step at full speed].
- **Map:** header-only, hardware-free `kinetic2::Engine`: a knot timeline (64
  knots per axis here), a window solver that re-plans every pending knot
  together, and a brake. The library is the sibling Kinetic checkout,
  consumed in place (PlatformIO `symlink://`, CMake by path), never copied
  here; canon_lint fails when its HEAD is not `kinetic.pin` or its consumed
  paths are dirty. An engine change lands in Kinetic first, then the operator
  moves the pin. A namespace rename is gated on the native suite, never a
  blind sed pass.
- **Deadlines and amplitude.** A segment's knot always holds its commanded
  time: the renderer fits the ceilings by shortening the knot's handles,
  never below `handle_floor` of the piece, then by trimming the knot toward
  its predecessor, never farther than `trim_max` of the window span; a piece
  no trim within that makes legal renders over the ceiling at its least-over
  trim (PieceOverCeiling). Free knots render by `smoothness` (0x1122, Kinetic
  `handles.hpp`). Amplitude is the one quantity a ceiling may shape; this is
  the operator-ratified exception (2026-09-02) to "ceilings are clamps, never
  targets". A run of samples renders as one fastest legal move to rest on the
  newest, stretched, never trimmed; a run whose move leaves the window renders
  as knots, which trim (Kinetic `solver.hpp` chaseRun). A jog is a HARD knot,
  the fastest move to rest, stretched, never trimmed: a jog that lands short
  is a wrong answer.
- **One activity clock (operator ruling 2026-09-02).** Every "is the stream
  alive" question keys on ONE reference in the engine's own clock: under
  Kinetic² the timeline itself, alive while a knot is pending. Two mechanisms
  answering the same question from different references is the defect class
  that produced every field hitch of 2026-09-02.
- **Safety:** the arbiter clamps every target to the window before it becomes
  a knot; the solver's referee scores every extremum of a piece against the
  ceilings and the window and never accepts an excursion; authored velocities
  are bounded (EndVelClamped). Exceptions are never instantiated; a
  non-finite knot is refused at `submit()`.
- **The position backstop (operator ruling 2026-10-06, bd val-1w8): no plan
  moves the carriage out of its frame.** The brakes (PAUSE, a generator's
  stop, a starved timeline) are profiles the referee never scores, so
  `MotionArbiter::steerTick()` clamps the rendered demand on every tick to the
  backstop's frame: the configured window held inside the rail, or the
  asserted rail while the engine plans in the rail frame (override's jog and
  its return). A home cycle's seek producer never reaches it. The frame
  widens only to the previous demand, so a carriage left outside it is moved
  back by a plan, never by the clamp. Into a held edge the feedforward
  carries the distance to the edge and nothing past it, and every steer is
  bounded from position truth so it carries the carriage at most to the edge
  within `kTickDtCapS`. A held plan is a hard stop at the edge: a brake that
  would have overrun the window ends there abruptly. The census counts
  engagements (`backstops`, a plan more than one step past, at onset) and
  sets plan.flags `clamped` while one holds
  [verified 2026-10-06 -- test_motion_arbiter, a PAUSE brake planned 65 mm
  past the window held at its edge]. This clamp is the FIRST line; the LP
  core's fence (below) is the second, and the only one an HP stall cannot
  skip.
- **A late tick is a stall, never a burst (bd val-1w8).** Every steer is
  priced over at most `kTickDtCapS`, two `kMotionTickUs`, so none exceeds the
  vmax of the set the plan in flight was planned under (the jog set's for a
  Manual plan and the RETURN, the input set's otherwise) plus that long of
  the input amax; a tick later than that is re-anchored to the plan, steers
  the plan's own velocity, and is counted (census `stalls`, on the status
  line). Between ticks the LP core renders the last steer open loop, so a
  steer-task stall (a flash write with the cache off; never the window
  solve, which runs on the planner) carries the carriage that steer for at
  most `kLeaseUs`, then the LP core stops it (the lease, below), and never
  past the frame's edge (the fence). The next tick counts the lapse (census
  `lease_lapses`, `lapses=` on the status line) and never re-anchors
  `_p_cmd_mm` to the count: a stall between the clock read and the steer
  reaches `steerTick()` with a `dt_s` measured before it, and
  the residual as feedforward over that is an uncommanded reversal (measured
  -600 mm/s against a 223 mm/s plan). The residual closes at the bounded kick
  [verified 2026-10-06 -- test_motion_arbiter, a 283 ms stall moves the
  carriage 0.896 mm at 223.5 mm/s, then steers 233 mm/s].
- **Whichever task samples the engine first after a submit needs a deep
  stack**: the window solve copies the pending knots and runs the solver's
  fixed arrays on the calling stack, KB-scale. That is the planner task only.
  Its size is not yet measured under Kinetic² (bd val-4q1); never size it
  down without a measured high-water mark under a real motion workload (T1
  class, `memory-budget.md` T21). That stack is HP-side and internal RAM only
  (`governance.md` §6).

## The LP-core emitter (`flagship_p4/ulp/`)

The LP core renders edges inside its fence while its lease is fresh, and
nothing else (amended by operator ruling 2026-10-06, bd val-fi5: "there should
be absolutely no possible way for the machine to leave the specified window
or end of the rail under any circumstance", and "the LP core gets two bounds
and nothing else"). Its signed edge count IS the machine's position. The
fence and the lease are below; every other number in this section is
measured on this stamp
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
- **An HP flash write does not touch its timing.** The LP core runs from LP
  SRAM with its GPIO in the LP domain [verified 2026-09-20 -- HP-side hammer
  erasing and writing 4 KB every 100 ms, ~1.0 s of cache-off in every 5 s
  window: `late=0`, `f_LP` unchanged, 0 illegal transitions, 0 outliers
  beyond the poll grid]. Never move edge rendering back to an HP-resident
  renderer. A cache-off window longer than `kLeaseUs` during motion now stops
  the carriage by design: the steer task cannot renew the lease through it.
  So a settings write waits for a still window: the published strip still
  for `kPersistStillUs` from now, or ESTOP, never a home cycle
  (`ValenceDevice.cpp`, operator ruling 2026-10-09, bd val-4rr).
- **The fence: two words, `g_fence_lo` and `g_fence_hi`, in `g_pos`'s frame.**
  No edge takes the count below the low word or above the high one; an edge
  past it is withheld (no store, no count, `g_fence_hits`), its deadline still
  passes, so the period stands, and a count already outside may move back,
  never further out. Checked once per edge, after the coarse wait and outside
  the fine wait, reading only the word for the edge's direction. The arbiter
  writes it (`MotionArbiter::syncFence()`) before every nonzero steer: the
  backstop's frame in counts, rounded outward by under a step
  (`fenceCount()`), or during a home cycle its whole search, max_rail plus a
  safety margin and two search margins past each end. The board stores the
  bound that narrows first, a memory fence after each store
  (`ValenceMotion.cpp` `LpEmitter::fence()`), so the pair the core can read
  between the stores lies inside the old fence or the new one. Open at load:
  the lease holds the core still until the first write
  [verified 2026-10-06 -- test_motion_arbiter, a steer a 3.9 ms stall
  carries past the window stops with the count exactly on the fence, 13
  edges withheld; the rig without the fence ran 13 steps past].
- **The lease: `g_lease`, renewed by any change.** `steerTick()` renews it on
  every tick; while a home cycle runs, the planner renews it after each seek
  step instead. When it has not moved for
  `g_lease_cycles` (`kLeaseCycles`, `kLeaseTicks` = 4 ticks = 4 ms, written by
  the HP before the core runs and read once) the core stores 0 to
  `g_step_q8`, counts `g_lapses`, and renders nothing until it moves again.
  Unleased at start: nothing renders before the first renewal, so main.cpp's
  boot liveness proof is one lease long (~40 edges at 4003 cycles per edge).
  Checked once per pass, so a lapse stops the edges at most one edge after
  the lease runs out.
- **What the two bounds cost the per-edge path.** 18 instructions on the
  common edge, 43 before and 61 after (ULP `-Os`, disassembled from
  `ulp_main.elf`): 16 before the fine wait (the lease and fence compares),
  one after the edge, and ONE between the fine wait's exit and the edge
  store, a stack reload of the edge table's base that register pressure put
  there (a constant latency, one load, on every edge alike). The fine wait
  loop itself is unchanged, three instructions, and the edge store keeps its
  `lw`/`sw` shape. The LP image's `.text` grew 2,428 to 2,584 bytes. The
  DIG-694 evidence above was taken on the previous shape: re-scope it.
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
  float math only, no heap allocation, no ISR context. ONE exception, by
  operator ruling 2026-10-06 (bd val-cp5, "10 ms is not acceptable"): the home
  sense's rising-edge interrupt calls `MotionArbiter::homeSenseRose()`, which
  may compare-and-swap the seek word, call the emitter's `park()` (a store of
  0 to `g_step_q8`) and `count()` (a load of `g_pos`), store the latched
  count, and nothing else: no float, no log, no allocation, no engine, no
  FreeRTOS call. It is not IRAM-resident, so a flash write holds it off.
- `g_step_q8` has more than one writer only for a stop: every writer other
  than `steer()` (on the steer task, or on the planner while a home cycle
  runs, never both in one tick) stores 0 (e-stop, power loss, the home
  sense's interrupt, and the LP core itself when its lease lapses). A steer
  racing an interrupt park is parked again by the planner (`homeSteer()`,
  a full fence between its steer and its read of the seek word); a steer
  racing a lapse is lost for one tick and steered again by the next.
- The HP-to-LP channel is the LP shared memory window. It carries a velocity,
  the fence, the lease and flags, single-writer per field (`g_step_q8` above
  excepted), never a rendered buffer. A renderer that buffers ahead turns
  every late refill into dead air on the output, which is the reason edge
  rendering lives on a core with nothing else to do.
- Task and core assignment is in `.claude/rules/subsystem-map.md` (C-1).
