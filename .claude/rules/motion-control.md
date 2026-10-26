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
Nucleus build: the P4 image, neutrino-sim, kinetic-wasm and the native suites.

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
residual never steer during a cycle; the engine is reset at the count
when the cycle ends, as after an e-stop. Exclusivity is the rail: a cycle
owns it from the request to its end, `accept()` refuses every intent
meanwhile, and one producer steers per tick: the seek producer runs on the
planner and renews the lease there, and `steerTick()` stands aside while a
leg is in flight. **The safety margin
(`kHomeSafetyMarginMm`, 5 mm) is never a commandable position**: 0.0 mm is a
margin off the low stop's datum, the rail ends a margin short of the high
one, so a 300 mm stop-to-stop rail is a 290 mm usable rail, and the usable
rail is what max_rail stores.

**Who may feed the planner is ONE mode (bd val-5ozv.1).** `ArbMode` in
`MotionArbiter.h`: Run, Paused, Jog, Returning, HomeQueued, the four seek
phases, Unpowered, Estop, written by the planner task alone and moved only by
the `(mode, event) -> (next, action)` rules there; the feed column
(`kArbFeed`) is what `accept()` admits in each. The cross-task methods still
post the SPEC 11 latches and requests as words, and the planner turns each into
an event at the point it already applied it, so the table moved no work in
time; the rail word stays one compare-and-swapped word (an e-stop racing a
start), and the handover, the wall and a refused plan's hold stay the one
trajectory, not modes. A cell with no rule is held, logged and counted (census
`arb_illegal`); every change of mode logs one line and counts
(`arb_transitions`) [verified 2026-10-24 -- test_arbiter_table: every cell
against the audited grid, 62 of 62 legal cells reached at runtime, 13
race-only (75 of 75 with the window-edit cells, 2026-10-25, bd val-5ozv.2;
86 of 86 with the window-adopt cells, bd val-5ozv.5);
test_motion_arbiter unchanged and passing with every illegal cell
aborting (MOTION_ARB_STRICT); kinetic_trace.json byte-identical].

Event-driven, never clocked: an intent becomes knots at arrival, the pending
window is solved at the next sample, and the sampler evaluates it.

- **The planner and the steer (bd val-8rt).** Two tasks on HP core 1
  (`subsystem-map.md`). The planner (priority 5) owns the engine: it drains
  the intents, samples the engine once per tick (`MotionArbiter::planTick()`)
  and publishes THE STRIP, the commanded position in mm every
  `kMotionTickUs` for `kStripLen` (128) entries: the plan read with
  `kinetic2::Engine::peek()`, which returns bit for bit what the sampler
  returns later (the same engine; a preview or the twin matches the board
  only under the same calls and `react_us`, and the hub picks `react_us` per
  submit from its measured solve and lead, `adaptiveReactUs`), the oscillator and the connector (below) summed, with the
  backstop frame and the limit set it was filled under, so the steer never
  reads the window. A plan change (a submit, a brake, a reset) refills it
  whole, from the clock after the tick's solve; a tick without one shifts it
  and appends the new tail. The steer (priority 6,
  `MotionArbiter::steerTick()`) copies the strip under the host's lock every
  tick, interpolates it at now and steers, so a window solve of up to 128 ms
  on the planner never delays a steer. **Past the strip's end the steer
  plays its TAIL (bd val-cy1g)**: a stop from the state the strip ends in,
  computed and certified at every fill (`certifiedStop()`): the fastest
  jerk-limited stop under the set the plan renders under, never a reversal,
  at rest at its end, inside the backstop frame, or braked harder to the
  edge, up to `kHardStopMax` times the set's accel and jerk, and counted
  (census `hard_stops`). The steer counts the run (census `planner_stalls`,
  logged once a second). An engine reset that rebases carries a new
  generation: the steer steers nothing from an older strip and re-anchors
  its feedforward at the reset's position. **THE HANDOVER (bd val-xfy2):
  consecutive strips of a generation are one trajectory.** A whole refill
  takes the new plan with the connector in flight when that holds the set
  across its first ticks against the trajectory the steer was rendering (a
  plan on time: the engine plans every change from its own state at the
  sample). Otherwise (a solve longer than `react_us`, the planner back after
  the tail, a PAUSE cutting the oscillator, a lapse) the trajectory in
  flight is kept `kHandoverTicks` and joined through a CONNECTOR: the gap to
  the new plan there, as a state on the grid, brought to rest at 0 by
  `Profile::point()` and summed with the plan, under the set, then half,
  then a quarter of it, the first whose sum holds every ceiling and the
  frame on the grid (census `late_plans`, a brake's or a cut's fade
  uncounted). None holds: the plan is REFUSED (census `plans_refused`): the
  trajectory in flight stops on a certified stop from the handover, the
  engine is reset at that stop's rest without a new generation, and every
  intent is refused until the carriage is there; the request is not
  incorporated. Nothing installs a plan as a velocity offset. The first
  steer after a tick the steer held (an e-stop, power, a re-origin, a home
  cycle, no strip) starts from the plan itself, which each of those reset at
  the carriage; a window move holds no tick.
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
  unhomed or uncommissioned; PAUSE cuts it at once and the refill's handover
  fades the gap it leaves through a connector, never counted a late plan; an
  engine reset stops it. Its parameters cross from the hub task as a seqlocked post
  (`setOscillator()`) [verified 2026-10-09 -- test_motion_arbiter, the
  oscillator cases: the strip under 300 mm strokes at the input set keeps
  every ceiling and the window, sheds to under a step at full speed].
  **Millimeters (Valence RFC-112, ahead of the pinned SPEC).** The amplitude,
  its drive's out bounds and osc.amplitude_effective are a peak in mm, capped
  by `OSC_MAX_AMPLITUDE_MM` and nothing else. The stage runs in mm, rail
  frame: `fillStrip()` converts the plan it reads, the ceilings are the input
  set's mm, the window is the backstop's, so a window move changes none of it
  and Kinetic's `rescale()` is never called. A swing wider than the room is
  narrowed about the plan, never shifted [verified 2026-10-10 --
  test_motion_arbiter: 5 mm by hand and a 10 mm V8 hold through a window a
  fifth the size; a window edge 3 mm from the rest narrows it to 3 mm about
  the rest, and the room given back plays 5 mm again]. The caps (RFC-112
  item 6) are `OSC_MAX_HZ` 30 and `OSC_MAX_AMPLITUDE_MM` 20, from the
  factory set's sine curve (`valence_config.h`); the waveform is a sine
  only, Kinetic's default, never set [verified 2026-10-11 --
  test_motion_arbiter: a 5 mm ask at 30 Hz renders 1.85 mm inside every
  ceiling, a period on the 1 ms grid].
  **Driven parameters (SPEC 9.7, bd val-o9r; Valence RFC-110, ahead of the
  pinned SPEC).** A parameter driven by a live input, a speed or position
  drive or an `axis` drive while an osc-drive stream is live (under
  `stream_quiet_release_ms` past its newest stamp, whichever session
  published it), puts the stage in Kinetic's driven mode: a sine whose
  frequency and amplitude follow drive points, the phase integrated. An
  `axis` drive with no live stream is exactly `fixed`, so the factory card
  (both drives `axis`) renders the sine and its dwells by hand. While an `axis`
  drive is bound a live stream also renders the stage whatever osc.enabled
  holds (`streamOscillator()`, before the plan read). A stream's start and
  end therefore switch the mode, which passes through rest: the kernel's
  fade out, then in, up to 150 ms each; quiet with osc.enabled off, it rests.
  The osc-drive stream's samples cross from the hub task at arrival through
  a single-producer ring (`postOscDrive()`), and `feedOscillator()`, on the
  planner, makes each a point at its stamp; a speed or position drive adds a
  point every `kOscPlanDriveTicks` at `kOscDriveLeadUs` ahead from the plan
  there. In the driven mode each parameter is its field (`fixed`, or `axis`
  with no live stream while osc.enabled holds) or SPEC 8.11's linear_clamp of
  the plan's speed (mm/s) or position (mm, client frame) or the newest sample
  (`axis`). No point lands nearer than `kOscDriveLeadUs` (156 ms, the
  osc-drive grant's schedule latency); with nothing asked for `kDriveQuietUs`
  (the stream's quiet window, static_asserted) it fades to rest; a point the
  stage cannot hold is dropped and counted. PAUSE, ESTOP and the render
  gates hold a stream as they hold the hand. The hub reports what plays:
  driven, 0x1140 and the ECHO carry both dwells 0, the
  written values kept, and the report changing with a stream's liveness is
  a hub-side cfg_gen (`ValenceDevice::publishOscillator()`, from the census's
  `osc_stream_live`). The stage, its plan buffer and
  the drives cost about 8.9 KB of the arbiter's internal RAM
  (`sizeof(MotionOscillator)` 7,288 B) [verified 2026-10-09 -- riscv32
  `sizeof` at Kinetic 725d448; test_motion_arbiter, a 50 Hz drive sweeping
  5 to 20 Hz keeps every ceiling and fades out on silence; a stream over a
  hand sine held a cycle at each extreme (0.81 of ticks within a tenth of
  the peak, the driven sine 0.30; 2026-10-11) passes through rest to the
  driven sine and, quiet, back through rest to the dwells, every ceiling
  held; a live stream renders nothing under PAUSE or ESTOP].
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
  moves the carriage out of its frame.** The frame is the configured window
  held inside the rail, or the asserted rail while the engine plans in the
  rail frame (override's jog and its return); a home cycle's seek producer
  never reaches it. The brakes (PAUSE, a generator's stop, a starved
  timeline) are profiles the referee never scores, and a capped run renders
  pieces past the window (Kinetic kin-pfbw), so the backstop holds whatever
  the plan does. **THE WALL (operator ruling 2026-10-23, bd val-vj1j)**: at
  every fill, where the strip (the plan, the oscillator and the connector)
  leaves the frame by more than a step, `MotionArbiter::applyWall()` stops it
  instead, at the edge: from the latest entry whose fastest jerk-limited stop
  under the set rests in the frame, its jerk eased so it rests on the edge
  rather than a tick short; when no entry has one, harder from the first,
  up to `kHardStopMax` times the set's accel and jerk (`certifiedStop()`,
  census `hard_stops`). The wall owns the trajectory from the tick the steer
  may render it to its rest; a plan change waits, and once it rests with the
  plan back in the frame the handover joins the plan from there. The census
  counts engagements (`backstops`, at commit), plan.flags `clamped` holds
  while one does. **THE LAST LINE**: `steerTick()` clamps the rendered
  demand on every tick to the frame, widened only to the previous demand, so
  a carriage left outside it is moved back by a plan, never by the clamp;
  into a held edge the feedforward carries the distance to the edge and
  nothing past it. It engages only where no stop kept the strip in (census
  `wall_clamps`, a plan more than one step past, at onset; `clamps=` on the
  status line). Every steer is bounded from position truth so it carries the
  carriage at most to the edge within one tick on a strip the wall kept in
  (`PlanStrip::in_frame`), within `kTickDtCapS` on one it could not; a later
  tick than that the fence holds [verified 2026-10-23 -- test_motion_arbiter:
  a move landing on the edge after a lapse steers jerk 1.00e7 against a
  1e7 mm/s^3 set (2.40e7 with the 2-tick bound's halving); a piece planned
  21 mm past the window stops on the edge at 40,604 mm/s^2 and 8.3e6
  mm/s^3 against 50,000 and 1e7 (90,287 and 9.0e7 held by the clamp); a
  PAUSE brake planned 81 mm past it stops on the edge, braked harder; an
  edge drawn 1 mm ahead at 400 mm/s holds by the clamp, counted]. The wall
  and the clamp are the FIRST line; the LP core's fence (below) is the
  second, and the only one an HP stall cannot skip.
- **Window edits are cells of the table (operator ruling 2026-10-25, bd
  val-5ozv.2).** `ArbEvent::WindowEdit` (setWindow()) and
  `ArbEvent::Reorigin` (force_home) are separate events. Run reframes: the
  pending knots re-target as window shares (bd val-17u) and a jog in flight
  keeps its target in mm, held in the new window. Paused, Jog and Returning
  recheck: the paused position is held in the new window and a RETURN in
  flight re-plans to it; under override the jog keeps the whole rail (SPEC
  11.1). A home cycle, ESTOP and a power loss DEFER: the post is not taken
  during a cycle (`takeWindow()`), and it applies in the mode the cycle or the
  latch ends in, a completed cycle's measured rail kept over the post's.
  **THE GLIDE**: a homed carriage the edit leaves outside the window, with
  nothing in flight carrying it in (pending knots or the re-clamped jog, never
  moving away from it), glides in: from the trajectory in flight at the
  handover, its speed first brought inside the jog set at the set it renders
  under (`Profile::converge`), then the jog set's fastest move to rest where
  it would stop, held in the window (`Profile::point`), as the connector over
  the engine reset at that rest; a glide that would leave the rail stops
  inside it first and glides from that rest. Intents wait for its end; the
  strip's frame (the wall, the clamp, the fence) is widened to its extent
  meanwhile. A wall at rest with the engine's own rest past the frame (a
  PAUSE brake planned past the edge, bd val-vj1j) glides the engine in from
  the carriage, a few steps at most. Nothing glides unhomed, under override,
  or after a re-origin. Census `window_edits`, `window_held`, `glides`,
  `glide_max_mm`; one log line per edit or glide [verified 2026-10-25 --
  test_motion_arbiter, the window-edit cases: a 100 mm glide at rest inside
  the jog set (50 mm/s, 244 mm/s^2), a glide from 520 mm/s slowed at the input
  set then inside the jog set from 15 ms on, never back outside, the stream
  after it landing on its knot with no connector; test_arbiter_table, every
  cell].
- **Adopting the stored window is not an edit (operator ruling 2026-10-25,
  bd val-5ozv.5: motion is only ever commanded).** The hub tags every window
  post (`WindowPost`): `edit` only for a session's own config-set, trials
  included; `adopt` for the boot's push, the measured rail after a home
  cycle, and a trial revert. `ArbEvent::WindowAdopt` is its own event, and
  its act (`ArbAct::adopt`) is recheck that never starts a glide: a carriage
  the adopted window leaves outside waits there (one `WINDOW` line, "adopted,
  waits outside"), and the first command plans from
  its live state into the window through the handover at its own set. A
  glide already in flight re-aims; an edit taken after an adoption that had
  not applied yet judges the carriage against the adopted window. A
  cycle, ESTOP and a power loss defer an adoption as they defer an edit
  [verified 2026-10-25 -- test_motion_arbiter, the window-adoption cases: the
  bench's boot (homed at 0, window [55, 255]) moves no step in 3 s whichever
  post the planner takes first, a stream segment then enters inside the input
  set and stays in, a jog inside the jog set; test_valence_device
  VD-WIN-POST; test_arbiter_table, every cell].
- **A late tick is a stall, never a burst (bd val-1w8).** Every steer is
  priced over at most `kTickDtCapS`, two `kMotionTickUs`, so none exceeds the
  vmax of the set the plan in flight was planned under (the jog set's for a
  Manual plan and the RETURN, the input set's otherwise) plus that long of
  the input amax; a tick later than that steers the plan's own mean velocity
  across it and is counted (census `stalls`, on the status line). Between
  ticks the LP core renders the last steer open loop, so a steer-task stall
  (a flash write with the cache off; never the window solve, which runs on
  the planner) carries the carriage that steer for at most `kLeaseUs`, then
  the LP core stops it (the lease, below), and never past the frame's edge
  (the fence): a stall in the last ticks of a stop at the edge meets the
  fence. The next tick counts the lapse (census `lease_lapses`,
  `lapses=` on the status line), parks, and asks the planner to re-plan
  from the carriage at rest (the handover from rest), steering again only
  from the strip that answers; it never re-anchors `_p_cmd_mm` to the count:
  a stall between the clock read and the steer reaches `steerTick()` with a
  `dt_s` measured before it, and the residual as feedforward over that is an
  uncommanded reversal (measured -600 mm/s against a 223 mm/s plan).
  **THE RESIDUAL** is what the emitter could not render on time (the ramp
  out of rest, the count's quantization), measured against the previous
  command, never the plan now (the feedforward runs one tick behind it by
  construction), deadbanded at a step: a velocity offset of at most one tick
  of the set's accel, reached under the accel the plan leaves free and
  changed by at most one tick of its jerk. It never joins a plan
  [verified 2026-10-22 -- test_motion_arbiter, the brake tail and handover
  cases: the steered velocity's change per tick inside the set's accel and
  its change inside the jerk through strip exhaustion, a late plan, a late
  reversal refused, a window write, a PAUSE cut and a lapse].
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
  So every runtime flash write (the settings and presets blobs, the trust
  ledger and gesture clear, the saved networks, the image buy) waits for
  REST, `motionFlashOk()`: the arbiter's mode rule (`kArbFlash`: Run, Paused
  and Jog `rest`, Estop and Unpowered `open`, Returning and a home cycle
  `hold`) and, under `rest`, no plan, seek, return, oscillation or stepping
  emitter. Settings apply in RAM at once; a power loss before rest loses the
  held write (bd val-4rr, val-y3da, operator 2026-10-25).
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
  edges withheld; the rig without the fence ran 13 steps past; 2026-10-23,
  the stall in the last ticks of a 50 m/s^2 stop on the edge: the count on
  the fence, 1 edge withheld].
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
