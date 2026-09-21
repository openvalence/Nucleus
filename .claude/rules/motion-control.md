---
paths:
  - "**"
---

# Motion control constraints

Architecture-level motion doctrine (one command one plan, the MotionArbiter
sole-caller rule, the two-chip split, HP-evaluates / LP-renders) lives in
`.claude/rules/architecture.md` §2. This file is the mechanism layer.

## VMotion (`lib/vmotion/`)

Every command becomes ONE trajectory planned from the engine's actual
(p, v, a); the sampler evaluates it. Event-driven, never clocked.

- **Map:** header-only, hardware-free `vmotion::Engine` wrapping vendored
  `lib/ruckig/`, which is BYTE-IDENTICAL to upstream. Wrap, never patch; see
  `lib/ruckig/VENDORED.md`. The namespace rename to `vmotion` is board work
  (`val-091.1`), gated on the native suite, not a sed pass.
- **Division of labor (MEASURED; re-run the bench before re-litigating).**
  Ruckig Community is a point-to-point planner, not a waveform interpolator.
  WAVEFORM (every duration-carrying segment, with NO duration floor: a 10 ms
  knot is a 10 ms span with its authored tangent) is a Hermite curve in the
  client's declared family over exactly the commanded duration, ceiling and
  window scanned. An illegal shape is first shortened toward a legal stroke
  that still holds the deadline (the Blend policy; the amplitude budget is a
  FLOOR the search must honor, never cross), and only a shape still illegal at
  that floor falls through to the Ruckig guard. Stretch (keep the stroke,
  overrun the deadline) is the one alternative contract. Amplitude is the one
  quantity a ceiling may shape; this is the operator-ratified exception
  (2026-09-02) to "ceilings are clamps, never targets". CHASE (bare points,
  no duration) is Ruckig replan-per-point. Sample synthesis is gone
  (2026-09-02): no client sends bare points at a rate that needs a holdback.
  SETTLE is brake-to-rest when a plan ends still-moving with no fresh command.
- **One activity clock (operator ruling 2026-09-02).** Every "is the stream
  alive" question in the engine (settle grace, cold start, staleness) keys on
  ONE reference stamped by every commit and every plan end, in the engine's
  own clock. A reset voids the plan and the pipeline; it never erases the
  stream's cadence. Two mechanisms answering the same question from different
  references is the defect class that produced every field hitch of
  2026-09-02.
- **Safety:** Ruckig Community has NO position limits and quintics can bulge,
  so the Engine owns the window: targets clamped, end velocities bound-safe,
  quintics legality-scanned, sampled output clamped. Exceptions are never
  instantiated; non-finite inputs are rejected at `commit()`.
- **Whichever task calls `commit()` needs a deep stack** because it nests
  KB-scale Ruckig temporaries. Never size one down without a measured
  high-water mark under a real motion workload (T1 class, `memory-budget.md`
  T21). That stack is HP-side and internal RAM only (`governance.md` §6).

## The LP-core emitter (`flagship_p4/ulp/`)

The LP core renders edges and does nothing else. Its signed edge count IS the
machine's position. All numbers below are measured on this stamp
[verified 2026-09-20 -- Rigol DHO4204, single-shot captures at 8 ns and 100 ns
per sample, decoded in numpy; board `val-091.3`, continued from the archived
SlopDrive-32 `sd-1bi.3`].

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
  The step-and-direction case file (FAS, MCPWM, RMT, the PCNT retime) is
  the archived SlopDrive-32 `.claude/rules/motion-control.md`; it describes a
  backend that does not exist here.

## ISR / IRAM / core discipline

- Motion code on the HP side uses microcritical sections, not ISRs: short
  float math only, no heap allocation, no ISR context.
- The HP-to-LP channel is the LP shared memory window. It carries a velocity
  and flags, single-writer per field, never a rendered buffer. A renderer that
  buffers ahead turns every late refill into dead air on the output, which is
  the reason edge rendering lives on a core with nothing else to do.
- Task and core assignment is in `.claude/rules/subsystem-map.md` (C-1).
