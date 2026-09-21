---
paths:
  - "**"
---

# Architecture constraints

The product: an extensible, high-performance modular linear motion control
platform on the OSSM Flagship PCB. Hardware-agnostic, community-extensible.
Valence Drive is the firmware; Valence is the protocol it speaks.

## 1. Hardware-agnostic modularity

- **Driver polymorphism.** All physical hardware interaction (motors, sensors,
  inputs) sits behind C++ interface classes, ABCs with pure virtuals.
- **One motion backend.** The only thing that moves the motor is the LP-core
  quadrature emitter, driven by a plan the HP core evaluates. No second
  renderer exists and none is added: a second renderer is the seam the old
  machine's three-board split was built to delete. The Modbus bus stays for
  motor configuration and the encoder audit only, behind a build switch a
  production build may omit; the drive is programmed once.
- **Build configuration.** ESP-IDF Kconfig plus conditional compilation
  isolate hardware driver implementations; unused driver objects are not
  compiled. `flagship_<chip>/sdkconfig.defaults` is the one home for silicon,
  memory-map and radio configuration (C-1).
- **Pin allocation.** No hardware pins inside functional classes. Pins, serial
  ports, and timer channels are constructor-injected or mapped in one board
  config header per project.
- **Module boundary doctrine (operator-ratified 2026-07-27, carried).**
  Functional cores are LIFTABLE: hardware-free, dependency-injected (clock,
  randomness, transport, motor handed IN, never grabbed), native-testable,
  stitchable into another codebase. `lib/vlog`, `lib/vglow` and `lib/vmotion`
  are the proof pattern, and the IDF port is its honest test: cores lift, glue
  is rewritten. Glue is the opposite and proudly so: the composition root
  (`main.cpp`), delegates, and board wiring are deliberately machine-specific,
  small, and honest. The smell to hunt is a core-shaped thing living inside
  glue. New functionality starts by deciding which of the two it is.

## 2. Core performance and safety (NON-NEGOTIABLE)

- **Non-blocking runtime.** Operational loops and real-time motion paths never
  block. Millisecond-scale sleeps are PROHIBITED during regular runtime; short
  blocking delays are permitted ONLY in boot/init, module hardware setup, and
  isolated slow-speed calibration/homing cycles. The ban means sleeps that
  stall a task's other duties. A bounded busy-wait under one millisecond with
  a measured hardware reason is not a delay in this sense (operator ruling
  2026-09-02).
- **Motion doctrine, event-driven and never clocked.** ONE COMMAND, ONE PLAN,
  the motion engine executes. Plans are computed at intent arrival from the
  machine's ACTUAL state (live position plus velocity); speed and accel are
  DERIVED from the intent and CLAMPED at ceilings. Ceilings are never targets,
  the one exception being deadline-less manual point moves, which plan at user
  ceilings. A loop computing positions on a clock is rebuilding a disease this
  project already cured. Segmentation follows commands and waveform structure
  only.
- **Two-chip split (ruling 2026-09-20).** The ESP32-P4 owns EVERYTHING that
  decides: motion, the Valence hub, policy, and the sockets. The ESP32-C6 is a
  WiFi NIC and nothing else -- Espressif's stock `esp_hosted` slave image over
  SDIO, no firmware of ours on it. There is no bridge, no second protocol
  terminator, and no forwarding hop; the P4 runs the WebSocket server itself.
  Sockets stay on the P4 by ruling: moving them to the C6 would recreate the
  retired C5 bridge and its trap class to save memory the P4 does not need.
  The encoder is the AUDITOR of the motion truth: the drive follows quadrature
  exactly unless asked for the impossible, so a calc-vs-encoder deviation
  means an infeasible demand reached the motor. Ceilings are therefore
  measured and enforced in the engine; an emitter cap is a fault detector,
  never a shaper. Landing state on the dev board (`val-091`).
- **MotionArbiter sole-caller rule.** The MotionArbiter is the ONLY component
  that commands the motion processor. Input sources (manual UI, TCode
  transports, PatternEngine, Valence sessions) never touch the emitter: they
  submit intents. The arbiter owns arbitration, limit-set selection (user set
  for manual, input set for machine-driven), and every safety gate -- homed,
  paused, e-stop, window clamping, soft-start.
- **Core separation on the P4.** The LP core RENDERS EDGES and nothing else:
  a phase accumulator in exact cycles, one store per edge, no allocation, no
  branchy work. The HP core EVALUATES THE PLAN and hands the LP core a
  velocity at its tick, and also runs the Valence hub, the network and
  storage. The LP core's signed edge count IS position truth. The division is
  measured, not aesthetic: the LP core runs from LP SRAM with its GPIO in the
  LP domain, so an HP flash write does not touch it
  [verified 2026-09-20 -- HP-side flash hammer, ~1.0 s of every 5 s with the
  cache disabled: LP `late=0`, `f_LP` 40.00 MHz unchanged, 2,156 scoped
  transitions, 0 illegal, 0 outliers]. The cache-off class that stalled the
  old machine's motion path does not exist here for an LP-core emitter.
  Mechanism and the emitter's own rules: `.claude/rules/motion-control.md`.
- **Cross-core data.** Anything shared between HP tasks uses FreeRTOS
  primitives (atomics, mutexes, queues). The HP-to-LP channel is the LP shared
  memory window, single-writer per field, and it carries a velocity and flags,
  never a rendered buffer. Callbacks from IDF event loops and drivers run on
  the library's own task: enqueue, never mutate owner state. See
  `.claude/rules/transport.md` T5.

## 3. Naming doctrine

Invented ecosystem-level things (protocols, subsystems, tools) get
zero-collision, SEO-unique names: "Valence", never "SyncManager". Ordinary
classes and variables keep plain descriptive names. The product names (OSSM
Flagship / Valence Drive / Valence) and the 2026-09-21 rename are in
`governance.md` §6; wire-visible names still change upstream only, never
from this repo.

## 4. Valence Tool -- the tool surface has ONE door

Valence Tool is the Valence project manager: one UI over every tool in the
ecosystem. Its home is the Valence repo so it ships with the SDK; a vendor
building a hub who never clones this repo still gets it. Landing state lives
on the dev board.

- **Every tool registers a manifest entry, and that is the whole of adding
  it.** A tool is declarative data: name, command, input globs, how its
  pass/fail reads, what toolchain it needs. Adding tool N+1 must require ZERO
  changes to Valence Tool's own code. If it has to learn about a tool, the
  registration is wrong: fix the manifest schema, not the console.
- **Valence Tool knows nothing tool-specific.** No branch anywhere may name a
  tool, a repo, a language, or a build system. It reads manifests and spawns
  processes. A single `if tool == ...` is the whole design failing, and it is
  a flag, not a shortcut.
- **Standalone invocation NEVER stops working.** Every tool stays runnable
  from a plain shell exactly as it is today. Valence Tool is a funnel, not a
  gate: CI, headless agents, and an operator with a terminal must never depend
  on it. A tool that only works through the UI is a defect.
- **A result carries a fingerprint of its inputs; this is C-4 in software.**
  Results are stored against a hash of the entry's declared inputs. When those
  inputs move the result goes STALE, never "failed": stale means
  no-longer-evidence, which is exactly C-4's "touched by commits since its
  stamp is hearsay". Never show a stale pass as a pass.
- **The manifest is the home for how-to-run.** PLANNED CHANGE: the build and
  deploy procedure in `.claude/rules/build-test-deploy.md` becomes a pointer
  into the manifest in the same commit that lands v0. Until then that file
  remains the home (C-1), so do not split it early and do not let both stand
  afterward.
