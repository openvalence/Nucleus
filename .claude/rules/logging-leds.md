---
paths:
  - "**"
---

# Geiger and Flux (NON-NEGOTIABLE usage)

Carried from the archived machine repo (SlopDrive-32
`.claude/rules/logging-leds.md`). Self-contained ecosystem modules:
hardware-free core plus thin platform glue. `lib/geiger` and `lib/flux` hold
the cores; their IDF glue twins land with the
hub port. Both are header-only via an explicit include path.

## Logging goes through Geiger. Only.

- `GLOGT/D/I/W/E/F("tag", fmt, ...)` from any task, either core. Bounded
  format plus spinlock slot copy; never blocks, never allocates, NOT ISR-safe.
- Throttle with `GLOGx_EVERY_MS`. Compile floor `GEIGER_COMPILE_LEVEL`.
- **ONE drain point**, on one task, with sinks implementing `geiger::ISink`
  registered in one place. The glue file that owns it names itself when it
  lands; there is never a second drain.
- **No `printf` debug output in firmware, and no new log macros.** Enforced by
  canon_lint `printf-outside-geiger` and `new-log-macro`, scoped to the hub
  sources. Exactly one file is exempt from both, because it IS the front door
  and the output device's driver; canon_lint names it, and a SECOND definer
  anywhere is the violation the check exists for. Bench sketches are out of
  scope: their report lines ARE their product, and they run before any sink
  exists.
- **Boot lifecycle:** the sink layer immediate-drains during single-task
  startup, the composition root disables that before task creation, and the
  console sink demotes to Warn and above once a richer reader exists.

## LEDs go through Flux. Only.

- Callers speak semantics: `fluxEngine().set(System::X, Status::Y)`.
  Board wiring lives in exactly one glue file per board.
- **Never drive an LED pin directly anywhere else.**
- The PCB has no LED fitted on the bench stamp yet. The grammar below is
  binding the day one exists; it is not re-litigated then.

## The two-axis LED grammar (operator rulings 2026-08-06)

COLOR names the SYSTEM speaking; EFFECT names that system's STATUS. One
pixel shows one pair; the arbiter takes the highest Status (the
time-sensitivity rank), ties to the higher System (Safety above all). The
grammar is FLEET-WIDE: the same color is the same system on every board.

| color | system | speaks on |
|---|---|---|
| green | all quiet (every system Nominal) | the floor |
| cyan | Link: a board-to-board bus | any board that has one |
| blue | Network: WiFi / WS | the board that owns the socket |
| amber | Motion plane, INCLUDING the drive/Modbus bus | the motion board |
| magenta | Session: auth, pairing ceremonies | any |
| white | Flash: THIS device's firmware being written | any |
| red | Safety: fault, e-stop | any |

| rank | status | effect | timing |
|---|---|---|---|
| 6 | Urgent | fast blink | 400 ms on / 200 off |
| 5 | Ceremony | blink | 700 ms on / 350 off |
| 4 | Degraded | slow blink | 1200 ms on / 600 off |
| 3 | Working | fast breathe | 1.4 s |
| 2 | Latched | solid | waiting on the operator |
| 1 | Nominal | slow breathe | 3 s |

- **Blink ON time is the knob; OFF is always half of it** (ruling): the color
  carries the message, the gap is the punctuation.
- **Boot is rainbow mode** until every system the board required has reported
  ready (`requireReady`/`markReady`). A rainbow that never ends IS the
  diagnostic: some subsystem never came up.
- **Pulses** (`pulse(System, ~100 ms)`) overlay the steady render as an ack
  blip in the system's color, but NEVER cover Ceremony or Urgent (ruling:
  severity layers).
- Unhomed renders Motion/Latched (solid amber), NOT a red fault -- the T15
  lesson expressed in the grammar instead of an enum ordering.
- **The LED liveness gate is a safety feature.** Animation advances only while
  every registered heartbeat pulses. Never defeat it. Frozen LEDs are a
  diagnostic.

## T6 -- log sinks must never block

**Rule:** any log sink is non-blocking by contract: drop-and-count when the
output is full. Never add a sink that can wait.
**Mechanism:** a USB console with no host attached blocks in the TX-full path,
on whatever task drains the log ring. One chatty subsystem then freezes that
task with zero CPU load visible.

## T7 -- LED freeze is a diagnostic, not a bug

**Rule:** never "fix" static LEDs by moving the Flux pump or removing a
heartbeat pulse.
**Mechanism:** it is a liveness gate. Frozen LEDs plus live cores means the
task hosting the pump is blocked, see T6. This distinction has solved a field
incident; preserve it.

## T15 -- fixed-priority status displays can mask a lower-priority, time-critical state

**Rule:** when a single highest-active-state-wins display picks ONE thing to
show, rank by TIME-SENSITIVITY (what is gone if missed right now), not by
severity. A persistent, rediscoverable condition may correctly rank BELOW a
narrow window a human must catch immediately.
**Mechanism:** a one-state display showed `Fault` whenever the machine was
simply unhomed, the ordinary state of a fresh boot. A 120 s push-to-pair
window opened by triple power-cycling a factory-fresh and therefore UNHOMED
device landed on exactly the device that was ALSO showing Fault, and Fault
won: red breathing instead of the pairing invitation, hiding a gone-if-missed
ceremony behind a condition still true the next time anyone looks.
**Fix:** the two-axis grammar above carries it as a RULE: Ceremony ranks above
Degraded and below Urgent, and unhomed is Motion/Latched rather than any fault
at all.

## T17 -- a flag reused across unrelated concerns can silence a sink for good

**Rule:** gate a sink's existence at RUNTIME, not compile-time, and never let
a flag whose stated job is something else (transport selection, a feature
toggle) also decide whether a diagnostic sink is registered at all.
**Mechanism:** a serial log sink was once wrapped in `#if !SERIAL_CONTROL_MODE`
inside the sink registrar, but that macro's actual job is picking the
factory-default transport. At the macro's normal value the `#if` never
registered the sink at all: not throttled, not floored, ABSENT, for the entire
life of the build. Every log call still went out over the web ring, so nothing
looked broken from the firmware's own side; only a human watching the console
would notice, by which point the boot banner and any early crash trace were
gone.
**Fix:** a sink's existence is never compile-time-conditional on a flag that
means something else. Visibility is composed from independent RUNTIME floors.

## Diagnostics are a DUMP, not a stream (operator ruling 2026-08-06)

Depth beats liveness. The failure that matters is "the ring recycled before
anyone read it", not "the ring is not fast enough", so the device records into
a deep PSRAM archive read ONCE, after something goes wrong.

- **Three rings, three jobs, and they do not merge.** A small
  severity-partitioned DISPLAY ring (a Debug flood can never bury an error), a
  flat deep ARCHIVE, and a power-loss-surviving ring for post-panic forensics.
  Every property of the display ring is a consequence of being small; scaling
  it instead of adding a second one means a multi-MB copy under a spinlock.
- **The archive dies with its boot.** Its memory is re-allocated at startup.
  Post-panic forensics is the third ring's job. Never cite the archive as a
  post-reboot instrument.
- **The writer NEVER stops for a reader; there is NO freeze.** Slots carry a
  monotonic seq; a reader detects being lapped and truncates WITH A FOOTER
  NOTICE instead of being protected. Never reintroduce a freeze, a lock, or
  any reader-owned state on this ring.
- **The footer's `next=<seq>` is the resume cursor** and it is the LAST line of
  every dump, on purpose: machine consumers parse it there and pass it back as
  `?from=`. That is what lets a reader pull the archive in bounded requests
  instead of one held-open stream.
- **Blocking during a full local dump is ACCEPTED, not a defect.** This is the
  instrument reached for when the machine is already unwell, so it carries no
  heap floor and no mid-body abort: it must not be the first thing memory
  pressure switches off.
- Filtering by GLOGx tag is what gives a new subsystem a route without a route
  table change.

The relay half of that story -- pulling the archive through a second board --
is legacy: the archived SlopDrive-32 `.claude/rules/logging-leds.md` is the
case file. Here the P4 serves its own diagnostics.
