---
paths:
  - "**"
---

# Where things already are

Orientation, so no agent spends calls rediscovering the shape of the tree.
`repo-layout.md` is the normative twin: where a NEW file goes. This one is
descriptive: where the existing things live. Structural on purpose, so it rots
slowly. For anything finer, ask codebase-memory rather than grepping for
`^class` (see `navigation.md`).

## The tree

| Path | What lives there |
|---|---|
| `flagship_p4/` | The P4 firmware project: motion, the Valence hub, policy, sockets. Pure ESP-IDF |
| `flagship_p4/src/` | HP-core sources and the composition root |
| `flagship_p4/src/hub/` | The Valence hub and its IDF glue: platform shims, the WS port, the UI token, catalog, config |
| `flagship_p4/src/system/` | Board services: on port 80 the shared HTTP server, OTA and the `/diag` archive (the Geiger archive sink); off it, the motor current monitor, the board monitor link, the boot self-check and the log bridge |
| `flagship_p4/src/system/BoardPins.h` | Every P4 pad the Flagship wires, named by function. Plain C macros, because `ulp/lp_quad.c` includes it too. `docs/board-map.md` is its by-function inventory (nets, parts, firmness, what touches each pin) |
| `flagship_p4/src/system/PowerMonitor.h` | The motor current monitor's register layer (U11, INA228 or INA237 on one footprint): identification, calibration, decoding, alert limits. Hardware-free; suite `test_power_monitor` |
| `flagship_p4/src/system/ValencePower.*` | `PowerMonitor.h`'s IDF host: the private I2C bus, the boot-time chip choice, reads and the latched ALERT re-arm. `app_main` calls `powerBegin()` before motion |
| `flagship_p4/src/system/Supervisor.h` | The P4 to board monitor (U12, CH32V003) link: register ids, block layouts, CRCs, encoders and decoders. C/C++ common subset, shared verbatim with `flagship_ch32v003/`. Contract: `docs/supervisor.md`. Suite `test_supervisor` |
| `flagship_p4/src/system/SelfCheck.h` | The boot self-check table: named checks in the ruling's order, each pass/fail/skipped with a reason; motor power only when every entry passed, plus the board-monitor judges over `Supervisor.h` blocks. Hardware-free; suite `test_self_check` |
| `flagship_p4/src/system/ValenceSelfCheck.*` | `SelfCheck.h`'s IDF host: holds MOTOR_EN and PRECHARGE_EN low from `app_main`'s first line, runs the checks once after the hub is up, re-logs the summary each minute |
| `flagship_p4/src/system/MotorSwitch.h` | The motor switch (U2, TPS48111) as a state machine: off, precharging, on, faulted; the pre-charge window and its derivation, the inrush, EN-node and MOTOR_V+ checks, the fault latch. Hardware-free; suite `test_motor_switch` |
| `flagship_p4/src/system/ValenceMotorSwitch.*` | The door (`.h`, IDF-free: the delegate and the sim call it) and the board host (`.cpp`): the switch task, ADC1's one reader (MSW_IMON, EN_NODE, and THERM handed out by `motorSwitchThermVolts()`), the INA ALERT re-arm, the cut on the calling task under one spinlock, and the push of `on` to the arbiter's power gate. `sim/valencesim/src/SimMotorSwitch.cpp` is the twin's host |
| `flagship_p4/src/system/ButtonGesture.h`, `EstopInput.h`, `Thermal.h`, `StatusLook.h` | Hardware-free cores of the board I/O: press / hold acted on release (suite `test_button_gesture`); the E-stop's NC and NO contacts as one debounced reading, the bench mask included (suite `test_estop_input`); the J7 thermistor table and the fan policy (suite `test_thermal`); the machine state and the Flux pair each state shows (suite `test_status_look`) |
| `flagship_p4/src/system/ValenceBoardIo.*` | The BoardIo task, host of `ValenceEstopInput.*` (NC on G39 and NO on G30, sampled first in every pass and published as one atomic byte, `estopInputRead()`; the ESTOP latch and the release refusal are `ValenceDevice`'s), `ValenceGlow.*` (the board's one Flux glue: RMT to the GRBW status pixel on G26), `ValenceFan.*` (LEDC on G27, PCNT tach on G38, the fan policy) and `ValenceButtons.*` (HOME on G49 and PAIR on G52: gestures logged and parked, `homeButtonTake()` / `pairButtonTake()`; the bindings are `ValenceDevice::tick()`'s) |
| `flagship_p4/src/system/ValenceDbg.h` | The DBG marker on G37: `dbgBegin()`, `dbgLevel()` (one store), `dbgPulse(n)`; called from nowhere by default |
| `flagship_p4/src/system/ValenceLogBridge.*` | The Geiger sink that publishes Warn and above on the Valence log channel 0x0008, only from the hub task's drain |
| `flagship_p4/src/hub/ValenceDevice.*` | The delegate and every STATE publisher. Hardware-free: the sim compiles it verbatim |
| `flagship_p4/src/hub/UiTokenTable.h` | The `/uitoken` slot table, rate gate and HMAC derivation, lock injected. Hardware-free: the sim compiles it verbatim; `ValenceUiToken.cpp` is only its board host (spinlock, secret, the :80 route). Suite `test_ui_token` |
| `flagship_p4/src/motion/MotionArbiter.*` | Every motion gate, the window clamp, limit sets and feedforward, emitter and clock injected. Hardware-free: the sim compiles it verbatim; `ValenceMotion.cpp` is only its task host |
| `flagship_p4/src/patterns/` | The two generators, separate rail sources (RFC-093): `ClassicGenerator` and `AdvancedGenerator` on the shared `PatternEngine` scheduler (hardware-free, the sim compiles it verbatim), their settings value and preset store, and `ValencePattern.cpp`, their board task host. Strokes leave as intents through `motionSubmit()` |
| `lib/strokeengine_patterns` | VENDORED StrokeEngine pattern classes, verbatim (`VENDORED.md`); `<Arduino.h>` comes from `flagship_p4/src/patterns/arduino_compat/` |
| `flagship_p4/ulp/` | LP core sources. One directory per project, fixed by the builder |
| `flagship_ch32v003/` | The board monitor's own PlatformIO project (bare metal, ch32v003fun): `src/monitor_core.c` holds every decision and is hardware-free (suite `test_supervisor`), `src/main.c` is the chip glue. The P4 programs it over SWIO; its image ships inside the Nucleus OTA |
| `flagship_p4/sdkconfig.defaults` | Silicon, memory map, PSRAM, radio. Hand-written, tracked |
| `sim/valencesim/` | The device twin: real hub, catalog, device and engine on a desktop. CMake, never pio |
| `lib/kinetic` | The trajectory engine (liftable, hardware-free) |
| `lib/geiger`, `lib/flux` | Logging and LED cores (liftable, hardware-free) |
| `lib/ruckig` | VENDORED, byte-identical to upstream. Do not restyle or respell (C-11 carve-out) |
| `lib/valence` | Symlink to the sibling Valence repo, pinned by `valence.pin`. READ-ONLY from here |
| `docs/` | Prose that outlives a run; `docs/flagship-board.md` forwards to the PCB rationale in the Hardware repo |
| `tools/` | Instruments. Mostly gitignored; the tracked ones are named in `.gitignore` |
| `artifacts/` | Per-run output. Gitignored except the anchor |

There is no C6 project and there will not be one: the C6 runs Espressif's
stock hosted slave image (`architecture.md` §2).

## Cores and tasks

The P4 has two HP RISC-V cores and one LP core. The division is doctrine
(`architecture.md` §2): **LP renders edges, HP evaluates the plan and runs the
hub.**

| Runner | Core | Role |
|---|---|---|
| `app_main` | HP core 0, with the `esp_hosted` SDIO service | Motor power held off first, boot report, subsystem start, the boot self-check last, periodic liveness line (free/maxblock for both heaps, LP counters, stack high-water, self-check verdict) |
| `Motion` (`motion/ValenceMotion.cpp`) | HP core 1, priority 6 | Hosts the MotionArbiter: plans on intent arrival, hands the LP core a velocity each tick. Stack `kMotionTaskStackBytes`, the deep one (`motion-control.md`) |
| `ValenceHub` (`hub/ValenceHub.cpp`) | HP core 1, priority 5 | The hub, single-task by design (`transport.md` T5), and the one Geiger drain, so the log bridge's sink runs here. Stack `kHubTaskStackBytes` |
| LP emitter (`ulp/lp_quad.c`) | LP | Quadrature edges from a phase accumulator. Its signed edge count is position truth |
| `MotorSw` (`system/ValenceMotorSwitch.cpp`) | HP core 0, priority 5 | The motor switch's host: a 5 ms watch on MSW_FLT_N and the EN node, the pre-charge window, the enable request's INA re-arm. Stack `kMotorSwitchTaskStackBytes` |
| `BoardIo` (`system/ValenceBoardIo.cpp`) | HP core 0, priority 3 | A 10 ms pass over the E-stop contacts (first), the HOME and PAIR buttons, the status pixel's Flux pump and the fan policy (stepped once a second). Nothing on a motion path: the E-stop reading is the hub task's to latch (`ValenceEstopInput.h`); its one direct safety act is the HOME hold's dead-hub fallback (`ValenceButtons.h`). Stack `kBoardIoTaskStackBytes` |
| `Pattern` (`patterns/ValencePattern.cpp`) | HP core 1, priority 4 | Both generators: wakes when either's half-stroke is due or settings arrive, submits it as an intent. Below the hub (5) and the motion task (6); stack `kPatternTaskStackBytes` |
| `esp_hosted` / WiFi / lwIP tasks | HP | Owned by the drivers, not by us. Their callbacks are not our task (`transport.md` T5) |

Every stack size and the measurement behind it live on its constant, never
here. Task stacks are internal RAM (`governance.md` §6).

## One fact, one home (C-1)

Stop grepping for these. They live in exactly one place.

| Fact | Home |
|---|---|
| Firmware version | `FIRMWARE_VERSION` in `flagship_p4/src/hub/valence_config.h` |
| Silicon revision, flash size, PSRAM, ULP reserve, radio pins | `flagship_p4/sdkconfig.defaults` |
| Build entry point, upload port, board id | `flagship_p4/platformio.ini` |
| Every P4 pin number (the C6 SDIO pins excepted, they stay in `sdkconfig.defaults`) | `flagship_p4/src/system/BoardPins.h` |
| The P4 to board monitor message set | `flagship_p4/src/system/Supervisor.h` |
| Wire numbers, CBOR keys, NACK codes, channels | sibling `Valence/spec/registry/registry.yaml` |
| Protocol behavior | sibling `Valence/spec/SPEC.md` |
| This machine's channel allocation | the `ch::` namespace in `flagship_p4/src/hub/ValenceCatalog.h` |
| PCB design rationale and the bench measurements behind it | `../Hardware/flagship/design-considerations.md` |
| Versions, deployment state, milestones, open bugs, rulings | the dev board (`bd`) |

## First moves that are never wasted

1. `bd ready` and `bd list --label area:<subsystem>` for current state. In a
   SUBAGENT run `bd prime` first; SessionStart hooks do not fire for you.
2. `search_graph(query="...")` in codebase-memory for "where is the thing that
   does X", especially when you do not know the symbol name.
3. `get_symbols_overview` (serena) for a file's shape. Never grep `^class`.
4. `python tools/canon_lint.py` before declaring work done. **Baseline is ZERO
   findings.** A finding is a defect, never furniture -- fix it or nothing
   else proceeds.
