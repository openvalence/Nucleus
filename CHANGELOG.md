# Changelog

Newest first. Each entry has two parts: what you will notice, and what changed in the code. No release images are published yet; versions are the `FIRMWARE_VERSION` builds (`flagship_p4/src/hub/valence_config.h`), with `-p4hub` on release images and `-p4hub-bench` on bench images.

## 0.1.71 (2026-10-25)

### For users

- **No input makes the machine move faster than its limits or past the stroke window.** A stream with knots too close together, duplicated, out of order or out of range could briefly drive the carriage over the speed or acceleration limit (the bench's anomaly log showed spans 12 and 20 times over a limit). Such a knot is now approached as fast as the limits allow and reached late or short, and the Anomalies log reports it as a trimmed knot with the share of its stroke kept.
- **Each anomaly says what it means, in units.** The share of a knot's stroke kept (%), how far over a limit a span was (x), the speed at a brake or as held (mm/s), why a knot was refused, the knot's position in mm on the stroke as you see it, and the time in ms since start-up.

### Technical

- **kinetic.pin -> 2d0ca3a** (Kinetic main: fix/traw-legality, kin-traw; the conditioner, kin-quis). Compiled paths changed: `conditioner.hpp` (new), `engine.hpp`, `profile.hpp`, `solver.hpp`, `types.hpp`, `tests/test_kinetic2.cpp`. Every knot is held to 0..1; every run end is judged as a brake start; `Engine::submit` refuses a past or duplicate knot before the horizon commit; the judged window never grows past 0..1; a profile phase into a constant one lands at zero acceleration. A fitted piece still over speed, acceleration or the window is replaced by `passKnot`, the fastest legal move toward its knot, reported `KnotTrimmed`. `trim_max` is the trim a client would like, never a license: past it the solver still renders legal (operator ruling 2026-10-25). Jerk-only excess still renders and reports `PieceOverCeiling` (kin-y6e). `Config::condition` compiles in, off by default (`KINETIC2_CONDITION`). Kinetic's fuzz, 480 runs: 0 over a ceiling, 0 outside the window; fingerprint 0xb43b8ddce5ab576b unchanged.
- **motion-anomaly 0x4100 bodies are self-describing** (val-vb3w, RFC-122). Key 4 `detail` is retired, never reused; each kind sends only the field its detail means: `kept` (key 6, %, knot_trimmed), `ceiling_ratio` (key 7, x, piece_over_ceiling), `refusal` (key 8, options non-finite / in the past / timeline full, knot_refused), `v_end` (key 9, mm/s in the client frame, settle and endvel_clamped). `target` (key 3) is mm in the client frame (`MotionArbiter::popAnomaly`, flip mirrored against the rail) with role telemetry.target bound within its entry. Key 5 `t_us` becomes `t_ms`, ms since boot, SPEC 7.2's EVENT time. Catalog-only. Neutrino etag 0d182b0609ae5411 -> bc2505faa40d813d.
- **Tests.** `test_valence_device` VD-ANOM-1/2: each kind's body and the declared schema, the mm target and its unit_id; `test_motion_arbiter`: the popAnomaly target flipped in a 50..450 mm window, the starved-stream settle's v_end in mm/s; the wall case whose knot lies 21 mm past the window now asserts the plan stays on the edge with no backstop (it asserted the plan left the window, which kin-traw ends). `test/fixtures/kinetic_trace.json` re-recorded once against the new pin (170 lines move); `kinetic.wasm` reproduces it bit for bit; the Neutrino wasm check passes at bc2505faa40d813d.

## 0.1.70 (2026-10-25)

### For users

- **Changing a setting while the machine moves no longer stops it.** A setting (speed, depth, any machine setting), a pairing change, a saved network, or the first start after an update, wrote to flash about 2 s later, and that write froze the motion control for about 18 ms: long enough for the carriage to stop hard wherever it was, then start again. The setting still takes effect at once; only saving it waits until the machine is at rest (nothing playing, or paused, or e-stopped, or motor power off).
- **A setting changed during motion is lost if power is cut before the machine comes to rest.** It is saved the moment the machine rests. `/diag` shows `save=pending:...` while a save waits.
- **A new firmware image is confirmed at its first rest.** If the machine starts moving within about 10 s of an update, the update confirms itself when the motion stops. Until then a power cut brings the previous firmware back.

### Technical

- **`kArbFlash`** (val-y3da, operator 2026-10-25: settings are adjusted during motion and never stop it). A per-mode flash-write rule beside `kArbFeed`: Run, Paused and Jog `rest`, Estop and Unpowered `open`, Returning and the five home modes `hold`. The census carries it (`MotionCensus::flash`); `motionFlashOk()` is `open`, or `rest` with `!motionRuns()` and no oscillation. Mechanism: with code in flash, an ESP32-P4 flash write disables the flash cache and IDF parks both cores' tasks for the write (`spi_flash/cache_utils.c`), so the steer cannot renew the 4 ms LP lease.
- **Every runtime flash write waits for `motionFlashOk()`**: the cfg and presets blobs (`ValenceDevice::tick()`, armed and coalescing), the trust ledger and the pgest clear (`TrustStore::tick()` holdWrites), the saved networks (`BoardStation::persist()` through `hubFlashHeld()`), and the image buy (`otaBuy()`, `ota::BuyGate` in `OtaArm.h`: owed once the liveness proof passes, taken at the first rest by the liveness loop or the hub task). A planned reboot writes everything and buys first. There is no buy deadline: only a reset before the first rest reverts an unbought image. Logged as `flash writes held: saved in RAM, written at rest` and `flash hold released after N ms`; `/diag` line 2 is `# save=none` or `# save=pending:cfg,presets,trust,nets,buy`.
- **Removed `MotionArbiter::stillFor()` and `motionStillFor()`** with `kPersistStillUs`: the persist gate was their only caller. Also the unused `_win_log_p`. val-ggj closed as superseded.
- **valence.pin -> 4127e70** (comment-only change under `lib/valence`).
- **Tests.** `test_motion_arbiter`: a write due mid-stream waits for rest and the stream never lapses the lease, the same write ungated lapses it at speed, and the rule per mode (a brake holds, ESTOP and power off open, a cycle and a RETURN hold). `test_valence_device` VD-PERSIST-1/2: a config-set during a stream applies at once and is written once at rest, an oscillation holds it, the mode's rule decides. `test_ota_arm` OA-11/12: the buy is owed, held through motion, taken once at rest.
- **Evaluated, not taken: a cache-safe steer** (val-mfs5). No task runs during a cache-off write, so it would have to be an IRAM ISR, and the P4 offers no FPU in ISRs; XIP from PSRAM (val-5gh) is the structural route.

## 0.1.69 (2026-10-25)

### For users

- **The carriage no longer moves by itself at start-up.** Since 0.1.67 every start glided the carriage about 55 mm when the saved stroke window did not include the position it started at. Now the saved window is taken as it is at start-up, after homing and when a trial setting reverts: a carriage outside it waits there, and the first command (a jog, a stream, a pattern) brings it in at that command's own speed and smoothness. Only a stroke window you change while the carriage is inside the old one still glides it in.

### Technical

- **`ArbEvent::WindowAdopt`** (val-5ozv.5, operator ruling 2026-10-25: motion is only ever commanded). `setWindow()` takes a `WindowPost` (`edit` or `adopt`, default `adopt`). The hub passes `edit` only for the push after a session's own config-set (trials included) and `adopt` for the boot push, the measured rail after a home cycle and a trial revert. The adopt cells (Run, Paused, Jog, Returning) run `ArbAct::adopt`: recheck that never starts a glide (a glide already in flight re-aims), logging `WINDOW: ... adopted, waits outside`; a cycle, ESTOP and power loss defer it as they defer an edit. An edit taken after an adoption that had not applied yet judges the carriage against the adopted window. The table is 11 modes by 26 events.
- **Cause.** On the bench the planner applied force_home's re-origin in one tick and the boot's window post in a later one, so the post was a `WindowEdit` that left the homed carriage at 0 outside [55, ...] and glided it. Both posts in one tick took the `Reorigin` cell and did not glide, so the move depended on task timing.
- **Tests.** `test_arbiter_table` checks the new column and drives every window case with both posts (86 of 86 legal cells reached). `test_motion_arbiter`: the bench's boot moves no step in 3 s whichever post the planner takes first, then a stream segment enters the window inside the input set and stays in; a jog from outside comes in at the jog set; an adoption after a home cycle moves nothing, and neither does an edit after it with the carriage still outside. `test_valence_device` VD-WIN-POST: a config-set and a trial are edits, a revert and the measured rail adopt. `kinetic_trace.json` unchanged.

## 0.1.68 (2026-10-25)

### For users

- No change in behavior. The diagnostics log keeps the whole record of a glide into a moved stroke window.

### Technical

- **The GLIDE log is two lines** (`GLIDE: [old] -> [new] mm, carriage X, D mm to T mm, N ms`, then `GLIDE why: ...`). The single line passed Geiger's 104-byte message limit, and `/diag` cut it after the carriage's first digit on the 0.1.67 bench.

## 0.1.67 (2026-10-25)

### For users

- **Changing the stroke window while the carriage sits outside it no longer leaves it stranded.** If a new window leaves the carriage outside, the machine glides it back in at jog speed and holds it inside the new edge; anything sent meanwhile waits for the glide to finish. An edit during homing, an e-stop or with motor power off waits and applies when that ends.
- **The start-up log shows one less false warning.** Bluetooth advertising at every normal start no longer logs "Failed to persist local IRK". Nothing about Bluetooth behavior changes.

### Technical

- **The arbiter's ownership is one mode and a transition table** (val-5ozv.1, no behavior change). `ArbMode` (Run, Paused, Jog, Returning, HomeQueued, the four seek phases, Unpowered, Estop) replaces the planner's `_returning` and the HomePhase atomic; it is written by the planner task alone and moved only by the `(mode, event)` rules in `MotionArbiter.h` (11 modes by 25 events). `kArbFeed` is what `accept()` admits in each mode. Every change logs one `MODE:` line (a burst of 8 per second, the rest counted); a cell with no rule is held, logged and counted. `env:native` defines `MOTION_ARB_STRICT`, so a suite reaching an illegal cell aborts. New suite `test_arbiter_table`.
- **Window edits per mode and the glide** (val-5ozv.2, operator ruling 2026-10-25). `WindowEdit` (setWindow()) and `Reorigin` (force_home) are separate events. Run reframes (knots re-target as window shares, a jog in flight keeps its mm target); Paused, Jog and Returning recheck; a home cycle, ESTOP and power loss defer until they end, a completed cycle's measured rail kept over the post's. A homed carriage the edit leaves outside, with nothing in flight carrying it in, glides in: `Profile::converge` into the jog set, then `Profile::point` to rest inside the window; intents wait for its end. `.claude/rules/motion-control.md` holds both.
- **Census**: `arb_mode`, `arb_transitions`, `arb_illegal`, `window_edits`, `window_held`, `glides`, `glide_max_mm`. `/diag` carries them as two `motion` `census` lines, logged at boot and on every change.
- **`CONFIG_BT_NIMBLE_HS_PVCY=n`** (val-64ir). The NimBLE host generated a local IRK at sync and tried to store it; with `SECURITY_ENABLE=n` the store write is compiled out (it needs `BLE_SM_SC`) and returns `BLE_HS_ENOTSUP` (rc=8). Nothing uses the IRK: there is no pairing, bonding or resolvable private address, and the advertisement and GATT sessions use the public address. Re-enable privacy together with security and `NVS_PERSIST` if pairing ever lands.

## 0.1.66 (2026-10-24)

### For users

- **Setting up the network has its own mode.** Hold PAIR for 10 seconds while the machine is idle with motor power off and it restarts into config mode: Bluetooth setup and the USB cable, no WiFi clients. It also starts there by itself when it has no saved network, or when it cannot join one within a minute of starting, and it restarts back into normal mode once a saved network answers. In normal mode Bluetooth only advertises where to find the machine on the network; it no longer accepts connections.
- **The USB cable is always trusted.** A computer on the USB cable gets full settings access without pairing, and can add a WiFi network without a pairing window. Nothing joins a network while the machine moves.
- **A radios-off setting.** Tuning / Radios turns off WiFi and Bluetooth from the next start; the machine then talks over the USB cable only. The config-mode hold turns them back on.
- **A move that ends on the edge of the stroke window lands smoothly.** It used to be cut short in its last few milliseconds; now every source (scripts, jogs, the oscillator, a late plan) stops at the edge inside the motion limits.
- **Bad tuning values can no longer freeze motion.** A zero, infinite or invalid limit falls back to the factory value instead of stalling the planner.
- **The 60AIM40F drive passes its probe** with Modbus control off.
- **Dense scripts plan faster and stay inside every limit** in the passages the planner could not finish before.
- **A new board profile** for a Stamp-P4 on the OSSM R+D board with its ESP32 as an I/O helper (not tested on hardware yet).

### Technical

- `kinetic.pin` -> `bdccf73` (Kinetic main): the pass cap's stalled-share step and bounded fallback judge (kin-f0yr), corner-ramp starts that stop asking (kin-5g6c), HARD point moves from the first asking piece when the cap stops a run (kin-pfbw), the moving origin's least-legal quarter trim and lead-ramp share (kin-vlk3), the speed share held per knot and bounded legality judging (kin-l97f). `KINETIC2_FINGERPRINT` unchanged (`0xb43b8ddce5ab576b`).
- `valence.pin` -> `a615ce0` (Valence main, RFC-121): the client's catalog reassembly block out of line with a 1 MiB default (val-qijf), the binding-aware tier and WELCOME identity `config_mode` (7), the JS client's 1 MiB budget and `identity.config_mode`.
- **The build guard covers `../Valence`.** `tools/kinetic_guard.py` also runs canon_lint's valence.pin rule, which now fails on an uncommitted change under the sibling's `lib/valence` (`pin-dirty`). `NUCLEUS_VALENCE_UNPINNED=1` opts out and stamps the label `-valence-unpinned`.
- **RFC-121 routes and config mode** (val-6058). `LinkMode.h` decides the boot route once: normal, radios off, or config mode by the gesture or by policy; the RFC-117 offline BLE fallback and its dwell are gone. BLE GATT runs in config mode only; normal mode advertises non-connectable with the endpoint tail (ipv4, ws_port) while the WebSocket is up. The gesture is PAIR held 10 s (`kConfigHoldMs`), accepted with motion idle and motor power off; the RFC-114 demote moves to the release of a 5 to 10 s hold. Policy: no credentials boots config mode; a normal boot with no address and no session after 60 s (`kJoinWindowMs`) reboots into it; policy config mode retries the saved list and reboots out once it has an address and no ready configure session is attached; the boot after skips the automatic entry (NVS `valence`/`next`). Every policy reboot sends GOODBYE REBOOTING and waits for idle motion and no join in flight.
- **The provisioning catalog**: config mode serves 0x0001, the safety trio, 0x000F and the network channels, copied entry for entry from the machine catalog (9 entries, 2,950 B). The machine catalog declares 0x0001. Machine etag `d5895d9c2c5bd88d` -> `0d182b0609ae5411`.
- **The cable is configure** through the library: `ValenceSerialPort` declares `BindingKind::serial`, `ValenceBle` `ble_gatt`, `ValenceWsPort` `websocket`; the hub tiers the serial session configure and grants a PAIR_REQ there at once (`pairing_mode` cable). `sessionsNow()` and the provisioning desk drop their own cable checks; the desk keeps the cable's window term. `hubBegin()` calls `setConfigMode()`. Neutrino's in-process link stays unspecified (ruling pending). wifi_join while a motion source runs is NACK SOURCE_CONFLICT; in config mode the join's ECHO carries `reboot_in_ms`.
- **Radios setting**: machine-modes (0x1030) `radios`, modes-set (0x3030) key 11 at configure, config blob v10. WELCOME `ws_port` and `ipv4` stay 0 without a WebSocket listener. Accessory headroom is capped at `catalog_max_entries` and `catalog_max_bytes`.
- **The wall** (val-vj1j, operator ruling 2026-10-23). `applyWall()` replaces a strip that leaves the backstop frame with a jerk-limited stop at its edge (else `certifiedStop()` up to `kHardStopMax` times the set, counted in `hard_stops`); the steer's clamp stays as the last line, counted as `wall_clamps` (status line `clamps=`). `.claude/rules/motion-control.md` describes the backstop.
- **Non-finite inputs** (val-xrj0). `fenceCount()` clamps to 2^22 steps and reads NaN as the datum; `setWindow()` refuses a span that is not finite; `limitsFor()` gives a zero, NaN or infinite ceiling the factory value.
- **AimDrive probe**: output enable (0x01) passes on any nonzero value; the 60AIM40F reads 7 with Modbus off.
- **nucleus-p4b** (val-omh3.1 to .3): env `flagship_p4_barnacle` (`-p4hub-barnacle`, Kconfig `NUCLEUS_BOARD_BARNACLE`), the barnacle pin profile, `lib/barnacle_link` (COBS + CRC-16 frames, the trip detector), the helper link and home-sense doors in `system/BarnacleLink.cpp`, `SelfCheck` `Result::absent`, and the `barnacle_shim` project for the R+D board's WROOM-32D. Host-tested only (val-omh3.6 holds the bring-up).
- `test/fixtures/kinetic_trace.json` re-recorded once against the pinned Kinetic, after all the merges.

## 0.1.65 (2026-10-23)

### For users

- **Busy scripts plan faster.** The motion planner works out a dense run of moves in about half the time it took before, so a script with many short strokes keeps its schedule more easily. A few dense passages render slightly differently, still inside every limit.

### Technical

- `kinetic.pin` -> `9b0117e` (Kinetic main: the emitRun week, kin-f0yr, and the render() look-ahead test fix, kin-jr8r). The solver memoizes each piece's judge in the axis Workspace (`JudgeMemo`, 20.6 KB per axis), builds each knot's piece once a pass, judges a start-corrected piece at its exact peaks (`overCorrected`, which moves `KINETIC2_FINGERPRINT` to `0xb43b8ddce5ab576b`), and gives a cascade's predecessors its share in one pass (worst 64 corpus windows: 949 passes to 311, host solve time 44% of before). kin-jr8r touched the suite only: the oscillator plans were short of `render()`'s look-ahead; the firmware's plan buffer (`kPlanExt`) was already sized to it.
- **The build refuses an unpinned planner.** `tools/kinetic_guard.py`, a pre-script of every `flagship_p4*` env and `env:native`, fails the build unless `../Kinetic` is at `kinetic.pin` with the paths Nucleus compiles clean (canon_lint's rule). `NUCLEUS_KINETIC_UNPINNED=1` builds anyway for planner development, warns, and stamps the git label `-kinetic-unpinned` (`/diag` `git=`). The 0.1.63 bench image had compiled whatever branch sat in `../Kinetic` (`.claude/rules/build-test-deploy.md`).
- `test/fixtures/kinetic_trace.json` re-recorded against the pinned Kinetic: the committed one had been recorded against an unpinned branch (299 lines move). `kinetic.wasm` reproduces it bit for bit; the Neutrino wasm check passes unchanged.

## 0.1.64 (2026-10-23)

### For users

- **A machine that restarts on its own says why.** After an unexpected restart, the diagnostics page now names what was stuck (the task the watchdog caught) or what asked for the restart, and keeps the warnings that led up to it instead of letting a once-a-minute reminder push them out. When nothing survived, it says so, which on the bench means the power was cut.

### Technical

- **The crash ring has two copies** (val-a2ub). `system/CrashRing.h` (hardware-free, suite `test_crash_ring`) is the ring; `ValenceDiag.cpp` keeps it in LP RAM (`RTC_NOINIT_ATTR`, uncached) and in HP SRAM (`__NOINIT_ATTR`, 640 B, written back from the L1 cache after every write), adopts the first valid one at boot and reports which held: `/diag` reads `(LP RAM, survived the reset)`, `(HP SRAM, ...)`, or `no breadcrumbs survived the reset`; the boot line reads `LP RAM held|lost, HP SRAM held|lost`. Layout magic `NUC1` -> `NUC2`: the first boot after this update reads no breadcrumbs.
- **Last words.** IDF's `esp_task_wdt_isr_user_handler()` writes the starved tasks and what each core was running before the task watchdog's panic; an `esp_register_shutdown_handler()` hook writes the task that called `esp_restart()` and the uptime, which names esp_hosted's own restart on an SDIO transport failure (`CONFIG_ESP_HOSTED_TRANSPORT_RESTART_ON_FAILURE=y`). `/diag` prints it as `last words:`.
- **The ring keeps distinct lines.** A Warn+ line already in the ring is not added again, so the self-check's once-a-minute `motor power held OFF` (Error) no longer evicts the lines that name a fault: an 8-crumb ring filled with it inside 8 minutes on the bench.
- **The C6's silence is a breadcrumb.** The 5 s health loop logs `RSSI read over esp_hosted failed: <err>` or `took over 1000 ms` at Warn, at most once a minute.
- Investigated, not changed: the 2026-10-22 bench reset (`boot=1 reset=other-wdt`, no breadcrumbs, the board dark for 33 to 47 minutes before it) matches the bench's known cold-power-on signature (val-2a8) and not any 0.1.63 code path; the MotionArbiter brake tail and handover hold no unbounded loop or long lock. Esp-IDF 5.5.4 maps every hardware watchdog reset the panic handler did not convert (MWDT, RTC WDT, super WDT) to `ESP_RST_WDT`.

## 0.1.63 (2026-10-22)

### For users

- **A late planner never stops the carriage dead.** When the planner falls behind while the machine moves, the carriage now comes to a smooth stop inside its speed, acceleration and jerk limits instead of stopping within one millisecond, and it never lunges to catch up when the planner is back.
- **Late moves join smoothly or not at all.** A move that arrives too late to take over from where the machine is now blends in inside the limits. When no smooth blend exists the machine finishes a smooth stop, refuses new moves until it is there, and starts again from that rest. The status line counts both.
- **No stutter on a window change mid-stroke**, and PAUSE fades a running oscillation out instead of snapping it.

### Technical

- **The strip's tail** (val-cy1g). Every published strip carries a stop past its last entry (`PlanStrip::tail`, a `kinetic2::Profile`): the fastest jerk-limited stop from the state the strip ends in (the plan, the oscillator and the connector summed, read off the grid as the steer renders it), under the set the plan renders under, certified at the fill (`certifiedStop()`): never a reversal, at rest at its end, inside the backstop frame, or braked harder to the edge up to `kHardStopMax` (16) times the set's accel and jerk and counted. Past the strip's end the steer plays it instead of holding the last entry.
- **The handover** (val-xfy2). The steer's late-plan rebase and the residual kick that closed its gap are gone. A whole refill starts at the clock after the solve and is checked against the trajectory the steer was rendering (`handover()`): a plan on time joins as it is; otherwise the trajectory is kept 2 ticks and joined through a connector, the gap brought to rest by `Profile::point()` and summed with the plan under the set, half or a quarter of it, the first whose sum holds every ceiling and the frame on the 1 ms grid; else the plan is refused, the trajectory stops on a certified stop, the engine is reset at its rest without a new generation and `accept()` refuses intents until it is there. Paths that leaned on the kick: late plans, the recovery after the tail, the reseed on a window move (no held tick now: each strip carries its frame and limits, so the steer never reads the window and parks only for a re-origin), the PAUSE cut of the oscillator (a fade, uncounted), a lease lapse (the steer parks and the planner re-plans from the carriage at rest), and the RETURN and jog supersede (both through the handover).
- **The residual** is measured against the previous command, which the feedforward trails by one tick by construction, not the plan now: the old kick saw that tick of travel as error and stayed on above about 5 mm/s. Deadband one step, at most one tick of the set's accel, reached under the accel the plan leaves free, changed by at most one tick of its jerk.
- **Census and status line**: `planner_stalls`, `late_plans`, `plans_refused`, `hard_stops` (`pstalls= lateplans= refused= hardstops=`). Busy holds while a connector or a refused plan's stop still moves the trajectory, so idle gates (OTA, settings writes, home) wait for it.
- Known edge: a move that ends exactly on the backstop's edge is still clipped in its last few ticks by the wall bound (val-1w8), which halves the velocity per tick over its last few ticks (val-1w8's 2-tick budget).
- `kinetic_trace.json` re-recorded. Suite `test_motion_arbiter`: 6 new cases (strip exhaustion mid-stroke and its recovery, a late plan, a late reversal refused, a window write, a PAUSE cut, a lapse) hold the steered velocity's change per tick to the set's accel and its change to the jerk; the five without the new counters failed on 0.1.62's arbiter (velocity steps of 0.23 to 1.3 m/s per ms).

## 0.1.62 (2026-10-20)

### For users

- Nothing to notice. The machine has room for a larger catalog of its own before an accessory loses its place.

### Technical

- **Catalog encode scratch 76 KiB -> 256 KiB** (val-1vs). `VALENCE_CATALOG_SCRATCH_BYTES` 77,824 -> 262,144 in `flagship_p4/valence_capacity.cmake`, in the PSRAM `HubBox`. The machine catalog has no fixed byte budget now: the 80% headroom rule on the scratch, `catalog_max_entries` (256, which still sets 6 accessories), `catalog_max_entry_bytes` (4,096 B) and `catalog_max_bytes` (1 MiB, Valence RFC-121, draft) bind. Free headroom 31,090 B -> 178,546 B; etag unchanged, `d5895d9c2c5bd88d`.
- Built and host-tested; not flashed yet.

## 0.1.61 (2026-10-20)

### For users

- **Dense scripts run without stalls.** The machine now gives its planner as much time as a new plan needs, up to 64 ms, instead of a fixed 4 ms. On the bench, a horizon that covers the solve removed the planner stalls and late-plan kicks on fast reversal streams, with no measurable delay on jogs.
- **"Home required" is right after a restart.** The safety status said homing was not needed after a reboot while every move was refused as not homed. It now always matches.

### Technical

- **Adaptive reaction horizon** (val-668m, operator ruling 2026-10-20). Per submit, the arbiter sets `react_us = clamp(2 x EMA(plan_us) + 4 ms, react_ms, min(64 ms, lead / 2))` and hands it to the engine through `setConfig` before the supersede and jog hand-offs. The lead is the knot's due time minus its arrival on the hub clock, EMA 1/4 per source; a source change restarts it. The catalog's `react_ms` (default 4 ms) is now the floor; its desc says so. Logged once per change, throttled. `flagship_p4/src/motion/MotionArbiter.{h,cpp}`, suite `test_motion_arbiter`.
- **home_required** (val-mvw4). The safety channel's `home_required` is derived from the census's homed on every refresh (`Hub::setHomeRequired` publishes on a change), so a cold start, a force_home and an e-stop that cuts power all read right. Suite `test_valence_device`, VD-HOME-COLD.
- **Bench images boot homed** (val-mvw4). The bench profile (`NUCLEUS_BENCH_NO_MOTOR`, env `flagship_p4_bench`) force-homes at boot and logs it. Release images never do this and never persist homed.
- Catalog etag `c19dff891e8d042d` -> `d5895d9c2c5bd88d` (the `react_ms` desc). The machine catalog encodes 31,169 B, 65 B over the 31,104 B machine budget in `flagship_p4/valence_capacity.cmake`; the 6 accessories still fit the encode scratch.
- Built and host-tested (native 49/49); not flashed yet.

## 0.1.60 (2026-10-19, bench image only)

### For users

- **No more dropped streams.** The machine could disconnect an app in the middle of a stream, once just 2 s after it joined.

### Technical

- **WS idle reaper** (val-ag22). `idleOut()` read `uint32(nowMs - lastRx)` with `nowMs` sampled before the slot walk, so a frame the httpd task stamped a millisecond later wrapped to about 4.29e9 ms and the hub reaped the client. The delta is signed now. Suite `test_ws_port`, WP-07. Bench: three 40 s segment streams at 50 ms knots, 0 slots reaped (0.1.58 reaped three). 0.1.59 was a bench build of the same fix; the release image went from 0.1.58 to 0.1.61.

## 0.1.58 (2026-10-18)

### For users

- **Firmware updates need a press on the machine.** Any app on your network can offer an update, but the machine accepts it only after a PAIR double press within 60 s of the request.
- **Updates wait for the machine to be idle.** An update is refused while anything moves or while motor power is on, and motor power stays off until the update finishes.
- Release firmware holds no update key.

### Technical

- **OTA arm flow** (val-ktal, operator ruling 2026-10-18). `POST /ota/arm` answers 202 with an id and a 60 s window (409 with a reason when refused); `GET /ota/arm?id=` reports pending, approved or expired; `POST /ota?id=` takes the raw image, 403 unless approved, 409 unless idle. One live request, 30 s to start the image after the press, one use, in the hardware-free `OtaArm.h` (suite `test_ota_arm`). A pending request owns the double press, so an approving press never also approves a configure request.
- **Idle gate.** Arm and image answer 409 unless motion is parked (census not busy, `step_q8` 0) and the motor switch is off or faulted. `canClearEstop()` refuses the release while an update is in flight.
- **Bench token path.** The `X-OTA-Token` override compiles only in the bench profile (`NUCLEUS_BENCH_NO_MOTOR`), which skips the press; the idle gate still applies. Measured: the token is absent from `flagship_p4`'s firmware.bin and present in `flagship_p4_bench`'s. `tools/ota.py` runs the arm flow for release images and the token for bench images.
- Port 80 handlers 4 -> 6; client sockets stay 3. About 20 B of new internal BSS.

## 0.1.57 (2026-10-18)

### For users

- **Advanced Penetration moves on the first Start.** From a fresh boot, master speed is 10 % (it was 0, so Start held still), max depth is 100 % (it was 10 %) and min depth stays 0 %. At 10 % it strokes the full window slowly, about 120 mm/s at the default ceiling.

### Technical

- The catalog defaults and `advpat::Settings` change together; the initial `min_depth` bound (100) matches the coupled pair. Catalog etag `30eb939bfbb4d965` -> `91c231e26fc260bd` (max depth) -> `c19dff891e8d042d` (master speed); the wasm check follows.

## 0.1.56 (2026-10-17)

### For users

- **Window and flip without configure.** An app with control access can now set the stroke window and flip. Every limit (jog and input ceilings, rail length, homing style and speed, schedule horizon, e-stop datagrams) still needs configure.
- More settings explain themselves: the pattern library roster, move position, home stroke and the provisioning fields have descriptions, and oscillator amplitude and active say what the value is.
- The built-in machine in Phosphor is named Neutrino.

### Technical

- **Write tiers** (RFC-119 amended 2026-10-17). The stroke window (`0x3000` keys 1, 2) and flip (`0x3030` key 8) drop their floor to control; every other key in those entries rises to configure through schema-field access. VD-TIER-MATRIX, VD-TIER-1b and VD-TIER-3 cover it. Catalog etag moves (Neutrino `30eb939bfbb4d965`).
- **Catalog descs** for the preset roster, jog, the force_home stroke and the provisioning payload (etag `268fb1c2fd233b8d` -> `b16115d6091122d2`, 31,092 B encoded).
- **Neutrino** (the device twin, now `sim/neutrino`, val-7ai). The hub name is Neutrino in WELCOME and DISCOVER; the native exe gains the dev tool channel (`--devtool-port`), per-client fault links and the physical inputs (val-sf7.7); a Wi-Fi drop drains the discovery port (val-6sa).
- `kinetic.pin` -> `ac3348e`, `valence.pin` -> `9b2675b` (spec prose only; `lib/valence` byte-identical).

## 0.1.55 (2026-10-15)

### For users

- **Only configure changes machine settings.** Control drives the machine inside the limits; pause and e-stop work at any access level.

### Technical

- **Write tiers** (val-aq5, Valence RFC-119 draft, operator ruling 2026-10-15). config-set, modes-set, kinetic-set and drive-set move to configure; machine-admin `save_config` and `servo_scan`, and home `force_home` with its stroke, are at configure; `clear_fault`, the home cycle, move, osc-set, the pattern writers and the three c2h streams stay at control. Suites VD-TIER-MATRIX (every writable entry against one table) and VD-TIER-LIVE. Catalog etag `18d4fd5ec0de7e5a` -> `b310da1445ea519a`. `valence.pin` -> `bb9258e`.
