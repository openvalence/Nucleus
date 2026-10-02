# valencesim -- the Nucleus device twin

A desktop binary that serves `valence.v1` the way the Flagship P4 does, so
client tests (Phosphor's `test/*.mjs`, `valence_probe.py`, anything built on
`valence-js`) run with no hardware. It is the device twin; Valence Bench
(`../Valence/hub/bench`) is the opposite tool, a catalog-shaped test double
with no device semantics.

## What is real and what is not

| Piece | Source | Status |
|---|---|---|
| Hub | `lib/valence` (`valence::Hub`), the pinned sibling | real |
| Catalog | `flagship_p4/src/hub/ValenceCatalog.h` + `boardFeatures()` | real, same etag as the board |
| Delegate and every STATE publisher | `flagship_p4/src/hub/ValenceDevice.cpp`, compiled verbatim | real |
| Motion planner | `lib/kinetic` (`kinetic::Engine` over vendored Ruckig) | real |
| Arbiter: gates, window clamp, limit sets, feedforward, census | `flagship_p4/src/motion/MotionArbiter.cpp`, compiled verbatim | real |
| Motion task plumbing | `src/SimMotion.cpp` | a ring on the one hub thread instead of FreeRTOS queues; census refreshed every pass, not at 50 Hz |
| Emitter and position truth | `src/SimMotion.cpp` | ideal: renders the arbiter's steering word exactly, so `late`, `resteers`, `catchups` and `stack_free` read 0 |
| Pattern generator: seven classic patterns, the advanced modulators, the brake | `flagship_p4/src/patterns/PatternEngine.cpp`, compiled verbatim | real |
| Pattern task plumbing | `src/SimPattern.cpp` | ticked every pass on the one hub thread instead of its own FreeRTOS task |
| Durable hub identity (WELCOME identity key 5, SPEC §6.3) | minted once like the board's `hub_iid` NVS key | persisted: `PREFIX.iid`, 8 bytes little-endian |
| Pattern presets (0x5220) | `PatternPresetStore` inside the delegate | persisted: `PREFIX.presets` holds the board's NVS `presets` blob, same debounce |
| `background_run` | the delegate's `PatternSettings` | in memory, same as the board: persisting it waits on an operator ruling (bd val-wcm) |
| WebSocket port | `../Valence/hub/bench/src/net/WsServerPort.cpp`, compiled from its home | real host binding |
| `/uitoken` token: slot table, rate gate, HMAC derivation | `flagship_p4/src/hub/UiTokenTable.h`, compiled verbatim | real |
| `/uitoken` endpoint | `src/SimUiToken.cpp` on IXWebSocket's HTTP server, a `std::mutex` for the board's spinlock | same contract, on 127.0.0.1 |
| Config and tuning persistence (0x1000, 0x1030, 0x1120-0x1122, cfg_gen) | `StoredState.h` codec, compiled verbatim | persisted: `PREFIX.cfg` holds the board's NVS `cfg` blob; a file stands in for NVS |
| Push-to-pair gesture | `--pairing-window` opens the hub's presence window at boot | the board's PAIR-button gesture is bd val-9u0.10; the twin follows it (bd val-sf7.6) |
| Geiger log lines from device code | `lib/geiger`'s host platform layer (`GEIGER_HOST_PLATFORM`), drained on the hub thread into the sim's log | real: device lines print beside the sim's own, stamped with the hub clock |
| Motor switch (`system/MotorSwitch.h` state machine) | `src/SimMotorSwitch.cpp` | absent by default: power on from boot, hub-status reads `on`, ESTOP is a halt that keeps home (`estop_cuts_power` false). `--motor-switch` runs the board's own machine on the hub clock with healthy readings (an RC pre-charge into 150 uF through 100 R), declares `estop_cuts_power` true, and enables at boot; `--msw-fault` injects one fault-line window |
| Hub-status (0x0006) heap figure | reported as 0 | the `deviceFreeHeapBytes()` contract in `ValenceDevice.h`: 0 where the host has no meaningful answer |

Parity is one-way: the firmware is never edited to close a sim gap. Anything
marked as a copy above is a drift seam and is tracked on the board.

## Build

WinLibs MinGW-w64 on PATH, from PowerShell:

```
$env:PATH = "C:\Users\Atlan\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin;" + $env:PATH
cmake -S sim/valencesim -B sim/valencesim/build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++ -DCMAKE_C_COMPILER=gcc
cmake --build sim/valencesim/build
```

Output: `sim/valencesim/build/valencesim.exe`, statically linked. The first
configure fetches IXWebSocket v11.4.6 and applies Valence Bench's
subprotocol-echo patch from `../Valence/hub/bench/cmake/`. The sibling
checkout must exist beside this repo (override with `-DVALENCE_ROOT=`).

## Run

```
valencesim [machine] [--port 82] [--http 80] [--homed] [--duration S]
           [--pairing-window] [--motor-switch [--msw-fault S]] [--state PREFIX]
           [--headless] [--no-mdns] [--enforce]
```

| Flag | Effect |
|---|---|
| `--port N` | WebSocket port, default 82 |
| `--http N` | `/uitoken` port on 127.0.0.1, default 80 |
| `--homed` | force_home at boot with the stored max rail, so motion is accepted at once |
| `--duration S` | exit after S seconds (0 = until Ctrl-C) |
| `--pairing-window` | open the presence window at boot: first knock on a fresh ledger gets configure |
| `--motor-switch` | model the board's motor switch: ESTOP cuts power and leaves the hub unhomed, `release` lands in PAUSE with `home_required` and starts the 150 ms pre-charge, `force_home` then `resume` runs again; motion is refused INTERLOCK until the switch reads `on` |
| `--msw-fault S` | with `--motor-switch`: MSW_FLT_N reads low from S seconds after boot for 2 s. The switch latches faulted, the hub latches ESTOP cause fault, `release` is refused CLEAR_REFUSED until the line clears |
| `--state PREFIX` | where the persisted blobs live (`PREFIX.cfg`, `PREFIX.presets`, `PREFIX.iid`); default `valencesim-state` beside the exe. Delete `.cfg` and `.presets` for factory values; deleting `.iid` makes the twin a different hub |
| `--headless`, `--no-mdns`, `--enforce`, `machine` | accepted for command-line compatibility; there is no TUI and no mDNS, and the device posture is the only one |

A HELLO that no pairing token and no live `/uitoken` vouches for lands at
`watch`, exactly as on the board (SPEC §12.2, §12.3). A test that sends intents
mints `/uitoken` per connect (`acquireToken` in valence-js); run
`valence-auth.mjs --expect-enforced` to prove the posture.

Kill any stale `valencesim` on 80/82 first, or a test talks to the wrong
binary.

## Verify

From `../Phosphor`: `node test/pairing-roundtrip.mjs` starts its own sim. For
the rest, start `valencesim machine --homed --duration 600`, then pass
`127.0.0.1 82` (or `--ip 127.0.0.1` / `--host 127.0.0.1`) to each test.
Per-file results are stamped on the board, not here.
