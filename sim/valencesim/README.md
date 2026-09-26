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
| Pattern generator: seven classic patterns, the advanced lanes, the brake | `flagship_p4/src/patterns/PatternEngine.cpp`, compiled verbatim | real |
| Pattern task plumbing | `src/SimPattern.cpp` | ticked every pass on the one hub thread instead of its own FreeRTOS task |
| Pattern presets (0x5220) | `PatternPresetStore` inside the delegate | persisted: `PREFIX.presets` holds the board's NVS `presets` blob, same debounce |
| `background_run` | the delegate's `PatternSettings` | in memory, same as the board: persisting it waits on an operator ruling (bd val-wcm) |
| WebSocket port | `../Valence/hub/bench/src/net/WsServerPort.cpp`, compiled from its home | real host binding |
| `/uitoken` | `src/SimUiToken.cpp` on IXWebSocket's HTTP server | same contract, random bytes instead of HMAC |
| Config and tuning persistence (0x1000, 0x1030, 0x1120-0x1122, cfg_gen) | `StoredState.h` codec, compiled verbatim | persisted: `PREFIX.cfg` holds the board's NVS `cfg` blob; a file stands in for NVS |
| Push-to-pair gesture | `--pairing-window` opens the hub's presence window at boot | the board has no gesture yet |
| Geiger log lines from device code | mute on host (Geiger has no host platform layer) | sim's own lines print |
| Hub-status heap figure | reported as 0 | no meaningful host answer |

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
           [--pairing-window] [--enforce] [--state PREFIX] [--headless] [--no-mdns]
```

| Flag | Effect |
|---|---|
| `--port N` | WebSocket port, default 82 |
| `--http N` | `/uitoken` port on 127.0.0.1, default 80 |
| `--homed` | force_home at boot with the stored max rail, so motion is accepted at once |
| `--duration S` | exit after S seconds (0 = until Ctrl-C) |
| `--pairing-window` | open the presence window at boot: first knock on a fresh ledger gets configure |
| `--enforce` | an unvouched HELLO lands at `watch`, exactly as on the board |
| `--state PREFIX` | where the persisted blobs live (`PREFIX.cfg`, `PREFIX.presets`); default `valencesim-state` beside the exe. Delete both for factory values |
| `--headless`, `--no-mdns`, `machine` | accepted for command-line compatibility; there is no TUI and no mDNS |

**The one deliberate difference from the board:** without `--enforce`, a HELLO
that no pairing token and no live `/uitoken` vouches for lands at `control`,
not `watch`. Phosphor's device tests were written against that floor
(`pairing-roundtrip.mjs` asserts it). Run with `--enforce` and
`valence-auth.mjs --expect-enforced` to prove the device posture.

Kill any stale `valencesim` on 80/82 first, or a test talks to the wrong
binary.

## Verify

From `../Phosphor`: `node test/pairing-roundtrip.mjs` starts its own sim. For
the rest, start `valencesim machine --homed --duration 600`, then pass
`127.0.0.1 82` (or `--ip 127.0.0.1` / `--host 127.0.0.1`) to each test.
Per-file results are stamped on the board, not here.
