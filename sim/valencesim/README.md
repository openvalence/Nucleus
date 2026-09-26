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
| Arbiter gates | `src/SimMotion.cpp` | host copy of `ValenceMotion.cpp`'s `MotionArbiter` |
| Emitter and position truth | `src/SimMotion.cpp` | ideal: the plan, quantized to steps |
| WebSocket port | `../Valence/hub/bench/src/net/WsServerPort.cpp`, compiled from its home | real host binding |
| `/uitoken` | `src/SimUiToken.cpp` on IXWebSocket's HTTP server | same contract, random bytes instead of HMAC |
| Config persistence | none | in memory for the life of the process |
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
           [--pairing-window] [--enforce] [--headless] [--no-mdns]
```

| Flag | Effect |
|---|---|
| `--port N` | WebSocket port, default 82 |
| `--http N` | `/uitoken` port on 127.0.0.1, default 80 |
| `--homed` | force_home at boot with the stored max rail, so motion is accepted at once |
| `--duration S` | exit after S seconds (0 = until Ctrl-C) |
| `--pairing-window` | open the presence window at boot: first knock on a fresh ledger gets configure |
| `--enforce` | an unvouched HELLO lands at `watch`, exactly as on the board |
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
