# Integral -- the Nucleus device twin

Formerly valencesim. Integral serves `valence.v1` the way the Flagship P4
does, so clients run and test with no hardware. It is the device twin; Valence
Bench (`../Valence/hub/bench`) is the opposite tool, a catalog-shaped test
double with no device semantics.

## Two fronts, one machine

The machine is `src/SimCore.{h,cpp}`: catalog, delegate, Hub, motion,
patterns, motor switch, the `/uitoken` table and persistence. It owns no
socket, no clock and no thread. Two fronts drive it:

| Front | Where | Used by |
|---|---|---|
| Native exe `valencesim` | `src/main.cpp`, `CMakeLists.txt` | Phosphor's test suites (`test/valence-sim.mjs` and the rest), the bench, the twin's pin recorder: a WebSocket server, `/uitoken` on HTTP, UDP discovery, files for state, the wall clock |
| `integral.wasm` + `integral.js` | `wasm/integral.cpp`, `wasm/CMakeLists.txt` | Phosphor's built-in machine, in process on every platform: a worker drives it over a MessagePort; no sidecar (bd val-atu, Phosphor ph-5u0g.1) |

## What is real and what is not

| Piece | Source | Status |
|---|---|---|
| Hub | `lib/valence` (`valence::Hub`), the pinned sibling | real |
| Catalog | `flagship_p4/src/hub/ValenceCatalog.h` + `boardFeatures()` | real, same etag as the board |
| Delegate and every STATE publisher | `flagship_p4/src/hub/ValenceDevice.cpp`, compiled verbatim | real |
| Motion planner | `../Kinetic` (`kinetic2::Engine`, Kinetic²), the sibling checkout `kinetic.pin` names | real |
| Arbiter: gates, window clamp, limit sets, feedforward, census | `flagship_p4/src/motion/MotionArbiter.cpp`, compiled verbatim | real |
| Motion task plumbing | `src/SimMotion.cpp` | a ring on the one hub thread instead of FreeRTOS queues; census refreshed every pass, not at 100 Hz |
| Emitter and position truth | `src/SimMotion.cpp` | ideal: renders the arbiter's steering word exactly, so `late`, `resteers`, `catchups` and `stack_free` read 0 |
| Generators: classic (seven patterns) and advanced (the modulators), two rail sources (RFC-093), the brake | `flagship_p4/src/patterns/PatternEngine.cpp`, compiled verbatim | real |
| Pattern task plumbing | `src/SimPattern.cpp` | ticked every pass on the one hub thread instead of its own FreeRTOS task |
| Durable hub identity (WELCOME identity key 5, SPEC §6.3) | minted once like the board's `hub_iid` NVS key | persisted: `PREFIX.iid`, 8 bytes little-endian |
| Pattern presets (0x5220) | `PatternPresetStore` inside the delegate | persisted: `PREFIX.presets` holds the board's NVS `presets` blob, same debounce |
| `background_run` | the delegate's `PatternSettings` | in memory, same as the board: persisting it waits on an operator ruling (bd val-wcm) |
| WebSocket port | native: `../Valence/hub/bench/src/net/WsServerPort.cpp`, compiled from its home; wasm: the host's MessagePort through `integral_send`/`integral_poll` | real host binding |
| UDP discovery responder (SPEC 13.8) | `flagship_p4/src/hub/ValenceDiscovery.cpp`, compiled verbatim (Winsock or POSIX here, lwIP on the board) | real: answers DISCOVER_PROBE on the registry port with the twin's name, `hub_instance_id`, WS port, version, etag and pairing window. The wasm front has no socket: its host hands each datagram to `integral_datagram`, which runs the `.h` responder and the ESTOP hook |
| `/uitoken` token: slot table, rate gate, HMAC derivation | `flagship_p4/src/hub/UiTokenTable.h`, compiled verbatim | real |
| `/uitoken` endpoint | `src/SimUiToken.cpp` (`serve`), on IXWebSocket's HTTP server in the exe and `integral_http` in wasm, a `std::mutex` for the board's spinlock | same contract; the exe binds 127.0.0.1 |
| Config and tuning persistence (0x1000, 0x1030, 0x1120, 0x1122, cfg_gen) | `StoredState.h` codec, compiled verbatim | persisted: `PREFIX.cfg` holds the board's NVS `cfg` blob; a file stands in for NVS |
| Push-to-pair gesture | `--pairing-window` opens the hub's presence window at boot; in wasm, `integral_pair_press` is a PAIR press the delegate takes as the board's (`ValenceDevice::serviceButtons`) | the press, not the button: no debounce or hold core (bd val-sf7.6) |
| Geiger log lines from device code | `lib/geiger`'s host platform layer (`GEIGER_HOST_PLATFORM`), drained on the hub thread into the sim's log | real: device lines print beside the sim's own, stamped with the hub clock |
| Log channel 0x0008 | a Geiger sink in `src/main.cpp`, Warn and above into `Hub::publishLog`, as the board's `system/ValenceLogBridge.cpp` | same contract: Warn floor, the hub's own truncation and replay ring; no task-name gate, because every drain here is on the hub thread |
| Motor switch (`system/MotorSwitch.h` state machine) | `src/SimMotorSwitch.cpp` | absent by default: power on from boot, hub-status reads `on`, ESTOP is a halt that keeps home (`estop_cuts_power` false). `--motor-switch` runs the board's own machine on the hub clock with healthy readings (an RC pre-charge into 150 uF through 100 R), declares `estop_cuts_power` true, and enables at boot; `--msw-fault` injects one fault-line window |
| Hub-status (0x0006) heap figure | reported as 0 | the `deviceFreeHeapBytes()` contract in `ValenceDevice.h`: 0 where the host has no meaningful answer |
| Drive link (RS485 Modbus to the drive, `system/ValenceDriveLink.h`) and DRV_ALM | `driveAlarmTake()` in `src/main.cpp` | absent: the twin has no drive, so DRV_ALM never asserts and the delegate's drive-alarm latch never fires; the latch itself is suite `test_valence_device`'s |

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
Valence and Kinetic checkouts must exist beside this repo (override with
`-DVALENCE_ROOT=` and `-DKINETIC_ROOT=`).

Linux and macOS: the same two `cmake` lines without the compiler flags (a
C++23 GCC or Clang, `cmake`, `ninja`, `git`; built with GCC 16 and with Clang
23 on libc++, macOS itself untried). Output `build/valencesim`; Linux GCC
links libstdc++ in. Phosphor's CI builds all three for its sidecar (Phosphor
`docs/BUILD.md`).

## Run

```
valencesim [machine] [--port 82] [--bind 0.0.0.0] [--http 80] [--homed] [--duration S]
           [--pairing-window] [--motor-switch [--msw-fault S]] [--state PREFIX]
           [--no-estop-udp] [--home-sense-at MM [--rail-end-at MM]] [--plan-delay-ms N]
           [--uncommissioned] [--no-discovery] [--discovery-port N]
           [--headless] [--no-mdns] [--enforce]
```

| Flag | Effect |
|---|---|
| `--port N` | WebSocket port, default 82 |
| `--bind ADDR` | WebSocket listen address, default `0.0.0.0` (every interface); `127.0.0.1` keeps the twin off the LAN (Phosphor's sidecar) |
| `--http N` | `/uitoken` port on 127.0.0.1, default 80 |
| `--homed` | force_home at boot with the stored max rail, so motion is accepted at once |
| `--uncommissioned` | boot as a first-run hub (RFC-079): the setup record is cleared, so stream and pattern motion are refused (pattern start NACKs `INTERLOCK`) until `config-set` writes have carried all eight keys; Manual moves still run. Without it the twin is COMMISSIONED whatever its state file says, so client tests stream at once |
| `--duration S` | exit after S seconds (0 = until Ctrl-C) |
| `--pairing-window` | open the presence window at boot: first knock on a fresh ledger gets configure |
| `--motor-switch` | model the board's motor switch: ESTOP cuts power and leaves the hub unhomed, `release` lands in PAUSE with `home_required` and starts the 150 ms pre-charge, `force_home` then `resume` runs again; motion is refused INTERLOCK until the switch reads `on` |
| `--msw-fault S` | with `--motor-switch`: MSW_FLT_N reads low from S seconds after boot for 2 s. The switch latches faulted, the hub latches ESTOP cause fault, `release` is refused CLEAR_REFUSED until the line clears |
| `--no-estop-udp` | RFC-053's runtime switch off at boot, over the stored machine-modes `datagram_estop` (a later write of it, configure tier, wins): an ESTOP datagram on the §13.8 port reads `disabled` and never latches, so a test run cannot be stopped by the LAN. Without it the twin latches on one, exactly as a raw 0xE5 over WS (`flagship_p4/src/hub/ValenceEstopDatagram.cpp`, compiled verbatim) |
| `--home-sense-at MM` | a home stop MM from the boot position, so home op 1 runs the board's own two-leg cycle (`MotionArbiter.cpp`, homing): the sense reads HIGH while the carriage is at or past MM on the side away from 0, and its rise reaches the arbiter's interrupt entry at tick resolution. The home leg runs toward 0, so the stop is negative (`--home-sense-at -120`), or positive with the flip on. Without it the twin has no sense line and home op 1 refuses `UNSUPPORTED_OP`, as the release board does |
| `--rail-end-at MM` | with `--home-sense-at`: the far stop the second leg stalls on, MM from the boot position on the other side of it; default the stored `max_rail` plus both 5 mm safety margins from the home stop. A completed cycle stores the usable rail, the distance between the two minus both margins, as `max_rail` (`--home-sense-at -120 --rail-end-at 380` measures about 490 mm) |
| `--plan-delay-ms N` | SIM-ONLY, a test fault with no board counterpart: every solve lands N ms after it starts, as a slow solve lands on the board's planner task. Intents are accepted on arrival; the plan tick that solves them runs N ms later with its strip anchored at the arrival, and intents that arrive meanwhile wait. The steer keeps rendering the strip it has, so a stream sees `LATE PLAN`, `PLANNER STALL` and a plan that runs out while its segments arrived on time: the HUB cause in Phosphor's link health classification (ph-9t5l). Default 0, no delay |
| `--no-discovery` | no UDP discovery socket: keeps a test run off the registry port |
| `--discovery-port N` | answer DISCOVER_PROBE on N instead of the registry's `udp_discovery.port`, so a test talks to this twin and no other. A port another process holds costs only discovery (logged), never the run |
| `--state PREFIX` | where the persisted blobs live (`PREFIX.cfg`, `PREFIX.presets`, `PREFIX.iid`); default `valencesim-state` beside the exe. Delete `.cfg` and `.presets` for factory values; deleting `.iid` makes the twin a different hub |
| `--headless`, `--no-mdns`, `--enforce`, `machine` | accepted for command-line compatibility; there is no TUI and no mDNS, and the device posture is the only one |

A HELLO that no pairing token and no live `/uitoken` vouches for lands at
`watch`, exactly as on the board (SPEC §12.2, §12.3). A test that sends intents
mints `/uitoken` per connect (`acquireToken` in valence-js); run
`valence-auth.mjs --expect-enforced` to prove the posture.

Kill any stale `valencesim` on 80/82 first, or a test talks to the wrong
binary.

## The wasm front

### Build

With the emsdk at `../.tools/emsdk` (the workspace's; `emsdk_env` or its
`upstream/emscripten` on PATH, plus Ninja):

```
emcmake cmake -S sim/valencesim/wasm -B sim/valencesim/wasm/build -G Ninja
cmake --build sim/valencesim/wasm/build
node sim/valencesim/wasm/check.mjs
```

Output `sim/valencesim/wasm/build/integral.js` (ES6 module factory
`createIntegral`) and `integral.wasm`, about 540 KB raw and 170 KB gzipped;
build products, never committed. Same sources and capacities as the exe, no
pthreads, no asyncify, no filesystem. `check.mjs` boots it, opens a
valence-js session over an in-memory socket, verifies the catalog against the
WELCOME etag (`--etag HEX` to expect another), and jogs to 80 mm.

### The C ABI

wasm32: pointers and `size_t` are u32; `now_us` is a BigInt. A pointer handed
out stays valid until the next call of the same function.

| Function | Contract |
|---|---|
| `integral_create(json_opts, state, state_len)` | Boots the machine. Options: `homed`, `pairing_window`, `uncommissioned`, `motor_switch` (bool), `msw_fault_s`, `home_sense_at_mm`, `rail_end_at_mm`, `plan_delay_ms` (number), the flags of the same names above. `state` is the blob `integral_state_get` last returned, or null. 1 booted, 0 refused. Once per module instance |
| `integral_tick(now_us)` | Runs every 1 ms pass up to `now_us` (any epoch, monotonic); more than 250 ms behind, the clock jumps and the arbiter's stall cap holds, as on a stalled board. The hub ticks every 5 ms of it. Returns bit0 when the state blob changed |
| `integral_connect(id)` | A socket opened: 1 attached, 0 refused (all five slots busy) |
| `integral_send(id, data, len)` | One WebSocket message (one frame, at most 512 B). 1 queued, 0 dropped (ring of 32 full, oversize, unknown id) |
| `integral_poll(id, &out, &len)` | 1: the next message to the client; 0: none; -1: the hub closed it (fire `onclose`; the id is free) |
| `integral_disconnect(id)` | The client closed; frames already sent are read at the next hub tick |
| `integral_http(method, path, body, len, &out, &out_len)` | Status and body. Only `GET /uitoken` exists (RFC-029 section 4); anything else is 404 |
| `integral_state_get(&out, &len)` | The persisted state as one blob: repeated `[u8 key][u32 LE length][bytes]`, keys 1 cfg, 2 presets, 3 hub_iid, the board's NVS blobs |
| `integral_destroy()` | Closes every client. The machine stays in memory; drop the module to free it |
| `integral_pair_press()` | One PAIR press: the presence window opens at the next hub tick |
| `integral_datagram(data, len, src_ipv4, ws_port, &out)` | One UDP datagram the host received on its SPEC 13.8 port, `src_ipv4` in host order, `ws_port` the WebSocket port it serves this machine on. Returns the length of the reply at `*out`, 0 for none, after the ESTOP hook (RFC-053) and the per-source limiter; send the reply to the datagram's source |

Log lines (the boot banner with the etag, device GLOG lines) arrive on the
module's `print`/`printErr`.

### The host contract

- The host owns time. `integral_tick` is the only thing that advances it;
  call it at least every few milliseconds with the host's monotonic clock.
  Ticking slower than the catalog's fastest rate (the 60 Hz motion STATE, a
  20 Hz jog) delays every STATE push and makes motion jump.
- After each tick, drain `integral_poll` for every open client.
- Persist: when `integral_tick` returns bit0, store `integral_state_get`'s
  blob (IndexedDB or localStorage) and pass it to the next `integral_create`.
  A machine started without it mints a new hub identity.
- One machine per module instance; a restart is a new instance.
- The HTTP side has one route; a session's token provider calls
  `integral_http("GET", "/uitoken")` and passes the hex token as bytes.

## Verify

From `../Phosphor`: `node test/pairing-roundtrip.mjs` starts its own sim. For
the rest, start `valencesim machine --homed --duration 600`, then pass
`127.0.0.1 82` (or `--ip 127.0.0.1` / `--host 127.0.0.1`) to each test.
Per-file results are stamped on the board, not here.
