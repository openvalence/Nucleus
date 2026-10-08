# kinetic.wasm

The P4's motion planner compiled to WebAssembly. A client uses it to render a
whole script ahead of time and to compare tuning against the firmware's output.
It is built from the planner's source files, listed below.

| Piece | Source (compiled by path, never copied) |
|---|---|
| The planner (Kinetic²): the knot timeline, the window solver, the brake | `../Kinetic/include/kinetic2/` |
| Intent path: window clamp, limit sets, mm to engine frame, rest reseed, tuning map, the intent-to-knot boundary | `flagship_p4/src/motion/MotionArbiter.cpp` |
| 0x2101 sample to intent | `flagship_p4/src/motion/StreamIntent.h` |
| Emitter (renders the steering word with ideal edge timing) | `sim/valencesim/src/IdealEmitter.h` |
| Clock, tick order, the C ABI | `kinetic_wasm.cpp` (plumbing only) |

The gates (e-stop, pause, override, power, ownership) compile in but are opened
once at create and are not exposed through the ABI; the hub performs gating.

## Build

```
emcmake cmake -S tools/kinetic-wasm -B tools/kinetic-wasm/build -G Ninja
cmake --build tools/kinetic-wasm/build
```

Output `tools/kinetic-wasm/build/kinetic.wasm`: standalone, zero imports, no
filesystem, no JS glue. It is a build product and is never committed here.

## The planner

The twin plans with Kinetic² (`kinetic2::Engine<1>`), the board's only planner
(operator ruling 2026-10-06, bd val-z1k). `kinetic_version()` names the kernel
and its version, so a consumer can tell which planner produced a render.

## The pin rule

A vendored `kinetic.wasm` matches a firmware image only when it was built from the
same Nucleus commit as the firmware it stands in for, with the sibling Valence
checkout at `valence.pin` and the Kinetic checkout at `kinetic.pin`.
`kinetic_version()` records the build:
`nucleus <git describe> kinetic2 <engine version>`. A `-dirty` build is never
vendored, and a consumer shows the string next to its render.

## The determinism rule

The same sequence of calls produces bit-identical output, because every build
of these sources compiles without floating-point contraction (`-ffp-contract=off`) and without
`-ffast-math`; the native suite does the same in `platformio.ini`. Never add
either flag to any build that claims to match. `node tools/kinetic-wasm/check.mjs` checks this against the fixture the native suite
`test/native/test_kinetic_wasm_trace` writes to `test/fixtures/kinetic_trace.json`:
every 1 ms sample of a 60 s script is compared bit for bit:

```
node tools/kinetic-wasm/check.mjs tools/kinetic-wasm/build/kinetic.wasm test/fixtures/kinetic_trace.json
```

## Loading

```js
const { instance } = await WebAssembly.instantiate(bytes, {});
const k = instance.exports;
k._initialize();                       // static constructors, once
const h = k.kinetic_create(1000, 50000, 2000000, 500, 0);
const out = k.malloc(64);
k.kinetic_set_window(h, 100, 400);
k.kinetic_submit_segment(h, 7500, 250, -32768, 120000);
k.kinetic_step(h, 0.001, out);         // read the struct below at `out`
```

## The C ABI

All times are the handle's own engine clock, microseconds from create; it
starts at 0 and only `kinetic_step` advances it. Millimeters are the client
frame of the travel window. The engine frame is that window normalized to
0..1.

| Function | Contract |
|---|---|
| `kinetic_create(vmax_mm_s, amax_mm_s2, jmax_mm_s3, rail_mm, horizon_ms)` | The input ceiling set, the rail, and the grant's schedule horizon (0 = `max_future_schedule_ms`, 250). Homed at 0 mm, window = the whole rail, factory tuning. Null on a non-finite or non-positive limit. |
| `kinetic_set_window(h, lo_mm, hi_mm)` | 1 applied, 0 refused. Like the board, the next tick parks and reseeds at rest. |
| `kinetic_expect(h, window_ms)` | The window each later segment expects successors for (`MotionIntent::expect_us`, Kinetic `Engine::expect`): its free knot renders through toward a provisional successor instead of at rest. Create sets the hub's, the larger of `stream_quiet_release_ms` (500) and the horizon; 0 renders every free knot at rest. Kept by `kinetic_reset`. |
| `kinetic_default_tuning(out)` / `kinetic_set_tuning(h, t)` | The factory set, and a whole set applied before the next segment (`kinetic_tuning` below). |
| `kinetic_submit_segment(h, pos_e4, dur_ms, end_vel_e3, start_us)` | One 0x2101 segment, planned at the current clock. 1 accepted, 0 refused by the planner, -1 zero duration (dropped, as the hub drops it). |
| `kinetic_step(h, dt_s, out)` | Advances the clock by `dt_s`, rounded to whole microseconds, evaluates, and writes one `kinetic_sample`. The board ticks at 1 ms. |
| `kinetic_reset(h)` | Back to the create state at t = 0, keeping limits, window and tuning. |
| `kinetic_now_us(h)`, `kinetic_destroy(h)`, `kinetic_version()` | The clock; release; the identity string (static storage). |

### Segment fields, in wire order

| # | Field | Unit |
|---|---|---|
| 1 | `pos_e4` u16 | 1e-4 of the window (0..10000) |
| 2 | `dur_ms` u16 | milliseconds, nonzero |
| 3 | `end_vel_e3` i16 | 1e-3 window per second; -32768 = unspecified (SPEC 5.4) |
| 4 | `start_us` f64 | the segment's start on the engine clock, whole microseconds. A start in the past is due now; one beyond `horizon_ms` is clamped to it (SPEC 5.4) |

A player stamps each segment ahead of its start, inside the horizon, exactly as
it would on the wire. A segment is a knot at its start plus its duration, a
start past the newest knot holds at rest until it, and a knot not after the
newest is refused (KnotRefused): nothing queued is ever replaced.

### `kinetic_sample`, 64 bytes, little-endian

| Offset | Field | Unit |
|---|---|---|
| 0 | `t_us` f64 | engine clock |
| 8 | `p` f64 | plan position, engine frame, UNCLAMPED (the engine's float, widened) |
| 16 | `v` f64 | engine frame units/s |
| 24 | `a` f64 | engine frame units/s^2 |
| 32 | `plan_mm` f32 | plan position as the census publishes it, window-clamped, mm |
| 36 | `velocity_mm_s` f32 | census plan velocity |
| 40 | `accel_mm_s2` f32 | `a` times the window span (unclamped) |
| 44 | `position_mm` f32 | the ideal emitter's count: what an on-time LP core renders |
| 48 | `target_mm` f32 | where the active plan ends |
| 52 | `anomalies` u32 | bit k = `kinetic2::AnomalyKind` k recorded since the previous step, the motion-anomaly kinds (3 is KnotTrimmed; 5 is KnotRefused; 6 is PieceOverCeiling, a piece no trim keeps inside a limit, rendered at its least-over trim, never a drop) |
| 56 | `mode` u8 | 0 idle, 1 toward a segment's knot, 2 toward a sample's, 3 a brake (the 0x1111 `mode` select's ordinals) |
| 57 | `plan_kind` u8 | 0 none, 1 bezier (the 0x1111 `plan_kind` select's ordinals) |
| 58 | `flags` u8 | bit0 busy, bit1 shaped (a knot trimmed toward its predecessor), bit2 clamped (raw `p` outside the window), bit3 refused (a submit since the previous step, or a refused knot) |
| 59 | reserved u8 | 0 |
| 60 | `plans` u32 | successful plans since create or reset |

### `kinetic_tuning`, 32 bytes

`MotionTuning` (`flagship_p4/src/motion/ValenceMotion.h`) as the 0x3120
writer speaks it, Kinetic²'s set (Valence RFC-108 item 7). The offsets are
fixed:

| Offset | Member |
|---|---|
| 0, 4, 8 | f32 `jmax_ovr`, `vmax_ovr`, `amax_ovr` |
| 12 | u32 `chase_dense_us` |
| 16 | u32 `react_us` (the reaction horizon, below) |
| 20 | f32 `smoothness` (0 crisp .. 1 smooth) |
| 24 | f32 `handle_floor` (0.05..0.33) |
| 28 | f32 `trim_max` (0.1..1) |

Start from `kinetic_default_tuning` and change only what the card changed.
The members map onto `kinetic2::Config` as follows (this ABI takes segments
only):

| Member | Effect |
|---|---|
| `jmax_ovr`, `vmax_ovr`, `amax_ovr` | the ceilings |
| `smoothness`, `handle_floor`, `trim_max` | `Config::smoothness`, `Config::handle_floor`, `Config::trim_max`: how free knots render, the shortest handle a ceiling fit may leave (share of the piece), and the farthest a knot is trimmed toward its predecessor (share of the window span) |
| `react_us` | `Config::react_us`, the reaction horizon in microseconds (factory 4000): a knot arriving while the carriage moves keeps the curve under it this far ahead of now, or through the next knot when that is nearer, and re-plans from the state there (RFC-105 (bb)). A longer horizon avoids re-planning inside a piece too short to change within the ceilings. A shorter horizon lets the next knot revise a plan that was based on one knot |
| `chase_dense_us` | ignored here; on the board it sets the samples grant's `schedule_latency_us` (`sampleLatencyUs()`), the delay a sample renders at |

## Differences from the board

The emitter is ideal and the tick is exact, so the board's task wake-up jitter
and the LP core's edge quantization are not modeled. The plan is identical to
the board's. The hub's quiet release, which re-solves a stream's pending knots
without the expectation when its session drops, has no call here.

## The lab's calls

`tools/kinetic-lab` drives the twin in the board's own wake order, with what
the hub adds to a segment, and reads the solver's decisions:

| Call | What |
|---|---|
| `kinetic_submit_segment2(h, pos_e4, dur_ms, end_vel_e3, start_us, supersede)` | `kinetic_submit_segment` with the RFC-087 supersede flag the hub sets on the first segment of a bundle the motion path takes (`ValenceDevice::onStreamBundle`) |
| `kinetic_submit_manual(h, target_mm)` | a Manual point, the jog as the hub submits it: window-held, the jog set, live |
| `kinetic_advance(h, dt_s)`, `kinetic_evaluate(h, out)` | `kinetic_step` split at the board's wake (`MotionTask::run`): the clock moves, the intents that arrived by then are accepted at that reading, then the tick evaluates |
| `kinetic_pending(h)`, `kinetic_solved(h, i, out)` | the knots still ahead and the i-th as the solver placed it, a 40 B `kinetic_knot` (time, p, v, a, the share a trim kept, a live jog's seconds late, the worst ceiling ratio, clamped; dropped and the pins are always 0). Read after `kinetic_evaluate`: a read between a submit and the tick solves early |

`kinetic_step` is unchanged (`advance` then `evaluate`), so a consumer that
submits on the tick renders the same bits as before.
