# kinetic.wasm -- the machine's planner, offline

The P4's own motion planner compiled to WebAssembly, so a client can render a
whole script through it ahead of time and tune against exactly what the
machine will do. It is not a model of the planner: it is the same source.

| Piece | Source (compiled by path, never copied) |
|---|---|
| Waveform shaping, Blend/Stretch, ceiling and window scan, Ruckig guard | `../Kinetic/include/kinetic/kinetic.hpp` + `../Kinetic/third_party/ruckig/src` |
| Intent path: window clamp, limit sets, mm to engine frame, rest reseed, tuning map | `flagship_p4/src/motion/MotionArbiter.cpp` |
| 0x2101 sample to intent | `flagship_p4/src/motion/StreamIntent.h` |
| Emitter (renders the steering word, every edge on time) | `sim/valencesim/src/IdealEmitter.h` |
| Clock, tick order, the C ABI | `kinetic_wasm.cpp` (plumbing only) |

The gates (e-stop, pause, override, power, ownership) compile in but are opened
once at create and never exposed: they are the hub's business, not a
renderer's.

## Build

```
emcmake cmake -S tools/kinetic-wasm -B tools/kinetic-wasm/build -G Ninja
cmake --build tools/kinetic-wasm/build
```

Output `tools/kinetic-wasm/build/kinetic.wasm`: standalone, zero imports, no
filesystem, no JS glue. It is a build product and is never committed here.

## The pin rule

A vendored `kinetic.wasm` is only "the machine" when it was built from the
same Nucleus commit as the firmware it stands in for, with the sibling Valence
checkout at `valence.pin`. `kinetic_version()` carries the proof:
`nucleus <git describe> kinetic <engine version>`. A `-dirty` build is never
vendored, and a consumer shows the string next to its render.

## The determinism rule

Same calls in, same bits out. That holds because every build of these sources
compiles without floating-point contraction (`-ffp-contract=off`) and without
`-ffast-math`; the native suite does the same in `platformio.ini`. Never add
either flag to any build that claims to match. The proof is
`node tools/kinetic-wasm/check.mjs` against the fixture the native suite
`test/native/test_kinetic_wasm_trace` writes to `test/fixtures/kinetic_trace.json`:
every 1 ms sample of a 60 s script is compared bit for bit.

## Loading

```js
const { instance } = await WebAssembly.instantiate(bytes, {});
const k = instance.exports;
k._initialize();                       // static constructors, once
const h = k.kinetic_create(1000, 50000, 2000000, 500, 0);
const out = k.malloc(64);
k.kinetic_set_window(h, 100, 400);
k.kinetic_submit_segment(h, 7500, 250, -32768, 120000, 1);
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
| `kinetic_default_tuning(out)` / `kinetic_set_tuning(h, t)` | The factory set, and a whole set applied before the next segment (`kinetic_tuning` below). |
| `kinetic_submit_segment(h, pos_e4, dur_ms, end_vel_e3, start_us, curve_family)` | One 0x2101 segment, planned at the current clock. 1 accepted, 0 refused by the planner, -1 zero duration (dropped, as the hub drops it). |
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
| 5 | `curve_family` u8 | the GRANTED family (registry `curve_families`: 0 unspecified, 1 c1_cubic, 2 c2_quintic) |

A player stamps each segment ahead of its start, inside the horizon, exactly as
it would on the wire; the engine plans a future start from where the previous
one ends (`Engine::commit`). A segment re-sent at an earlier start replaces everything
queued from that start on.

### `kinetic_sample`, 64 bytes, little-endian

| Offset | Field | Unit |
|---|---|---|
| 0 | `t_us` f64 | engine clock |
| 8 | `p` f64 | plan position, engine frame, UNCLAMPED (the double the engine evaluates) |
| 16 | `v` f64 | engine frame units/s |
| 24 | `a` f64 | engine frame units/s^2 |
| 32 | `plan_mm` f32 | plan position as the census publishes it, window-clamped, mm |
| 36 | `velocity_mm_s` f32 | census plan velocity |
| 40 | `accel_mm_s2` f32 | `a` times the window span (unclamped) |
| 44 | `position_mm` f32 | the ideal emitter's count: what an on-time LP core renders |
| 48 | `target_mm` f32 | where the active plan ends |
| 52 | `anomalies` u32 | bit k = `kinetic::AnomalyType` k recorded since the previous step |
| 56 | `mode` u8 | `kinetic::Mode`: 0 idle, 1 waveform, 2 chase, 3 settle |
| 57 | `plan_kind` u8 | `kinetic::PlanKind`: 0 none, 1 quintic, 2 Ruckig, 3 cubic |
| 58 | `flags` u8 | bit0 busy, bit1 shaped (Blend spent amplitude or shape), bit2 fallback (Ruckig guard or a stretched deadline), bit3 clamped (raw `p` outside the window), bit4 refused (a submit since the previous step) |
| 59 | reserved u8 | 0 |
| 60 | `plans` u32 | successful plans since create or reset |

### `kinetic_tuning`, 52 bytes

`MotionTuning` (`flagship_p4/src/motion/ValenceMotion.h`) as the 0x1030 /
0x3120 cards speak it, in this order: nine f32 (`jmax_ovr`, `vmax_ovr`,
`amax_ovr`, `chase_gain`, `chase_lookahead`, `handoff_k`, `smooth_budget`,
`amplitude_budget`, `overshoot_guard`), two u32 (`chase_dense_us`,
`settle_grace_us`), then u8 `chase_ff`, `chase_accel_ff`, `chase_aim_extrap`,
`curve_policy` (0 follow, 1 C1, 2 C2), `infeasible_policy` (offset 48; 0
stretch, 1 blend), `blend_steps`, and two reserved zero bytes. Start from
`kinetic_default_tuning` and change only what the card changed.

## What it is not

The emitter is ideal and the tick is exact, so the timing jitter of the
board's task wake-ups and the LP core's edge quantization are absent; the
plan, which is what the planner decides, is the machine's.
