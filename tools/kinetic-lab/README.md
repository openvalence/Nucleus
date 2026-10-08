# Kinetic² lab

A local page that runs a funscript through the machine's own motion path and
shows what the planner did, with every tuning field live. Nothing reaches the
machine until it renders correctly here (operator ruling 2026-10-06, bd val-b7m).

| Piece | Where it runs from |
|---|---|
| The planner: MotionArbiter + Kinetic² + the wire decode + the ideal emitter | `tools/kinetic-wasm/build/kinetic.wasm`, the sources the P4 links |
| The player: parse, the curve modes, the scheduler (spans, offers, end velocities) | `../Phosphor/plugins/factory/funscript-player/{funscript,interp,scheduler}.js`, served verbatim at `/player/` |
| The host between them: the segments door, the hub's anchor and supersede rules, the motion task's wake order, the virtual clocks | `host.js` (a Worker) |
| The page: controls, the plot, the stats, recordings | `lab.js`, `index.html`, `lab.css` |

## Run it

Build the twin once (`tools/kinetic-wasm/README.md`), then:

```
node tools/kinetic-lab/serve.mjs
```

Open the printed URL. The default run is a built-in PCHIP sine at the factory
limits and tuning; pick another built-in, open a `.funscript`, or drop one on
the page. Every change (curve mode, range, window, limits, tuning, link) renders
the whole run again and redraws.

`node tools/kinetic-lab/check.mjs` drives the page headless (Playwright from
`../Phosphor/node_modules`, or `PLAYWRIGHT_DIR`): zero page errors, the sine
runs with nothing dropped, a saved recording replays the plan bit for bit, the
Nucleus trace fixture loads as a recording. Screenshots land in `SHOTS_DIR`
(default: the OS temp dir, `kinetic-lab/`).

## What a run is

Time 0 is the player's and the hub's clock together. The player prerolls to the
first knot (`scheduler.preroll`), then plays: once per player tick (16.7 ms, an
animation frame) the scheduler offers the spans ahead and the door packs one
bundle of those starting within half the horizon, stamped on the client's
hub-clock estimate (off by `clock error`), which arrives `network` ms later. On
arrival the hub resolves each stamp against its clock (a past stamp is due now,
one beyond the horizon is clamped), sets RFC-087 supersede on the bundle's first
accepted segment, and the motion task wakes, accepts, and evaluates. The task
also evaluates on every 1 ms tick; those ticks are the samples plotted.

The plot: the actions (gray), the player's curve through the range (purple: the
mode, smoothing, slew and scale, what Phosphor draws), the plan (blue), the
error between them (pink lane, its own scale), anomalies as ticks at the top,
bundle arrivals at the bottom with the knots' wire starts under them, and when
zoomed in every knot the solver placed: a ring where it spent (orange trimmed,
red stretched, magenta dropped). The stats are over the script's own span, not
the preroll or the tail.

## Recordings

"Save recording" writes what the hub received: `[arrival ms, "seg", pos_e4,
dur_ms, end_vel_e3, start_us, family, supersede]` per segment, plus the
machine, link and tuning. Loading one replays it without the player (the curve
is then unknown; the plan, spends and anomalies still show). The
`test/fixtures/kinetic_trace.json` shape loads the same way. A board-side
recorder that writes this shape is the way to replay a bench incident here.

## Not the machine

The gates (e-stop, pause, override, power, ownership) are opened once and never
exercised. The solve costs nothing offline, so a stall the board would take
from a long solve is invisible here; the plan is the same bits.
