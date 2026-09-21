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
| `flagship_p4/src/hub/` | The Valence hub and its IDF glue: log front door, platform shims, catalog, config |
| `flagship_p4/ulp/` | LP core sources. One directory per project, fixed by the builder |
| `flagship_p4/sdkconfig.defaults` | Silicon, memory map, PSRAM, radio. Hand-written, tracked |
| `lib/kinetic` | The trajectory engine (liftable, hardware-free) |
| `lib/geiger`, `lib/flux` | Logging and LED cores (liftable, hardware-free) |
| `lib/ruckig` | VENDORED, byte-identical to upstream. Do not restyle or respell (C-11 carve-out) |
| `lib/valence` | Symlink to the sibling Valence repo, pinned by `valence.pin`. READ-ONLY from here |
| `docs/` | Prose that outlives a run; `docs/flagship-board.md` is the PCB's design rationale |
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
| `app_main` | HP | Boot report, subsystem start, periodic liveness line (free/maxblock for both heaps, LP counters) |
| LP emitter (`ulp/lp_quad.c`) | LP | Quadrature edges from a phase accumulator. Its signed edge count is position truth |
| `esp_hosted` / WiFi / lwIP tasks | HP | Owned by the drivers, not by us. Their callbacks are not our task (`transport.md` T5) |

The hub task and the motion task land with the port (`val-091.3`,
`val-091.4`); their core and priority assignment gets a row here in the commit
that creates them, not before. Task stacks are internal RAM
(`governance.md` §6).

## One fact, one home (C-1)

Stop grepping for these. They live in exactly one place.

| Fact | Home |
|---|---|
| Firmware version | not yet allocated |
| Silicon revision, flash size, PSRAM, ULP reserve, radio pins | `flagship_p4/sdkconfig.defaults` |
| Build entry point, upload port, board id | `flagship_p4/platformio.ini` |
| Wire numbers, CBOR keys, NACK codes, channels | sibling `Valence/spec/registry/registry.yaml` |
| Protocol behavior | sibling `Valence/spec/SPEC.md` |
| This machine's channel allocation | not yet allocated; lands with the hub port |
| PCB design rationale and the bench measurements behind it | `docs/flagship-board.md` |
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
