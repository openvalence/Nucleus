---
paths:
  - "**"
---

# Where files go

The repo root is a closed set: the per-board projects, `lib/`, `docs/`,
`tools/`, `artifacts/`, the pins, and the license and readme surface. It is
not a landing zone. A new loose file at the root is a defect, and a hook
blocks it (`.claude/hooks/tidy_check.py`).

| What | Home | Tracked? |
|---|---|---|
| Firmware for one chip | `flagship_<chip>/` -- its OWN PlatformIO project | yes |
| LP core sources for that chip | `flagship_<chip>/ulp/` -- one directory per project, fixed by the builder | yes |
| Liftable libraries (hardware-free cores) | `lib/` | yes |
| The Valence surface | `lib/valence`, a symlink to the sibling checkout | the symlink only |
| Per-run output: captures, traces, sweeps, ELF archives | `artifacts/` | no |
| Vendor manuals, datasheets, schematics | `docs/reference/` | no |
| Prose that outlives the run: findings, designs, baselines | `docs/` | yes |
| Scripts, probes, one-off instruments | `tools/` | see `.gitignore` |
| Doctrine | `.claude/rules/` | yes |
| Status, versions, results worth keeping | the dev board (`bd`) | n/a |

**A new board is a new project, never a new environment in an existing one.**
The reason is measured and it is in `governance.md` §6: the ULP build hook
resolves exactly one `$PROJECT_DIR/ulp`, so a shared project compiles one
board's LP sources into another board's image.

## Instruments write to `artifacts/`

Every `tools/` script defaults `--out` under `artifacts/`. A new script does
the same. A bare filename resolves against the CALLER's cwd, so it lands
wherever the harness happened to start, which is how 29 result files ended up
in the machine repo's root once.

`artifacts/` is gitignored except `.gitkeep`, which exists so a fresh clone
has the directory the tools write into without any of them calling mkdir.

## Board labels: one axis per namespace

`bd` labels carry four unrelated axes. Keep them in separate namespaces or the
set rots into a pile nobody filters by.

| Axis | Form | Examples |
|---|---|---|
| Subsystem | `area:<name>` | `area:motion` `area:comms` `area:system` `area:webui` `area:tooling` `area:docs` |
| Cross-cutting flag | bare | `safety` |
| Process state | bare | `awaiting-stamp` `pending-ruling` `parked` |
| Provenance and tier | bare | `ledger-queue` `tier0-paved-path` |

Every issue gets exactly ONE `area:` label. Set it at creation
(`bd create --labels area:motion`); children inherit it from `--parent` unless
`--no-inherit-labels`. Filter with `bd list --label area:motion`, or
`--label-pattern 'area:*'` to see the whole taxonomy.

`area:` names the subsystem the FIX lands in, not where the symptom showed up.
An NVS clobber triggered by homing is `area:system`, not `area:motion`.

## A file is evidence, not a fact

A measurement that matters gets quoted onto the board, where it is stamped
(C-4) and survives the file being regenerated. Leaving the number only in
`artifacts/foo.json` means the next person has to rediscover both the file and
what it proved.

## Deleting

Anything under `artifacts/` is regenerable by re-running its instrument, so it
is safe to delete without ceremony. That is the whole point of the boundary:
if deleting a file needs a discussion, it was never an artifact.
