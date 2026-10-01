# Rename inventory: slop-* residue in Nucleus

The INVENTORY for board `val-smy`. No rename here is performed; each row is
a proposal waiting on the rulings at the bottom. The naming table itself
(Valence, Nucleus, Phosphor, Geiger, Flux, Kinetic, Valence Sim, Valence
Probe, Valence Trace, Valence Tool, Canon) is ruled and lives in
`.claude/rules/governance.md` §6; this file only maps the leftovers onto it.

Method [verified 2026-10-01 -- `grep -rni slop` over the tree excluding
`.git`, `artifacts/`, `.pio`, `build/`, `__pycache__`, `lib/valence`; a
string-literal grep for `"...slop..."` over sources, tools, sim and configs;
`bd list --all --json` scanned for slop tokens in every issue]. Rerun the
same greps after the grind; this file is evidence, not a live count.

## 1. Wire-visible: NONE remaining

No string literal in `flagship_p4/`, `lib/`, `test/`, `sim/` or `tools/`
contains "slop". The wire strings (`valence.v1`, `/valence`, `"Nucleus"`,
`nucleus-p4`, `kinetic-*` channel names) landed with RFC-060 and the
2026-09-21 rows in `governance.md` §6. Nothing below needs a Valence RFC.
`lib/valence` is the sibling's surface and its residue is the sibling's
inventory, not this one.

## 2. Code comments citing archived files (internal)

| Where | Token | Proposal |
|---|---|---|
| `flagship_p4/src/hub/ValenceCatalog.h` :33-34, 49, 105, 204, 219, 621, 1140, 1576, 1818, 1862, 1875, 1995 | `SlopSyncHubService(.cpp)` (12) | Repoint each to its live twin (`hub/ValenceHub.cpp`, `hub/ValenceDevice.cpp`) where the invariant exists here; delete the citation where it does not. Already flagged as a follow-up on `val-smy` |
| same file :33, 1671, 2050 | `SlopDriveHubDelegate::applyIntent` (3) | Repoint to `ValenceDevice`'s `applyIntent` |
| `flagship_p4/src/hub/ValencePlatform.h` :12-13 | "the S3's `SlopSyncPlatform.h`" | Delete the lineage line (C-12: no history in headers) |
| `flagship_p4/src/hub/valence_config.h` :14 | "carried verbatim from the archived SlopDrive-32 S3 product" | Delete the lineage clause, or keep under ruling 1 |
| `flagship_p4/src/hub/ValenceHub.h` :82 | archived `memory-budget.md` T27 | Repoint to this repo's `.claude/rules/memory-budget.md` T27, which carries it |
| `flagship_p4/src/main.cpp` :432 | archived `memory-budget.md` T21 | Repoint to this repo's `memory-budget.md` T21 |
| `lib/kinetic/include/kinetic/kinetic.hpp` :2402 | archived `webui.md` T18 | Repoint to `../Phosphor/.claude/rules/webui.md` T18 |
| `lib/kinetic/include/kinetic/kinetic.hpp` :115 | scenario-trace bench lives in the archive | Keep (the tool was not ported) or port it; ruling 1 |
| `test/native/test_kinetic/test_main.cpp` :2027 | `SlopSyncHubService::drainMotionStream` | No live twin by that name (`grep drainMotionStream flagship_p4/src` is empty); find the equivalent intake path and repoint, else delete |
| `lib/strokeengine_patterns/include/patternShim.h` :7 | "This file is SlopDrive-32 machinery" | STALE CLAIM, not a citation: the shim is Nucleus's now. Respell to "Nucleus machinery" |
| same file :31 | "SlopDrive will reinterpret these" | "Nucleus reinterprets these" (it does, in `flagship_p4/src/patterns/`) |
| `lib/strokeengine_patterns/VENDORED.md` :6-7, 30 | archive shas `b8684b9`, `d745423`, `999edd0` | KEEP: vendoring provenance must name where the copy came from |

## 3. Tool and hook provenance headers (internal)

Each opens with "PORT of the archived SlopDrive-32 repo's <same path>".

| Files | Proposal |
|---|---|
| `.clang-tidy`, `.claude/hooks/beads-gate.sh`, `style-check.sh`, `style_check.py`, `tidy_check.py`, `vendor-lock.sh`, `tools/canon_lint.py` | Drop the PORT line (C-12 file-header rule: no history) or keep it as provenance; ruling 2 |
| `tools/lag_probe.py` :24 | Repoint archived `webui.md` T18 to Phosphor's `webui.md` T18 |
| `tools/lp_leg_capture.py` :13 | KEEP: the Rigol scope guide was not carried here; the archive is its only home |

## 4. Doctrine and README prose (intentional legacy pointers)

`.claude/rules/` (cpp-safety, cpp-style, governance, logging-leds,
motion-control, transport) and `README.md` cite "the archived SlopDrive-32"
repo as the legacy case file for traps this board does not inherit
(`val-091.5`). These are the deliberate pointers the 2026-09-21 ECOSYSTEM
RENAME row permits ("SlopDrive-32 survives ONLY as a citation of the archived
S3-era reference, marked as archived at every site"). KEEP. The archive's own
board id `sd-1bi.3` in `motion-control.md` stays: it is that repo's id.

`governance.md` §6 amendment rows keep their original spellings by ruling
(the 2026-09-21 LIBRARY RENAME row). KEEP.

## 5. Board (`bd`) references

| Issue | State | Residue | Proposal |
|---|---|---|---|
| `val-091` | open | title "Valence Drive bring-up"; description cites SlopDrive-32, SlopSync | Retitle "Nucleus bring-up on the OSSM Flagship"; description note pointing at the rename rows |
| `val-091.11` | open | SlopDeck (7), `slopsync_probe` (5), SlopSync, SlopSyncHubService.cpp | Comment mapping SlopDeck -> Phosphor, `slopsync_probe` -> `valence_probe.py`; leave history text |
| `val-091.13` | open | `webui/src-tauri/target/release/slopdeck.exe` | Update the path once Phosphor's binary name is final |
| `val-3tz` | open | title "slopsync_probe.py ANOMALY_KINDS ..." | Retitle to `valence_probe.py` |
| `val-b7r` | open | title "slopsync_probe.py hardcodes SlopDrive's ..." | Retitle to `valence_probe.py` / "Nucleus's" |
| `val-fjx` | open | description: SlopSync, SlopDeck | Leave; a comment if it is ever worked |
| `val-091.5`, `val-smy` | in progress | rule names and ruling history | Leave: they ARE the rename record |
| `val-091.1/.2/.3/.4/.7/.9/.10/.12` | closed | SlopSync, SlopDeck, SlopHub (the hub task's old name), SlopLog/SlopGlow, `slopsync.v1` | Leave: closed issues are history, like commit messages |
| `.beads/interactions.jsonl`, `.beads/backup/`, the Dolt store | n/a | SlopDeck, SlopHub | Never edited: bd's append-only audit log and its backups |

## 6. False positives (English and third-party; never renamed)

- "slope", "sloped", "slopes", "G-slope", `SLOPE`: math, in `kinetic.hpp`,
  `flux_core.hpp`, `ValenceDevice.cpp`, `ValenceCatalog.h`, `pattern.h`,
  `test_kinetic`.
- "allow slop for the sampled edges", `test/native/test_flux/test_main.cpp`
  :149: English for tolerance. Respelling it to "tolerance" would leave the
  grep clean; optional.
- `:TRIG:EDGE:SLOP`, `tools/lp_scope.py` :68: a Rigol SCPI command. Must stay.

## 7. Out of scope by construction

Git history; `artifacts/` (gitignored; holds `slopdeck-live-*.png` and
`val-091.11-slopdeck-*.png` from before the rename); `lib/valence` (the
sibling's); the operator's `SlopSpace.code-workspace`; the Android package
`com.slopdeck` (follows Phosphor, its repo).

## Rulings owed (operator)

1. **Archive citations in code comments.** C-12 says comments never reference
   removed code; the 2026-09-21 row allows SlopDrive-32 citations marked
   archived. Which wins for the ~20 code-comment citations in §2: repoint to
   the live twin and delete the rest (recommended), or keep as marked archive
   citations?
2. **"PORT of" provenance lines** in hooks, `.clang-tidy` and `canon_lint.py`
   (§3): drop under the C-12 file-header rule, or keep as provenance?
3. **Closed beads** (§5): confirm they stay as history (recommended).
4. **Open bead retitles** (§5: `val-091`, `val-3tz`, `val-b7r`): approve the
   proposed titles, which are internal and need no RFC.
5. **`patternShim.h`** (§2): the shim sits under a vendored directory, but the
   two stale "SlopDrive" lines are ours, not upstream's. Confirm it may be
   edited despite the directory's vendored status.
