---
paths:
  - "**"
---

# Canon -- governance law (C-1..C-12)

The rule system every agent, every commit, and every document in this repo
answers to. Carried verbatim from the archived machine repo (SlopDrive-32
`.claude/rules/governance.md`), where it was written because that project's
knowledge rotted once: status was appended as prose across many files, facts
had many homes, claims were never stamped with how they were verified, and
agents resolved contradictions silently by picking whichever source they read
first. A fleet audit in July 2026 un-rotted it. These rules make that a
one-time event, and this repo starts with them rather than rediscovering them.

The model is the Valence registry: one source of truth, everything else
derived, a check mode, and a ritual that fixes the doc BEFORE the code.

The operator is not CS-trained. That is a design input, not a caveat: rules
relying on the operator catching subtle drift in review do not work. Drift is
caught mechanically or flagged explicitly, never absorbed silently.

## 1. The Map of Truth -- one fact, one home

Every fact lives in exactly ONE home. Everywhere else refers to it by name or
link and never restates it in full. The same fact stated twice with different
values is not information, it is a flag (§3).

| Domain | Sole home |
|---|---|
| Wire numbers (frames, CBOR keys, NACK codes, channels, limits) | Valence repo `spec/registry/registry.yaml` (sibling checkout, pinned by `valence.pin`) |
| Protocol behavior | Valence repo `spec/SPEC.md` |
| This machine's device-channel allocation | not yet allocated; it lands with the hub port (board `val-091.3`) and gets a row here in the same commit |
| Governance law | this file |
| Engineering doctrine | the other files in `.claude/rules/` |
| Volatile project/device state (fw versions, deployment, milestones) | the Beads dev board (`bd`, `val-` prefix) |
| Operator preferences | `CLAUDE.md` (repo root, gitignored) |
| Firmware version constant | `FIRMWARE_VERSION` in `flagship_p4/src/hub/valence_config.h` |
| Per-board silicon, memory-map and radio configuration | `flagship_<chip>/sdkconfig.defaults` (hand-written) and `flagship_<chip>/platformio.ini` |
| Hardware design rationale for the PCB | `docs/flagship-board.md` |
| Subsystem deep detail | that subsystem's own README / spec |
| Public docs site content | Valence repo docs-site, generated from its spec homes, never hand-forked |

`CLAUDE.md` is the auto-loaded entry point: operator preferences plus binding
pointers. It holds NO rules and NO status.

## 2. The Laws

**C-1 ONE HOME PER FACT.** See the map. Adding a fact means deciding its home
first. Restating a fact elsewhere requires a pointer, not a copy.

**C-2 STATUS IS NOT PROSE.** Anything that changes over time (versions,
live/deployed/landed claims, milestone state, known bugs, deferred work) lives
on the dev board as discrete stamped issues, never as narrative paragraphs
appended into doctrine or architecture docs.

**C-3 SAME-COMMIT TRUTH.** A commit that changes behavior, removes a feature,
or completes planned work MUST update the affected fact's home in that same
commit. Doc drift is a defect of the same severity as a failing test.

**C-4 STAMP OR IT IS HEARSAY.** Status claims carry a verification stamp:
`[verified YYYY-MM-DD -- method]` (`-- console capture after flash`,
`-- pio test -e native exit 0`, `-- scope, 1,078 edges`). An unstamped claim,
or one whose subject has been touched by commits since its stamp, is hearsay:
re-verify before building on it, or flag.

**C-5 THE FLAG DUTY.** Contradictions between sources, between a doc and the
code, or between a request and doctrine are NEVER resolved silently. Raise a
Canon Flag (§3) and stop that thread until the operator rules. This is the
core rule; everything else exists to make flags rare.

**C-6 FROZEN MEANS FROZEN.** The frozen list (conformance artifacts, golden
vectors, frozen public APIs -- Valence's own list, hash-pinned from this side
in `tools/canon_lint.py`) is touched only after a flag and an explicit
operator "yes, break compatibility". No exceptions for "it is just a comment".

**C-7 DOCTRINE CHANGES ARE MINI-RFCS.** Changing a rule in this file requires
proposed text plus rationale presented to the operator, explicit approval, and
a dated entry in the Amendments log (§6). Agents never edit this file
unilaterally. Generalized spec-gap ritual: when work needs a rule no doc
defines, write it into the correct home FIRST, then code against it.

**C-8 "WORKING" REQUIRES EVIDENCE.** No claim of live/working/deployed/fixed
-- in a doc, an issue, a commit message, or chat -- without naming the
evidence: the command run and the observed result. "Deployed" specifically
means version-verified on-device, not "upload completed".

**C-9 DELETE LOUDLY, DEPRECATE VISIBLY.** Removing code requires proof of no
remaining references (state the searches run: `flagship_*/src/`,
`flagship_*/ulp/`, `lib/`, `test/`, `tools/`, `docs/`, the per-board
`platformio.ini` and `CMakeLists.txt`) recorded in the commit message. A doc
describing a removed system gets a dated deprecation banner on the same commit
that removes the system, never left to read as current.

**C-10 PERIODIC SCRUB.** The truth-scrub audit (multi-agent contradiction
hunt) runs at every milestone merge and whenever the operator smells drift.
Confirmed findings feed fixes AND, where they reveal a missing rule, a C-7
amendment proposal.

**C-11 AMERICAN ENGLISH ONLY.** All prose, comments, identifiers, and UI
strings use American spellings (behavior, color, center, license, gray,
initialize). Exceptions: vendored third-party code verbatim, legal license
texts verbatim, and frozen wire artifacts -- a British spelling baked into a
frozen byte sequence or a released wire string is FLAGGED, never silently
respelled, because respelling it is a protocol break. Enforced by
`.claude/hooks/style_check.py` at write time and `tools/canon_lint.py`
tree-wide.

**C-12 COMMENTS ARE CONSTRAINTS, NOT STORIES.** See `.claude/rules/cpp-style.md`
for the mechanized form. A comment states what the code cannot show: an
invariant, a trap, a unit, a contract. Comments NEVER reference removed code
or libraries -- what is gone is gone, and the past lives in git history.

## 3. The Canon Flag protocol

An agent MUST flag, and stop that thread of work, when any of these hits:

1. Two authoritative sources contradict (doc/doc, doc/code, comment/code,
   board/device).
2. Work would touch anything on the frozen list, or modify or delete code on a
   motion or safety path whose liveness the agent cannot prove statically.
3. The operator's request conflicts with a Law, with a NON-NEGOTIABLE in any
   rules file, or with how the rest of the codebase works.
4. A fact the work depends on is unstamped or stale (C-4) and cannot be
   re-verified without hardware the agent should not drive unasked.

Flag format, verbatim structure:

```
CANON FLAG -- <one line: what conflicts>
Source A: <file:line> "<quote>"
Source B: <file:line> "<quote>"   (or: <the request> / <observed device state>)
My read: <which I believe is correct and why, or "cannot determine statically">
Your call: <the single specific question the operator must answer>
```

Rules of engagement: unrelated work may continue, the flagged thread may not.
Never "fix" one side to match the other before the ruling. After the ruling
the resolution is written into the fact's ONE home, stamped, and the losing
statements are corrected or deleted, in the same commit.

**Operator override.** When the operator explicitly asks for something
doctrine forbids, the agent flags it (trigger 3), explains the conflict and
its consequences plainly, and the discussion runs to a conclusion. If the
ruling stands against current doctrine that is not an exception to be quietly
carved out, it is a C-7 amendment: the rule is updated to say what the
operator actually wants, dated in the Amendments log. Doctrine follows the
operator; it is never silently violated AND never silently diverges from
operator intent.

**One collision question is CLOSED and never re-raised as a flag:** the
"Valence" name search (operator ruling 2026-09-20, §6). It names an
open-source, non-commercial project, not a company.

## 4. Volatile truth lives on the board

All volatile truth lives on the Beads dev board as stamped issues, never as
prose in a rules file. Agents read the board at the start of substantive work
(`bd ready`, `bd list --status=open`; `bd prime` FIRST in a subagent, which
never receives the SessionStart hook). Update it in the same commit as the
change (C-3). TodoWrite is prohibited and blocked by a hook.

## 5. The mechanical floor -- canon_lint

`tools/canon_lint.py` greps the codebase for known doctrine violations (the
classes that have actually bitten this project family) and exits nonzero on a
hit. Agents run it before declaring any substantive change done. It is
judgment-free by design: everything it catches is a hard rule, so a hit is a
defect, not a conversation. The check list grows via C-7 when new violation
classes are confirmed, typically by a C-10 scrub. Current checks are named in
that file's header; it is the one home for which checks exist.

## 6. Amendments

| Date | Change | Approved by |
|---|---|---|
| 2026-09-21 | OTA AND THE DIAGNOSTICS ARCHIVE LANDED (val-091.16/.17), and the Map of Truth row for the firmware version constant is allocated with them: `FIRMWARE_VERSION` in `flagship_p4/src/hub/valence_config.h`, which was already the string the hub's WELCOME identity sent and is now the C-8 evidence a deploy is checked against. `build-test-deploy.md`'s Deployment section is rewritten in the same commit: this board takes remote updates. RECORDED AS VETO-ABLE -- the row named a home that had existed since the hub port, so this is a C-3 correction rather than a rule change, but it is logged here because the version constant is now load-bearing for every deploy claim. | agent, operator veto pending |
| 2026-09-20 | Canon carried into Valence Drive from the machine repo (C-1..C-12, flag protocol, canon_lint), Map of Truth repointed: `val-` board, per-board sdkconfig/platformio homes, firmware version constant NOT YET ALLOCATED. The machine repo's S3-era incident record stays there and is cited by pointer, never copied. | operator |
| 2026-09-20 | NAMING: the PCB is the OSSM FLAGSHIP, this firmware is VALENCE DRIVE, the protocol becomes VALENCE. The protocol rebrand is RFC-shaped (the name is in wire bytes, `transport.md` T11), drafted through Valence's `spec/RFC-QUEUE.md`, and is decoupled from this repo, which consumes the spec by pinned sha. Nothing wire-visible is respelled here. The name-collision search is closed and is not a flag. | operator |
| 2026-09-20 | PER-BOARD PlatformIO PROJECTS (`flagship_<chip>/`), and the reason is measured, not tidiness: the ULP build hook resolves exactly one directory, `$PROJECT_DIR/ulp`, with no per-environment override (platform 55.03.39, `espidf.py:2957`, `ulp.py:64`). A `ulp/` directory shared with another board's environment is compiled into that board's image too. A separate `PROJECT_DIR` scopes it structurally. Never merge the boards back into one project. | operator |
| 2026-09-20 | PURE ESP-IDF, not Arduino, and measured: under `framework=arduino` the ULP binary is built in hybrid mode's throwaway stage and never reaches the final link (`nm` on `firmware.elf` carried no `_binary_ulp_main_bin_*` symbol). `sdkconfig.defaults` is the native home for what `custom_sdkconfig` had to smuggle. vlog and vglow get IDF glue twins at the hub port. | operator |
| 2026-09-20 | SOCKETS STAY ON THE P4; the C6 is a NIC running Espressif's stock hosted slave image. The S3's SRAM pressure came from an on-chip WiFi driver plus a BT controller; here the driver is off-chip, the part has 2.3x the SRAM and 32 MB PSRAM, and the measured network cost is 154 KB and tunable. A sockets-on-C6 design recreates the C5 bridge and its T31-T33 seam to save memory the P4 does not need. BLE later is a NimBLE host on the P4 with the controller on the C6. | operator |
| 2026-09-20 | TASK STACKS LIVE IN INTERNAL RAM. `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` is the IDF 5.5.4 default and only PERMITS a PSRAM stack; nothing here asks for one. PSRAM is unreachable while the flash cache is disabled, so putting a task stack there needs its own operator ruling. | operator |
| 2026-09-21 | ECOSYSTEM RENAME LANDED. No live mention of "slop" anywhere; commit history excepted. The protocol repo is VALENCE (library `lib/valence`, namespace `valence::`, lint `tools/valence_lint.py`), consumed here through `lib/valence` -> `../Valence/lib/valence` and pinned by `valence.pin` (was `slopsync.pin`). Wire strings landed with RFC-060 upstream and are respelled here in the same pass, superseding the 2026-09-20 NAMING row's "nothing wire-visible is respelled here": ws subprotocol `valence.v1`, endpoint `/valence`. The catalog etag moved 0034d22cc3b11512 -> 5bba8cb1a2f618c0 (T11: one `desc` string is protocol bytes). vmotion/vglow/vlog, Phosphor, Canon, Valence Sim, Valence Trace follow the same table. SlopDrive-32 survives ONLY as a citation of the archived S3-era reference, marked as archived at every site. | operator |
| 2026-09-21 | FIRMWARE RENAME: "Valence Drive" / "Vdrive" becomes **NUCLEUS** (full name Valence Nucleus; the hub every bond attaches to). The repo folder is `Nucleus`; the board prefix stays `val-` and the project dir stays `flagship_p4/`. One wire-visible string moved with it: `VALENCE_PRODUCT` in `flagship_p4/src/hub/valence_config.h`, `"ValenceDrive"` -> `"Nucleus"`, the product field the hub sends in HELLO/WELCOME identity (key 37); `VALENCE_HUB_NAME` followed as `"valence-p4"` -> `"nucleus-p4"`. Earlier amendment rows keep their historical wording. "Valence" alone still means the protocol. | operator |
| 2026-09-21 | LIBRARY RENAME: the three first-party libraries are **Flux** (`lib/flux`, `flux::`, LED grammar, was vglow), **Kinetic** (`lib/kinetic`, `kinetic::`, motion planner, was vmotion -- which also collided with VMware vMotion), and **Geiger** (`lib/geiger`, `geiger::`, logging, was vlog). The log macros are `GLOGT/D/I/W/E/F` and `GLOGx_EVERY_MS`, the compile floor is `GEIGER_COMPILE_LEVEL`, and the canon_lint checks are `printf-outside-geiger` and `led-outside-flux`. The motion catalog channels are respelled on the wire in the same pass (T11): `kinetic-diag/-limits/-chase/-waveform/-set`, with `ch::sm_*` becoming `ch::kinetic_*`. The catalog etag moved 5bba8cb1a2f618c0 -> 5f4635e7e419a966. Rows above this one keep their original spellings: they are history, not current names. | operator |
