---
paths:
  - "**"
---

# C++ style

Carried from the machine repo (SlopDrive-32 `.claude/rules/cpp-style.md`).
Navigation doctrine is NOT restated here (C-1): `.claude/rules/navigation.md`
is its home, and its rule -- grep to find, LSP to understand -- binds every
C++ task in this repo.

## Coding style and extensibility

- Strict OOP; `.h`/`.cpp` isolation; lifecycle hooks (`init()`, `update()`,
  `emergencyStop()`) on functional modules.
- `float` over `double`. Double math only at plan-time events, never
  per-sample.
- The language standard is ESP-IDF's, not ours: IDF 5.5.4 appends
  `-std=gnu++2b` project-wide for every non-linux target
  (`tools/cmake/build.cmake`). Do not restate it in a project `CMakeLists.txt`
  -- that would be a second home for a fact IDF already owns (C-1).

### Comment style law (operator-stamped 2026-07-28)

Scope: `flagship_*/src/`, `flagship_*/ulp/`, `lib/`, `test/` C++ and C. This
is the mechanized form of CANON C-12.

- **FILE HEADER.** Every `.h`/`.hpp`/`.c`/`.cpp` opens with
  `// <Name> -- <one-line job>`, then `// Constraints:` lines holding only
  load-bearing rules (threading, ownership, units, never-do's), then `// See:`
  pointers if real ones exist. No history, no authorship, no dates, no feature
  lists.
- **SECTION BANNERS.** `// ---- <section name> ----` dash-padded to column 80.
  Name only: no hex ids, no numbering, no box art. RFC-nnn and T-nn references
  in a banner name are POINTERS, which is allowed, not numbering. Files over
  ~150 lines are divided into their natural sections this way.
- **STYLE.** `//` everywhere; `/* */` only in license headers. Multi-line is
  consecutive `//` lines indented with the code they bind to.
- **CONTENT.** A comment is exactly one of: a CONSTRAINT, an INVARIANT the
  code cannot show, a POINTER (rules file, SPEC, docs), or
  `// TODO(<board id or RFC-nnn>): <change>`. A TODO without a home reference
  is a finding. Narration, history, restated code, and diff justification are
  deleted; a story worth keeping moves to a rules file with a pointer left
  behind.
- **VOICE.** American English, fragments fine, no first person, no emoji.

### Minimalism-mode precedence (operator ruling 2026-07-29)

The ponytail agent mode governs `docs/`, host `tools/`, and the web UI. It
does NOT govern `flagship_*/src/`, `flagship_*/ulp/`, `lib/`, or the SlopSync
repo; there the rules files outrank it. Diff size is not a correctness
argument.

- Memory and concurrency reasoning is never the thing that gets shortened.
  Stack cost, heap/BSS/PSRAM placement, and which task or core a callback runs
  on are stated before a change is called done.
- Single-implementation indirection that doctrine mandates -- the VLog and
  VGlow sole paths, the MotionArbiter sole-caller rule -- is law, not
  speculative abstraction to delete.
- A SPEC-defined protocol field is not dead weight because one implementation
  currently ignores it. The spec decides; changes ride the RFC ritual.
- Every field bug on record was the small obvious change. The cost landed on
  stack depth, allocation lifetime, and task context, none of which a
  diff-size heuristic can see.
