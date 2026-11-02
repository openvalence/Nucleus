---
paths:
  - "**"
---

# User-visible strings: the standard and the string pass

Every string a user reads reads like professional shipping software written
by someone who has never seen the code (operator ruling 2026-11-01,
governance.md §6). The example of what this kills: the log line
`image bought: %s is now the boot slot`, internal jargon only its author
understands. This file is the one home for that standard in every repo of
the family and the site; repos point here and keep only their own
mechanics (Phosphor `docs/COPY.md`, this repo's `logging-leds.md`).

## Covered strings

- Log lines: `GLOG*`, `ESP_LOG*`, `printf` in bench sketches, Garnet's logs.
- Catalog `desc`, `label`, option labels and group strings (T11: wire bytes).
- Refusals: NACK `detail`, HTTP reasons and bodies, `refuse*` text, any
  reason a client shows verbatim.
- Garnet screen text (`lv_label_set_text*` and friends).
- Phosphor UI text and messages: tooltips, hints, placeholders, status lines,
  health lines, dialogs, the Neutrino dev tool's page.
- Site copy (openvalence.github.io).
- CHANGELOG "For users" sections and release notes.

Not covered: comments (C-12 rules them), identifiers, test names, rules
files, the docs site (its own writing rules and defined technical terms,
Valence `docs-site/docs/community/contributing.md`), amendment rows and RFC
bodies (records).

## The rubric

Everywhere:
- American English (C-11). No em dashes: a colon, a comma or two sentences.
  Never the word "AI".
- Plain words over internal names. Name things the way the user sees them:
  the motor (never servo, drive or AIM drive in user-facing text; technical
  docs keep precise terms), the machine, the carriage, the rail, the window
  (the stroke range on the rail), Phosphor (the app), the remote (Garnet
  hardware), the R+D board or the helper (the barnacle helper). Never invent
  a new name for a named thing. Code-internal words never reach a user
  ("buy", "take", "arm", "lease", "door", "geiger").
- Keep every fact: numbers, units, conditions, the next step. Never add a
  fact the source does not show. A writer unsure what a string means keeps
  it and says "unsure".
- No reassurance, marketing or filler ("successfully", "please", "simply",
  "just"); no vague negations ("nothing moves"): state the precise fact.
  Plain technical terms of motion control and industrial equipment (axis,
  e-stop latch, fault, interlock, link loss, timeout, latency, retransmit).

By kind:
- **Log lines** (read by a DIY builder in Phosphor's Log): event first,
  then the object and the outcome. Lowercase first word, no trailing period.
  Every format specifier (`%s %u %d %08lx %.1f`, a `PRIu32` splice) stays,
  same count, same order, same meaning. `key=value` pieces and tags stay as
  they are. A line reporting a problem names what the user can do when the
  source shows it. Short. Geiger keeps 103 bytes of each message
  (`kMsgBytes`, `lib/geiger`) and cuts the rest silently: every fact must
  fit, measured with realistic values (a `%s` at its longest source string).
  A line that cannot fit splits into two lines; it never drops a fact.
- **Catalog `desc` / `label`** (every client renders them verbatim): one
  Blender-tooltip fragment, about 4 to 12 words. Controls start with the
  verb and name the object and the effect ("Reverse the machine's travel
  direction"). Readouts say what is counted and its scope, never the label
  or the value again ("Times the machine link dropped and reconnected this
  session"). No trailing period, no rationale. Hard limits are the
  registry's: `desc_max_bytes` (128), `option_label_max_bytes` (24); descs
  are also the one unbounded cost in `catalog_max_entry_bytes`, so tighter
  wins.
- **Refusals**: why, in the user's words, and the next step when the source
  shows it. No longer than the original unless the buffer shows room; NACK
  `detail` is `nack_detail_max_bytes` (48) and is diagnostic, never parsed
  (SPEC §16.1).
- **Garnet screen text**: no longer than the original (a 320x240 screen).
- **Phosphor UI**: terse, Blender style, status lines are fragments; the
  mechanics (60-character lint, `data-tip`, health lines) are `docs/COPY.md`.
- **Site copy**: industrial, plain, factual; long form is fine.
  Change a string only for jargon, machine-written prose (Wikipedia's
  "Signs of AI writing": inflated significance, "serves as", rule-of-three
  padding, "not just X but Y", vague attributions, promotional words), a
  wrong name per the list above, or a British spelling. Headings stay short.
- **CHANGELOG "For users"**: what changed for the person at the machine, in
  the log-line and UI terms above; no file names or bead ids.
- **Instructions** (guide steps, flashing and bring-up procedures, release
  notes, test checklists, README how-tos): standard technical English, the
  ASD-STE100 writing rules. Numbered steps; one instruction per sentence, in
  the imperative and the active voice; procedural sentences at most 20 words,
  descriptive ones at most 25; one topic per paragraph; keep the articles; a
  word keeps one meaning across the page; a warning or caution comes before
  the step it applies to and says the hazard and its result.

Hard constraints on every change:
- **Machine-parsed text stays** unless every parser changes in the same
  commit: a test or tool matching a fragment, a regex, Phosphor's Log
  filters, the diagnostics footer `next=<seq>` (logging-leds.md).
- **A desc's opening clause can be a label.** The catalog has no per-field
  label key, so a field with no role label shows its desc's opening clause
  as its label in Phosphor (`src/model/format.js` `descLabel`: 2 to 3 words
  before the first `.,;:`, at most 30 characters). A rewrite keeps that
  clause until catalog fields carry their own label (an RFC).
- **Wire strings are protocol bytes** (transport.md T11): a catalog string
  change moves the etag on purpose and says so in the commit; frozen
  artifacts never change (C-6); a released wire string that breaks C-11 is
  flagged, never respelled.
- **Verdict "keep"** when the original already meets the bar; a change that
  only reshuffles words is not a change.

## The method: the string pass

Facts come from the source, never from the writer.

1. **Extract** the strings in scope with `tools/strings/extract.py` in the
   workspace root (whole repos, `--diff <base>`, or a file list). Each item
   carries its file, line, kind, the code or page context around it, and
   its siblings in the same call (ternary alternatives are separate items).
2. **Batch** with `tools/strings/batch.py`: adds `refs`, the tests, tools and
   parsers that contain the text.
3. **Rewrite blind**: the named workflow `string-sweep` gives each batch to a
   Haiku writer that sees only the string, its context and this file.
4. **Check**: a Sonnet checker rejects or fixes any rewrite that changes a
   fact, adds a claim, drops or reorders a specifier, breaks a byte limit,
   renames a named thing, touches parsed text, or reads worse; it lists the
   refs that must change with each string.
5. **Apply** the checked finals to the real literals, with every ref, then
   the repo's lint and suites. A bulk sweep publishes a veto page of
   before/after rows first; the operator's vetoes are dropped before the
   apply. The skill `string-pass` (workspace `.claude/skills/`) walks it.

## When it runs

- **Every commit that adds or changes a user-visible string** runs the pass
  on those strings before it lands (the formulas carry it as a step). A
  string written by hand and committed unchecked is a defect, like a lint
  finding.
- **Bulk sweeps** on the operator's request, whole repos or the site, always
  with the veto page.
