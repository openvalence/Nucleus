#!/usr/bin/env python3
"""Canon mechanical floor -- judgment-free doctrine checks.

PORT of the archived SlopDrive-32 repo's tools/canon_lint.py, curated for
Nucleus. Defined by .claude/rules/governance.md SS5. Every hit is a
defect BY DEFINITION: these checks encode only hard rules (violation classes
that have actually bitten this project family). If a check fires falsely, the
fix is a C-7 amendment to the exemption lists in this file -- never ignoring
the output.

Checks here: printf-outside-vlog, valence-purity, this-assign,
new-log-macro, led-outside-vglow, borrowed-member, sole-caller,
static-in-critical, british-spelling (codespell plus the camelCase subword
gap), and the valence.pin rule with the frozen-artifact hash cross-check.

NOT ported, each for a stated reason:
  links2004-ghost  -- names a WebSocket stack that never existed in this tree.
  ws-client-held   -- AsyncTCP is not here; the rule survives as a legacy
                      pointer in cpp-safety.md T29.
  sampler-stack-shrunk -- pinned a task name that does not exist yet. The rule
                      (never shrink the stack that calls commit()) lives in
                      motion-control.md and gets a check when the task lands.
  delay-in-realtime -- the Arduino delay() symbol does not exist under pure
                      ESP-IDF, and its IDF equivalent is legitimate in an
                      ordinary task loop, so one regex cannot separate the
                      violation from the normal case. The prohibition stands
                      in architecture.md SS2 as a review rule.
  channel-map drift -- there is no channel map yet (governance.md SS1).

Still deliberately NOT here: lifetime rules (no ref/span to a local, lambda
capture-by-value). Those need clang-tidy.

Usage:
    python tools/canon_lint.py

Exit codes: 0 clean, 1 findings, 2 could not run.
"""

import hashlib
import io
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SIBLING = ROOT.parent / "Valence"
PIN_FILE = ROOT / "valence.pin"

# ---------------------------------------------------------------- frozen (C-6)
# The conformance artifacts live in the pinned SIBLING checkout. Byte-identical
# there or it is a protocol break; Valence's own tools/valence_lint.py
# carries the same pins as its half of the belt-and-suspenders check.
FROZEN_SHA256_SIBLING = {
    "lib/valence/include/valence/conformance/mini_catalog.hpp":
        "6613fea1cfa92e0de327dca17498bd18da64dd1e280ed0e2ed15ae6eaec3a228",
    "spec/vectors/fixtures/mini-catalog.yaml":
        "7576f08b5c190a5c720b5ec09a1fe3476fc97d3e0f11ba417720c953d2cfe44e",
}

VENDORED_PREFIXES = ("lib/ruckig/", "lib/valence/", "managed_components/")
BINARY_SUFFIXES = (".bin", ".png", ".jpg", ".webp", ".ico", ".pdf",
                   ".woff", ".woff2", ".idx", ".gz", ".lock", ".elf", ".uf2")

# ------------------------------------------------------------------ C-11
# American English only. codespell's en-GB_to_en-US builtin dictionary is the
# mechanism; a hand-rolled British-word regex is not reinvented here. HARD
# DEPENDENCY: if codespell is not importable, the check FAILS LOUDLY, never
# silently skips. Minimum version 2.4.
try:
    import codespell_lib
    _CODESPELL_IMPORT_ERROR = None
except ImportError as e:
    codespell_lib = None
    _CODESPELL_IMPORT_ERROR = str(e)

# Real words this project's prose uses that codespell's en-GB_to_en-US
# dictionary is verified NOT to carry. Anything codespell already catches must
# NOT be duplicated here -- verify against its dictionary before adding, and
# name the gap in a comment.
BRITISH_SPELLING_EXTRAS = {
    "travelled": "traveled",
    "travelling": "traveling",
    "traveller": "traveler",
    "travellers": "travelers",
}
_BRITISH_EXTRAS_RX = re.compile(
    r"\b(?:%s)\b" % "|".join(sorted(BRITISH_SPELLING_EXTRAS, key=len, reverse=True)),
    re.IGNORECASE,
)

# ------------------------------------------------------- C-11 (camelCase gap)
# codespell's word regex is r"[\w\-'']+" -- \w includes underscore, so it
# treats "colourMode", "waveform_centred" and "kColourTable" as ONE token each
# and never matches them against the dictionary key. This closes that gap by
# splitting every identifier on case boundaries, underscores and digit
# boundaries before the dictionary check -- same codespell dictionary, no
# hand-rolled wordlist. Any subword >=4 chars that matches is a hit.
_IDENT_WORD_RX = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
_DIGIT_BOUNDARY_RX = re.compile(r"(?<=[A-Za-z])(?=[0-9])|(?<=[0-9])(?=[A-Za-z])")
_CAMEL_BOUNDARY_RX = re.compile(r"(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])")

# Files the spelling scan must never touch: this file names the banned words by
# construction (the extras dict above), style_check.py holds the en-GB fallback
# wordlist for the same reason, legal texts are verbatim by law, and .beads/ is
# a generated append-only log that QUOTES issue text.
BRITISH_SPELLING_SCAN_EXEMPT = ("THIRD_PARTY_LICENSES.md", "LICENSE", "NOTICE",
                                "tools/canon_lint.py",
                                ".claude/hooks/style_check.py")
BRITISH_SPELLING_SCAN_EXEMPT_PREFIXES = (".beads/",)


def _split_subwords(token):
    out = []
    for piece in token.split("_"):
        if not piece:
            continue
        piece = _DIGIT_BOUNDARY_RX.sub(" ", piece)
        for chunk in piece.split(" "):
            if not chunk:
                continue
            chunk = _CAMEL_BOUNDARY_RX.sub(" ", chunk)
            out.extend(c for c in chunk.split(" ") if c)
    return out


def _load_gb_dictionary():
    """The exact dictionary codespell ships -- read from its own package data,
    never retyped. This just gives us subword-level access to it."""
    if codespell_lib is None:
        return {}
    data_dir = Path(codespell_lib.__file__).resolve().parent / "data"
    path = data_dir / "dictionary_en-GB_to_en-US.txt"
    out = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or "->" not in line:
            continue
        brit, us = line.split("->", 1)
        out[brit.strip().lower()] = us.strip()
    return out


def _subword_hits(token, dictionary):
    hits = []
    for sub in _split_subwords(token):
        if len(sub) < 4 or sub.lower() == token.lower():
            continue  # whole-word hits are codespell's own job, not this gap
        low = sub.lower()
        if low in dictionary:
            hits.append((sub, dictionary[low]))
        elif low in BRITISH_SPELLING_EXTRAS:
            hits.append((sub, BRITISH_SPELLING_EXTRAS[low]))
    return hits


def _spelling_exempt(rel):
    return (rel in BRITISH_SPELLING_SCAN_EXEMPT
            or rel.startswith(BRITISH_SPELLING_SCAN_EXEMPT_PREFIXES))


GREP_CHECKS = [
    dict(
        name="printf-outside-vlog",
        msg="printf in hub firmware (logging-leds.md: logging goes through VLog. Only.)",
        rx=re.compile(r"\b(?:printf|puts|fputs|vprintf)\s*\("),
        # Scoped to the hub sources and the logging library, deliberately not to
        # the whole of flagship_*/src/: the composition root today is a BENCH
        # IMAGE whose report lines ARE its product, and they run before any sink
        # exists.
        include=("flagship_p4/src/hub/", "lib/vlog/"),
        # The sink itself. Same standing as the machine repo's AppLog: the one
        # file allowed to touch the output device, because it IS the output
        # device's driver.
        exempt=("lib/vlog/include/vlog/vlog.h",),
    ),
    dict(
        name="valence-purity",
        msg="platform header inside hardware-free lib/valence (transport.md: hardware-free, std headers only)",
        rx=re.compile(r'#\s*include\s*[<"](?:Arduino\.h|freertos/|esp_|driver/|soc/|nvs)'),
        include=("lib/valence/include/",),
        exempt=(),
    ),
    dict(
        name="this-assign",
        msg="*this = T{...} reset pattern (cpp-safety.md T1: a KB-scale stack temporary blew the "
            "task stack; use in-place destroy + placement-new)",
        rx=re.compile(r"\*\s*this\s*=\s*"),
        include=("flagship_", "lib/"),
        exempt=("lib/ruckig/", "lib/valence/"),
    ),
    dict(
        name="new-log-macro",
        msg="new SLOG* macro definition (logging-leds.md: no new log macros; "
            "VLog is the only logging path)",
        rx=re.compile(r"^\s*#\s*define\s+SLOG"),
        include=("flagship_", "lib/vlog/"),
        # The VLog front door defines the macro surface; that is its job, and it
        # binds one platform layer per host so no consumer ever supplies its
        # own. A SECOND definer anywhere is the violation this check exists for.
        exempt=("lib/vlog/include/vlog/vlog.h",),
    ),
    dict(
        name="led-outside-vglow",
        msg="LED driven outside VGlow (logging-leds.md: callers speak semantics; "
            "board wiring lives in one glue file per board)",
        rx=re.compile(r"\bled_strip_\w+\s*\(|\bgpio_set_level\s*\(\s*\w*LED\w*"),
        include=("flagship_", "lib/vmotion/", "lib/vlog/"),
        exempt=(),
    ),
    dict(
        name="borrowed-member",
        msg="span/string_view stored as a class member (cpp-safety.md: parameters "
            "only; members own their data)",
        rx=re.compile(r"^\s+(?:std::)?(?:span|string_view)[^;=]*\s+(?:_|m_)[A-Za-z]\w*\s*;"),
        include=("flagship_", "lib/v"),
        exempt=(),
    ),
    dict(
        name="sole-caller",
        msg="input source commanding the motion processor directly (architecture.md SS2: "
            "the MotionArbiter is the ONLY caller; input sources submit intents)",
        rx=re.compile(r"(?:\.|->)(?:moveTo|streamTo|home)\s*\("),
        include=("flagship_p4/src/ui/", "flagship_p4/src/hub/",
                 "flagship_p4/src/patterns/"),
        exempt=(),
    ),
]


def governed_files():
    """Every file the lint governs: tracked PLUS untracked-but-not-ignored.

    Plain `git ls-files` is a false-clean generator. A brand new file is
    invisible to every check until someone remembers to `git add` it, which is
    precisely the window where the checks matter most. `--others
    --exclude-standard` adds untracked files while still honoring .gitignore.
    """
    out = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard"],
        cwd=ROOT, capture_output=True, text=True, check=True).stdout
    seen = set()
    for f in out.splitlines():
        if f in seen:
            continue          # a staged-and-modified path is listed twice
        seen.add(f)
        if f.startswith(VENDORED_PREFIXES) or f.endswith(BINARY_SUFFIXES):
            continue
        if not (ROOT / f).is_file():
            continue
        yield f


def run_grep_checks():
    findings = []
    for rel in governed_files():
        applicable = [c for c in GREP_CHECKS
                      if any(rel.startswith(p) for p in c["include"])
                      and not any(rel.startswith(e) for e in c["exempt"])]
        if not applicable:
            continue
        try:
            text = (ROOT / rel).read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for lineno, line in enumerate(text.splitlines(), 1):
            for c in applicable:
                m = c["rx"].search(line)
                if m:
                    findings.append((c["name"], rel, lineno, m.group(0), c["msg"]))
    return findings


def run_critical_static_check():
    """cpp-safety.md T4: a function-local static inside a critical section
    aborts the core.

    Not a GREP_CHECK because it needs the span between two tokens. The window
    is capped at 40 lines: a critical section longer than that is its own
    defect, and an uncapped span swallows the whole file on an unbalanced
    match.
    """
    rx_static = re.compile(r"^\s*static\s+[A-Za-z_][\w:<>]*\s+\w+\s*[;({=]", re.M)
    findings = []
    for rel in governed_files():
        if not rel.startswith(("flagship_", "lib/v")):
            continue
        if not rel.endswith((".c", ".cpp", ".h", ".hpp")):
            continue
        try:
            lines = (ROOT / rel).read_text(encoding="utf-8",
                                           errors="replace").splitlines()
        except OSError:
            continue
        depth_start = None
        for i, line in enumerate(lines):
            if "portENTER_CRITICAL" in line or "taskENTER_CRITICAL" in line:
                depth_start = i
            elif "portEXIT_CRITICAL" in line or "taskEXIT_CRITICAL" in line:
                depth_start = None
            elif depth_start is not None:
                if i - depth_start > 40:
                    depth_start = None
                elif rx_static.match(line):
                    findings.append((
                        "static-in-critical", rel, i + 1, line.strip()[:70],
                        "function-local static inside a critical section "
                        "(cpp-safety.md T4: the init guard and __cxa_atexit "
                        "registration abort the core; hoist to file scope)"))
    return findings


def run_camelcase_check():
    """C-11 gap-closer: British spelling hiding inside a camelCase/PascalCase/
    combined-snake_case identifier, where codespell's whole-word match cannot
    see it. Filenames are in scope too -- a British-spelled path is the same
    defect as a British-spelled variable."""
    if codespell_lib is None:
        return []  # run_codespell_check reports the missing dependency loudly
    dictionary = _load_gb_dictionary()
    findings = []

    all_files = list(governed_files())
    for rel in all_files:
        for component in rel.split("/"):
            stem = component.rsplit(".", 1)[0] if "." in component else component
            for tok in _IDENT_WORD_RX.findall(stem):
                for sub, sugg in _subword_hits(tok, dictionary):
                    findings.append(("british-spelling-subword", rel, 0,
                                     f"{tok} (filename subword {sub!r} -> {sugg})",
                                     "British spelling inside a filename component "
                                     "(CANON C-11 -- camelCase/subword gap)"))

    for rel in (f for f in all_files if not _spelling_exempt(f)):
        try:
            text = (ROOT / rel).read_text(encoding="utf-8", errors="strict")
        except (UnicodeDecodeError, OSError):
            continue
        for lineno, line in enumerate(text.splitlines(), 1):
            for tok in _IDENT_WORD_RX.findall(line):
                if len(tok) < 4:
                    continue
                for sub, sugg in _subword_hits(tok, dictionary):
                    findings.append(("british-spelling-subword", rel, lineno,
                                     f"{tok} (subword {sub!r} -> {sugg})",
                                     "British spelling inside a compound identifier "
                                     "(CANON C-11 -- camelCase/subword gap, codespell "
                                     "cannot see across case/underscore boundaries)"))
    return findings


def run_codespell_check():
    """C-11 American-English check: codespell's en-GB_to_en-US builtin
    dictionary plus the small BRITISH_SPELLING_EXTRAS gap-list. Hard
    dependency -- codespell missing is a loud finding, never a silent skip."""
    if codespell_lib is None:
        return [("codespell-missing", "tools/canon_lint.py", 0, "",
                 f"codespell is not installed ({_CODESPELL_IMPORT_ERROR}) -- "
                 "pip install codespell (>=2.4) -- the British-spelling rule "
                 "has no fallback and refuses to silently skip")]

    files = [f for f in governed_files() if not _spelling_exempt(f)]
    if not files:
        return []
    abs_paths = [str(ROOT / f) for f in files]

    buf = io.StringIO()
    old_stdout = sys.stdout
    sys.stdout = buf
    try:
        codespell_lib.main(*abs_paths, "--builtin", "en-GB_to_en-US")
    finally:
        sys.stdout = old_stdout

    findings = []
    hit_rx = re.compile(r"^(.*):(\d+): (\S+) ==>")
    for line in buf.getvalue().splitlines():
        m = hit_rx.match(line)
        if not m:
            continue
        try:
            rel = Path(m.group(1)).resolve().relative_to(ROOT).as_posix()
        except ValueError:
            rel = m.group(1)
        findings.append(("british-spelling", rel, int(m.group(2)), m.group(3),
                         "British spelling (CANON C-11: American English only "
                         "-- codespell en-GB_to_en-US)"))

    for rel in files:
        try:
            text = (ROOT / rel).read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for lineno, line in enumerate(text.splitlines(), 1):
            m = _BRITISH_EXTRAS_RX.search(line)
            if m:
                findings.append(("british-spelling", rel, lineno, m.group(0),
                                 "British spelling (CANON C-11: American English "
                                 "only -- house extras list)"))
    return findings


def run_pin_check():
    """valence.pin RULE: FAIL if ../Valence is missing or its HEAD does not
    match the pin; WARN (not fail) if the sibling working tree is dirty; FAIL
    if the sibling's frozen conformance artifacts do not match our pinned
    hashes (C-6)."""
    findings = []
    try:
        pinned = PIN_FILE.read_text(encoding="utf-8").splitlines()[0].strip()
    except OSError:
        return [("pin-missing", "valence.pin", 0, "", "valence.pin is missing")]

    if not SIBLING.is_dir():
        return [("pin-sibling-missing", "../Valence", 0, "",
                 "sibling checkout not found next to this repo -- clone Valence alongside Nucleus")]

    r = subprocess.run(["git", "rev-parse", "HEAD"], cwd=SIBLING,
                       capture_output=True, text=True)
    if r.returncode != 0:
        return [("pin-sibling-not-git", "../Valence", 0, "",
                 "sibling exists but `git rev-parse HEAD` failed there")]
    head = r.stdout.strip()
    if head != pinned:
        findings.append(("pin-mismatch", "valence.pin", 0, head[:16],
                         f"../Valence HEAD {head[:16]} != pinned {pinned[:16]} -- "
                         "bump valence.pin (and re-run the gauntlet) or check out the pinned sha"))

    dirty = subprocess.run(["git", "status", "--porcelain"], cwd=SIBLING,
                           capture_output=True, text=True).stdout.strip()
    if dirty:
        print("WARN: ../Valence working tree is dirty (not a lint failure)")

    for rel, want in FROZEN_SHA256_SIBLING.items():
        p = SIBLING / rel
        if not p.exists():
            findings.append(("pin-frozen-missing", f"../Valence/{rel}", 0, "",
                             "frozen artifact is GONE from the sibling"))
            continue
        got = hashlib.sha256(p.read_bytes()).hexdigest()
        if got != want:
            findings.append(("pin-frozen-changed", f"../Valence/{rel}", 0, got[:16],
                             "frozen artifact modified in the sibling (C-6) -- protocol break unless amended"))
    return findings


def main(argv):
    findings = (run_grep_checks() + run_pin_check() + run_codespell_check()
                + run_camelcase_check() + run_critical_static_check())

    if not findings:
        print("canon_lint: clean (0 findings)")
        return 0

    findings.sort(key=lambda f: (f[0], f[1], f[2]))
    by_check = {}
    for name, rel, lineno, match, msg in findings:
        by_check.setdefault(name, []).append((rel, lineno, match, msg))
    for name, items in by_check.items():
        print("[%s] %d hit(s) -- %s" % (name, len(items), items[0][3]))
        for rel, lineno, match, _ in items[:40]:
            print("   %s:%d  %r" % (rel, lineno, match))
        if len(items) > 40:
            print("   ... and %d more" % (len(items) - 40))
    print("canon_lint: %d finding(s)" % len(findings))
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
