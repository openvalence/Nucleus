# TIDY GATE (PreToolUse, Write): the repo root is a closed set, see
# .claude/rules/repo-layout.md. PORT of the archived SlopDrive-32 repo's
# .claude/hooks/tidy_check.py,
# unchanged: the routing table is tree-independent. Fires only on NEW files whose parent is the
# root; overwriting an existing root file is untouched, so this never blocks
# editing a tracked root file.
import json
import os
import sys

# Generated or reference payloads. Source and config extensions are absent on
# purpose: a genuinely new top-level .py or .ini is a deliberate act.
BLOCK_EXT = {
    ".json", ".jsonl", ".csv", ".log", ".txt", ".md", ".html", ".pdf",
    ".png", ".jpg", ".jpeg", ".svg", ".bin", ".elf", ".map", ".zip",
    ".js", ".mjs", ".diy",
}

ROUTE = (
    ((".json", ".jsonl", ".csv", ".log", ".bin", ".elf", ".map", ".zip"),
     "artifacts/", "per-run output"),
    ((".pdf", ".png", ".jpg", ".jpeg", ".svg", ".diy"),
     "docs/reference/", "reference material (screenshots of a run go in artifacts/)"),
    ((".md", ".txt", ".html"), "docs/", "prose that outlives the run"),
    ((".js", ".mjs"), "tools/", "scripts and probes"),
)


def verdict(path):
    root = (os.environ.get("CLAUDE_PROJECT_DIR") or "").replace("\\", "/").rstrip("/")
    p = (path or "").replace("\\", "/")
    if not root or not p:
        return None
    if os.path.dirname(p).lower() != root.lower():
        return None
    if os.path.exists(p):
        return None
    ext = os.path.splitext(p)[1].lower()
    if ext not in BLOCK_EXT:
        return None
    dest, why = next((d, w) for exts, d, w in ROUTE if ext in exts)
    return os.path.basename(p), dest, why


def main():
    try:
        sys.stdin.reconfigure(encoding="utf-8")
        data = json.load(sys.stdin)
    except Exception:
        return 0
    v = verdict((data.get("tool_input") or {}).get("file_path"))
    if not v:
        return 0
    name, dest, why = v
    print(
        "TIDY GATE: %s would be a new loose file in the repo root, which is a "
        "closed set (.claude/rules/repo-layout.md).\n"
        "  Write it to %s%s instead  [%s]\n"
        "If this really belongs at the root, say so and the operator adds it "
        "deliberately; the gate does not guess." % (name, dest, name, why),
        file=sys.stderr,
    )
    return 2


def selftest():
    root = os.path.abspath(".").replace("\\", "/")
    os.environ["CLAUDE_PROJECT_DIR"] = root
    cases = [
        (root + "/drift_full.json", "artifacts/"),
        (root + "/YZ-Manual.pdf", "docs/reference/"),
        (root + "/HANDOFF.md", "docs/"),
        (root + "/smoke7.js", "tools/"),
    ]
    for p, dest in cases:
        v = verdict(p)
        assert v and v[1] == dest, "%s: got %r, want %s" % (p, v, dest)
        print("ok   blocks %-28s -> %s" % (os.path.basename(p), dest))
    allowed = [
        (root + "/artifacts/drift_full.json", "not at root"),
        (root + "/platformio.ini", "extension not blocked"),
        (root + "/README.md", "already exists"),
        ("/some/other/repo/x.json", "outside project root"),
    ]
    for p, why in allowed:
        assert verdict(p) is None, "%s should be allowed (%s)" % (p, why)
        print("ok   allows %-28s (%s)" % (os.path.basename(p), why))
    print("tidy_check selftest: all pass")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        sys.exit(main())
