# Style gate body. Reads the hook JSON on stdin, inspects only the text the
# tool call would ADD (new_string / content), exit 2 blocks the write.
# PORT of the archived SlopDrive-32 repo's .claude/hooks/style_check.py; only
# SKIP_PATHS changed
# (this tree has different vendored directories).
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

SKIP_PATHS = (
    "/.beads/", "/node_modules/", "/managed_components/", "/site/",
    "license", "/vectors/",
    # vendored third-party trees; inert in the spec repo
    "/lib/ruckig/",
)

# Fallback only. codespell's en-GB dictionary is preferred when installed
# (operator ruling 2026-07-28: no rival hand-rolled wordlists; this short list
# exists so the hook still catches the common cases without codespell).
BRITISH_RX = re.compile(
    r"\b(behaviours?|colours?|favours?|honours?|centres?|metres?|litres?|"
    r"fibres?|licence[sd]?|defence[sd]?|offence[sd]?|catalogue[sd]?|"
    r"analyse[sd]?|analysing|organis(?:e[sd]?|ing|ations?)|"
    r"initialis(?:e[sd]?|ing)|serialis(?:e[sd]?|ing)|synchronis(?:e[sd]?|ing)|"
    r"normalis(?:e[sd]?|ing)|optimis(?:e[sd]?|ing)|minimis(?:e[sd]?|ing)|"
    r"maximis(?:e[sd]?|ing)|greys?|artefacts?|aluminium|whilst|amongst|"
    r"travell(?:ed|ing|ers?)|cancelled|labelled|modelled|signalled|"
    r"programmes?|tyres?|moulds?|judgements?|acknowledgements?)\b",
    re.IGNORECASE,
)

CODE_EXT = {".c", ".cc", ".cpp", ".h", ".hpp", ".js", ".mjs", ".ts", ".py", ".sh"}
HASH_COMMENT_EXT = {".py", ".sh"}


def governed(path):
    # C-11/C-12 bind this repo and the sibling Valence checkout, nothing else.
    # Without this, the gate fires on scratchpad temp files and agent memory
    # outside either tree, where the canon has no jurisdiction. Fails CLOSED:
    # no project context means check anyway.
    root = (os.environ.get("CLAUDE_PROJECT_DIR") or "").replace("\\", "/").lower().rstrip("/")
    if not root:
        return True
    sibling = root.rsplit("/", 1)[0] + "/valence"
    return path.startswith(root + "/") or path.startswith(sibling + "/")


def added_only(ti):
    # Edit sends the whole new_string, anchor lines included, so an untouched
    # line carrying an em dash used to fail the write. Compare against
    # old_string and keep only lines that are genuinely new.
    # ponytail: line-set diff, not a real diff -- a line MOVED within the hunk
    # reads as unchanged. Upgrade to difflib if that ever hides a real hit.
    new, old = ti.get("new_string"), ti.get("old_string")
    if new and old:
        seen = set(old.splitlines())
        return ["\n".join(ln for ln in new.splitlines() if ln not in seen)]
    return [t for t in (new, ti.get("content")) if t]


def comment_ratio(path, text):
    ext = "." + path.rsplit(".", 1)[-1] if "." in path else ""
    if ext not in CODE_EXT:
        return None
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    if len(lines) < 8:
        return None
    if ext in HASH_COMMENT_EXT:
        com = sum(1 for ln in lines if ln.startswith("#"))
    else:
        com = sum(1 for ln in lines if ln.startswith(("//", "/*", "*", "*/")))
    return com / len(lines)


def codespell_hits(text):
    exe = shutil.which("codespell")
    if not exe:
        return None
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False, encoding="utf-8") as f:
        f.write(text)
        tmp = f.name
    try:
        out = subprocess.run(
            [exe, "--builtin", "en-GB_to_en-US", tmp],
            capture_output=True, text=True, timeout=15,
        )
        return [ln.rsplit(":", 1)[-1].strip() for ln in out.stdout.splitlines() if "==>" in ln]
    except Exception:
        return None


def main():
    try:
        sys.stdin.reconfigure(encoding="utf-8")  # Windows defaults to cp1252
        data = json.load(sys.stdin)
    except Exception:
        return 0
    ti = data.get("tool_input", {})
    path = (ti.get("file_path") or "").replace("\\", "/").lower()
    texts = added_only(ti)
    if not texts or not any(t.strip() for t in texts):
        return 0
    if not governed(path) or any(s in path for s in SKIP_PATHS):
        return 0
    text = "\n".join(texts)

    errs = []
    if "—" in text:
        errs.append('em dash (U+2014) in new text: use "--", a comma, or restructure')
    hits = codespell_hits(text)
    if hits:
        errs.append("British spellings (codespell en-GB): " + "; ".join(hits[:6]))
    elif hits is None:
        m = sorted(set(w.lower() for w in BRITISH_RX.findall(text)))
        if m:
            errs.append("British spellings: " + ", ".join(m[:8]) + " (C-11: American English)")
    ratio = comment_ratio(path, text)
    if ratio is not None and ratio > 0.5:
        errs.append(
            f"comment-to-code ratio {ratio:.0%} in new code: comments state "
            "constraints, not stories (C-12). Move narration to the docs and link it."
        )
    if errs:
        print("STYLE GATE: " + " | ".join(errs), file=sys.stderr)
        return 2
    return 0


def selftest():
    # Fixtures are built from parts so this file does not trip its own gate.
    em, gb = "\u2014", "behavi" + "our"
    root = "c:/repo"
    os.environ["CLAUDE_PROJECT_DIR"] = root
    cases = [
        ("em dash in added text blocks",
         {"file_path": root + "/src/x.cpp", "old_string": "int a;",
          "new_string": "int a;\n// hi " + em + " there"}, 2),
        ("em dash only in anchor passes",
         {"file_path": root + "/src/x.cpp", "old_string": "// k " + em + " p\nint a;",
          "new_string": "// k " + em + " p\nint b;"}, 0),
        ("out-of-tree path is not governed",
         {"file_path": "c:/temp/scratch/x.py", "content": "s = '" + gb + "'"}, 0),
        ("sibling Valence is governed",
         {"file_path": "c:/valence/spec/RFC-QUEUE.md", "content": "draft " + em + " text"}, 2),
        ("en-GB spelling in repo blocks",
         {"file_path": root + "/src/y.cpp", "content": "// " + gb + " of the thing"}, 2),
    ]
    for name, ti, want in cases:
        p = subprocess.run([sys.executable, __file__], input=json.dumps({"tool_input": ti}),
                           capture_output=True, text=True)
        assert p.returncode == want, "%s: exit %d, want %d\n%s" % (name, p.returncode, want, p.stderr)
        print("ok  " + name)
    print("style_check selftest: all pass")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        sys.exit(main())
