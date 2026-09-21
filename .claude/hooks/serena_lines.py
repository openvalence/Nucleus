# SERENA LINE BASE (PostToolUse, mcp__serena__*): serena reports 0-BASED line
# numbers by design (LSP native; see its own info_prompts.yml). Everything else
# an agent touches -- Read, Grep, editors, file:line links -- is 1-based, so a
# number passed straight through lands one line early. Not configurable, so the
# correction is injected next to the numbers instead of documented far away.
import json
import sys

# Only tools that actually emit line numbers. Memory and edit tools do not,
# and a reminder on those is pure noise.
LINE_BEARING = (
    "find_symbol", "find_referencing_symbols", "get_symbols_overview",
    "find_declaration", "find_implementations", "get_diagnostics_for_file",
)

NOTE = ("Serena line numbers above are 0-BASED. Add 1 before citing any of them "
        "as file:line, in a commit message, on the board, or in a comment. "
        "An empty reference result means the index is cold or off, never "
        "'no callers' (.claude/rules/serena.md).")


def main():
    try:
        sys.stdin.reconfigure(encoding="utf-8")
        data = json.load(sys.stdin)
    except Exception:
        return 0
    name = data.get("tool_name") or ""
    if not name.startswith("mcp__serena__"):
        return 0
    if not any(name.endswith(t) for t in LINE_BEARING):
        return 0
    print(json.dumps({"hookSpecificOutput": {
        "hookEventName": "PostToolUse",
        "additionalContext": NOTE,
    }}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
