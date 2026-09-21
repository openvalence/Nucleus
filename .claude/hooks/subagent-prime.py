# SUBAGENT PRIME (SubagentStart): SessionStart hooks do NOT fire for subagents,
# so `bd prime` never reaches them. A third of this repo's tool traffic is
# subagent spawns, and each one starts with no idea what is already known,
# already open, or already ruled on. Doctrine reaches them via .claude/rules
# (all paths:"**"); this supplies the volatile half.
# Fails SILENT and exit 0 always: a hook that breaks agent startup is worse
# than one that adds nothing.
import json
import subprocess
import sys

TIMEOUT = 10


def bd(*args):
    try:
        r = subprocess.run(("bd",) + args, capture_output=True, text=True,
                           timeout=TIMEOUT)
        return r.stdout if r.returncode == 0 else ""
    except Exception:
        return ""


def main():
    # ONE bd call, not one per area: this runs on every subagent spawn and the
    # per-area loop cost 2.1 s of pure startup latency.
    try:
        d = json.loads(bd("list", "--status=open", "--json") or "[]")
        items = d if isinstance(d, list) else d.get("issues", [])
    except Exception:
        return 0

    tally = {}
    urgent = []
    for i in items:
        for lab in (i.get("labels") or []):
            if lab.startswith("area:"):
                tally[lab[5:]] = tally.get(lab[5:], 0) + 1
        if i.get("priority") in (0, 1, "0", "1") and len(urgent) < 6:
            urgent.append("%s %s" % (i.get("id", "?"), (i.get("title") or "")[:70]))

    counts = ["%s %d" % (k, v) for k, v in
              sorted(tally.items(), key=lambda kv: -kv[1])]

    if not counts and not urgent:
        return 0

    parts = ["You are a SUBAGENT: SessionStart hooks did not run for you, so "
             "this is your board context. TodoWrite is blocked in this repo "
             "(a hook rejects it); task state lives on the dev board."]
    if counts:
        parts.append("Open by area: " + ", ".join(counts) + ".")
    if urgent:
        parts.append("Open P1: " + "; ".join(urgent) + ".")
    parts.append("Run `bd show <id>` before assuming a thing is unknown, and "
                 "`bd ready` for claimable work. Check the board BEFORE "
                 "investigating: the answer is often already stamped there.")

    print(json.dumps({"hookSpecificOutput": {
        "hookEventName": "SubagentStart",
        "additionalContext": " ".join(parts),
    }}))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        sys.exit(0)
