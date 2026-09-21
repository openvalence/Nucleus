#!/usr/bin/env bash
# BEADS GATE (PreToolUse, TodoWrite): task tracking lives on the dev board
# (governance.md §4). TodoWrite state dies with the session and is invisible
# to every other agent and to the operator; the board survives compaction and
# is the sole home for volatile work state (C-1, C-2).
# PORT of SlopDrive-32/.claude/hooks/beads-gate.sh, unchanged.
# The harness injects its own "consider using TodoWrite" reminders, which is
# why prose alone did not hold: this is the mechanical floor (§5).
>&2 cat <<'MSG'
BEADS GATE: TodoWrite is prohibited in this repo. Task tracking lives on the
dev board, which survives compaction and is visible to the operator.

  bd ready                      # find work with no blockers
  bd create --title="..." --description="why + what" --type=task --priority=2
  bd update <id> --claim        # start work
  bd close <id> [<id>...]       # finish

Run `bd prime` first if you have no board context (after compaction, or in a
subagent, which never receives the SessionStart hook output).
MSG
exit 2
