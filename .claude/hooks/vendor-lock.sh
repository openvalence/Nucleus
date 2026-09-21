#!/usr/bin/env bash
# VENDOR-LOCK (PreToolUse, Edit|Write): the sibling SlopSync checkout's
# NORMATIVE + CONSUMED surfaces are READ-ONLY from this repo: the vendored
# library (lib/slopsync, a symlink to ../SlopSync/lib/slopsync, sha pinned
# in slopsync.pin), spec/SPEC.md, spec/registry/, and codegen
# output. Divergence is EXPECTED and resolves upstream via RFCs (drafted in
# the sibling's spec/RFC-QUEUE.md, which stays writable), then lands there
# and comes back as a pin bump. Other sibling files (its .claude, docs,
# clients) are governed by SlopSync's own hooks when working there.
# PORT of SlopDrive-32/.claude/hooks/vendor-lock.sh; same protected surface,
# same one writable door (spec/RFC-QUEUE.md).
set -u
IN=$(cat)
FILE=$(printf '%s' "$IN" | python -c "import json,sys;print(json.load(sys.stdin).get('tool_input',{}).get('file_path',''))" 2>/dev/null || true)
[ -z "$FILE" ] && exit 0
NORM=$(printf '%s' "$FILE" | tr '\\' '/' | tr '[:upper:]' '[:lower:]')
case "$NORM" in
  */documents/slopsync/* | ../slopsync/* | ../../slopsync/*) ;;
  *) exit 0 ;;
esac
case "$NORM" in
  */slopsync/lib/slopsync/* | */slopsync/spec/spec.md | */slopsync/spec/registry/* | */slopsync/*/generated/*)
    >&2 printf 'VENDOR-LOCK: %s is vendored/normative SlopSync surface, read-only from Valence Drive. Protocol changes belong in an RFC upstream: draft it in ../SlopSync/spec/RFC-QUEUE.md, get the ruling, land it there, then bump slopsync.pin here. Never edit the vendored spec to match this code.\n' "$FILE"
    exit 2
    ;;
esac
# The sibling's own protect-generated hook does NOT run from this repo: hooks
# are not loaded across workspace roots. Its exclusive surface is covered here.
case "$NORM" in
  */slopsync/docs-site/site/*)
    >&2 printf 'VENDOR-LOCK: %s is docs-site build output, regenerated from ../SlopSync/docs-site/docs. Edit the source there and rebuild; never hand-edit the built site.\n' "$FILE"
    exit 2
    ;;
esac
exit 0
