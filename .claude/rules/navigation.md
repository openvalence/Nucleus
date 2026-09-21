---
paths:
  - "**"
---

# Code navigation: discovery, then ground truth

Two tools, two jobs. **Grep to FIND, LSP to UNDERSTAND.** This is a workflow
rule, not a preference; follow it on every C++ task. Using the discovery tool
for a C-9 deletion proof is how live code gets deleted with a clean
conscience.

- Grep/Glob for DISCOVERY: finding files, string and pattern search, "where is
  this text mentioned".
- Symbolic tools for UNDERSTANDING: definitions, references, types.
  `find_symbol`, `find_referencing_symbols`, `find_declaration`,
  `get_symbols_overview`, hover.
- After locating a file, navigate WITHIN it via symbols, never by reading the
  whole file. `get_symbols_overview` first, then `find_symbol` with a
  `name_path` and only the depth you need. Reading a 2,600-line translation
  unit to answer "what does this class expose" is the thing this rule exists
  to stop.

## Discovery: codebase-memory (start here)

Finding your way in: what exists, where a subsystem lives, which functions are
hairy. `search_graph(query="...")` takes NATURAL LANGUAGE and is the only tool
here that does, so it is the right first move when you do not yet know the
symbol name. Also `get_architecture`, plus complexity and degree metadata.
Its line numbers are 1-BASED.

## Ground truth: serena (before you touch anything)

Its tools are DEFERRED, so load the schema once per session:

```
ToolSearch("select:mcp__serena__find_symbol,mcp__serena__find_referencing_symbols")
```

Use serena, not the graph, for the definition of a symbol, every caller of it,
and anything a decision rests on. Its line numbers are 0-BASED: add 1 before
citing one as `file:line`.

## The boundary is measured, not a style preference

The graph resolves call edges by NAME, not by type. Tested 2026-08-04 on the
machine repo. One probe, wrong in both directions:

- MISSED a call through a member reference, though the member's declaration
  sat in the same class's header.
- INVENTED an edge into an unrelated `operator==`, built from a
  `xQueueReceive(...) == pdTRUE` comparison, crossing into a target that
  compiles into no firmware build at all.

It is strong where names are unique and weak where they collide, which is the
opposite of when you need the help. Serena asks clangd, which is the
compiler's own answer.

## C-9 proof-of-no-callers

Serena AND Grep, reconciled. The graph counts as neither. Grep cannot see
through an `#include` or tell a declaration from a call; serena is limited by
its index. They fail in opposite directions, which is why both are required.

An EMPTY serena reference result is INDEX SUSPECT, never "no callers". Treat
it that way until some symbol you know is called comes back non-empty. Two
causes: a cold index for about a minute after session start, or the
background index switched off in `.clangd`. See `serena.md` for the check and
the repair.

## Precedence over the installed reminder

codebase-memory installs SessionStart and SubagentStart hooks saying to use its
tools FIRST for ANY code exploration. In this repo that holds for DISCOVERY
only. It does not override the C-9 rule above, and this file wins. The hooks
are vendor-installed at user scope and get rewritten on upgrade, so the
override lives here rather than as an edit to them.
