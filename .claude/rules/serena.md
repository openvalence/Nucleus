---
paths:
  - "**"
---

# Serena / clangd navigation upkeep

## The compile database is PER PROJECT, and the build makes it

Each board is its own PlatformIO project, so each has its own database at
`flagship_<chip>/.pio/build/<env>/compile_commands.json`. The ESP-IDF CMake
build emits it directly -- no generator script, no `compiledb` target
[verified 2026-09-20 -- 28 C++ compile commands in the flagship_p4 database,
every one carrying `-std=gnu++2b`, `val-091.3`]. **Rebuilding the project IS
the regeneration.** Rebuild after any change to `platformio.ini`,
`sdkconfig.defaults`, or a `CMakeLists.txt` include list, or symbol search
goes quietly stale.

Point `.clangd` at the project whose code you are reading. There is one
database per board and they carry different flags and different include roots;
a query answered against the wrong one is worse than no answer, because it
looks like an answer. `.clangd` at the repo root is that pointer and its ONE
home: it names `flagship_p4/.pio/build/flagship_p4`, strips the GCC-only flags
clang rejects, and carries a fragment per build that is not the HP core.

The LP/ULP translation unit is a SECOND database. `ulp_embed_binary` runs its
own cmake project, so `flagship_p4/ulp/lp_quad.c` has no entry in the main
database and its real command lives at
`flagship_p4/.pio/build/flagship_p4/esp-idf/src/ulp_main/compile_commands.json`.
A path fragment in `.clangd` routes `flagship_p4/ulp/` there. Without it clangd
infers an HP-core command and every LP register reads as unknown.

**`BuiltinHeaders: QueryDriver` is not optional here** [verified 2026-09-20 --
`clangd --check`, `val-091.6`]. `tool-clangd-esp` ships an EMPTY resource dir
(`lib/clang/21/include` has no `stddef.h`), so clangd's own builtin headers do
not exist and EVERY file reports `'<cstdint>' file not found` however good the
database is. QueryDriver takes them from the GCC driver named in the compile
command, where they do exist. It needs `--query-driver` to allow that driver;
the host's `CLANGD_FLAGS` environment variable already globs
`.platformio/packages/toolchain-*/bin/*`, which covers riscv32-esp-elf. Without
that allowance the setting silently does nothing, which reads as the database
being wrong.

Do NOT reach for `pio run -t compiledb`. On the machine repo that target
emitted only framework and managed-component translation units, zero
first-party entries, in every environment -- and reported "up to date" while
doing nothing unless the old file was deleted first.

## Verify against real symbols, never by assuming

Two checks, both required. They catch different failures.

1. **Parse health:** `get_symbols_overview` on
   `flagship_p4/src/hub/ValenceCatalog.h` must report kind `Namespace` for
   `valence`. Kind `Variable` means the file is being parsed as C.
2. **Index health:** `find_referencing_symbols` on `hubBegin` in
   `flagship_p4/src/hub/ValenceHub.cpp` must return `main.cpp`. Intra-file hits
   with nothing cross-file means the background index is off or stale, and
   check 1 passes anyway, so the namespace check alone will not catch it.

Both pass [verified 2026-09-20 -- clangd LSP `documentSymbol` gave
`valence kind=Namespace`, `references` gave `main.cpp:317`, `val-091.6`]. An
empty reference result never proves a symbol is unused (`navigation.md`: a C-9
deletion needs both Serena and Grep to agree).

**A missing include root reads exactly like a broken language server.** clangd
can only be as complete as the compile command, so a component the board's
`idf_component_register` never declared in `REQUIRES` is absent from the
database, and its types go unknown in the editor while the namespace check
above still passes. Before blaming the index, replay the recorded command with
the real GCC: if it does not compile, nothing downstream of it can.

## Two traps that make correct output read as wrong

**Line numbers are 0-BASED.** Every `body_location` and reference line is one
less than the editor, Grep, and `file:line` links. Add 1 before citing.
Measured 2026-08-04: Serena reported a call site at 147, the file has it at
148.

**A cold index answers `{}`.** Serena spawns clangd at session start and the
background index needs roughly a minute before cross-file references resolve;
the same query returned `{}` at 23 seconds and the correct hit a few minutes
later. Never read an early-session empty as "no callers": re-run it, and if it
stays empty, check the index setting.

## Local constraints

- `.clangd` sets `Index: Background: Build`, and it MUST stay Build. Under
  `Skip` there is no project-wide index: clangd knows only the symbols in
  files it has parsed, so `find_referencing_symbols` resolves intra-file and
  returns EMPTY across translation units -- indistinguishable from "no
  callers", which is how a C-9 deletion proof silently becomes a lie. The cost
  is a one-time background index of the database's translation units.
- Scope symbolic queries with `relative_path` anyway; an unscoped
  `find_symbol` walks files and can terminate the language server.
- The vendored Valence surface under `lib/valence` is read-only from this
  repo. Navigate into it freely; changes there are RFCs, not edits.
