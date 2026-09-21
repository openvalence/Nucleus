---
paths:
  - "**"
---

# Memory safety and lifetime (operator directive 2026-07-31 -- binding)

Carried from the archived machine repo (SlopDrive-32
`.claude/rules/cpp-safety.md`). Where this file and older doctrine disagree on
a memory-safety question, this file wins.

**Scope, and it is deliberate.** These rules bind `flagship_*/src/`,
`flagship_*/ulp/`, and the first-party libraries `lib/geiger`, `lib/flux`,
`lib/kinetic`. They do NOT bind `lib/ruckig`, ESP-IDF, or anything the
component manager fetches into `managed_components/`. That code is
third-party, is re-vendored from upstream, and is full of constructs the safe
subset rejects. Enforcing there would mean choosing between a permanently red
build and patching code we do not own. `lib/valence` is the sibling repo's
surface: read-only here, governed by its own doctrine.

## The safe subset

Preferred in the scope above, applied opportunistically whenever a file is
touched for any reason:

| Instead of | Use |
|---|---|
| C arrays | `std::array<T, N>` |
| pointer plus length parameters | `std::span<T>` |
| raw `new`/`delete`/`malloc`/`free` | fixed-capacity storage; `std::unique_ptr` only when dynamic lifetime is genuinely unavoidable |
| heap allocation after startup | static / fixed-capacity |
| `std::vector` / `std::string` in steady-state code | fixed-capacity containers |
| `strcpy`/`strcat`/`sprintf` | `snprintf` at minimum |
| null as "no value" | `std::optional<T>` |
| `union` | `std::variant<A, B>` |
| bool/error-code returns for fallible operations | `std::expected<T, E>` |
| C-style casts | `static_cast`; `reinterpret_cast` only in the hardware layer |
| manual acquire/release | RAII, cleanup in destructors |

The LP core source is C, compiled by the LP toolchain, and is the one place
this table yields to what that toolchain accepts. Its own constraints are in
`motion-control.md`.

## The rules no tool enforces -- these are the ones that actually bite

Treat a violation as severity-critical in review.

- Never return a reference, `span`, or `string_view` to a local.
- Never store a `span` or `string_view` as a class member. Parameters only;
  members own their data.
- **Lambdas handed to tasks, timers or callbacks capture BY VALUE.** Never
  `[&]` for anything that outlives the enclosing scope.
- Never mutate a container while iterating it.
- Rule of zero: most classes declare no destructor, copy, or move at all.

## Concurrency

New mutable state is owned by exactly one task and reached by message, not by
shared memory. Introducing shared mutable state behind a mutex needs an
operator ruling first. The one structural exception is the HP-to-LP shared
memory window, which is single-writer per field by construction
(`motion-control.md`).

## What the toolchain enforces mechanically

- **Ten warnings are hard errors and the list is a floor, never weakened:**
  `dangling-reference`, `dangling-pointer`, `use-after-free`,
  `free-nonheap-object`, `return-type`, `uninitialized`, `array-bounds`,
  `stringop-overflow`, `nonnull`, `sizeof-pointer-memaccess`.
  **NOT YET WIRED in this repo.** Its home when it lands is
  `target_compile_options` on the project's own component, scoped so it stops
  at our sources and never reaches IDF or `managed_components/`. Do not claim
  the floor until the flags are in the build.
- Blanket `-Werror` is the goal, NOT reachable on day one: on the machine repo
  the first enable produced ~509 warnings, largely `-Wshadow` and
  `-Wconversion`, most from third-party headers. Burn the backlog down per
  file, then widen.
- Run the serial monitor with an exception-decoder filter so panic backtraces
  decode to `file:line` instead of raw addresses.

## Static analysis -- configured, NOT yet operational

`.clang-tidy` exists at repo root with the directive's `WarningsAsErrors`
floor intact and a `HeaderFilterRegex` scoped to our code. It is not running.
Both routes failed on this host in the machine repo and neither has been
retried here:

- `pio check` dies with `WinError 206: filename or extension is too long`. It
  inlines every ESP-IDF include path into one clang-tidy command and blows the
  Windows 32 KB command-line limit.
- Invoking `clang-tidy -p .` against a generated `compile_commands.json`
  reaches the compiler but hits hard errors from GCC-only flags clang rejects,
  plus a missing `stddef.h`.

The fix shape is a sanitized `compile_commands.json`: strip GCC-only flags,
add clang's builtin include path. **Do not claim clang-tidy coverage until
that lands.**

**A per-edit clang-tidy hook is REFUSED as specified.** On this host a
single-file check has not completed inside ten minutes. It belongs at a
pre-commit or on-demand gate; the fast feedback loop is the compiler.

## Lifetime traps that have bitten this codebase family

### T1 -- object reset via `*this = T{}` is a stack bomb

**Rule:** reset large objects with in-place destroy plus placement-new
(`obj.~T(); new (&obj) T();`), never `*this = T{}` or `obj = T{}`.
**Mechanism:** the right-hand `T{}` is a full temporary constructed ON THE
CURRENT STACK before assignment. A multi-KB object on a small FreeRTOS task
stack overflows it instantly. Host-side tests never catch this class because
desktop threads get megabyte stacks. Enforced by canon_lint `this-assign`.
Bit the machine repo: a ~9 KB session temporary panicking the hub task on
every client connect (July 2026).

### T4 -- no function-local statics inside critical sections

**Rule:** never declare a function-local `static` of class type inside
`portENTER_CRITICAL` or any no-abort context; hoist to file scope.
**Mechanism:** first execution of a function-local static registers its
destructor via `__cxa_atexit` and takes an init-guard lock. Both can allocate
or abort, and inside a critical section that aborts the core. A path that has
never run live hides this until the first real use.

### T9 -- C++ default arguments bind to the STATIC type

**Rule:** forwarding proxies and wrappers pass explicit sentinels through to
the base; they never restate the base's default arguments.
**Mechanism:** default args are substituted at the CALL SITE from the declared
static type of the expression, not the dynamic type, so a proxy that
redeclares a default silently overrides a subclass's different default.

### T29 -- legacy, not reachable here

An async-TCP dispatch loop that destroyed its own caller's object. There is no
AsyncTCP in this repo. The case file, including the `0xfefefefe` poison-read
technique, is the archived SlopDrive-32 `.claude/rules/cpp-safety.md` T29;
the reusable rule is: before a loop calls user code, ask whether that call can
free the
object the loop is iterating on, and if it can, keep the liveness check
OUTSIDE the object.
