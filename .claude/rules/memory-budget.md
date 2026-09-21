---
paths:
  - "**"
---

# Memory discipline

The P4's numbers, and the rules the machine repo paid for in field bugs. Every
figure here is stamped; re-measure rather than reason from an old one.

## The P4's actual memory, measured

[verified 2026-09-20 -- clean build, flash COM15, console capture; board
`val-091.2` and `val-091.3`]

| | |
|---|---|
| Linkable SRAM, from the linker map | `sram_low` 0x2CBD0 + `sram_high` 0x60000 = **576 KB** |
| Boot heap, no network, no PSRAM | 589 KB |
| Internal heap free at `app_main`, PSRAM enabled | 564,067 B |
| Internal free / maxblock, steady state with PSRAM, both emitters and WiFi up | 222,275 / **155,648** |
| PSRAM free / maxblock, same run | 33,551,348 / 33,030,144 |
| PSRAM device | 32 MB, hex mode, 200 MHz |
| Static RAM in the bench image | 23,308 B (flash 676,026 B) |
| Cost of the network stack | ~154 KB internal, nothing in PSRAM |

- **The build summary's "RAM: 327680" is a LIE on this board.** It is the
  board manifest's number, copied from an S3. The chip's linkable SRAM is the
  linker map's, 576 KB. Never size anything against the summary line.
- **155,648 B (152 KiB) internal maxblock is the real headroom** for
  allocations that MUST be internal: DMA descriptors, ISR buffers, task
  stacks, and anything touched while the flash cache is disabled. A service
  with large state does not belong there (T2).
- **The reserve pool is 32,768 B** (`CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL`),
  held back from `malloc()` so PSRAM-eligible traffic cannot starve
  internal-only allocations. Allocations at or below 4,096 B never reach PSRAM
  at all (`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL`), so that is the floor any
  heap census reasons against. Both values live in
  `flagship_p4/sdkconfig.defaults` (C-1); do not lower the reserve without a
  measured internal maxblock.
- **250 MHz PSRAM is unreachable on this stamp:** its Kconfig depends on
  `!ESP32P4_SELECTS_REV_LESS_V3` and this is ECO2 silicon.
- **Task stacks stay in internal RAM** (ruling 2026-09-20,
  `governance.md` §6). PSRAM is unreachable while the flash cache is disabled.

## Pools, never per-event malloc

- Steady-state comms allocate nothing: fixed-capacity rings, sized for burst
  tolerance rather than for the steady rate, with a
  never-blocks-never-allocates producer contract.
- Big scratch lives in members, not stack locals. A kilobyte of stack deep
  inside a hub update is not free (T1 class).

## Budgeting rules proven by blood

- **Budget any diagnostic's RAM against heap minimum UNDER LOAD, never idle
  free.** On the machine repo a 9 KB `.bss` diagnostic table flipped an A/B
  arm from 4-reboots-in-6 to 0-in-7 and separately pushed the internal minimum
  free to 88 bytes. The clean-looking arm was the artifact.
- **A detector reports, it never adjudicates.** A double-free detector that
  called `abort()` on detection bricked the OTA recovery path: the device
  rebooted mid-flash and could no longer be updated over the network.
- **Oversized allocations cost their size plus the hole they leave.**
  Shrinking a 16 KB task stack to 8 KB returned 11,196 B, not 8,192. But never
  trim a stack whose comment records a canary blowout (T1).

## T2 -- big statics starve the internal heap

**Rule:** services with large state live in PSRAM via placement-new from the
composition root, never as ordinary statics or BSS.
**Mechanism:** BSS eats internal SRAM at link time, and the network stack
allocates from the same internal heap at runtime. On the machine repo a
~100 KB static left ~13 KB free heap and killed the network stack. Here the
hub service is the same shape and the same rule applies: 152 KiB of internal
maxblock is not a place to park a service that has 32 MB of PSRAM available.
Watch a boot heap beacon that prints free AND maxblock for both heaps.

## T21 -- free heap is not contiguous heap, and a high-water mark only knows the paths it has walked

Two measurement traps, one family: the number that is easy to read is not the
number that decides anything.

**Fragmentation, not exhaustion.** An allocator can hold plenty of total free
space split into pieces whose largest is under what one request needs, and
then that request fails forever while every free-heap reading looks
survivable. Measured on the machine repo: `free=23408 maxblock=11252` against
a 12,288 B need, 5 of 5 serves refused. **Watch `maxblock`; `free` is the
comforting lie.** The P4 liveness line prints `int_free`, `int_max`,
`psram_free` and `psram_max` for exactly this reason.

**A high-water mark is only as good as the workload since boot.**
`uxTaskGetStackHighWaterMark` reports the deepest point actually reached,
which on an idle machine means nothing interesting ran yet. A motion task
measured with the motor unplugged has never executed its reason for existing,
and a flashing task's deepest path is followed by a reboot that resets the
mark. Sizing from that census panics mid-session weeks later, and the canary
names the task but not the day you caused it.
**Fix:** re-scan periodically and report ONLY tasks that have gone DEEPER than
last reported, so a bench session that exercises motion names the stacks it
actually grew. Trim only after a representative workload has produced no new
regressions.

## T27 -- instrumentation expensive enough to change what it measures

**Rule:** a diagnostic on a hot path is judged by its CALL RATE, not by its
correctness. Before adding a probe, multiply its cost by how often the
enclosing function actually runs. If the product is a meaningful fraction of
the budget of the task it sits on, the probe will manufacture a different
failure and hide the one you are hunting.
**Mechanism:** `heap_caps_check_integrity_all()` walks every block header in
every region, so milliseconds. That is nothing once per session and fatal once
per frame. A probe that starves its own task produces a watchdog reboot, and a
watchdog reboot looks like a finding. You then discover a bug you created.
**Bit the machine repo twice in one session:** probes in a 5 ms loop saturated
the hub task, made the device unreachable and needed a serial rescue; a probe
at frame entry fired ~56,000 whole-heap scans in one run and turned a
reproducible panic into a watchdog reset, destroying the evidence.
**Fix:** probes live on per-SESSION paths only (connect, refuse, disconnect)
and any hot-loop probe is rate-limited to 1 Hz.

## The S3's heap case file is NOT re-derived here

The machine repo's `.claude/rules/memory-budget.md` holds a long record that
this board does not inherit: split-plane starvation under WS load and its
refusal floors (T19 and its three addenda), the twelve-trial hunt for a heap
corruption whose culprit was never found and the seven things that did NOT
find it (T28), and the six ways a diagnostic lies about itself (T30). Read it
before building a load shedder, an A/B harness, or any heap detector; do not
copy its numbers, which predate this silicon by a whole architecture. Two of
its lessons are load-bearing enough to restate as rules and are above (T27,
and "a detector reports, it never adjudicates").
