---
paths:
  - "**"
---

# Building, testing, and deployment

This file is the LAW: scope, gates, and the traps that make a green result a
lie. Per-board build entry points live in each `flagship_<chip>/platformio.ini`
(C-1); this file never restates a flag value.

## Building

- pio: `%USERPROFILE%\.platformio\penv\Scripts\platformio.exe`, host
  Windows 11.
- **Run it from PowerShell, never Git Bash.** A Git Bash invocation of a built
  binary has returned exit 127 with no output while the same binary ran
  correctly under PowerShell. A Bash 127 is a shell artifact, never a broken
  build or a broken suite.
- Build: `pio run -d flagship_p4`. Flash: `pio run -d flagship_p4 -t upload`.
  `-d` is not optional: each board is **its own PlatformIO project**
  (`governance.md` §6), and the reason is the ULP build hook, which resolves
  exactly one `$PROJECT_DIR/ulp` with no per-environment override.
- `python tools/canon_lint.py` gates every substantive change; zero findings
  is the bar.
- Host tests for the liftable libraries (`lib/vlog`, `lib/vglow`,
  `lib/vmotion`) are the check the namespace rename is gated on. That
  environment does not exist yet: do not cite a native suite as a gate until
  it does.

## The bench

- **COM15 is the P4.** COM5, COM7, COM11 and COM13 belong to other boards on
  this bench (an RP2350, a C3 air-quality node, an S3 and a C5). Never flash
  this project at any of them.
- **Console is USB-Serial/JTAG at 115200** (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`;
  the stamp has no other console). **Open the port with DTR and RTS LOW.** A
  plain port-open sends a DTR/RTS pulse that USB-Serial/JTAG interprets as a
  reset: on the S3 that killed a live operator session and its whole diag
  archive. Opening with the handshake lines asserted is safe only when a
  reboot is acceptable, such as catching a boot banner after a flash.

## Configuration homes

- `flagship_<chip>/sdkconfig.defaults` is HAND-WRITTEN and tracked; the
  generated `sdkconfig.<env>` beside it is DERIVED and gitignored. Edit the
  defaults, never the generated file, and never both.
- Three facts have a half on each side of the PlatformIO / IDF boundary, and
  both halves must move together: silicon revision (`board = esp32-p4`, not
  `esp32-p4_r3`, and `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y`), flash size
  (`board_upload.flash_size` and `CONFIG_ESPTOOLPY_FLASHSIZE_16MB`), and the
  upload port. **This stamp is ECO2 engineering silicon**, pre rev 3.0. A
  rev.300 image panics with an illegal instruction AT THE APP ENTRY POINT and
  watchdog-loops forever: it fails before any of your code runs, so it reads
  as a dead board.

## Traps, each measured 2026-09-20

**1. THE CLEAN-BUILD TRAP -- any config change that moves the memory map.**
After a `sdkconfig.defaults` change that touches silicon revision, cache size,
PSRAM, or the ULP reserve, `$BUILD_DIR/memory.ld` is NOT regenerated;
`sections.ld` is. The mismatched pair links every SRAM section at 0x0. The
symptom is an entry point around `0xc94` and
`Instruction access fault PC=0x00000c94` from the bootloader, then a watchdog
loop. **Cure is a CLEAN build: delete `.pio`.** Do not chase it in the config,
and do not read it as a bad config value.

**2. `-O2`, NEVER `-Os`.** ESP-IDF's RISC-V `memcpy`/`memmove`/`strcpy` carry
`#error ... DIG-694` behind `#ifdef __OPTIMIZE_SIZE__` -- a silicon erratum
requiring at least two instructions between a load and the store of that
register. A size-optimized P4 build cannot compile newlib at all. The
motion-side half of this fact is the emitter's one-store-per-edge rule
(`motion-control.md`).

**3. The ULP shape is fixed by the builder, not by taste.** LP core sources
live in `<project>/ulp/` and nowhere else; the app is embedded with
`ulp_embed_binary`; and **the app name MUST be `ulp_main`** because
pioarduino's `ulp.py` hardcodes it. `framework = arduino` cannot ship a ULP
binary at all: hybrid mode builds it in a throwaway stage and the final link
never sees it (measured: `nm` on `firmware.elf` carried no
`_binary_ulp_main_bin_*`). That is why this project is pure ESP-IDF, and it is
a ruling, not a preference. The machine repo's `custom_sdkconfig` trap --
where editing that block wiped and reinstalled framework packages, so a failed
first build was not necessarily a real failure -- does NOT apply here and must
not be cited as a reason to re-run a failed build.

## Deployment

**There is no OTA on this board yet, and no bridge to flash it through.**
Every flash is a bench act over USB. Do not describe anything here as
"deployed"; C-8 means version-verified on-device, and there is no version
constant yet (`governance.md` §1). The planned route is ESP-IDF's native
`esp_ota_ops` with its own rollback, which the part supports directly -- the
two-hop forwarder the machine repo needed is a consequence of a topology this
board does not have.

**Before flashing:** machine idle, operator aware. **After flashing:** read
the boot banner and confirm the thing you changed is the thing that came up.
Upload completed is not deployed (C-8).

## T10 -- a test runner misreports its own suite

- **Trust the process exit code, or run the built binary directly.** Never
  trust a runner's parsed summary. "0 test cases" plus a clean exit is a
  cosmetic lie that leaves the PREVIOUS suite's binary in place: running that
  stale binary five times gave five identical pass lines for five different
  suites whose real counts differ. The tell was identical counts across
  suites.
- **A filter name needs its subdirectory** (`native/test_foo`, not
  `test_foo`). The bare name matches NOTHING and exits clean.
- A harness that suppresses its own failure cannot distinguish "survived" from
  "never ran". Assert the load landed on EVERY run, not once.

## T12 -- registry and spec drift is caught by tools, not eyes

**Rule:** after any `registry.yaml` change, regenerate, run `--check`, and run
the catalog lint. A catalog that "did not encode" is usually NOT a sizing
problem, it is entries out of ascending-id order.

## T20 -- a hand-copied vocabulary drifts silently, and a RETIRED one lies

**Rule:** a vocabulary with more than one consumer language gets a generator
and a staleness gate, or it gets one consumer. Never a generator on one side
and a comment saying "transcribed from" on the other. When a vocabulary is
retired, DELETE its identifiers rather than aliasing them onto the successor;
a working alias is how this survived undetected for months.
**Mechanism:** when two languages consume one registry and only one generates
its constants, the hand-written side has no failure mode that looks like
failure. A wrong wire NUMBER crashes or NACKs; a wrong wire NAME renders. The
copy compiles, the session goes live, the page draws, and the only symptom is
a label nobody cross-checks. Drift accumulates one skipped registry addition
at a time, and every skip is individually invisible.
**The census matters more than the one bug:** diffing all 24 hand tables
against the registry found SEVEN drifted -- missing frame types, three missing
NACK codes surfacing as raw numbers, missing CBOR keys, missing session-event
kinds, and one table carrying 16 of 68 entries. Not one had been noticed. A
tombstoned vocabulary succeeded on the same wire key with a different base and
14 entries instead of 5 mislabeled EVERY settings tab in EVERY client from the
moment the firmware emitted it, and nothing errored.
**Fix:** one generator emits every consumer's artifact, with a `--check` mode
covering all of them. What legitimately stays hand-written is named and
justified in the file banner, because "this one is fine to transcribe" is the
belief that produced all seven.
