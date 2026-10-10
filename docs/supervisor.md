# Board monitor (supervisor) contract

The OSSM Flagship carries a WCH CH32V003F4U6 (U12) as a board monitor: it
watches the rails and the regen clamp continuously, trims the clamp threshold,
and can cut motor power. The P4 programs it over SWIO and talks to it over I2C.
This file is the contract between the two firmwares. Status (what is built,
measured, flashed) lives on the board: `bd show val-091.19`, `val-091.20`.

- Hardware truth: `../Hardware/flagship/SPEC.md` (cited as `SPEC Lnn`, line
  numbers of the 2026-10-01 working tree) and
  `../Hardware/flagship/design-considerations.md` (`DC Lnn`). Pin and net facts
  below were read from `ossm-flagship.kicad_sch` on 2026-10-01.
- Message set, one home for both sides: `flagship_p4/src/system/Supervisor.h`.
- Monitor firmware: `flagship_ch32v003/` (its own PlatformIO project, per
  `repo-layout.md`). Decisions: `src/monitor_core.c` (hardware-free, tested by
  `test/native/test_supervisor`). Glue: `src/main.c`.

Anything marked **NOT FIRM** is a proposal; the list at the end is the set of
rulings owed.

## 1. What the monitor does and does not own

Its job (SPEC L89): prove and watch the rails, the input stage and the regen
clamp so the P4 polls less, and hold a cut on the motor switch. It **can cut
motor power, never enable it**: FAULT_N is open-drain into the motor-switch EN
wired-OR, the same node the INA ALERT, the bus OVP trip and the E-stop pull
(SPEC L74, L81). It **can only lower the clamp threshold** (SPEC L89).

It does not own, and the schematic gives it no path to:

- **Motor current and bus over-current.** The INA228/INA237 sits after the
  motor switch (SPEC L92) and trips the EN node itself (DC L374-378); the
  TPS48111 has its own over-current and short-circuit trips (SPEC L49).
- **The fan.** FAN_PWM and FAN_TACH are P4 pins (SPEC L78, L84).
- **MOTOR_V+ and MOTOR_EN.** Its former MOTOR_V+ channel reads +BUS since
  SPEC L92; switch-stuck-on detection is the P4's (val-091.21 step 5). Filed as
  Hardware `hw-c7j`.
- **Stopping a stuck-on clamp.** If Q501 fails short, the regen load sits
  across the supply and nothing on the board can open it. Filed as Hardware
  `hw-cjn` (P1, safety).

## 2. Pins (schematic, U12)

| U12 pin | Function | Net / network |
|---|---|---|
| 1 PD7 | NRST | 10k to +3V3_SYS, 100 nF; never reconfigured |
| 15 PD1 | SWIO | 10k to +3V3_SYS, TP1101; 100 R to P4 G50 (`MON.MON_SWIO`) |
| 16 PD2 | A3 VIN_RAW | 160k / 10k, 100 nF |
| 17 PD3 | A4 +BUS | 160k / 10k, 100 nF |
| 18 PD4 | A7 +12V | 39k / 10k, 100 nF |
| 19 PD5 | A5 +5V | 10k / 10k, 100 nF |
| 2 PA1 | A1 +5V_SYS | 10k / 10k, 100 nF |
| 20 PD6 | A6 +3V3_ACC | 10k / 10k, 100 nF |
| 3 PA2 | A0 SHUNT_TEMP | 1k tap of TH501 (10k B3435 to GND) under R512 10k to +3V3_SYS (SPEC L120) |
| 11 PC4 | A2 CLAMP_MON | 1k tap of the Q501 gate divider 10k / 3.3k: ~3 V on, 0 off |
| 10 PC3 | CLAMP_TRIM (TIM1_CH3) | 10k pull-down at the pin, 10k, 1 uF, 47k into U3 +IN |
| 14 PC7 | FAULT_N | 1k into `MSW.EN_NODE` |
| 7 PC0 | PUMP_FLT | U14 FLT, 10k to +3V3_SYS, active low (SPEC L93) |
| 8 PC1 / 9 PC2 | I2C1 SDA / SCL | INA bus: P4 G34 / G36, 4.7k pull-ups, U11 at 0x40 (SPEC L84) |
| 13 PC6 | MON_LED | 100 R into the 10-LED bank (SPEC L143, L149) |
| 5 PD0, 12 PC5 | spare | NC; both wanted by `hw-cjn` / `hw-c7j` |

The ADC is 10-bit and referenced to VDD (+3V3_SYS, the stamp's buck). VDD is
measured through the internal ~1.2 V reference every scan and every rail is
corrected by it; `McState.vrefint_mv` is the calibration knob. SHUNT_TEMP is
ratiometric to VDD (its pull-up is VDD), so it needs no correction.

## 3. Sampling, thresholds, timing

One scan of all eight channels plus Vrefint every 1 ms (val-091.19), polled at
241 ADC cycles per channel: the 100 nF at each pin is the charge source, and
the 160k / 10k dividers give each channel a ~1 ms RC, so the scan oversamples
the filter. Every compare is a window with a consecutive-sample debounce.

| Check | Level | Debounce | Action | Source |
|---|---|---|---|---|
| Boot | first 100 ms | n/a | FAULT_N held | NOT FIRM |
| VDD | 3.0-3.6 V | 10 ms | cut (readings untrusted) | NOT FIRM |
| +12V | 10.8-13.2 V | 10 ms | cut: it supplies the clamp comparator U3 | rail: SPEC L89; window NOT FIRM |
| +5V_SYS | 4.5-5.5 V | 10 ms | cut: quadrature buffer, drive 485_5V | NOT FIRM |
| +5V | 4.5-5.5 V | 10 ms | warn | NOT FIRM (val-091.21 gates on it at boot) |
| +3V3_ACC | 3.0-3.6 V | 10 ms | warn: accessory rail | SPEC L89 ("accessory rails only warn", val-091.19) |
| Input stage | VIN_RAW >= 20 V and +BUS < VIN_RAW - 3 V | 200 ms | cut | check: SPEC L92; 20 V: SPEC L46; timing NOT FIRM (LTC ramp ~40 ms, SPEC L54) |
| Bus over-voltage | +BUS > 47 V | 5 ms | cut | NOT FIRM; sits above the 44.5 V clamp and below the ~49 V hardware trip and the ~50 V INA alert (SPEC L61, L74) |
| Regen load warm | SHUNT_TEMP pin <= 418 mV (85 C) | 100 ms | trim to duty 0 | rule: val-091.19; temperature NOT FIRM |
| Regen load hot | pin <= 238 mV (110 C) | 100 ms | cut | NOT FIRM |
| Clamp feeding | CLAMP_MON on, +BUS <= supply + 0.5 V | 2 ms | trim to duty 0, held until re-latched | rule: val-091.19 |
| Clamp stuck | the same, with the trim already at the ceiling | 100 ms | cut and report | NOT FIRM |
| P4 heartbeat | silent past 500 ms once armed | n/a | cut | NOT FIRM; not in SPEC |
| Pump fault | PUMP_FLT low | none | warn | SPEC L93 |

Every cut LATCHES: FAULT_N stays pulled until the P4 sends CLEAR_LATCHED and
the live condition is gone. This mirrors the INA ALERT latch the P4 already
re-arms (`ValencePower.h`). The first fault since the last clear is recorded.

NTC levels: TH501 is 10k at 25 C with B = 3435 K (SPEC L98); R(85 C) = 1.45k,
R(110 C) = 0.78k, pin = 3.3 V x R / (10k + R). An open NTC reads cold and never
trips; a shorted one reads hot and cuts.

## 4. Regen clamp trim

Transfer function, from the nominal parts (R501 348k, R502 12.1k, R503 3.01M,
R1118 + R1119 = 57k, TLV431 1.24 V), threshold at duty d:

    Vth(d) = 44.62 V - 6.105 x d x VDD

SPEC L89 gives duty 0 = 44.5 V, reset (pin floating) = 43.5 V, full duty =
24.5 V; the nominal-part figure for duty 0 is 44.62 V. The firmware rounds the
duty DOWN, so the threshold lands at or above the target, never below.

- **Target**: the supply plus 3.0 V with the LTC4364 fitted, plus 1.2 V on a
  budget build (SPEC L89). On a budget build F1 sits on its third pad and
  VIN_RAW reads 0, so the supply is read from +BUS (SPEC L108).
- **Latched once** (val-091.19: "at power-up"): after boot, once the supply has
  held within 0.3 V for 200 ms and is at least 20 V. Not tracked afterward.
  Tracking would follow a supply sag downward and then sit below the
  recovered supply, which is the one way this feature can turn the supply
  into the regen load's power source.
- **Budget detection** is automatic: VIN_RAW under 2 V with +BUS at or above
  20 V. NOT FIRM (the P4 could declare it instead).
- **Limits of reach**: full duty is ~24.5 V at VDD 3.3 V and rises as VDD
  falls. A 24 V budget build asks for 25.2 V, at the edge; at VDD 3.15 V the
  trim saturates at ~25.4 V. Saturation errs high, which is safe.
- **Output states**: before the latch and while HELD the pin floats (the
  reset level, 43.5 V); on a warm regen load or after a feeding event it is
  driven at duty 0 (44.5 V, the ceiling); otherwise it is driven at the
  target duty.
- **Clamp test** (val-091.19, val-091.21 step 6): on command, threshold to
  +BUS - 2 V for 30-50 ms; PASS if CLAMP_MON is seen on in the second half of
  the pulse (the trim filter's ~8 ms time constant has settled by then). The
  feeding and stuck checks are suspended for the pulse and 50 ms after.
  Refused (FAIL) unless the trim is latched and nothing is cutting. NOT FIRM:
  energy. At 36 V the 7.5 R pair takes ~173 W, ~4.3 J per resistor over
  50 ms, while the pulse figure on record is ~1 J at 2.5 ms (SPEC L117).

## 5. The link (I2C)

The monitor is an I2C target on the INA's private bus (SDA G34, SCL G36,
400 kHz; SPEC L84, L89), at **0x2C** (NOT FIRM; anything outside 0x40-0x4F).
The P4 joins the bus handle `ValencePower` already owns (`ValencePower.h`), so
both devices' transactions serialize in the IDF driver. They block for up to a
few ms: never from the motion task or a timer callback.

Every block, both directions, is `[id][len][payload][crc8]`: `len` counts the
whole block, CRC-8 (polynomial 0x07, init 0) covers every byte before it. A
read is a one-byte write of the id, a repeated start, and a read of `len`
bytes. A wrong id, length or CRC drops the block whole; the monitor counts
drops and raises `SV_W_LINK_DROP`.

| Id | Block | Dir | Len | Content |
|---|---|---|---|---|
| 0x00 | IDENT | read | 14 | magic "NM", link version, fw major/minor/patch, image CRC-32, reset cause (RCC_RSTSCKR 31:24) |
| 0x10 | STATUS | read | 37 | seq, flags, faults live/latched, warnings, first fault, clamp test result, 8 readings (mV), VDD, trim target, trim duty, clamp-on fraction |
| 0x20 | LIMITS | read/write | 27 | four rail windows, bus OV, NTC warm/trip, heartbeat timeout |
| 0x30 | HEARTBEAT | write | 4 | counter |
| 0x31 | COMMAND | write | 6 | op, 16-bit arg |

- **STATUS is a snapshot.** The monitor publishes it double-buffered every
  scan and the I2C interrupt copies the whole block at the address match, so
  one read never mixes two scans. `seq` advances every scan: a frozen `seq` is
  a hung monitor.
- **LIMITS can tighten, never loosen.** A written block is merged field by
  field against the compiled defaults, keeping the tighter side
  (`sv_limits_tighten`). The P4 cannot disable a check.
- **HEARTBEAT** arms the watchdog on its first arrival; after that a gap past
  the timeout cuts. Only a CHANGED counter feeds it, so a stuck writer
  repeating one block does not keep the motor enabled. NOT FIRM: whether the
  monitor watches the P4 at all, the 500 ms, and arming-by-first-heartbeat. A
  dedicated heartbeat line is not an option: the P4 has no free pin (SPEC L151).
- **COMMAND**: `CLEAR_LATCHED`, `CLAMP_TEST` (ms), `TRIM_RELEASE` (float the
  trim pin and hold it), `TRIM_RELATCH`.

The P4 side, for val-091.21: read IDENT at boot (section 6); heartbeat and
STATUS poll from the motor-switch task, the power bus's one owner after the
self-check's verdict (val-9hr; it reads STATUS once a second today, for
+BUS on 0x1010); on an EN-node drop (G23) read STATUS, publish
the latched faults over Valence, clear once the cause is gone.

**Shared-bus risk.** SPEC L87 put Qwiic on its own bus so that a hung
accessory cannot stall sensorless homing. The monitor is a new target on the
homing bus: a monitor stuck holding SDA low would stall the INA. Mitigations
on the P4 side: a bus clear (nine SCL pulses) and, failing that, a monitor
reset through the SWIO debug module. There is no free P4 pin for a separate
link. Listed as a ruling owed.

## 6. Programming from the P4 over SWIO (val-091.20)

**Wiring** (SPEC L89, schematic): P4 G50 -> R1102 100 R -> PD1 (SWIO), R1101
10k pull-up to +3V3_SYS. Rescue pads TP1101 SWIO, TP1102 3V3, TP1103 GND for
a WCH-LinkE. The pins are firm. The bit timing is not: nothing in SPEC sets
it, so the bit-level driver waits for a scope session (below). No SWIO code is
written yet.

**Protocol.** WCH's single-wire debug interface (QingKe V2 debug manual);
the reference implementation is ch32v003fun's minichlink (its ESP32-S2 and
RP2040-PIO programmers are prior art on comparable hosts). The line idles
high; the host starts every bit by pulling low, a short low is a 1 and a long
low a 0, and on a read the target stretches the low to answer 0. A
transaction is a start, a 7-bit debug-module register address, a read/write
bit, and 32 data bits. The registers are the RISC-V Debug Module's
(DMCONTROL 0x10, DMSTATUS 0x11, ABSTRACTCS 0x16, COMMAND 0x17, DATA0 0x04,
PROGBUF 0x20-0x27) plus WCH's CPBR 0x7C, CFGR 0x7D, SHDWCFGR 0x7E; minichlink
enables target output by writing 0x5AA50400 to SHDWCFGR then CFGR. NOT FIRM
until scoped: the exact short/long low times and sample point (the 10k
pull-up with ~15 pF sets a ~150 ns rise on every released edge).

**Execution on the P4.** A bit-bang with interrupts masked per transaction
(tens of microseconds) on the hub task's core, or the RMT. Masking interrupts
stops only that core's interrupts and task switches; the other HP core keeps
running, and the LP-core quadrature emitter is a separate processor that never
sees it. Never on the motion task.

**Sequence**, motor switch off throughout:

1. MOTOR_EN held low. If the monitor answers I2C, send `TRIM_RELEASE`: the
   trim pin floats at its reset level, so a halt, an erase and a reset all
   leave the clamp at 43.5 V.
2. Attach: output enable (SHDWCFGR, CFGR), DMCONTROL `dmactive | haltreq`,
   confirm `allhalted` in DMSTATUS.
3. Through the debug module, drive the flash controller: unlock, erase the
   16 KB code flash, program in 64-byte pages.
4. Verify: read the flash back over SWIO and compare.
5. Reset through DMCONTROL `ndmreset`, release, wait for boot.
6. Read IDENT over I2C; the image CRC must match. Retry once, then report
   "monitor missing" and keep the motor off (SPEC L89, val-091.21 step 1).

The monitor firmware must never reconfigure PD1 or PD7 (`main.c` header):
SWIO is the only programming path the product has. NOT FIRM: whether the IWDG
keeps counting while the core is halted; if it does, the programmer sets the
debug freeze before a long operation.

**The image in the OTA.** `pio run -d flagship_ch32v003` produces the
monitor's `firmware.bin`; the P4 build embeds it as a binary blob, so every
Nucleus OTA carries exactly one monitor image. The monitor image is never
committed (it is a build output); the P4 build depends on the monitor build.
NOT FIRM: the mechanism (a pre-build step in `flagship_p4/platformio.ini` or a
CMake step).

**Version check at boot.** The monitor computes a CRC-32 over its whole 16 KB
code flash at boot and reports it in IDENT. The P4 computes the same over the
embedded image padded with 0xFF to 16 KB. Reprogram when there is no answer, a
bad block, a different link version, or ANY CRC difference
(`sv_needs_flash`). Deliberately "different", not val-091.20's "older than":
after an OTA rollback the older P4 image must take the monitor back with it,
because the link is versioned with the P4.

## 7. Safe states

| Moment | FAULT_N | CLAMP_TRIM | Clamp threshold |
|---|---|---|---|
| Monitor in reset, blank, or halted | floating (released) | floating | 43.5 V |
| First instruction after SystemInit | pulled | floating | 43.5 V |
| Boot window, before the trim latch | pulled until 100 ms, then by state | floating | 43.5 V |
| Running | by state | target duty | supply + margin |
| Warm load, feeding seen | by state | duty 0 | 44.5 V |
| Monitor hung | held as it was | held as it was | IWDG resets within 100 ms -> first row |

A blank or dead monitor cannot cut, so the P4 refuses motor enable without a
verified monitor (SPEC L89, val-091.21 step 1). The motor switch EN pull-down
covers the P4's own boot and reset (DC L144-146).

## 8. Not firm: rulings owed

1. Rail windows and which rails cut: proposed +/-10 %; +12V and +5V_SYS cut,
   +5V and +3V3_ACC warn. val-091.21 gates boot on +5V.
2. Bus over-voltage as a third layer at 47 V / 5 ms, or none.
3. Regen load temperatures: 85 C releases the trim, 110 C cuts, or no cut.
4. P4 heartbeat: watch it at all (not in SPEC), 500 ms, armed by the first
   heartbeat, latched.
5. I2C address 0x2C.
6. Boot: FAULT_N held for the first 100 ms of every monitor boot, so a monitor
   reset mid-session drops the motor.
7. Budget build detected by the monitor, or declared by the P4.
8. Clamp test energy at 50 ms (~4.3 J per resistor at 36 V) against the
   PWR263S pulse curve at that width.
9. Version rule: exact image match replaces val-091.20's "older than".
10. The shared INA bus (section 5): accept with the bus-clear and SWIO-reset
    mitigations, or a hardware change.
11. Clamp ceiling: SPEC says 44.5 V at duty 0; the nominal parts give 44.62 V
    once the trim network's 57k is counted. The firmware uses 44.62 V, which
    costs at most 0.12 V of the 3 V margin if the real value is lower.
12. MON_LED bank colors: see the flag below. The bank is held dark until then.
13. Hardware: `hw-cjn` (P1, a stuck-on clamp has no actuator) and `hw-c7j`
    (the monitor no longer sees the motor switch).

```
CANON FLAG -- the monitor's LED bank vs the fleet-wide LED grammar
Source A: .claude/rules/logging-leds.md "LEDs go through Flux. Only." and
          "The grammar is FLEET-WIDE: the same color is the same system on
          every board."
Source B: Hardware flagship/SPEC.md L143 "one LED per U12 check (operator:
          high / low / good / off per state)", map at L149
My read: the bank is a per-check meter, not one system speaking, and the
          CH32V003 (16 KB flash, 2 KB RAM, C) cannot host Flux. Both rulings
          can stand if the bank is scoped out of the grammar, but colors for
          high / low / good would still need choosing.
Your call: is the monitor bank exempt from the Flux grammar, and if so, which
          colors mean high, low and good?
```
