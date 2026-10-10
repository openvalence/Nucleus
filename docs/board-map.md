# OSSM Flagship board map: every P4 connection, by function

The inventory of what the Flagship wires to the ESP32-P4 Stamp and the C6
add-on, against what Nucleus does with each connection. Navigation, not a
home: pin numbers live in `flagship_p4/src/system/BoardPins.h` (C-1), nets and
parts in the Hardware repo's `flagship/ossm-flagship.kicad_sch`, rulings and
firmness in `../Hardware/flagship/SPEC.md`, open work on the board (`bd`). A
row here that disagrees with any of those is a Canon flag, never a silent fix.

[verified 2026-10-01 -- U8 pad-to-net extracted from the Hardware working
copy's `ossm-flagship.kicad_pcb` (saved 17:58, schematic 17:57, labels
cross-checked), SPEC.md read in full, firmware read at commit f3b6148]

**Firm** means a SPEC ruling fixes the pad (or the stamp does). **Provisional**
means the pad comes only from SPEC 2026-09-23 "Provisional P4 pin map (swap
freely at layout)" and was never re-ruled; Hardware bead hw-kzr asks for the
ratification, and BoardPins.h marks each one `TODO(hw-kzr)`.

## Headline

- **Firm and wired:** quadrature A/B (LP core), the private I2C bus (motor
  current monitor, and the board monitor's IDENT/STATUS at boot), the C6 SDIO
  link, the USB console, the E-stop pair (the BoardIo task's reader, latched
  by the hub delegate) and its bypass (held low), status LED data, fan PWM
  and tach, the PAIR button (the BoardIo task, `system/ValenceBoardIo.cpp`).
  The DBG marker has its surface (`system/ValenceDbg.h`) and is driven from
  nowhere by default.
- **Provisional and wired:** the motor switch: MOTOR_EN and PRECHARGE_EN
  (held low from the first line of `app_main`, raised only by the enable
  sequence), MSW_FLT_N, MSW_IMON and EN_NODE (the switch task's watch); THERM
  (sampled on that same task); the HOME button (bound by the hub delegate:
  press homes, hold reboots, val-091.26).
- **Firm, wired, not on the wire:** the accessory headers (pump and EXT PWM,
  AUX1/AUX2, IO1-IO5; the Qwiic bus and the header UART open on first use),
  `system/ValenceAccessoryIo.cpp` on the BoardIo task; catalog val-091.71.
  The PD daughterboard: PD_INT polled and the TPS26750 at 0x21 read,
  `system/ValencePdSource.cpp` on the same task; its contract gates motor
  power and reaches a client only as the self-check row and log lines
  (catalog val-091.75).
- **Firm and unwired:** board monitor SWIO.
- **Provisional, wired, drive status not on the wire:** RS485 TX/RX/DE and
  DRV_ALM/DRV_RDY, `system/ValenceDriveLink.cpp` on its own task. A DRV_ALM
  alarm latches ESTOP, cause fault; the link's probe and poll reach a client
  only as the self-check's drive-link row and log lines (catalog val-091.73).
- **Provisional and unwired:** CLAMP_MON, SHUNT_TEMP.
- **ADC1 has ONE reader**, the motor switch task: every ADC1 pad (MSW_IMON,
  EN_NODE, THERM, and CLAMP_MON / SHUNT_TEMP when they land) joins its poll.
  A second reader's collision reads the EN node as NaN, which cuts motor
  power (`ValenceMotorSwitch.cpp` header).
- Every unwired row has a bead under val-091 (column "Owed").

## Motion and motor power

| Function | Net | P4 pad | Firmware today | Firmness | Owed |
|---|---|---|---|---|---|
| Quadrature A to the drive, through the always-on 74AHCT125 gate 1 (U10) to J4 MOTION, tapped on TP601 | `/DRV.QUAD_A` | GPIO3 (LPG3), stamp pad 45 | LP core emitter: `ulp/lp_quad.c:47` (PIN_A), edge table at `:99`, inside the fence and lease the arbiter writes; steered by `motion/ValenceMotion.cpp:81` | Firm: SPEC 2026-09-23 "P4 pin map after the LP-quadrature-only ruling" | scope rerun val-091.18; reset pull-down hw-7mo |
| Quadrature B, gate 2 (U10), TP604 | `/DRV.QUAD_B` | GPIO5 (LPG5), pad 35 | same, `ulp/lp_quad.c:48` | Firm: same row | same |
| Motor switch main-FET enable: INP of the motor switch controller (U2, TPS48111), 100k pull-down R409 | `/MSW.MOTOR_EN` | GPIO21, pad 22 | Driven LOW from `app_main`'s first line (`system/ValenceSelfCheck.cpp`, `selfCheckHoldMotorOff`); HIGH only in `on`, after a full pre-charge window (`system/MotorSwitch.h`), written by `system/ValenceMotorSwitch.cpp` `driveLocked` | Provisional (hw-kzr) | bench val-091.56 |
| Pre-charge enable: INP_G of U2, through R412 100 R, 100k pull-down R410 | `/MSW.PRECHARGE_EN` | GPIO22, pad 21 | Driven LOW, same sites; HIGH for the 150 ms window only (`MotorSwitch.h` `kPrechargeUs`, its derivation on the constant) | Provisional | bench val-091.56 |
| Motor switch fault: U2 FLT_T and FLT_I tied, 10k pull-up R411, active low | `/MSW.MSW_FLT_N` | GPIO20, pad 23 | Read at boot by the self-check through `motorSwitchFaultLine()`, then every 5 ms by the switch task: low latches the switch faulted and the hub ESTOP, cause fault (`ValenceDevice.cpp` `tick`) | Provisional | split hw-3qf; bench val-091.56 |
| Home sense: the stall level that ends each homing approach, at either end of the rail, active HIGH, push-pull from its source | (bench wire) | GPIO23 under `CONFIG_NUCLEUS_BENCH_QUAD_LP10_12` only; none on the flagship (G23 is EN_NODE there) | `system/ValenceHomeSense.cpp`: input, pull-down, the level read, its rising-edge interrupt parking an armed seek (`MotionArbiter::homeSenseRose()`) and waking the planner task, which reads the rise again 100 us on (`kHomeSenseDebounceUs`) to confirm it; a 50 us pull-up probe before each cycle refuses an undriven line; home op 1 runs `motion/MotionArbiter.cpp` homing, two constant-velocity legs off the planner (input contract and the 5 mm safety margins on its constants in `MotionArbiter.h`). Release map -1: home op 1 refuses UNSUPPORTED_OP | Bench only | flagship pin hw-6r6 |
| Motor switch current monitor: U2 IMON (8.25k), ADC1 | `/MSW.MSW_IMON` | GPIO19, pad 24 | ADC1 oneshot, calibrated, 4-sample mean, 0.15 V/A; judged at the end of the pre-charge window against `kInrushCeilingA` (`MotorSwitch.h`) | Provisional (ADC1-bound) | bench val-091.56 |
| EN/UVLO wired-OR node of U2: pulled low by the INA ALERT, the bus overvoltage trip (Q405), the E-stop (Q901) and the board monitor's FAULT_N; ADC1, ~1.8 V running, ~0 V tripped | `/MSW.EN_NODE` | GPIO23, pad 20 | Absent on the bench Kconfig (G23 is the home sense there; reads NaN, so motor power never enables). ADC1 oneshot, every 5 ms: under `kEnNodeMinV` while pre-charging or on latches faulted (`en_node`); read fresh after the INA ALERT re-arm on every enable | Provisional (ADC1-bound) | cause naming val-091.57; bench val-091.56 |

**The bench profile** (operator ruling 2026-10-02, val-091.58). A bare
devkit has no motor switch, so the self-check never passes and the arbiter's
motor-power gate would refuse every intent. The build profile
`NUCLEUS_BENCH_NO_MOTOR` (env `flagship_p4_bench`: `pio run -d flagship_p4 -e
flagship_p4_bench`) makes that one gate advisory: motion is admitted with the
switch reporting off, the first admission logs `BENCH: motor power gate
bypassed` once per boot, and every other gate (e-stop, PAUSE, homed,
commissioned, the window) is unchanged. Nothing else moves: MOTOR_EN still
follows the self-check, hub-status and the census still report the switch as
it is, WELCOME still declares `estop_cuts_power`, and FIRMWARE_VERSION carries
`-bench`, so the image cannot pass for a release. It is a compile-time
profile, never a runtime switch; the release env never sets it. Never flash it
at a Flagship that carries a motor: with the gate advisory, a plan can render
into an unpowered drive and move position truth without the carriage.

## Private I2C bus and the board monitor

| Function | Net | P4 pad | Firmware today | Firmness | Owed |
|---|---|---|---|---|---|
| SDA: motor current monitor (U11, INA237 or INA228, 0x40, A0/A1 to GND) and board monitor (U12, CH32V003, 0x2C, not ruled); 4.7k pull-up R24 | `/I2C.INA_SDA` | GPIO34, pad 33 | Bus opened by `system/ValencePower.cpp` from `BOARD_GPIO_INA_SDA`, which also holds U12's device handle (`boardMonitorRead()`); app_main reads it during boot, the motor-switch task owns it from the self-check's verdict on (`powerClaimBus()`) | Firm: SPEC 2026-09-23 pin-map-after-LP row | test pads hw-kcu |
| SCL, 4.7k pull-up R25 | `/I2C.INA_SCL` | GPIO36, pad 47 | same, `BOARD_GPIO_INA_SCL` | Firm | same |
| U11 identification (INA237 or INA228, one footprint): the INA237 documents no DEVICE_ID, so MANUFACTURER_ID must read TI and DEVICE_ID (0x3F) is cross-checked against a probe: CONFIG bit 5 (TEMPCOMP) holds a written 1 on the INA228 and reads 0 on the INA237; an unknown id decides by the probe alone; a contradiction, a non-TI id or an I2C error leaves `powerChip()` none: current sensing off, the boot self-check's power-monitor entry fails, motor power is held off | (I2C) | | `PowerMonitor.h` `ina2xx::identifyPart`, decision `ina2xx::identify` (suite `test_power_monitor`, both parts as fake register files) | Firm | bench val-091.22 (SOVL 15 A ruled 2026-10-09) |
| U11 ALERT, latched: joins the EN node above, cuts motor power with no firmware | `/MSW.EN_NODE` | (GPIO23) | DIAG_ALRT read (`powerTakeAlerts()`) by the switch task on every enable request, before the window, its over-current and over-voltage flags logged: that read plus the MOTOR_EN toggle is the re-arm (`ValenceMotorSwitch.cpp` `serviceEnable`) | Firm | bench val-091.56 |
| SWIO to the board monitor's PD1 through R1102 100 R (10k pull-up R1101; rescue pad TP1101) | `/MON.MON_SWIO` | GPIO50, pad 10 | nothing | Firm: SPEC 2026-09-23 board-monitor row | val-091.20 |
| Board monitor FAULT_N (U12 PC7 through R1121 1k): can cut motor power, never enable it | onto `/MSW.EN_NODE` | (GPIO23) | seen only as an EN-node drop (row above), cause not read | Firm | val-091.19, val-091.57 |
| Board monitor's own ADC taps (VIN_RAW, +BUS, +12V, +5V, +5V_SYS, +3V3_ACC, SHUNT_TEMP, CLAMP_MON), CLAMP_TRIM PWM, PUMP_FLT (PC0), status LED bank (PC6) | U12 local | not on the P4 | IDENT and STATUS blocks (`system/Supervisor.h`) read once at boot by the self-check, `ValenceSelfCheck.cpp:58`; no runtime poll, no heartbeat | Firm (address not ruled) | val-091.19, val-091.57 |

## Regen clamp, thermal, fan

| Function | Net | P4 pad | Firmware today | Firmness | Owed |
|---|---|---|---|---|---|
| Regen clamp gate monitor: 10k + 3.3k divider off the clamp FET gate (Q501), C504 1 nF; on-time capture for dissipation | `/REGEN.CLAMP_MON` | GPIO17, pad 26 | nothing | Provisional (ADC1-bound) | val-091.25 |
| Regen load temperature: TH501 10k NTC, 10k pull-up R512, C503 100 nF, ADC1 | `/REGEN.SHUNT_TEMP` | GPIO18, pad 25 | nothing | Provisional (ADC1-bound) | val-091.25 |
| Fan drive: Q701 to the high-side P-FET Q702, HIGH = on, R701 100k pull-down; output LC-filtered to DC | `/FAN.FAN_PWM` | GPIO27 (USB1.1 D+), pad 17 | LEDC low-speed timer 0 / channel 0, 50 kHz, 10-bit (`system/ValenceFan.cpp:72`, `kPwmHz`); duty from `Thermal.h`'s `FanPolicy`: curve with hysteresis, full-duty kick on every start, integral loop on tach RPM, stall retry; no tach pulse by the end of the first kick is no fan fitted, duty 0 for the boot (val-30d) | Firm: SPEC 2026-09-23 pin-map-after-LP row | bench val-091.66; wire channel val-091.65 |
| Fan tach, open collector, 10k pull-up, 1k series | `/FAN.FAN_TACH` | GPIO38, pad 32 | PCNT unit, falling edges, 1 us glitch filter, read and cleared once a second, 2 pulses per revolution (`ValenceFan.cpp:93`) | Firm: same row | bench val-091.66 |
| Thermistor header J7: 1k series R27 + C14 100 nF, ADC1 | `/FAN.THERM` | GPIO16, pad 27 | ADC1 oneshot, 4-sample mean once a second on the motor switch task (`ValenceMotorSwitch.cpp:281`), handed over by `motorSwitchThermVolts()`; 10k B3950 table with a calibration offset in `system/Thermal.h`; open (the pull-up reads full scale), shorted, or colder than the `kSaneMinC` sanity window reads NaN and a fitted fan runs at `kNoSensorFraction`; a floating pin on a bare devkit reads a plausible temperature and is not detectable from THERM alone (the no-fan verdict keeps the fan off there) | Provisional (ADC1-bound) | bench val-091.66; wire channel val-091.65 |

## Drive comms and status

| Function | Net | P4 pad | Firmware today | Firmness | Owed |
|---|---|---|---|---|---|
| RS485 transmit data into the transceiver (U9, THVD1450) D | `/DRV.RS485_TX` | GPIO33, pad 50 | HP UART1 (`BOARD_UART_RS485`) in RS485 half-duplex mode, 8N1 (`system/ValenceDriveLink.cpp`); a Modbus RTU master on the DriveLnk task (`system/ModbusRtu.h`: FC 0x03/0x06/0x10, CRC, three attempts, a reply deadline, the 3.5-character gap) that only ever reads (`system/AimDrive.h`) | Provisional | bench val-091.69; test pads hw-kcu |
| RS485 receive from U9 R | `/DRV.RS485_RX` | GPIO32, pad 46 | UART1 RX with the internal pull-up: U9's R is high-impedance while DE (tied to /RE) is high | Provisional | bench val-091.69 |
| RS485 direction: U9 DE and /RE tied, HIGH transmits | `/DRV.RS485_DE` | GPIO41, pad 49 | Driven LOW from `app_main`'s second line (`driveLinkHoldIdle()`; hw-3jk: the tied pins float near 1.65 V until driven). Then UART1's RTS, routed only after RS485 half-duplex mode has set it low: high for our frames, low again at TX_DONE. Built without `CONFIG_NUCLEUS_DRIVE_LINK` it stays a GPIO held low | Provisional | pull-down hw-3jk; bench val-091.69 |
| Drive alarm (WR, opto NPN to COM), 10k pull-up + 1k R20 | `/DRV.DRV_ALM` | GPIO31, pad 51 | Input, internal pull-up beside the board's; sampled every 5 ms on the DriveLnk task, 20 ms debounce (`aim::StatusLines`). Acted on only once the motor switch has been `on` for `aim::kSettleMs` (1 s): an assertion then, or one already there, hands the hub delegate one alarm, which latches ESTOP, cause fault (`ValenceDevice.cpp` `tick`). Read normally open (0x07 even); the probe fails a drive set otherwise | Provisional | bench val-091.69; channel val-091.73 |
| Drive ready / following error (RDY), 10k pull-up + 1k R21 | `/DRV.DRV_RDY` | GPIO28, pad 52 | Input, internal pull-up; debounced like DRV_ALM and reported (`driveLinkStatus().ready`, the liveness line), never acted on: it opens whenever the following error passes 0.5 degrees (manual p. 5), which motion can do | Provisional | channel val-091.73; bench val-091.69 |
| Encoder index ZO | not brought out | none | n/a: absolute position comes over Modbus (SPEC 2026-09-23 drive-connector row), the 0x16/0x17 pair polled while the link is up | Firm | audit val-091.74 |

The drive link probes once motor power has held `aim::kSettleMs`: the
identity register (0x15 reads address 1) across 19200, 115200, 38400 and
9600 baud, then what the LP core's quadrature needs (0x00 = 0, 0x01 = 1,
0x19 = 2, 0x07 even), the stall-alarm digit (0x18; the factory 600 has it
off), the alarm code (0x0E), the temperature (0x12) and the encoder pair. The
boot self-check's drive-link row is the one POST-ENABLE row: pending at boot,
recorded when a probe's verdict changes, and never a gate on motor power
(`SelfCheck.h` `postEnable()`). After a probe that read the whole map the
alarm code and the encoder are polled, one read every 50 ms, and a DRV_ALM
edge reads the alarm code next [host-verified 2026-10-02 -- `pio test -e
native` suites test_drive_link 35/291, test_self_check 22/147,
test_valence_device 25/901, each binary run directly, exit 0; `pio run -d
flagship_p4` SUCCESS; bench owed, val-091.69].

## E-stop, buttons, LED, debug

| Function | Net | P4 pad | Firmware today | Firmness | Owed |
|---|---|---|---|---|---|
| E-stop NC contact (J9, and the daughterboard M8 in parallel), 2.2k pull-up, 1k R903; HIGH = open | `/ESTOP.ESTOP_NC` | GPIO39, pad 31 | Polled every 10 ms on the BoardIo task (`system/ValenceEstopInput.cpp`, the pads' one configurer and reader) into `EstopInput.h`: (NC, NO) decoded released / pressed / unplugged / wiring fault, 30 ms debounce, contacts that never settle in 200 ms read wiring fault; published to the hub task as one atomic byte. `ValenceDevice::tick()` latches ESTOP while it reads anything but released (pressed: cause user; unplugged or miswired: cause fault; an EN-node switch fault waits 100 ms for it to name the stop) and `canClearEstop()` refuses the release until it reads released ("e-stop pressed at the machine"); releasing the button clears nothing. The boot self-check's estop row reads the same reading. Bench profile (`NUCLEUS_BENCH_NO_MOTOR`): unplugged reads released, flagged masked and logged, the row SKIPPED; the release build latches it and fails the row ("no E-stop found") | Firm: SPEC 2026-09-23 E-stop row (+2026-09-28, 2026-10-01, 2026-10-02) | bench val-091.69 |
| E-stop NO contact, 2.2k pull-up, 1k R904 | `/ESTOP.ESTOP_NO` | GPIO30, pad 53 | same | Firm | bench val-091.69 |
| E-stop bypass drive (Q903, masks any state where NO reads open: unplugged, a broken NO wire, a 2-wire NC button), 100k pull-down R906 | `/ESTOP.ESTOP_BYP` | GPIO29, pad 54 | Driven LOW for the boot: `ValenceSelfCheck.cpp` `selfCheckHoldMotorOff` | Firm | bypass toggle val-091.70 |
| HOME button SW1 (press homes, hold resets, on release), 10k pull-up, 1k series; J13 in parallel | `/UI.BTN_HOME` | GPIO49, pad 12 | Polled every 10 ms on the BoardIo task (`system/ValenceButtons.cpp`) into `ButtonGesture.h` (30 ms debounce, hold at 3 s, stuck at 30 s, acted on release), logged and parked for the hub (`homeButtonTake()`). Bound in `ValenceDevice::tick()` (operator ruling 2026-10-02): press = home op 1 through `applyHome()` (refused UNSUPPORTED_OP and logged until homing exists; force_home never on the button, RFC-025); hold = graceful reboot (brake, ESTOP, NVS flush, GOODBYE REBOOTING, restart, `ValenceHub.cpp` `rebootNow`); a hold the hub task has not taken in 2 s is a dead hub, and the BoardIo task cuts motor power and restarts itself | Provisional | bench val-091.66 |
| PAIR button SW2 (held at power-on = config mode), 10k pull-up, 1k series; J13 in parallel | `/UI.BTN_PAIR` | GPIO52, pad 55 | Sampled with HOME, same core (`pairButtonTake()`). Press opens the presence window (SPEC 12.3, operator ruling 2026-10-02: tap pairs, superseding the SPEC row's "hold pairs"); hold is reserved and only logged; held through power-on fires nothing (config mode is val-9u0.14's) | Firm: SPEC 2026-09-23 pin-map-after-LP row | config mode val-9u0.14; bench val-091.66 |
| Status LED data: 74AHCT125 gate 3 (U10) to the GRBW pixel D5 (XL-3528RGBW, 32 bits per pixel), chained on to J12 NEOPIXEL OUT; R601 10k pull-down | `/DRV.LED_DATA` | GPIO26 (USB1.1 D-), pad 19 | RMT TX at 10 MHz, one GRBW frame per 20 ms (`system/ValenceGlow.cpp:67`), the board's one Flux glue; `StatusLook.h` maps the machine state to a Flux pair; pixel 0 only, the J12 chain is never written | Firm: same row | bench val-091.66 |
| DBG marker through R602 1k to TP603; the ROM boot log appears on it at every reset (U0TX) | `/DRV.DBG` | GPIO37, pad 30 | `system/ValenceDbg.h`: `dbgBegin()` claims the pad, `dbgLevel()` is one register store, `dbgPulse(n)`; called from nowhere by default | Firm: SPEC 2026-09-23 board-monitor row | bench val-091.66 |
| BOOT strap to tweezer pad TP903 (held low at power-on = download mode) | `Net-(TP903-Pad1)` | GPIO35, pad 36 (pad 48 NC) | never used, by rule | Firm | none |
| CHIP_EN to tweezer pad TP901 | `Net-(U8-CHIP_EN)` | CHIP_EN, pad 34 | n/a | Firm | none |

## Accessory headers and the PD daughterboard

All on spare LP pads, driven by HP peripherals through the GPIO matrix, never
the LP UART or LP I2C (SPEC 2026-09-23 LP rule restated; val-091.18 measures
the cost to quadrature), and never set up with `gpio_config()` or
`gpio_reset_pin()` (BoardPins.h, val-091.72). All firm: SPEC 2026-09-23
accessory-breakouts row, 2026-09-27 J15 row, 2026-09-30 PD row.

The headers' host is `system/ValenceAccessoryIo.cpp` on the BoardIo task, its
rules the hardware-free `system/AccessoryIo.h`. Every output is off at boot
and after every ESTOP (`motionEstop()` calls `accessoryIoEstop()`; the pads
follow within one 10 ms pass). Nothing here is on a catalog channel yet
[host-verified 2026-10-02 -- `pio test -e native` suite test_accessory_io
exit 0, `pio run -d flagship_p4` SUCCESS; bench owed, val-091.69].

| Function | Net | P4 pad | Firmware today | Owed |
|---|---|---|---|---|
| Pump PWM, 220 R, 100k pull-down; J10 (12 V through the pump eFuse U14, whose FLT goes to the board monitor) | `/ACC.PUMP_PWM` | GPIO13 (LPG13), pad 29 | LEDC timer 1 / channel 1, 1 kHz, 10-bit (`kPwmHz`, its reasoning on the constant), duty 0 from boot. `accessoryPwmSet(Pwm::pump, duty)`, any task, clamped to 0..1, applied on the next BoardIo pass; an ESTOP zeroes it and only a new request turns it back on. No P4 pin enables U14: its EN/UVLO is a divider off +12V (on above ~9 V). PUMP_FLT reaches the P4 only as the board monitor's `SV_W_PUMP_FLT`, which nothing reads at runtime | catalog val-091.71; PUMP_FLT val-091.19; bench val-091.69 |
| External PWM, 220 R, 100k pull-down; J11 | `/ACC.EXT_PWM` | GPIO2 (LPG2), pad 37 | LEDC timer 1 / channel 2: the pump's frequency, the same request and ESTOP rules (`Pwm::ext`) | val-091.71; bench val-091.69 |
| AUX1 button input, 10k pull-up, 1k series; J13 | `/ACC.AUX1` | GPIO4 (LPG4), pad 16 | Input with the internal pull-up beside the board's; polled every 10 ms into `ButtonGesture.h` (the HOME timings); press and hold logged (Warn, tag `accessory`) and parked (`accessoryAuxTake()`), stuck logged only. No bindings | val-091.71; bench val-091.69 |
| AUX2 button input, same; J13 | `/ACC.AUX2` | GPIO6 (LPG6), pad 18 | same | same |
| Qwiic SDA, 2.2k pull-up; J14, and PD_SDA through R1024 33 R to J16 | `/ACC.QWIIC_SDA` | GPIO11 (LPG11), pad 4 | One HP I2C master, opened by the first `qwiicI2cBus()` call (any task) on a free HP port; joiners add their own devices, never a second master. The PD controller joins at 0x21 (`ValencePdSource.cpp`, below) | bench val-091.69 |
| Qwiic SCL, 2.2k pull-up; J14, and PD_SCL through R1025 33 R | `/ACC.QWIIC_SCL` | GPIO9 (LPG9), pad 6 | same | same |
| Header UART TX, 220 R; J15 | `/ACC.GPIO_TX` | GPIO14 (LPG14), pad 2 | Untouched until a consumer calls `accessoryUartOpen(baud)`: HP UART2 (`BOARD_UART_ACC`; the drive link holds UART1), 8N1, 512 B receive ring, the caller's task owns it from then on; one consumer per boot. No consumer yet | bench val-091.69 |
| Header UART RX, 220 R; J15 | `/ACC.GPIO_RX` | GPIO15 (LPG15), pad 1 | same; internal pull-up while open | same |
| Header IO1..IO5, 220 R each; J15 | `/ACC.GPIO_IO1..5` | GPIO12/10/8/7/1, pads 3/5/7/8/9 | Inputs with pull-downs from boot, which is an output's released state. `accessoryGpioConfigure()` sets input, input with pull-up or output once per boot; an output starts released and `accessoryGpioSet()` drives it; an ESTOP releases every output (never a driven low, which an active-low load reads as on); `accessoryGpioGet()` reads the pad | val-091.71; bench val-091.69 |

The PD daughterboard's host is `system/ValencePdSource.cpp` on the BoardIo
task; the register decode and the motor power profile are the hardware-free
`system/PdSource.h` (register facts from TI SLVUCR7, the TPS26750 Technical
Reference Manual, cross-checked against Linux `drivers/usb/typec/tipd`). The
main board never negotiates: it reads the contract the daughterboard's EEPROM
configuration negotiated and judges it against the input ceilings; the
contract-watts against ceilings table and the peak model's three unmeasured
knobs sit beside the model in `PdSource.h` [host-verified 2026-10-03 -- `pio
test -e native` suites test_pd_source 20/103 and test_self_check, each binary
run directly, exit 0; `pio run -d flagship_p4` SUCCESS; bench owed,
val-091.69].

| Function | Net | P4 pad | Firmware today | Owed |
|---|---|---|---|---|
| PD daughterboard interrupt: the TPS26750's I2Ct_IRQ (open drain, active low, R5 100k to its LDO) through R1023 220 R to J16 pad 4 (J15.12 is NC) | `/ACC.GPIO_IO6` | GPIO0 (LPG0), pad 11 | Input with the internal pull-up (`gpio_set_direction` / `gpio_set_pull_mode`, never `gpio_config`, val-091.72), polled every 10 ms on the BoardIo task: low is an armed INT_EVENT1 event (hard reset, plug, new contract, power status, cannot provide), read, cleared through INT_CLEAR1 and the contract re-read and logged (Warn, tag `pd`) on every renegotiation; a held line is serviced at most every 100 ms. If the INT_MASK1 write does not take, the contract is polled once a second instead | catalog val-091.75; bench val-091.69 |
| PD controller (TPS26750) on the Qwiic bus, I2C 0x21 (ADCIN1 = ADCIN2 = GND) | via J16 | (GPIO11 / GPIO9) | Joins `qwiicI2cBus()` at boot at 100 kHz. A NACK is a DC-input build. MODE (0x03) 'APP ' is ready, 'BOOT' and 'PTCH' are not, anything else is a stranger never addressed again. In APP: INT_MASK1 armed read-modify-write, INT_EVENT1 cleared, then ACTIVE_CONTRACT_PDO (0x34) and _RDO (0x35) decoded per USB PD (fixed, variable, battery, PPS, AVS). The contract sets +BUS (36 V behind the 48 V build's buck) and a motion budget (90 % of its watts); the input ceilings' peak over that budget refuses motor power (`Refusal::source`) and cuts it if on (`motorSwitchSetSourceOk()`). The boot self-check's pd-source row names the contract or its absence (absent passes: a DC build has no source to wait for) and the bus-window row narrows to the contract's bus | catalog val-091.75; the peak model's knobs and bench val-091.69 |

## C6 host link, USB, power pins

| Function | Net | P4 pad | Firmware today | Firmness |
|---|---|---|---|---|
| SDIO 3.0 4-bit to the stacked Stamp-AddOn C6: CLK 43, CMD 44, D0-D3 45-48, slave reset 42 | stamp-internal header | not on stamp pads | `flagship_p4/sdkconfig.defaults:64-74` (Kconfig owns them), brought up in `main.cpp:76` (`wifi_up`), version read at `:97` and judged by the self-check's host-link entry | Firm: stamp hardware |
| USB-C on the stamp: USB-Serial/JTAG console, the serial rescue path and the Valence serial binding (SPEC §13.5) | G24/G25 | pads 43/44 NC on the board | IDF console, moved onto the IDF driver at hub start so console text and Valence frames share one ring without tearing each other (`hub/ValenceSerialPort.cpp`) | Firm |
| USB 2.0 host pair, MIPI DSI lanes, stamp 5V_USB_IN | NC | pads 40/41, 57-64, 15 | none | Firm |
| Stamp VIN (from the 5 V buck), SYS_5V out (+5V_SYS), SOC_3.3V out (+3V3_SYS) | `+5V`, `+5V_SYS`, `+3V3_SYS` | pads 14, 39, 28 | none | Firm |

## I2C addresses

| Bus | Device | Address | Source |
|---|---|---|---|
| Private (G34/G36) | Motor current monitor U11 | 0x40 | A0/A1 to GND on the board; `ValencePower.cpp:26` |
| Private | Board monitor U12 | 0x2C, not ruled | `SV_I2C_ADDR` in `system/Supervisor.h`; must stay outside 0x40-0x4F (val-091.19) |
| Qwiic (LPG11/LPG9) | PD controller TPS26750 on the daughterboard | 0x21 | SPEC 2026-09-30 rev A row (ADCIN1 = ADCIN2 = GND); identified by MODE 'APP ' (SLVUCR7 4.1), `system/PdSource.h` |
| Qwiic | user accessories | any free address | n/a |
