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
  link, the USB console, the E-stop pair and its bypass (boot self-check).
- **Provisional and wired:** the motor switch: MOTOR_EN and PRECHARGE_EN
  (held low from the first line of `app_main`, raised only by the enable
  sequence), MSW_FLT_N, MSW_IMON and EN_NODE (the switch task's watch).
- **Firm and unwired:** board monitor SWIO, status LED data, fan PWM and tach,
  PAIR button, DBG marker, all thirteen accessory LP pads, PD_INT.
- **Provisional and unwired:** CLAMP_MON, SHUNT_TEMP, THERM, RS485
  TX/RX/DE, DRV_ALM, DRV_RDY, HOME button.
- Every unwired row has a bead under val-091 (column "Owed").

## Motion and motor power

| Function | Net | P4 pad | Firmware today | Firmness | Owed |
|---|---|---|---|---|---|
| Quadrature A to the drive, through the always-on 74AHCT125 gate 1 (U10) to J4 MOTION, tapped on TP601 | `/DRV.QUAD_A` | GPIO3 (LPG3), stamp pad 45 | LP core emitter: `ulp/lp_quad.c:32` (PIN_A), edge table at `:77`; steered by `motion/ValenceMotion.cpp:77` | Firm: SPEC 2026-09-23 "P4 pin map after the LP-quadrature-only ruling" | scope rerun val-091.18; reset pull-down hw-7mo |
| Quadrature B, gate 2 (U10), TP604 | `/DRV.QUAD_B` | GPIO5 (LPG5), pad 35 | same, `ulp/lp_quad.c:33` | Firm: same row | same |
| Motor switch main-FET enable: INP of the motor switch controller (U2, TPS48111), 100k pull-down R409 | `/MSW.MOTOR_EN` | GPIO21, pad 22 | Driven LOW from `app_main`'s first line (`system/ValenceSelfCheck.cpp`, `selfCheckHoldMotorOff`); HIGH only in `on`, after a full pre-charge window (`system/MotorSwitch.h`), written by `system/ValenceMotorSwitch.cpp` `driveLocked` | Provisional (hw-kzr) | bench val-091.56 |
| Pre-charge enable: INP_G of U2, through R412 100 R, 100k pull-down R410 | `/MSW.PRECHARGE_EN` | GPIO22, pad 21 | Driven LOW, same sites; HIGH for the 150 ms window only (`MotorSwitch.h` `kPrechargeUs`, its derivation on the constant) | Provisional | bench val-091.56 |
| Motor switch fault: U2 FLT_T and FLT_I tied, 10k pull-up R411, active low | `/MSW.MSW_FLT_N` | GPIO20, pad 23 | Read at boot by the self-check through `motorSwitchFaultLine()`, then every 5 ms by the switch task: low latches the switch faulted and the hub ESTOP, cause fault (`ValenceDevice.cpp` `tick`) | Provisional | split hw-3qf; bench val-091.56 |
| Motor switch current monitor: U2 IMON (8.25k), ADC1 | `/MSW.MSW_IMON` | GPIO19, pad 24 | ADC1 oneshot, calibrated, 4-sample mean, 0.15 V/A; judged at the end of the pre-charge window against `kInrushCeilingA` (`MotorSwitch.h`) | Provisional (ADC1-bound) | bench val-091.56 |
| EN/UVLO wired-OR node of U2: pulled low by the INA ALERT, the bus overvoltage trip (Q405), the E-stop (Q901) and the board monitor's FAULT_N; ADC1, ~1.8 V running, ~0 V tripped | `/MSW.EN_NODE` | GPIO23, pad 20 | ADC1 oneshot, every 5 ms: under `kEnNodeMinV` while pre-charging or on latches faulted (`en_node`); read fresh after the INA ALERT re-arm on every enable | Provisional (ADC1-bound) | cause naming val-091.57; bench val-091.56 |

## Private I2C bus and the board monitor

| Function | Net | P4 pad | Firmware today | Firmness | Owed |
|---|---|---|---|---|---|
| SDA: motor current monitor (U11, INA237 or INA228, 0x40, A0/A1 to GND) and board monitor (U12, CH32V003, 0x2C, not ruled); 4.7k pull-up R24 | `/I2C.INA_SDA` | GPIO34, pad 33 | `system/ValencePower.cpp:24` (its own constant), bus opened at `:89`; the self-check borrows it, `ValenceSelfCheck.cpp:45` | Firm: SPEC 2026-09-23 pin-map-after-LP row | move to BoardPins.h val-091.33; test pads hw-kcu |
| SCL, 4.7k pull-up R25 | `/I2C.INA_SCL` | GPIO36, pad 47 | `ValencePower.cpp:25` | Firm | same |
| U11 ALERT, latched: joins the EN node above, cuts motor power with no firmware | `/MSW.EN_NODE` | (GPIO23) | DIAG_ALRT read (`powerTakeAlerts()`) by the switch task on every enable request, before the window, its over-current and over-voltage flags logged: that read plus the MOTOR_EN toggle is the re-arm (`ValenceMotorSwitch.cpp` `serviceEnable`) | Firm | bench val-091.56 |
| SWIO to the board monitor's PD1 through R1102 100 R (10k pull-up R1101; rescue pad TP1101) | `/MON.MON_SWIO` | GPIO50, pad 10 | nothing | Firm: SPEC 2026-09-23 board-monitor row | val-091.20 |
| Board monitor FAULT_N (U12 PC7 through R1121 1k): can cut motor power, never enable it | onto `/MSW.EN_NODE` | (GPIO23) | seen only as an EN-node drop (row above), cause not read | Firm | val-091.19, val-091.57 |
| Board monitor's own ADC taps (VIN_RAW, +BUS, +12V, +5V, +5V_SYS, +3V3_ACC, SHUNT_TEMP, CLAMP_MON), CLAMP_TRIM PWM, PUMP_FLT (PC0), status LED bank (PC6) | U12 local | not on the P4 | IDENT and STATUS blocks (`system/Supervisor.h`) read once at boot by the self-check, `ValenceSelfCheck.cpp:61`; no runtime poll, no heartbeat | Firm (address not ruled) | val-091.19, val-091.57 |

## Regen clamp, thermal, fan

| Function | Net | P4 pad | Firmware today | Firmness | Owed |
|---|---|---|---|---|---|
| Regen clamp gate monitor: 10k + 3.3k divider off the clamp FET gate (Q501), C504 1 nF; on-time capture for dissipation | `/REGEN.CLAMP_MON` | GPIO17, pad 26 | nothing | Provisional (ADC1-bound) | val-091.25 |
| Regen load temperature: TH501 10k NTC, 10k pull-up R512, C503 100 nF, ADC1 | `/REGEN.SHUNT_TEMP` | GPIO18, pad 25 | nothing | Provisional (ADC1-bound) | val-091.25 |
| Fan drive: Q701 to the high-side P-FET Q702, HIGH = on, R701 100k pull-down; output LC-filtered to DC | `/FAN.FAN_PWM` | GPIO27 (USB1.1 D+), pad 17 | nothing | Firm: SPEC 2026-09-23 pin-map-after-LP row | val-091.28 |
| Fan tach, open collector, 10k pull-up, 1k series | `/FAN.FAN_TACH` | GPIO38, pad 32 | nothing | Firm: same row | val-091.28 |
| Thermistor header J7: 1k series R27 + C14 100 nF, ADC1 | `/FAN.THERM` | GPIO16, pad 27 | nothing | Provisional (ADC1-bound) | val-091.28 |

## Drive comms and status

| Function | Net | P4 pad | Firmware today | Firmness | Owed |
|---|---|---|---|---|---|
| RS485 transmit data into the transceiver (U9, THVD1450) D | `/DRV.RS485_TX` | GPIO33, pad 50 | nothing | Provisional | val-091.29; test pads hw-kcu |
| RS485 receive from U9 R | `/DRV.RS485_RX` | GPIO32, pad 46 | nothing | Provisional | val-091.29 |
| RS485 direction: U9 DE and /RE tied, HIGH transmits | `/DRV.RS485_DE` | GPIO41, pad 49 | nothing | Provisional | val-091.29 |
| Drive alarm (WR, opto NPN to COM), 10k pull-up + 1k R20 | `/DRV.DRV_ALM` | GPIO31, pad 51 | nothing | Provisional | val-091.29 |
| Drive ready / following error (RDY), 10k pull-up + 1k R21 | `/DRV.DRV_RDY` | GPIO28, pad 52 | nothing | Provisional | val-091.29 |
| Encoder index ZO | not brought out | none | n/a: absolute position comes over Modbus (SPEC 2026-09-23 drive-connector row) | Firm | none |

## E-stop, buttons, LED, debug

| Function | Net | P4 pad | Firmware today | Firmness | Owed |
|---|---|---|---|---|---|
| E-stop NC contact (J9, and the daughterboard M8 in parallel), 2.2k pull-up, 1k R903; HIGH = open | `/ESTOP.ESTOP_NC` | GPIO39, pad 31 | Decoded once at boot: `ValenceSelfCheck.cpp:149` | Firm: SPEC 2026-09-23 E-stop row (+2026-09-28, 2026-10-01) | runtime latch val-091.23 |
| E-stop NO contact, 2.2k pull-up, 1k R904 | `/ESTOP.ESTOP_NO` | GPIO30, pad 53 | same | Firm | val-091.23 |
| E-stop bypass drive (Q903, masks an unplugged cable only), 100k pull-down R906 | `/ESTOP.ESTOP_BYP` | GPIO29, pad 54 | Driven LOW: `ValenceSelfCheck.cpp:225` | Firm | bypass policy val-091.23 |
| HOME button SW1 (press homes, hold resets, on release), 10k pull-up, 1k series; J13 in parallel | `/UI.BTN_HOME` | GPIO49, pad 12 | nothing | Provisional | val-091.26 |
| PAIR button SW2 (hold pairs, held at power-on = config mode), 10k pull-up, 1k series; J13 in parallel | `/UI.BTN_PAIR` | GPIO52, pad 55 | nothing | Firm: SPEC 2026-09-23 pin-map-after-LP row | val-9u0.10, val-9u0.14 |
| Status LED data: 74AHCT125 gate 3 (U10) to the GRBW pixel D5 (XL-3528RGBW, 32 bits per pixel), chained on to J12 NEOPIXEL OUT; R601 10k pull-down | `/DRV.LED_DATA` | GPIO26 (USB1.1 D-), pad 19 | nothing | Firm: same row | val-091.27 |
| DBG marker through R602 1k to TP603; the ROM boot log appears on it at every reset (U0TX) | `/DRV.DBG` | GPIO37, pad 30 | nothing | Firm: SPEC 2026-09-23 board-monitor row | val-091.32 |
| BOOT strap to tweezer pad TP903 (held low at power-on = download mode) | `Net-(TP903-Pad1)` | GPIO35, pad 36 (pad 48 NC) | never used, by rule | Firm | none |
| CHIP_EN to tweezer pad TP901 | `Net-(U8-CHIP_EN)` | CHIP_EN, pad 34 | n/a | Firm | none |

## Accessory headers and the PD daughterboard

All on spare LP pads, driven by HP peripherals through the GPIO matrix, never
the LP UART or LP I2C (SPEC 2026-09-23 LP rule restated; val-091.18 measures
the cost to quadrature). All firm: SPEC 2026-09-23 accessory-breakouts row,
2026-09-27 J15 row, 2026-09-30 PD row. All unwired; owed under val-091.30
unless noted.

| Function | Net | P4 pad |
|---|---|---|
| Pump PWM, 220 R, 100k pull-down; J10 (12 V through the pump eFuse U14, whose FLT goes to the board monitor) | `/ACC.PUMP_PWM` | GPIO13 (LPG13), pad 29 |
| External PWM, 220 R, 100k pull-down; J11 | `/ACC.EXT_PWM` | GPIO2 (LPG2), pad 37 |
| AUX1 button input, 10k pull-up, 1k series; J13 | `/ACC.AUX1` | GPIO4 (LPG4), pad 16 |
| AUX2 button input, same; J13 | `/ACC.AUX2` | GPIO6 (LPG6), pad 18 |
| Qwiic SDA, 2.2k pull-up; J14, and PD_SDA through R1024 33 R to J16 | `/ACC.QWIIC_SDA` | GPIO11 (LPG11), pad 4 |
| Qwiic SCL, 2.2k pull-up; J14, and PD_SCL through R1025 33 R | `/ACC.QWIIC_SCL` | GPIO9 (LPG9), pad 6 |
| Header UART TX, 220 R; J15 | `/ACC.GPIO_TX` | GPIO14 (LPG14), pad 2 |
| Header UART RX, 220 R; J15 | `/ACC.GPIO_RX` | GPIO15 (LPG15), pad 1 |
| Header IO1..IO5, 220 R each; J15 | `/ACC.GPIO_IO1..5` | GPIO12/10/8/7/1, pads 3/5/7/8/9 |
| PD daughterboard interrupt through R1023 220 R to J16 pad 4 (J15.12 is NC) | `/ACC.GPIO_IO6` | GPIO0 (LPG0), pad 11 |
| PD controller (TPS26750) on the Qwiic bus, I2C 0x21 | via J16 | owed under val-091.31 |

## C6 host link, USB, power pins

| Function | Net | P4 pad | Firmware today | Firmness |
|---|---|---|---|---|
| SDIO 3.0 4-bit to the stacked Stamp-AddOn C6: CLK 43, CMD 44, D0-D3 45-48, slave reset 42 | stamp-internal header | not on stamp pads | `flagship_p4/sdkconfig.defaults:64-74` (Kconfig owns them), brought up in `main.cpp:72` (`wifi_up`), version read at `:93` and judged by the self-check's host-link entry | Firm: stamp hardware |
| USB-C on the stamp: USB-Serial/JTAG console and the serial rescue path | G24/G25 | pads 43/44 NC on the board | IDF console | Firm |
| USB 2.0 host pair, MIPI DSI lanes, stamp 5V_USB_IN | NC | pads 40/41, 57-64, 15 | none | Firm |
| Stamp VIN (from the 5 V buck), SYS_5V out (+5V_SYS), SOC_3.3V out (+3V3_SYS) | `+5V`, `+5V_SYS`, `+3V3_SYS` | pads 14, 39, 28 | none | Firm |

## I2C addresses

| Bus | Device | Address | Source |
|---|---|---|---|
| Private (G34/G36) | Motor current monitor U11 | 0x40 | A0/A1 to GND on the board; `ValencePower.cpp:26` |
| Private | Board monitor U12 | 0x2C, not ruled | `SV_I2C_ADDR` in `system/Supervisor.h`; must stay outside 0x40-0x4F (val-091.19) |
| Qwiic (LPG11/LPG9) | PD controller TPS26750 on the daughterboard | 0x21 | SPEC 2026-10-01 rev A row |
| Qwiic | user accessories | any free address | n/a |
