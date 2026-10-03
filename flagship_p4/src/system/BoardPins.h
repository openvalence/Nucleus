// BoardPins -- every ESP32-P4 pad the OSSM Flagship wires, named by function
// Constraints:
// - C AND C++. ulp/lp_quad.c includes this file, so it holds plain integer
//   macros only: no IDF types, no namespaces. C++ callers cast at the use
//   site (static_cast<gpio_num_t>(BOARD_GPIO_X)).
// - The one home for a P4 pin number in this project (architecture.md
//   section 1). One exception, owned elsewhere: the C6 SDIO pins live in
//   sdkconfig.defaults because Kconfig consumes them.
// - LPGn is LP_IO n is GPIO n, verified on silicon (val-091.4). The LP core
//   drives QUAD_A and QUAD_B and NOTHING ELSE; every other LP pad is optional
//   accessory I/O for HP peripherals through the GPIO matrix, never the LP
//   UART or LP I2C (Hardware SPEC.md 2026-09-23 LP rule, val-091.18).
// - NEVER gpio_config() OR gpio_reset_pin() AN LP PAD (GPIO0..15). On IDF
//   5.5 both run rtc_gpio_deinit(), which can gate off the LP GPIO clock the
//   emitter's two pads run on (val-091.72). Set an LP pad up with
//   gpio_set_direction() and gpio_set_pull_mode(), or through an HP
//   peripheral driver's own pin setup.
// - Analog nets must stay on ADC1 pads (GPIO16..23). Never move a function
//   onto a strap: G35 BOOT, G36 mode select, G37 ROM log, G38 download sample.
// - A TODO(hw-kzr) pin is from the SPEC's provisional map ("swap freely at
//   layout") and has not been re-ruled; it may still move.
// See: docs/board-map.md (net names, parts, firmness, what firmware does with
// each pin), ../Hardware/flagship/SPEC.md

#pragma once

// ---- motion: quadrature to the drive, LP core only --------------------------
// Through the always-enabled 74AHCT125 gates 1-2 (U10) to J4 MOTION, tapped
// before the buffer on TP601/TP604.
#define BOARD_GPIO_QUAD_A 3   // LPG3
#define BOARD_GPIO_QUAD_B 5   // LPG5

// ---- motor switch (U2, TPS48111) --------------------------------------------
// MOTOR_EN and PRECHARGE_EN carry 100k pull-downs (R409, R410): a P4 in reset
// holds the switch off. Drive them LOW until the self-check opens the gate.
// TODO(hw-kzr): every pin in this section is provisional.
#define BOARD_GPIO_MOTOR_EN     21  // INP: main FETs
#define BOARD_GPIO_PRECHARGE_EN 22  // INP_G: pre-charge FET through R412 100 R
#define BOARD_GPIO_MSW_FLT_N    20  // FLT_T and FLT_I tied, active low (hw-3qf)
#define BOARD_GPIO_MSW_IMON     19  // ADC1: switch current monitor
// ADC1: the EN/UVLO wired-OR every hardware kill pulls (INA ALERT, bus OVP
// trip, E-stop, board monitor FAULT_N). ~1.8 V running, ~0 V tripped.
#define BOARD_GPIO_EN_NODE      23

// ---- private I2C: motor current monitor (U11 at 0x40) + board monitor (U12) -
#define BOARD_GPIO_INA_SDA 34   // JTAG strap, ignored at default eFuses
#define BOARD_GPIO_INA_SCL 36   // strap high by default; 4.7k pull-up keeps it
// SWIO to the board monitor's PD1 through R1102 100 R (val-091.20).
#define BOARD_GPIO_MON_SWIO 50

// ---- regen clamp ------------------------------------------------------------
// TODO(hw-kzr): both pins are provisional.
#define BOARD_GPIO_CLAMP_MON  17  // ADC1 + on-time capture: clamp gate divider
#define BOARD_GPIO_SHUNT_TEMP 18  // ADC1: TH501, 10k NTC beside the regen load

// ---- drive comms (U9, THVD1450) and drive status ----------------------------
// TODO(hw-kzr): every pin in this section is provisional.
#define BOARD_GPIO_RS485_TX 33   // D
#define BOARD_GPIO_RS485_RX 32   // R
#define BOARD_GPIO_RS485_DE 41   // DE and /RE tied: HIGH transmits
#define BOARD_GPIO_DRV_ALM  31   // drive WR alarm: opto NPN to COM, 10k pull-up
#define BOARD_GPIO_DRV_RDY  28   // drive RDY / following error, same stage

// ---- external E-stop (J9) ---------------------------------------------------
// (NC, NO): 0/1 normal, 1/0 pressed, 1/1 unplugged, 0/0 wiring fault. The stop
// itself is hardware; these only report it.
#define BOARD_GPIO_ESTOP_NC  39
#define BOARD_GPIO_ESTOP_NO  30
// Masks any state where NO reads open (unplugged, a broken NO wire, a 2-wire
// NC button), never a pressed button with NO closed (SPEC 2026-10-02
// correction); 100k pull-down.
#define BOARD_GPIO_ESTOP_BYP 29

// ---- buttons, status LED, debug ---------------------------------------------
#define BOARD_GPIO_BTN_HOME 49   // TODO(hw-kzr): provisional. Active low.
#define BOARD_GPIO_BTN_PAIR 52   // active low
// Through 74AHCT125 gate 3 to the GRBW status pixel (32 bits per pixel), whose
// DOUT continues to J12 NEOPIXEL OUT. R601 10k holds it low through boot.
#define BOARD_GPIO_LED_DATA 26   // USB1.1 D-, free while OTG is off
// U0TX: the ROM boot log appears here on EVERY reset.
#define BOARD_GPIO_DBG      37

// ---- fan and thermistor -----------------------------------------------------
#define BOARD_GPIO_FAN_PWM  27   // HIGH = fan on; USB1.1 D+, free with OTG off
#define BOARD_GPIO_FAN_TACH 38   // open collector, 10k pull-up
#define BOARD_GPIO_THERM    16   // TODO(hw-kzr): provisional. ADC1, J7 NTC

// ---- accessory headers, spare LP pads, HP peripherals only ------------------
#define BOARD_GPIO_PUMP_PWM    13   // LPG13, J10
#define BOARD_GPIO_EXT_PWM      2   // LPG2, J11
#define BOARD_GPIO_AUX1         4   // LPG4, J13, active low
#define BOARD_GPIO_AUX2         6   // LPG6, J13, active low
#define BOARD_GPIO_QWIIC_SDA   11   // LPG11, J14 and the PD daughterboard (J16)
#define BOARD_GPIO_QWIIC_SCL    9   // LPG9, J14 and J16
#define BOARD_GPIO_ACC_UART_TX 14   // LPG14, J15
#define BOARD_GPIO_ACC_UART_RX 15   // LPG15, J15
// The HP UART behind those two pads, opened on a consumer's first ask
// (ValenceAccessoryIo.cpp). Never UART0 (G37's ROM log) or the LP UART.
#define BOARD_UART_ACC          2
#define BOARD_GPIO_ACC_IO1     12   // LPG12, J15
#define BOARD_GPIO_ACC_IO2     10   // LPG10, J15
#define BOARD_GPIO_ACC_IO3      8   // LPG8, J15
#define BOARD_GPIO_ACC_IO4      7   // LPG7, J15
#define BOARD_GPIO_ACC_IO5      1   // LPG1, J15
#define BOARD_GPIO_PD_INT       0   // LPG0, J16 pogo; J15.12 is no-connect

