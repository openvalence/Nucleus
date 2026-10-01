#pragma once

// funconfig -- ch32v003fun build configuration for the board monitor
// Constraints:
// - 48 MHz from the internal HSI through the PLL; the I2C FREQ field and the
//   20 kHz trim PWM in src/main.c are derived from it.
// - Debug printf over SWIO is off: SWIO belongs to the P4's programmer.

#define CH32V003 1
#define FUNCONF_USE_HSI 1
#define FUNCONF_USE_PLL 1
#define FUNCONF_SYSTEM_CORE_CLOCK 48000000
#define FUNCONF_USE_DEBUGPRINTF 0
