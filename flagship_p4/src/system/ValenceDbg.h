#pragma once

// ValenceDbg -- the DBG marker pin (G37, through R602 1k to TP603) as a scope
// trigger: a level and a burst of pulses
// Constraints:
// - Used from nowhere by default. A bench image calls dbgBegin() once, then
//   dbgLevel() / dbgPulse() at the sites under measurement; nothing else in
//   the firmware drives G37.
// - G37 is U0TX: the ROM prints its boot log on it at EVERY reset, so a
//   capture ignores the pin until dbgBegin() has claimed the pad. The console
//   is USB-Serial/JTAG (sdkconfig.defaults), so nothing in the app uses UART0.
// - dbgLevel() is ONE register store (GPIO out1 set or clear), from any task
//   or an ISR, no lock, no branch at a constant argument. dbgPulse(n) is 2n
//   stores back to back: each pulse lasts one GPIO bus write, which the
//   scope measures (val-091.66); for a wider mark use two dbgLevel() calls
//   around the work.
// - Before dbgBegin() both calls write the output register of a pad still
//   routed to the UART, so they change nothing on the pin.
// See: BoardPins.h, docs/board-map.md (E-stop, buttons, LED, debug),
// bd val-091.32

#include <cstdint>

#include <driver/gpio.h>
#include <soc/gpio_struct.h>

#include "system/BoardPins.h"

namespace valence {

static_assert(BOARD_GPIO_DBG >= 32, "dbgLevel writes the out1 register bank");

// Claims G37 as a push-pull output, low. Call once, from task context.
inline void dbgBegin() {
    const auto pad = static_cast<gpio_num_t>(BOARD_GPIO_DBG);
    gpio_reset_pin(pad);
    gpio_set_level(pad, 0);
    gpio_set_direction(pad, GPIO_MODE_OUTPUT);
}

inline void dbgLevel(bool high) {
    constexpr uint32_t kMask = 1u << (BOARD_GPIO_DBG - 32);
    if (high) GPIO.out1_w1ts.val = kMask;
    else      GPIO.out1_w1tc.val = kMask;
}

inline void dbgPulse(uint32_t n = 1) {
    for (uint32_t i = 0; i < n; ++i) {
        dbgLevel(true);
        dbgLevel(false);
    }
}

}  // namespace valence
