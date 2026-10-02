// ValenceButtons -- implementation. See ValenceButtons.h for ownership and
// why nothing here acts on a gesture.
// Constraints:
// - BoardIo task only, except homeButtonTake(). BSS: one gesture core and
//   one atomic byte.

#include "system/ValenceButtons.h"

#include <atomic>
#include <cstdint>

#include <driver/gpio.h>

#include "geiger/geiger.h"
#include "system/BoardPins.h"

namespace valence {

namespace {

constexpr const char* kTag = "button";

button::Button g_home;
std::atomic<uint8_t> g_parked{uint8_t(button::Gesture::none)};

}  // namespace

void buttonsBegin() {
    gpio_config_t io{};
    io.pin_bit_mask = 1ull << BOARD_GPIO_BTN_HOME;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;   // R1012 10k is fitted; this only covers a bare stamp
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
}

void buttonsService(uint32_t nowMs) {
    const bool pressed = gpio_get_level(static_cast<gpio_num_t>(BOARD_GPIO_BTN_HOME)) == 0;
    const button::Gesture g = g_home.sample(pressed, nowMs);
    if (g == button::Gesture::none) return;
    if (g == button::Gesture::stuck)
        GLOGW(kTag, "HOME held past %lu s: ignored as stuck (shorted J13 or a jammed SW1?)",
              static_cast<unsigned long>(button::kStuckMs / 1000));
    else
        GLOGW(kTag, "HOME %s at the board", button::gestureName(g));
    g_parked.store(uint8_t(g), std::memory_order_release);
}

button::Gesture homeButtonTake() {
    return button::Gesture(g_parked.exchange(uint8_t(button::Gesture::none), std::memory_order_acquire));
}

}  // namespace valence
