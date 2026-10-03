// ValenceButtons -- implementation. See ValenceButtons.h for ownership, the
// bindings' home and the one fallback this file acts on.
// Constraints:
// - BoardIo task only, except the two take functions. BSS: two gesture cores,
//   two atomic bytes and the fallback's clock.

#include "system/ValenceButtons.h"

#include <atomic>
#include <cstdint>

#include <driver/gpio.h>
#include <esp_system.h>

#include "geiger/geiger.h"
#include "motion/ValenceMotion.h"
#include "system/BoardPins.h"

namespace valence {

namespace {

constexpr const char* kTag = "button";

struct Pad {
    const char* name;
    int gpio;
    button::Button core;
    std::atomic<uint8_t> parked{uint8_t(button::Gesture::none)};
};

Pad g_home{"HOME", BOARD_GPIO_BTN_HOME};
Pad g_pair{"PAIR", BOARD_GPIO_BTN_PAIR};

// When a HOME hold was parked, while it is still parked. BoardIo task only.
bool g_holdParked = false;
uint32_t g_holdParkedAtMs = 0;

button::Gesture sample(Pad& p, uint32_t nowMs) {
    const bool pressed = gpio_get_level(static_cast<gpio_num_t>(p.gpio)) == 0;
    const button::Gesture g = p.core.sample(pressed, nowMs);
    if (g == button::Gesture::none) return g;
    if (g == button::Gesture::stuck)
        GLOGW(kTag, "%s held past %lu s: ignored as stuck (shorted J13 or a jammed switch?)", p.name,
              static_cast<unsigned long>(button::kStuckMs / 1000));
    else
        GLOGW(kTag, "%s %s at the board", p.name, button::gestureName(g));
    p.parked.store(uint8_t(g), std::memory_order_release);
    return g;
}

button::Gesture take(Pad& p) {
    return button::Gesture(p.parked.exchange(uint8_t(button::Gesture::none), std::memory_order_acquire));
}

}  // namespace

void buttonsBegin() {
    gpio_config_t io{};
    io.pin_bit_mask = (1ull << BOARD_GPIO_BTN_HOME) | (1ull << BOARD_GPIO_BTN_PAIR);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;   // R1012 / its PAIR twin are fitted; this only covers a bare stamp
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
}

void buttonsService(uint32_t nowMs) {
    if (sample(g_home, nowMs) == button::Gesture::hold) {
        g_holdParked = true;
        g_holdParkedAtMs = nowMs;
    }
    sample(g_pair, nowMs);

    if (!g_holdParked) return;
    if (g_home.parked.load(std::memory_order_acquire) != uint8_t(button::Gesture::hold)) {
        g_holdParked = false;   // taken (or overwritten by a newer gesture)
        return;
    }
    if (nowMs - g_holdParkedAtMs < kHoldFallbackMs) return;
    GLOGE(kTag, "HOME hold not taken by the hub in %lu ms: hub not running, cutting motor power "
                "and restarting without a GOODBYE",
          static_cast<unsigned long>(kHoldFallbackMs));
    motionEstop();
    esp_restart();
}

button::Gesture homeButtonTake() { return take(g_home); }
button::Gesture pairButtonTake() { return take(g_pair); }

}  // namespace valence
