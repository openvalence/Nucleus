// ValenceButtons -- implementation. See ValenceButtons.h for ownership, the
// bindings' home and the one fallback this file acts on.
// Constraints:
// - BoardIo task only, except the two take functions. BSS: two gesture cores,
//   two atomic bytes and the fallback's clock.
// - An absent pad (BoardPins.h -1) is never configured and reads released.
// - On the barnacle (CONFIG_NUCLEUS_BOARD_BARNACLE) PAIR is the R+D board's
//   S2, read through the helper link (BarnacleLink.h). While the helper reads
//   stale PAIR reads released and its core is re-seated, so a press the link
//   lost mid-way completes no gesture, and the new core arms only on a
//   release it sees (ButtonGesture.h).

#include "system/ValenceButtons.h"

#include <atomic>
#include <cstdint>
#include <new>
#include <optional>

#include <driver/gpio.h>
#include <esp_system.h>
#include <sdkconfig.h>

#include "geiger/geiger.h"
#include "motion/ValenceMotion.h"
#include "system/BoardPins.h"
#ifdef CONFIG_NUCLEUS_BOARD_BARNACLE
#include "system/BarnacleLink.h"
#endif

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

button::PairButton g_pairCore;
std::atomic<uint8_t> g_pairParked{uint8_t(button::PairGesture::none)};

// When a HOME hold was parked, while it is still parked. BoardIo task only.
bool g_holdParked = false;
uint32_t g_holdParkedAtMs = 0;

// Active low; an absent pad reads released.
bool padPressed(int gpio) { return gpio >= 0 && gpio_get_level(static_cast<gpio_num_t>(gpio)) == 0; }

// PAIR now; nullopt when it cannot be read.
std::optional<bool> pairPressed() {
#ifdef CONFIG_NUCLEUS_BOARD_BARNACLE
    return helperButtonPressed();
#else
    return padPressed(BOARD_GPIO_BTN_PAIR);
#endif
}

bool g_pairUnread = false;   // the last sample could not read PAIR

button::Gesture sample(Pad& p, uint32_t nowMs) {
    const bool pressed = padPressed(p.gpio);
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

void samplePair(uint32_t nowMs) {
    const std::optional<bool> now = pairPressed();
    if (!now) {
        if (!g_pairUnread) {
            g_pairUnread = true;
            g_pairCore.~PairButton();
            new (&g_pairCore) button::PairButton();
        }
        return;
    }
    g_pairUnread = false;
    const button::PairGesture g = g_pairCore.sample(*now, nowMs);
    if (g == button::PairGesture::none) return;
    if (g == button::PairGesture::stuck)
        GLOGW(kTag, "PAIR held past %lu s: stuck (shorted J13 or a jammed switch?)",
              static_cast<unsigned long>(button::kStuckMs / 1000));
    else
        GLOGW(kTag, "PAIR %s at the board", button::pairGestureName(g));
    // A stuck release binds nothing: its config hold was parked at kConfigHoldMs.
    if (g != button::PairGesture::stuck) g_pairParked.store(uint8_t(g), std::memory_order_release);
}

button::Gesture take(Pad& p) {
    return button::Gesture(p.parked.exchange(uint8_t(button::Gesture::none), std::memory_order_acquire));
}

}  // namespace

void buttonsBegin() {
    gpio_config_t io{};
    for (const int n : {BOARD_GPIO_BTN_HOME, BOARD_GPIO_BTN_PAIR})
        if (n >= 0) io.pin_bit_mask |= 1ull << n;
    if (io.pin_bit_mask == 0) return;
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
    samplePair(nowMs);

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
button::PairGesture pairButtonTake() {
    return button::PairGesture(g_pairParked.exchange(uint8_t(button::PairGesture::none), std::memory_order_acquire));
}

}  // namespace valence
