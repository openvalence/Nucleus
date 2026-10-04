// ValenceHomeSense -- the board host of the home sense: pad setup, the edge
// latch, the level read and the pull-up probe
// Constraints:
// - Pad setup is gpio_set_direction() and gpio_set_pull_mode(), never
//   gpio_config() (BoardPins.h: the same door for every pad, LP or not).
// - g_rose is the one word the ISR writes; high() on the motion task is its
//   one reader and clears it. A lock-free atomic store, no log, no allocation.
// - probe() busy-waits 2 x kProbeSettleUs on the calling task: the internal
//   ~45 kOhm pull against the bench lead's ~100 pF is a 4.5 us time constant,
//   so 50 us is past ten of them.
// See: ValenceHomeSense.h, MotionArbiter.h (homing), BoardPins.h

#include "system/ValenceHomeSense.h"

#include <atomic>

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_rom_sys.h>

#include "geiger/geiger.h"
#include "system/BoardPins.h"

namespace valence {
namespace {

constexpr const char* kTag = "home";
constexpr bool kHasPin = BOARD_GPIO_HOME_SENSE >= 0;
constexpr uint32_t kProbeSettleUs = 50;

gpio_num_t pad() { return static_cast<gpio_num_t>(BOARD_GPIO_HOME_SENSE); }

std::atomic<bool> g_rose{false};

void onRise(void*) { g_rose.store(true, std::memory_order_relaxed); }

class BoardHomeSense final : public HomeSense {
public:
    bool present() const override { return kHasPin && _ready; }

    Probe probe() override {
        if (!present()) return Probe::undriven;
        if (gpio_get_level(pad()) != 0) return Probe::high;
        gpio_set_pull_mode(pad(), GPIO_PULLUP_ONLY);
        esp_rom_delay_us(kProbeSettleUs);
        const bool floats = gpio_get_level(pad()) != 0;
        gpio_set_pull_mode(pad(), GPIO_PULLDOWN_ONLY);
        esp_rom_delay_us(kProbeSettleUs);
        g_rose.store(false, std::memory_order_relaxed);   // the probe's own rise
        return floats ? Probe::undriven : Probe::low;
    }

    bool high() override {
        if (!present()) return false;
        const bool rose = g_rose.exchange(false, std::memory_order_relaxed);
        return rose || gpio_get_level(pad()) != 0;
    }

    void begin() {
        if constexpr (kHasPin) {
            gpio_set_direction(pad(), GPIO_MODE_INPUT);
            gpio_set_pull_mode(pad(), GPIO_PULLDOWN_ONLY);
            gpio_set_intr_type(pad(), GPIO_INTR_POSEDGE);
            // Already installed by another driver is not an error here.
            const esp_err_t svc = gpio_install_isr_service(0);
            if (svc != ESP_OK && svc != ESP_ERR_INVALID_STATE) {
                GLOGE(kTag, "G%d: GPIO ISR service failed (%s): home refused", BOARD_GPIO_HOME_SENSE,
                      esp_err_to_name(svc));
                return;
            }
            if (gpio_isr_handler_add(pad(), &onRise, nullptr) != ESP_OK ||
                gpio_intr_enable(pad()) != ESP_OK) {
                GLOGE(kTag, "G%d: rising-edge interrupt not armed: home refused", BOARD_GPIO_HOME_SENSE);
                return;
            }
            _ready = true;
            GLOGI(kTag, "home sense on G%d: active high, pull-down, rising edge latched (reads %d)",
                  BOARD_GPIO_HOME_SENSE, gpio_get_level(pad()));
        } else {
            GLOGI(kTag, "no home sense on this board: home op 1 refuses");
        }
    }

private:
    bool _ready = false;
};

BoardHomeSense g_sense;

}  // namespace

HomeSense& homeSenseBegin() {
    g_sense.begin();
    return g_sense;
}

}  // namespace valence
