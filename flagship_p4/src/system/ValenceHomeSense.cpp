// ValenceHomeSense -- the board host of the home sense: pad setup, the rise's
// park, stamp and wake, the confirmed level read and the pull-up probe
// Constraints:
// - Pad setup is gpio_set_direction() and gpio_set_pull_mode(), never
//   gpio_config() (BoardPins.h: the same door for every pad, LP or not).
// - The ISR calls the arbiter's park (`rose`) before anything else, then
//   writes g_rose_us and g_rose, stamp first, then the wake; high() on the
//   planner task is the stamp's one reader and clears g_rose. Lock-free atomic
//   stores and the two calls, no log, no allocation.
// - The GPIO ISR service is installed without ESP_INTR_FLAG_IRAM, so the
//   handler is held off while the flash cache is disabled; the LP core keeps
//   rendering the seek through a flash write.
//   TODO(val-sv6): an IRAM-resident park that a flash write cannot hold off.
// - high() busy-waits at most kHomeSenseDebounceUs on the planner task, once
//   per rise: the second read of a rise that young. The source drives the
//   line push-pull, so two reads that far apart reject a coupled spike and
//   cost a real stall nothing it can measure.
// - probe() busy-waits 2 x kProbeSettleUs on the calling task: the internal
//   ~45 kOhm pull against the bench lead's ~100 pF is a 4.5 us time constant,
//   so 50 us is past ten of them.
// See: ValenceHomeSense.h, MotionArbiter.h (homing), BoardPins.h

#include "system/ValenceHomeSense.h"

#include <atomic>

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>

#include "geiger/geiger.h"
#include "system/BoardPins.h"

namespace valence {
namespace {

constexpr const char* kTag = "home";
constexpr bool kHasPin = BOARD_GPIO_HOME_SENSE >= 0;
constexpr uint32_t kProbeSettleUs = 50;

gpio_num_t pad() { return static_cast<gpio_num_t>(BOARD_GPIO_HOME_SENSE); }

std::atomic<uint32_t> g_rose_us{0};   // esp_timer's low word at the last rise
std::atomic<bool> g_rose{false};
std::atomic<HomeSenseIsr> g_rose_cb{nullptr};
std::atomic<HomeSenseIsr> g_wake{nullptr};

uint32_t nowUs32() { return static_cast<uint32_t>(esp_timer_get_time()); }

// The park first: nothing ahead of it but the dispatcher.
void onRise(void*) {
    if (const HomeSenseIsr rose = g_rose_cb.load(std::memory_order_relaxed)) rose();
    g_rose_us.store(nowUs32(), std::memory_order_relaxed);
    g_rose.store(true, std::memory_order_release);
    if (const HomeSenseIsr wake = g_wake.load(std::memory_order_relaxed)) wake();
}

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
        if (!present() || gpio_get_level(pad()) == 0) return false;
        if (!g_rose.exchange(false, std::memory_order_acquire)) return true;
        const uint32_t age = nowUs32() - g_rose_us.load(std::memory_order_relaxed);
        if (age >= kHomeSenseDebounceUs) return true;
        esp_rom_delay_us(kHomeSenseDebounceUs - age);
        return gpio_get_level(pad()) != 0;
    }

    void begin(HomeSenseIsr rose, HomeSenseIsr wake) {
        g_rose_cb.store(rose, std::memory_order_relaxed);
        g_wake.store(wake, std::memory_order_relaxed);
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
            GLOGI(kTag, "home sense on G%d: active high, pull-down, a rise confirmed %lu us on (reads %d)",
                  BOARD_GPIO_HOME_SENSE, static_cast<unsigned long>(kHomeSenseDebounceUs),
                  gpio_get_level(pad()));
        } else {
            GLOGI(kTag, "no home sense on this board: home op 1 refuses");
        }
    }

private:
    bool _ready = false;
};

BoardHomeSense g_sense;

}  // namespace

HomeSense& homeSenseBegin(HomeSenseIsr rose, HomeSenseIsr wake) {
    g_sense.begin(rose, wake);
    return g_sense;
}

}  // namespace valence
