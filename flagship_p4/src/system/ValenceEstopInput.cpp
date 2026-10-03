// ValenceEstopInput -- implementation. See ValenceEstopInput.h for ownership
// and why it polls.
// Constraints:
// - BoardIo task only, except estopInputRead(). BSS: the reader core (~16 B),
//   one atomic byte, one log flag.
// - The byte is the whole cross-task contract: state, known and masked are
//   packed so the hub task reads one sample's fields together, never one
//   sample's state with another's flags.

#include "system/ValenceEstopInput.h"

#include <atomic>
#include <cstdint>

#include <driver/gpio.h>

#include "geiger/geiger.h"
#include "hub/valence_config.h"
#include "system/BoardPins.h"

namespace valence {

namespace {

constexpr const char* kTag = "estop";

estop::Reader g_reader{kBenchNoMotor};
std::atomic<uint8_t> g_published{estop::Reading{}.pack()};
bool g_stopped = false;   // the last published reading stopped

bool readsHigh(int gpio) { return gpio_get_level(static_cast<gpio_num_t>(gpio)) != 0; }

void logChange(const estop::Reading& r) {
    if (r.masked) {
        GLOGW(kTag, "BENCH: no e-stop wired (NC and NO open), read as released");
    } else if (r.state == estop::Contacts::released) {
        if (g_stopped)
            GLOGW(kTag, "e-stop reads released at the machine: an ESTOP it latched holds until release");
        else
            GLOGI(kTag, "e-stop present and released");
    } else if (r.state == estop::Contacts::pressed) {
        GLOGW(kTag, "e-stop PRESSED at the machine");
    } else {
        GLOGE(kTag, "e-stop %s at the machine", estop::contactsName(r.state));
    }
    g_stopped = r.stops();
}

}  // namespace

void estopInputBegin() {
    gpio_config_t io{};
    io.pin_bit_mask = (1ull << BOARD_GPIO_ESTOP_NC) | (1ull << BOARD_GPIO_ESTOP_NO);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;   // R901/R902 are fitted; this only covers a bare stamp
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
}

void estopInputService(uint32_t nowMs) {
    if (!g_reader.sample(readsHigh(BOARD_GPIO_ESTOP_NC), readsHigh(BOARD_GPIO_ESTOP_NO), nowMs)) return;
    const estop::Reading r = g_reader.reading();
    g_published.store(r.pack(), std::memory_order_release);
    logChange(r);
}

estop::Reading estopInputRead() {
    return estop::Reading::unpack(g_published.load(std::memory_order_acquire));
}

}  // namespace valence
