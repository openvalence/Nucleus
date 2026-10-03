// ValenceSerialPort -- the USB-Serial/JTAG byte pipe under the serial binding:
// the IDF driver, installed once and shared with the console
// Constraints:
// - The console (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG) moves onto the driver in
//   the same call that installs it. Left on its polling path, the console and
//   the driver's ISR would both fill the one 64-byte TX FIFO and interleave
//   inside a USB packet, tearing frames.
// - NEVER BLOCKS: both ring calls take zero ticks. A write is one
//   all-or-nothing ring send, so console text never lands inside a frame. A
//   zero-tick write can also lose the driver's TX mutex to a console write in
//   flight on the other core; the port queues such a frame for the next tick.
// - The driver mallocs its rings. Each stays at or under
//   CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL (4,096 B, sdkconfig.defaults), so both
//   are internal RAM: 5,120 B of rings plus the driver's own object.
// - The ISR lands on the core that installs it. hubBegin() runs on app_main,
//   core 0, beside the esp_hosted SDIO service and off the hub task's core.
// See: ValenceSerialPort.h; ESP-IDF driver/usb_serial_jtag.h

#include "ValenceSerialPort.h"

#include <driver/usb_serial_jtag.h>
#include <driver/usb_serial_jtag_vfs.h>

namespace valence {

namespace {

constexpr uint32_t kTxRingBytes = 4096;
constexpr uint32_t kRxRingBytes = 1024;

class UsbSerialPipe final : public ISerialPipe {
public:
    size_t read(std::span<std::byte> out) override {
        const int n = usb_serial_jtag_read_bytes(out.data(), uint32_t(out.size()), 0);
        return n > 0 ? size_t(n) : 0;
    }
    bool write(std::span<const std::byte> bytes) override {
        return usb_serial_jtag_write_bytes(bytes.data(), bytes.size(), 0) == int(bytes.size());
    }
};

UsbSerialPipe g_pipe;

}  // namespace

ISerialPipe* usbSerialPipeBegin() {
    usb_serial_jtag_driver_config_t cfg{};
    cfg.tx_buffer_size = kTxRingBytes;
    cfg.rx_buffer_size = kRxRingBytes;
    const esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        GLOGE(detail::kSerialTag, "USB-Serial/JTAG driver install failed: %s", esp_err_to_name(err));
        return nullptr;
    }
    usb_serial_jtag_vfs_use_driver();
    GLOGI(detail::kSerialTag, "USB serial binding on the console CDC: %lu B TX ring, %lu B RX ring",
          static_cast<unsigned long>(kTxRingBytes), static_cast<unsigned long>(kRxRingBytes));
    return &g_pipe;
}

}  // namespace valence
