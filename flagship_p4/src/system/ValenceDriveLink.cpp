// ValenceDriveLink -- the drive link's board host: the RS485 UART, the two
// status pads, the task, and the crossings out of it
// Constraints:
// - See ValenceDriveLink.h for the task, the crossings and the DE rule;
//   AimDrive.h and ModbusRtu.h hold every decision.
// - RS485 HALF-DUPLEX IS SET BEFORE THE PINS ARE ROUTED. The mode drives the
//   UART's RTS low (sw_rts = 1, IDF 5.5.4 esp32p4 uart_ll.h), so G41 already
//   reads receive the instant it leaves the GPIO driving it low. From then on
//   RTS IS DE: high while a frame goes out, low again at TX_DONE (uart.c).
// - No TX ring: uart_write_bytes() copies a frame into the 128-byte TX FIFO
//   and returns, and no frame exceeds modbus::kMaxFrameBytes.
// - RX carries the pad's pull-up: the transceiver's R output is
//   high-impedance while DE (tied to /RE) is high, and a floating RX would
//   read our own transmit time as noise.
// - The UART ISR is not in IRAM. Through a flash write a reply waits in the
//   128-byte RX FIFO, and Master::poll() reads before it judges the deadline,
//   so a reply that arrived in time is never thrown away as late.
// - Built without CONFIG_NUCLEUS_DRIVE_LINK, the code below still compiles
//   (if constexpr) so the omitted build cannot rot, but the UART is never
//   installed and no transaction ever starts.
// See: ValenceDriveLink.h, AimDrive.h, ModbusRtu.h, BoardPins.h,
// bd val-091.29

#include "system/ValenceDriveLink.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include "geiger/geiger.h"
#include "system/BoardPins.h"
#include "system/ValenceMotorSwitch.h"

namespace valence {

namespace {

constexpr const char* kTag = "drive";

#if defined(CONFIG_NUCLEUS_DRIVE_LINK)
constexpr bool kBuilt = true;
#else
constexpr bool kBuilt = false;
#endif

constexpr uart_port_t kUart = uart_port_t(BOARD_UART_RS485);
static_assert(BOARD_UART_RS485 >= 1 && BOARD_UART_RS485 < SOC_UART_HP_NUM, "an HP UART, never UART0");
// The driver's RX ring must exceed the 128-byte hardware FIFO.
constexpr int kRxRingBytes = 256;

constexpr gpio_num_t pin(int n) { return static_cast<gpio_num_t>(n); }

static_assert(std::is_trivially_copyable_v<DriveLinkStatus>, "the mailbox copies it bytewise");
static_assert(std::is_trivially_copyable_v<DriveRead>, "the request queue copies it bytewise");
static_assert(std::is_trivially_copyable_v<DriveReadResult>, "the result queue copies it bytewise");

// ---- the UART as the master sees it -----------------------------------------------

class UartPort final : public modbus::IPort {
public:
    void setBaud(uint32_t baud) override { uart_set_baudrate(kUart, baud); }
    void discardInput() override { uart_flush_input(kUart); }
    void write(std::span<const uint8_t> frame) override {
        uart_write_bytes(kUart, frame.data(), frame.size());
    }
    size_t read(std::span<uint8_t> into) override {
        const int n = uart_read_bytes(kUart, into.data(), uint32_t(into.size()), 0);
        return n > 0 ? size_t(n) : 0;
    }
};

// ---- state: the task's alone after driveLinkBegin() ---------------------------------

UartPort g_port;
aim::DrivePower g_power;
aim::StatusLines g_lines;
aim::Link g_link{g_port};
TaskHandle_t g_task = nullptr;
// Written once by driveLinkBegin() before the task exists.
bool g_uartUp = false;

// ---- the crossings -------------------------------------------------------------------

std::atomic<bool> g_alarmPending{false};

StaticQueue_t g_statusQ;
std::array<uint8_t, sizeof(DriveLinkStatus)> g_statusStore{};
QueueHandle_t g_status = nullptr;

StaticQueue_t g_readQ;
std::array<uint8_t, kDriveLinkQueueDepth * sizeof(DriveRead)> g_readStore{};
QueueHandle_t g_reads = nullptr;

StaticQueue_t g_resultQ;
std::array<uint8_t, kDriveLinkQueueDepth * sizeof(DriveReadResult)> g_resultStore{};
QueueHandle_t g_results = nullptr;

// ---- the UART ---------------------------------------------------------------------------

bool uartBegin() {
    uart_config_t cfg{};
    cfg.baud_rate = int(aim::kBauds[0]);
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    // Order is the DE rule (file header): mode before pins.
    esp_err_t err = uart_driver_install(kUart, kRxRingBytes, 0, 0, nullptr, 0);
    if (err == ESP_OK) err = uart_param_config(kUart, &cfg);
    if (err == ESP_OK) err = uart_set_mode(kUart, UART_MODE_RS485_HALF_DUPLEX);
    if (err == ESP_OK)
        err = uart_set_pin(kUart, BOARD_GPIO_RS485_TX, BOARD_GPIO_RS485_RX, BOARD_GPIO_RS485_DE,
                           UART_PIN_NO_CHANGE);
    if (err == ESP_OK) err = gpio_pullup_en(pin(BOARD_GPIO_RS485_RX));
    if (err != ESP_OK) {
        GLOGE(kTag, "UART%d RS485 setup failed (%s): no drive link this boot; DE held low",
              int(kUart), esp_err_to_name(err));
        return false;
    }
    return true;
}

// ---- reads for other tasks ---------------------------------------------------------------

void answer(uint32_t tag, const modbus::Result& r) {
    DriveReadResult out;
    out.tag = tag;
    out.result = r;
    if (xQueueSend(g_results, &out, 0) != pdTRUE)
        GLOGW_EVERY_MS(10000, kTag, "a read's answer was dropped: no one is taking results");
}

void serveReads(bool linked) {
    DriveRead q{};
    modbus::Result refused;
    refused.status = modbus::Status::not_sent;
    // Off the air, every request is answered at once.
    if (!linked || g_link.snapshot().state != aim::LinkState::up) {
        while (xQueueReceive(g_reads, &q, 0) == pdTRUE) answer(q.tag, refused);
        return;
    }
    if (g_link.canSubmit() && xQueueReceive(g_reads, &q, 0) == pdTRUE &&
        !g_link.submit(modbus::readHolding(aim::kAddress, q.reg, q.count), q.tag))
        answer(q.tag, refused);
}

// ---- the report ---------------------------------------------------------------------------

void reportAlarm() {
    if (g_lines.raisedAtArming())
        GLOGE(kTag, "DRV_ALM asserted as the drive came up: the drive is in alarm, or its WR is "
                    "normally closed (0x07 odd); the hub latches ESTOP");
    else
        GLOGE(kTag, "DRV_ALM: the drive raised an alarm and halted; the hub latches ESTOP");
}

void reportLink(const aim::Snapshot& was, const aim::Snapshot& is) {
    if (is.state != was.state) {
        if (is.state == aim::LinkState::up)
            GLOGI(kTag, "link up at %lu baud: %s", static_cast<unsigned long>(is.baud),
                  is.probe.outcome == aim::ProbeOutcome::ok ? "probe passed" : "probe found a fault");
        else if (was.state == aim::LinkState::up && is.state == aim::LinkState::down)
            GLOGW(kTag, "link lost: %u polls unanswered; probing again in %lu ms",
                  unsigned(aim::kLostAfterFailures), static_cast<unsigned long>(aim::kReprobeMs));
        else
            GLOGD(kTag, "link %s -> %s", aim::linkStateName(was.state), aim::linkStateName(is.state));
    }
    if (is.alarmCodeKnown && (!was.alarmCodeKnown || is.alarmCode != was.alarmCode)) {
        if (is.alarmCode != 0)
            GLOGW(kTag, "drive alarm code 0x%02x (%s)", unsigned(is.alarmCode), aim::alarmName(is.alarmCode));
        else if (was.alarmCodeKnown)
            GLOGI(kTag, "drive alarm code cleared");
    }
}

DriveLinkStatus compose() {
    DriveLinkStatus s;
    s.built = kBuilt;
    s.uart = g_uartUp;
    s.alarm = g_lines.alarm();
    s.ready = g_lines.ready();
    s.armed = g_lines.armed();
    s.alarms = g_lines.raised();
    s.link = g_link.snapshot();
    return s;
}

bool differs(const DriveLinkStatus& a, const DriveLinkStatus& b) {
    return a.link.seq != b.link.seq || a.alarm != b.alarm || a.ready != b.ready || a.armed != b.armed ||
           a.alarms != b.alarms;
}

// ---- the task -------------------------------------------------------------------------------

void taskMain(void*) {
    const bool linked = kBuilt && g_uartUp;
    DriveLinkStatus published = compose();
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(kDriveLinkPeriodMs));
        const uint32_t nowMs = uint32_t(esp_timer_get_time() / 1000);
        g_power.sample(motorSwitchStatus().state == motorswitch::State::on, nowMs);
        const bool raised = g_lines.sample(gpio_get_level(pin(BOARD_GPIO_DRV_ALM)) == 0,
                                           gpio_get_level(pin(BOARD_GPIO_DRV_RDY)) == 0, g_power, nowMs);
        if (raised) {
            g_alarmPending.store(true, std::memory_order_release);
            reportAlarm();
        }
        serveReads(linked);
        if (linked) {
            g_link.step(nowMs, g_power, raised);
            if (const std::optional<aim::External> r = g_link.takeResult()) answer(r->tag, r->result);
        }
        const DriveLinkStatus now = compose();
        if (!differs(now, published)) continue;
        reportLink(published.link, now.link);
        xQueueOverwrite(g_status, &now);
        published = now;
    }
}

}  // namespace

// ---- the door ----------------------------------------------------------------------------------

void driveLinkHoldIdle() {
    // Level first, then direction: the pad never drives high.
    gpio_set_level(pin(BOARD_GPIO_RS485_DE), 0);
    gpio_config_t out{};
    out.pin_bit_mask = 1ULL << BOARD_GPIO_RS485_DE;
    out.mode = GPIO_MODE_OUTPUT;
    gpio_config(&out);
    gpio_set_level(pin(BOARD_GPIO_RS485_DE), 0);

    // The board fits 10k pull-ups; the internal ones give a bare stamp a
    // defined reading, which is HIGH: no alarm, not ready.
    gpio_config_t in{};
    in.pin_bit_mask = (1ULL << BOARD_GPIO_DRV_ALM) | (1ULL << BOARD_GPIO_DRV_RDY);
    in.mode = GPIO_MODE_INPUT;
    in.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&in);
}

bool driveLinkBegin() {
    g_status = xQueueCreateStatic(1, sizeof(DriveLinkStatus), g_statusStore.data(), &g_statusQ);
    g_reads = xQueueCreateStatic(kDriveLinkQueueDepth, sizeof(DriveRead), g_readStore.data(), &g_readQ);
    g_results = xQueueCreateStatic(kDriveLinkQueueDepth, sizeof(DriveReadResult), g_resultStore.data(), &g_resultQ);
    if constexpr (kBuilt) g_uartUp = uartBegin();
    // Seeded before the task exists, so every reader finds a status.
    const DriveLinkStatus first = compose();
    xQueueOverwrite(g_status, &first);
    if (xTaskCreatePinnedToCore(&taskMain, "DriveLnk", kDriveLinkTaskStackBytes, nullptr, 2, &g_task, 0) !=
        pdPASS) {
        g_task = nullptr;
        GLOGE(kTag, "drive-link task did not start: DRV_ALM and DRV_RDY unwatched, no drive link");
        return false;
    }
    if (!kBuilt)
        GLOGI(kTag, "drive link not in this build: DRV_ALM and DRV_RDY watched, DE held low");
    else if (g_uartUp)
        GLOGI(kTag, "drive link on UART%d, TX G%d RX G%d DE G%d: probing once motor power settles",
              int(kUart), BOARD_GPIO_RS485_TX, BOARD_GPIO_RS485_RX, BOARD_GPIO_RS485_DE);
    return true;
}

DriveLinkStatus driveLinkStatus() {
    DriveLinkStatus s;
    if (g_status == nullptr || xQueuePeek(g_status, &s, 0) != pdTRUE) s.built = kBuilt;
    return s;
}

bool driveAlarmTake() { return g_alarmPending.exchange(false, std::memory_order_acq_rel); }

bool driveLinkRead(const DriveRead& r) {
    return g_reads != nullptr && xQueueSend(g_reads, &r, 0) == pdTRUE;
}

bool driveLinkTakeResult(DriveReadResult& out) {
    return g_results != nullptr && xQueueReceive(g_results, &out, 0) == pdTRUE;
}

uint32_t driveLinkStackFree() {
    // IDF reports BYTES, not the vanilla FreeRTOS words.
    return g_task != nullptr ? uint32_t(uxTaskGetStackHighWaterMark(g_task)) : 0;
}

}  // namespace valence
