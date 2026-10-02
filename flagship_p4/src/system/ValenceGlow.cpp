// ValenceGlow -- implementation. See ValenceGlow.h for ownership, StatusLook.h
// for what each machine state shows.
// Constraints:
// - BoardIo task only, except glowSetSelfCheck(). No lock: the engine, the
//   output and every g_ variable below without std::atomic are that task's.
// - BSS: the engine (two 16-pixel frames, ~0.2 KB), the 4-byte frame buffer
//   and the RMT handles. The RMT driver allocates its channel object from the
//   internal heap once, at glowBegin().
// - The frame buffer is rewritten only after rmt_tx_wait_all_done() says the
//   last frame left: the encoder reads it during the transmission.

#include "system/ValenceGlow.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/rmt_tx.h>

#include "flux/flux_core.hpp"
#include "geiger/geiger.h"
#include "hub/ValenceHub.h"
#include "motion/ValenceMotion.h"
#include "system/BoardPins.h"
#include "system/StatusLook.h"
#include "system/ValenceMotorSwitch.h"
#include "system/ValenceOta.h"

namespace valence {

namespace {

constexpr const char* kTag = "glow";

// ---- THE KNOBS --------------------------------------------------------------

// Brightness ceiling, 0..255, applied after the effect and BEFORE the
// driver's gamma: 96 lands near 12 % duty on a full channel. The 3528 at
// full is a flashlight at arm's length.
constexpr uint8_t kBrightness = 96;
// Frame pacing, and how often the machine's facts are re-read.
constexpr uint32_t kFrameMs = 20;
constexpr uint32_t kFactsMs = 100;
// The hub heartbeat: sampled this often, stale after this long without a
// new tick (the hub ticks every 5 ms).
constexpr uint32_t kBeatSampleMs = 250;
constexpr uint16_t kHubStaleMs = 1000;

// WS2812-class bit timing in 100 ns ticks: 0 = 0.3 us high + 0.9 us low,
// 1 = 0.9 us high + 0.3 us low. Reset is the idle-low gap between frames.
// TODO(val-091.66): scope against the XL-3528RGBW-2812B datasheet.
constexpr uint32_t kRmtHz = 10'000'000;
constexpr uint16_t kT0High = 3;
constexpr uint16_t kT0Low = 9;
constexpr uint16_t kT1High = 9;
constexpr uint16_t kT1Low = 3;

// ---- the pixel --------------------------------------------------------------

// One GRBW pixel on an RMT channel. White comes from the W die: the gray part
// of the color moves there, so white is one LED, not three.
class RmtGrbwOutput final : public flux::IGlowOutput {
public:
    bool begin() {
        rmt_tx_channel_config_t ch{};
        ch.gpio_num = static_cast<gpio_num_t>(BOARD_GPIO_LED_DATA);
        ch.clk_src = RMT_CLK_SRC_DEFAULT;
        ch.resolution_hz = kRmtHz;
        ch.mem_block_symbols = 48;   // one block; a frame is 32 symbols
        ch.trans_queue_depth = 1;
        if (rmt_new_tx_channel(&ch, &_chan) != ESP_OK) return false;

        rmt_bytes_encoder_config_t enc{};
        enc.bit0.duration0 = kT0High;
        enc.bit0.level0 = 1;
        enc.bit0.duration1 = kT0Low;
        enc.bit0.level1 = 0;
        enc.bit1.duration0 = kT1High;
        enc.bit1.level0 = 1;
        enc.bit1.duration1 = kT1Low;
        enc.bit1.level1 = 0;
        enc.flags.msb_first = 1;
        if (rmt_new_bytes_encoder(&enc, &_enc) != ESP_OK) return false;
        if (rmt_enable(_chan) != ESP_OK) return false;
        _ok = true;
        return true;
    }

    size_t pixelCount() const override { return 1; }
    void set(size_t, flux::Rgb c) override { _pending = c; }

    // Every frame is sent, changed or not: a pixel knocked to a wrong color
    // by a transient is repainted within one frame.
    void show() override {
        if (!_ok) return;
        // A frame still on the wire keeps its buffer; this one waits a pass.
        if (rmt_tx_wait_all_done(_chan, 0) != ESP_OK) return;
        uint8_t r = flux::gamma8(_pending.r);
        uint8_t g = flux::gamma8(_pending.g);
        uint8_t b = flux::gamma8(_pending.b);
        const uint8_t w = std::min({r, g, b});
        r = uint8_t(r - w);
        g = uint8_t(g - w);
        b = uint8_t(b - w);
        _frame = {g, r, b, w};
        rmt_transmit_config_t tx{};
        rmt_transmit(_chan, _enc, _frame.data(), _frame.size(), &tx);
    }

private:
    rmt_channel_handle_t _chan = nullptr;
    rmt_encoder_handle_t _enc = nullptr;
    std::array<uint8_t, 4> _frame{};
    flux::Rgb _pending{};
    bool _ok = false;
};

RmtGrbwOutput g_pixel;
flux::GlowEngine g_engine(g_pixel);

flux::HeartbeatSource* g_hubBeat = nullptr;
uint32_t g_lastTicks = 0;
uint32_t g_lastBeatMs = 0;
uint32_t g_lastFactsMs = 0;
uint32_t g_lastFrameMs = 0;
look::MachineState g_state = look::MachineState::boot;

std::atomic<bool> g_checked{false};
std::atomic<bool> g_passed{false};

look::Facts gatherFacts() {
    const MotionCensus mo = motionCensus();
    look::Facts f;
    f.checked = g_checked.load(std::memory_order_acquire);
    f.check_passed = g_passed.load(std::memory_order_relaxed);
    f.switch_faulted = motorSwitchStatus().state == motorswitch::State::faulted;
    f.estop = mo.estop;
    f.paused = mo.paused;
    f.busy = mo.busy;
    f.homed = mo.homed;
    f.flashing = otaInFlight();
    return f;
}

// The glue owns Motion, Safety and Flash: all three back to Nominal, then the
// row's pair. The engine's arbiter does the rest.
void show(look::MachineState s) {
    g_engine.set(flux::System::Motion, flux::Status::Nominal);
    g_engine.set(flux::System::Safety, flux::Status::Nominal);
    g_engine.set(flux::System::Flash, flux::Status::Nominal);
    if (s == look::MachineState::boot) return;
    g_engine.markReady(flux::System::Motion);
    const look::LookRow& row = look::lookFor(s);
    g_engine.set(row.system, row.status);
}

constexpr const char* stateName(look::MachineState s) {
    switch (s) {
        case look::MachineState::boot:     return "boot";
        case look::MachineState::idle:     return "idle";
        case look::MachineState::homed:    return "homed";
        case look::MachineState::running:  return "running";
        case look::MachineState::paused:   return "paused";
        case look::MachineState::estop:    return "estop";
        case look::MachineState::fault:    return "fault";
        case look::MachineState::flashing: return "flashing";
        default:                           return "?";
    }
}

}  // namespace

bool glowBegin() {
    const bool ok = g_pixel.begin();
    g_engine.setBrightness(kBrightness);
    g_engine.requireReady(uint8_t(1u << uint8_t(flux::System::Motion)));
    show(look::MachineState::boot);
    if (ok) GLOGI(kTag, "status pixel on G%d: GRBW, brightness %u/255", BOARD_GPIO_LED_DATA,
                  unsigned(kBrightness));
    else    GLOGE(kTag, "status pixel RMT channel on G%d failed: the LED stays dark",
                  BOARD_GPIO_LED_DATA);
    return ok;
}

void glowService(uint32_t nowMs) {
    if (nowMs - g_lastBeatMs >= kBeatSampleMs) {
        g_lastBeatMs = nowMs;
        const uint32_t ticks = hubCensus().ticks;
        // Gated only once the hub runs: before hubBegin() the counter is
        // legitimately still, and a boot rainbow must not freeze for it.
        if (g_hubBeat == nullptr && ticks != 0) g_hubBeat = g_engine.addHeartbeat(kHubStaleMs);
        if (g_hubBeat != nullptr && ticks != g_lastTicks) g_hubBeat->pulse();
        g_lastTicks = ticks;
    }
    if (nowMs - g_lastFactsMs >= kFactsMs) {
        g_lastFactsMs = nowMs;
        const look::MachineState s = look::stateFor(gatherFacts());
        if (s != g_state) {
            GLOGD(kTag, "%s -> %s", stateName(g_state), stateName(s));
            g_state = s;
            show(s);
        }
    }
    if (nowMs - g_lastFrameMs >= kFrameMs) {
        g_lastFrameMs = nowMs;
        g_engine.update(nowMs);
    }
}

void glowSetSelfCheck(bool passed) {
    g_passed.store(passed, std::memory_order_relaxed);
    g_checked.store(true, std::memory_order_release);
}

}  // namespace valence
