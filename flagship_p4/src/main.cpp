// flagship_p4 -- Valence Drive bring-up on the OSSM Flagship: silicon report,
// the PARLIO and LP core quadrature emitters side by side for the scope, and
// the network up through the C6
// Constraints:
// - BENCH FIRMWARE, not the product. Pure ESP-IDF on purpose: the ULP binary
//   reaches the final link only in an IDF project (platformio.ini header).
// - PIN_A/PIN_B/PIN_SYNC are the only HP pins driven. The LP core drives
//   LPG15/LPG12 (ulp/lp_quad.c). Nothing else depends on either choice.
// - PARLIO replays a time-sampled bitmap from DMA, so edge placement quantizes
//   to kParlioClockHz. That quantization is the thing being measured -- do not
//   "fix" it by raising the clock without recording the old number first.
// - The LP core clock is MEASURED, never assumed. kLpCyclesPerEdge is exact,
//   so scope edges/s times kLpCyclesPerEdge IS the LP clock.
// - WiFi is esp_wifi_remote over esp_hosted: the esp_wifi_* calls below run on
//   the C6. Pins and bus live in sdkconfig.defaults, credentials in secrets.h.
// See: docs/flagship-board.md section 8, bd val-091

#include <cmath>
#include <cstdio>
#include <cstring>
#include <driver/parlio_tx.h>
#include <esp_chip_info.h>
#include <esp_event.h>
#include <esp_flash.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <esp_hosted.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <sdkconfig.h>
#include <ulp_lp_core.h>
#include "hub/ValenceHub.h"
#include "motion/ValenceMotion.h"
#include "secrets.h"
#include "ulp_main.h"

// ---- PARLIO emitter geometry ---------------------------------------------------

// The Stamp-P4 alternates castellated and through-hole down each edge. G17 and
// G21 are both through-hole and sit four positions apart, enough clearance for
// two probes. The LP core test uses the mirrored pair LPG15/LPG12 opposite.
static constexpr gpio_num_t PIN_A = GPIO_NUM_17;
static constexpr gpio_num_t PIN_B = GPIO_NUM_21;
// SYNC pulses high for the first samples of every buffer. It rides the SAME
// DMA payload as A and B, so it marks the loop boundary exactly: trigger the
// scope here and the wrap is always at the left edge of the window.
static constexpr gpio_num_t PIN_SYNC = GPIO_NUM_38;   // strapping pin, but only at reset

// Sample clock. 208.608 steps/mm at 950 mm/s is 198,178 transitions/s, a
// 5.05 us step; 10 MHz quantizes that to 2%, 1 MHz to 20%.
static constexpr uint32_t kParlioClockHz = 1000000;   // LOW-SPEED TEST

static constexpr double kPeakTransitionsPerSec = 4172.0;   // 20 mm/s at 208.608 steps/mm

// One full sweep: peak -> 0 -> peak, then the DMA wraps and repeats.
static constexpr uint32_t kSweepMs = 400;

// One quadrature cycle: 00 -> 01 -> 11 -> 10, Gray coded so exactly one line
// changes per transition. That property is what makes it quadrature.
static constexpr uint8_t kGray[4] = {0b00, 0b01, 0b11, 0b10};

static constexpr size_t kSamples     = size_t(uint64_t(kParlioClockHz) * kSweepMs / 1000);
static constexpr size_t kBufBytes    = kSamples * 4 / 8;   // 4 bits per sample
static constexpr size_t kSyncSamples = 50;                 // 50 us marker
static uint8_t* g_pattern = nullptr;

static parlio_tx_unit_handle_t g_tx = nullptr;
static double g_raw_total = 0, g_tgt_total = 0, g_peak = 0;   // seam bookkeeping

// ---- LP core emitter steering ---------------------------------------------------

// Cycles per edge handed to the LP core, EXACT: f_LP = scope edges/s * this.
// 4003 is PRIME on purpose: the poll loop is 5 cycles and 4000 would land every
// edge on the same phase and hide that quantization (measured, val-091 / sd-1bi.3).
static constexpr uint32_t kLpCyclesPerEdge = 4003;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

// ---- network -------------------------------------------------------------------

static volatile bool g_got_ip = false;
static char g_ip[16] = "-";

static void wifi_event(void*, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_got_ip = false; strcpy(g_ip, "-");
        esp_wifi_connect();   // dumb retry is fine for a bench image
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto* e = static_cast<ip_event_got_ip_t*>(data);
        snprintf(g_ip, sizeof g_ip, IPSTR, IP2STR(&e->ip_info.ip));
        g_got_ip = true;
    }
}

static bool wifi_up() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); err = nvs_flash_init();
    }
    if (err != ESP_OK) { printf("nvs_flash_init: %s\n", esp_err_to_name(err)); return false; }
    if ((err = esp_netif_init()) != ESP_OK)                  { printf("esp_netif_init: %s\n", esp_err_to_name(err)); return false; }
    if ((err = esp_event_loop_create_default()) != ESP_OK)   { printf("event loop: %s\n", esp_err_to_name(err)); return false; }
    esp_netif_create_default_wifi_sta();

    // esp_hosted 2.x does not self-start under esp_wifi_init: bring the SDIO
    // transport up, connect to the slave, and read its firmware version. That
    // version line is the evidence the C6 carries the hosted slave image.
    const int64_t t0 = esp_timer_get_time();
    int rc = esp_hosted_init();
    printf("esp_hosted_init: %d in %.0f ms\n", rc, (esp_timer_get_time() - t0) / 1000.0);
    if (rc != 0) return false;
    rc = esp_hosted_connect_to_slave();
    printf("esp_hosted_connect_to_slave: %d at %.0f ms\n", rc, (esp_timer_get_time() - t0) / 1000.0);
    if (rc != 0) return false;
    esp_hosted_coprocessor_fwver_t ver = {};
    rc = esp_hosted_get_coprocessor_fwversion(&ver);
    printf("C6 slave firmware: rc=%d version %u.%u.%u\n", rc, unsigned(ver.major1), unsigned(ver.minor1), unsigned(ver.patch1));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    printf("esp_wifi_init (remoted to the C6): %s at %.0f ms\n",
           esp_err_to_name(err), (esp_timer_get_time() - t0) / 1000.0);
    if (err != ESP_OK) return false;

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event, nullptr);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event, nullptr);

    wifi_config_t w = {};
    strncpy(reinterpret_cast<char*>(w.sta.ssid),     SECRET_WIFI_SSID,     sizeof w.sta.ssid - 1);
    strncpy(reinterpret_cast<char*>(w.sta.password), SECRET_WIFI_PASSWORD, sizeof w.sta.password - 1);
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK)        { printf("set_mode: %s\n", esp_err_to_name(err)); return false; }
    if ((err = esp_wifi_set_config(WIFI_IF_STA, &w)) != ESP_OK)    { printf("set_config: %s\n", esp_err_to_name(err)); return false; }
    if ((err = esp_wifi_start()) != ESP_OK)                        { printf("wifi_start: %s\n", esp_err_to_name(err)); return false; }

    // POWER SAVE OFF, and this is a LATENCY constraint, not a preference. The
    // IDF default is WIFI_PS_MIN_MODEM: the station sleeps between DTIM
    // beacons and the AP HOLDS INBOUND FRAMES until the next wake, so every
    // client-to-hub motion sample waits out a beacon interval it did not have
    // to. Both retired boards disabled it for the same reason.
    // NON-FATAL on failure -- this is an esp_hosted RPC to the C6, and a hub
    // that streams with extra latency still beats one that refuses to boot --
    // but LOUD, because a silent revert to PS_MIN_MODEM reads as a motion-path
    // problem and gets hunted there. Measured both ways on the bench; the
    // numbers live on bd val-091.11.
    if ((err = esp_wifi_set_ps(WIFI_PS_NONE)) != ESP_OK)
        printf("wifi_set_ps(NONE): %s -- inbound frames will wait for DTIM\n", esp_err_to_name(err));
    else
        printf("wifi power save: OFF (WIFI_PS_NONE)\n");
    return true;
}

// ---- PARLIO pattern -------------------------------------------------------------

// Time-sampled bitmap, LSB-first, four bits per sample (A, B, SYNC, unused).
// Step timing is a PHASE ACCUMULATOR in exact fixed point: phase advances by
// (rate / clock) each sample and a transition fires on every crossing. The
// emitted instant is never fed back into the next edge's reference, so the
// quantization error is bounded at one sample and NEVER ACCUMULATES.
static void build_pattern() {
    memset(g_pattern, 0, kBufBytes);
    const double half = double(kSamples) / 2.0;

    // THE BUFFER MUST HOLD A WHOLE NUMBER OF COMPLETE QUADRATURE CYCLES. It
    // loops, and the Gray index restarts at 0 on every wrap. If the total
    // transition count is not a multiple of 4 the sequence JUMPS at the seam,
    // an illegal both-lines-change the drive will flag or miscount. Scale the
    // peak by a fraction of a percent so the integral lands exactly.
    const double raw_total = kPeakTransitionsPerSec * double(kSamples)
                          / (2.0 * double(kParlioClockHz));
    const double tgt_total = 4.0 * std::round(raw_total / 4.0);
    const double peak      = kPeakTransitionsPerSec * (tgt_total / raw_total);
    g_raw_total = raw_total; g_tgt_total = tgt_total; g_peak = peak;

    uint64_t phase = 0;   // Q32 fraction of one transition
    uint32_t gray  = 0;

    for (size_t s = 0; s < kSamples; ++s) {
        // V-shaped velocity: PEAK at both ends, zero in the middle. The DMA
        // wrap lands at FULL SPEED where a seam is glaring; the reversal is
        // still tested, at the center of the buffer.
        const double frac = fabs(1.0 - double(s) / half);
        const double rate = peak * frac;   // transitions/s

        phase += uint64_t((rate / double(kParlioClockHz)) * 4294967296.0);
        gray  += uint32_t(phase >> 32);
        phase &= 0xFFFFFFFFull;

        uint8_t v = kGray[gray & 3];
        if (s < kSyncSamples) v |= 0b100;   // SYNC on bit 2
        const size_t bit = s * 4;
        for (int k = 0; k < 3; ++k)
            if (v & (1u << k))
                g_pattern[(bit + k) / 8] |= uint8_t(1u << ((bit + k) % 8));
    }
}

static bool start_parlio() {
    // DMA reads this, so it must be internal and DMA-capable, not PSRAM.
    g_pattern = static_cast<uint8_t*>(heap_caps_malloc(kBufBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (!g_pattern) { printf("alloc of %u bytes failed\n", unsigned(kBufBytes)); return false; }
    build_pattern();

    parlio_tx_unit_config_t cfg = {};
    cfg.clk_src               = PARLIO_CLK_SRC_DEFAULT;
    cfg.data_width            = 4;
    cfg.clk_in_gpio_num       = GPIO_NUM_NC;
    cfg.input_clk_src_freq_hz = 0;
    cfg.output_clk_freq_hz    = kParlioClockHz;
    cfg.clk_out_gpio_num      = GPIO_NUM_NC;
    cfg.valid_gpio_num        = GPIO_NUM_NC;
    cfg.trans_queue_depth     = 4;
    cfg.max_transfer_size     = kBufBytes;
    cfg.dma_burst_size        = 16;
    cfg.sample_edge           = PARLIO_SAMPLE_EDGE_POS;
    cfg.bit_pack_order        = PARLIO_BIT_PACK_ORDER_LSB;
    for (auto& g : cfg.data_gpio_nums) g = GPIO_NUM_NC;
    cfg.data_gpio_nums[0] = PIN_A;
    cfg.data_gpio_nums[1] = PIN_B;
    cfg.data_gpio_nums[2] = PIN_SYNC;

    esp_err_t err = parlio_new_tx_unit(&cfg, &g_tx);
    if (err != ESP_OK) { printf("parlio_new_tx_unit failed: %s\n", esp_err_to_name(err)); return false; }
    err = parlio_tx_unit_enable(g_tx);
    if (err != ESP_OK) { printf("parlio_tx_unit_enable failed: %s\n", esp_err_to_name(err)); return false; }

    parlio_transmit_config_t tcfg = {};
    tcfg.idle_value              = 0;
    tcfg.flags.loop_transmission = true;   // gapless, runs until disable()
    err = parlio_tx_unit_transmit(g_tx, g_pattern, kBufBytes * 8, &tcfg);
    if (err != ESP_OK) { printf("parlio_tx_unit_transmit failed: %s\n", esp_err_to_name(err)); return false; }
    return true;
}

// ---- LP core -----------------------------------------------------------------

static bool start_lp_core() {
    esp_err_t err = ulp_lp_core_load_binary(ulp_main_bin_start,
                                            size_t(ulp_main_bin_end - ulp_main_bin_start));
    if (err != ESP_OK) { printf("ulp_lp_core_load_binary failed: %s\n", esp_err_to_name(err)); return false; }

    // Steer BEFORE run so the first edge is already on cadence. The LP core
    // parks on step == 0, so ordering is a nicety, not a race.
    // THIS CONSTANT RATE IS THE PRE-MOTION LIVENESS PROOF ONLY. motionBegin()
    // takes the wheel a few seconds later and parks the emitter; from then on
    // the arbiter is the sole writer of these two words (ValenceMotion.cpp).
    ulp_g_step_q8 = kLpCyclesPerEdge << 8;
    ulp_g_dir     = 1;

    ulp_lp_core_cfg_t cfg = {};
    cfg.wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU;   // start once, never sleep
    err = ulp_lp_core_run(&cfg);
    if (err != ESP_OK) { printf("ulp_lp_core_run failed: %s\n", esp_err_to_name(err)); return false; }
    return true;
}

// ---- reporting ---------------------------------------------------------------

static void report() {
    esp_chip_info_t info{};
    esp_chip_info(&info);
    uint32_t flash_bytes = 0;
    esp_flash_get_size(nullptr, &flash_bytes);

    printf("\n=== flagship_p4: Valence Drive bench, ESP32-P4, pure ESP-IDF ===\n");
    printf("chip         : model %d, %d cores, revision %d\n", int(info.model), info.cores, info.revision);
    printf("cpu (config) : %d MHz\n", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    printf("flash        : %lu MB\n", static_cast<unsigned long>(flash_bytes / (1024u * 1024u)));
    printf("internal heap: %u free\n", unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    printf("psram        : %u total, %u free\n",
           unsigned(heap_caps_get_total_size(MALLOC_CAP_SPIRAM)),
           unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    printf("IDF          : %s\n", esp_get_idf_version());
}

static void report_parlio() {
    const double step_us = 1e6 / g_peak;
    printf("\n--- PARLIO quadrature, V-profile (wrap at FULL SPEED) ---\n");
    printf("pins         : A=GPIO%d  B=GPIO%d  SYNC=GPIO%d\n", int(PIN_A), int(PIN_B), int(PIN_SYNC));
    printf("sample clock : %lu Hz -> %.3f us quantization\n",
           static_cast<unsigned long>(kParlioClockHz), 1e6 / double(kParlioClockHz));
    printf("peak rate    : %.0f transitions/s\n", g_peak);
    printf("step at peak : %.3f us -> quantization is %.1f%% of a step\n",
           step_us, 100.0 / (step_us * double(kParlioClockHz) / 1e6));
    printf("profile      : peak -> 0 -> peak over %lu ms, wrap at PEAK\n", static_cast<unsigned long>(kSweepMs));
    printf("seam         : %.2f transitions raw -> snapped to %.0f (x4 exact)\n", g_raw_total, g_tgt_total);
    printf("               peak trimmed %.4f%% so the loop closes in phase\n",
           100.0 * (g_peak - kPeakTransitionsPerSec) / kPeakTransitionsPerSec);
    printf("buffer       : %lu samples, %lu bytes internal DMA\n",
           static_cast<unsigned long>(kSamples), static_cast<unsigned long>(kBufBytes));
}

static void report_lp() {
    printf("\n--- LP core quadrature (ulp/lp_quad.c), 40 MHz XTAL ---\n");
    printf("pins         : A=LPG15  B=LPG12   (LPG15 is also LPRXD: LP UART RX is sacrificed)\n");
    printf("cycles/edge  : %lu exact  ->  f_LP = scope edges/s x %lu\n",
           static_cast<unsigned long>(kLpCyclesPerEdge), static_cast<unsigned long>(kLpCyclesPerEdge));
    printf("LP image     : %u bytes embedded (reserve-sized; real program ~2.4 KB per size on ulp_main.elf)\n",
           unsigned(ulp_main_bin_end - ulp_main_bin_start));
}

// ---- stack high-water census ---------------------------------------------------

// T21: a high-water mark only knows the paths it has actually walked, so this
// prints ONLY a task that went DEEPER than the last line printed for it.
// Silence means "no new worst case", never "not measured", and a bench boot
// that never ran the workload is not evidence for any stack size.
// Sizes are the ones the create sites pass; app_main's comes from Kconfig.
namespace {
struct StackWatch {
    const char* name;
    uint32_t total;                   // bytes the task was created with
    uint32_t worst_free = UINT32_MAX; // deepest reported so far, bytes remaining
};
StackWatch g_stacks[] = {
    {"SlopHub",  valence::kHubTaskStackBytes},
    {"Motion",   valence::kMotionTaskStackBytes},
    {"app_main", uint32_t(CONFIG_ESP_MAIN_TASK_STACK_SIZE)},
};
void note_stack(size_t i, uint32_t free_bytes) {
    if (free_bytes == 0 || free_bytes >= g_stacks[i].worst_free) return;
    g_stacks[i].worst_free = free_bytes;
    printf("[stack] %-8s deeper: %lu B free of %lu  (%.0f%% headroom)\n",
           g_stacks[i].name,
           static_cast<unsigned long>(free_bytes),
           static_cast<unsigned long>(g_stacks[i].total),
           100.0 * double(free_bytes) / double(g_stacks[i].total));
}
}  // namespace

// ---- entry -------------------------------------------------------------------

extern "C" void app_main() {
    // USB-Serial/JTAG needs a moment to re-enumerate after flash; print into
    // the void otherwise. Same courtesy the IDF LP example extends.
    vTaskDelay(pdMS_TO_TICKS(1000));
    report();

    const bool parlio_ok = start_parlio();
    if (parlio_ok) report_parlio();
    else           printf("\n--- PARLIO emitter FAILED to start ---\n");

    const bool lp_ok = start_lp_core();
    if (lp_ok) report_lp();
    else       printf("\n--- LP core FAILED to start ---\n");

    printf("\n--- network: esp_hosted over SDIO to the C6, STA \"%s\" ---\n", SECRET_WIFI_SSID);
    const bool wifi_ok = wifi_up();
    if (!wifi_ok) printf("--- network FAILED to start ---\n");

    // The motion path. It TAKES OVER the LP emitter's steering words from
    // start_lp_core()'s constant-rate liveness value and parks it; from here
    // the arbiter is their sole writer.
    // BEFORE the hub, and that order is load-bearing: the hub's boot publish of
    // every motion STATE channel reads motionCensus().
    printf("\n--- motion path (arbiter + vmotion + LP emitter) ---\n");
    const bool motion_ok = valence::motionBegin();
    if (!motion_ok) printf("--- motion path FAILED to start ---\n");

    // The SlopSync hub. Independent of the emitters above by construction: it
    // owns its own task on core 1 and shares no peripheral with them, so a hub
    // failure must never take the bench firmware down with it. It is now the
    // ONLY source of motion intents on this board.
    printf("\n--- SlopSync hub ---\n");
    const bool hub_ok = valence::hubBegin();
    if (!hub_ok) printf("--- SlopSync hub FAILED to start ---\n");
    printf("\n");

    // Liveness line every 5 s.
    // NO f_LP ESTIMATE HERE ANY MORE. It divided the LP core's mcycle stamps by
    // wall time, which only reads as a clock while the emitter runs at ONE known
    // rate; steered by the motion path it printed 4.71 to 109 MHz for a 40.00 MHz
    // crystal. The LP clock is MEASURED and its home is
    // .claude/rules/motion-control.md; an instrument that lies is worse than one
    // that is absent.
    uint32_t n = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ++n;
        const valence::HubCensus census = valence::hubCensus();
        // The hub-status channel's RSSI, pushed from HERE and not read on the
        // hub task: the radio is on the C6, so this call is an esp_hosted RPC
        // measured at 151 ms, which on a 5 ms tick is the instrument breaking
        // the thing it measures. This loop has nothing to starve.
        if (wifi_ok && g_got_ip) {
            wifi_ap_record_t ap{};
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) valence::hubSetLinkRssi(ap.rssi);
        }
        // maxblock, not free, is the number that decides anything: a serve or a
        // DMA descriptor needs ONE contiguous block, and a fragmented heap
        // reads healthy on free right up to the allocation that fails
        // (SlopDrive-32 .claude/rules/memory-budget.md T21).
        const valence::MotionCensus mo = valence::motionCensus();
        note_stack(0, census.stackFree);
        note_stack(1, mo.stack_free);
        note_stack(2, uint32_t(uxTaskGetStackHighWaterMark(nullptr)));
        printf("[flagship_p4] %lus  int_free=%u int_max=%u  psram_free=%u psram_max=%u  "
               "parlio=%s  lp=%s  edges=%lu late=%lu catchup=%lu  wifi=%s ip=%s  "
               "hub=%s sess=%lu ws=%lu/%lu  "
               "mot=%s pos=%.3fmm steps=%+ld resid=%+ld intents=%lu/%lu stack=%lu\n",
               static_cast<unsigned long>(n * 5),
               unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
               unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
               unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
               unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)),
               g_tx ? "running" : "down",
               lp_ok ? "running" : "down",
               static_cast<unsigned long>(ulp_g_edges),
               static_cast<unsigned long>(ulp_g_late),
               static_cast<unsigned long>(ulp_g_catchup),
               wifi_ok ? (g_got_ip ? "up" : "connecting") : "down",
               g_ip,
               hub_ok ? "up" : "down",
               static_cast<unsigned long>(census.sessions),
               static_cast<unsigned long>(census.wsFrames),
               static_cast<unsigned long>(census.wsDrops),
               mo.estop ? "estop" : (mo.busy ? "moving" : (mo.homed ? "idle" : "unhomed")),
               double(mo.position_mm),
               static_cast<long>(mo.steps),
               static_cast<long>(mo.residual_steps),
               static_cast<unsigned long>(mo.intents),
               static_cast<unsigned long>(mo.rejected),
               static_cast<unsigned long>(mo.stack_free));
    }
}
