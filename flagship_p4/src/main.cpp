// flagship_p4 -- Nucleus bring-up on the OSSM Flagship: silicon report, the
// LP core quadrature emitter, and the network up through the C6
// Constraints:
// - BENCH FIRMWARE, not the product. Pure ESP-IDF on purpose: the ULP binary
//   reaches the final link only in an IDF project (platformio.ini header).
// - Every pin is a BoardPins.h name. The LP core drives QUAD_A/QUAD_B
//   (ulp/lp_quad.c); nothing in this file drives a board net.
// - The LP core clock is MEASURED, never assumed. kLpCyclesPerEdge is exact,
//   so scope edges/s times kLpCyclesPerEdge IS the LP clock.
// - WiFi is esp_wifi_remote over esp_hosted: the esp_wifi_* calls below run on
//   the C6. Pins and bus live in sdkconfig.defaults, credentials in secrets.h.
// See: docs/flagship-board.md section 8, bd val-091

#include <cstdio>
#include <cstring>
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
#include "hub/ValenceProvisioning.h"
#include "motion/ValenceMotion.h"
#include "patterns/ValencePattern.h"
#include "system/ValenceDiag.h"
#include "system/ValenceOta.h"
#include "system/BoardPins.h"
#include "system/ValenceBoardIo.h"
#include "system/ValenceDbg.h"   // compiled here so the build checks it; called from nowhere by default
#include "system/ValenceDriveLink.h"
#include "system/ValenceFan.h"
#include "system/ValenceGlow.h"
#include "system/ValenceLogBridge.h"
#include "system/ValenceMotorSwitch.h"
#include "system/ValencePower.h"
#include "system/ValenceSelfCheck.h"
#include "secrets.h"
#include "ulp_main.h"

// ---- LP core emitter steering ---------------------------------------------------

// Cycles per edge handed to the LP core, EXACT: f_LP = scope edges/s * this.
// 4003 is PRIME on purpose: the poll loop is 5 cycles and 4000 would land every
// edge on the same phase and hide that quantization (measured, val-091 / sd-1bi.3).
static constexpr uint32_t kLpCyclesPerEdge = 4003;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

// ---- network -------------------------------------------------------------------

static volatile bool g_got_ip = false;
static bool g_hosted_ok = false;   // the C6 answered over SDIO; app_main only
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
    g_hosted_ok = (rc == 0);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    printf("esp_wifi_init (remoted to the C6): %s at %.0f ms\n",
           esp_err_to_name(err), (esp_timer_get_time() - t0) / 1000.0);
    if (err != ESP_OK) return false;

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event, nullptr);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event, nullptr);

    // A provisioned network (SPEC 13.9, written only after it joined) wins
    // over the compiled-in one, and a provisioned SSID is never printed.
    wifi_config_t w = {};
    if (valence::stationCredentials(w.sta.ssid, w.sta.password))
        printf("STA credentials: provisioned (NVS)\n");
    else
        printf("STA credentials: secrets.h, \"%s\"\n", SECRET_WIFI_SSID);
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

    printf("\n=== flagship_p4: Nucleus bench, ESP32-P4, pure ESP-IDF ===\n");
    printf("chip         : model %d, %d cores, revision %d\n", int(info.model), info.cores, info.revision);
    printf("cpu (config) : %d MHz\n", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    printf("flash        : %lu MB\n", static_cast<unsigned long>(flash_bytes / (1024u * 1024u)));
    printf("internal heap: %u free\n", unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    printf("psram        : %u total, %u free\n",
           unsigned(heap_caps_get_total_size(MALLOC_CAP_SPIRAM)),
           unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    printf("IDF          : %s\n", esp_get_idf_version());
}

static void report_lp() {
    printf("\n--- LP core quadrature (ulp/lp_quad.c), 40 MHz XTAL ---\n");
    printf("pins         : A=LPG%d  B=LPG%d (QUAD_A, QUAD_B)\n", BOARD_GPIO_QUAD_A, BOARD_GPIO_QUAD_B);
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
    {"ValenceHub",  valence::kHubTaskStackBytes},
    {"Motion",   valence::kMotionTaskStackBytes},
    {"Pattern",  valence::kPatternTaskStackBytes},
    {"app_main", uint32_t(CONFIG_ESP_MAIN_TASK_STACK_SIZE)},
    {"MotorSw",  valence::kMotorSwitchTaskStackBytes},
    {"BoardIo",  valence::kBoardIoTaskStackBytes},
    {"DriveLnk", valence::kDriveLinkTaskStackBytes},
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
    // BEFORE ANYTHING, the delay included: motor power stays off until the
    // self-check proves the board (val-091.21). The 100k pull-downs hold the
    // switch off through reset; this makes it an actively driven low.
    valence::selfCheckHoldMotorOff();
    // RS485 DE and /RE float near 1.65 V until driven (Hardware hw-3jk): DE
    // low before anything else, so the transceiver never drives the bus unasked.
    valence::driveLinkHoldIdle();

    // USB-Serial/JTAG needs a moment to re-enumerate after flash; print into
    // the void otherwise. Same courtesy the IDF LP example extends.
    vTaskDelay(pdMS_TO_TICKS(1000));
    report();

    // FIRST, so the archive holds the boot itself. It needs only PSRAM, and
    // the boot sequence is precisely the history a bench console loses.
    const bool diag_ok = valence::diagBegin();
    if (!diag_ok) printf("--- diagnostics archive FAILED to allocate ---\n");

    // The motor current monitor, before motion: its latched ALERT can hold the
    // motor switch open. Non-fatal; a bare stamp has no part fitted.
    valence::powerBegin();

    // The motor switch's host, state off, BEFORE the self-check: the table's
    // motor-rail entries read its state, and the switch needs the power
    // monitor above for its ALERT re-arm and MOTOR_V+. It enables nothing
    // until the self-check's verdict below says so.
    if (!valence::motorSwitchBegin()) printf("--- motor switch host FAILED: motor power stays off ---\n");

    // The status pixel, the fan and the HOME button, early so the boot
    // rainbow covers the rest of the boot. AFTER the motor switch: THERM is
    // sampled on its ADC poll.
    if (!valence::boardIoBegin()) printf("--- BoardIo task FAILED: no status pixel, fan off ---\n");

    // DRV_ALM and DRV_RDY from here on, the drive's Modbus probe once motor
    // power settles. AFTER the motor switch: its task reads the switch.
    if (!valence::driveLinkBegin()) printf("--- drive-link task FAILED: DRV_ALM unwatched ---\n");

    const bool lp_ok = start_lp_core();
    if (lp_ok) report_lp();
    else       printf("\n--- LP core FAILED to start ---\n");

    printf("\n--- network: esp_hosted over SDIO to the C6 ---\n");
    const bool wifi_ok = wifi_up();
    if (!wifi_ok) printf("--- network FAILED to start ---\n");

    // The motion path. It TAKES OVER the LP emitter's steering words from
    // start_lp_core()'s constant-rate liveness value and parks it; from here
    // the arbiter is their sole writer.
    // BEFORE the hub, and that order is load-bearing: the hub's boot publish of
    // every motion STATE channel reads motionCensus().
    printf("\n--- motion path (arbiter + kinetic + LP emitter) ---\n");
    const bool motion_ok = valence::motionBegin();
    if (!motion_ok) printf("--- motion path FAILED to start ---\n");

    // The pattern generator: AFTER motion (it gates on motionCensus()) and
    // BEFORE the hub (the hub's boot publish pushes it its first settings).
    // Non-fatal: a machine with no generator still takes streams and moves.
    if (!valence::patternBegin()) printf("--- pattern generator FAILED to start ---\n");

    // The Valence hub. Independent of the emitters above by construction: it
    // owns its own task on core 1 and shares no peripheral with them, so a hub
    // failure must never take the bench firmware down with it. It is now the
    // ONLY source of motion intents on this board.
    printf("\n--- Valence hub ---\n");
    const bool hub_ok = valence::hubBegin();
    if (!hub_ok) printf("--- Valence hub FAILED to start ---\n");

    // The operator surface rides the :80 instance the hub's /uitoken already
    // started, so these come AFTER hubBegin(). Both are non-fatal: a machine
    // that cannot be updated or dumped still runs.
    valence::otaBegin();
    if (diag_ok) valence::diagAttachRoutes();

    // The self-check runs LAST so it can judge everything above, and after the
    // log bridge so its verdict reaches the wire (0x0008) through the hub
    // task's drain. Its verdict is the motor switch's gate: a pass runs the
    // enable sequence now, anything else holds motor power off for the boot.
    if (!valence::logBridgeBegin()) printf("--- log bridge FAILED: Geiger sink table full ---\n");
    valence::SelfCheckFacts facts;
    facts.lpEdges = ulp_g_edges;
    facts.hostLink = g_hosted_ok;
    const bool selfCheckPassed = valence::selfCheckRun(facts);
    valence::motorSwitchSetSelfCheck(selfCheckPassed);
    valence::glowSetSelfCheck(selfCheckPassed);
    printf("\n");

    // Liveness line every 5 s.
    // NO f_LP ESTIMATE HERE ANY MORE. It divided the LP core's mcycle stamps by
    // wall time, which only reads as a clock while the emitter runs at ONE known
    // rate; steered by the motion path it printed 4.71 to 109 MHz for a 40.00 MHz
    // crystal. The LP clock is MEASURED and its home is
    // .claude/rules/motion-control.md; an instrument that lies is worse than one
    // that is absent.
    uint32_t n = 0;
    uint32_t lastTicks = 0;
    bool haveTicks = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ++n;
        if (n % 12 == 0) valence::selfCheckRemind();   // once a minute
        valence::selfCheckDriveLink();
        const valence::SelfCheckSummary sc = valence::selfCheckSummary();
        const valence::HubCensus census = valence::hubCensus();
        // THE BUY (val-091.16). THREE conditions, and the third is the one that
        // is easy to leave out: hubBegin() returning true says the hub was
        // BUILT, not that its task is still alive, so an image whose hub hangs
        // after startup would otherwise buy itself out of the rollback that
        // exists for it. Requiring the tick counter to ADVANCE BETWEEN TWO
        // observations is the cheap liveness proof, and it lands one interval
        // later than the task watchdog's own window -- so a hung hub resets
        // first and the bootloader restores its pair.
        if (hub_ok && g_got_ip && haveTicks && census.ticks > lastTicks)
            valence::otaMarkAppValid();
        lastTicks = census.ticks;
        haveTicks = true;
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
        // (.claude/rules/memory-budget.md T21).
        const valence::MotionCensus mo = valence::motionCensus();
        note_stack(0, census.stackFree);
        note_stack(1, mo.stack_free);
        note_stack(2, valence::patternStackFree());
        note_stack(3, uint32_t(uxTaskGetStackHighWaterMark(nullptr)));
        note_stack(4, valence::motorSwitchStackFree());
        note_stack(5, valence::boardIoStackFree());
        note_stack(6, valence::driveLinkStackFree());
        const valence::MotorSwitchStatus msw = valence::motorSwitchStatus();
        const valence::DriveLinkStatus drv = valence::driveLinkStatus();
        const valence::FanStatus fan = valence::fanStatus();
        printf("[flagship_p4] %lus  int_free=%u int_max=%u  psram_free=%u psram_max=%u  "
               "lp=%s  edges=%lu late=%lu catchup=%lu  wifi=%s ip=%s  "
               "hub=%s sess=%lu+%lup socks=%lu/%lu ws=%lu/%lu  "
               "mot=%s pos=%.3fmm steps=%+ld resid=%+ld intents=%lu/%lu stack=%lu "
               "faults=%lu stalls=%lu backstops=%lu  selfcheck=%s:%u/%u %s  msw=%s  drv=%s alm=%u rdy=%u  "
               "therm=%.1fC fan=%.0f%%/%.0frpm\n",
               static_cast<unsigned long>(n * 5),
               unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
               unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
               unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
               unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)),
               lp_ok ? "running" : "down",
               static_cast<unsigned long>(ulp_g_edges),
               static_cast<unsigned long>(ulp_g_late),
               static_cast<unsigned long>(ulp_g_catchup),
               wifi_ok ? (g_got_ip ? "up" : "connecting") : "down",
               g_ip,
               hub_ok ? "up" : "down",
               static_cast<unsigned long>(census.sessions),
               static_cast<unsigned long>(census.parked),
               static_cast<unsigned long>(census.wsSockets),
               static_cast<unsigned long>(census.uiSockets),
               static_cast<unsigned long>(census.wsFrames),
               static_cast<unsigned long>(census.wsDrops),
               mo.estop ? "estop" : (mo.busy ? "moving" : (mo.homed ? "idle" : "unhomed")),
               double(mo.position_mm),
               static_cast<long>(mo.steps),
               static_cast<long>(mo.residual_steps),
               static_cast<unsigned long>(mo.intents),
               static_cast<unsigned long>(mo.rejected),
               static_cast<unsigned long>(mo.stack_free),
               static_cast<unsigned long>(mo.emitter_faults),
               static_cast<unsigned long>(mo.stalls),
               static_cast<unsigned long>(mo.backstops),
               sc.allowed ? "pass" : "held", unsigned(sc.failed), unsigned(sc.skipped), sc.first,
               valence::motorswitch::stateName(msw.state),
               drv.built ? valence::aim::linkStateName(drv.link.state) : "absent",
               unsigned(drv.alarm), unsigned(drv.ready),
               double(fan.celsius), double(fan.duty * 100.0f), double(fan.rpm));
    }
}
