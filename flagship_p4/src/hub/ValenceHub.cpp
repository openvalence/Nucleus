// ValenceHub -- composition root for the Valence hub on the P4
// Constraints:
// - See ValenceHub.h for the single-task, PSRAM and construction-order rules.
// - Every static_assert below pins ValenceCatalog.h's hand-mirrored defaults
//   to valence_config.h. The catalog is library-only and cannot include the
//   config header; this TU sees both, so it is where the mirror is nailed.
// - The delegate and every retained STATE publisher are ValenceDevice, which
//   is hardware-free so the host twin (sim/valencesim) runs it verbatim. What
//   stays here is what only the P4 has: NVS, PSRAM, esp_netif, the task.
// - THE 0x1000 CONFIG AND ITS cfg_gen ARE PERSISTED IN NVS (namespace
//   "valence", key "cfg"). The load happens BEFORE the first retained 0x1000
//   push, so a subscriber's first snapshot is the stored truth and never a
//   default that a later load overwrites. Adoption is a plain assignment into
//   the device: it never becomes an intent, an ECHO or a cfg_gen bump.
// - NVS WRITES RUN ON THE HUB TASK (T5: never in a transport callback) and are
//   DEBOUNCED by ValenceDevice's kCfgPersistDebounceMs of quiet. Wear
//   arithmetic: the blob is 40 B, which NVS stores as 3 of its 32 B entries; a
//   4 KB NVS page holds 126 entries, so ~42 rewrites fill a page and cost one
//   sector erase. At the debounce floor of one write per 2 s that is one erase
//   per ~84 s, and the 100,000-cycle endurance floor is then ~97 days of
//   config being changed without pause -- on ONE page, before NVS wear-levels
//   across the others. A slider drag is one write, not one per frame.
// See: Valence SPEC.md §4.2, §6.3, §9.1, §9.3; ValenceDevice.h

#include "ValenceHub.h"

#include <array>
#include <cmath>
#include <optional>

#include <esp_heap_caps.h>
#include <esp_netif.h>
#include <esp_random.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <new>
#include <nvs.h>

#include "ValenceCatalog.h"
#include "ValenceDevice.h"
#include "geiger/geiger.h"
#include "ValencePlatform.h"
#include "ValenceUiToken.h"
#include "ValenceWsPort.h"
#include "system/ValenceHttp.h"
#include "valence_config.h"

namespace valence {

// ---- anti-drift guards: catalog mirror vs valence_config.h -------------------
static_assert(factory::window_min  == 0.0f,                        "catalog window_min drifted");
static_assert(factory::window_max  == DEFAULT_MAX_RAIL_MM,         "catalog window_max drifted");
static_assert(factory::user_speed  == DEFAULT_USER_MAX_SPEED_MM_S, "catalog user_speed drifted");
static_assert(factory::user_accel  == DEFAULT_USER_ACCEL_MM_S2,    "catalog user_accel drifted");
static_assert(factory::input_speed == DEFAULT_MAX_SPEED_MM_S,      "catalog input_speed drifted");
static_assert(factory::input_accel == DEFAULT_ACCEL_MM_S2,         "catalog input_accel drifted");
static_assert(factory::input_jerk  == DEFAULT_INPUT_MAX_JERK_MM_S3,"catalog input_jerk drifted");
static_assert(factory::max_rail    == DEFAULT_MAX_RAIL_MM,         "catalog max_rail drifted");
static_assert(ceiling::speed_max   == MAX_SPEED_MM_S,              "catalog speed ceiling drifted");
static_assert(ceiling::accel_max   == MAX_ACCEL_MM_S2,             "catalog accel ceiling drifted");
static_assert(ceiling::jerk_max    == MAX_JERK_MM_S3,              "catalog jerk ceiling drifted");

namespace {

constexpr const char* kTag = "hub";

// The Valence socket. 82 on every machine in this ecosystem; /uitoken rides
// plain HTTP on 80 regardless (see ValenceUiToken.h).
constexpr uint16_t kWsPort = 82;

// ---- NVS persistence for 0x1000 and its cfg_gen ------------------------------
// One blob, one write. cfg_gen rides WITH the values because §4.2 makes it a
// property of the config content, not of the boot: a client's `precondition`
// CAS compares against it, and a generation that restarted at 1 while the
// values survived would let a stale CAS silently succeed.

constexpr const char* kNvsNamespace = "valence";
constexpr const char* kNvsCfgKey    = "cfg";
constexpr uint32_t kCfgMagic   = 0x56434647u;  // "VCFG"
constexpr uint16_t kCfgVersion = 1;            // bump when StoredConfig changes
// Quiet time after the last applied change before the write lands. See the
// wear arithmetic in the file header.

struct CfgBlob {
    uint32_t     magic;
    uint16_t     version;
    uint16_t     cfg_gen;
    StoredConfig cfg;
};
static_assert(sizeof(CfgBlob) == 40, "CfgBlob layout moved: bump kCfgVersion");

bool inRange(float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; }

// An out-of-range blob is REJECTED WHOLE, never clamped into shape. Clamping
// here would be a machine-originated config change, which §4.2 says must bump
// cfg_gen -- at boot, against a generation we are in the middle of restoring.
// Falling back to the factory defaults is the one answer that needs no bump.
bool blobValid(const CfgBlob& b) {
    const StoredConfig& c = b.cfg;
    return b.magic == kCfgMagic && b.version == kCfgVersion && b.cfg_gen != 0
        && inRange(c.window_min,  0.0f, ceiling::rail_mm)
        && inRange(c.window_max,  0.0f, ceiling::rail_mm)
        && c.window_min < c.window_max
        && inRange(c.user_speed,  ceiling::speed_min, ceiling::speed_max)
        && inRange(c.user_accel,  ceiling::accel_min, ceiling::accel_max)
        && inRange(c.input_speed, ceiling::speed_min, ceiling::speed_max)
        && inRange(c.input_accel, ceiling::accel_min, ceiling::accel_max)
        && inRange(c.input_jerk,  ceiling::jerk_min,  ceiling::jerk_max)
        && inRange(c.max_rail,    ceiling::rail_min,  ceiling::rail_mm);
}

// false leaves both outputs untouched, which means the factory defaults stand.
bool loadStoredConfig(StoredConfig& cfg, uint16_t& gen) {
    nvs_handle_t h;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &h) != ESP_OK) return false;
    CfgBlob b{};
    size_t len = sizeof(b);
    const esp_err_t err = nvs_get_blob(h, kNvsCfgKey, &b, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof(b)) return false;
    if (!blobValid(b)) {
        GLOGW(kTag, "stored config rejected (magic/version/range) -- factory defaults stand");
        return false;
    }
    cfg = b.cfg;
    gen = b.cfg_gen;
    return true;
}

// Hub task only (T5). nvs_commit() blocks on the flash write; the debounce is
// (ValenceDevice's kCfgPersistDebounceMs) is what keeps that off the tick more
// than once per debounce interval.
void saveStoredConfig(const StoredConfig& cfg, uint16_t gen) {
    nvs_handle_t h;
    if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK) {
        GLOGW(kTag, "config persist: nvs_open failed");
        return;
    }
    const CfgBlob b{kCfgMagic, kCfgVersion, gen, cfg};
    // The write is TIMED because it is a flash write on the hub task: this
    // number is what says whether the debounce is enough, and an unmeasured
    // blocking call on a 5 ms tick is exactly the assumption that has cost
    // this project family a session before (memory-budget.md T27).
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = nvs_set_blob(h, kNvsCfgKey, &b, sizeof(b));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    const uint32_t us = uint32_t(esp_timer_get_time() - t0);
    if (err != ESP_OK) GLOGW(kTag, "config persist failed: %s", esp_err_to_name(err));
    else GLOGI(kTag, "config persisted, cfg_gen=%u, %lu us on the hub task",
               unsigned(gen), static_cast<unsigned long>(us));
}

// ---- the PSRAM-resident box --------------------------------------------------
// MEMBER ORDER IS CONSTRUCTION ORDER AND IT IS LOAD-BEARING: the catalog, the
// clock, the rng and the device (the delegate) must all be final before the
// Hub is built,
// because the Hub constructor encodes the catalog and draws boot_id from the
// rng right there. The Hub sits in a std::optional so the catalog can be
// FILLED between the two -- an init-list construction could not.
// The WS port rides along in PSRAM for its RX rings (5 slots x 32 x 512 B =
// 82 KB): the producer is the httpd TASK, never an ISR, so external memory is
// legal here. Do not move ISR-reachable state here by analogy.
struct HubBox {
    valence::Catalog32 catalog{};
    EspClock clock{};
    EspRandom rng{};
    ValenceDevice device{};
    std::optional<valence::Hub> hub{};
    ValenceWsPort port{};
    ValenceUiTokenMinter minter{};
};

HubBox* g_box = nullptr;
TaskHandle_t g_hubTask = nullptr;
uint32_t g_ticks = 0;
uint32_t g_lastEndpointMs = 0;
uint32_t g_endpointIpv4 = 0;

// WELCOME keys 46/47 (RFC-046): the hub's own reachable endpoint. 0/0 omits
// both from the wire, so a client that connected before DHCP finished simply
// gets no hint; every later WELCOME-shaped message picks up the real value.
// Polled on the hub task because setEndpoint() is a Hub call and the hub is
// single-task.
void refreshEndpoint() {
    esp_netif_t* nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (nif == nullptr) return;
    esp_netif_ip_info_t ip{};
    if (esp_netif_get_ip_info(nif, &ip) != ESP_OK) return;
    if (ip.ip.addr == g_endpointIpv4) return;
    g_endpointIpv4 = ip.ip.addr;
    // esp_netif stores the address in network byte order; WELCOME key 47 is a
    // plain u32 host value, so swap once here rather than at every reader.
    g_box->hub->setEndpoint(kWsPort, __builtin_bswap32(ip.ip.addr));
}

// RFC-048: a durable cross-boot identity for WELCOME identity key 5. 0 means
// "never set" and the key is omitted, which a conforming client tolerates but
// cannot use to recognize this machine again -- so it is minted once from the
// hardware RNG and kept in NVS.
uint64_t loadOrMintInstanceId() {
    nvs_handle_t h;
    if (nvs_open("valence", NVS_READWRITE, &h) != ESP_OK) return 0;
    uint64_t id = 0;
    if (nvs_get_u64(h, "hub_iid", &id) != ESP_OK || id == 0) {
        do {
            id = (uint64_t(esp_random()) << 32) | esp_random();
        } while (id == 0);
        nvs_set_u64(h, "hub_iid", id);
        nvs_commit(h);
        GLOGI(kTag, "minted hub_instance_id %08lx%08lx",
              static_cast<unsigned long>(id >> 32), static_cast<unsigned long>(id & 0xFFFFFFFFu));
    }
    nvs_close(h);
    return id;
}

// ---- the hub task ------------------------------------------------------------
// CORE 1. The LP core owns motion and the PARLIO emitter is pure DMA, so HP
// core 1 carries nothing else; core 0 runs app_main and the esp_hosted SDIO
// service that the whole network path depends on. Keeping the hub's 5 ms tick
// and its bounded socket writes off the core that services the radio bridge is
// the same separation the S3 makes between comms and real time, drawn where
// this silicon actually puts the work.
void hubTask(void*) {
    // SUBSCRIBED TO THE TASK WATCHDOG, AND THAT IS THE OTHER HALF OF OTA
    // ROLLBACK (val-091.16). A new image is PENDING_VERIFY until the liveness
    // loop buys it, and the bootloader only reverts an unbought image on a
    // RESET -- which a hung task does not produce by itself. With
    // ESP_TASK_WDT_PANIC this loop going quiet IS that reset. Non-fatal if the
    // subscribe fails: an unwatched hub still runs.
    if (esp_task_wdt_add(nullptr) != ESP_OK)
        GLOGW(kTag, "task watchdog subscribe failed: a hung hub will not self-reset");
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
        esp_task_wdt_reset();
        ++g_ticks;
        const uint32_t nowMs = uint32_t(esp_timer_get_time() / 1000);

        // Port first: it performs the deferred attach/detach and re-arms the
        // per-tick BLOB pacing budget that update() is about to spend.
        g_box->port.loop(nowMs);
        g_box->hub->update(g_box->clock.nowUs());

        // The device half: deferred latch clear, the motion plane's STATE, the
        // 0x1000 republish and the hub-status line. It says when the debounced
        // config write is due; the write itself is NVS and so lives here.
        if (g_box->device.tick(nowMs)) saveStoredConfig(g_box->device.config(), g_box->hub->cfgGen());
        if (uint32_t(nowMs - g_lastEndpointMs) >= 1000u) {
            g_lastEndpointMs = nowMs;
            refreshEndpoint();
        }

        geiger::drainToSinks();
    }
}

}  // namespace

// ---- public surface ----------------------------------------------------------

// ValenceDevice's platform seam (ValenceDevice.h). esp_timer is the clock the
// motion engine plans against, which is what makes it the right one here.
uint64_t deviceNowUs() { return uint64_t(esp_timer_get_time()); }
uint32_t deviceFreeHeapBytes() { return uint32_t(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)); }

valence::Hub* hub() { return (g_box && g_box->hub) ? &*g_box->hub : nullptr; }

void hubSetLinkRssi(int8_t rssi) {
    if (g_box) g_box->device.setLinkRssi(rssi);
}

HubCensus hubCensus() {
    HubCensus c{};
    if (!g_box || !g_box->hub) return c;
    for (size_t i = 0;; ++i) {
        const valence::HubSession* s = g_box->hub->sessionBySlot(i);
        if (s == nullptr) break;
        if (!s->occupied()) continue;
        if (s->state == valence::HubSessionState::STALE) ++c.parked;
        else ++c.sessions;
    }
    c.ticks = g_ticks;
    c.wsFrames = g_box->port.framesRx();
    c.wsDrops = g_box->port.drops();
    c.wsSockets = uint32_t(g_box->port.openSockets());
    c.uiSockets = uint32_t(http80Sockets());
    c.stackFree = g_hubTask ? uint32_t(uxTaskGetStackHighWaterMark(g_hubTask)) : 0;
    return c;
}

bool hubBegin() {
    geiger::logBegin();

    // PSRAM, via placement-new (T2). The catalog alone is ~22 KB of pooled
    // field storage and the hub carries the session table and the retained
    // store; in .bss that is internal RAM the network stack allocates from at
    // runtime. The TASK STACK below stays internal on purpose.
    void* mem = heap_caps_malloc(sizeof(HubBox), MALLOC_CAP_SPIRAM);
    if (mem == nullptr) {
        GLOGE(kTag, "PSRAM alloc of %u bytes for the hub failed", unsigned(sizeof(HubBox)));
        geiger::drainToSinks();
        return false;
    }
    g_box = new (mem) HubBox();
    g_box->minter.begin();
    g_box->device.bindTokenGate(&g_box->minter);

    if (!buildValenceCatalog(g_box->catalog, boardFeatures())) {
        GLOGE(kTag, "catalog build overflowed a Catalog32 pool");
        geiger::drainToSinks();
        return false;
    }

    // BEFORE the Hub exists, so the device is already holding stored truth
    // when the retained 0x1000 push below seeds the channel. A load that ran
    // after the first publish would make the first snapshot a lie any
    // subscriber has already adopted.
    StoredConfig stored{};
    uint16_t storedGen = 0;
    const bool haveStored = loadStoredConfig(stored, storedGen);
    if (haveStored) g_box->device.adoptConfig(stored);

    // The arbiter's window and ceilings ARE the stored config: pushed before
    // the hub exists so the first 0x1000 snapshot and the machine agree.
    g_box->device.pushConfigToMotion();

    g_box->hub.emplace(g_box->catalog, g_box->clock, g_box->rng, g_box->device);
    // cfg_gen survives the reboot with the values it belongs to (§4.2). The
    // library exposes advance-only (bumpConfigGeneration), which is correct for
    // its RFC-011 job, so the restore walks the u16 up to the stored value; it
    // terminates by wrapping and costs one increment per step, nothing more.
    if (haveStored) {
        while (g_box->hub->cfgGen() != storedGen) g_box->hub->bumpConfigGeneration();
        GLOGI(kTag, "config adopted from NVS, cfg_gen=%u", unsigned(storedGen));
    }
    if (g_box->hub->catalogEncodedBytes() == 0) {
        GLOGE(kTag, "catalog encoded to ZERO bytes -- it did not fit the hub scratch (%u B)",
              unsigned(valence::Hub::catalogScratchCapacity()));
        geiger::drainToSinks();
        return false;
    }
    g_box->hub->setIdentity(VALENCE_PRODUCT, FIRMWARE_VERSION, VALENCE_HUB_NAME);
    g_box->hub->setHubInstanceId(loadOrMintInstanceId());
    refreshEndpoint();

    // EVERY advertised STATE gets its truthful at-rest value before the first
    // client can subscribe (ValenceDevice.cpp's file header says why).
    g_box->device.attach(*g_box->hub);

    auto etag = g_box->hub->catalogEtag();
    GLOGI(kTag, "catalog: %u entries, %u B encoded (scratch %u B)",
          unsigned(g_box->catalog.count), unsigned(g_box->hub->catalogEncodedBytes()),
          unsigned(valence::Hub::catalogScratchCapacity()));
    GLOGI(kTag, "catalog etag: %02x%02x%02x%02x%02x%02x%02x%02x",
          unsigned(etag[0]), unsigned(etag[1]), unsigned(etag[2]), unsigned(etag[3]),
          unsigned(etag[4]), unsigned(etag[5]), unsigned(etag[6]), unsigned(etag[7]));
    GLOGI(kTag, "hub box %u B in PSRAM, boot_id=%08lx, %s",
          unsigned(sizeof(HubBox)), static_cast<unsigned long>(g_box->hub->bootId()),
          FIRMWARE_VERSION);

    if (!g_box->port.begin(&*g_box->hub, kWsPort)) {
        GLOGE(kTag, "WS port failed to start on :%u", unsigned(kWsPort));
        geiger::drainToSinks();
        return false;
    }
    // Non-fatal: a hub with no /uitoken still serves every watch-tier client
    // and every paired one. Losing the mint costs the browser onramp, not the
    // machine.
    if (!g_box->minter.attachRoutes()) GLOGW(kTag, "/uitoken unavailable");

    // Stack: internal by construction (plain xTaskCreatePinnedToCore). The size
    // and the measurement that set it live on kHubTaskStackBytes in ValenceHub.h.
    if (xTaskCreatePinnedToCore(hubTask, "ValenceHub", kHubTaskStackBytes, nullptr, 5,
                                &g_hubTask, 1) != pdPASS) {
        GLOGE(kTag, "hub task create failed");
        geiger::drainToSinks();
        return false;
    }
    geiger::drainToSinks();
    return true;
}

}  // namespace valence