// ValenceHub -- composition root for the Valence hub on the P4
// Constraints:
// - See ValenceHub.h for the single-task, PSRAM and construction-order rules.
// - Every static_assert below pins ValenceCatalog.h's hand-mirrored defaults
//   to valence_config.h. The catalog is library-only and cannot include the
//   config header; this TU sees both, so it is where the mirror is nailed.
// - The delegate and every retained STATE publisher are ValenceDevice, which
//   is hardware-free so the host twin (sim/valencesim) runs it verbatim. What
//   stays here is what only the P4 has: NVS, PSRAM, esp_netif, the task.
// - PERSISTED STATE IS FOUR NVS BLOBS in namespace "valence", one per concern:
//   "cfg" (StoredState.h: 0x1000, 0x1030 and 0x1120-0x1122 with cfg_gen, 86 B),
//   "presets" (PatternPresetStore: the 24 slots with their generation,
//   1,735 B), "trust" (the §12.3 trust ledger, tokens included, at most
//   trust_ledger_max_bytes) and "pgest" (the power-cycle gesture counter,
//   1 B); TrustStore.h owns the last two. The device owns what the bytes
//   mean; this file owns only the byte IO. "cfg" and "presets" load BEFORE
//   the first retained push, so a subscriber's first snapshot is the stored
//   truth and never a default a later load overwrites; "trust" loads before
//   the WS port starts, so no HELLO is ever judged against an empty ledger.
//   Adoption never becomes an intent, an ECHO or a cfg_gen bump.
// - NVS WRITES RUN ON THE HUB TASK (T5: never in a transport callback), each
//   DEBOUNCED by ValenceDevice's kCfgPersistDebounceMs of quiet, and are HELD
//   while an OTA transfer is in flight (otaInFlight()). Wear arithmetic, 4 KB
//   page = 126 entries of 32 B: "cfg" costs ~5 entries a write (blob index,
//   data header, 3 data), so ~25 writes fill a page and cost one sector erase;
//   at the 2 s floor that is one erase per ~50 s, and the 100,000-cycle floor
//   is ~58 days of tuning without pause on ONE page, before NVS wear-levels
//   across its five. "presets" costs ~57 entries, ~2 writes a page, but a save
//   is a deliberate operator act, not a stream. A slider drag is one write.
//   "trust" is written only when its bytes change (grants, approvals,
//   revocations, a roster label), at most once per 2 s; "pgest" once at boot
//   (app_main, before the hub task exists) and once when uptime passes
//   pairing_gesture_max_uptime_ms (hub task).
// See: Valence SPEC.md §4.2, §6.3, §9.1, §9.3, §12.3; ValenceDevice.h,
// TrustStore.h

#include "ValenceHub.h"

#include <array>
#include <span>
#include <optional>

#include <esp_heap_caps.h>
#include <esp_netif.h>
#include <esp_random.h>
#include <esp_system.h>
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
#include "TrustStore.h"
#include "ValencePlatform.h"
#include "ValenceUiToken.h"
#include "ValenceWsPort.h"
#include "system/ValenceHttp.h"
#include "system/ValenceOta.h"
#include "valence_config.h"

namespace valence {

// ---- anti-drift guards: catalog mirror vs valence_config.h -------------------
static_assert(factory::window_min  == 0.0f,                        "catalog window_min drifted");
static_assert(factory::window_max  == DEFAULT_MAX_RAIL_MM,         "catalog window_max drifted");
static_assert(factory::jog_speed   == DEFAULT_JOG_MAX_SPEED_MM_S,  "catalog jog_speed drifted");
static_assert(factory::jog_accel   == DEFAULT_JOG_ACCEL_MM_S2,     "catalog jog_accel drifted");
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

// ---- NVS persistence -----------------------------------------------------------
// Byte IO only. What a blob holds, its version and its validation belong to
// the device (StoredState.h, PatternPresetStore); this side never parses one.

constexpr const char* kNvsNamespace  = "valence";
constexpr const char* kNvsCfgKey     = "cfg";
constexpr const char* kNvsPresetsKey = "presets";

// The larger blob sizes the one scratch both share; they are never live at
// the same time (boot loads, then hub-task writes, one after the other).
constexpr size_t kBlobScratchBytes = PatternPresetStore::kBlobBytes;
static_assert(stored::kConfigBlobBytes <= kBlobScratchBytes, "cfg blob outgrew the scratch");

// Absent covers a namespace or key never written; Failed is every other
// error, including a blob larger than the scratch. A wrong-size blob that
// fits comes back as-is: rejecting it is the decoder's job, and it does.
KeyLoad loadKey(const char* key, std::span<std::byte> scratch) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) return {};
    if (err != ESP_OK) return {KeyLoadStatus::Failed, {}};
    size_t len = scratch.size();
    err = nvs_get_blob(h, key, scratch.data(), &len);
    nvs_close(h);
    if (err == ESP_ERR_NVS_NOT_FOUND) return {};
    if (err != ESP_OK) return {KeyLoadStatus::Failed, {}};
    return {KeyLoadStatus::Loaded, scratch.first(len)};
}

// The stored bytes, or an empty span for absent, unreadable or too large.
std::span<const std::byte> loadBlob(const char* key, std::span<std::byte> scratch) {
    return loadKey(key, scratch).bytes;
}

// Hub task only (T5), except TrustStore::boot()'s counter write on app_main
// before the hub task exists. nvs_commit() blocks on the flash write; the
// debounce is what keeps that off the tick more than once per interval.
// `blob` may sit in PSRAM: esp_flash_write bounces a non-DRAM source through
// a 32 B stack buffer (esp_flash_api.c, direct_write), so the cache-off
// window never reads it.
bool saveBlob(const char* key, std::span<const std::byte> blob) {
    if (blob.empty()) {
        GLOGW(kTag, "persist %s: encode failed, nothing written", key);
        return false;
    }
    nvs_handle_t h;
    if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK) {
        GLOGW(kTag, "persist %s: nvs_open failed", key);
        return false;
    }
    // The write is TIMED because it is a flash write on the hub task: this
    // number is what says whether the debounce is enough, and an unmeasured
    // blocking call on a 5 ms tick is exactly the assumption that has cost
    // this project family a session before (memory-budget.md T27).
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = nvs_set_blob(h, key, blob.data(), blob.size());
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    const uint32_t us = uint32_t(esp_timer_get_time() - t0);
    if (err != ESP_OK) GLOGW(kTag, "persist %s failed: %s", key, esp_err_to_name(err));
    else GLOGI(kTag, "persisted %s, %u B, %lu us", key, unsigned(blob.size()),
               static_cast<unsigned long>(us));
    return err == ESP_OK;
}

// TrustStore's storage seam over the same namespace. Stateless.
class NvsKeyStore final : public IKeyStore {
public:
    KeyLoad load(const char* key, std::span<std::byte> scratch) override { return loadKey(key, scratch); }
    bool save(const char* key, std::span<const std::byte> bytes) override { return saveBlob(key, bytes); }
};

NvsKeyStore g_nvs;

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
    // Persist scratch. PSRAM is legal for it: see saveBlob().
    std::array<std::byte, kBlobScratchBytes> blobScratch{};
    EspClock clock{};
    EspRandom rng{};
    ValenceDevice device{};
    std::optional<valence::Hub> hub{};
    ValenceWsPort port{};
    ValenceUiTokenMinter minter{};
    // Two ledger-sized buffers (encode scratch and the last stored bytes);
    // hub task only after boot.
    TrustStore trust{};
};

HubBox* g_box = nullptr;
TaskHandle_t g_hubTask = nullptr;
uint32_t g_ticks = 0;
uint32_t g_lastEndpointMs = 0;
uint32_t g_endpointIpv4 = 0;
// kPersist* bits tick() reported due that no write has landed for yet. Hub
// task only.
uint8_t g_persistDue = 0;

// Hub task only. cfg_gen is read HERE, at write time, by which point
// Hub::update() has already applied this tick's bump (SPEC 4.2).
void persistDue() {
    std::span<std::byte> scratch(g_box->blobScratch);
    if (g_persistDue & kPersistConfig)
        saveBlob(kNvsCfgKey, scratch.first(g_box->device.encodeConfigBlob(scratch, g_box->hub->cfgGen())));
    if (g_persistDue & kPersistPresets)
        saveBlob(kNvsPresetsKey, scratch.first(g_box->device.encodePresetsBlob(scratch)));
    g_persistDue = 0;
}

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
// CORE 1. The LP core renders motion's edges, so HP core 1 carries no
// emitter; core 0 runs app_main and the esp_hosted SDIO
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
        // 0x1000 republish and the hub-status line. It says which debounced
        // writes are due; the writes themselves are NVS and so live here.
        // HELD, never dropped, while an update is writing the other app slot:
        // the motion gate is already down, and an NVS commit (now and then a
        // sector erase) has no business interleaving with that transfer. A
        // successful update reboots before the hold drains, so a change made
        // DURING an update does not survive it; one made before it does.
        g_persistDue |= g_box->device.tick(nowMs);
        const bool ota = otaInFlight();
        if (g_persistDue != 0 && !ota) persistDue();
        // nowMs is uptime, which is also the §12.3 gesture's clock.
        const uint8_t trust = g_box->trust.tick(g_box->hub->pairing(), g_nvs, nowMs, ota);
        if (trust & kTrustLedgerFailed)
            GLOGW_EVERY_MS(30000, kTag, "trust ledger not persisted: retrying every %lu ms",
                           static_cast<unsigned long>(kLedgerWriteMinIntervalMs));
        if (trust & kTrustGestureFailed)
            GLOGW(kTag, "pairing gesture counter not cleared: this boot may count as short");
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
    // when the retained pushes below seed the channels. A load that ran after
    // the first publish would make the first snapshot a lie any subscriber has
    // already adopted. A rejected blob is logged and the factory values stand.
    std::span<std::byte> scratch(g_box->blobScratch);
    uint16_t storedGen = 0;
    std::span<const std::byte> blob = loadBlob(kNvsCfgKey, scratch);
    const bool haveStored = !blob.empty() && g_box->device.adoptConfigBlob(blob, storedGen);
    if (!blob.empty() && !haveStored)
        GLOGW(kTag, "stored config rejected (magic/version/size/range) -- factory values stand");
    blob = loadBlob(kNvsPresetsKey, scratch);
    if (!blob.empty()) {
        if (g_box->device.adoptPresetsBlob(blob)) GLOGI(kTag, "pattern presets adopted from NVS");
        else GLOGW(kTag, "stored presets rejected (magic/version/size/name) -- store starts empty");
    }

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
    // SPEC 11.2: ESTOP opens the motor switch on this board (motionEstop()),
    // so an ESTOP is a category 0 stop that loses home.
    g_box->hub->setEstopCutsPower(true);
    g_box->hub->setHubInstanceId(loadOrMintInstanceId());
    refreshEndpoint();

    // EVERY advertised STATE gets its truthful at-rest value before the first
    // client can subscribe (ValenceDevice.cpp's file header says why).
    g_box->device.attach(*g_box->hub);

    // SPEC §12.3: the ledger before the WS port starts, so no HELLO is judged
    // against an empty one, and the boot's half of the power-cycle gesture.
    const TrustBoot tb = g_box->trust.boot(g_box->hub->pairing(), g_nvs,
                                           esp_reset_reason() == ESP_RST_POWERON);
    if (tb.ledgerRejected)
        GLOGE(kTag, "stored trust ledger unreadable or rejected: no pairings; "
                    "the power-cycle gesture opens pairing");
    else GLOGI(kTag, "trust ledger: %u paired%s", unsigned(tb.paired), tb.ledgerLoaded ? "" : " (none stored)");
    if (!tb.counterSaved) GLOGW(kTag, "pairing gesture counter not saved: this boot does not count");
    if (tb.openWindow()) {
        g_box->hub->openPresenceWindow();
        if (tb.claimable)
            GLOGW(kTag, "pairing open for %lu s: unclaimed, the first knock gets configure",
                  static_cast<unsigned long>(limits::pairing_window_default_s));
        else
            GLOGW(kTag, "pairing open for %lu s: power-cycle gesture (%u short boots)",
                  static_cast<unsigned long>(limits::pairing_window_default_s), unsigned(tb.shortBoots));
    }

    auto etag = g_box->hub->catalogEtag();
    GLOGI(kTag, "catalog: %u entries, %u B encoded (scratch %u B)",
          unsigned(g_box->catalog.count), unsigned(g_box->hub->catalogEncodedBytes()),
          unsigned(valence::Hub::catalogScratchCapacity()));
    const CatalogHeadroom room = catalogHeadroom(g_box->catalog, g_box->hub->catalogEncodedBytes());
    GLOGI(kTag, "accessory headroom: %u accessories; free %u entries, %u layout, %u schema, %u safe, %lu B",
          unsigned(room.accessories), unsigned(room.entries), unsigned(room.layout),
          unsigned(room.schema), unsigned(room.safe), static_cast<unsigned long>(room.bytes));
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