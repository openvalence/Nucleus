// ValenceProvisioning -- the P4 station: every join on its own task (boot,
// reconnect and wifi_join), the saved-network list in NVS, and what it reports
// back to the hub task's NetworkDesk
// Constraints:
// - esp_wifi_* on this board are esp_hosted RPCs to the C6, ~150 ms each
//   (ValenceHub.h, hubSetLinkRssi). Joins and scans therefore run on the
//   Station task, never on the hub task, which only fills a request, offers a
//   list and polls.
// - THIS TASK IS THE ONLY CALLER OF esp_wifi_connect(). main.cpp's event
//   handler logs and returns; a drop wakes this task, which applies the join
//   rule (RFC-116): one saved network connects at once, as before; two or
//   more scan every channel first and take the strongest visible one, a near
//   tie to the earlier entry, then the unseen ones in list order.
// - FOUR TASKS, EACH THROUGH ITS OWN FIELDS. The hub task writes the join
//   request and the offered list and reads the outcome and the reports; the
//   Station task reads the first two and writes the rest; the event loop task
//   writes the link state, the address and the disconnect reason; the
//   liveness loop writes the link RSSI. Each handoff publishes data with a
//   RELEASE store of a sequence, an atomic or an event bit and reads it after
//   the matching ACQUIRE.
// - NEVER esp_wifi_get_config(): esp_hosted's host driver logs the station
//   SSID at WARN on every reply to it (rpc_utils.c, rpc_copy_wifi_sta_config).
// - Credentials are never logged: a join names its slot, the BSSID tail and
//   the RSSI. Every config built here is wiped once handed over.
// - NO STRANDING (SPEC §13.9): a failed wifi_join stores nothing, and the
//   Station task goes back to the saved list at once.
// See: Valence SPEC.md §13.9; RFC-QUEUE.md RFC-116; ValenceProvisioning.h,
// NetworkDesk.h

#include "ValenceProvisioning.h"
#include "StationScan.h"

#include <algorithm>
#include <atomic>
#include <cstring>

#include <esp_event.h>
#include <esp_heap_caps.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <nvs.h>

#include "secrets.h"
#include "system/ValenceOta.h"

namespace valence {

namespace {

constexpr const char* kTag = detail::kProvisioningTag;

// The Station task. It waits on a notification and spends its life in
// esp_hosted RPCs; below the hub (5) and BoardIo (3), on core 0 with the SDIO
// service it talks through. The stack is the number to re-measure: the line
// logged after each join carries its high-water mark (T21).
constexpr uint32_t kTaskStackBytes = 4096;
constexpr UBaseType_t kTaskPriority = 2;

constexpr EventBits_t kGotIp = 1u << 0;
constexpr EventBits_t kConnected = 1u << 1;
constexpr EventBits_t kDropped = 1u << 2;
constexpr EventBits_t kScanDone = 1u << 3;

// One saved entry gets this long, in at most two connects, before the next.
constexpr uint32_t kAttemptMs = 12000;
constexpr int kAttemptTries = 2;
// A wifi_join retries its connect up to this many times inside
// provision_join_timeout_ms.
constexpr int kProvisionTries = 6;
constexpr uint32_t kScanMs = 8000;
// Between failed passes over the list.
constexpr uint32_t kRetryMinMs = 1000;
constexpr uint32_t kRetryMaxMs = 4000;
// Scan records kept; extra BSSIDs are dropped by the driver.
constexpr uint16_t kScanRecords = 20;

// NVS namespace "valence": "nets" is the list (SavedNetworks::encode). "sta"
// is the single network an older image stored; it is read once, migrated
// into slot 1 and erased.
constexpr const char* kNvsNamespace = "valence";
constexpr const char* kNvsNets = "nets";
constexpr const char* kNvsLegacy = "sta";
struct LegacyStation {
    uint8_t magic[2];
    uint8_t version;
    uint8_t ssidLen;
    uint8_t passLen;
    uint8_t ssid[32];
    uint8_t pass[64];
};

struct Credentials {
    std::array<char, net::kSsidMax> ssid{};
    std::array<char, net::kPskHexLen> pass{};
    uint8_t ssidLen = 0;
    uint8_t passLen = 0;
    std::string_view ssidView() const { return {ssid.data(), ssidLen}; }
    std::string_view passView() const { return {pass.data(), passLen}; }
    void assign(std::string_view s, std::string_view p) {
        net::wipeBytes(this, sizeof *this);
        std::memcpy(ssid.data(), s.data(), std::min(s.size(), ssid.size()));
        std::memcpy(pass.data(), p.data(), std::min(p.size(), pass.size()));
        ssidLen = uint8_t(std::min(s.size(), ssid.size()));
        passLen = uint8_t(std::min(p.size(), pass.size()));
    }
};

bool saveBlob(const char* key, std::span<const uint8_t> bytes, bool eraseLegacy) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, key, bytes.data(), bytes.size());
        if (err == ESP_OK && eraseLegacy) nvs_erase_key(h, kNvsLegacy);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    if (err != ESP_OK) GLOGE(kTag, "saved networks not written: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

class BoardStation final : public IStation, public INetStation {
public:
    bool begin();
    const net::SavedNetworks& bootList() const { return _mine; }
    void setLinkRssi(int8_t rssi) { _linkRssi.store(rssi, std::memory_order_relaxed); }

    // IStation (hub task)
    bool beginJoin(std::string_view ssid, std::string_view passphrase) override;
    std::optional<JoinOutcome> pollJoin() override;
    // INetStation (hub task)
    bool offerList(const net::SavedNetworks& list) override;
    uint8_t liveId() const override { return _liveId.load(std::memory_order_acquire); }
    int8_t rssiOf(uint8_t id) const override;
    bool persist(std::span<const uint8_t> blob) override;

private:
    static void taskEntry(void* self) { static_cast<BoardStation*>(self)->run(); }
    static void onEvent(void* self, esp_event_base_t base, int32_t id, void* data);
    void load();
    [[noreturn]] void run();
    void takeList();
    bool joinSaved();
    size_t scanOrder(std::span<uint8_t, net::SavedNetworks::kCapacity> order);
    // One credential pair, up to `tries` connects inside `budgetMs`.
    JoinOutcome attempt(std::string_view ssid, std::string_view pass, uint32_t budgetMs, int tries);
    bool requestPending() const { return _reqSeq.load(std::memory_order_acquire) != _handled; }
    void reportLive(uint8_t id) { _liveId.store(id, std::memory_order_release); }

    // hub task -> Station task: the join request, published by _reqSeq.
    Credentials _req{};
    std::atomic<uint32_t> _reqSeq{0};
    // Station task -> hub task: the outcome, published by _doneSeq.
    JoinOutcome _out{};
    std::atomic<uint32_t> _doneSeq{0};
    // hub task -> Station task: the list, published by _listSeq; the hub
    // writes _offered only while _listTaken == _listSeq.
    net::SavedNetworks _offered{};
    std::atomic<uint32_t> _listSeq{0};
    std::atomic<uint32_t> _listTaken{0};
    // Station task -> hub task.
    std::atomic<uint8_t> _liveId{0};
    // Four (id, rssi) byte pairs from the last scan.
    std::atomic<uint64_t> _heard{0};
    // Liveness loop -> hub task.
    std::atomic<int8_t> _linkRssi{net::kRssiUnseen};
    // Hub task only.
    uint32_t _polledSeq = 0;
    // Event loop task -> Station task.
    std::atomic<uint32_t> _ip{0};
    std::atomic<uint16_t> _reason{0};
    std::atomic<bool> _started{false};
    std::atomic<bool> _linkUp{false};
    // Station task only (and app_main before the task starts).
    net::SavedNetworks _mine{};
    Credentials _joined{};
    uint32_t _handled = 0;
    wifi_ap_record_t* _records = nullptr;   // kScanRecords, PSRAM when it can

    EventGroupHandle_t _events = nullptr;
    TaskHandle_t _task = nullptr;
};

BoardStation g_station;

void BoardStation::load() {
    std::array<uint8_t, net::SavedNetworks::kBlobBytes> blob{};
    size_t len = blob.size();
    nvs_handle_t h;
    bool haveList = false;
    LegacyStation legacy{};
    size_t legacyLen = sizeof legacy;
    bool haveLegacy = false;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &h) == ESP_OK) {
        haveList = nvs_get_blob(h, kNvsNets, blob.data(), &len) == ESP_OK && len == blob.size();
        haveLegacy = nvs_get_blob(h, kNvsLegacy, &legacy, &legacyLen) == ESP_OK && legacyLen == sizeof legacy &&
                     legacy.magic[0] == 'S' && legacy.magic[1] == 'T' && legacy.version == 1 &&
                     legacy.ssidLen >= 1 && legacy.ssidLen <= sizeof legacy.ssid && legacy.passLen <= sizeof legacy.pass;
        nvs_close(h);
    }
    if (haveList && _mine.decode(blob)) {
        GLOGI(kTag, "saved networks: %u of %u", unsigned(_mine.count()), unsigned(_mine.capacity()));
    } else {
        const char* from = haveLegacy ? "the provisioned network" : "secrets.h";
        std::optional<std::pair<std::string_view, std::string_view>> provisioned;
        if (haveLegacy)
            provisioned.emplace(std::string_view(reinterpret_cast<const char*>(legacy.ssid), legacy.ssidLen),
                                std::string_view(reinterpret_cast<const char*>(legacy.pass), legacy.passLen));
        _mine = firstBootList(provisioned, SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
        if (_mine.count() == 1) {
            _mine.encode(blob);
            const bool saved = saveBlob(kNvsNets, blob, haveLegacy);
            GLOGI(kTag, "saved networks: migrated %s into slot 1%s", from, saved ? "" : " (not yet written)");
        } else {
            GLOGW(kTag, "saved networks: none, and %s is not a valid network", from);
        }
    }
    net::wipeBytes(blob.data(), blob.size());
    net::wipeBytes(&legacy, sizeof legacy);
}

bool BoardStation::begin() {
    load();
    _events = xEventGroupCreate();
    if (_events == nullptr) return false;
    _records = static_cast<wifi_ap_record_t*>(
        heap_caps_calloc(kScanRecords, sizeof(wifi_ap_record_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (_records == nullptr)
        _records = static_cast<wifi_ap_record_t*>(calloc(kScanRecords, sizeof(wifi_ap_record_t)));
    if (_records == nullptr) return false;
    if (esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &onEvent, this) != ESP_OK ||
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &onEvent, this) != ESP_OK)
        return false;
    return xTaskCreatePinnedToCore(taskEntry, "Station", kTaskStackBytes, this, kTaskPriority, &_task, 0) ==
           pdPASS;
}

void BoardStation::onEvent(void* self, esp_event_base_t base, int32_t id, void* data) {
    auto* st = static_cast<BoardStation*>(self);
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        st->_started.store(true, std::memory_order_release);
        xTaskNotifyGive(st->_task);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const auto* d = static_cast<const wifi_event_sta_disconnected_t*>(data);
        st->_linkUp.store(false, std::memory_order_release);
        // Our own disconnect, before a switch, says nothing about a join.
        if (d->reason != WIFI_REASON_ASSOC_LEAVE) {
            st->_reason.store(uint16_t(d->reason), std::memory_order_release);
            xEventGroupSetBits(st->_events, kDropped);
        }
        xTaskNotifyGive(st->_task);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        xEventGroupSetBits(st->_events, kConnected);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_SCAN_DONE) {
        xEventGroupSetBits(st->_events, kScanDone);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto* e = static_cast<const ip_event_got_ip_t*>(data);
        st->_ip.store(e->ip_info.ip.addr, std::memory_order_release);
        st->_linkUp.store(true, std::memory_order_release);
        xEventGroupSetBits(st->_events, kGotIp);
    }
}

bool BoardStation::beginJoin(std::string_view ssid, std::string_view passphrase) {
    if (_task == nullptr) return false;
    const uint32_t seq = _reqSeq.load(std::memory_order_relaxed);
    if (_doneSeq.load(std::memory_order_acquire) != seq) return false;
    _req.assign(ssid, passphrase);
    _reqSeq.store(seq + 1, std::memory_order_release);
    xTaskNotifyGive(_task);
    return true;
}

std::optional<JoinOutcome> BoardStation::pollJoin() {
    const uint32_t seq = _reqSeq.load(std::memory_order_relaxed);
    if (seq == _polledSeq || _doneSeq.load(std::memory_order_acquire) != seq) return std::nullopt;
    _polledSeq = seq;
    net::wipeBytes(&_req, sizeof _req);
    return _out;
}

bool BoardStation::offerList(const net::SavedNetworks& list) {
    if (_task == nullptr) return true;   // no station: nothing to hand over
    const uint32_t seq = _listSeq.load(std::memory_order_relaxed);
    if (_listTaken.load(std::memory_order_acquire) != seq) return false;
    _offered = list;
    _listSeq.store(seq + 1, std::memory_order_release);
    xTaskNotifyGive(_task);
    return true;
}

int8_t BoardStation::rssiOf(uint8_t id) const {
    if (id == 0) return net::kRssiUnseen;
    if (id == liveId()) return _linkRssi.load(std::memory_order_relaxed);
    const uint64_t heard = _heard.load(std::memory_order_acquire);
    for (int i = 0; i < 4; ++i) {
        if (uint8_t(heard >> (16 * i)) == id) return int8_t(uint8_t(heard >> (16 * i + 8)));
    }
    return net::kRssiUnseen;
}

bool BoardStation::persist(std::span<const uint8_t> blob) {
    // NVS writes wait out an OTA transfer (ValenceHub.cpp, the persist rule).
    if (otaInFlight()) return false;
    // Not retried: a failed write only means the next boot has the old list.
    if (saveBlob(kNvsNets, blob, false)) GLOGI(kTag, "saved networks written");
    return true;
}

void BoardStation::takeList() {
    const uint32_t seq = _listSeq.load(std::memory_order_acquire);
    if (seq == _listTaken.load(std::memory_order_relaxed)) return;
    _mine = _offered;
    _listTaken.store(seq, std::memory_order_release);
    // The link stays on whatever it joined; its entry is found again by
    // content, since a save gives an entry a new id.
    uint8_t live = 0;
    if (_linkUp.load(std::memory_order_acquire)) {
        for (size_t i = 0; i < _mine.count(); ++i)
            if (_mine.ssid(i) == _joined.ssidView() && _mine.passphrase(i) == _joined.passView()) live = _mine.id(i);
    }
    reportLive(live);
}

size_t BoardStation::scanOrder(std::span<uint8_t, net::SavedNetworks::kCapacity> order) {
    std::array<int8_t, net::SavedNetworks::kCapacity> rssi{};
    std::array<net::Sighting, kScanRecords> heard{};
    uint16_t n = 0;
    wifi_scan_config_t sc{};
    sc.show_hidden = false;
    sc.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    xEventGroupClearBits(_events, kScanDone);
    if (esp_wifi_scan_start(&sc, false) == ESP_OK &&
        (xEventGroupWaitBits(_events, kScanDone, pdTRUE, pdFALSE, pdMS_TO_TICKS(kScanMs)) & kScanDone)) {
        n = kScanRecords;
        if (esp_wifi_scan_get_ap_records(&n, _records) != ESP_OK) n = 0;
    } else {
        esp_wifi_scan_stop();
        GLOGW(kTag, "scan did not finish; trying the list in order");
    }
    for (uint16_t i = 0; i < n; ++i) {
        const char* s = reinterpret_cast<const char*>(_records[i].ssid);
        heard[i] = {std::string_view(s, strnlen(s, sizeof _records[i].ssid)), _records[i].rssi};
    }
    const size_t k = net::joinOrder(_mine, std::span<const net::Sighting>(heard.data(), n), order, rssi,
                                    kStationRssiFloorDbm);
    uint64_t packed = 0;
    for (size_t i = 0; i < _mine.count(); ++i)
        packed |= (uint64_t(_mine.id(i)) | uint64_t(uint8_t(rssi[i])) << 8) << (16 * i);
    _heard.store(packed, std::memory_order_release);
    GLOGI(kTag, "scan: %u BSSIDs; saved slots seen %d %d %d %d dBm", unsigned(n), int(rssi[0]), int(rssi[1]),
          int(rssi[2]), int(rssi[3]));
    return k;
}

JoinOutcome BoardStation::attempt(std::string_view ssid, std::string_view pass, uint32_t budgetMs, int tries) {
    JoinOutcome out;
    wifi_config_t next{};
    std::memcpy(next.sta.ssid, ssid.data(), std::min(ssid.size(), sizeof next.sta.ssid));
    std::memcpy(next.sta.password, pass.data(), std::min(pass.size(), sizeof next.sta.password));
    stationScanPolicy(next);
    _reason.store(0, std::memory_order_relaxed);
    if (esp_wifi_set_config(WIFI_IF_STA, &next) != ESP_OK) {
        net::wipeBytes(&next, sizeof next);
        out.why = JoinFailure::station_down;
        return out;
    }
    const TickType_t t0 = xTaskGetTickCount();
    const TickType_t limit = pdMS_TO_TICKS(budgetMs);
    bool associated = false;
    for (int t = 0; t < tries && !out.joined; ++t) {
        xEventGroupClearBits(_events, kGotIp | kConnected | kDropped);
        esp_wifi_connect();
        for (;;) {
            const TickType_t spent = xTaskGetTickCount() - t0;
            if (spent >= limit) break;
            const EventBits_t bits =
                xEventGroupWaitBits(_events, kGotIp | kConnected | kDropped, pdTRUE, pdFALSE, limit - spent);
            if (bits & kConnected) associated = true;
            if (bits & kGotIp) {
                // Only the requested SSID counts.
                wifi_ap_record_t ap{};
                if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK &&
                    std::strncmp(reinterpret_cast<const char*>(ap.ssid), reinterpret_cast<const char*>(next.sta.ssid),
                                 sizeof next.sta.ssid) == 0) {
                    out.joined = true;
                    // esp_netif keeps the address in network order; cbor_keys
                    // 47 is the host value with the first octet high.
                    out.ipv4 = __builtin_bswap32(_ip.load(std::memory_order_acquire));
                    GLOGI(kTag, "via ..%02x:%02x ch %u rssi %d dBm", ap.bssid[4], ap.bssid[5], unsigned(ap.primary),
                          int(ap.rssi));
                    _linkRssi.store(ap.rssi, std::memory_order_relaxed);
                }
                net::wipeBytes(&ap, sizeof ap);
                if (out.joined) break;
            }
            if (bits & kDropped) break;
        }
        if (xTaskGetTickCount() - t0 >= limit) break;
    }
    net::wipeBytes(&next, sizeof next);
    if (out.joined) return out;
    switch (_reason.load(std::memory_order_acquire)) {
        case WIFI_REASON_AUTH_EXPIRE:
        case WIFI_REASON_MIC_FAILURE:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
            out.why = JoinFailure::auth;
            break;
        case WIFI_REASON_NO_AP_FOUND:
        case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
        case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
        case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
            out.why = JoinFailure::not_found;
            break;
        default:
            out.why = associated ? JoinFailure::no_address : JoinFailure::timeout;
            break;
    }
    // Stop a connect still under way, so the next attempt starts clean.
    esp_wifi_disconnect();
    return out;
}

bool BoardStation::joinSaved() {
    const size_t n = _mine.count();
    if (n == 0) {
        GLOGW_EVERY_MS(60000, kTag, "no saved network: provision one over USB serial");
        return false;
    }
    std::array<uint8_t, net::SavedNetworks::kCapacity> order{};
    size_t k = 1;
    if (n > 1) k = scanOrder(order);
    for (size_t j = 0; j < k; ++j) {
        if (requestPending()) return false;
        const size_t i = order[j];
        const JoinOutcome r = attempt(_mine.ssid(i), _mine.passphrase(i), kAttemptMs, kAttemptTries);
        if (r.joined) {
            _joined.assign(_mine.ssid(i), _mine.passphrase(i));
            reportLive(_mine.id(i));
            GLOGI(kTag, "joined slot %u of %u", unsigned(i + 1), unsigned(n));
            return true;
        }
        GLOGW(kTag, "slot %u of %u did not join: %s", unsigned(i + 1), unsigned(n),
              r.why == JoinFailure::auth ? "authentication failed"
              : r.why == JoinFailure::not_found ? "not found" : "timed out");
    }
    return false;
}

void BoardStation::run() {
    uint32_t backoffMs = kRetryMinMs;
    for (;;) {
        const bool idle = _linkUp.load(std::memory_order_acquire) || !_started.load(std::memory_order_acquire);
        ulTaskNotifyTake(pdTRUE, idle ? portMAX_DELAY : pdMS_TO_TICKS(backoffMs));
        takeList();
        const uint32_t seq = _reqSeq.load(std::memory_order_acquire);
        if (seq != _handled) {
            _handled = seq;
            reportLive(0);
            esp_wifi_disconnect();
            _out = attempt(_req.ssidView(), _req.passView(), limits::provision_join_timeout_ms, kProvisionTries);
            if (_out.joined) _joined.assign(_req.ssidView(), _req.passView());
            _doneSeq.store(seq, std::memory_order_release);
            GLOGI(kTag, "wifi_join %s; Station stack %lu B free of %lu", _out.joined ? "done" : "failed",
                  static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr)),
                  static_cast<unsigned long>(kTaskStackBytes));
            // Joined: the hub stores it and offers the list back, and
            // takeList() finds the entry. Failed: straight back to the list.
            if (_out.joined) continue;
        }
        if (!_started.load(std::memory_order_acquire) || _linkUp.load(std::memory_order_acquire)) continue;
        if (joinSaved()) {
            backoffMs = kRetryMinMs;
        } else {
            reportLive(0);
            backoffMs = std::min(backoffMs * 2, kRetryMaxMs);
        }
    }
}

}  // namespace

IStation& provisioningStation() { return g_station; }
INetStation& stationNets() { return g_station; }

bool stationBegin() {
    const bool ok = g_station.begin();
    if (!ok) GLOGE(kTag, "Station task not started: no joins");
    return ok;
}

const net::SavedNetworks& stationBootList() { return g_station.bootList(); }

void stationSetLinkRssi(int8_t rssi) { g_station.setLinkRssi(rssi); }

}  // namespace valence
