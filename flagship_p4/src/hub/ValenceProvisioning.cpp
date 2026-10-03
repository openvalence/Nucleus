// ValenceProvisioning -- the P4 station behind the provisioning desk: the join
// on its own task, the outcome handed back to the hub task, the credentials in
// NVS
// Constraints:
// - esp_wifi_* on this board are esp_hosted RPCs to the C6, ~150 ms each
//   (ValenceHub.h, hubSetLinkRssi). The join therefore runs on the Provision
//   task, never on the hub task, which only fills the request and polls.
// - THREE TASKS, EACH THROUGH ITS OWN FIELDS. The hub task writes the request
//   and reads the outcome; the Provision task reads the request and writes the
//   outcome; the event loop task writes the address and the disconnect
//   reason. Each handoff publishes data with a RELEASE store of a sequence or
//   an event bit and reads it after the matching ACQUIRE.
// - NEVER esp_wifi_get_config(): esp_hosted's host driver logs the station
//   SSID at WARN on every reply to it (rpc_utils.c, rpc_copy_wifi_sta_config).
//   The prior network is rebuilt from stationCredentials() instead, which is
//   exactly what boot used.
// - NO STRANDING (SPEC §13.9): a failed join restores the prior network
//   before its outcome is posted, and NVS holds new credentials only after
//   they joined (persist(), hub task, held while an OTA transfer runs).
// - Credentials are never logged. The Provision task wipes its copies; the
//   hub task wipes the request once the outcome is in.
// - Requires a started station (main.cpp wifi_up). Config mode, which starts
//   none, brings its own up first (bd val-9u0.14).
// See: Valence SPEC.md §13.9; ValenceProvisioning.h

#include "ValenceProvisioning.h"

#include <atomic>
#include <cstring>

#include <esp_event.h>
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

// The Provision task. It waits on a notification and spends its life in
// esp_hosted RPCs; below the hub (5) and BoardIo (3), on core 0 with the SDIO
// service it talks through. The stack is the number to re-measure: the line
// logged after each join carries its high-water mark (T21).
constexpr uint32_t kTaskStackBytes = 4096;
constexpr UBaseType_t kTaskPriority = 2;

constexpr EventBits_t kGotIp = 1u << 0;
constexpr EventBits_t kConnected = 1u << 1;

// NVS "sta" in namespace "valence": one blob, so the pair is written
// atomically. Version 1; any other magic, version or length is ignored and the
// secrets.h pair stands.
constexpr const char* kNvsNamespace = "valence";
constexpr const char* kNvsKey = "sta";
struct StoredStation {
    uint8_t magic[2];
    uint8_t version;
    uint8_t ssidLen;
    uint8_t passLen;
    uint8_t ssid[32];
    uint8_t pass[64];
};
constexpr uint8_t kMagic0 = 'S';
constexpr uint8_t kMagic1 = 'T';
constexpr uint8_t kVersion = 1;

// volatile: a dead store to memory nobody reads again may be dropped.
void wipeBytes(void* p, size_t n) {
    volatile uint8_t* b = static_cast<volatile uint8_t*>(p);
    for (size_t i = 0; i < n; ++i) b[i] = 0;
}

void copyField(std::span<uint8_t> dst, std::string_view src) {
    std::memset(dst.data(), 0, dst.size());
    std::memcpy(dst.data(), src.data(), src.size() < dst.size() ? src.size() : dst.size());
}

class BoardStation final : public IStation {
public:
    bool begin();
    bool beginJoin(std::string_view ssid, std::string_view passphrase) override;
    std::optional<JoinOutcome> pollJoin() override;
    bool persist(std::string_view ssid, std::string_view passphrase) override;

private:
    static void taskEntry(void* self) { static_cast<BoardStation*>(self)->run(); }
    static void onEvent(void* self, esp_event_base_t base, int32_t id, void* data);
    [[noreturn]] void run();
    JoinOutcome join();
    // Puts `cfg` on the station and reconnects. RPCs; Provision task only.
    static void apply(wifi_config_t& cfg);

    // hub task -> Provision task: the request, published by _reqSeq.
    std::array<char, Provisioning::kSsidMax + 1> _reqSsid{};
    std::array<char, Provisioning::kPskHexLen + 1> _reqPass{};
    std::atomic<uint32_t> _reqSeq{0};
    // Provision task -> hub task: the outcome, published by _doneSeq.
    JoinOutcome _out{};
    std::atomic<uint32_t> _doneSeq{0};
    // Hub task only.
    uint32_t _polledSeq = 0;
    // Event loop task -> Provision task, published by the event bits.
    std::atomic<uint32_t> _ip{0};
    std::atomic<uint16_t> _reason{0};

    EventGroupHandle_t _events = nullptr;
    TaskHandle_t _task = nullptr;
};

BoardStation g_station;

bool BoardStation::begin() {
    _events = xEventGroupCreate();
    if (_events == nullptr) return false;
    // Beside main.cpp's own handler, which keeps reconnecting on every drop.
    if (esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &onEvent, this) != ESP_OK ||
        esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &onEvent, this) != ESP_OK ||
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &onEvent, this) != ESP_OK)
        return false;
    return xTaskCreatePinnedToCore(taskEntry, "Provision", kTaskStackBytes, this, kTaskPriority, &_task, 0) ==
           pdPASS;
}

void BoardStation::onEvent(void* self, esp_event_base_t base, int32_t id, void* data) {
    auto* st = static_cast<BoardStation*>(self);
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const auto* d = static_cast<const wifi_event_sta_disconnected_t*>(data);
        // Our own disconnect from the prior network says nothing about the new one.
        if (d->reason != WIFI_REASON_ASSOC_LEAVE) st->_reason.store(uint16_t(d->reason), std::memory_order_release);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        xEventGroupSetBits(st->_events, kConnected);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto* e = static_cast<const ip_event_got_ip_t*>(data);
        st->_ip.store(e->ip_info.ip.addr, std::memory_order_release);
        xEventGroupSetBits(st->_events, kGotIp);
    }
}

bool BoardStation::beginJoin(std::string_view ssid, std::string_view passphrase) {
    if (_task == nullptr) return false;
    const uint32_t seq = _reqSeq.load(std::memory_order_relaxed);
    if (_doneSeq.load(std::memory_order_acquire) != seq) return false;
    copyField(std::span<uint8_t>(reinterpret_cast<uint8_t*>(_reqSsid.data()), _reqSsid.size()), ssid);
    copyField(std::span<uint8_t>(reinterpret_cast<uint8_t*>(_reqPass.data()), _reqPass.size()), passphrase);
    _reqSeq.store(seq + 1, std::memory_order_release);
    xTaskNotifyGive(_task);
    return true;
}

std::optional<JoinOutcome> BoardStation::pollJoin() {
    const uint32_t seq = _reqSeq.load(std::memory_order_relaxed);
    if (seq == _polledSeq || _doneSeq.load(std::memory_order_acquire) != seq) return std::nullopt;
    _polledSeq = seq;
    wipeBytes(_reqSsid.data(), _reqSsid.size());
    wipeBytes(_reqPass.data(), _reqPass.size());
    return _out;
}

void BoardStation::apply(wifi_config_t& cfg) {
    esp_wifi_set_config(WIFI_IF_STA, &cfg);
    esp_wifi_disconnect();
    // main.cpp's handler reconnects on the drop too; a second connect while
    // one is under way is refused harmlessly.
    esp_wifi_connect();
}

JoinOutcome BoardStation::join() {
    JoinOutcome out;
    // Built the way boot builds it (main.cpp wifi_up): zeroed, then the pair.
    wifi_config_t next{};
    std::memcpy(next.sta.ssid, _reqSsid.data(), sizeof next.sta.ssid);
    std::memcpy(next.sta.password, _reqPass.data(), sizeof next.sta.password);

    xEventGroupClearBits(_events, kGotIp | kConnected);
    _reason.store(0, std::memory_order_relaxed);
    const TickType_t t0 = xTaskGetTickCount();
    if (esp_wifi_set_config(WIFI_IF_STA, &next) != ESP_OK) {
        wipeBytes(&next, sizeof next);
        out.why = JoinFailure::station_down;
        return out;
    }
    esp_wifi_disconnect();
    esp_wifi_connect();

    const TickType_t limit = pdMS_TO_TICKS(limits::provision_join_timeout_ms);
    bool associated = false;
    for (;;) {
        const TickType_t spent = xTaskGetTickCount() - t0;
        if (spent >= limit) break;
        const EventBits_t bits = xEventGroupWaitBits(_events, kGotIp | kConnected, pdTRUE, pdFALSE, limit - spent);
        if (bits & kConnected) associated = true;
        if (!(bits & kGotIp)) continue;
        // An address from a reconnect to the PRIOR network can race the
        // switch; only the requested SSID counts.
        wifi_ap_record_t ap{};
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK &&
            std::strncmp(reinterpret_cast<const char*>(ap.ssid), reinterpret_cast<const char*>(next.sta.ssid),
                         sizeof next.sta.ssid) == 0) {
            out.joined = true;
            // esp_netif keeps the address in network order; cbor_keys 47 is
            // the host value with the first octet high.
            out.ipv4 = __builtin_bswap32(_ip.load(std::memory_order_acquire));
            break;
        }
        wipeBytes(&ap, sizeof ap);
    }
    wipeBytes(&next, sizeof next);
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
    // No stranding: back onto what boot would use.
    wifi_config_t prior{};
    stationCredentials(prior.sta.ssid, prior.sta.password);
    apply(prior);
    wipeBytes(&prior, sizeof prior);
    return out;
}

void BoardStation::run() {
    uint32_t handled = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const uint32_t seq = _reqSeq.load(std::memory_order_acquire);
        if (seq == handled) continue;
        handled = seq;
        _out = join();
        _doneSeq.store(seq, std::memory_order_release);
        GLOGI(kTag, "join %s; Provision stack %lu B free of %lu", _out.joined ? "done" : "failed",
              static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr)),
              static_cast<unsigned long>(kTaskStackBytes));
    }
}

bool BoardStation::persist(std::string_view ssid, std::string_view passphrase) {
    // NVS writes wait out an OTA transfer (ValenceHub.cpp, the persist rule).
    if (otaInFlight()) return false;
    StoredStation s{};
    s.magic[0] = kMagic0;
    s.magic[1] = kMagic1;
    s.version = kVersion;
    s.ssidLen = uint8_t(ssid.size());
    s.passLen = uint8_t(passphrase.size());
    copyField(s.ssid, ssid);
    copyField(s.pass, passphrase);
    nvs_handle_t h;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, kNvsKey, &s, sizeof s);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    wipeBytes(&s, sizeof s);
    // Not retried: the station is joined either way, and a failed write only
    // means the next boot uses the prior pair.
    if (err != ESP_OK) GLOGE(kTag, "provisioned network not saved: %s", esp_err_to_name(err));
    else GLOGI(kTag, "provisioned network saved");
    return true;
}

}  // namespace

IStation& provisioningStation() { return g_station; }

bool provisioningBegin() {
    const bool ok = g_station.begin();
    if (!ok) GLOGW(kTag, "station join task not started: wifi_join answers station down");
    return ok;
}

bool stationCredentials(std::span<uint8_t, 32> ssid, std::span<uint8_t, 64> passphrase) {
    StoredStation s{};
    size_t len = sizeof s;
    nvs_handle_t h;
    bool ok = false;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &h) == ESP_OK) {
        ok = nvs_get_blob(h, kNvsKey, &s, &len) == ESP_OK && len == sizeof s && s.magic[0] == kMagic0 &&
             s.magic[1] == kMagic1 && s.version == kVersion && s.ssidLen >= 1 &&
             s.ssidLen <= sizeof s.ssid && s.passLen <= sizeof s.pass;
        nvs_close(h);
    }
    if (ok) {
        copyField(ssid, std::string_view(reinterpret_cast<const char*>(s.ssid), s.ssidLen));
        copyField(passphrase, std::string_view(reinterpret_cast<const char*>(s.pass), s.passLen));
    } else {
        copyField(ssid, SECRET_WIFI_SSID);
        copyField(passphrase, SECRET_WIFI_PASSWORD);
    }
    wipeBytes(&s, sizeof s);
    return ok;
}

}  // namespace valence
