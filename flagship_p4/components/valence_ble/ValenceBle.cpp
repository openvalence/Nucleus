// ValenceBle -- the board half of the BLE GATT binding: the C6's controller
// over esp_hosted VHCI, the NimBLE host on the P4, the pinned service
// Constraints:
// - BRING-UP RUNS ON ITS OWN TASK: esp_hosted_bt_controller_init/enable are
//   RPCs to the C6, never run on the hub task. A C6 image without a BT
//   controller refuses them; BLE then stays off for the boot and says so.
// - NIMBLE CALLS RUN ON THE HOST TASK. The hub task posts one event to the
//   host's queue (setAdvertising()); the event applies the wanted state.
//   Notifications are the exception: ble_gatts_notify_custom() takes the
//   host lock itself and is called from the hub task (the pipe's send()).
// - ONE CONNECTION (CONFIG_BT_NIMBLE_MAX_CONNECTIONS); while it lasts the
//   hub does not advertise.
// - NO LE SECURITY YET: SPEC §12.9's SHOULD for LE Secure Connections is not
//   met; credentials cross this link in the clear (H4). Planned with the
//   clients' bonding flow.
// - The advertisement (≤ 31 B): flags, the 128-bit service UUID, the name cut
//   to fit. The scan response: the full name and the RFC-072 record
//   (company 0xFFFF little-endian + the ble_adv_flags byte).
// See: src/hub/ValenceBle.h; Valence SPEC.md §13.4; esp_hosted docs/bluetooth_design.md

#include "ValenceBle.h"

#include <esp_hosted.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <string_view>

#include "geiger/geiger.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace valence::ble {

namespace {

constexpr const char* kTag = "ble";
// Bring-up only: two RPCs and the host's init, no frame buffers.
constexpr uint32_t kUpTaskStackBytes = 4096;
constexpr uint16_t kNoConn = 0xFFFF;
// 31 B legacy advertisement: flags 3 + 128-bit UUID 18 + name header 2.
constexpr size_t kAdvNameMax = 31 - 3 - 18 - 2;

// The registry's UUID text (ble_identity) as NimBLE's little-endian bytes.
constexpr uint8_t hexNibble(char c) {
    return uint8_t(c <= '9' ? c - '0' : (c | 0x20) - 'a' + 10);
}
constexpr ble_uuid128_t uuid128(std::string_view s) {
    ble_uuid128_t u{};
    u.u.type = BLE_UUID_TYPE_128;
    size_t byte = 0;
    for (size_t i = 0; i + 1 < s.size() && byte < 16;) {
        if (s[i] == '-') {
            ++i;
            continue;
        }
        u.value[15 - byte++] = uint8_t(hexNibble(s[i]) << 4 | hexNibble(s[i + 1]));
        i += 2;
    }
    return u;
}

const ble_uuid128_t kServiceUuid = uuid128(ble_identity::service_uuid);
const ble_uuid128_t kWriteUuid = uuid128(ble_identity::write_char_uuid);
const ble_uuid128_t kNotifyUuid = uuid128(ble_identity::notify_char_uuid);

BleFramePipe g_pipe;
std::atomic<uint16_t> g_conn{kNoConn};
uint16_t g_notifyHandle = 0;
uint8_t g_ownAddrType = 0;
const char* g_name = "valence";

std::atomic<bool> g_started{false};
std::atomic<bool> g_ready{false};
std::atomic<bool> g_wantAdv{false};
std::atomic<uint8_t> g_advFlags{0};
// Host task only.
uint8_t g_appliedFlags = 0xFF;
ble_npl_event g_applyEvent;
// Host task only: the one write being copied out of its mbuf chain.
std::array<std::byte, kFrameBufferCapacity> g_writeBuf{};

class Notifier final : public IBleNotifier {
public:
    bool notify(std::span<const std::byte> value) override {
        const uint16_t conn = g_conn.load(std::memory_order_acquire);
        if (conn == kNoConn) return false;
        os_mbuf* om = ble_hs_mbuf_from_flat(value.data(), uint16_t(value.size()));
        if (om == nullptr) return false;
        return ble_gatts_notify_custom(conn, g_notifyHandle, om) == 0;   // consumes om
    }
};
Notifier g_notifier;

int onAccess(uint16_t, uint16_t, ble_gatt_access_ctxt* ctxt, void*) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    const uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0 || len > g_writeBuf.size()) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    uint16_t got = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, g_writeBuf.data(), len, &got) != 0) return BLE_ATT_ERR_UNLIKELY;
    const bool taken = g_pipe.onWrite(std::span<const std::byte>(g_writeBuf.data(), got));
    std::memset(g_writeBuf.data(), 0, got);   // a wifi_join's passphrase does not linger
    return taken ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

const ble_gatt_chr_def kChars[] = {
    {.uuid = &kWriteUuid.u, .access_cb = onAccess, .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
    {.uuid = &kNotifyUuid.u, .access_cb = onAccess, .flags = BLE_GATT_CHR_F_NOTIFY, .val_handle = &g_notifyHandle},
    {},
};
const ble_gatt_svc_def kServices[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &kServiceUuid.u, .characteristics = kChars},
    {},
};

int onGap(ble_gap_event* e, void*);

void advertise() {
    ble_hs_adv_fields adv{};
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.uuids128 = &kServiceUuid;
    adv.num_uuids128 = 1;
    adv.uuids128_is_complete = 1;
    const size_t nameLen = std::strlen(g_name);
    adv.name = reinterpret_cast<const uint8_t*>(g_name);
    adv.name_len = uint8_t(std::min(nameLen, kAdvNameMax));
    adv.name_is_complete = nameLen <= kAdvNameMax;
    int rc = ble_gap_adv_set_fields(&adv);
    if (rc != 0) {
        GLOGW(kTag, "advertisement rejected: rc=%d", rc);
        return;
    }
    ble_gap_adv_params p{};
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(g_ownAddrType, nullptr, BLE_HS_FOREVER, &p, onGap, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) GLOGW(kTag, "advertising failed to start: rc=%d", rc);
}

void setScanResponse(uint8_t flags) {
    const uint16_t company = uint16_t(ble_identity::msd_company_id);
    const uint8_t msd[3] = {uint8_t(company & 0xFF), uint8_t(company >> 8), flags};
    ble_hs_adv_fields rsp{};
    rsp.name = reinterpret_cast<const uint8_t*>(g_name);
    rsp.name_len = uint8_t(std::strlen(g_name));
    rsp.name_is_complete = 1;
    rsp.mfg_data = msd;
    rsp.mfg_data_len = sizeof(msd);
    const int rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc == 0) g_appliedFlags = flags;
    else GLOGW(kTag, "scan response rejected: rc=%d", rc);
}

// Host task: the wanted state, applied.
void apply(ble_npl_event*) {
    if (!g_ready.load(std::memory_order_acquire)) return;
    const uint16_t conn = g_conn.load(std::memory_order_acquire);
    if (!g_wantAdv.load(std::memory_order_acquire)) {
        if (ble_gap_adv_active()) ble_gap_adv_stop();
        if (conn != kNoConn) ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    const uint8_t flags = g_advFlags.load(std::memory_order_relaxed);
    if (flags != g_appliedFlags) setScanResponse(flags);
    if (conn == kNoConn && !ble_gap_adv_active()) advertise();
}

int onGap(ble_gap_event* e, void*) {
    switch (e->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (e->connect.status == 0) {
                g_pipe.onConnect();
                g_conn.store(e->connect.conn_handle, std::memory_order_release);
                GLOGI(kTag, "central connected");
            } else {
                apply(nullptr);
            }
            return 0;
        case BLE_GAP_EVENT_DISCONNECT:
            g_conn.store(kNoConn, std::memory_order_release);
            g_pipe.onDisconnect();
            GLOGI(kTag, "central gone: reason 0x%x", unsigned(e->disconnect.reason));
            apply(nullptr);
            return 0;
        case BLE_GAP_EVENT_MTU:
            g_pipe.onMtu(e->mtu.value);
            GLOGI(kTag, "ATT MTU %u", unsigned(e->mtu.value));
            return 0;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            apply(nullptr);
            return 0;
        default:
            return 0;
    }
}

void onSync() {
    if (ble_hs_util_ensure_addr(0) != 0 || ble_hs_id_infer_auto(0, &g_ownAddrType) != 0) {
        GLOGE(kTag, "no identity address: BLE stays off");
        return;
    }
    g_ready.store(true, std::memory_order_release);
    GLOGW(kTag, "BLE GATT up (SPEC 13.4), name \"%s\"", g_name);
    apply(nullptr);
}

void onReset(int reason) {
    g_ready.store(false, std::memory_order_release);
    GLOGW(kTag, "host reset by the controller: reason %d", reason);
}

void hostTask(void*) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void upTask(void*) {
    esp_err_t err = esp_hosted_bt_controller_init();
    if (err == ESP_OK) err = esp_hosted_bt_controller_enable();
    if (err != ESP_OK) {
        GLOGE(kTag, "the C6 refused its BT controller (%s): BLE stays off this boot", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }
    if ((err = nimble_port_init()) != ESP_OK) {
        GLOGE(kTag, "NimBLE host init failed (%s): BLE stays off this boot", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }
    ble_hs_cfg.sync_cb = onSync;
    ble_hs_cfg.reset_cb = onReset;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(g_name);
    int rc = ble_gatts_count_cfg(kServices);
    if (rc == 0) rc = ble_gatts_add_svcs(kServices);
    if (rc != 0) {
        GLOGE(kTag, "GATT service rejected: rc=%d: BLE stays off this boot", rc);
        vTaskDelete(nullptr);
        return;
    }
    ble_npl_event_init(&g_applyEvent, apply, nullptr);
    nimble_port_freertos_init(hostTask);
    vTaskDelete(nullptr);
}

}  // namespace

BleFramePipe& pipe() {
    g_pipe.begin(g_notifier);
    return g_pipe;
}

void bringUp(const char* hubName) {
    if (g_started.exchange(true)) return;
    g_name = hubName;
    if (xTaskCreatePinnedToCore(upTask, "BleUp", kUpTaskStackBytes, nullptr, 2, nullptr, 0) != pdPASS)
        GLOGE(kTag, "BLE bring-up task create failed: BLE stays off this boot");
}

bool ready() { return g_ready.load(std::memory_order_acquire); }

void setAdvertising(bool on, uint8_t advFlags) {
    const bool onChanged = g_wantAdv.exchange(on) != on;
    const bool flagsChanged = g_advFlags.exchange(advFlags, std::memory_order_relaxed) != advFlags;
    if ((onChanged || flagsChanged) && ready()) ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &g_applyEvent);
}

}  // namespace valence::ble
