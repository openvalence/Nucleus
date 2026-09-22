// ValenceOta -- implementation. See ValenceOta.h for the raw-body rule, the
// one motion gate, the T31 reasoning and the buy-late contract; nothing is
// restated here.

#include "system/ValenceOta.h"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <span>

#include <esp_heap_caps.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "geiger/geiger.h"
#include "motion/ValenceMotion.h"
#include "secrets.h"
#include "system/ValenceHttp.h"
#include "valence/core/crypto.hpp"

namespace valence {

namespace {

constexpr const char* kTag = "ota";

// One flash sector. Internal RAM only, and that is a hard requirement rather
// than a preference: esp_ota_write reads this buffer with the flash cache
// disabled, and a PSRAM source would be unreachable exactly then.
constexpr size_t kChunkBytes = 4096;

// An IDF app image is a 24-byte header plus segments; anything this small is a
// truncated upload, refused BEFORE esp_ota_begin so a typo never erases a slot.
constexpr size_t kMinImageBytes = 64 * 1024;

// Namespace scope, not a function-local static (T4). Nothing here runs inside
// a critical section, but one shape across this tree is cheaper than two rules.
valence::SoftwareCrypto s_cmp;

bool g_bought = false;

// There is exactly one update at a time (a second is refused), so a flag is
// the whole of the state.
bool g_inFlight = false;

bool tokenOk(httpd_req_t* req) {
    char got[96] = {};
    if (httpd_req_get_hdr_value_str(req, "X-OTA-Token", got, sizeof(got)) != ESP_OK) return false;
    const char* want = SECRET_OTA_TOKEN;
    const size_t n = std::strlen(got);
    if (n != std::strlen(want)) return false;
    return s_cmp.constantTimeEqual(
        std::span<const std::byte>(reinterpret_cast<const std::byte*>(got), n),
        std::span<const std::byte>(reinterpret_cast<const std::byte*>(want), n));
}

esp_err_t fail(httpd_req_t* req, const char* status, const char* why) {
    GLOGW(kTag, "refused: %s", why);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "close");
    const esp_err_t rc = httpd_resp_sendstr(req, why);
    httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
    return rc;
}

// The reboot cannot happen on the httpd task: esp_restart() there cuts the
// response off mid-flight and the tool reads a transport error for a
// successful flash. One short-lived task, one delay, one restart.
void rebootTask(void*) {
    vTaskDelay(pdMS_TO_TICKS(600));
    GLOGW(kTag, "rebooting into the new slot");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

esp_err_t handlePost(httpd_req_t* req) {
    if (!tokenOk(req)) return fail(req, "401 Unauthorized", "bad or missing X-OTA-Token");
    if (g_inFlight)    return fail(req, "409 Conflict", "an update is already in flight");

    const size_t total = size_t(req->content_len);
    if (total < kMinImageBytes)
        return fail(req, "400 Bad Request", "body too short to be an app image");

    const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
    if (target == nullptr) return fail(req, "500 Internal Server Error", "no spare OTA slot");
    if (total > target->size)
        return fail(req, "400 Bad Request", "image larger than the target slot");

    auto* buf = static_cast<uint8_t*>(heap_caps_malloc(kChunkBytes, MALLOC_CAP_INTERNAL));
    if (buf == nullptr) return fail(req, "503 Service Unavailable", "no internal buffer");

    g_inFlight = true;
    // THE ONE GATE. Parks the emitter on this task before any flash write, so
    // the motion plane is already still when the first stall lands.
    motionEstop();
    GLOGW(kTag, "update starting: %u B -> %s at 0x%06lx, motion parked",
          unsigned(total), target->label, static_cast<unsigned long>(target->address));

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, total, &handle);
    if (err != ESP_OK) {
        heap_caps_free(buf);
        g_inFlight = false;
        return fail(req, "500 Internal Server Error", esp_err_to_name(err));
    }

    const int64_t t0 = esp_timer_get_time();
    size_t got = 0;
    while (got < total) {
        const size_t want = (total - got < kChunkBytes) ? (total - got) : kChunkBytes;
        const int n = httpd_req_recv(req, reinterpret_cast<char*>(buf), want);
        if (n <= 0) {
            esp_ota_abort(handle);
            heap_caps_free(buf);
            g_inFlight = false;
            return fail(req, "400 Bad Request", "body ended early");
        }
        // esp_ota_write validates the image magic on the FIRST call, so a
        // non-image body is rejected here rather than after a whole transfer.
        err = esp_ota_write(handle, buf, size_t(n));
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            heap_caps_free(buf);
            g_inFlight = false;
            return fail(req, "400 Bad Request", esp_err_to_name(err));
        }
        got += size_t(n);
    }
    heap_caps_free(buf);

    // Validates the image (header, segment table, checksum) before the slot is
    // ever named bootable. A truncated or corrupt body dies HERE.
    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        g_inFlight = false;
        return fail(req, "400 Bad Request", esp_err_to_name(err));
    }
    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        g_inFlight = false;
        return fail(req, "500 Internal Server Error", esp_err_to_name(err));
    }

    const double secs = double(esp_timer_get_time() - t0) / 1e6;
    GLOGW(kTag, "%u B written to %s in %.1f s (%.0f kB/s), rebooting",
          unsigned(got), target->label, secs,
          double(got) / 1024.0 / (secs > 0.0 ? secs : 1.0));

    char body[176];
    snprintf(body, sizeof(body),
             "ok slot=%s bytes=%u seconds=%.1f\n"
             "rebooting; the new image reverts unless its hub and WiFi come up\n",
             target->label, unsigned(got), secs);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "close");
    const esp_err_t sent = httpd_resp_sendstr(req, body);
    httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
    xTaskCreate(rebootTask, "otaReboot", 3072, nullptr, 5, nullptr);
    return sent;
}

}  // namespace

bool otaBegin() {
    httpd_handle_t srv = http80();
    if (srv == nullptr) return false;
    httpd_uri_t u{"/ota", HTTP_POST, handlePost, nullptr};
    if (httpd_register_uri_handler(srv, &u) != ESP_OK) {
        GLOGE(kTag, "route registration failed");
        return false;
    }
    GLOGI(kTag, "POST /ota on :80, running %s%s", otaRunningSlot(),
          otaPendingVerify() ? " (PENDING_VERIFY: not bought yet)" : "");
    return true;
}

void otaMarkAppValid() {
    if (g_bought) return;
    g_bought = true;
    const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) GLOGW(kTag, "image bought: %s is now the boot slot", otaRunningSlot());
    else GLOGE(kTag, "mark_app_valid failed: %s", esp_err_to_name(err));
}

const char* otaRunningSlot() {
    const esp_partition_t* p = esp_ota_get_running_partition();
    return p ? p->label : "?";
}

bool otaPendingVerify() {
    const esp_partition_t* p = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    if (p == nullptr || esp_ota_get_state_partition(p, &st) != ESP_OK) return false;
    return st == ESP_OTA_IMG_PENDING_VERIFY;
}

}  // namespace valence
