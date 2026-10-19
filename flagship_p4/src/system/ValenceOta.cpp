// ValenceOta -- implementation. See ValenceOta.h for the routes, the press
// rule, the idle gate, the T31 reasoning and the buy-late contract; nothing is
// restated here.

#include "system/ValenceOta.h"

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <esp_heap_caps.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_random.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "geiger/geiger.h"
#include "hub/valence_config.h"
#include "motion/ValenceMotion.h"
#include "system/OtaArm.h"
#include "system/ValenceHttp.h"
#include "system/ValenceMotorSwitch.h"

#if NUCLEUS_BENCH_NO_MOTOR
#include <span>

#include "secrets.h"
#include "valence/core/crypto.hpp"
#endif

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

bool g_bought = false;

// Claimed at the top of POST /ota, before the gates, so the ESTOP release
// sees it from the first moment the idle verdict is relied on. Written on the
// httpd task, read on the hub task; relaxed, nothing is published under it.
std::atomic<bool> g_inFlight{false};

// The one request. Every call holds g_armLock, and nothing else runs under
// it: no logging, no allocation.
ota::ArmGate g_arm;
portMUX_TYPE g_armLock = portMUX_INITIALIZER_UNLOCKED;

uint32_t nowMs() { return uint32_t(esp_timer_get_time() / 1000); }

#if NUCLEUS_BENCH_NO_MOTOR
// Namespace scope, not a function-local static (T4).
valence::SoftwareCrypto s_cmp;

bool benchTokenOk(httpd_req_t* req) {
    char got[96] = {};
    if (httpd_req_get_hdr_value_str(req, "X-OTA-Token", got, sizeof(got)) != ESP_OK) return false;
    const char* want = SECRET_OTA_TOKEN;
    const size_t n = std::strlen(got);
    if (n != std::strlen(want)) return false;
    return s_cmp.constantTimeEqual(
        std::span<const std::byte>(reinterpret_cast<const std::byte*>(got), n),
        std::span<const std::byte>(reinterpret_cast<const std::byte*>(want), n));
}
#else
bool benchTokenOk(httpd_req_t*) { return false; }
#endif

// ?id=<hex>, 0 when absent or not hex.
uint32_t queryId(httpd_req_t* req) {
    char q[48] = {};
    char v[12] = {};
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) return 0;
    if (httpd_query_key_value(q, "id", v, sizeof(v)) != ESP_OK) return 0;
    char* end = nullptr;
    const unsigned long id = std::strtoul(v, &end, 16);
    return (end != v && *end == '\0') ? uint32_t(id) : 0;
}

const char* idleRefusalNow() {
    const MotionCensus c = motionCensus();
    return ota::idleRefusal(c.busy || c.step_q8 != 0, motorSwitchStatus().state);
}

// Every answer closes its socket: the :80 budget is three (ValenceHttp.cpp).
esp_err_t reply(httpd_req_t* req, const char* status, const char* type, const char* body) {
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    const esp_err_t rc = httpd_resp_sendstr(req, body);
    httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
    return rc;
}

esp_err_t fail(httpd_req_t* req, const char* status, const char* why) {
    GLOGW(kTag, "refused %s: %s", status, why);
    return reply(req, status, "text/plain", why);
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

esp_err_t handleArm(httpd_req_t* req) {
    if (otaInFlight()) return fail(req, "409 Conflict", "an update is already in flight");
    if (const char* why = idleRefusalNow()) return fail(req, "409 Conflict", why);
    uint32_t id = 0;
    while (id == 0) id = esp_random();
    portENTER_CRITICAL(&g_armLock);
    const bool armed = g_arm.arm(id, nowMs());
    portEXIT_CRITICAL(&g_armLock);
    if (!armed) return fail(req, "409 Conflict", "another update request is waiting for its PAIR double press");
    GLOGW(kTag, "update request %08lx: double-press PAIR within %lu s", static_cast<unsigned long>(id),
          static_cast<unsigned long>(ota::kArmWindowMs / 1000));
    char body[48];
    snprintf(body, sizeof(body), "{\"id\":\"%08lx\",\"window_s\":%lu}", static_cast<unsigned long>(id),
             static_cast<unsigned long>(ota::kArmWindowMs / 1000));
    return reply(req, "202 Accepted", "application/json", body);
}

esp_err_t handleStatus(httpd_req_t* req) {
    const uint32_t id = queryId(req);
    portENTER_CRITICAL(&g_armLock);
    const ota::ArmState st = g_arm.status(id, nowMs());
    portEXIT_CRITICAL(&g_armLock);
    char body[32];
    snprintf(body, sizeof(body), "{\"state\":\"%s\"}", ota::armStateName(st));
    return reply(req, "200 OK", "application/json", body);
}

// Every early return after the claim releases it.
esp_err_t refuseClaimed(httpd_req_t* req, const char* status, const char* why) {
    g_inFlight = false;
    return fail(req, status, why);
}

esp_err_t handlePost(httpd_req_t* req) {
    if (g_inFlight.exchange(true)) return fail(req, "409 Conflict", "an update is already in flight");
    if (const char* why = idleRefusalNow()) return refuseClaimed(req, "409 Conflict", why);
    const bool bench = benchTokenOk(req);
    if (!bench) {
        const uint32_t id = queryId(req);
        portENTER_CRITICAL(&g_armLock);
        const bool approved = g_arm.take(id, nowMs());
        portEXIT_CRITICAL(&g_armLock);
        if (!approved)
            return refuseClaimed(req, "403 Forbidden",
                                 "not approved: POST /ota/arm, then double-press PAIR on the machine");
    }

    const size_t total = size_t(req->content_len);
    if (total < kMinImageBytes) return refuseClaimed(req, "400 Bad Request", "body too short to be an app image");

    const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
    if (target == nullptr) return refuseClaimed(req, "500 Internal Server Error", "no spare OTA slot");
    if (total > target->size) return refuseClaimed(req, "400 Bad Request", "image larger than the target slot");

    auto* buf = static_cast<uint8_t*>(heap_caps_malloc(kChunkBytes, MALLOC_CAP_INTERNAL));
    if (buf == nullptr) return refuseClaimed(req, "503 Service Unavailable", "no internal buffer");

    GLOGW(kTag, "update starting (%s): %u B -> %s at 0x%06lx", bench ? "bench token" : "PAIR approved",
          unsigned(total), target->label, static_cast<unsigned long>(target->address));

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, total, &handle);
    if (err != ESP_OK) {
        heap_caps_free(buf);
        return refuseClaimed(req, "500 Internal Server Error", esp_err_to_name(err));
    }

    const int64_t t0 = esp_timer_get_time();
    size_t got = 0;
    while (got < total) {
        const size_t want = (total - got < kChunkBytes) ? (total - got) : kChunkBytes;
        const int n = httpd_req_recv(req, reinterpret_cast<char*>(buf), want);
        if (n <= 0) {
            esp_ota_abort(handle);
            heap_caps_free(buf);
            return refuseClaimed(req, "400 Bad Request", "body ended early");
        }
        // esp_ota_write validates the image magic on the FIRST call, so a
        // non-image body is rejected here rather than after a whole transfer.
        err = esp_ota_write(handle, buf, size_t(n));
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            heap_caps_free(buf);
            return refuseClaimed(req, "400 Bad Request", esp_err_to_name(err));
        }
        got += size_t(n);
    }
    heap_caps_free(buf);

    // Validates the image (header, chip id and revision, segment table,
    // SHA-256) before the slot is ever named bootable. A truncated or corrupt
    // body dies HERE.
    err = esp_ota_end(handle);
    if (err != ESP_OK) return refuseClaimed(req, "400 Bad Request", esp_err_to_name(err));
    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) return refuseClaimed(req, "500 Internal Server Error", esp_err_to_name(err));

    const double secs = double(esp_timer_get_time() - t0) / 1e6;
    GLOGW(kTag, "%u B written to %s in %.1f s (%.0f kB/s), rebooting", unsigned(got), target->label, secs,
          double(got) / 1024.0 / (secs > 0.0 ? secs : 1.0));

    char body[176];
    snprintf(body, sizeof(body),
             "ok slot=%s bytes=%u seconds=%.1f\n"
             "rebooting; the new image reverts unless its hub and WiFi come up\n",
             target->label, unsigned(got), secs);
    const esp_err_t sent = reply(req, "200 OK", "text/plain", body);
    xTaskCreate(rebootTask, "otaReboot", 3072, nullptr, 5, nullptr);
    return sent;
}

}  // namespace

bool otaBegin() {
    httpd_handle_t srv = http80();
    if (srv == nullptr) return false;
    const httpd_uri_t routes[] = {
        {"/ota/arm", HTTP_POST, handleArm, nullptr},
        {"/ota/arm", HTTP_GET, handleStatus, nullptr},
        {"/ota", HTTP_POST, handlePost, nullptr},
    };
    for (const httpd_uri_t& u : routes) {
        if (httpd_register_uri_handler(srv, &u) != ESP_OK) {
            GLOGE(kTag, "route registration failed: %s", u.uri);
            return false;
        }
    }
    GLOGI(kTag, "POST /ota/arm, GET /ota/arm, POST /ota on :80 (PAIR double press%s), running %s%s",
          NUCLEUS_BENCH_NO_MOTOR ? " or the bench token" : "", otaRunningSlot(),
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

bool otaInFlight() { return g_inFlight.load(std::memory_order_relaxed); }

bool otaPressTake() {
    portENTER_CRITICAL(&g_armLock);
    const bool took = g_arm.press(nowMs());
    portEXIT_CRITICAL(&g_armLock);
    return took;
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
