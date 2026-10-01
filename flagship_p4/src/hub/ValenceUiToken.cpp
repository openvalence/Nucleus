// ValenceUiTokenMinter -- implementation. See ValenceUiToken.h for the CORS
// rule, the port-80 rule and the spinlock placement, and UiTokenTable.h for the
// token itself; nothing is restated here.

#include "ValenceUiToken.h"

#include <cstdio>
#include <cstring>
#include <optional>

#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

#include "geiger/geiger.h"
#include "system/ValenceHttp.h"

#include "valence/core/crypto.hpp"

namespace valence {

namespace {

constexpr const char* kTag = "uitoken";

// INTERNAL RAM, BY NECESSITY, NOT TIDINESS -- see the header. There is exactly
// one minter, so a file-scope mux is the honest shape.
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// NAMESPACE SCOPE, NOT A FUNCTION-LOCAL STATIC, AND THAT IS LOAD-BEARING (T4):
// consume() calls this from INSIDE portENTER_CRITICAL, where a lazily
// constructed function-local static would take an init guard and register a
// destructor via __cxa_atexit -- either of which aborts the core in a
// no-abort context. At namespace scope it is built during static init and the
// critical section below does pure arithmetic.
valence::SoftwareCrypto s_cmp;

ValenceUiTokenMinter* g_minter = nullptr;

uint32_t nowMs() { return uint32_t(esp_timer_get_time() / 1000); }

void hexEncode(std::span<const std::byte> in, char* out) {
    static const char* kHex = "0123456789abcdef";
    size_t o = 0;
    for (std::byte b : in) {
        out[o++] = kHex[(uint8_t(b) >> 4) & 0x0F];
        out[o++] = kHex[uint8_t(b) & 0x0F];
    }
    out[o] = '\0';
}

}  // namespace

void ValenceUiTokenMinter::begin() {
    uint8_t seed[32] = {};
    esp_fill_random(seed, sizeof(seed));
    for (size_t i = 0; i < _secret.size(); ++i) _secret[i] = std::byte(seed[i]);
    std::memset(seed, 0, sizeof(seed));
    g_minter = this;
}

esp_err_t ValenceUiTokenMinter::handleGet(httpd_req_t* req) {
    // ---- DO NOT ADD CORS HEADERS BELOW THIS LINE ----------------------------
    // Their ABSENCE is the security mechanism (see the header). There is no
    // user-visible bug that adding one would fix.
    // ------------------------------------------------------------------------
    char body[128];
    const uint8_t code = (g_minter != nullptr) ? g_minter->mintJson(body, sizeof(body))
                                               : uint8_t(2);
    if (g_minter == nullptr) snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"not_ready\"}");
    if (code == 2) httpd_resp_set_status(req, "429 Too Many Requests");
    else httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_type(req, "application/json");
    // Connection: close is the POLITE half (the client stops reusing this
    // socket); httpd_sess_trigger_close below is the ENFORCING half, because
    // esp_http_server keeps a session alive regardless of what header we set.
    // Both, or a browser parks the socket on keep-alive -- see the header.
    httpd_resp_set_hdr(req, "Connection", "close");
    const esp_err_t sent = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
    return sent;
}

bool ValenceUiTokenMinter::attachRoutes() {
    httpd_handle_t srv = http80();
    if (srv == nullptr) return false;
    httpd_uri_t ut{"/uitoken", HTTP_GET, handleGet, nullptr};
    if (httpd_register_uri_handler(srv, &ut) != ESP_OK) {
        GLOGE(kTag, "route registration failed");
        return false;
    }
    GLOGI(kTag, "GET /uitoken on :80 (no CORS headers, by design; %lu ms TTL, control tier)",
          static_cast<unsigned long>(UiTokenTable::kTtlMs));
    return true;
}

uint8_t ValenceUiTokenMinter::mintJson(char* body, size_t cap) {
    const uint32_t now = nowMs();

    // ---- Pass 1 (locked, ~1 us): rate limit + claim a counter ---------------
    portENTER_CRITICAL(&s_mux);
    const std::optional<uint32_t> counter = _table.claimMint(now);
    portEXIT_CRITICAL(&s_mux);
    if (!counter) {
        snprintf(body, cap, "{\"ok\":false,\"error\":\"rate_limited\"}");
        return 2;
    }

    // ---- The HMAC runs UNLOCKED, deliberately -------------------------------
    // portENTER_CRITICAL disables interrupts and four SHA-256 compressions are
    // tens of microseconds -- rude for a job that shares nothing.
    const UiTokenTable::Token tok = UiTokenTable::derive(_secret, *counter, now);

    // ---- Pass 2 (locked, ~1 us): install ------------------------------------
    portENTER_CRITICAL(&s_mux);
    _table.install(tok, now);
    ++_minted;
    portEXIT_CRITICAL(&s_mux);

    char hex[UiTokenTable::kTokenBytes * 2 + 1];
    hexEncode(std::span<const std::byte>(tok), hex);
    // "tier" is stated so a client never has to guess what it got, and so the
    // ceiling is documented at the point of issue.
    snprintf(body, cap, "{\"ok\":true,\"token\":\"%s\",\"ttl_ms\":%lu,\"tier\":\"control\"}",
             hex, static_cast<unsigned long>(UiTokenTable::kTtlMs));
    return 0;
}

bool ValenceUiTokenMinter::consume(std::span<const std::byte> token) {
    const uint32_t now = nowMs();
    portENTER_CRITICAL(&s_mux);
    const bool hit = _table.consume(token, now, s_cmp);
    portEXIT_CRITICAL(&s_mux);
    if (hit) ++_consumed;
    return hit;
}

}  // namespace valence
