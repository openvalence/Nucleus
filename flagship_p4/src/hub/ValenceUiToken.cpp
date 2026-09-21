// ValenceUiTokenMinter -- implementation. See ValenceUiToken.h for the CORS
// rule, the port-80 rule and the spinlock placement; nothing is restated here.
//
// token = HMAC(boot secret, counter || now)[0..15] (RFC-028.3). The HMAC makes
// it unguessable; the slot table makes it single-use and expiring. Both halves
// are required -- a stateless token cannot be revoked on use.

#include "ValenceUiToken.h"

#include <cstdio>
#include <cstring>

#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

#include "geiger/geiger.h"

#include "valence/core/crypto.hpp"
#include "valence/wire/hmac_sha256.hpp"

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

size_t ValenceUiTokenMinter::openSockets() const {
    if (_srv == nullptr) return 0;
    int fds[2]{};                   // exactly cfg.max_open_sockets
    size_t n = sizeof(fds) / sizeof(fds[0]);
    if (httpd_get_client_list(_srv, &n, fds) != ESP_OK) return 0;
    return n;
}

bool ValenceUiTokenMinter::attachRoutes() {
    // SECOND httpd instance, and the split is not cosmetic: the Valence socket
    // owns 82 while clients mint over plain HTTP on 80. Two instances MUST NOT
    // share a ctrl_port -- the second one silently refuses to start.
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port = 32770;
    // TWO client sockets, and every mint closes its own (handleGet). This
    // instance also costs three fds of its own (listener, ctrl_fd, msg_fd --
    // httpd_main.c:353/400/407). The socket budget and its arithmetic have ONE
    // home (C-1): flagship_p4/sdkconfig.defaults, CONFIG_LWIP_MAX_SOCKETS.
    cfg.max_open_sockets = 2;
    cfg.max_uri_handlers = 4;
    cfg.lru_purge_enable = true;

    esp_err_t err = httpd_start(&_srv, &cfg);
    if (err != ESP_OK) {
        GLOGE(kTag, "httpd_start on :80 failed: %d", int(err));
        return false;
    }
    httpd_uri_t ut{"/uitoken", HTTP_GET, handleGet, nullptr};
    if (httpd_register_uri_handler(_srv, &ut) != ESP_OK) {
        GLOGE(kTag, "route registration failed");
        return false;
    }
    GLOGI(kTag, "GET /uitoken on :80 (no CORS headers, by design; %lu ms TTL, control tier)",
          static_cast<unsigned long>(kTtlMs));
    return true;
}

uint8_t ValenceUiTokenMinter::mintJson(char* body, size_t cap) {
    const uint32_t now = nowMs();

    // ---- Pass 1 (locked, ~1 us): rate limit + claim a counter ---------------
    // One mint per kMinIntervalMs, device-wide. A page needs exactly one token
    // per session, so this is generous for real use and flattens the
    // spray-requests-and-grab-whichever-lands pattern.
    uint32_t counter = 0;
    portENTER_CRITICAL(&s_mux);
    if (_lastMintMs != 0 && (now - _lastMintMs) < kMinIntervalMs) {
        portEXIT_CRITICAL(&s_mux);
        snprintf(body, cap, "{\"ok\":false,\"error\":\"rate_limited\"}");
        return 2;
    }
    _lastMintMs = now;
    counter = ++_counter;
    portEXIT_CRITICAL(&s_mux);

    // ---- The HMAC runs UNLOCKED, deliberately -------------------------------
    // portENTER_CRITICAL disables interrupts and four SHA-256 compressions are
    // tens of microseconds -- rude for a job that shares nothing. The claimed
    // counter already makes every mint's material unique.
    std::array<std::byte, 8> material{};
    for (size_t i = 0; i < 4; ++i) material[i] = std::byte((counter >> (8 * i)) & 0xFF);
    for (size_t i = 0; i < 4; ++i) material[4 + i] = std::byte((now >> (8 * i)) & 0xFF);
    auto mac = valence::hmacSha256(std::span<const std::byte>(_secret),
                                    std::span<const std::byte>(material));
    std::array<std::byte, kTokenBytes> tok{};
    for (size_t i = 0; i < kTokenBytes; ++i) tok[i] = mac[i];

    // ---- Pass 2 (locked, ~1 us): install ------------------------------------
    // Oldest-expiring slot loses. Four is deliberately small: this is a
    // handshake credential, not a session store.
    portENTER_CRITICAL(&s_mux);
    size_t victim = 0;
    for (size_t i = 1; i < kSlots; ++i) {
        if (_slots[i].used && !_slots[victim].used) { victim = i; continue; }
        if (_slots[i].used == _slots[victim].used && _slots[i].expiresMs < _slots[victim].expiresMs)
            victim = i;
    }
    _slots[victim].token = tok;
    _slots[victim].expiresMs = now + kTtlMs;
    _slots[victim].used = false;
    ++_minted;
    portEXIT_CRITICAL(&s_mux);

    char hex[kTokenBytes * 2 + 1];
    hexEncode(std::span<const std::byte>(tok), hex);
    // "tier" is stated so a client never has to guess what it got, and so the
    // ceiling is documented at the point of issue.
    snprintf(body, cap, "{\"ok\":true,\"token\":\"%s\",\"ttl_ms\":%lu,\"tier\":\"control\"}",
             hex, static_cast<unsigned long>(kTtlMs));
    return 0;
}

bool ValenceUiTokenMinter::consume(std::span<const std::byte> token) {
    if (token.size() != kTokenBytes) return false;
    const uint32_t now = nowMs();
    bool hit = false;
    portENTER_CRITICAL(&s_mux);
    for (auto& s : _slots) {
        if (s.used) continue;
        if (int32_t(now - s.expiresMs) >= 0) {   // wrap-safe expiry compare
            s.used = true;
            continue;
        }
        if (s_cmp.constantTimeEqual(std::span<const std::byte>(s.token), token)) {
            s.used = true;   // single-use: consumed whether or not anything follows
            hit = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    if (hit) ++_consumed;
    return hit;
}

}  // namespace valence
