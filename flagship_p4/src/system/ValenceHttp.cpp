// ValenceHttp -- implementation. See ValenceHttp.h for the one-instance rule
// and the socket budget; nothing is restated here.

#include "system/ValenceHttp.h"

#include "geiger/geiger.h"

namespace valence {

namespace {

constexpr const char* kTag = "http";

// Three at once is the worst case the routes can produce: an OTA upload and a
// /diag dump both hold a socket for their whole transfer, and a browser tab
// may be minting at the same moment. lru_purge_enable would evict the oldest
// of them, which during a flash write is the one that must not be evicted.
constexpr size_t kClientSockets = 3;

// /uitoken, /ota, /diag*, plus one spare so adding a route is not a
// head-scratching 404.
constexpr size_t kUriHandlers = 4;

httpd_handle_t g_srv = nullptr;

}  // namespace

httpd_handle_t http80() {
    if (g_srv != nullptr) return g_srv;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port = 32770;   // :82 owns 32769 (ValenceWsPort)
    cfg.max_open_sockets = kClientSockets;
    cfg.max_uri_handlers = kUriHandlers;
    cfg.lru_purge_enable = true;
    // /diag registers "/diag*" so one handler serves the tag filter too.
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    // The default 4 KB cannot hold /ota's flash bookkeeping or /diag's line
    // assembly on top of esp_http_server's own frames.
    cfg.stack_size = 8192;

    const esp_err_t err = httpd_start(&g_srv, &cfg);
    if (err != ESP_OK) {
        GLOGE(kTag, "httpd_start on :80 failed: %d", int(err));
        g_srv = nullptr;
        return nullptr;
    }
    GLOGI(kTag, "operator surface up on :80 (%u client sockets)", unsigned(kClientSockets));
    return g_srv;
}

size_t http80Sockets() {
    if (g_srv == nullptr) return 0;
    int fds[kClientSockets]{};   // exactly cfg.max_open_sockets
    size_t n = kClientSockets;
    if (httpd_get_client_list(g_srv, &n, fds) != ESP_OK) return 0;
    return n;
}

}  // namespace valence
