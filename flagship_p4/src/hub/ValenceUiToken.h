#pragma once

// ValenceUiToken -- GET /uitoken on port 80, RFC-029 §4
// Constraints:
// - NO CORS HEADERS. EVER. The endpoint's ENTIRE security property is the
//   browser's same-origin policy: a page on another origin may SEND this
//   request but the browser will not let it READ the answer, so the token
//   reaches a same-origin UI and nothing else. `Access-Control-Allow-Origin`
//   here would hand control of the machine to every page on the internet.
// - THE :80 SERVER IS NOT THIS CLASS'S. It registers a route onto the shared
//   instance (system/ValenceHttp.h), which /ota and /diag also use; a second
//   server on the same port cannot start.
// - PORT 80, not the WS port. Valence clients build "http://<ip>/uitoken"
//   with no port regardless of where the socket lives (valence_probe.py's
//   mint_uitoken), so serving it only on 82 leaves the mint silently failing
//   and every session landing at WATCH tier -- which reads as a transport
//   fault and is not.
// - The honest limit, stated in code because a doc nobody reads is not a
//   disclosure: a native process on the LAN can curl this and get a
//   control-tier token. Accepted. The class this closes is the browser-borne
//   one, which is the class that reaches a user who never opened the UI.
// - Properties: single-use, ~60 s TTL, rate-limited, CONTROL tier and never
//   configure -- a browser-borne credential must not re-key the trust ledger.
//   The slot table, rate gate and derivation are UiTokenTable.h, the one copy
//   the sim compiles too; this class owns only the lock, the secret and :80.
// - THE SPINLOCK IS A FILE-SCOPE STATIC IN THE .cpp, not a member (T4 and
//   T2 together): this object is a member of the PSRAM-resident hub box, and
//   a portMUX_TYPE must live in internal RAM -- the compare-and-set it is
//   taken with is defined on internal memory only, and external RAM is
//   additionally unreachable inside a flash-cache-disabled window.
// - THREADING: mintJson() runs on the :80 httpd task, consume() on the hub
//   task. Neither touches valence::Hub, so the hub's one-task invariant is
//   untouched.
// - A MINT IS ONE REQUEST, SO THE SOCKET CLOSES WITH THE ANSWER. A browser
//   would otherwise hold this connection open on keep-alive for minutes for a
//   fetch it will never repeat, and this board's whole lwIP socket table is
//   small enough that a few parked tabs lock out both listeners (val-091.15).
//   Two client sockets are all this server ever needs.
// See: Valence RFC-029 §4, SPEC.md §12.2

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <esp_http_server.h>

#include "UiTokenTable.h"
#include "ValenceDevice.h"

namespace valence {

class ValenceUiTokenMinter final : public IUiTokenGate {
public:
    // Seeds the per-boot HMAC secret from the hardware TRNG. Tokens never
    // survive a reboot, which is correct: a reboot is a trust boundary.
    void begin();

    // Registers GET /uitoken on the shared :80 instance.
    bool attachRoutes();

    // Hub task: is this the bytes of a live, unexpired, unused token? A true
    // return CONSUMES it -- single-use is not advisory.
    bool consume(std::span<const std::byte> token) override;

    // Fills `body` with the JSON answer. 0 ok / 2 rate-limited.
    uint8_t mintJson(char* body, size_t cap);

    uint32_t minted() const { return _minted; }
    uint32_t consumed() const { return _consumed; }

private:
    static esp_err_t handleGet(httpd_req_t* req);

    UiTokenTable _table{};             // every access holds s_mux
    UiTokenTable::Secret _secret{};    // written once in begin(), read-only after
    uint32_t _minted = 0;
    uint32_t _consumed = 0;
};

}  // namespace valence
