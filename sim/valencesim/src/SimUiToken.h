#pragma once

// SimUiToken -- GET /uitoken for the host twin, RFC-029 §4
// Constraints:
// - Same contract as the P4's ValenceUiTokenMinter, whose header carries the
//   reasoning: NO CORS HEADERS, single-use, 60 s TTL, one mint per 250 ms
//   device-wide, CONTROL tier and never configure, Connection: close.
// - THE TOKEN IS THE BOARD'S: slot table, rate gate and HMAC derivation are
//   flagship_p4/src/hub/UiTokenTable.h, compiled verbatim. This class owns
//   only the lock, the per-boot secret and the route; the listener is the
//   front's (src/main.cpp's HTTP server, or integral_http in the wasm build).
// - THREADING: serve() may run on an HTTP connection thread, consume()
//   on the hub thread. The one mutex covers the table and nothing else, and
//   the HMAC runs outside it, as on the board; neither side touches
//   valence::Hub.
// See: flagship_p4/src/hub/ValenceUiToken.h, Valence RFC-029 §4

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <string_view>

#include "hub/UiTokenTable.h"
#include "hub/ValenceDevice.h"

namespace valence {

class SimUiToken final : public IUiTokenGate {
public:
    SimUiToken();

    // The HTTP route: GET /uitoken mints, anything else is 404. Returns the
    // status and fills `body` (JSON for /uitoken, "not found" otherwise).
    int serve(std::string_view method, std::string_view path, std::string& body);

    bool consume(std::span<const std::byte> token) override;

    // 200 with the token JSON, or 429 when rate-limited. Fills `body`.
    int mint(std::string& body);

private:
    std::mutex _m;
    UiTokenTable _table{};             // every access holds _m
    UiTokenTable::Secret _secret{};    // written once in the constructor
    SoftwareCrypto _cmp{};
};

}  // namespace valence
