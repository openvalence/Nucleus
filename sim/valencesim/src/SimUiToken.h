#pragma once

// SimUiToken -- GET /uitoken for the host twin, RFC-029 §4
// Constraints:
// - Same contract as the P4's ValenceUiTokenMinter, whose header carries the
//   reasoning: NO CORS HEADERS, single-use, 60 s TTL, one mint per 250 ms
//   device-wide, CONTROL tier and never configure, Connection: close.
// - THE TOKEN IS THE BOARD'S: slot table, rate gate and HMAC derivation are
//   flagship_p4/src/hub/UiTokenTable.h, compiled verbatim. This class owns
//   only the lock, the per-boot secret and the HTTP listener.
// - THREADING: mint runs on an IXWebSocket HTTP connection thread, consume()
//   on the hub thread. The one mutex covers the table and nothing else, and
//   the HMAC runs outside it, as on the board; neither side touches
//   valence::Hub.
// See: flagship_p4/src/hub/ValenceUiToken.h, Valence RFC-029 §4

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>

#include "hub/UiTokenTable.h"
#include "hub/ValenceDevice.h"

namespace ix {
class HttpServer;
}

namespace valence {

class SimUiToken final : public IUiTokenGate {
public:
    SimUiToken();
    ~SimUiToken() override;

    // Starts the HTTP listener on `port` (127.0.0.1). False if it cannot bind.
    bool begin(uint16_t port, std::string& err);
    void stop();

    bool consume(std::span<const std::byte> token) override;

    // 200 with the token JSON, or 429 when rate-limited. Fills `body`.
    int mint(std::string& body);

private:
    std::mutex _m;
    UiTokenTable _table{};             // every access holds _m
    UiTokenTable::Secret _secret{};    // written once in the constructor
    SoftwareCrypto _cmp{};
    std::unique_ptr<ix::HttpServer> _server;
};

}  // namespace valence
