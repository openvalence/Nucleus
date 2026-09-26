#pragma once

// SimUiToken -- GET /uitoken for the host twin, RFC-029 §4
// Constraints:
// - Same contract as the P4's ValenceUiTokenMinter, whose header carries the
//   reasoning: NO CORS HEADERS, single-use, 60 s TTL, one mint per 250 ms
//   device-wide, CONTROL tier and never configure, Connection: close.
// - THREADING: mint runs on an IXWebSocket HTTP connection thread, consume()
//   on the hub thread. The one mutex covers the slot table and nothing else;
//   neither side touches valence::Hub.
// - The token is 16 bytes from std::random_device, not the P4's HMAC over a
//   counter. Both are unguessable per boot; the wire cannot tell them apart.
// See: flagship_p4/src/hub/ValenceUiToken.h, Valence RFC-029 §4

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <span>
#include <string>

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
    static constexpr size_t kTokenBytes = 16;        // = valence limits::token_bytes
    static constexpr size_t kSlots = 4;
    static constexpr uint32_t kTtlMs = 60000;
    static constexpr uint32_t kMinIntervalMs = 250;

    struct Slot {
        std::array<std::byte, kTokenBytes> token{};
        uint32_t expiresMs = 0;
        bool used = true;
    };

    std::mutex _m;
    std::array<Slot, kSlots> _slots{};
    bool _everMinted = false;
    uint32_t _lastMintMs = 0;
    std::random_device _rd;
    std::unique_ptr<ix::HttpServer> _server;
};

}  // namespace valence
