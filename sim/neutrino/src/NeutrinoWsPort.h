#pragma once

// NeutrinoWsPort -- the native front's valence.v1 WebSocket port: Valence
// Bench's per-connection transport, with a NeutrinoLink between each one and
// the hub so the dev tool can shape every client's link
// Constraints:
// - THREADS ARE THE BENCH PORT'S. IXWebSocket runs one thread per connection;
//   those threads only feed a slot's ValenceBenchWsTransport (RX ring and
//   open/close flags, its own lock). Everything else here, the links
//   included, is the hub thread's: loop(), tick(), the fault setters, info().
// - The hub attaches the LINK, never the transport, so every frame both ways
//   crosses the fault layer; with no fault set it passes straight through.
// - A copy of bench::ValenceBenchWsPort's accept, close and sweep logic
//   (../Valence/hub/bench/src/net/WsServerPort.cpp): a drift seam, tracked on
//   the board. The transport class itself is compiled from its home.
// See: NeutrinoLink.h, sim/neutrino/README.md

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <ixwebsocket/IXWebSocketServer.h>

#include "NeutrinoLink.h"
#include "common/SessionLog.h"
#include "net/WsServerPort.h"
#include "valence/hub/hub.hpp"

namespace valence {

class NeutrinoWsPort {
public:
    static constexpr uint8_t kSlots = kHubMaxSessions + 1;

    bool begin(Hub* hub, uint16_t port, bench::SessionLog* log, const std::string& bindHost);
    void stop();

    // Hub thread, every hub tick: open/close events (attach/detach), the
    // idle and stall sweeps.
    void loop(uint32_t nowMs);
    // Hub thread, every pass: each link hands its transport what is due.
    void tick(uint64_t nowUs);

    // ---- the dev tool's knobs (hub thread) ----------------------------------
    void setGlobalFaults(const LinkFaults& f);
    const LinkFaults& globalFaults() const { return _global; }
    void setClientFaults(uint8_t slot, const LinkFaults& f);
    void setHeld(bool held);
    void kick(uint8_t slot);

    struct SlotInfo {
        bool inUse = false;
        std::string peer;
        bool muted = false;
        uint32_t rxDrops = 0;
        LinkStats link{};
        LinkFaults faults{};
    };
    SlotInfo info(uint8_t slot) const;

private:
    void onMessage(std::shared_ptr<ix::ConnectionState> state, ix::WebSocket& ws,
                   const ix::WebSocketMessagePtr& msg);
    void applyFaults(uint8_t slot) { _links[slot].setFaults(combineFaults(_global, _own[slot])); }

    std::unique_ptr<ix::WebSocketServer> _server;
    Hub* _hub = nullptr;
    bench::SessionLog* _log = nullptr;
    std::array<bench::ValenceBenchWsTransport, kSlots> _slots{};
    std::array<NeutrinoLink, kSlots> _links{};
    std::array<LinkFaults, kSlots> _own{};
    std::array<bool, kSlots> _attached{};
    LinkFaults _global{};
    uint32_t _nowMs = 0;

    std::mutex _mapM;   // connection threads: id -> slot
    std::unordered_map<std::string, uint8_t> _slotById;
};

}  // namespace valence
