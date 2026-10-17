#pragma once

// NeutrinoObserver -- the dev tool channel: a localhost-only WebSocket
// (--devtool-port) that streams the machine's snapshots out as JSON and takes
// the physical inputs and network faults the protocol never carries
// Constraints:
// - THREE KINDS OF THREAD, TWO CROSSINGS. The hub thread calls begin(),
//   pass() and wifiDown() and is the only caller of SimCore, the Hub and the
//   port. IXWebSocket connection threads only parse a command line and push it
//   into the input queue (and log). The broadcaster thread only takes the
//   newest snapshot from the mailbox, encodes it and sends it. Neither
//   non-hub thread ever reaches SimCore, the Hub, motion or a link.
// - The hub's side of both crossings never waits (NeutrinoDevtool.h).
// - BOUND TO 127.0.0.1, never configurable: the inputs include an e-stop and a
//   PAIR button, which no LAN peer may press.
// - SIM-ONLY: nothing here exists on the board. The buttons run through the
//   board's own ButtonGesture cores; their gestures reach the delegate through
//   the same take functions the board's BoardIo task fills.
// See: NeutrinoDevtool.h, sim/valencesim/README.md (the dev tool)

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <ixwebsocket/IXWebSocketServer.h>

#include "NeutrinoDevtool.h"
#include "NeutrinoWsPort.h"
#include "SimCore.h"
#include "system/ButtonGesture.h"

namespace valence {

class NeutrinoObserver {
public:
    static constexpr uint32_t kSnapshotPeriodUs = 33333;   // display rate, 30 Hz

    NeutrinoObserver() = default;
    NeutrinoObserver(const NeutrinoObserver&) = delete;
    NeutrinoObserver& operator=(const NeutrinoObserver&) = delete;
    ~NeutrinoObserver() { stop(); }

    // Hub thread, after the core and the port began. False when the port
    // cannot bind (`err` says why).
    bool begin(SimCore& core, NeutrinoWsPort& port, uint16_t devtoolPort, uint16_t wsPort, uint16_t httpPort,
               std::string& err);
    void stop();

    // Hub thread, every pass after SimCore::pass(): inputs, buttons, the
    // Wi-Fi drop's clock, the 1 ms sample, and the snapshot at display rate.
    void pass(uint64_t nowUs);
    // Hub thread. A Wi-Fi drop is running: the front drains the discovery
    // port and discards what arrives (devtool::discoveryHook), as a board off
    // the air never receives it.
    bool wifiDown() const { return _wifiHeld; }

private:
    void apply(const devtool::Input& in, uint64_t nowUs);
    void fillSnapshot(uint64_t nowUs);
    void broadcast();

    // ---- hub thread only ----------------------------------------------------
    SimCore* _core = nullptr;
    NeutrinoWsPort* _port = nullptr;
    button::PairButton _pairCore;
    button::Button _homeCore;
    bool _pairDown = false;
    bool _homeDown = false;
    bool _estopDown = false;
    bool _wifiHeld = false;
    uint64_t _wifiUntilUs = 0;
    uint64_t _nextSnapUs = 0;
    uint64_t _seq = 0;
    size_t _logRev = 0;
    devtool::Snapshot _build;
    std::array<devtool::Input, devtool::kInputDepth> _drained{};

    // ---- the crossings ------------------------------------------------------
    devtool::DevInputQueue _inputs;
    devtool::SnapshotMailbox _mailbox;

    // ---- written once in begin(), before any thread starts; read-only after --
    std::string _hello;
    bench::SessionLog* _log = nullptr;

    std::unique_ptr<ix::WebSocketServer> _server;
    std::thread _broadcaster;
    std::atomic<bool> _stop{false};
};

}  // namespace valence
