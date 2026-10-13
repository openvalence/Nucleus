#pragma once

// LinkMode -- when the hub runs BLE GATT, what its advertisement says, and
// what a single press of PAIR means offline
// Constraints:
// - Hardware-free; suite test_config_mode. The hub task owns the one instance.
// - CONFIG MODE (SPEC §13.4.1, RFC-079) is decided once, at boot, by PAIR held.
//   It never ends without a reboot: no station association, no WebSocket, BLE
//   for the whole mode, the window granting `configure`.
// - OFFLINE (Valence RFC-117 draft) is "no reachable WebSocket listener", the
//   state ble_adv_flags.ws_available clear names. Offline continuously for
//   kOfflineBleAfterMs, or with no credentials at all, BLE starts by itself.
// - BLE STOPS only online, with no BLE session attached (none was, or it
//   migrated to WS, SPEC §6.3), after kBleDwellMs up. A hub healthy on WiFi
//   never starts it unless config mode was asked for.
// - These three numbers are hub policy, not registry limits (RFC-117 item 1).
// See: Valence SPEC.md §12.3, §13.4, §13.4.1, §13.9; RFC-114, RFC-117 drafts

#include <cstddef>
#include <cstdint>

#include "valence/generated/registry_constants.hpp"

namespace valence::linkmode {

inline constexpr uint32_t kOfflineBleAfterMs = 30000;
inline constexpr uint32_t kBleDwellMs = 60000;

class Policy {
public:
    // Once, before the hub task: the boot gesture and whether any network
    // credentials exist to try.
    void boot(bool configMode, bool haveCredentials, uint32_t nowMs) {
        _config = configMode;
        _offlineSinceMs = nowMs;
        if (configMode || !haveCredentials) startBle(nowMs);
    }

    // Hub task, every tick. `online`: an IP and the WS listener up.
    // `bleAttached`: a BLE link carries a session now.
    void tick(bool online, bool bleAttached, uint32_t nowMs) {
        if (_config) return;
        if (!online) {
            if (_online) _offlineSinceMs = nowMs;
            _online = false;
            if (!_ble && uint32_t(nowMs - _offlineSinceMs) >= kOfflineBleAfterMs) startBle(nowMs);
            return;
        }
        _online = true;
        if (_ble && !bleAttached && uint32_t(nowMs - _bleSinceMs) >= kBleDwellMs) _ble = false;
    }

    bool configMode() const { return _config; }
    bool bleWanted() const { return _ble; }
    // No LAN client can reach the hub: the single press approves (RFC-117 item 2).
    bool offline() const { return _config || !_online; }

    // The scan response's flags byte (SPEC §13.4, registry ble_adv_flags).
    uint8_t advFlags(bool windowOpen) const {
        uint8_t f = 0;
        if (windowOpen) f |= ble_adv_flags::pairing_window_open;
        if (!offline()) f |= ble_adv_flags::ws_available;
        if (_config) f |= ble_adv_flags::config_mode;
        return f;
    }

private:
    void startBle(uint32_t nowMs) {
        _ble = true;
        _bleSinceMs = nowMs;
    }

    bool _config = false;
    bool _online = false;
    bool _ble = false;
    uint32_t _offlineSinceMs = 0;
    uint32_t _bleSinceMs = 0;
};

// What one press of PAIR does (RFC-114 item 4 online; RFC-117 item 2 offline).
enum class PressAct : uint8_t {
    open_window,   // the single-grant presence window
    approve,       // the one configure ask, as RFC-114's double press
    refuse_lan,    // an ask is pending but a LAN session exists: approve nothing
    nothing,       // offline, claimed, no ask: a press hands nothing to radio range
};

// `configureAsks`: fresh pending knocks asking `configure`; several still
// map to approve, which the hub refuses as several_pending. `lanSessions`:
// occupied sessions on a network binding, parked ones included.
constexpr PressAct singlePress(bool offline, bool configMode, bool claimed, size_t configureAsks,
                               size_t lanSessions) {
    if (!offline) return PressAct::open_window;
    if (configureAsks > 0) return lanSessions > 0 ? PressAct::refuse_lan : PressAct::approve;
    return configMode || !claimed ? PressAct::open_window : PressAct::nothing;
}

}  // namespace valence::linkmode
