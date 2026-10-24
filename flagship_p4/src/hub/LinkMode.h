#pragma once

// LinkMode -- how this boot reaches clients (SPEC §13.1's two routes, §13.4.1
// config mode), what the BLE advertisement says, when the hub's own policy
// wants a reboot, and what a single press of PAIR means
// Constraints:
// - Hardware-free; suite test_config_mode. The hub task owns the one Policy.
// - THE ROUTE IS DECIDED ONCE, BEFORE THE RADIOS START (decideBoot()), and
//   changes only by a reboot. The previous boot speaks to this one through
//   one persisted one-shot flag (Next), cleared once read.
// - BLE GATT SESSIONS ONLY IN CONFIG MODE (SPEC §13.4): normal mode
//   advertises scannable and non-connectable with no GATT service, the
//   radios-off route advertises nothing.
// - rebootWanted() is the policy half only. The composition adds the rest of
//   SPEC §13.4.1's rule before it reboots: no motion source running, no join
//   in flight, GOODBYE REBOOTING to every session first (ValenceHub.cpp).
// - kJoinWindowMs is device policy, not a registry limit (RFC-121).
// See: Valence SPEC.md §12.3, §13.1, §13.4, §13.4.1, §13.9

#include <cstddef>
#include <cstdint>

#include "valence/generated/registry_constants.hpp"

namespace valence::linkmode {

// The boot-time join reaches an address within this long, or the hub reboots
// into config mode (unless a session is attached).
inline constexpr uint32_t kJoinWindowMs = 60000;

// What the previous boot asked of this one. Stored as one byte; the values
// are the stored encoding, append-only.
enum class Next : uint8_t {
    none = 0,
    config_gesture = 1,       // the gesture was made while the hub ran
    config_join_failed = 2,   // the boot-time join missed kJoinWindowMs
    skip_auto = 3,            // policy config mode rebooted out: no automatic entry this boot
};

enum class Boot : uint8_t {
    normal,           // radios on: WebSocket; BLE advertises, never connects
    radios_off,       // the setting: serial only, one client, no config mode
    config_gesture,   // the gesture: the window opens at boot and grants configure
    config_policy,    // no credentials, or the last boot's join missed its window
};

struct BootFacts {
    Next next = Next::none;
    bool pairHeld = false;          // PAIR held through power-on: the gesture at boot
    bool radiosOff = false;         // the stored setting
    bool haveCredentials = false;   // any saved network to try
};

// The gesture turns radios a setting disabled back on for its boot.
constexpr Boot decideBoot(const BootFacts& f) {
    if (f.pairHeld || f.next == Next::config_gesture) return Boot::config_gesture;
    if (f.radiosOff) return Boot::radios_off;
    if (f.next == Next::skip_auto) return Boot::normal;
    if (!f.haveCredentials || f.next == Next::config_join_failed) return Boot::config_policy;
    return Boot::normal;
}

constexpr bool isConfig(Boot b) { return b == Boot::config_gesture || b == Boot::config_policy; }

constexpr const char* bootName(Boot b) {
    switch (b) {
        case Boot::normal:         return "normal";
        case Boot::radios_off:     return "radios off";
        case Boot::config_gesture: return "config mode (gesture)";
        case Boot::config_policy:  return "config mode (policy)";
    }
    return "?";
}

// The status pixel's network look (device-defined, RFC-121).
enum class NetLook : uint8_t { joining, online, lost, config, radios_off };

class Policy {
public:
    // Once, before the hub task. `autoEntry`: a missed boot-time join may
    // reboot into config mode (false on the boot after a reboot out of it).
    void boot(Boot b, bool autoEntry, uint32_t nowMs) {
        _boot = b;
        _autoEntry = autoEntry;
        _bootMs = nowMs;
        _nowMs = nowMs;
    }

    // Hub task, every tick. `online`: an address and the WS listener up.
    void tick(bool online, uint32_t nowMs) {
        _online = online && _boot == Boot::normal;
        if (_online) _everOnline = true;
        _nowMs = nowMs;
    }

    Boot route() const { return _boot; }
    bool configMode() const { return isConfig(_boot); }
    bool advertising() const { return _boot != Boot::radios_off; }
    bool wsAvailable() const { return _online; }

    // The scan response's flags byte (SPEC §13.4, registry ble_adv_flags).
    uint8_t advFlags(bool windowOpen) const {
        uint8_t f = 0;
        if (windowOpen) f |= ble_adv_flags::pairing_window_open;
        if (wsAvailable()) f |= ble_adv_flags::ws_available;
        if (configMode()) f |= ble_adv_flags::config_mode;
        return f;
    }

    NetLook look() const {
        if (_boot == Boot::radios_off) return NetLook::radios_off;
        if (configMode()) return NetLook::config;
        if (_online) return NetLook::online;
        return _everOnline ? NetLook::lost : NetLook::joining;
    }

    // The policy reboot due now, Next::none for none. `sessionAttached`: any
    // live session (a cable host holds a normal boot). `configureHeld`: a
    // ready session holding configure, which keeps policy config mode open.
    // `stationUp`: the station holds an address.
    Next rebootWanted(bool sessionAttached, bool configureHeld, bool stationUp) const {
        if (_boot == Boot::normal)
            return _autoEntry && !_everOnline && !sessionAttached && uint32_t(_nowMs - _bootMs) >= kJoinWindowMs
                       ? Next::config_join_failed
                       : Next::none;
        if (_boot == Boot::config_policy && stationUp && !configureHeld) return Next::skip_auto;
        return Next::none;
    }

private:
    Boot _boot = Boot::normal;
    bool _autoEntry = true;
    bool _online = false;
    bool _everOnline = false;
    uint32_t _bootMs = 0;
    uint32_t _nowMs = 0;
};

// What one press of PAIR does (RFC-114 item 4; SPEC §12.3 for config mode).
enum class PressAct : uint8_t {
    open_window,   // the single-grant presence window
    approve,       // the one configure ask, as RFC-114's double press
    nothing,       // policy config mode, claimed, no ask: a press hands nothing to radio range
};

// `configureAsks`: fresh pending knocks asking `configure`; several still
// map to approve, which the hub refuses as several_pending.
constexpr PressAct singlePress(Boot b, bool claimed, size_t configureAsks) {
    if (b != Boot::config_policy) return PressAct::open_window;
    if (configureAsks > 0) return PressAct::approve;
    return claimed ? PressAct::nothing : PressAct::open_window;
}

}  // namespace valence::linkmode
