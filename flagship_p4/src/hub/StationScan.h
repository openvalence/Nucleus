// The station's AP choice, applied to every wifi_config_t the board hands
// esp_wifi_set_config (boot in main.cpp, a join and its restore in
// ValenceProvisioning.cpp). Without it the IDF takes the first AP found, and
// several APs share one SSID after an infrastructure move.
//
// Do not set bssid_set or a channel here: btm_enabled lets the router steer
// the station, and esp_wifi clears a pinned BSSID under it anyway. The scan
// repeats on every (re)connect because sort_method has no memory.
// esp_hosted forwards each of these fields to the C6 (rpc_req.c, the STA
// case of the set-config request).
#pragma once

#include <esp_wifi.h>

namespace valence {

// Below this the link cannot carry a motion stream; the station reports
// NO_AP_FOUND_IN_RSSI_THRESHOLD and keeps retrying instead of joining it.
inline constexpr int8_t kStationRssiFloorDbm = -85;

inline void stationScanPolicy(wifi_config_t& cfg) {
    cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    cfg.sta.threshold.rssi = kStationRssiFloorDbm;
    cfg.sta.rm_enabled = 1;    // 802.11k: neighbor reports
    cfg.sta.btm_enabled = 1;   // 802.11v: the router may steer us
}

}  // namespace valence
