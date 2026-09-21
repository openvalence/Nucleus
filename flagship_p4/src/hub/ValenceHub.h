#pragma once

// ValenceHub -- the Valence composition root on the P4: catalog, clock, rng,
// delegate, the Hub itself, and the one task that pumps it
// Constraints:
// - THE HUB IS SINGLE-TASK BY CONTRACT (T5). Every valence::Hub method is
//   called from the hub task and nowhere else; there is no mutex and none is
//   wanted. Transports marshal their own callbacks across (ValenceWsPort).
// - The Hub and the Catalog32 live in PSRAM via placement-new (T2): a ~22 KB
//   catalog plus the hub's session/retained storage in .bss would eat the
//   internal heap the network stack allocates from. TASK STACKS STAY
//   INTERNAL: PSRAM is unreachable while the flash cache is off.
// - CONSTRUCTION ORDER IS LOAD-BEARING: catalog, clock and rng must all exist
//   and be final before the Hub constructor runs -- it encodes the catalog and
//   draws boot_id from the rng right there.
// - NOTHING HERE OWNS MOTION. The delegate submits intents and reads
//   motionCensus(); the arbiter owns every gate, the window clamp and the
//   limit set (architecture.md section 2). hubBegin() therefore runs AFTER
//   motionBegin(): the boot publish of every motion STATE channel reads the
//   census, and a hub that came up first would seed them from a dead struct.
// See: Valence SPEC.md §6, §8, §9; ValenceCatalog.h

#include <cstdint>

#include "valence/hub/hub.hpp"

namespace valence {

// The hub task's stack, in bytes, and the ONE home for that number (C-1): the
// create site and main.cpp's high-water watch table both read it here, so the
// reported total can never drift from the allocated one.
//
// RAISED 8,192 -> 16,384 ON A MEASUREMENT, not on taste. Under the val-091.9
// workload (a Phosphor session live, one full probe run, a motion bench cycle)
// the deepest free was 1,840 B of 8,192 -- 22 % headroom [verified 2026-09-21
// -- uxTaskGetStackHighWaterMark, COM15]. The deep path is a client's catalog
// BLOB transfer, and it GREW when val-091.11 advertised the motion plane: that
// same path now chunks a bigger blob. T21 forbids sizing DOWN without a mark
// and this is the other direction: 16 KB is the motion task's size, internal,
// and it costs 8,192 B of internal heap.
inline constexpr uint32_t kHubTaskStackBytes = 16384;

// Brings up logging, the catalog, the hub and the hub task. Returns false if
// any of that failed; the caller reports and carries on (the bench emitters
// are independent of the hub).
bool hubBegin();

// The composed hub, or nullptr before hubBegin() succeeds. For the boot report
// and the census line only -- never a door for another task to call into.
valence::Hub* hub();

// Census numbers for the 5 s liveness line, read from any task.
struct HubCensus {
    uint32_t sessions = 0;
    uint32_t ticks = 0;
    uint32_t wsFrames = 0;
    uint32_t wsDrops = 0;
    // Hub task stack high-water HEADROOM in bytes, 0 before the task exists.
    // T21: a mark only knows the paths that have run, so it is a floor on this
    // boot's workload, never a sizing number on its own.
    uint32_t stackFree = 0;
};
HubCensus hubCensus();

// Link RSSI for the 0x0006 hub-status snapshot, in dBm; 0 means "no reading".
// IT IS PUSHED IN, NOT PULLED, AND THAT IS THE WHOLE POINT: the radio is on
// the C6, so esp_wifi_sta_get_ap_info() is an esp_hosted RPC, MEASURED at
// 151,008 us on this board [verified 2026-09-20 -- boot log, first call timed
// with esp_timer]. On a 5 ms tick that is a thirty-tick stall every second,
// which is the instrument manufacturing the fault it is meant to observe
// (archived SlopDrive-32 repo, memory-budget.md T27). The caller is main.cpp's
// 5 s liveness loop, a task with nothing to starve.
void hubSetLinkRssi(int8_t rssi);

}  // namespace valence
