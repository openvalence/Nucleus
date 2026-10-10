#pragma once

// ValenceTcpTally -- names the port whose TCP segments the board tallies
// Constraints:
// - Call once, from the hub's WebSocket bring-up, before a client can
//   connect; any task.
// See: ValenceTcpTally.cpp

#include <cstdint>

namespace valence {

void tcpTallyWatch(uint16_t port);

}  // namespace valence
