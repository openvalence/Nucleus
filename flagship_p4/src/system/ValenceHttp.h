#pragma once

// ValenceHttp -- the one esp_http_server instance on port 80, shared by every
// operator-facing route
// Constraints:
// - ONE HOME for the :80 server and its config (C-1). /uitoken, /ota and /diag
//   all register onto the handle this returns; none of them starts a server.
//   Two instances cannot share a port, and two esp_http_server instances must
//   never share a ctrl_port either -- the second one silently refuses to start.
// - The client-socket count here is one term of the lwIP socket arithmetic,
//   whose ONE home is flagship_p4/sdkconfig.defaults (CONFIG_LWIP_MAX_SOCKETS).
//   Changing kClientSockets means changing that arithmetic in the same commit.
// - STACK: /ota and /diag both run their whole transfer on this server's task,
//   so it is sized for them, not for the 128-byte /uitoken answer.
// - NO CORS HEADERS ON ANY ROUTE. See ValenceUiToken.h for why that absence is
//   the security property.
// See: ValenceUiToken.h, ValenceOta.h, ValenceDiag.h

#include <cstddef>

#include <esp_http_server.h>

namespace valence {

// Starts the instance on first call and returns the same handle afterward.
// nullptr means httpd_start failed; every caller treats that as non-fatal --
// the machine runs without its operator surface, it just cannot be updated or
// dumped.
httpd_handle_t http80();

// Client sockets the instance currently holds, against the budget above.
size_t http80Sockets();

}  // namespace valence
