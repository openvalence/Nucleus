#pragma once

// ValenceLogBridge -- Geiger onto the Valence log channel (0x0008, registry
// `log`, RFC-017): Warn and above, published from the hub task's drain
// Constraints:
// - The sink's write() runs inside geiger::drainToSinks() and publishes ONLY
//   when that drain runs on the hub task: valence::Hub is single-task by
//   contract (transport.md T5) and app_main also drains during boot. A line
//   drained anywhere else still reaches the console and /diag, never the wire.
// - Floor Warn: the wire carries what an operator must see after the fact;
//   Info and below stay on the console and in /diag. The hub truncates long
//   lines (Hub::publishLog), it never drops them for length, and its replay
//   ring hands a late subscriber the last 32.
// - Owner: app_main calls logBridgeBegin() once, after hubBegin(). State is
//   one sink object in BSS, a few bytes.
// See: Valence lib/valence/include/valence/hub/hub.hpp (publishLog),
// .claude/rules/logging-leds.md

namespace valence {

// Registers the bridge sink. Returns false when Geiger's sink table is full.
bool logBridgeBegin();

}  // namespace valence
