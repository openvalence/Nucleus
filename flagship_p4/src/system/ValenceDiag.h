#pragma once

// ValenceDiag -- the diagnostics ARCHIVE: every Geiger record into a PSRAM
// ring, read once over HTTP after something goes wrong
// Constraints:
// - A DUMP, NOT A STREAM. Depth beats liveness: the failure that matters is
//   "the ring recycled before anyone read it", so this is megabytes deep and
//   is read whole, after the fact. GET /diag is the archive, GET /diag/<tag>
//   filters one GLOG tag, ?from=<seq> composes with both.
// - THE WRITER NEVER STOPS FOR A READER. There is no freeze, no lock and no
//   reader-owned state on this ring. A lapped reader detects it and truncates
//   WITH A FOOTER NOTICE; it is never protected, because a read can be a slow
//   pager and a gate held that long is the latched-gate class.
// - THE FOOTER'S next=<seq> IS THE RESUME CURSOR and it is the LAST line of
//   every dump on purpose: a machine consumer parses it there and passes it
//   back as ?from=.
// - THE ARCHIVE DIES WITH ITS BOOT. PSRAM is re-allocated at diagBegin().
//   Post-panic forensics is the RTC_NOINIT crash ring, whose last few Warn+
//   breadcrumbs from the PREVIOUS boot head every dump.
// - Blocking the :80 task for a full dump is ACCEPTED. This is the instrument
//   reached for when the machine is already unwell; it carries no heap floor
//   and allocates nothing, so memory pressure cannot switch it off.
// - The console sink keeps its own floor; the archive takes everything that
//   clears the logger's runtime floor.
// See: .claude/rules/logging-leds.md, bd val-091.17

#include <cstddef>
#include <cstdint>

namespace valence {

// Allocates the PSRAM archive, adopts the previous boot's crash breadcrumbs
// and registers the Geiger sink. Call it BEFORE anything worth logging, and
// before the network: it needs nothing but PSRAM. False means no archive (the
// console sink is unaffected).
bool diagBegin();

// Registers GET /diag* on the shared :80 instance. Non-fatal on failure.
bool diagAttachRoutes();

// Archive bytes actually allocated, 0 when diagBegin() failed. For the boot
// banner.
size_t diagArchiveBytes();

}  // namespace valence
