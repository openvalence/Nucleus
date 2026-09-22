#pragma once

// valence_config.h -- the machine constants the catalog advertises and the hub
// identifies itself with
// Constraints:
// - ONE HOME for each of these on this board. FIRMWARE_VERSION is what the
//   boot banner and WELCOME identity both read; never spell a version anywhere
//   else.
// - The DEFAULT_*/MAX_* block below is the SOURCE the catalog's
//   valence::factory and valence::ceiling tables mirror. ValenceCatalog.h may
//   not include this file (it is library-only by contract), so ValenceHub.cpp
//   sees both and static_asserts them together. Change a number here and the
//   build fails until the catalog follows.
// - Values carried verbatim from the archived SlopDrive-32 S3 product
//   (include/system/config_api.h) so a client that knows one machine's limits
//   is not surprised by the other. They describe the MOTION plane, which
//   has_motion=false does not yet expose; they are still the stored
//   configuration and are published truthfully on 0x1000.
// See: .claude/rules/governance.md (C-1), ValenceCatalog.h

#define FIRMWARE_VERSION "0.1.5-p4hub"

// Identity strings for WELCOME key 37 (RFC-016a). Static storage, so the views
// the hub holds outlive it.
#define VALENCE_PRODUCT  "Nucleus"
#define VALENCE_HUB_NAME "nucleus-p4"

// ---- Stroke geometry --------------------------------------------------------
#define DEFAULT_MAX_RAIL_MM 500.0f

// ---- Speed / accel / jerk: factory defaults and hard ceilings ---------------
#define MAX_SPEED_MM_S              10000.0f
#define DEFAULT_MAX_SPEED_MM_S      950.0f
#define DEFAULT_USER_MAX_SPEED_MM_S 50.0f
#define DEFAULT_USER_ACCEL_MM_S2    200.0f
#define DEFAULT_ACCEL_MM_S2         50000.0f
#define MAX_ACCEL_MM_S2             100000.0f
#define DEFAULT_INPUT_MAX_JERK_MM_S3 2000000.0f
#define MAX_JERK_MM_S3              50000000.0f
