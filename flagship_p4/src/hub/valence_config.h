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
// - The values describe the MOTION plane, which has_motion=false does not
//   yet expose; they are still the stored configuration and are published
//   truthfully on 0x1000.
// - NUCLEUS_BENCH_NO_MOTOR is a BUILD PROFILE, never a runtime switch: the
//   CMake option of that name (flagship_p4/CMakeLists.txt, env
//   flagship_p4_bench) defines it to 1. It makes the motion arbiter's
//   motor-power gate advisory so the devkit can run patterns with the switch
//   off, and it suffixes FIRMWARE_VERSION with "-bench" so that image can
//   never pass for a release in WELCOME identity. Nothing else may read it.
// See: .claude/rules/governance.md (C-1), ValenceCatalog.h, MotionArbiter.h
// (the gate), bd val-091.58

#ifndef NUCLEUS_BENCH_NO_MOTOR
#define NUCLEUS_BENCH_NO_MOTOR 0
#endif

#if NUCLEUS_BENCH_NO_MOTOR
#define FIRMWARE_VERSION "0.1.10-p4hub-bench"
#else
#define FIRMWARE_VERSION "0.1.10-p4hub"
#endif

// Identity strings for WELCOME key 37 (RFC-016a). Static storage, so the views
// the hub holds outlive it.
#define VALENCE_PRODUCT  "Nucleus"
#define VALENCE_HUB_NAME "nucleus-p4"

// ---- Stroke geometry --------------------------------------------------------
#define DEFAULT_MAX_RAIL_MM 500.0f
#define MIN_RAIL_MM         10.0f

// ---- Homing: the approach speed's factory value and floor (MotionArbiter.h) --
#define DEFAULT_HOME_SPEED_MM_S 40.0f
#define MIN_HOME_SPEED_MM_S     5.0f

// ---- Speed / accel / jerk: factory defaults and hard ceilings ---------------
#define MAX_SPEED_MM_S              10000.0f
#define DEFAULT_MAX_SPEED_MM_S      1000.0f
#define DEFAULT_JOG_MAX_SPEED_MM_S  50.0f
#define DEFAULT_JOG_ACCEL_MM_S2     200.0f
#define DEFAULT_ACCEL_MM_S2         50000.0f
#define MAX_ACCEL_MM_S2             100000.0f
#define DEFAULT_INPUT_MAX_JERK_MM_S3 2000000.0f
#define MAX_JERK_MM_S3              50000000.0f

#ifdef __cplusplus
namespace valence {
// The bench profile as C++ reads it. True only in a NUCLEUS_BENCH_NO_MOTOR build.
inline constexpr bool kBenchNoMotor = NUCLEUS_BENCH_NO_MOTOR != 0;
}  // namespace valence
#endif
