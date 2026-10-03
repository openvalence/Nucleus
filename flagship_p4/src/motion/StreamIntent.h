#pragma once

// StreamIntent -- one c2h motion-input sample (0x2100 point, 0x2101 timed
// segment) decoded into the arbiter's MotionIntent, the one home of that
// mapping
// Constraints:
// - HARDWARE-FREE and inline: the hub delegate (ValenceDevice.cpp) and the
//   offline planner (tools/kinetic-wasm) compile this one copy, so a sample
//   reaches the engine as the same float bits on both. Never restate it.
// - lo_mm/span_mm are the CLIENT-frame window (RFC-088, clientWindow()); the
//   arbiter mirrors the result onto the physical rail.
// - Units are the catalog layouts': position u16 in 1e-4 of the window,
//   duration u16 ms, velocity i16 in 1e-3 window/s. anchor_us is already
//   resolved into the engine clock; the lead cap is the caller's (SPEC 5.4).
// See: SPEC 5.4 (the unspecified sentinel), SPEC 9.6, ValenceDevice.cpp
// (onStreamBundle)

#include <cstdint>
#include <optional>

#include "ValenceMotion.h"
#include "valence/generated/registry_constants.hpp"

namespace valence {

inline MotionIntent pointIntent(uint16_t pos_e4, int16_t vel_e3, float lo_mm, float span_mm,
                                uint64_t anchor_us) {
    const float norm = float(pos_e4) / 10000.0f;
    MotionIntent in;
    in.source       = MotionSource::Stream;
    in.target_mm    = lo_mm + norm * span_mm;
    in.anchor_us    = anchor_us;
    in.end_vel_mm_s = float(vel_e3) / 1000.0f * span_mm;
    in.has_end_vel  = (vel_e3 != 0);
    return in;
}

// nullopt for a zero duration: durationless points belong on 0x2100.
inline std::optional<MotionIntent> segmentIntent(uint16_t pos_e4, uint16_t dur_ms, int16_t end_vel_e3,
                                                 float lo_mm, float span_mm, uint8_t curve_family,
                                                 uint64_t anchor_us) {
    if (dur_ms == 0) return std::nullopt;
    const float norm = float(pos_e4) / 10000.0f;
    MotionIntent in;
    in.source       = MotionSource::Stream;
    in.target_mm    = lo_mm + norm * span_mm;
    in.anchor_us    = anchor_us;
    in.duration_us  = uint32_t(dur_ms) * 1000u;
    in.curve_family = curve_family;
    // SPEC 5.4 `unspecified`: 0 is a real slope (a reversal ends AT rest), so
    // absence is the registry's sentinel. Unspecified leaves has_end_vel
    // false and the engine resolves it (SPEC 9.6).
    if (end_vel_e3 != limits::segment_end_vel_unspecified) {
        in.end_vel_mm_s = float(end_vel_e3) / 1000.0f * span_mm;
        in.has_end_vel  = true;
    }
    return in;
}

}  // namespace valence
