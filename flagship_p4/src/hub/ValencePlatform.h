#pragma once

// ValencePlatform -- the P4 bindings for Valence's injected seams
// (IClock / IRandom)
// Constraints:
// - SPEC §17.2 makes determinism a conformance requirement, so ALL time and
//   entropy enter the library through these two interfaces. This header is the
//   ONE place they are bound to hardware; nothing wire-visible lives here.
// - SPEC §7.2: hub time is u32 microseconds since boot and WRAPS every
//   ~71.6 min BY SPEC. The truncation below is deliberate -- widening it would
//   desync the hub's wrap-safe compares from the wire clock.
// - Lifted verbatim in behavior from the S3's SlopSyncPlatform.h (archived
//   SlopDrive-32 repo); only the namespace differs.
// See: Valence SPEC.md §7.2, §17.2

#include <cstddef>
#include <cstdint>
#include <span>

#include <esp_random.h>
#include <esp_timer.h>

#include "valence/core/clock.hpp"
#include "valence/core/rng.hpp"

namespace valence {

class EspClock final : public valence::IClock {
public:
    uint32_t nowUs() const override {
        return static_cast<uint32_t>(esp_timer_get_time() & 0xFFFFFFFFull);
    }
};

// esp_random() is the SoC hardware entropy source, valid once the RF subsystem
// is up. On this board the radio lives on the C6 over esp_hosted, but the P4's
// own RNG entropy pool is seeded independently and is what feeds session ids,
// boot_id, WELCOME nonces and pairing tokens (§6.1, §12.2).
class EspRandom final : public valence::IRandom {
public:
    uint32_t nextU32() override { return esp_random(); }

    void fill(std::span<std::byte> out) override {
        if (!out.empty()) esp_fill_random(out.data(), out.size());
    }
};

}  // namespace valence
