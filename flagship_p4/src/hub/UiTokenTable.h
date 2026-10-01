#pragma once

// UiTokenTable -- the /uitoken credential store, hardware-free, RFC-029 §4
// Constraints:
// - NOT THREAD-SAFE ON ITS OWN, BY DESIGN. claimMint(), install() and consume()
//   run with the HOST's lock held; derive() reads no member and runs unlocked.
//   The lock is the one thing that differs per host: the board's is a portMUX
//   in internal RAM (ValenceUiToken.cpp), the sim's a std::mutex
//   (SimUiToken.cpp). Neither host may call a locked method without it.
// - NOTHING HERE ALLOCATES, LOGS OR READS A CLOCK. Time arrives as u32 ms and
//   every comparison is wrap-safe, so the locked methods are pure arithmetic
//   and safe inside portENTER_CRITICAL.
// - THE COMPARATOR IS PASSED IN, never constructed here: on the board it must
//   be a namespace-scope object built at static init (cpp-safety.md T4), which
//   only the host can guarantee.
// - Properties the endpoint promises: single-use, kTtlMs, one mint per
//   kMinIntervalMs device-wide, at most kSlots live tokens, CONTROL tier only.
// See: flagship_p4/src/hub/ValenceUiToken.h, sim/valencesim/src/SimUiToken.h,
//      Valence SPEC.md §12.8, test/native/test_ui_token

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "valence/core/crypto.hpp"
#include "valence/wire/hmac_sha256.hpp"

namespace valence {

class UiTokenTable {
public:
    static constexpr size_t kTokenBytes = 16;        // = valence limits::token_bytes
    static constexpr size_t kSecretBytes = 32;
    static constexpr size_t kSlots = 4;              // a few tabs' worth, no more
    static constexpr uint32_t kTtlMs = 60000;        // RFC-029 §4: short
    static constexpr uint32_t kMinIntervalMs = 250;  // rate limit, per device

    using Token = std::array<std::byte, kTokenBytes>;
    using Secret = std::array<std::byte, kSecretBytes>;

    // LOCKED. The counter this mint derives from, or nullopt when the previous
    // mint was under kMinIntervalMs ago. A refused mint changes nothing.
    std::optional<uint32_t> claimMint(uint32_t nowMs) {
        if (_everMinted && (nowMs - _lastMintMs) < kMinIntervalMs) return std::nullopt;
        _everMinted = true;
        _lastMintMs = nowMs;
        return ++_counter;
    }

    // UNLOCKED. token = HMAC(boot secret, counter LE || now LE)[0..15]
    // (RFC-028.3). The claimed counter alone makes every mint's material unique.
    static Token derive(const Secret& secret, uint32_t counter, uint32_t nowMs) {
        std::array<std::byte, 8> material{};
        for (size_t i = 0; i < 4; ++i) material[i] = std::byte((counter >> (8 * i)) & 0xFF);
        for (size_t i = 0; i < 4; ++i) material[4 + i] = std::byte((nowMs >> (8 * i)) & 0xFF);
        const auto mac = hmacSha256(std::span<const std::byte>(secret),
                                    std::span<const std::byte>(material));
        Token tok{};
        for (size_t i = 0; i < kTokenBytes; ++i) tok[i] = mac[i];
        return tok;
    }

    // LOCKED. A used or never-minted slot loses first, then the one expiring
    // soonest, so a fifth live token evicts the oldest.
    void install(const Token& token, uint32_t nowMs) {
        size_t victim = 0;
        for (size_t i = 1; i < kSlots; ++i) {
            if (_slots[i].used && !_slots[victim].used) {
                victim = i;
                continue;
            }
            if (_slots[i].used == _slots[victim].used &&
                int32_t(_slots[i].expiresMs - _slots[victim].expiresMs) < 0) {
                victim = i;
            }
        }
        _slots[victim].token = token;
        _slots[victim].expiresMs = nowMs + kTtlMs;
        _slots[victim].used = false;
    }

    // LOCKED. Is this a live, unexpired, unused token? True CONSUMES it:
    // single-use is not advisory. Expired slots met on the way are retired.
    bool consume(std::span<const std::byte> token, uint32_t nowMs, const ICrypto& cmp) {
        if (token.size() != kTokenBytes) return false;
        for (auto& s : _slots) {
            if (s.used) continue;
            if (int32_t(nowMs - s.expiresMs) >= 0) {
                s.used = true;
                continue;
            }
            if (cmp.constantTimeEqual(std::span<const std::byte>(s.token), token)) {
                s.used = true;
                return true;
            }
        }
        return false;
    }

private:
    struct Slot {
        Token token{};
        uint32_t expiresMs = 0;
        bool used = true;   // a never-minted slot is "already used"
    };

    std::array<Slot, kSlots> _slots{};
    uint32_t _counter = 0;
    uint32_t _lastMintMs = 0;
    bool _everMinted = false;
};

}  // namespace valence
