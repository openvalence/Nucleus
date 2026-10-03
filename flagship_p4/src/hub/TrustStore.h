#pragma once

// TrustStore -- the trust ledger's persistence and the SPEC §12.3 power-cycle
// gesture counter, hardware-free
// Constraints:
// - HARDWARE-FREE and header-only: the P4 composition binds IKeyStore to NVS,
//   the native suite to a fake. Storage enters through IKeyStore only.
// - SINGLE-TASK: boot() runs once, on the composing task, before the hub task
//   exists and before any session can attach; tick() runs on the hub task only
//   (transport.md T5), after Hub::update().
// - THE LEDGER IS ONE BLOB under kLedgerKey, at most trust_ledger_max_bytes
//   (one NVS page). It CARRIES TOKENS: it never leaves the device and is never
//   sent on the wire (Valence session/pairing.hpp, encodeLedger).
// - WRITTEN ONLY ON CHANGE: a write lands when the encoded bytes differ from
//   the bytes last stored, at most once per kLedgerWriteMinIntervalMs, and
//   never while holdWrites is true (an OTA transfer in flight). A change made
//   inside the interval is held, never dropped.
// - CLAIMABLE AT BOOT means zero configure tokens AND no stored ledger that
//   failed to load. A stored ledger that is unreadable or rejected leaves the
//   hub unclaimable with an empty ledger; the gesture is the way back in.
// - THE GESTURE COUNTS POWER-ON BOOTS ONLY. A panic, watchdog or software
//   reset breaks the sequence: a crash loop is not physical presence, and a
//   remote fault must never open a pairing window.
// - A counter byte above pairing_gesture_boot_count reads as 0: corruption
//   can delay a gesture, never fake one.
// See: Valence SPEC.md §12.3, §12.6; Valence session/pairing.hpp;
// ValenceHub.cpp (NVS binding and wear), bd val-fvn

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "valence/generated/registry_constants.hpp"
#include "valence/session/pairing.hpp"

namespace valence {

// ---- storage seam ---------------------------------------------------------------

enum class KeyLoadStatus : uint8_t {
    Absent,   // the key was never written
    Loaded,   // `bytes` holds the stored value
    Failed,   // present but unreadable, or larger than the scratch
};

struct KeyLoad {
    KeyLoadStatus status = KeyLoadStatus::Absent;
    std::span<const std::byte> bytes{};   // a prefix of the caller's scratch
};

class IKeyStore {
public:
    virtual ~IKeyStore() = default;
    virtual KeyLoad load(const char* key, std::span<std::byte> scratch) = 0;
    // True once the bytes are durable (committed).
    virtual bool save(const char* key, std::span<const std::byte> bytes) = 0;
};

// ---- the store --------------------------------------------------------------------

inline constexpr const char* kLedgerKey = "trust";
inline constexpr const char* kGestureKey = "pgest";
// The wear bound on ledger writes. A burst of roster changes (a grant, then
// the grantee's first HELLO) coalesces into one write after it.
inline constexpr uint32_t kLedgerWriteMinIntervalMs = 2000;
inline constexpr size_t kLedgerBlobBytes = size_t(limits::trust_ledger_max_bytes);

struct TrustBoot {
    bool ledgerLoaded = false;     // a stored ledger was adopted
    bool ledgerRejected = false;   // a stored ledger exists and did not load
    bool claimable = false;        // §12.3(c) factory-fresh: zero configure tokens
    bool gesture = false;          // the power-cycle gesture armed this boot
    bool counterSaved = false;     // the boot's counter write landed
    uint8_t shortBoots = 0;        // consecutive short power-on boots before this one
    size_t paired = 0;             // ledger entries after the load

    bool openWindow() const { return claimable || gesture; }
};

// tick()'s report, for the composition to log. Bits.
inline constexpr uint8_t kTrustLedgerWritten  = 0x01;
inline constexpr uint8_t kTrustLedgerFailed   = 0x02;   // encode or save failed; retried
inline constexpr uint8_t kTrustGestureCleared = 0x04;
inline constexpr uint8_t kTrustGestureFailed  = 0x08;   // not retried

class TrustStore {
public:
    // Adopts the stored ledger into `p` and steps the gesture counter. Call
    // once, after the Hub exists and before its transports start.
    // powerOnBoot: the last reset was a power-on, not a panic, watchdog or
    // software reset.
    TrustBoot boot(PairingManager& p, IKeyStore& store, bool powerOnBoot) {
        TrustBoot b;
        const KeyLoad l = store.load(kLedgerKey, _scratch);
        if (l.status == KeyLoadStatus::Loaded && p.decodeLedger(l.bytes)) {
            b.ledgerLoaded = true;
            remember(l.bytes);
        } else if (l.status != KeyLoadStatus::Absent) {
            b.ledgerRejected = true;
        }
        // Adoption is not a change: decodeLedger() marks the ledger dirty.
        p.clearDirty();
        b.paired = p.entryCount();
        b.claimable = !b.ledgerRejected && !p.hasConfigureToken();

        uint8_t count = 0;
        std::array<std::byte, 1> raw{};
        const KeyLoad g = store.load(kGestureKey, raw);
        const bool haveCount = g.status == KeyLoadStatus::Loaded && g.bytes.size() == 1 &&
                               uint8_t(g.bytes[0]) <= kGestureBoots;
        if (haveCount) count = uint8_t(g.bytes[0]);
        // This boot counts as short until tick() sees the uptime pass the
        // limit. The gesture firing consumes the count, so next stays 0.
        uint8_t next = 0;
        if (powerOnBoot) {
            b.shortBoots = count;
            if (count >= kGestureBoots) b.gesture = true;
            else next = uint8_t(count + 1);
        }
        if (haveCount && count == next) {
            b.counterSaved = true;
        } else {
            raw[0] = std::byte(next);
            b.counterSaved = store.save(kGestureKey, raw);
        }
        _gestureClearDue = b.counterSaved && next != 0;
        return b;
    }

    // Hub task, every tick. uptimeMs is milliseconds since boot; it is also
    // the gesture's uptime clock. Returns kTrust* bits.
    uint8_t tick(PairingManager& p, IKeyStore& store, uint32_t uptimeMs, bool holdWrites) {
        if (p.dirty()) {
            p.clearDirty();
            _ledgerPending = true;
        }
        if (holdWrites) return 0;
        uint8_t out = 0;
        if (_gestureClearDue && uptimeMs >= limits::pairing_gesture_max_uptime_ms) {
            _gestureClearDue = false;
            const std::array<std::byte, 1> zero{};
            out |= store.save(kGestureKey, zero) ? kTrustGestureCleared : kTrustGestureFailed;
        }
        if (_ledgerPending && (!_wroteOnce || uptimeMs - _lastWriteMs >= kLedgerWriteMinIntervalMs))
            out |= writeLedger(p, store, uptimeMs);
        return out;
    }

    // Hub task, before a planned reboot: a pending ledger change is written
    // NOW, inside the write interval, or it is lost with the boot. Returns
    // kTrust* bits; 0 when nothing was pending.
    uint8_t flush(PairingManager& p, IKeyStore& store, uint32_t uptimeMs) {
        if (p.dirty()) {
            p.clearDirty();
            _ledgerPending = true;
        }
        return _ledgerPending ? writeLedger(p, store, uptimeMs) : 0;
    }

private:
    static constexpr uint8_t kGestureBoots = uint8_t(limits::pairing_gesture_boot_count);

    uint8_t writeLedger(PairingManager& p, IKeyStore& store, uint32_t uptimeMs) {
        _ledgerPending = false;
        const size_t n = p.encodeLedger(_scratch);
        const std::span<const std::byte> blob(_scratch.data(), n);
        if (n != 0 && _storedValid && std::ranges::equal(blob, stored())) return 0;
        _wroteOnce = true;
        _lastWriteMs = uptimeMs;
        if (n != 0 && store.save(kLedgerKey, blob)) {
            remember(blob);
            return kTrustLedgerWritten;
        }
        _ledgerPending = true;   // retried at the interval
        return kTrustLedgerFailed;
    }

    void remember(std::span<const std::byte> bytes) {
        std::ranges::copy(bytes, _stored.begin());
        _storedLen = bytes.size();
        _storedValid = true;
    }
    std::span<const std::byte> stored() const { return {_stored.data(), _storedLen}; }

    std::array<std::byte, kLedgerBlobBytes> _scratch{};
    std::array<std::byte, kLedgerBlobBytes> _stored{};
    size_t _storedLen = 0;
    bool _storedValid = false;
    bool _ledgerPending = false;
    bool _wroteOnce = false;
    uint32_t _lastWriteMs = 0;
    bool _gestureClearDue = false;
};

}  // namespace valence
