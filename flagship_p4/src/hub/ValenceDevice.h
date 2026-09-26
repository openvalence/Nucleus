#pragma once

// ValenceDevice -- this machine's half of the Valence hub: the HubDelegate,
// the stored 0x1000 config, and every retained STATE publisher
// Constraints:
// - HARDWARE-FREE. No IDF, FreeRTOS or board header may be included here or in
//   ValenceDevice.cpp: the host device twin (sim/valencesim) compiles both
//   verbatim, and a platform include is how the twin stops being a twin. Time
//   and heap enter through the two functions below, motion through
//   motion/ValenceMotion.h; each composition links its own implementation.
// - SINGLE-TASK, like the Hub it serves (T5). Every method runs on the hub
//   task except setLinkRssi(), which is a relaxed atomic store.
// - CONSTRUCTION ORDER IS LOAD-BEARING: build this, adopt any stored config,
//   construct the Hub over it as its delegate, THEN attach(). attach() is the
//   boot publish of every retained STATE; before it the hub has no motion
//   plane values to serve.
// - HONEST OR ABSENT: an intent this machine cannot really apply is NACKed
//   UNSUPPORTED_OP and never echoed (ValenceDevice.cpp carries the detail).
// See: ValenceHub.cpp (the P4 composition), sim/valencesim (the host one),
// Valence SPEC.md §4.2, §6.3, §9.1, §9.3, §12.2

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "ValenceCatalog.h"
#include "motion/ValenceMotion.h"
#include "patterns/PatternPresetStore.h"
#include "patterns/PatternSettings.h"
#include "valence/hub/hub.hpp"

namespace valence {

// The 0x1000 snapshot and the 0x3000 writer both speak exactly these eight
// values, and so does the P4's NVS blob -- change one and that blob's version
// changes with it (ValenceHub.cpp, kCfgVersion).
struct StoredConfig {
    float window_min  = factory::window_min;
    float window_max  = factory::window_max;
    float user_speed  = factory::user_speed;
    float user_accel  = factory::user_accel;
    float input_speed = factory::input_speed;
    float input_accel = factory::input_accel;
    float input_jerk  = factory::input_jerk;
    float max_rail    = factory::max_rail;

    bool operator==(const StoredConfig&) const = default;
};

// The consume side of /uitoken as validateToken sees it. A true return
// CONSUMES the token: single-use is not advisory.
class IUiTokenGate {
public:
    virtual ~IUiTokenGate() = default;
    virtual bool consume(std::span<const std::byte> token) = 0;
};

// The planes THIS board advertises, and their one home: the P4 composition and
// the host twin both build their catalog from it, so the twin can never serve
// a different etag by drifting a flag.
// Operator ruling 2026-09-21: the board acts like a normal machine with no
// motor, no Modbus drive and no current sensor. The motion plane is REAL --
// the arbiter, the engine and the LP emitter are all live -- so it is
// advertised, and so is the pattern generator that drives it
// (flagship_p4/src/patterns/); the two absent subsystems are what stays gated.
inline DeviceFeatures boardFeatures() {
    DeviceFeatures feat{};
    feat.has_motion  = true;
    feat.has_drive   = false;  // no Modbus drive on this board (val-091)
    feat.has_pattern = true;
    return feat;
}

// ---- supplied by the composition --------------------------------------------
// deviceNowUs(): the 64-bit monotonic clock motion plans against. It MUST be
// the clock the linked ValenceMotion implementation reads, or every stream
// anchor lands at the wrong instant. deviceFreeHeapBytes(): the 0x0007 heap
// figure; 0 where the host has no meaningful answer.
uint64_t deviceNowUs();
uint32_t deviceFreeHeapBytes();

class ValenceDevice final : public HubDelegate {
public:
    void bindTokenGate(IUiTokenGate* g) { _tokenGate = g; }

    // The role a HELLO lands at when no pairing token and no live /uitoken
    // vouches for it. The firmware never raises it from watch; the host twin
    // may, and says so on its command line.
    void setUnvouchedRole(AccessLevel r) { _unvouchedRole = r; }

    // Boot adoption. Deliberately NOT an intent and NOT a change: no ECHO, no
    // dirty flag, no cfg_gen bump. The retained 0x1000 push attach() makes IS
    // the announcement.
    void adoptConfig(const StoredConfig& c) {
        _cfg = c;
        _cfgDirty = false;
    }
    const StoredConfig& config() const { return _cfg; }

    // The stored config IS the arbiter's window and ceilings; this is the one
    // door between them (C-1).
    void pushConfigToMotion() const;

    // Binds the hub and publishes every retained STATE at its truthful at-rest
    // value. Call once, after the Hub is constructed over this delegate.
    void attach(Hub& hub);

    // The device half of one hub tick. Call on the hub task right after
    // Hub::update(). Returns true when the debounced config write is due; the
    // composition persists config() with the hub's cfgGen() at that moment.
    bool tick(uint32_t nowMs);

    // 0x0007 link RSSI in dBm, 0 = no reading. PUSHED IN from whichever task
    // owns the radio; never read on the hub task (see ValenceHub.h).
    void setLinkRssi(int8_t rssi) { _linkRssi.store(rssi, std::memory_order_relaxed); }

    // ---- HubDelegate --------------------------------------------------------
    AccessLevel validateToken(std::span<const std::byte> instance_id,
                              std::span<const std::byte> token, bool hasToken) override;
    Result<IntentValueMap, NackCode> applyIntent(uint16_t channel_id,
                                                 const IntentValueMap& requested,
                                                 AccessLevel role, bool& cfgChanged) override;
    std::optional<uint8_t> sourceForChannel(uint16_t channel_id) override;
    bool canClearEstop() override;
    void onEstop(uint8_t cause, uint8_t origin) override;
    void onStreamBundle(uint16_t channel_id, uint32_t session_id,
                        const BundleView& bundle) override;
    void onSessionJoined(uint32_t session_id) override;
    void onSessionLeft(uint32_t session_id) override;
    void onSourceOwnership(uint8_t source_id, uint32_t owner_session, uint8_t reason) override;
    std::optional<BlobView> readBlob(uint8_t ns, uint8_t store_id, uint8_t slot) override;

private:
    Result<IntentValueMap, NackCode> applyMove(const IntentValueMap& requested);
    Result<IntentValueMap, NackCode> applyHome(const IntentValueMap& requested);
    Result<IntentValueMap, NackCode> applyModes(const IntentValueMap& requested, bool& cfgChanged);
    Result<IntentValueMap, NackCode> applyTuning(const IntentValueMap& requested, bool& cfgChanged);
    void noteTuning(const MotionTuning& next, bool& cfgChanged);
    Result<IntentValueMap, NackCode> applyPattern(const IntentValueMap& requested);
    Result<IntentValueMap, NackCode> applyPatternAdvanced(const IntentValueMap& requested);
    Result<IntentValueMap, NackCode> applyPresets(const IntentValueMap& requested);
    void pushPattern();
    void publishPatternPlane(const MotionCensus& mo);

    void publishHubStatus();
    void publishMachineConfig();

    Hub* _hub = nullptr;
    IUiTokenGate* _tokenGate = nullptr;
    AccessLevel _unvouchedRole = AccessLevel::watch;

    StoredConfig _cfg{};
    bool _cfgDirty = false;
    // The live tuning set behind 0x1030 and 0x1120-0x1122, seeded from the
    // engine's factory set at attach(). This copy IS the setting; the engine
    // holds whatever tick() last pushed from it.
    MotionTuning _tune{};
    // Which of those four cards an applied write changed, bit per card
    // (ValenceDevice.cpp, kCard*). tick() pushes and republishes, then clears.
    uint8_t _tuneDirty = 0;
    StoredConfig _lastPublishedCfg{};
    bool _cfgEverSent = false;
    // force_home cleared the arbiter's latch; the hub's own ESTOP bit drops in
    // tick(), one tick later. DEFERRED on purpose: Hub::clearEstop() publishes
    // and broadcasts, and applyIntent runs inside the hub's intent dispatch.
    bool _clearLatch = false;

    // The live pattern generator settings. This copy IS the setting (the
    // delegate is its one writer); the generator's task runs on whatever
    // tick() last pushed, with the stroke frame filled from _cfg at push time.
    PatternSettings _pat{};
    bool _patDirty = false;
    PatternPresetStore _presets{};
    // readBlob()'s answer lives here until the hub's next call (hub.hpp's
    // BlobView contract); re-encoded on every resume, never cached.
    std::array<std::byte, 128> _blobScratch{};
    // Last bytes SENT per pattern-plane channel: each republishes on a change
    // of its own bytes, including an enabled_mask that moved with homed/estop.
    std::array<std::byte, 20> _sentPatState{};
    std::array<std::byte, 9> _sentApBase{};
    std::array<std::array<std::byte, 7>, advpat::BASE_COUNT> _sentApMod{};
    std::array<std::byte, 4> _sentRoster{};
    bool _patPlaneSent = false;

    // Debounced persist: armed by every applied change, re-armed by the next,
    // so a slider drag costs ONE write after the operator lets go.
    bool _persistArmed = false;
    uint32_t _persistDueMs = 0;

    uint32_t _lastMotionMs = 0;
    uint32_t _lastPlanMs = 0;
    uint32_t _lastSlowMs = 0;
    uint32_t _lastStatusMs = 0;

    // One byte, relaxed: a stale reading is a stale reading either way, and
    // nothing orders against it.
    std::atomic<int8_t> _linkRssi{0};
};

}  // namespace valence
