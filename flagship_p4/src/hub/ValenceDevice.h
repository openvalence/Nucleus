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
#include <expected>
#include <optional>
#include <span>
#include <string_view>

#include "IngressDropTally.h"
#include "StoredState.h"
#include "ValenceCatalog.h"
#include "motion/ValenceMotion.h"
#include "patterns/PatternPresetStore.h"
#include "patterns/PatternSettings.h"
#include "system/ValenceMotorSwitch.h"
#include "valence/hub/hub.hpp"

namespace valence {

// tick()'s persist-due bits: which blob's debounced write is due.
inline constexpr uint8_t kPersistConfig  = 0x01;  // StoredState.h: 0x1000 + tuning + cfg_gen
inline constexpr uint8_t kPersistPresets = 0x02;  // PatternPresetStore: the 0x5220 slots

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

// ---- catalog capacity (RFC-077 item 8) ----------------------------------------
// The user-space room this build leaves beside the machine's own catalog: what
// the accessories roster advertises (val-9u0.19) and nothing more. Capacities
// and the per-accessory budget are build flags whose one home is
// flagship_p4/valence_capacity.cmake; ValenceDevice.cpp refuses to compile
// without them. bytes keeps the 80% headroom floor on the encode scratch.
struct CatalogHeadroom {
    uint16_t entries = 0;
    uint16_t layout = 0;
    uint16_t schema = 0;
    uint16_t safe = 0;      // RFC-076 safe-value slots
    uint32_t bytes = 0;
    uint8_t  accessories = 0;   // whole per-accessory budgets that fit, at most the build's count
};
CatalogHeadroom catalogHeadroom(const Catalog32& c, size_t encodedBytes);

// ---- supplied by the composition --------------------------------------------
// deviceNowUs(): the 64-bit monotonic clock motion plans against. It MUST be
// the clock the linked ValenceMotion implementation reads, or every stream
// anchor lands at the wrong instant. deviceFreeHeapBytes(): the 0x0006 heap
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

    // ---- persistence ----------------------------------------------------------
    // The composition owns STORAGE (NVS on the P4, a file in the twin); this
    // class owns what the bytes MEAN. Boot adoption is deliberately NOT an
    // intent and NOT a change: no ECHO, no dirty flag, no cfg_gen bump, and
    // attach()'s retained pushes ARE the announcement. Call both adopts before
    // the Hub is built over this delegate. An error or false = the blob was
    // rejected whole and the factory values stand; the config's error names
    // the check that refused it. The composition restores cfgGen into the Hub
    // itself: only it holds the Hub at that point.
    std::expected<void, stored::ConfigReject> adoptConfigBlob(std::span<const std::byte> blob,
                                                              uint16_t& cfgGen);
    bool adoptPresetsBlob(std::span<const std::byte> blob);
    // Bytes written, 0 when `out` is too small. A key on trial (RFC-099)
    // contributes its pre-trial value, never the live one.
    size_t encodeConfigBlob(std::span<std::byte> out, uint16_t cfgGen) const;
    size_t encodePresetsBlob(std::span<std::byte> out) const { return _presets.encode(out); }

    const StoredConfig& config() const { return _cfg; }

    // The stored config IS the arbiter's window and ceilings; this is the one
    // door between them (C-1).
    void pushConfigToMotion() const;

    // HOST TWIN ONLY (valencesim's commissioning posture): replaces the
    // first-run record for this run, before pushConfigToMotion(). Marks
    // nothing dirty; a later config persist writes whatever it then holds.
    // The board never calls it: its record comes from the owner's writes.
    void setSetupWritten(uint8_t mask) { _modes.setup_written = mask; }

    // Binds the hub and the catalog it was built over, then publishes every
    // retained STATE at its truthful at-rest value. Call once, after the Hub
    // is constructed over this delegate. The catalog outlives this object, as
    // it outlives the hub; it is read for field names only.
    void attach(Hub& hub, const Catalog32& catalog);

    // The device half of one hub tick. Call on the hub task right after
    // Hub::update(). Returns the kPersist* bits whose debounced write is due
    // NOW; each is returned once, so a composition that cannot write yet
    // holds the bits itself. The config blob takes the hub's cfgGen() at
    // write time.
    uint8_t tick(uint32_t nowMs);

    // ---- the board buttons (operator ruling 2026-10-02, bd val-091.26) --------
    // tick() takes the gestures ValenceButtons.h parks and binds them: HOME
    // press is home op 1 through applyHome(), as the wire's; HOME hold brakes
    // motion, then latches ESTOP (cause user: the power cut), then reports
    // rebootDue(); PAIR press opens the presence window (SPEC 12.3); PAIR hold
    // is reserved and only logged. The reboot itself is the composition's:
    // takePendingPersist(), the GOODBYEs, the restart. Never clears.
    bool rebootDue() const { return _reboot == Reboot::due; }
    // The kPersist* bits armed and not yet due, disarmed: a planned reboot
    // writes them now instead of after the debounce.
    uint8_t takePendingPersist();

    // ---- the e-stop at the machine (bd val-091.23) --------------------------
    // tick() latches ESTOP while ValenceEstopInput.h's reading stops, and
    // canClearEstop() refuses the release until it reads released. Releasing
    // the button clears nothing.

    // 0x0006 link RSSI in dBm, 0 = no reading. PUSHED IN from whichever task
    // owns the radio; never read on the hub task (see ValenceHub.h).
    void setLinkRssi(int8_t rssi) { _linkRssi.store(rssi, std::memory_order_relaxed); }

    // ---- HubDelegate --------------------------------------------------------
    AccessLevel validateToken(std::span<const std::byte> instance_id,
                              std::span<const std::byte> token, bool hasToken) override;
    Result<IntentValueMap, NackCode> applyIntent(uint16_t channel_id,
                                                 const IntentValueMap& requested,
                                                 AccessLevel role, bool& cfgChanged) override;
    std::string_view intentNackDetail(uint16_t channel_id, NackCode code) override;
    std::optional<uint8_t> sourceForChannel(uint16_t channel_id) override;
    bool canClearEstop() override;
    bool admitsUnderPause(uint16_t channel_id, const IntentValueMap& value,
                          bool overrideLatched) override;
    uint16_t scheduleHorizonMs(uint16_t channel_id) override;
    uint32_t scheduleLatencyUs(uint16_t channel_id) override;
    void onEstop(uint8_t cause, uint8_t origin) override;
    void onStreamBundle(uint16_t channel_id, uint32_t session_id,
                        const BundleView& bundle) override;
    void onSessionJoined(uint32_t session_id) override;
    void onSessionLeft(uint32_t session_id) override;
    void onSourceOwnership(uint8_t source_id, uint32_t owner_session, uint8_t reason) override;
    uint8_t sourceKind(uint8_t source_id) override;
    bool sourceQuiet(uint8_t source_id) override;
    std::optional<BlobView> readBlob(uint8_t ns, uint8_t store_id, uint8_t slot) override;
    // RFC-099 trial writes. Trialable: 0x3000 keys 1-8, 0x3030 key 4
    // (overshoot_clamp), 0x3120 every key but 10. Never chase_dense, the
    // horizon or the flip: each is refused on live state, so a revert could
    // be refused too.
    std::optional<IntentValue> trialBaseline(uint16_t channel_id, uint8_t key) override;
    Result<IntentValueMap, NackCode> applyTrialIntent(uint16_t channel_id, const IntentValueMap& requested,
                                                      AccessLevel role, bool& cfgChanged) override;
    void restoreTrial(uint16_t channel_id, const IntentValueMap& baselines, bool& cfgChanged) override;
    void onTrialCommit(uint16_t channel_id, const IntentValueMap& keys) override;

private:
    Result<IntentValueMap, NackCode> applyConfig(const IntentValueMap& requested, bool& cfgChanged);
    // `r`, marking a durable change when it applied outside a trial.
    Result<IntentValueMap, NackCode> durable(Result<IntentValueMap, NackCode> r);
    void noteSetupWritten(uint8_t wrote);
    // A window key (0x3000 key 1 or 2) is on trial: the flip is refused, its
    // baseline is in the client frame the flip would mirror.
    bool windowOnTrial() const;
    Result<IntentValueMap, NackCode> applyMove(const IntentValueMap& requested);
    Result<IntentValueMap, NackCode> applyHome(const IntentValueMap& requested);
    Result<IntentValueMap, NackCode> applyModes(const IntentValueMap& requested, bool& cfgChanged);
    Result<IntentValueMap, NackCode> applyTuning(const IntentValueMap& requested, bool& cfgChanged);
    void noteTuning(const MotionTuning& next, bool& cfgChanged);
    Result<IntentValueMap, NackCode> applyPattern(const IntentValueMap& requested);
    Result<IntentValueMap, NackCode> applyPatternAdvanced(const IntentValueMap& requested);
    Result<IntentValueMap, NackCode> applyPresets(const IntentValueMap& requested);
    Result<IntentValueMap, NackCode> applySafety(const IntentValueMap& requested);
    Result<IntentValueMap, NackCode> refuseUnpowered(const char* what);
    // `code`, with `detail` as its SPEC 16.1 reason.
    Result<IntentValueMap, NackCode> refuse(NackCode code, const char* detail);
    void noteDetail(const char* detail);
    // INVALID_VALUE for a present value that numberOf() or boolOf() refused:
    // NaN, an infinity, or no number at all. The detail is "<field>: not a
    // number", the field named as the catalog names `key` on `channel_id`.
    // Every writer checks each key it reads BEFORE touching anything: clampf()
    // passes NaN, so a clamp alone would store one.
    Result<IntentValueMap, NackCode> refuseNotANumber(uint16_t channel_id, uint8_t key);
    // A generator start's machine gates, in 0x3200's order: unpowered,
    // uncommissioned, unhomed. nullopt admits; the rail is acquired after.
    std::optional<NackCode> startRefusal(const MotionCensus& c, const char* what);
    bool railOwned() const;
    bool publishGrantLive(uint16_t channel_id) const;
    void haltGenerator();
    void pushPattern();
    void publishPatternPlane(const MotionCensus& mo);

    void serviceButtons(uint32_t nowMs);
    void homeFromButton();
    void beginReboot(uint32_t nowMs);

    void publishHubStatus();
    void publishMachineConfig();
    // The travel window as clients see it (RFC-088): _cfg holds it physical,
    // and with the flip on the same window reads mirrored against `rail`.
    struct Window {
        float lo;
        float hi;
        bool operator==(const Window&) const = default;
    };
    Window clientWindow(float rail) const;
    bool flipOpen(const MotionCensus& c) const;

    Hub* _hub = nullptr;
    const Catalog32* _catalog = nullptr;
    IUiTokenGate* _tokenGate = nullptr;
    AccessLevel _unvouchedRole = AccessLevel::watch;

    StoredConfig _cfg{};
    bool _cfgDirty = false;
    // RFC-099: set while a trial write or a revert runs. Neither persists nor
    // counts toward the first-run record.
    bool _trialApply = false;
    // A durable change since the last tick: only it arms the config persist.
    bool _durableDirty = false;
    // The hub's trialGen() the trial_mask fields last carried.
    uint32_t _trialGenSent = 0;
    // The live tuning set behind 0x1030 and 0x1120-0x1122: the engine's
    // factory set until a stored one is adopted. This copy IS the setting; the
    // engine holds whatever motionSetTuning() last carried from it.
    MotionTuning _tune = motionDefaultTuning();
    // The stored 0x1030 modes beside the tuning (StoredState.h).
    StoredModes _modes{};
    // The schedule_horizon and flipped mask bits as last published.
    bool _horizonOpenSent = true;
    bool _flipOpenSent = false;
    // The client-frame window 0x1000 last carried: it moves with the flip and
    // the rail, neither of which changes _cfg.
    Window _sentWindow{0.0f, 0.0f};
    // Which of those four cards an applied write changed, bit per card
    // (ValenceDevice.cpp, kCard*). tick() pushes and republishes, then clears.
    uint8_t _tuneDirty = 0;
    StoredConfig _lastPublishedCfg{};
    bool _cfgEverSent = false;
    // force_home asks tick() to release a held ESTOP and to clear
    // home_required, one tick later. DEFERRED on purpose: Hub::releaseEstop()
    // and Hub::setHomeRequired() publish and broadcast, and applyIntent runs
    // inside the hub's intent dispatch.
    bool _clearLatch = false;
    bool _homeDone = false;
    // SPEC 11.4 ownership as the hub reported it, indexed by MotionSource:
    // the session id that owns each source, 0 = unowned.
    std::array<uint32_t, 4> _owner{};
    // RFC-098: the motion task's intent count (accepted plus rejected) when
    // the last jog was submitted. The jog's slot stays held until the count
    // moves past it, so a jog admitted this tick is never quiet.
    std::optional<uint32_t> _jogMark;
    // A `return` is running: set on its acceptance, cleared when the
    // arbiter's census.returns moves past _returnsAtRequest, or by ESTOP.
    bool _returnPending = false;
    uint32_t _returnsAtRequest = 0;
    // census.homes as tick() last acted on it: a move past it is a completed
    // home cycle, which clears home_required.
    uint32_t _homesSeen = 0;
    // The motor switch's fault count as tick() last acted on it.
    uint16_t _mswFaultsSeen = 0;
    // A switch fault not yet acted on, and the tick it was first seen: an
    // EN-node one waits for the e-stop reader to name it (tick()).
    bool _mswFaultWaiting = false;
    uint32_t _mswFaultSinceMs = 0;
    // SPEC 16.1 NACK detail for the refusal this delegate last made, NUL
    // terminated; empty when it gave no reason. Three callers refuse and the
    // hub may ask after each: applyIntent(), admitsUnderPause() and
    // canClearEstop(). Each clears it on entry, so intentNackDetail() never
    // answers a stale one. Hub task only. Text over nack_detail_max_bytes is
    // cut by the hub.
    std::array<char, limits::nack_detail_max_bytes + 1> _nackDetail{};
    // The switch status hub-status last carried, so a change publishes now.
    MotorSwitchStatus _mswSent{};
    // A HOME hold's reboot: braking until at rest or _rebootBrakeUntilMs,
    // then due.
    enum class Reboot : uint8_t { none, braking, due };
    Reboot _reboot = Reboot::none;
    uint32_t _rebootBrakeUntilMs = 0;

    // The live pattern generator settings. This copy IS the setting (the
    // delegate is its one writer); the generator's task runs on whatever
    // tick() last pushed, with the stroke frame filled from _cfg at push time.
    PatternSettings _pat{};
    bool _patDirty = false;
    PatternPresetStore _presets{};
    // readBlob()'s answer lives here until the hub's next call (hub.hpp's
    // BlobView contract); re-encoded on every resume, never cached. Sized for
    // the largest preset item with its digest (static_assert in readBlob).
    std::array<std::byte, 160> _blobScratch{};
    // Last bytes SENT per pattern-plane channel: each republishes on a change
    // of its own bytes, including an enabled_mask that moved with homed/estop.
    std::array<std::byte, 20> _sentPatState{};
    std::array<std::byte, 15> _sentApBase{};
    std::array<std::array<std::byte, 7>, advpat::BASE_COUNT> _sentApMod{};
    std::array<std::byte, 4> _sentRoster{};
    bool _patPlaneSent = false;

    // Debounced persist, one timer per blob: armed by every applied change,
    // re-armed by the next, so a slider drag costs ONE write after the
    // operator lets go.
    bool _persistArmed = false;
    uint32_t _persistDueMs = 0;
    bool _presetsArmed = false;
    uint32_t _presetsDueMs = 0;
    // The store's generation as last armed or adopted. Every CRUD mutation
    // bumps the store's own, so a difference IS "the slots changed".
    uint16_t _presetsGenSeen = _presets.generation();   // declared after _presets

    // The hub half of 0x1111 sync_dropped, folded every tick.
    IngressDropTally _ingressDrops{};

    uint32_t _lastMotionMs = 0;
    uint32_t _lastPlanMs = 0;
    uint32_t _lastSlowMs = 0;
    uint32_t _lastStatusMs = 0;

    // One byte, relaxed: a stale reading is a stale reading either way, and
    // nothing orders against it.
    std::atomic<int8_t> _linkRssi{0};
};

}  // namespace valence
