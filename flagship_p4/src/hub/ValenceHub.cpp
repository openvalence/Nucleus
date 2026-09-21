// ValenceHub -- composition root for the Valence hub on the P4
// Constraints:
// - See ValenceHub.h for the single-task, PSRAM and construction-order rules.
// - Every static_assert below pins ValenceCatalog.h's hand-mirrored defaults
//   to valence_config.h. The catalog is library-only and cannot include the
//   config header; this TU sees both, so it is where the mirror is nailed.
// - The delegate is HONEST OR ABSENT: an intent it cannot really apply is
//   NACKed UNSUPPORTED_OP (0x0303, registry.yaml:609) and NEVER echoed. A
//   refusal that IS a machine state (e-stop latched, not homed) carries that
//   state's own code instead, so a client gets a reason it can act on.
// - EVERY advertised STATE channel is published at boot with its truthful
//   at-rest value and kept truthful after. An advertised-but-never-published
//   STATE leaves a subscriber holding "no idea" where the protocol promised a
//   value, and a generic client sits at syncing forever.
// - THE 0x1000 CONFIG AND ITS cfg_gen ARE PERSISTED IN NVS (namespace
//   "valence", key "cfg"). The load happens BEFORE the first retained 0x1000
//   push, so a subscriber's first snapshot is the stored truth and never a
//   default that a later load overwrites. Adoption is a plain assignment into
//   the delegate: it never becomes an intent, an ECHO or a cfg_gen bump.
// - NVS WRITES RUN ON THE HUB TASK (T5: never in a transport callback) and are
//   DEBOUNCED by kCfgPersistDebounceMs of quiet. Wear arithmetic: the blob is
//   40 B, which NVS stores as 3 of its 32 B entries; a 4 KB NVS page holds 126
//   entries, so ~42 rewrites fill a page and cost one sector erase. At the
//   debounce floor of one write per 2 s that is one erase per ~84 s, and the
//   100,000-cycle endurance floor is then ~97 days of config being changed
//   without pause -- on ONE page, before NVS wear-levels across the others. A
//   slider drag is one write, not one per frame.
// See: Valence SPEC.md §4.2, §6.3, §9.1, §9.3

#include "ValenceHub.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <optional>

#include <esp_heap_caps.h>
#include <esp_netif.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <new>
#include <nvs.h>

#include "ValenceCatalog.h"
#include "geiger/geiger.h"
#include "ValencePlatform.h"
#include "ValenceUiToken.h"
#include "ValenceWsPort.h"
#include "motion/ValenceMotion.h"
#include "valence_config.h"

#include "valence/util/byte_io.hpp"

namespace valence {

// ---- anti-drift guards: catalog mirror vs valence_config.h -------------------
static_assert(factory::window_min  == 0.0f,                        "catalog window_min drifted");
static_assert(factory::window_max  == DEFAULT_MAX_RAIL_MM,         "catalog window_max drifted");
static_assert(factory::user_speed  == DEFAULT_USER_MAX_SPEED_MM_S, "catalog user_speed drifted");
static_assert(factory::user_accel  == DEFAULT_USER_ACCEL_MM_S2,    "catalog user_accel drifted");
static_assert(factory::input_speed == DEFAULT_MAX_SPEED_MM_S,      "catalog input_speed drifted");
static_assert(factory::input_accel == DEFAULT_ACCEL_MM_S2,         "catalog input_accel drifted");
static_assert(factory::input_jerk  == DEFAULT_INPUT_MAX_JERK_MM_S3,"catalog input_jerk drifted");
static_assert(factory::max_rail    == DEFAULT_MAX_RAIL_MM,         "catalog max_rail drifted");
static_assert(ceiling::speed_max   == MAX_SPEED_MM_S,              "catalog speed ceiling drifted");
static_assert(ceiling::accel_max   == MAX_ACCEL_MM_S2,             "catalog accel ceiling drifted");
static_assert(ceiling::jerk_max    == MAX_JERK_MM_S3,              "catalog jerk ceiling drifted");

namespace {

using valence::AccessLevel;
using valence::IntentValue;
using valence::IntentValueMap;
using valence::NackCode;
using Ret = valence::Result<IntentValueMap, NackCode>;

constexpr const char* kTag = "hub";

// The Valence socket. 82 on every machine in this ecosystem; /uitoken rides
// plain HTTP on 80 regardless (see ValenceUiToken.h).
constexpr uint16_t kWsPort = 82;

// ---- the stored machine configuration ---------------------------------------
// The 0x1000 snapshot and the 0x3000 writer both speak exactly these eight
// values, and so does the NVS blob below -- change one and the blob version
// changes with it.
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

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---- packed-layout writers ---------------------------------------------------
// The STATE layouts below are long and every offset in them is wire-visible, so
// nothing spells an offset twice: a miscounted subspan is a silent field shift
// that renders as plausible numbers in the wrong columns.

// Free functions over a caller-owned buffer and a caller-owned cursor, not a
// writer object: a cursor that HELD the span would be a borrowed member, which
// the safe subset forbids outright (cpp-safety.md).
void packU8(std::span<std::byte> o, size_t& n, uint8_t v)   { n += valence::putU8(o.subspan(n), v); }
void packU16(std::span<std::byte> o, size_t& n, uint16_t v) { n += valence::putU16(o.subspan(n), v); }
void packI16(std::span<std::byte> o, size_t& n, int16_t v)  { packU16(o, n, uint16_t(v)); }
void packU32(std::span<std::byte> o, size_t& n, uint32_t v) { n += valence::putU32(o.subspan(n), v); }
void packF32(std::span<std::byte> o, size_t& n, float v)    { n += valence::putF32(o.subspan(n), v); }

// A packed field's wire value is value*scale, SATURATED at the type. Saturating
// beats wrapping: a position past the top of a u16 reads as the far end of the
// rail, never as the near end.
uint16_t wireU16(float v, float scale) {
    const float x = v * scale;
    if (!(x > 0.0f)) return 0;                 // NaN takes this branch
    return x >= 65535.0f ? uint16_t(65535) : uint16_t(x + 0.5f);
}
int16_t wireI16(float v, float scale) {
    const float x = v * scale;
    if (!std::isfinite(x)) return 0;
    if (x >= 32767.0f) return 32767;
    if (x <= -32768.0f) return -32768;
    return int16_t(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

const valence::IntentValueField* findField(const IntentValueMap& m, uint8_t key) {
    for (uint32_t i = 0; i < m.count; ++i) {
        if (m.fields[i].key == key) return &m.fields[i];
    }
    return nullptr;
}

float fieldF32(const valence::IntentValueField* f, float dflt) {
    if (!f) return dflt;
    switch (f->value.kind) {
        case IntentValue::Kind::F32:  return f->value.f32_val;
        case IntentValue::Kind::U64:  return float(f->value.u64_val);
        case IntentValue::Kind::I64:  return float(f->value.i64_val);
        default:                      return dflt;
    }
}

uint64_t fieldU64(const valence::IntentValueField* f, uint64_t dflt) {
    if (!f) return dflt;
    switch (f->value.kind) {
        case IntentValue::Kind::U64: return f->value.u64_val;
        case IntentValue::Kind::I64: return f->value.i64_val < 0 ? dflt : uint64_t(f->value.i64_val);
        default:                     return dflt;
    }
}

// 0x2101 field 3's "no end velocity" sentinel. 0 is a legitimate slope, so it
// cannot mean absent; INT16_MIN is the value the catalog reserves.
constexpr int16_t kSegNoEndVel = -32768;

// How far ahead of now a stream sample's t_off may resolve before it is treated
// as a client clock that lost sync. Past this the sample is pulled back rather
// than parked: a quarter second of runway is already far more than any bundle
// span, so a larger lead is a resync failure, not a schedule.
constexpr int32_t kStreamFarFutureUs = 250000;

// ---- NVS persistence for 0x1000 and its cfg_gen ------------------------------
// One blob, one write. cfg_gen rides WITH the values because §4.2 makes it a
// property of the config content, not of the boot: a client's `precondition`
// CAS compares against it, and a generation that restarted at 1 while the
// values survived would let a stale CAS silently succeed.

constexpr const char* kNvsNamespace = "valence";
constexpr const char* kNvsCfgKey    = "cfg";
constexpr uint32_t kCfgMagic   = 0x56434647u;  // "VCFG"
constexpr uint16_t kCfgVersion = 1;            // bump when StoredConfig changes
// Quiet time after the last applied change before the write lands. See the
// wear arithmetic in the file header.
constexpr uint32_t kCfgPersistDebounceMs = 2000;

struct CfgBlob {
    uint32_t     magic;
    uint16_t     version;
    uint16_t     cfg_gen;
    StoredConfig cfg;
};
static_assert(sizeof(CfgBlob) == 40, "CfgBlob layout moved: bump kCfgVersion");

bool inRange(float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; }

// An out-of-range blob is REJECTED WHOLE, never clamped into shape. Clamping
// here would be a machine-originated config change, which §4.2 says must bump
// cfg_gen -- at boot, against a generation we are in the middle of restoring.
// Falling back to the factory defaults is the one answer that needs no bump.
bool blobValid(const CfgBlob& b) {
    const StoredConfig& c = b.cfg;
    return b.magic == kCfgMagic && b.version == kCfgVersion && b.cfg_gen != 0
        && inRange(c.window_min,  0.0f, ceiling::rail_mm)
        && inRange(c.window_max,  0.0f, ceiling::rail_mm)
        && c.window_min < c.window_max
        && inRange(c.user_speed,  ceiling::speed_min, ceiling::speed_max)
        && inRange(c.user_accel,  ceiling::accel_min, ceiling::accel_max)
        && inRange(c.input_speed, ceiling::speed_min, ceiling::speed_max)
        && inRange(c.input_accel, ceiling::accel_min, ceiling::accel_max)
        && inRange(c.input_jerk,  ceiling::jerk_min,  ceiling::jerk_max)
        && inRange(c.max_rail,    ceiling::rail_min,  ceiling::rail_mm);
}

// false leaves both outputs untouched, which means the factory defaults stand.
bool loadStoredConfig(StoredConfig& cfg, uint16_t& gen) {
    nvs_handle_t h;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &h) != ESP_OK) return false;
    CfgBlob b{};
    size_t len = sizeof(b);
    const esp_err_t err = nvs_get_blob(h, kNvsCfgKey, &b, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof(b)) return false;
    if (!blobValid(b)) {
        GLOGW(kTag, "stored config rejected (magic/version/range) -- factory defaults stand");
        return false;
    }
    cfg = b.cfg;
    gen = b.cfg_gen;
    return true;
}

// Hub task only (T5). nvs_commit() blocks on the flash write; the debounce is
// what keeps that off the tick more than once per kCfgPersistDebounceMs.
void saveStoredConfig(const StoredConfig& cfg, uint16_t gen) {
    nvs_handle_t h;
    if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK) {
        GLOGW(kTag, "config persist: nvs_open failed");
        return;
    }
    const CfgBlob b{kCfgMagic, kCfgVersion, gen, cfg};
    // The write is TIMED because it is a flash write on the hub task: this
    // number is what says whether the debounce is enough, and an unmeasured
    // blocking call on a 5 ms tick is exactly the assumption that has cost
    // this project family a session before (memory-budget.md T27).
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = nvs_set_blob(h, kNvsCfgKey, &b, sizeof(b));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    const uint32_t us = uint32_t(esp_timer_get_time() - t0);
    if (err != ESP_OK) GLOGW(kTag, "config persist failed: %s", esp_err_to_name(err));
    else GLOGI(kTag, "config persisted, cfg_gen=%u, %lu us on the hub task",
               unsigned(gen), static_cast<unsigned long>(us));
}

// ---- the delegate ------------------------------------------------------------
// Two pure virtuals and nothing else is overridden that this board cannot back
// with real behavior. Every other channel the catalog advertises is either
// hub-owned (safety latch, pairing, session admin) or gated out by
// has_motion=false.
class ValenceDelegate final : public valence::HubDelegate {
public:
    void bindMinter(ValenceUiTokenMinter* m) { _minter = m; }

    // §12.2. A tokenless HELLO is a WORKING STATE, not a failure: it lands at
    // WATCH, which can subscribe to everything and use the role-exempt stop
    // and estop ops. Only a live single-use /uitoken credential upgrades to
    // CONTROL, and never to configure -- a browser-borne credential must not
    // be able to re-key the trust ledger. The trust ledger itself is the other
    // door and is the hub's own (pairing); this override covers /uitoken only.
    AccessLevel validateToken(std::span<const std::byte> instance_id,
                              std::span<const std::byte> token, bool hasToken) override {
        (void)instance_id;
        if (!hasToken || _minter == nullptr) return AccessLevel::watch;
        return _minter->consume(token) ? AccessLevel::control : AccessLevel::watch;
    }

    Ret applyIntent(uint16_t channel_id, const IntentValueMap& requested, AccessLevel role,
                    bool& cfgChanged) override {
        (void)role;
        if (channel_id == ch::move) return applyMove(requested);
        if (channel_id == ch::home) return applyHome(requested);
        if (channel_id == ch::modes_set || channel_id == ch::kinetic_set) {
            // HONEST OR ABSENT. Both writers are advertised because their
            // read-side cards are real, but nothing on this board applies
            // either: the modes name a drive backend and a homing style that
            // do not exist, and the kinetic tuning is not wired to a live
            // setter. Their cards therefore publish an all-zero enabled_mask
            // and their writes NACK, which is one statement, not two. An echo
            // of a value the machine did not take is the ground-truth defect
            // this refusal exists to avoid (bd val-091.11).
            return Ret::err(NackCode::UNSUPPORTED_OP);
        }
        if (channel_id != ch::config_set) {
            // Every op on 0x0005 the hub does not handle itself (stop / hold /
            // pause / resume / override / bypass). The library's contract is
            // that an unimplemented op returns UNSUPPORTED_OP so the hub
            // latches NOTHING. estop and estop_clear never reach here -- the
            // hub owns both -- so the red button works regardless.
            return Ret::err(NackCode::UNSUPPORTED_OP);
        }

        const auto* f1 = findField(requested, 1);  // window_min
        const auto* f2 = findField(requested, 2);  // window_max
        const auto* f3 = findField(requested, 3);  // user_speed
        const auto* f4 = findField(requested, 4);  // user_accel
        const auto* f5 = findField(requested, 5);  // input_speed
        const auto* f6 = findField(requested, 6);  // input_accel
        const auto* f7 = findField(requested, 7);  // input_jerk
        const auto* f8 = findField(requested, 8);  // max_rail

        StoredConfig next = _cfg;
        if (f1) next.window_min  = clampf(fieldF32(f1, next.window_min),  0.0f, ceiling::rail_mm);
        if (f2) next.window_max  = clampf(fieldF32(f2, next.window_max),  0.0f, ceiling::rail_mm);
        if (f3) next.user_speed  = clampf(fieldF32(f3, next.user_speed),  ceiling::speed_min, ceiling::speed_max);
        if (f4) next.user_accel  = clampf(fieldF32(f4, next.user_accel),  ceiling::accel_min, ceiling::accel_max);
        if (f5) next.input_speed = clampf(fieldF32(f5, next.input_speed), ceiling::speed_min, ceiling::speed_max);
        if (f6) next.input_accel = clampf(fieldF32(f6, next.input_accel), ceiling::accel_min, ceiling::accel_max);
        if (f7) next.input_jerk  = clampf(fieldF32(f7, next.input_jerk),  ceiling::jerk_min, ceiling::jerk_max);
        if (f8) next.max_rail    = clampf(fieldF32(f8, next.max_rail),    ceiling::rail_min, ceiling::rail_mm);

        // The ONE refusal that is value VALIDATION, not a clamp: an inverted
        // window has no legal nearest value, so it is rejected rather than
        // silently reordered.
        if (next.window_min >= next.window_max) return Ret::err(NackCode::INVALID_VALUE);

        // RFC-002 as tightened for v1.0: cfgChanged means CHANGED, never merely
        // ACCEPTED. A value-identical write still gets its post-clamp ECHO.
        cfgChanged = !(next == _cfg);
        _cfg = next;
        if (cfgChanged) _cfgDirty = true;

        // Key-complete ECHO: every key the client sent comes back with the
        // value the hub actually holds (SPEC §9.3).
        IntentValueMap applied{};
        uint32_t n = 0;
        if (f1) applied.fields[n++] = {1, IntentValue::ofF32(_cfg.window_min)};
        if (f2) applied.fields[n++] = {2, IntentValue::ofF32(_cfg.window_max)};
        if (f3) applied.fields[n++] = {3, IntentValue::ofF32(_cfg.user_speed)};
        if (f4) applied.fields[n++] = {4, IntentValue::ofF32(_cfg.user_accel)};
        if (f5) applied.fields[n++] = {5, IntentValue::ofF32(_cfg.input_speed)};
        if (f6) applied.fields[n++] = {6, IntentValue::ofF32(_cfg.input_accel)};
        if (f7) applied.fields[n++] = {7, IntentValue::ofF32(_cfg.input_jerk)};
        if (f8) applied.fields[n++] = {8, IntentValue::ofF32(_cfg.max_rail)};
        applied.count = n;
        return Ret::ok(applied);
    }

    // ---- 0x3100 move ---------------------------------------------------------
    // MANUAL source: the wire operator is the one driving. The arbiter lets a
    // Manual intent through an unhomed machine (the push-to-home case a local
    // button would use); this board has no such button, so the WIRE door is
    // gated on homed here. force_home is that door (operator ruling
    // 2026-09-21), and a refusal carries its reason rather than a count.
    Ret applyMove(const IntentValueMap& requested) {
        const MotionCensus c = motionCensus();
        if (c.estop) return Ret::err(NackCode::ESTOP_ACTIVE);
        if (!c.homed) return Ret::err(NackCode::NOT_HOMED);

        MotionIntent in;
        in.source    = MotionSource::Manual;
        in.target_mm = fieldF32(findField(requested, 1), 0.0f);
        if (!std::isfinite(in.target_mm)) return Ret::err(NackCode::INVALID_VALUE);
        if (!motionSubmit(in)) return Ret::err(NackCode::INTERLOCK);

        // Ground truth: echo the post-clamp position, against the SAME rail the
        // arbiter clamps a Manual intent to. `bypass` echoes false always --
        // nothing here bypasses anything, and echoing the request back would
        // report a capability the machine does not have.
        IntentValueMap applied{};
        applied.count = 2;
        applied.fields[0] = {1, IntentValue::ofF32(clampf(in.target_mm, 0.0f, c.rail_mm))};
        applied.fields[1] = {2, IntentValue::ofBool(false)};
        return Ret::ok(applied);
    }

    // ---- 0x3101 home ---------------------------------------------------------
    Ret applyHome(const IntentValueMap& requested) {
        const uint64_t op = fieldU64(findField(requested, 1), 0);
        IntentValueMap applied{};
        switch (op) {
            case 1:  // real homing
                // There is no motor and no encoder on this board, so a homing
                // cycle has nothing to feel for. Saying so once is the whole
                // handling; op 2 is how this machine becomes homed.
                GLOGW_EVERY_MS(60000, kTag,
                               "home op 1 refused: no drive and no encoder on this board, "
                               "nothing to home against -- use force_home (op 2)");
                return Ret::err(NackCode::UNSUPPORTED_OP);

            case 2: {  // force_home {stroke}
                // *** HAZARD, RFC-025. The hazard note lives on motionForceHome()
                // in ValenceMotion.h; do not restate it (C-1). What matters
                // HERE is the lockstep: the arbiter's latch drops inside that
                // call, and the hub's own ESTOP bit is dropped by the hub task
                // on the next tick, because clearEstop() broadcasts and this
                // runs inside the hub's own intent dispatch.
                const float asked = fieldF32(findField(requested, 2), 250.0f);
                const float stroke = motionForceHome(asked);
                _clearLatch = true;
                applied.count = 2;
                applied.fields[0] = {1, IntentValue::ofU64(2)};
                applied.fields[1] = {2, IntentValue::ofF32(stroke)};
                return Ret::ok(applied);
            }

            case 3:  // clear_override
                // On a machine that can really home, this returns it to real
                // homing. Here op 1 does not exist, so "back to real homing"
                // has no state to return to and accepting it would echo a
                // transition that did not happen.
                return Ret::err(NackCode::UNSUPPORTED_OP);

            default:
                return Ret::err(NackCode::INVALID_VALUE);
        }
    }

    // §11.4: the sources this machine has. move is the operator's hand, both
    // c2h motion streams are ONE machine-driven source -- a client uses 0x2100
    // or 0x2101, never both, and ownership is what enforces that.
    std::optional<uint8_t> sourceForChannel(uint16_t channel_id) override {
        if (channel_id == ch::move) return uint8_t(MotionSource::Manual);
        if (channel_id == ch::motion_input || channel_id == ch::motion_segment)
            return uint8_t(MotionSource::Stream);
        return std::nullopt;
    }

    // §11.2 (b): the machine-domain precondition the library cannot see. The
    // emitter's steering word IS that answer -- 0 means parked, and the engine
    // resets itself one motion tick after the latch, so busy falls too.
    // This is also the ONE hook the hub calls on the clear path and the hub
    // guarantees the clear proceeds iff it returns true, so dropping the
    // arbiter's latch here keeps both sides in lockstep. Clearing never
    // rehomes: homed stays false and motion stays refused until force_home.
    bool canClearEstop() override {
        const MotionCensus c = motionCensus();
        if (c.busy || c.step_q8 != 0) return false;
        motionEstopClear();
        return true;
    }

    // §11.2: motion stops before protocol bookkeeping. The emitter is parked on
    // THIS task inside motionEstop(), before this returns, so the stop precedes
    // the latch as the spec requires.
    void onEstop(uint8_t cause, uint8_t origin) override {
        motionEstop();
        GLOGW(kTag, "ESTOP latched: cause=%u origin=%u", unsigned(cause), unsigned(origin));
    }

    // ---- 0x2100 / 0x2101 stream ingress --------------------------------------
    // Runs on the hub task, synchronously inside Hub::update(). The hub has
    // already validated the §5.4 caps, the granted rate, ownership and the
    // deadman (hub.hpp's contract on this method); this decodes and submits and
    // re-checks none of it. Decoding is BY FIXED OFFSET against the catalog's
    // own 0x2100 (4 B point) / 0x2101 (6 B timed segment) field order -- the
    // same convention the publishers above encode with.
    void onStreamBundle(uint16_t channel_id, uint32_t session_id,
                        const valence::BundleView& bundle) override {
        const bool isSegment = (channel_id == ch::motion_segment);
        if (channel_id != ch::motion_input && !isSegment) return;

        // RFC-030: the session's GRANTED (post-curve-policy) family, looked up
        // once per bundle. Chase points never carry one -- the family is a
        // waveform-reconstruction concept.
        const uint8_t curveFamily =
            (isSegment && _hub != nullptr) ? _hub->publishCurveFamily(session_id, channel_id) : 0;

        // t_base/t_off are u32 HUB-us, the same wrapping domain EspClock reads
        // (§7.2). now64 stays the FULL 64-bit reading so the anchor never wraps
        // itself; only the WIRE stamp being resolved against it does.
        const int64_t now64 = esp_timer_get_time();
        const uint32_t now32 = uint32_t(uint64_t(now64) & 0xFFFFFFFFull);

        uint32_t dropped = 0;
        uint32_t farClamped = 0;
        const uint8_t n = bundle.sampleCount();
        for (uint8_t i = 0; i < n; ++i) {
            // Nearest-window resolve (§7.2): a wrap-aware signed subtract, safe
            // because the wire stamp is near now by construction (the bundle
            // span is capped far under the 32-bit wrap).
            int32_t delta = int32_t(bundle.sampleTimeUs(i) - now32);
            if (delta > kStreamFarFutureUs) { delta = kStreamFarFutureUs; ++farClamped; }
            if (delta < 0) delta = 0;

            const auto sample = bundle.sample(i);
            const float norm = float(valence::getU16(sample.subspan(0, 2))) / 10000.0f;

            MotionIntent in;
            in.source    = MotionSource::Stream;
            in.target_mm = _cfg.window_min + norm * (_cfg.window_max - _cfg.window_min);
            in.anchor_us = uint64_t(now64 + int64_t(delta));

            if (isSegment) {
                const uint16_t durMs = valence::getU16(sample.subspan(2, 2));
                const int16_t endV = int16_t(valence::getU16(sample.subspan(4, 2)));
                if (durMs == 0) { ++dropped; continue; }  // durationless points belong on 0x2100
                in.duration_us  = uint32_t(durMs) * 1000u;
                in.curve_family = curveFamily;
                // -32768 is the NO-END-VELOCITY sentinel: 0 is a legitimate
                // slope (a reversal ends AT rest), so 0 cannot mean absent.
                if (endV != kSegNoEndVel) {
                    in.end_vel_mm_s = float(endV) / 1000.0f * (_cfg.window_max - _cfg.window_min);
                    in.has_end_vel  = true;
                }
            } else {
                const int16_t vel = int16_t(valence::getU16(sample.subspan(2, 2)));
                in.end_vel_mm_s = float(vel) / 1000.0f * (_cfg.window_max - _cfg.window_min);
                in.has_end_vel  = (vel != 0);
            }
            if (!motionSubmit(in)) ++dropped;
        }

        motionNoteStream(1, n, dropped);
        if (farClamped) {
            GLOGW_EVERY_MS(2000, kTag,
                           "motion stream: %u sample(s) clamped from a far-future t_off "
                           "(missed CLOCK resync on the client?)", unsigned(farClamped));
        }
    }

    void bindHub(valence::Hub* h) { _hub = h; }
    bool takeClearLatch() {
        const bool v = _clearLatch;
        _clearLatch = false;
        return v;
    }

    void onSessionJoined(uint32_t session_id) override {
        GLOGI(kTag, "session %lu joined", static_cast<unsigned long>(session_id));
    }
    void onSessionLeft(uint32_t session_id) override {
        GLOGI(kTag, "session %lu left", static_cast<unsigned long>(session_id));
    }

    // Boot adoption from NVS. Deliberately NOT an intent and NOT a change: no
    // ECHO, no dirty flag, no cfg_gen bump. Adoption must never transmit --
    // the persisted generation is restored separately and the retained 0x1000
    // push that follows IS the announcement.
    void adoptConfig(const StoredConfig& c) { _cfg = c; _cfgDirty = false; }

    const StoredConfig& config() const { return _cfg; }
    bool takeConfigDirty() {
        const bool d = _cfgDirty;
        _cfgDirty = false;
        return d;
    }

private:
    StoredConfig _cfg{};
    bool _cfgDirty = false;
    // force_home cleared the arbiter's latch; the hub's own ESTOP bit is
    // dropped by the hub task on the next tick. DEFERRED on purpose:
    // Hub::clearEstop() publishes a safety snapshot and broadcasts, and
    // applyIntent runs inside the hub's own intent dispatch.
    bool _clearLatch = false;
    ValenceUiTokenMinter* _minter = nullptr;
    valence::Hub* _hub = nullptr;
};

// ---- the PSRAM-resident box --------------------------------------------------
// MEMBER ORDER IS CONSTRUCTION ORDER AND IT IS LOAD-BEARING: the catalog, the
// clock, the rng and the delegate must all be final before the Hub is built,
// because the Hub constructor encodes the catalog and draws boot_id from the
// rng right there. The Hub sits in a std::optional so the catalog can be
// FILLED between the two -- an init-list construction could not.
// The WS port rides along in PSRAM for its RX rings (5 slots x 32 x 512 B =
// 82 KB): the producer is the httpd TASK, never an ISR, so external memory is
// legal here. Do not move ISR-reachable state here by analogy.
struct HubBox {
    valence::Catalog32 catalog{};
    EspClock clock{};
    EspRandom rng{};
    ValenceDelegate delegate{};
    std::optional<valence::Hub> hub{};
    ValenceWsPort port{};
    ValenceUiTokenMinter minter{};
};

HubBox* g_box = nullptr;
TaskHandle_t g_hubTask = nullptr;
uint32_t g_ticks = 0;
uint32_t g_lastStatusMs = 0;
StoredConfig g_lastPublishedCfg{};
bool g_cfgEverSent = false;
// Debounced NVS write-through. Armed by every applied change, re-armed by the
// next one, so a slider drag costs ONE write kCfgPersistDebounceMs after the
// operator lets go.
bool g_cfgPersistArmed = false;
uint32_t g_cfgPersistDueMs = 0;
uint32_t g_endpointIpv4 = 0;
// Written by main's slow loop, read by the hub task. One byte, relaxed: a
// stale reading is a stale reading either way, and nothing orders against it.
std::atomic<int8_t> g_linkRssi{0};

// ---- retained STATE ----------------------------------------------------------
// EVERY STATE channel the catalog advertises is published here at boot with
// its truthful at-rest value. An advertised-but-never-published STATE channel
// leaves a subscriber holding "no idea" where the protocol promised it a
// value, and Phosphor sits at 'syncing' forever with nothing to report.
// The hub seeds 0x0003 safety, 0x000A pending-pairing and 0x000D roster
// itself (hub_impl.hpp's constructor); these are the rest.

void publishControlOwner() {
    // 4 x {source u8, owner u32} ascending, exactly Hub::buildControlOwnerPayload.
    // Every source unowned at boot, which is the truth. The hub republishes
    // this channel itself on every ownership transition; this call is the SEED
    // that keeps a subscriber from holding "no idea" before the first one.
    std::array<std::byte, 20> buf{};
    std::span<std::byte> s(buf);
    for (uint8_t i = 0; i < 4; ++i) {
        valence::putU8(s.subspan(size_t(i) * 5, 1), i);
        valence::putU32(s.subspan(size_t(i) * 5 + 1, 4), 0);
    }
    g_box->hub->publishState(valence::channels::control_owner, s);
}

void publishHubStatus() {
    // 4+4+1+1+4 = 14 B, matching the 0x0007 layout in ValenceCatalog.h.
    // RSSI is PUSHED IN from main's slow loop, never read here -- see
    // hubSetLinkRssi() in ValenceHub.h for the 151 ms that buys.
    const int8_t rssi = g_linkRssi.load(std::memory_order_relaxed);

    std::array<std::byte, 14> buf{};
    std::span<std::byte> s(buf);
    valence::putU32(s.subspan(0, 4), uint32_t(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    valence::putU32(s.subspan(4, 4), uint32_t(esp_timer_get_time() / 1000000));
    valence::putU8(s.subspan(8, 1), uint8_t(rssi));
    valence::putU8(s.subspan(9, 1), uint8_t(g_box->hub->sessionCount()));
    valence::putU32(s.subspan(10, 4), g_box->hub->logDropped());
    g_box->hub->publishState(valence::channels::hub_status, s);
}

void publishMachineConfig() {
    // 37 B, matching the 0x1000 layout in ValenceCatalog.h.
    const StoredConfig& c = g_box->delegate.config();
    std::array<std::byte, 37> buf{};
    std::span<std::byte> s(buf);
    valence::putF32(s.subspan(0, 4), c.window_min);
    valence::putF32(s.subspan(4, 4), c.window_max);
    valence::putF32(s.subspan(8, 4), c.user_speed);
    valence::putF32(s.subspan(12, 4), c.user_accel);
    valence::putF32(s.subspan(16, 4), c.input_speed);
    valence::putF32(s.subspan(20, 4), c.input_accel);
    valence::putF32(s.subspan(24, 4), c.max_rail);
    valence::putF32(s.subspan(28, 4), c.input_jerk);
    // enabled_mask: all eight limits are writable at all times on this hub.
    // Nothing refuses a config-set; out-of-range values clamp, which is what
    // min/max is for. A bit held low here would gray a control the machine
    // would in fact accept.
    valence::putU8(s.subspan(32, 1), 0xFF);
    // measured_stroke: 0 means NOT MEASURED. force_home ASSERTS a stroke that
    // nothing measured, and the arbiter's rail is where that assertion lives;
    // reporting it here would dress an assertion as a measurement. This board
    // has no way to measure a stroke, so the field is 0 forever.
    valence::putF32(s.subspan(33, 4), 0.0f);
    g_box->hub->publishState(ch::machine_config, s);
    g_lastPublishedCfg = c;
    g_cfgEverSent = true;
}

// ---- the motion plane's retained STATE ---------------------------------------
// Every channel below reads ONE motionCensus(), so no two of them can disagree
// about the same instant. The byte layouts mirror ValenceCatalog.h's field
// order for each id exactly; that mirroring is the whole contract (there is no
// packed struct to static_assert against).

// A layout whose hand-count disagrees with its buffer is a SILENT FIELD SHIFT:
// the writers stop when the buffer runs out and the tail goes out as zeros,
// which renders as plausible numbers in the wrong columns. This is the check that
// fails if any layout below is miscounted.
void publishPacked(uint16_t id, std::span<const std::byte> buf, size_t written) {
    if (written != buf.size()) {
        GLOGE_EVERY_MS(5000, kTag, "channel %04x packed %u of %u B -- layout miscounted",
                       unsigned(id), unsigned(written), unsigned(buf.size()));
    }
    g_box->hub->publishState(id, buf);
}

uint32_t g_lastMotionMs = 0;
uint32_t g_lastPlanMs = 0;
uint32_t g_lastSlowMs = 0;

void publishMotion(const MotionCensus& m) {
    std::array<std::byte, 9> buf{};
    size_t n = 0;
    packU16(buf, n, wireU16(m.position_mm, 100.0f));   // pos_10um
    // tgt_10um is the plan's AIM, m.target_mm, never m.plan_mm. plan_mm is
    // where the plan is right now, which tracks pos within a step or two, so
    // publishing it here draws the target marker on top of the position and a
    // client can never see the planner leading.
    packU16(buf, n, wireU16(m.target_mm, 100.0f));     // tgt_10um
    packI16(buf, n, wireI16(m.velocity_mm_s, 10.0f));  // speed
    // flags: homed, homing, gen_running, paused, override, estop, stream.
    // homing and gen_running are permanently 0 and that is the truth, not a
    // stub: there is no homing cycle and no pattern generator on this board.
    packU8(buf, n, uint8_t((m.homed ? 0x01u : 0u) | (m.paused ? 0x08u : 0u) |
                 (m.estop ? 0x20u : 0u) | (m.stream ? 0x40u : 0u)));
    packU16(buf, n, wireU16(m.demand_mm, 100.0f));     // raw_10um: the asked position
    // AT RATE, not on change. An on-change gate looks like an economy and is a
    // ground-truth hazard on a hero channel: a machine at rest stops pushing,
    // so a subscriber that joined late, or whose retained value never landed,
    // has nothing to show and cannot tell a still carriage from a dead feed.
    publishPacked(ch::motion, buf, n);
}

void publishPlanStrip(const MotionCensus& m) {
    std::array<std::byte, 18> buf{};
    size_t n = 0;
    // flags: active, live_mode, grad_mode. live_mode and grad_mode named a
    // legacy interpolator split that has no counterpart in this engine.
    packU8(buf, n, m.busy ? 0x01u : 0u);
    packU8(buf, n, m.mode);                            // style: idle/waveform/chase/settle
    packU16(buf, n, wireU16(m.plan_start, 10000.0f));
    packU16(buf, n, wireU16(m.plan_end, 10000.0f));
    packU16(buf, n, wireU16(m.plan_cur, 10000.0f));
    packI16(buf, n, wireI16(m.plan_vel, 1000.0f));
    packU32(buf, n, m.plan_duration_us);
    packU32(buf, n, m.plan_elapsed_us);
    publishPacked(ch::plan_strip, buf, n);
}

void publishMotionDiag(const MotionCensus& m) {
    std::array<std::byte, 92> buf{};
    size_t n = 0;
    packU32(buf, n, m.plans);
    packU32(buf, n, m.failures);
    packU32(buf, n, m.anomalies);
    packU8(buf, n, m.mode);
    packU8(buf, n, m.plan_kind);
    for (uint32_t k : m.anom) packU32(buf, n, k);
    packU32(buf, n, m.plan_us_last);
    packU32(buf, n, m.plan_us_max);
    packF32(buf, n, m.plan_us_avg);
    packU32(buf, n, m.stream_bundles);
    packU32(buf, n, m.stream_samples);
    // sync_enqueued: every decoded sample that was not dropped reached the
    // arbiter, because the delegate submits inside the same loop that counts.
    packU32(buf, n, m.stream_samples - m.stream_dropped);
    packU32(buf, n, m.stream_dropped);
    // sync_seg_bundles is not separated here: both stream channels land in one
    // counter, and splitting it would need a second pair the census does not
    // carry. It reads 0, which understates rather than invents.
    packU32(buf, n, 0);
    packU16(buf, n, 0);                                // reset_gen: nothing resets these
    publishPacked(ch::motion_diag, buf, n);
}

void publishOdometer(const MotionCensus& m) {
    std::array<std::byte, 20> buf{};
    size_t n = 0;
    packU32(buf, n, m.strokes);
    packF32(buf, n, m.distance_mm / 1000.0f);          // distance_m
    packF32(buf, n, m.peak_mm_s);
    packF32(buf, n, 0.0f);                             // energy_wh: no power monitor
    packU32(buf, n, uint32_t(esp_timer_get_time() / 1000));
    publishPacked(ch::odometer, buf, n);
}

// Published ONCE at boot: nothing on this board changes any of it, and 0x3030
// NACKs every write, so a republish would carry no news.
void publishMachineModes() {
    std::array<std::byte, 6> buf{};
    size_t n = 0;
    packU8(buf, n, 0);   // blend_mode_reserved
    packU8(buf, n, 0);   // stream_speed_reserved
    packU8(buf, n, 0);   // overshoot_clamp: inert, off
    // enabled_mask 0: the machine accepts NONE of these. overshoot_clamp is
    // inert, the backend is fixed by what is soldered, and home_style picks
    // between two homing cycles neither of which exists here.
    packU8(buf, n, 0);
    packU8(buf, n, 2);   // motion_backend: quadrature, the LP-core emitter
    packU8(buf, n, 0);   // home_style: reported only because the field exists; mask is low
    publishPacked(ch::machine_modes, buf, n);
}

// The three kinetic cards. Read-only on this board (0x3120 NACKs), so their
// enabled_mask is 0 and they publish once at boot.
void publishKineticCards() {
    const MotionTuning t = motionTuning();
    {
        std::array<std::byte, 13> buf{};
        size_t n = 0;
        // The overrides are genuinely 0: this arbiter derives every ceiling
        // from the mm limit set, which is exactly what "0" means on this card.
        packF32(buf, n, 0.0f);   // jmax_ovr
        packF32(buf, n, 0.0f);   // vmax_ovr
        packF32(buf, n, 0.0f);   // amax_ovr
        packU8(buf, n, 0);       // enabled_mask
        publishPacked(ch::kinetic_limits, buf, n);
    }
    {
        std::array<std::byte, 20> buf{};
        size_t n = 0;
        packU8(buf, n, t.chase_ff ? 1 : 0);
        packU8(buf, n, t.chase_accel_ff ? 1 : 0);
        packF32(buf, n, t.chase_gain);
        packF32(buf, n, t.chase_lookahead);
        packU32(buf, n, t.chase_dense_us);     // scale 1000, unit ms: the wire carries us
        packU8(buf, n, t.chase_aim_extrap ? 1 : 0);
        packF32(buf, n, t.handoff_k);
        packU8(buf, n, 0);                     // enabled_mask
        publishPacked(ch::kinetic_chase, buf, n);
    }
    {
        std::array<std::byte, 16> buf{};
        size_t n = 0;
        packU8(buf, n, t.curve_policy);
        packU8(buf, n, t.infeasible_policy);
        packF32(buf, n, t.smooth_budget);
        packF32(buf, n, t.amplitude_budget);
        packU8(buf, n, t.blend_steps);
        packU32(buf, n, t.settle_grace_us);    // scale 1000, unit ms: the wire carries us
        packU8(buf, n, 0);                     // enabled_mask
        publishPacked(ch::kinetic_waveform, buf, n);
    }
}

// The stored config IS the arbiter's window and ceilings. One function so the
// two can never be set from different places and drift (C-1): boot adoption and
// every applied 0x3000 write both come through here.
void pushConfigToMotion(const StoredConfig& c) {
    motionSetWindow(c.window_min, c.window_max, c.max_rail);
    motionSetUserLimits(c.user_speed, c.user_accel);
    motionSetInputLimits(c.input_speed, c.input_accel, c.input_jerk);
}

// WELCOME keys 46/47 (RFC-046): the hub's own reachable endpoint. 0/0 omits
// both from the wire, so a client that connected before DHCP finished simply
// gets no hint; every later WELCOME-shaped message picks up the real value.
// Polled on the hub task because setEndpoint() is a Hub call and the hub is
// single-task.
void refreshEndpoint() {
    esp_netif_t* nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (nif == nullptr) return;
    esp_netif_ip_info_t ip{};
    if (esp_netif_get_ip_info(nif, &ip) != ESP_OK) return;
    if (ip.ip.addr == g_endpointIpv4) return;
    g_endpointIpv4 = ip.ip.addr;
    // esp_netif stores the address in network byte order; WELCOME key 47 is a
    // plain u32 host value, so swap once here rather than at every reader.
    g_box->hub->setEndpoint(kWsPort, __builtin_bswap32(ip.ip.addr));
}

// RFC-048: a durable cross-boot identity for WELCOME identity key 5. 0 means
// "never set" and the key is omitted, which a conforming client tolerates but
// cannot use to recognize this machine again -- so it is minted once from the
// hardware RNG and kept in NVS.
uint64_t loadOrMintInstanceId() {
    nvs_handle_t h;
    if (nvs_open("valence", NVS_READWRITE, &h) != ESP_OK) return 0;
    uint64_t id = 0;
    if (nvs_get_u64(h, "hub_iid", &id) != ESP_OK || id == 0) {
        do {
            id = (uint64_t(esp_random()) << 32) | esp_random();
        } while (id == 0);
        nvs_set_u64(h, "hub_iid", id);
        nvs_commit(h);
        GLOGI(kTag, "minted hub_instance_id %08lx%08lx",
              static_cast<unsigned long>(id >> 32), static_cast<unsigned long>(id & 0xFFFFFFFFu));
    }
    nvs_close(h);
    return id;
}

// ---- the hub task ------------------------------------------------------------
// CORE 1. The LP core owns motion and the PARLIO emitter is pure DMA, so HP
// core 1 carries nothing else; core 0 runs app_main and the esp_hosted SDIO
// service that the whole network path depends on. Keeping the hub's 5 ms tick
// and its bounded socket writes off the core that services the radio bridge is
// the same separation the S3 makes between comms and real time, drawn where
// this silicon actually puts the work.
void hubTask(void*) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
        ++g_ticks;
        const uint32_t nowMs = uint32_t(esp_timer_get_time() / 1000);

        // Port first: it performs the deferred attach/detach and re-arms the
        // per-tick BLOB pacing budget that update() is about to spend.
        g_box->port.loop(nowMs);
        g_box->hub->update(g_box->clock.nowUs());

        // force_home dropped the arbiter's latch inside applyIntent; the hub's
        // own ESTOP bit drops HERE, one tick later, because clearEstop()
        // publishes and broadcasts and applyIntent runs inside the hub's intent
        // dispatch. canClearEstop() still gates it, so the two never disagree.
        if (g_box->delegate.takeClearLatch() && g_box->hub->estopLatched()) {
            if (g_box->hub->clearEstop()) GLOGW(kTag, "ESTOP latch cleared by force_home");
            else GLOGW(kTag, "force_home could not clear the ESTOP latch: motion is not parked");
        }

        // The motion plane, from ONE census so no two channels disagree about
        // the same instant. 0x1100 publishes on change under its 60 Hz ceiling;
        // 0x1110 is a strip that is only news while a plan runs.
        const MotionCensus mo = motionCensus();
        if (uint32_t(nowMs - g_lastMotionMs) >= 33u) {
            g_lastMotionMs = nowMs;
            publishMotion(mo);
        }
        if (uint32_t(nowMs - g_lastPlanMs) >= 50u) {
            g_lastPlanMs = nowMs;
            publishPlanStrip(mo);
        }
        if (uint32_t(nowMs - g_lastSlowMs) >= 1000u) {
            g_lastSlowMs = nowMs;
            publishMotionDiag(mo);
            publishOdometer(mo);
        }

        if (g_box->delegate.takeConfigDirty() ||
            (g_cfgEverSent && !(g_lastPublishedCfg == g_box->delegate.config()))) {
            pushConfigToMotion(g_box->delegate.config());
            publishMachineConfig();
            // Re-armed, not accumulated: the write lands only after the changes
            // stop. cfg_gen is read at write time, by which point hub->update()
            // above has already applied this tick's bump (§4.2).
            g_cfgPersistArmed = true;
            g_cfgPersistDueMs = nowMs + kCfgPersistDebounceMs;
        }
        if (g_cfgPersistArmed && int32_t(nowMs - g_cfgPersistDueMs) >= 0) {
            g_cfgPersistArmed = false;
            saveStoredConfig(g_box->delegate.config(), g_box->hub->cfgGen());
        }
        if (uint32_t(nowMs - g_lastStatusMs) >= 1000u) {
            g_lastStatusMs = nowMs;
            publishHubStatus();
            refreshEndpoint();
        }

        geiger::drainToSinks();
    }
}

}  // namespace

// ---- public surface ----------------------------------------------------------

valence::Hub* hub() { return (g_box && g_box->hub) ? &*g_box->hub : nullptr; }

void hubSetLinkRssi(int8_t rssi) { g_linkRssi.store(rssi, std::memory_order_relaxed); }

HubCensus hubCensus() {
    HubCensus c{};
    if (!g_box || !g_box->hub) return c;
    for (size_t i = 0;; ++i) {
        const valence::HubSession* s = g_box->hub->sessionBySlot(i);
        if (s == nullptr) break;
        if (!s->occupied()) continue;
        if (s->state == valence::HubSessionState::STALE) ++c.parked;
        else ++c.sessions;
    }
    c.ticks = g_ticks;
    c.wsFrames = g_box->port.framesRx();
    c.wsDrops = g_box->port.drops();
    c.wsSockets = uint32_t(g_box->port.openSockets());
    c.uiSockets = uint32_t(g_box->minter.openSockets());
    c.stackFree = g_hubTask ? uint32_t(uxTaskGetStackHighWaterMark(g_hubTask)) : 0;
    return c;
}

bool hubBegin() {
    geiger::logBegin();

    // PSRAM, via placement-new (T2). The catalog alone is ~22 KB of pooled
    // field storage and the hub carries the session table and the retained
    // store; in .bss that is internal RAM the network stack allocates from at
    // runtime. The TASK STACK below stays internal on purpose.
    void* mem = heap_caps_malloc(sizeof(HubBox), MALLOC_CAP_SPIRAM);
    if (mem == nullptr) {
        GLOGE(kTag, "PSRAM alloc of %u bytes for the hub failed", unsigned(sizeof(HubBox)));
        geiger::drainToSinks();
        return false;
    }
    g_box = new (mem) HubBox();
    g_box->minter.begin();
    g_box->delegate.bindMinter(&g_box->minter);

    // Operator ruling 2026-09-21: this board acts like a normal machine with no
    // motor, no Modbus drive and no current sensor. The motion plane is REAL --
    // the arbiter, the engine and the LP emitter are all live -- so it is
    // advertised; the two absent subsystems are what stays gated.
    DeviceFeatures feat{};
    feat.has_motion  = true;
    feat.has_drive   = false;  // no Modbus drive on this board (val-091)
    feat.has_pattern = false;  // no pattern engine ported yet (val-091.12)
    if (!buildValenceCatalog(g_box->catalog, feat)) {
        GLOGE(kTag, "catalog build overflowed a Catalog32 pool");
        geiger::drainToSinks();
        return false;
    }

    // BEFORE the Hub exists, so the delegate is already holding stored truth
    // when the retained 0x1000 push below seeds the channel. A load that ran
    // after the first publish would make the first snapshot a lie any
    // subscriber has already adopted.
    StoredConfig stored{};
    uint16_t storedGen = 0;
    const bool haveStored = loadStoredConfig(stored, storedGen);
    if (haveStored) g_box->delegate.adoptConfig(stored);

    // The arbiter's window and ceilings ARE the stored config: pushed before
    // the hub exists so the first 0x1000 snapshot and the machine agree.
    pushConfigToMotion(g_box->delegate.config());

    g_box->hub.emplace(g_box->catalog, g_box->clock, g_box->rng, g_box->delegate);
    g_box->delegate.bindHub(&*g_box->hub);
    // cfg_gen survives the reboot with the values it belongs to (§4.2). The
    // library exposes advance-only (bumpConfigGeneration), which is correct for
    // its RFC-011 job, so the restore walks the u16 up to the stored value; it
    // terminates by wrapping and costs one increment per step, nothing more.
    if (haveStored) {
        while (g_box->hub->cfgGen() != storedGen) g_box->hub->bumpConfigGeneration();
        GLOGI(kTag, "config adopted from NVS, cfg_gen=%u", unsigned(storedGen));
    }
    if (g_box->hub->catalogEncodedBytes() == 0) {
        GLOGE(kTag, "catalog encoded to ZERO bytes -- it did not fit the hub scratch (%u B)",
              unsigned(valence::Hub::catalogScratchCapacity()));
        geiger::drainToSinks();
        return false;
    }
    g_box->hub->setIdentity(VALENCE_PRODUCT, FIRMWARE_VERSION, VALENCE_HUB_NAME);
    g_box->hub->setHubInstanceId(loadOrMintInstanceId());
    refreshEndpoint();

    publishControlOwner();
    publishMachineConfig();
    publishHubStatus();
    publishMachineModes();
    publishKineticCards();
    {
        // EVERY advertised STATE gets its truthful at-rest value before the
        // first client can subscribe. Without this a subscriber holds "no idea"
        // where the protocol promised it a value and sits at syncing forever.
        const MotionCensus mo = motionCensus();
        publishMotion(mo);
        publishPlanStrip(mo);
        publishMotionDiag(mo);
        publishOdometer(mo);
    }

    auto etag = g_box->hub->catalogEtag();
    GLOGI(kTag, "catalog: %u entries, %u B encoded (scratch %u B)",
          unsigned(g_box->catalog.count), unsigned(g_box->hub->catalogEncodedBytes()),
          unsigned(valence::Hub::catalogScratchCapacity()));
    GLOGI(kTag, "catalog etag: %02x%02x%02x%02x%02x%02x%02x%02x",
          unsigned(etag[0]), unsigned(etag[1]), unsigned(etag[2]), unsigned(etag[3]),
          unsigned(etag[4]), unsigned(etag[5]), unsigned(etag[6]), unsigned(etag[7]));
    GLOGI(kTag, "hub box %u B in PSRAM, boot_id=%08lx, %s",
          unsigned(sizeof(HubBox)), static_cast<unsigned long>(g_box->hub->bootId()),
          FIRMWARE_VERSION);

    if (!g_box->port.begin(&*g_box->hub, kWsPort)) {
        GLOGE(kTag, "WS port failed to start on :%u", unsigned(kWsPort));
        geiger::drainToSinks();
        return false;
    }
    // Non-fatal: a hub with no /uitoken still serves every watch-tier client
    // and every paired one. Losing the mint costs the browser onramp, not the
    // machine.
    if (!g_box->minter.attachRoutes()) GLOGW(kTag, "/uitoken unavailable");

    // Stack: internal by construction (plain xTaskCreatePinnedToCore). The size
    // and the measurement that set it live on kHubTaskStackBytes in ValenceHub.h.
    if (xTaskCreatePinnedToCore(hubTask, "ValenceHub", kHubTaskStackBytes, nullptr, 5,
                                &g_hubTask, 1) != pdPASS) {
        GLOGE(kTag, "hub task create failed");
        geiger::drainToSinks();
        return false;
    }
    geiger::drainToSinks();
    return true;
}

}  // namespace valence
