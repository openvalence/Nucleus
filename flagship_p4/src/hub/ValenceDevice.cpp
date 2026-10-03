// ValenceDevice -- the delegate and the retained STATE publishers, hardware-free
// Constraints:
// - See ValenceDevice.h for the hardware-free, single-task and construction
//   order rules.
// - The delegate is HONEST OR ABSENT: an intent it cannot really apply is
//   NACKed UNSUPPORTED_OP (0x0303, registry.yaml:609) and NEVER echoed. A
//   refusal that IS a machine state (e-stop latched, not homed) carries that
//   state's own code instead, so a client gets a reason it can act on.
// - EVERY advertised STATE channel is published at attach() with its truthful
//   at-rest value and kept truthful after. An advertised-but-never-published
//   STATE leaves a subscriber holding "no idea" where the protocol promised a
//   value, and a generic client sits at syncing forever. The hub seeds 0x0003
//   safety, 0x000A pending-pairing and 0x000D roster itself.
// - The persist debounce (kCfgPersistDebounceMs) is the wear bound on the P4's
//   NVS pages; the arithmetic lives on ValenceHub.cpp's file header. What a
//   blob holds is StoredState.h's and PatternPresetStore's; background_run is
//   in neither, on purpose (bd val-wcm, pending ruling).
// See: Valence SPEC.md §4.2, §6.3, §9.1, §9.3, §11.2, §11.4

#include "ValenceDevice.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string_view>

#include "geiger/geiger.h"
#include "motion/StreamIntent.h"
#include "motion/ValenceMotion.h"
#include "patterns/ValencePattern.h"
#include "system/ValenceButtons.h"
#include "system/ValenceDriveLink.h"
#include "system/ValenceEstopInput.h"
#include "system/ValenceMotorSwitch.h"

#include "valence/util/byte_io.hpp"

namespace valence {

// ---- anti-drift guards: catalog mirror vs the pattern generator ---------------
// ValenceCatalog.h stays library-only and cannot include the generator; this
// TU sees both, so it is where the mirror is nailed.
static_assert(kPresetCapacity == PatternPresetStore::kCapacity, "catalog preset capacity drifted");
static_assert(kPresetNameMax == PatternPresetStore::kNameMax, "catalog preset name_max drifted");
static_assert(kPresetPayloadBytes == PatternPresetStore::kPayloadBytes, "catalog preset per_item_max drifted");
static_assert(kApBaseCount == advpat::BASE_COUNT, "catalog modulator count drifted");
static_assert(kApPercentBaseCount == advpat::PERCENT_BASE_COUNT, "catalog percent knob count drifted");
static_assert(apBaseKey(advpat::DWELL_CREST) == 46 && apBaseKey(advpat::DWELL_TROUGH) == 47,
              "RFC-095 dwell writer keys drifted");
static_assert([] {
    for (size_t i = 0; i < kSourceLabels.size(); ++i)
        if (std::string_view(kSourceLabels[i]) != kMotionSourceNames[i]) return false;
    return kSourceLabels.size() == kMotionSourceNames.size();
}(), "catalog source labels drifted");

// A build without the capacity flags would get the library's defaults: a
// different Catalog32 from the board's, and no accessory budget at all.
#ifndef NUCLEUS_ACCESSORIES
#error "catalog capacity flags missing: apply flagship_p4/valence_capacity.cmake to this build"
#endif

CatalogHeadroom catalogHeadroom(const Catalog32& c, size_t encodedBytes) {
    CatalogHeadroom h;
    h.entries = uint16_t(Catalog32::kEntryCapacity - c.count);
    h.layout  = uint16_t(Catalog32::kLayoutCapacity - c.layoutUsed);
    h.schema  = uint16_t(Catalog32::kSchemaCapacity - c.schemaUsed);
    h.safe    = uint16_t(Catalog32::kSafeCapacity - c.safeUsed);
    const size_t floor = Hub::catalogScratchCapacity() * 4 / 5;
    h.bytes = encodedBytes < floor ? uint32_t(floor - encodedBytes) : 0;
    const size_t fit = std::min({size_t(h.entries) / NUCLEUS_ACCESSORY_ENTRIES,
                                 size_t(h.layout) / NUCLEUS_ACCESSORY_LAYOUT_FIELDS,
                                 size_t(h.schema) / NUCLEUS_ACCESSORY_SCHEMA_FIELDS,
                                 size_t(h.safe) / NUCLEUS_ACCESSORY_SAFE_FIELDS,
                                 size_t(h.bytes) / NUCLEUS_ACCESSORY_CATALOG_BYTES,
                                 size_t(NUCLEUS_ACCESSORIES)});
    h.accessories = uint8_t(fit);
    return h;
}

namespace {

using Ret = Result<IntentValueMap, NackCode>;

constexpr const char* kTag = "hub";

// SPEC 16.1 NACK details shared by more than one refusal. Each stays under
// nack_detail_max_bytes (48), so the hub never cuts one.
constexpr const char* kDetailEstop = "e-stop latched";
constexpr const char* kDetailPaused = "paused";
constexpr const char* kDetailReturning = "returning to the paused position";

// Quiet time after the last applied change before a blob's persist is due,
// the same for both blobs. It coalesces a burst (a slider drag streams
// 0x3120 writes; a rename follows a save) into ONE flash write after the
// operator lets go: a write per INTENT would put an NVS commit, and now and
// then a sector erase, on the hub task at the intent rate.
constexpr uint32_t kCfgPersistDebounceMs = 2000;

// A HOME hold's brake gets this long to reach rest before the ESTOP cuts
// power anyway. The arbiter brakes at the input decel, well under a second
// from the speed ceiling.
constexpr uint32_t kRebootBrakeMs = 2000;

// How long an EN-node motor switch fault waits for the e-stop reader to name
// it. The switch's 5 ms watch sees the e-stop's hardware stop first; the
// reader settles a press one 10 ms BoardIo sample, the button's NC-open to
// NO-closed travel and estop::kDebounceMs later, plus a hub tick. Covers
// 50 ms of travel. Only the protocol latch waits: power is already off and
// the emitter parked.
// TODO(val-091.69): from a scoped slam of the real button.
constexpr uint32_t kEstopNameMs = 100;
static_assert(kEstopNameMs > estop::kDebounceMs + 25, "the naming window cannot cover a press");

// Registry safety_causes (registry.yaml:574): a pressed button is the
// operator's act, `user` ("physical button"); a missing or miswired input is
// the hub's own detection, `fault`.
uint8_t estopCause(estop::Contacts c) {
    return c == estop::Contacts::pressed ? safety_causes::user : safety_causes::fault;
}

// SPEC 16.1 detail for a release the e-stop contacts refuse (SPEC 11.2 (a):
// the cause is not resolved while they read a stop); nullptr when they allow
// it. A reading not yet known refuses: a stop nobody can see is not resolved.
const char* estopRefusal(const estop::Reading& r) {
    if (!r.known) return "e-stop input not read";
    switch (r.state) {
        case estop::Contacts::released:     return nullptr;
        case estop::Contacts::pressed:      return "e-stop pressed at the machine";
        case estop::Contacts::unplugged:    return "no e-stop found at the machine";
        case estop::Contacts::wiring_fault: return "e-stop wiring fault at the machine";
    }
    return "e-stop input not read";
}

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---- packed-layout writers ---------------------------------------------------
// The STATE layouts below are long and every offset in them is wire-visible, so
// nothing spells an offset twice: a miscounted subspan is a silent field shift
// that renders as plausible numbers in the wrong columns.
// Free functions over a caller-owned buffer and a caller-owned cursor, not a
// writer object: a cursor that HELD the span would be a borrowed member, which
// the safe subset forbids outright (cpp-safety.md).
void packU8(std::span<std::byte> o, size_t& n, uint8_t v)   { n += putU8(o.subspan(n), v); }
void packU16(std::span<std::byte> o, size_t& n, uint16_t v) { n += putU16(o.subspan(n), v); }
void packI16(std::span<std::byte> o, size_t& n, int16_t v)  { packU16(o, n, uint16_t(v)); }
void packU32(std::span<std::byte> o, size_t& n, uint32_t v) { n += putU32(o.subspan(n), v); }
void packF32(std::span<std::byte> o, size_t& n, float v)    { n += putF32(o.subspan(n), v); }

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

const IntentValueField* findField(const IntentValueMap& m, uint8_t key) {
    for (uint32_t i = 0; i < m.count; ++i) {
        if (m.fields[i].key == key) return &m.fields[i];
    }
    return nullptr;
}

uint64_t fieldU64(const IntentValueField* f, uint64_t dflt) {
    if (!f) return dflt;
    switch (f->value.kind) {
        case IntentValue::Kind::U64: return f->value.u64_val;
        case IntentValue::Kind::I64: return f->value.i64_val < 0 ? dflt : uint64_t(f->value.i64_val);
        default:                     return dflt;
    }
}

// A numeric intent value as a float, or nullopt for a non-numeric or
// non-finite one. clampf() passes NaN straight through, so a clamp is only as
// good as this check in front of it.
std::optional<float> numberOf(const IntentValueField* f) {
    if (!f) return std::nullopt;
    float v = 0.0f;
    switch (f->value.kind) {
        case IntentValue::Kind::F32: v = f->value.f32_val; break;
        case IntentValue::Kind::U64: v = float(f->value.u64_val); break;
        case IntentValue::Kind::I64: v = float(f->value.i64_val); break;
        default: return std::nullopt;
    }
    if (!std::isfinite(v)) return std::nullopt;
    return v;
}

// A uint schema field: clamped, then rounded to the nearest whole value.
uint32_t wholeIn(float v, float lo, float hi) { return uint32_t(clampf(v, lo, hi) + 0.5f); }

// A bool schema field: a CBOR bool, or a number read as 0 / nonzero. nullopt
// for anything else, which the caller NACKs INVALID_VALUE.
std::optional<bool> boolOf(const IntentValueField* f) {
    if (!f) return std::nullopt;
    if (f->value.kind == IntentValue::Kind::Bool) return f->value.bool_val;
    const std::optional<float> v = numberOf(f);
    if (!v) return std::nullopt;
    return *v != 0.0f;
}

// A 0..100 knob as the generator stores it: a whole number, clamped. The
// generator's own setters clamp again to each control's live bounds.
int knobOf(float v) { return int(wholeIn(v, 0.0f, 100.0f)); }

// One 0x3000 key into `c`, clamped. The window arrives in the client frame
// (RFC-088) and is stored physical: on a flipped rail the client's low bound
// is the physical high one.
void setConfigKey(StoredConfig& c, uint8_t key, float v, float rail, bool flipped) {
    switch (key) {
        case 1: {
            const float lo = clampf(v, 0.0f, ceiling::rail_mm);
            if (flipped) c.window_max = clampf(rail - lo, 0.0f, ceiling::rail_mm);
            else c.window_min = lo;
            break;
        }
        case 2: {
            const float hi = clampf(v, 0.0f, ceiling::rail_mm);
            if (flipped) c.window_min = clampf(rail - hi, 0.0f, ceiling::rail_mm);
            else c.window_max = hi;
            break;
        }
        case 3: c.jog_speed   = clampf(v, ceiling::speed_min, ceiling::speed_max); break;
        case 4: c.jog_accel   = clampf(v, ceiling::accel_min, ceiling::accel_max); break;
        case 5: c.input_speed = clampf(v, ceiling::speed_min, ceiling::speed_max); break;
        case 6: c.input_accel = clampf(v, ceiling::accel_min, ceiling::accel_max); break;
        case 7: c.input_jerk  = clampf(v, ceiling::jerk_min, ceiling::jerk_max); break;
        case 8: c.max_rail    = clampf(v, ceiling::rail_min, ceiling::rail_mm); break;
        default: break;
    }
}

// The 0x3120 schema's keys in wire order; 4, 5, 15 and 19 are released.
constexpr std::array<uint8_t, 16> kTuningKeys{1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18, 20};
// chase_dense: a live samples grant commits to it (applyTuning()).
constexpr uint8_t kChaseDenseKey = 10;

// One 0x3120 key into `t`, clamped into tuning_bounds; the value as the ECHO
// carries it. nullopt for a key outside the schema.
std::optional<IntentValue> setTuningKey(MotionTuning& t, uint8_t key, float v) {
    namespace b = tuning_bounds;
    switch (key) {
        case 1:  t.jmax_ovr = clampf(v, 0.0f, b::jmax_ovr_max);  return IntentValue::ofF32(t.jmax_ovr);
        case 2:  t.vmax_ovr = clampf(v, 0.0f, b::vmax_ovr_max);  return IntentValue::ofF32(t.vmax_ovr);
        case 3:  t.amax_ovr = clampf(v, 0.0f, b::amax_ovr_max);  return IntentValue::ofF32(t.amax_ovr);
        case 6:  t.chase_ff = wholeIn(v, 0.0f, 1.0f) != 0;         return IntentValue::ofU64(t.chase_ff);
        case 7:  t.chase_accel_ff = wholeIn(v, 0.0f, 1.0f) != 0;   return IntentValue::ofU64(t.chase_accel_ff);
        case 8:  t.chase_gain = clampf(v, 0.0f, b::chase_gain_max);     return IntentValue::ofF32(t.chase_gain);
        case 9:  t.chase_lookahead = clampf(v, 0.0f, b::lookahead_max); return IntentValue::ofF32(t.chase_lookahead);
        case 10:  // ms on the wire, us in the engine
            t.chase_dense_us = uint32_t(clampf(v, b::dense_ms_min, b::dense_ms_max) * 1000.0f + 0.5f);
            return IntentValue::ofF32(float(t.chase_dense_us) / 1000.0f);
        case 11: t.chase_aim_extrap = wholeIn(v, 0.0f, 1.0f) != 0; return IntentValue::ofU64(t.chase_aim_extrap);
        case 12: t.handoff_k = clampf(v, 0.0f, b::handoff_k_max);   return IntentValue::ofF32(t.handoff_k);
        case 13: t.curve_policy = uint8_t(wholeIn(v, 0.0f, float(b::curve_policy_max)));
                 return IntentValue::ofU64(t.curve_policy);
        case 14: t.infeasible_policy = uint8_t(wholeIn(v, 0.0f, float(b::infeasible_max)));
                 return IntentValue::ofU64(t.infeasible_policy);
        case 16: t.smooth_budget = clampf(v, 0.0f, b::budget_max);    return IntentValue::ofF32(t.smooth_budget);
        case 17: t.amplitude_budget = clampf(v, 0.0f, b::budget_max); return IntentValue::ofF32(t.amplitude_budget);
        case 18: t.blend_steps = uint8_t(wholeIn(v, float(b::blend_steps_min), float(b::blend_steps_max)));
                 return IntentValue::ofU64(t.blend_steps);
        case 20:  // ms on the wire, us in the engine
            t.settle_grace_us = uint32_t(clampf(v, 0.0f, b::settle_ms_max) * 1000.0f + 0.5f);
            return IntentValue::ofF32(float(t.settle_grace_us) / 1000.0f);
        default: return std::nullopt;
    }
}

// The current value of one 0x3120 key, exactly as setTuningKey() echoes it.
std::optional<IntentValue> tuningKeyValue(const MotionTuning& t, uint8_t key) {
    switch (key) {
        case 1:  return IntentValue::ofF32(t.jmax_ovr);
        case 2:  return IntentValue::ofF32(t.vmax_ovr);
        case 3:  return IntentValue::ofF32(t.amax_ovr);
        case 6:  return IntentValue::ofU64(t.chase_ff);
        case 7:  return IntentValue::ofU64(t.chase_accel_ff);
        case 8:  return IntentValue::ofF32(t.chase_gain);
        case 9:  return IntentValue::ofF32(t.chase_lookahead);
        case 10: return IntentValue::ofF32(float(t.chase_dense_us) / 1000.0f);
        case 11: return IntentValue::ofU64(t.chase_aim_extrap);
        case 12: return IntentValue::ofF32(t.handoff_k);
        case 13: return IntentValue::ofU64(t.curve_policy);
        case 14: return IntentValue::ofU64(t.infeasible_policy);
        case 16: return IntentValue::ofF32(t.smooth_budget);
        case 17: return IntentValue::ofF32(t.amplitude_budget);
        case 18: return IntentValue::ofU64(t.blend_steps);
        case 20: return IntentValue::ofF32(float(t.settle_grace_us) / 1000.0f);
        default: return std::nullopt;
    }
}

// A trial baseline as a number. Baselines are only ever F32 or U64 here.
float baselineNumber(uint8_t key, const IntentValue& v) {
    const IntentValueField f{key, v};
    return numberOf(&f).value_or(0.0f);
}

// The four tuning cards, one bit each in ValenceDevice::_tuneDirty.
constexpr uint8_t kCardModes    = 0x01;  // 0x1030
constexpr uint8_t kCardLimits   = 0x02;  // 0x1120
constexpr uint8_t kCardChase    = 0x04;  // 0x1121
constexpr uint8_t kCardWaveform = 0x08;  // 0x1122

// Which cards differ between two tuning sets. Field-to-card membership is the
// catalog's (ValenceCatalog.h, the kinetic-* and machine-modes entries).
uint8_t cardsChanged(const MotionTuning& a, const MotionTuning& b) {
    uint8_t m = 0;
    if ((a.overshoot_guard > 0.0f) != (b.overshoot_guard > 0.0f)) m |= kCardModes;
    if (a.jmax_ovr != b.jmax_ovr || a.vmax_ovr != b.vmax_ovr || a.amax_ovr != b.amax_ovr)
        m |= kCardLimits;
    if (a.chase_ff != b.chase_ff || a.chase_accel_ff != b.chase_accel_ff ||
        a.chase_gain != b.chase_gain || a.chase_lookahead != b.chase_lookahead ||
        a.chase_dense_us != b.chase_dense_us || a.chase_aim_extrap != b.chase_aim_extrap ||
        a.handoff_k != b.handoff_k)
        m |= kCardChase;
    if (a.curve_policy != b.curve_policy || a.infeasible_policy != b.infeasible_policy ||
        a.smooth_budget != b.smooth_budget || a.amplitude_budget != b.amplitude_budget ||
        a.blend_steps != b.blend_steps || a.settle_grace_us != b.settle_grace_us)
        m |= kCardWaveform;
    return m;
}

// A layout whose hand-count disagrees with its buffer is a SILENT FIELD SHIFT:
// the writers stop when the buffer runs out and the tail goes out as zeros,
// which renders as plausible numbers in the wrong columns. This is the check
// that fails if any layout below is miscounted.
void publishPacked(Hub& hub, uint16_t id, std::span<const std::byte> buf, size_t written) {
    if (written != buf.size()) {
        GLOGE_EVERY_MS(5000, kTag, "channel %04x packed %u of %u B -- layout miscounted",
                       unsigned(id), unsigned(written), unsigned(buf.size()));
    }
    hub.publishState(id, buf);
}

// ---- the motion plane's retained STATE ---------------------------------------
// Every channel below reads ONE motionCensus(), so no two of them can disagree
// about the same instant. The byte layouts mirror ValenceCatalog.h's field
// order for each id exactly; that mirroring is the whole contract (there is no
// packed struct to static_assert against).

void publishMotion(Hub& hub, const MotionCensus& m, bool genRunning) {
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
    // homing is permanently 0 and that is the truth, not a stub: there is no
    // homing cycle on this board. gen_running is the generator DRIVING, not
    // merely switched on (patternActive()).
    packU8(buf, n, uint8_t((m.homed ? 0x01u : 0u) | (genRunning ? 0x04u : 0u) |
                 (m.paused ? 0x08u : 0u) | (m.override_mode ? 0x10u : 0u) |
                 (m.estop ? 0x20u : 0u) | (m.stream ? 0x40u : 0u)));
    packU16(buf, n, wireU16(m.demand_mm, 100.0f));     // raw_10um: the asked position
    // AT RATE, not on change. An on-change gate looks like an economy and is a
    // ground-truth hazard on a hero channel: a machine at rest stops pushing,
    // so a subscriber that joined late, or whose retained value never landed,
    // has nothing to show and cannot tell a still carriage from a dead feed.
    publishPacked(hub, ch::motion, buf, n);
}

void publishPlanStrip(Hub& hub, const MotionCensus& m) {
    std::array<std::byte, 18> buf{};
    size_t n = 0;
    // flags: active, live_mode, grad_mode. live_mode and grad_mode named a
    // legacy interpolator split that has no counterpart in this engine.
    packU8(buf, n, m.busy ? 0x01u : 0u);
    // style: kinetic::Mode, or `hold` for a hold segment, so a long dwell
    // reads as a live plan and never as a stall.
    packU8(buf, n, m.plan_hold ? kPlanStyleHold : m.mode);
    packU16(buf, n, wireU16(m.plan_start, 10000.0f));
    packU16(buf, n, wireU16(m.plan_end, 10000.0f));
    packU16(buf, n, wireU16(m.plan_cur, 10000.0f));
    packI16(buf, n, wireI16(m.plan_vel, 1000.0f));
    packU32(buf, n, m.plan_duration_us);
    packU32(buf, n, m.plan_elapsed_us);
    publishPacked(hub, ch::plan_strip, buf, n);
}

// hubDropped: bundles the hub dropped whole at ingress (IngressDropTally.h).
void publishMotionDiag(Hub& hub, const MotionCensus& m, uint32_t hubDropped) {
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
    // sync_dropped: the arbiter's refused samples plus the bundles the hub
    // dropped whole at ingress, PAUSE included (SPEC 11.1, RFC-074 clause 4).
    // A dropped bundle counts ONE: the library keeps no sample count for it,
    // so the field understates samples rather than inventing them.
    packU32(buf, n, m.stream_dropped + hubDropped);
    // sync_seg_bundles is not separated here: both stream channels land in one
    // counter, and splitting it would need a second pair the census does not
    // carry. It reads 0, which understates rather than invents.
    packU32(buf, n, 0);
    packU16(buf, n, 0);                                // reset_gen: nothing resets these
    publishPacked(hub, ch::motion_diag, buf, n);
}

void publishOdometer(Hub& hub, const MotionCensus& m) {
    std::array<std::byte, 20> buf{};
    size_t n = 0;
    packU32(buf, n, m.strokes);
    packF32(buf, n, m.distance_mm / 1000.0f);          // distance_m
    packF32(buf, n, m.peak_mm_s);
    packF32(buf, n, 0.0f);                             // energy_wh: no power monitor
    packU32(buf, n, uint32_t(deviceNowUs() / 1000));
    publishPacked(hub, ch::odometer, buf, n);
}

// Layout per ValenceCatalog.h's machine-modes entry: 7 B, plus home_style
// only where has_drive put it in the catalog.
void publishMachineModes(Hub& hub, const MotionTuning& t, const StoredModes& m, bool horizonOpen,
                         bool flipOpen) {
    std::array<std::byte, 9> buf{};
    const bool drive = boardFeatures().has_drive;
    const size_t len = drive ? 9 : 8;
    size_t n = 0;
    packU8(buf, n, 0);   // blend_mode_reserved
    packU8(buf, n, 0);   // stream_speed_reserved
    packU8(buf, n, t.overshoot_guard > 0.0f ? 1 : 0);   // overshoot_clamp
    // enabled_mask: bit 0 overshoot_clamp, accepted at all times. home_style
    // (has_drive only) stays low: nothing here runs a homing cycle.
    // schedule_horizon drops while a segments grant is live (applyModes()).
    // flipped drops whenever applyModes() would refuse it (flipOpen()).
    const uint8_t horizonBit = drive ? 0x04 : 0x02;
    const uint8_t flipBit = uint8_t(horizonBit << 1);
    packU8(buf, n, uint8_t(0x01 | (horizonOpen ? horizonBit : 0) | (flipOpen ? flipBit : 0)));
    packU8(buf, n, 2);   // motion_backend, read-only: quadrature, the LP-core emitter
    if (drive) packU8(buf, n, 0);   // home_style
    packU8(buf, n, m.horizon);      // schedule_horizon
    packU8(buf, n, m.flipped ? 1 : 0);   // flipped
    packU8(buf, n, uint8_t(hub.trialMask(ch::machine_modes)));   // trial_mask (RFC-099)
    publishPacked(hub, ch::machine_modes, std::span<const std::byte>(buf).first(len), n);
}

// The three kinetic cards, each only when `cards` names it. Every setting on
// them is applied (0x3120), so every enabled_mask bit is high.
void publishKineticCards(Hub& hub, const MotionTuning& t, uint8_t cards) {
    if (cards & kCardLimits) {
        std::array<std::byte, 14> buf{};
        size_t n = 0;
        packF32(buf, n, t.jmax_ovr);
        packF32(buf, n, t.vmax_ovr);
        packF32(buf, n, t.amax_ovr);
        packU8(buf, n, 0x07);    // enabled_mask
        packU8(buf, n, uint8_t(hub.trialMask(ch::kinetic_limits)));
        publishPacked(hub, ch::kinetic_limits, buf, n);
    }
    if (cards & kCardChase) {
        std::array<std::byte, 21> buf{};
        size_t n = 0;
        packU8(buf, n, t.chase_ff ? 1 : 0);
        packU8(buf, n, t.chase_accel_ff ? 1 : 0);
        packF32(buf, n, t.chase_gain);
        packF32(buf, n, t.chase_lookahead);
        packU32(buf, n, t.chase_dense_us);     // scale 1000, unit ms: the wire carries us
        packU8(buf, n, t.chase_aim_extrap ? 1 : 0);
        packF32(buf, n, t.handoff_k);
        packU8(buf, n, 0x7F);                  // enabled_mask
        packU8(buf, n, uint8_t(hub.trialMask(ch::kinetic_chase)));
        publishPacked(hub, ch::kinetic_chase, buf, n);
    }
    if (cards & kCardWaveform) {
        std::array<std::byte, 17> buf{};
        size_t n = 0;
        packU8(buf, n, t.curve_policy);
        packU8(buf, n, t.infeasible_policy);
        packF32(buf, n, t.smooth_budget);
        packF32(buf, n, t.amplitude_budget);
        packU8(buf, n, t.blend_steps);
        packU32(buf, n, t.settle_grace_us);    // scale 1000, unit ms: the wire carries us
        packU8(buf, n, 0x3F);                  // enabled_mask
        packU8(buf, n, uint8_t(hub.trialMask(ch::kinetic_waveform)));
        publishPacked(hub, ch::kinetic_waveform, buf, n);
    }
}

// ---- the pattern plane's retained STATE ----------------------------------------
// ON CHANGE of the bytes last SENT, never on a timer and never on cfg_gen:
// these channels move on session-volatile writes that bump nothing, and an
// enabled_mask moves with homed and e-stop, which no write announces.

// The eight modulators in BaseId order, which is NOT ascending channel order:
// the catalog puts them speed-in/out, accel-in/out, depth-1/2, crest/trough
// (ValenceCatalog.h).
constexpr std::array<uint16_t, advpat::BASE_COUNT> kModChannels{
    ch::pattern_adv_mod_depth1,   ch::pattern_adv_mod_depth2,  ch::pattern_adv_mod_speedin,
    ch::pattern_adv_mod_speedout, ch::pattern_adv_mod_accelin, ch::pattern_adv_mod_accelout,
    ch::pattern_adv_mod_crest,    ch::pattern_adv_mod_trough};

// A base control's wire value as the generator stores it: a percent knob, or
// a dwell in hundredths of a stroke (RFC-095). setBase() clamps again.
int apBaseOf(uint8_t id, float v) {
    return id < advpat::PERCENT_BASE_COUNT ? knobOf(v) : int(wholeIn(v * 100.0f, 0.0f, 65535.0f));
}

// The applied value of one 0x3210 knob key (never kApRunKey), for its ECHO.
IntentValue apEchoOf(const advpat::Settings& ap, uint8_t key) {
    if (key == 2) return IntentValue::ofU64(ap.master.value);
    for (uint8_t id = 0; id < advpat::BASE_COUNT; ++id) {
        const advpat::BaseControl& b = *ap.byId(id);
        if (key == apBaseKey(id)) {
            return id < advpat::PERCENT_BASE_COUNT ? IntentValue::ofU64(b.value)
                                                   : IntentValue::ofF32(float(b.value) / 100.0f);
        }
        const uint8_t at = apModKeyBase(id);
        if (key >= at && key < at + 6) {
            const advpat::Modifier& m = b.modifier;
            const std::array<uint8_t, 6> mod{m.amount, m.in_step, m.in_wait,
                                              m.out_step, m.out_wait, m.offset};
            return IntentValue::ofU64(mod[size_t(key - at)]);
        }
    }
    return IntentValue::ofU64(0);
}

template <size_t N>
void publishIfChanged(Hub& hub, uint16_t id, const std::array<std::byte, N>& buf, size_t written,
                      std::array<std::byte, N>& sent, bool force) {
    if (!force && buf == sent) return;
    sent = buf;
    publishPacked(hub, id, buf, written);
}

}  // namespace

// ---- HubDelegate ---------------------------------------------------------------

// §12.2. A HELLO nothing vouches for is a WORKING STATE, not a failure: at
// WATCH it can subscribe to everything and use the role-exempt pause and estop
// ops. Only a live single-use /uitoken credential upgrades to CONTROL, and
// never to configure -- a browser-borne credential must not be able to re-key
// the trust ledger. The trust ledger itself is the other door and is the hub's
// own (pairing), consulted before this override is ever reached.
AccessLevel ValenceDevice::validateToken(std::span<const std::byte> instance_id,
                                         std::span<const std::byte> token, bool hasToken) {
    (void)instance_id;
    if (hasToken && _tokenGate != nullptr && _tokenGate->consume(token)) return AccessLevel::control;
    return _unvouchedRole;
}

Ret ValenceDevice::applyIntent(uint16_t channel_id, const IntentValueMap& requested,
                               AccessLevel role, bool& cfgChanged) {
    (void)role;
    _nackDetail[0] = '\0';
    if (channel_id == ch::move) return applyMove(requested);
    if (channel_id == ch::home) return applyHome(requested);
    if (channel_id == ch::modes_set) return durable(applyModes(requested, cfgChanged));
    if (channel_id == ch::kinetic_set) return durable(applyTuning(requested, cfgChanged));
    // None of the three moves cfg_gen (cfgChanged stays false). The presets
    // persist, but on their own blob, and their change signal is the roster
    // generation.
    if (channel_id == ch::pattern_cmd) return applyPattern(requested);
    if (channel_id == ch::pattern_advanced_cmd) return applyPatternAdvanced(requested);
    if (channel_id == ch::pattern_presets_cmd) return applyPresets(requested);
    if (channel_id == channels::safety_intents) return applySafety(requested);
    if (channel_id != ch::config_set) return Ret::err(NackCode::UNSUPPORTED_OP);
    return durable(applyConfig(requested, cfgChanged));
}

Ret ValenceDevice::durable(Ret r) {
    if (r && !_trialApply) _durableDirty = true;
    return r;
}

Ret ValenceDevice::applyConfig(const IntentValueMap& requested, bool& cfgChanged) {
    const auto* f1 = findField(requested, 1);  // window_min
    const auto* f2 = findField(requested, 2);  // window_max
    const auto* f3 = findField(requested, 3);  // jog_speed
    const auto* f4 = findField(requested, 4);  // jog_accel
    const auto* f5 = findField(requested, 5);  // input_speed
    const auto* f6 = findField(requested, 6);  // input_accel
    const auto* f7 = findField(requested, 7);  // input_jerk
    const auto* f8 = findField(requested, 8);  // max_rail
    const std::array<const IntentValueField*, 8> keys{f1, f2, f3, f4, f5, f6, f7, f8};
    for (size_t k = 0; k < keys.size(); ++k)
        if (keys[k] != nullptr && !numberOf(keys[k])) return refuseNotANumber(ch::config_set, uint8_t(k + 1));

    // The window arrives in the client frame (RFC-088) and is stored physical.
    const float rail = motionCensus().rail_mm;
    StoredConfig next = _cfg;
    for (size_t k = 0; k < keys.size(); ++k)
        if (keys[k] != nullptr) setConfigKey(next, uint8_t(k + 1), *numberOf(keys[k]), rail, _modes.flipped);

    // The one refusal of finite values: an inverted window has no legal
    // nearest value, so it is rejected rather than silently reordered.
    if (next.window_min >= next.window_max) return Ret::err(NackCode::INVALID_VALUE);

    // RFC-002 as tightened for v1.0: cfgChanged means CHANGED, never merely
    // ACCEPTED. A value-identical write still gets its post-clamp ECHO.
    cfgChanged = !(next == _cfg);
    _cfg = next;
    if (cfgChanged) _cfgDirty = true;

    // The first-run record (RFC-079 setup): every key this accepted write
    // carried, changed or not, counts as written. A confirmed default is a
    // confirmation. Not a cfg_gen change; the dirty flag persists it. A trial
    // is not a confirmation until its commit (onTrialCommit()).
    uint8_t wrote = 0;
    for (size_t k = 0; k < keys.size(); ++k)
        if (keys[k] != nullptr) wrote |= uint8_t(1u << k);
    if (!_trialApply) noteSetupWritten(wrote);

    // Key-complete ECHO: every key the client sent comes back with the
    // value the hub actually holds (SPEC §9.3).
    IntentValueMap applied{};
    uint32_t n = 0;
    const Window now = clientWindow(rail);
    if (f1) applied.fields[n++] = {1, IntentValue::ofF32(now.lo)};
    if (f2) applied.fields[n++] = {2, IntentValue::ofF32(now.hi)};
    if (f3) applied.fields[n++] = {3, IntentValue::ofF32(_cfg.jog_speed)};
    if (f4) applied.fields[n++] = {4, IntentValue::ofF32(_cfg.jog_accel)};
    if (f5) applied.fields[n++] = {5, IntentValue::ofF32(_cfg.input_speed)};
    if (f6) applied.fields[n++] = {6, IntentValue::ofF32(_cfg.input_accel)};
    if (f7) applied.fields[n++] = {7, IntentValue::ofF32(_cfg.input_jerk)};
    if (f8) applied.fields[n++] = {8, IntentValue::ofF32(_cfg.max_rail)};
    applied.count = n;
    return Ret::ok(applied);
}

void ValenceDevice::noteSetupWritten(uint8_t wrote) {
    if ((_modes.setup_written | wrote) == _modes.setup_written) return;
    const bool was = commissioned(_modes);
    _modes.setup_written |= wrote;
    _cfgDirty = true;
    if (!was && commissioned(_modes)) GLOGI(kTag, "commissioned: every setup field written");
}

// SPEC 16.1: asked by the hub right after applyIntent() refused, on the same
// task, before the NACK is encoded. The view is into _nackDetail, which lives
// until the next applyIntent() clears it.
std::string_view ValenceDevice::intentNackDetail(uint16_t channel_id, NackCode code) {
    (void)channel_id;
    (void)code;
    return std::string_view(_nackDetail.data());
}

// ---- 0x3030 modes / 0x3120 kinetic tuning -----------------------------------
// Both write ONE MotionTuning, the device's own copy. The ECHO carries the
// post-clamp value that copy now holds, and tick() hands the copy to the
// motion task before the next hub tick ends, so the engine plans its next
// intent under it. A key outside the channel's schema is not applied and is
// absent from the ECHO (§9.3); a write that applies no key at all NACKs
// INVALID_VALUE, as does a non-numeric or non-finite value anywhere in it
// (refuseNotANumber()). Clamp bounds are StoredState.h's tuning_bounds, the
// same ones a stored set is validated against at boot; tick() persists a
// changed set with 0x1000.
// Key 10 (chase_dense) is refused INTERLOCK while a samples grant is live: it
// sets that grant's schedule_latency_us, a commitment for the grant's life
// (SPEC 5.4, RFC-059), and the library has no publish re-GRANT to move it.

void ValenceDevice::noteTuning(const MotionTuning& next, bool& cfgChanged) {
    const uint8_t cards = cardsChanged(_tune, next);
    cfgChanged = cards != 0;
    _tuneDirty |= cards;
    _tune = next;
}

// Keys 4 (overshoot_clamp) and 7 (schedule_horizon); the whole request is
// validated before anything is applied. A horizon change is refused INTERLOCK
// while a segments grant is live: the grant advertised the old value for its
// life (SPEC 5.4), and the library re-reads this one on every bundle.
// Key 8 (flipped, RFC-088) is gated in the spec's order: SOURCE_CONFLICT while
// a source owns the rail, NOT_HOMED while unhomed, INTERLOCK under override or
// in motion. An unchanged value is an ordinary no-op ECHO.
Ret ValenceDevice::applyModes(const IntentValueMap& requested, bool& cfgChanged) {
    const auto* f4 = findField(requested, 4);   // overshoot_clamp
    const auto* f7 = findField(requested, 7);   // schedule_horizon
    const auto* f8 = findField(requested, 8);   // flipped
    if (!f4 && !f7 && !f8) return Ret::err(NackCode::INVALID_VALUE);
    if (f4 && !numberOf(f4)) return refuseNotANumber(ch::modes_set, 4);
    if (f7 && !numberOf(f7)) return refuseNotANumber(ch::modes_set, 7);
    if (f8 && !boolOf(f8)) return refuseNotANumber(ch::modes_set, 8);
    StoredModes nextModes = _modes;
    if (f7) nextModes.horizon = uint8_t(wholeIn(*numberOf(f7), 0.0f, float(kHorizonMs.size() - 1)));
    if (nextModes.horizon != _modes.horizon && publishGrantLive(ch::motion_segment))
        return Ret::err(NackCode::INTERLOCK);
    if (f8) nextModes.flipped = *boolOf(f8);
    if (nextModes.flipped != _modes.flipped) {
        const MotionCensus c = motionCensus();
        if (railOwned()) return Ret::err(NackCode::SOURCE_CONFLICT);
        if (!c.homed) return Ret::err(NackCode::NOT_HOMED);
        if (windowOnTrial()) return refuse(NackCode::INTERLOCK, "window on trial: commit or revert first");
        if (!flipOpen(c)) return Ret::err(NackCode::INTERLOCK);
    }

    IntentValueMap applied{};
    uint32_t n = 0;
    bool tuneChanged = false;
    if (f4) {
        MotionTuning next = _tune;
        const bool on = wholeIn(*numberOf(f4), 0.0f, 1.0f) != 0;
        next.overshoot_guard = overshootGuardFor(on, motionDefaultTuning().overshoot_guard);
        noteTuning(next, tuneChanged);
        applied.fields[n++] = {4, IntentValue::ofU64(on ? 1 : 0)};
    }
    if (f7) applied.fields[n++] = {7, IntentValue::ofU64(nextModes.horizon)};
    if (f8) applied.fields[n++] = {8, IntentValue::ofU64(nextModes.flipped ? 1 : 0)};
    const bool modesChanged = !(nextModes == _modes);
    // At rest by the gate above, so the frame moves under a still carriage.
    if (nextModes.flipped != _modes.flipped) {
        motionSetFlipped(nextModes.flipped);
        GLOGW(kTag, "FLIP %s: position 0 is the %s end", nextModes.flipped ? "on" : "off",
              nextModes.flipped ? "far" : "home");
    }
    _modes = nextModes;
    // Same card, same publish and persist path as the tuning (tick()).
    if (modesChanged) _tuneDirty |= kCardModes;
    cfgChanged = tuneChanged || modesChanged;
    applied.count = n;
    return Ret::ok(applied);
}

// Any session holding a publish grant of `channel_id`, live or parked: its
// grant carries the horizon and the latency this hub advertised.
bool ValenceDevice::publishGrantLive(uint16_t channel_id) const {
    if (_hub == nullptr) return false;
    for (size_t i = 0; i < kHubMaxSessions; ++i) {
        const HubSession* s = _hub->sessionBySlot(i);
        if (s == nullptr || !s->occupied()) continue;
        for (const auto& pg : s->publishGrants)
            if (pg.used && pg.channel_id == channel_id) return true;
    }
    return false;
}

// RFC-087: only the segments channel has a horizon. The library advertises
// 500 and 1000 and omits the key for 250, its default.
uint16_t ValenceDevice::scheduleHorizonMs(uint16_t channel_id) {
    return channel_id == ch::motion_segment ? kHorizonMs[_modes.horizon] : 0;
}

// RFC-059, computed from the pipeline, never a second constant. Segments
// start AT their stamp, planned and parked ahead of it, so the delay is the
// motion tick's one-tick hop. A samples point is reached at its stamp once it
// arrives a planning interval ahead (RFC-084); the chase-planning budget is
// that interval at its longest, the densest cadence the engine still plans
// as a stream (chase_dense), plus the same hop.
uint32_t ValenceDevice::scheduleLatencyUs(uint16_t channel_id) {
    if (channel_id == ch::motion_segment) return kMotionTickUs;
    if (channel_id == ch::motion_input) return _tune.chase_dense_us + kMotionTickUs;
    return 0;
}

Ret ValenceDevice::applyTuning(const IntentValueMap& requested, bool& cfgChanged) {
    MotionTuning t = _tune;
    IntentValueMap applied{};
    uint32_t n = 0;
    for (const uint8_t key : kTuningKeys) {
        const auto* f = findField(requested, key);
        if (!f) continue;
        const std::optional<float> v = numberOf(f);
        if (!v) return refuseNotANumber(ch::kinetic_set, key);
        // n cannot overrun: requested carries at most kIntentMaxValueFields
        // fields, and each key here consumes one of them at most once.
        const std::optional<IntentValue> out = setTuningKey(t, key, *v);
        if (!out) continue;
        if (key == kChaseDenseKey && t.chase_dense_us != _tune.chase_dense_us && publishGrantLive(ch::motion_input))
            return Ret::err(NackCode::INTERLOCK);
        applied.fields[n++] = {key, *out};
    }
    if (n == 0) return Ret::err(NackCode::INVALID_VALUE);
    applied.count = n;
    noteTuning(t, cfgChanged);
    return Ret::ok(applied);
}

// ---- 0x3200 pattern-cmd, 0x3210 pattern-advanced-cmd, 0x3220 presets ----------
// All three write ONE PatternSettings, the device's own copy, validate the
// whole request BEFORE touching it (a NACK changes nothing), and echo what the
// copy then holds. tick() hands the copy to the generator's task.

// Keys 1-6 are the generator's run and stroke knobs and all six are refused
// together while e-stop is latched or the machine is unhomed, which is exactly
// when 0x1200's enabled_mask drops bits 0-5. Key 7 (background_run) is a
// standing policy, accepted at all times: its mask bit never drops.
Ret ValenceDevice::applyPattern(const IntentValueMap& requested) {
    const auto* f1 = findField(requested, 1);  // running
    const auto* f2 = findField(requested, 2);  // pattern
    const auto* f3 = findField(requested, 3);  // speed
    const auto* f4 = findField(requested, 4);  // depth
    const auto* f5 = findField(requested, 5);  // stroke
    const auto* f6 = findField(requested, 6);  // sensation
    const auto* f7 = findField(requested, 7);  // background_run
    const bool live = f1 || f2 || f3 || f4 || f5 || f6;
    if (!live && !f7) return Ret::err(NackCode::INVALID_VALUE);
    if (live) {
        const MotionCensus c = motionCensus();
        if (c.estop) return refuse(NackCode::ESTOP_ACTIVE, kDetailEstop);
        // Only a start is motion; a knob turned with the switch off is not.
        if (f1 && boolOf(f1).value_or(false))
            if (const auto why = startRefusal(c, "classic start")) return Ret::err(*why);
        if (!c.homed) return Ret::err(NackCode::NOT_HOMED);
    }
    if (f1 && !boolOf(f1)) return refuseNotANumber(ch::pattern_cmd, 1);
    if (f7 && !boolOf(f7)) return refuseNotANumber(ch::pattern_cmd, 7);
    for (const auto* f : {f2, f3, f4, f5, f6})
        if (f && !numberOf(f)) return refuseNotANumber(ch::pattern_cmd, f->key);

    // RFC-093: a start takes the rail, refused while the advanced generator
    // holds it (the arbiter logs whose it is); a stop hands it back.
    if (f1 && *boolOf(f1) && !motionAcquireRail(MotionSource::Pattern))
        return Ret::err(NackCode::SOURCE_CONFLICT);
    PatternSettings& p = _pat;
    if (f1) p.running = *boolOf(f1);
    if (f1 && !p.running) motionReleaseRail(MotionSource::Pattern);
    if (f2) p.setPattern(int(wholeIn(*numberOf(f2), 0.0f, float(PatternSettings::kPatternCount - 1))));
    if (f3) p.speed = PatternSettings::percent(*numberOf(f3));
    if (f4) p.depth = PatternSettings::percent(*numberOf(f4));
    if (f5) p.stroke = PatternSettings::percent(*numberOf(f5));
    if (f6) p.sensation = PatternSettings::percent(*numberOf(f6));
    if (f7) p.background_run = *boolOf(f7);
    _patDirty = true;

    IntentValueMap applied{};
    uint32_t n = 0;
    if (f1) applied.fields[n++] = {1, IntentValue::ofBool(p.running)};
    if (f2) applied.fields[n++] = {2, IntentValue::ofU64(p.pattern)};
    if (f3) applied.fields[n++] = {3, IntentValue::ofF32(p.speed)};
    if (f4) applied.fields[n++] = {4, IntentValue::ofF32(p.depth)};
    if (f5) applied.fields[n++] = {5, IntentValue::ofF32(p.stroke)};
    if (f6) applied.fields[n++] = {6, IntentValue::ofF32(p.sensation)};
    if (f7) applied.fields[n++] = {7, IntentValue::ofBool(p.background_run)};
    applied.count = n;
    return Ret::ok(applied);
}

// Keys 2-44 and 46-59 are knobs, refused only while e-stop is latched (when
// every pattern-advanced and modulator enabled_mask bit drops); none moves the
// machine by itself, so none is gated on homed. Key 45 `running` is the
// advanced generator's start and stop, gated as 0x3200's running is, plus
// SOURCE_CONFLICT while the classic generator holds the rail (RFC-093). Keys
// 46 and 47 are the dwells (RFC-095) in strokes, stored in hundredths. Key 1
// is the retired mode switch: a permanent gap, never read, never echoed. Base
// controls apply in key order (max depth before min depth), each re-coupling
// the depth pair, and the echo reads back AFTER all of them.
Ret ValenceDevice::applyPatternAdvanced(const IntentValueMap& requested) {
    const MotionCensus c = motionCensus();
    if (c.estop) return refuse(NackCode::ESTOP_ACTIVE, kDetailEstop);
    bool any = false;
    for (uint8_t key = 2; key <= kApLastKey; ++key) {
        if (key == kApRunKey) continue;
        const auto* f = findField(requested, key);
        if (!f) continue;
        if (!numberOf(f)) return refuseNotANumber(ch::pattern_advanced_cmd, key);
        any = true;
    }
    const auto* run = findField(requested, kApRunKey);
    if (run && !boolOf(run)) return refuseNotANumber(ch::pattern_advanced_cmd, kApRunKey);
    if (!any && !run) return Ret::err(NackCode::INVALID_VALUE);
    const bool start = run && *boolOf(run);
    if (start) {
        if (const auto why = startRefusal(c, "advanced start")) return Ret::err(*why);
        if (!motionAcquireRail(MotionSource::Advanced)) return Ret::err(NackCode::SOURCE_CONFLICT);
    }

    advpat::Settings& ap = _pat.ap;
    if (const auto* f = findField(requested, 2)) ap.master.set(knobOf(*numberOf(f)));
    for (uint8_t id = 0; id < advpat::BASE_COUNT; ++id)
        if (const auto* f = findField(requested, apBaseKey(id))) ap.setBase(id, apBaseOf(id, *numberOf(f)));
    for (uint8_t id = 0; id < advpat::BASE_COUNT; ++id) {
        advpat::Modifier& m = ap.byId(id)->modifier;
        std::array<int, 6> v{m.amount, m.in_step, m.in_wait, m.out_step, m.out_wait, m.offset};
        for (uint8_t sub = 0; sub < 6; ++sub)
            if (const auto* f = findField(requested, uint8_t(apModKeyBase(id) + sub))) v[sub] = knobOf(*numberOf(f));
        m.set(v[0], v[1], v[2], v[3], v[4], v[5]);
    }
    if (run) {
        _pat.adv_running = start;
        if (!start) motionReleaseRail(MotionSource::Advanced);
    }
    _patDirty = true;

    IntentValueMap applied{};
    uint32_t n = 0;
    for (uint8_t key = 2; key <= kApLastKey; ++key) {
        if (!findField(requested, key)) continue;
        applied.fields[n++] = {key, key == kApRunKey ? IntentValue::ofBool(_pat.adv_running)
                                                     : apEchoOf(ap, key)};
    }
    applied.count = n;
    return Ret::ok(applied);
}

// The four SPEC §8.7 verbs; the op-select ordinals are the registry's store_ops
// (RFC-067), which the catalog's labels {reserved, save, load, delete, rename}
// index-align with. save captures LIVE state (a client payload, an import, is
// not offered); load applies through the same clamps an intent takes into the
// advanced generator's knobs, starting nothing, and its truth arrives on the
// ordinary pattern-plane STATE. The echoed name is the STORE's copy.
Ret ValenceDevice::applyPresets(const IntentValueMap& requested) {
    const auto* opF = findField(requested, 1);
    const auto* slotF = findField(requested, 2);
    if (opF && !numberOf(opF)) return refuseNotANumber(ch::pattern_presets_cmd, 1);
    if (slotF && !numberOf(slotF)) return refuseNotANumber(ch::pattern_presets_cmd, 2);
    const auto op = numberOf(opF);
    const auto slotV = numberOf(slotF);
    if (!op || !slotV || *slotV < 0.0f || *slotV >= float(PatternPresetStore::kCapacity))
        return Ret::err(NackCode::INVALID_VALUE);
    const uint8_t slot = uint8_t(wholeIn(*slotV, 0.0f, float(PatternPresetStore::kCapacity - 1)));
    const auto* nameF = findField(requested, 3);
    const std::string_view name =
        (nameF && nameF->value.kind == IntentValue::Kind::Tstr) ? nameF->value.tstr_val : std::string_view{};

    const uint32_t verb = wholeIn(*op, 0.0f, 255.0f);
    switch (verb) {
        case store_ops::save:
            if (!_presets.save(slot, name, _pat.capturePreset())) return Ret::err(NackCode::INVALID_VALUE);
            GLOGI(kTag, "preset saved: slot %u", unsigned(slot));
            break;
        case store_ops::load: {
            if (motionCensus().estop) return refuse(NackCode::ESTOP_ACTIVE, kDetailEstop);
            const PatternPresetStore::Slot* s = _presets.slot(slot);
            if (s == nullptr) return Ret::err(NackCode::INVALID_VALUE);
            _pat.applyPreset(s->payload);
            _patDirty = true;
            GLOGI(kTag, "preset loaded: slot %u", unsigned(slot));
            break;
        }
        case store_ops::delete_item:
            if (!_presets.remove(slot)) return Ret::err(NackCode::INVALID_VALUE);
            GLOGI(kTag, "preset deleted: slot %u", unsigned(slot));
            break;
        case store_ops::rename:
            if (!_presets.rename(slot, name)) return Ret::err(NackCode::INVALID_VALUE);
            break;
        default:
            return Ret::err(NackCode::INVALID_VALUE);
    }
    IntentValueMap applied{};
    applied.fields[0] = {1, IntentValue::ofU64(verb)};
    applied.fields[1] = {2, IntentValue::ofU64(slot)};
    applied.count = 2;
    if (verb == 1 || verb == 4) {
        applied.fields[2] = {3, IntentValue::ofTstr(_presets.slot(slot)->nameView())};
        applied.count = 3;
    }
    return Ret::ok(applied);
}

// The generator runs on its own task and sees only whole copies; the stroke
// frame is the stored config's, stamped in at push time so the two can never
// disagree about the window.
void ValenceDevice::pushPattern() {
    PatternSettings s = _pat;
    // The client frame: the generator reads the mirrored census, and the
    // arbiter mirrors its strokes back (RFC-088).
    const Window w = clientWindow(motionCensus().rail_mm);
    s.frame = {w.lo, w.hi, _cfg.input_speed, _cfg.input_accel};
    patternSetSettings(s);
}

void ValenceDevice::publishPatternPlane(const MotionCensus& mo) {
    Hub& hub = *_hub;
    const bool force = !_patPlaneSent;
    _patPlaneSent = true;
    const PatternSettings& p = _pat;
    {
        // enabled_mask: bits 0-5 are applyPattern()'s own refusals, bit 6
        // (background_run) never drops.
        const uint8_t mask = uint8_t(((mo.estop || !mo.homed) ? 0x00u : 0x3Fu) | 0x40u);
        std::array<std::byte, 20> buf{};
        size_t n = 0;
        packU8(buf, n, p.running ? 1 : 0);
        packU8(buf, n, p.pattern);
        packF32(buf, n, p.speed);
        packF32(buf, n, p.depth);
        packF32(buf, n, p.stroke);
        packF32(buf, n, p.sensation);
        packU8(buf, n, mask);
        packU8(buf, n, p.background_run ? 1 : 0);
        publishIfChanged(hub, ch::pattern_state, buf, n, _sentPatState, force);
    }
    {
        // enabled_mask: bits 0-6 (knobs) drop under e-stop, bit 7 (running)
        // also unhomed, applyPatternAdvanced()'s own refusals. enabled_mask2:
        // the two dwells, knobs, e-stop alone.
        const advpat::Settings& ap = p.ap;
        const uint8_t mask = mo.estop ? 0x00u : uint8_t(0x7Fu | (mo.homed ? 0x80u : 0x00u));
        const uint8_t mask2 = mo.estop ? 0x00u : 0x03u;
        std::array<std::byte, 15> buf{};
        size_t n = 0;
        packU8(buf, n, 0);   // ap_mode_reserved
        packU8(buf, n, uint8_t(ap.master.value));
        packU8(buf, n, uint8_t(ap.max_depth.value));
        packU8(buf, n, uint8_t(ap.min_depth.value));
        packU8(buf, n, uint8_t(ap.in_speed.value));
        packU8(buf, n, uint8_t(ap.out_speed.value));
        packU8(buf, n, uint8_t(ap.in_accel.value));
        packU8(buf, n, uint8_t(ap.out_accel.value));
        packU8(buf, n, mask);
        packU8(buf, n, p.adv_running ? 1 : 0);
        packU16(buf, n, ap.dwell_crest.value);
        packU16(buf, n, ap.dwell_trough.value);
        packU8(buf, n, mask2);
        publishIfChanged(hub, ch::pattern_advanced, buf, n, _sentApBase, force);
    }
    for (uint8_t id = 0; id < advpat::BASE_COUNT; ++id) {
        const advpat::Modifier& m = p.ap.byId(id)->modifier;
        std::array<std::byte, 7> buf{};
        size_t n = 0;
        packU8(buf, n, m.amount);
        packU8(buf, n, m.in_step);
        packU8(buf, n, m.in_wait);
        packU8(buf, n, m.out_step);
        packU8(buf, n, m.out_wait);
        packU8(buf, n, m.offset);
        packU8(buf, n, mo.estop ? 0x00 : 0x3F);
        publishIfChanged(hub, kModChannels[id], buf, n, _sentApMod[id], force);
    }
    {
        std::array<std::byte, 4> buf{};
        size_t n = 0;
        packU16(buf, n, _presets.generation());
        packU8(buf, n, _presets.count());
        packU8(buf, n, PatternPresetStore::kCapacity);
        publishIfChanged(hub, ch::pattern_presets_roster, buf, n, _sentRoster, force);
    }
}

// ---- 0x3100 move, the jog ------------------------------------------------------
// MANUAL source: the wire operator is the one driving. The arbiter lets a
// Manual intent through an unhomed machine (the push-to-home case a local
// button would use); this board has no such button, so the WIRE door is
// gated on homed here. force_home is that door (operator ruling
// 2026-09-21), and a refusal carries its reason rather than a count.
// SPEC 11.4: a jog never takes the rail from a source. Without override it is
// refused SOURCE_CONFLICT while the stream or the generator owns the rail; the
// hub has already admitted it under PAUSE only with override latched.
Ret ValenceDevice::applyMove(const IntentValueMap& requested) {
    const MotionCensus c = motionCensus();
    if (c.estop) return refuse(NackCode::ESTOP_ACTIVE, kDetailEstop);
    if (!c.power_gate) return refuseUnpowered("move");
    if (!c.homed) return Ret::err(NackCode::NOT_HOMED);
    const bool overrideOn = _hub != nullptr &&
                            (_hub->safetyModes() & safety_mode_bits::OVERRIDE) != 0;
    if (!overrideOn && railOwned()) return Ret::err(NackCode::SOURCE_CONFLICT);
    if (_returnPending) return refuse(NackCode::INTERLOCK, kDetailReturning);

    // A jog with no position is refused, never read as 0: that is a move to
    // the window's near end nobody asked for.
    const auto* f1 = findField(requested, 1);   // position
    if (f1 == nullptr) return Ret::err(NackCode::INVALID_VALUE);
    if (!numberOf(f1)) return refuseNotANumber(ch::move, 1);
    MotionIntent in;
    in.source    = MotionSource::Manual;
    in.target_mm = *numberOf(f1);
    _jogMark = c.intents + c.rejected;
    if (!motionSubmit(in)) return refuse(NackCode::INTERLOCK, "motion path refused the intent");

    // Ground truth: echo the post-clamp position, against the SAME bounds the
    // arbiter clamps a Manual intent to: the whole rail under override, the
    // travel window inside the rail otherwise.
    const Window w = clientWindow(c.rail_mm);
    const float lo = overrideOn ? 0.0f : std::max(0.0f, w.lo);
    const float hi = overrideOn ? c.rail_mm : std::min(w.hi, c.rail_mm);
    IntentValueMap applied{};
    applied.count = 1;
    applied.fields[0] = {1, IntentValue::ofF32(clampf(in.target_mm, lo, hi))};
    return Ret::ok(applied);
}

// The arbiter's power gate as a NACK: INTERLOCK (0x0402, registry.yaml
// nack_codes, "hub-specific safety interlock"). The reason rides the NACK's
// detail (SPEC 16.1) for the client; the throttled log line is the bench's.
// Both detail shapes stay under nack_detail_max_bytes (48) at the longest
// state and fault names, so the hub never has to cut one.
Ret ValenceDevice::refuse(NackCode code, const char* detail) {
    noteDetail(detail);
    return Ret::err(code);
}

void ValenceDevice::noteDetail(const char* detail) {
    std::snprintf(_nackDetail.data(), _nackDetail.size(), "%s", detail);
}

// A schema-field name is 1..24 bytes (catalog.cddl), so the detail fits
// nack_detail_max_bytes (48). A key the catalog does not name is cited by
// number.
Ret ValenceDevice::refuseNotANumber(uint16_t channel_id, uint8_t key) {
    std::string_view name;
    if (const CatalogEntry* e = _catalog != nullptr ? _catalog->find(channel_id) : nullptr)
        for (const SchemaField& f : _catalog->schemaFields(*e))
            if (f.key == key) name = f.name;
    if (name.empty())
        std::snprintf(_nackDetail.data(), _nackDetail.size(), "key %u: not a number", unsigned(key));
    else
        std::snprintf(_nackDetail.data(), _nackDetail.size(), "%.*s: not a number", int(name.size()),
                      name.data());
    return Ret::err(NackCode::INVALID_VALUE);
}

Ret ValenceDevice::refuseUnpowered(const char* what) {
    const MotorSwitchStatus sw = motorSwitchStatus();
    GLOGW_EVERY_MS(1000, kTag, "%s refused INTERLOCK: motor power is off (switch %s, last fault: %s)",
                   what, motorswitch::stateName(sw.state), motorswitch::faultName(sw.last_fault));
    if (sw.state == motorswitch::State::faulted)
        std::snprintf(_nackDetail.data(), _nackDetail.size(), "motor power off: %s",
                      motorswitch::faultName(sw.last_fault));
    else
        std::snprintf(_nackDetail.data(), _nackDetail.size(), "motor power off (switch %s)",
                      motorswitch::stateName(sw.state));
    return Ret::err(NackCode::INTERLOCK);
}

std::optional<NackCode> ValenceDevice::startRefusal(const MotionCensus& c, const char* what) {
    if (!c.power_gate) return refuseUnpowered(what).error();
    if (!commissioned(_modes)) {
        GLOGW_EVERY_MS(1000, kTag, "%s refused INTERLOCK: not commissioned "
                       "(setup fields written 0x%02x of 0x%02x)",
                       what, unsigned(_modes.setup_written), unsigned(kSetupRequiredMask));
        std::snprintf(_nackDetail.data(), _nackDetail.size(), "not commissioned: setup 0x%02x of 0x%02x",
                      unsigned(_modes.setup_written), unsigned(kSetupRequiredMask));
        return NackCode::INTERLOCK;
    }
    if (!c.homed) return NackCode::NOT_HOMED;
    return std::nullopt;
}

// The rail is a source's while the stream owns it, while either generator is
// started (RFC-093: a generator owns the rail from start to stop), or while
// one still drives it (the stop's brake, or background_run unattended).
bool ValenceDevice::railOwned() const {
    return _owner[uint8_t(MotionSource::Stream)] != 0 || _pat.running || _pat.adv_running ||
           patternActive();
}

// SPEC 11.1: what the hub may let through while PAUSE is latched. The home
// verb always (it is not source-mapped here, so this is belt and braces); the
// jog only under override and never while the return runs; a generator
// writer that starts nothing (a knob or a stop is not a motion intent). Every
// stream bundle is the hub's to drop and never reaches this.
// A refusal names its reason for intentNackDetail() (SPEC 16.1); an admission
// clears it, so a stale reason never rides a later NACK.
bool ValenceDevice::admitsUnderPause(uint16_t channel_id, const IntentValueMap& value,
                                     bool overrideLatched) {
    _nackDetail[0] = '\0';
    bool admit = false;
    const char* why = kDetailPaused;
    if (channel_id == ch::home) {
        admit = true;
    } else if (channel_id == ch::move) {
        admit = overrideLatched && !_returnPending;
        if (overrideLatched) why = kDetailReturning;
        else why = "paused: override to jog";
    } else if (channel_id == ch::pattern_cmd || channel_id == ch::pattern_advanced_cmd) {
        const uint8_t key = channel_id == ch::pattern_cmd ? 1 : kApRunKey;
        const std::optional<bool> running = boolOf(findField(value, key));
        admit = !(running && *running);
        why = "paused: resume to start";
    }
    if (!admit) noteDetail(why);
    return admit;
}

// ---- 0x0005 safety-intents ------------------------------------------------------
// The delegate half of SPEC 11.1. estop and release never reach here (the hub
// owns both). The hub latches PAUSE on an accepted pause and clears it on an
// accepted resume, and has already refused a resume under ESTOP, override or
// home_required, so resume needs no gate of its own. override latches PAUSE
// and the override mode in the hub on acceptance; return latches nothing, and
// tick() drops the hub's override bit when the arbiter reports the arrival.
// The four retired numbers are UNSUPPORTED_OP, so the hub latches NOTHING.
Ret ValenceDevice::applySafety(const IntentValueMap& requested) {
    const uint64_t op = fieldU64(findField(requested, 1), 0);
    switch (op) {
        case safety_ops::override:
            // The rail is the operator's from here; the suspended source stays
            // suspended, so hand and stream never command position at once.
            // ESTOP drops override, so it is never latched under one.
            if (motionCensus().estop) return refuse(NackCode::ESTOP_ACTIVE, kDetailEstop);
            motionOverride();
            GLOGW(kTag, "OVERRIDE: PAUSE held, the rail is the operator's, jog enabled");
            break;
        case safety_ops::return_op: {
            const MotionCensus c = motionCensus();
            if (c.estop) return refuse(NackCode::ESTOP_ACTIVE, kDetailEstop);
            // No override, or a return already running: nothing to start, and
            // the ECHO says what is true (no further gate, SPEC 11.1).
            const bool overrideOn = (_hub->safetyModes() & safety_mode_bits::OVERRIDE) != 0;
            if (overrideOn && !_returnPending) {
                // An arrival on the spot (unpowered, nothing to travel) counts
                // in the arbiter before this returns, and the census shows it
                // within a publish interval: tick() drops the hub's override
                // bit on that count exactly as for a rendered return.
                const ReturnStart r = motionReturn();
                if (r == ReturnStart::unpowered) return refuseUnpowered("return");
                if (r != ReturnStart::none) {
                    _returnsAtRequest = c.returns;
                    _returnPending = true;
                    GLOGI(kTag, r == ReturnStart::arrived
                                    ? "RETURN: already at the paused position, override drops"
                                    : "RETURN: back to the paused position at the jog set");
                }
            }
            break;
        }
        case safety_ops::pause:
            // Never refused: pausing needs no homing and no role (the catalog
            // marks it watch, ROLE-EXEMPT). The arbiter latches BEFORE it asks
            // for the brake, which closes the preemption window: the pattern
            // task (core 1, priority 4) can hold a half-stroke it built before
            // this call and submit it after; accept() refuses it, or it was
            // accepted first and is braked. The generator keeps its settings
            // and parks on the census's paused bit, so resume re-arms it.
            motionPause(true);
            GLOGW(kTag, "PAUSE: every source suspended, braking to rest");
            break;
        case safety_ops::resume:
            motionPause(false);
            GLOGI(kTag, "RESUME: sources re-armed");
            break;
        default:
            return Ret::err(NackCode::UNSUPPORTED_OP);
    }
    IntentValueMap applied{};
    applied.count = 1;
    applied.fields[0] = {1, IntentValue::ofU64(op)};
    return Ret::ok(applied);
}

// Pushed NOW rather than on the next tick: the push wakes the pattern task, so
// 0x1200 running and 0x1100 gen_running fall at the next publish.
void ValenceDevice::haltGenerator() {
    _pat.running = false;
    _pat.adv_running = false;
    _patDirty = false;
    if (boardFeatures().has_pattern) pushPattern();
}

// ---- 0x3101 home ---------------------------------------------------------------
Ret ValenceDevice::applyHome(const IntentValueMap& requested) {
    const auto* f1 = findField(requested, 1);   // op
    const auto* f2 = findField(requested, 2);   // stroke
    if (f1 && !numberOf(f1)) return refuseNotANumber(ch::home, 1);
    if (f2 && !numberOf(f2)) return refuseNotANumber(ch::home, 2);
    const uint64_t op = fieldU64(f1, 0);
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
            // in ValenceMotion.h; do not restate it (C-1). A completed home
            // clears home_required, and on this bench op it also releases a
            // held ESTOP latch into PAUSE. Both are tick()'s, on the next hub
            // tick: releaseEstop() and setHomeRequired() publish and
            // broadcast, and this runs inside the hub's own intent dispatch.
            const float asked = numberOf(f2).value_or(250.0f);
            const float stroke = motionForceHome(asked);
            _clearLatch = true;
            _homeDone = true;
            applied.count = 2;
            applied.fields[0] = {1, IntentValue::ofU64(2)};
            applied.fields[1] = {2, IntentValue::ofF32(stroke)};
            return Ret::ok(applied);
        }

        default:
            return Ret::err(NackCode::INVALID_VALUE);
    }
}

// §11.4: the sources this machine has. move is the operator's hand, both
// c2h motion streams are ONE machine-driven source -- a client uses 0x2100
// or 0x2101, never both, and ownership is what enforces that.
std::optional<uint8_t> ValenceDevice::sourceForChannel(uint16_t channel_id) {
    if (channel_id == ch::move) return uint8_t(MotionSource::Manual);
    if (channel_id == ch::motion_input || channel_id == ch::motion_segment)
        return uint8_t(MotionSource::Stream);
    // Each generator's run/stop writer is its own source (RFC-093). Ownership
    // is what lets a session release stop it (onSourceOwnership); the preset
    // writer only sets knobs and owns nothing.
    if (channel_id == ch::pattern_cmd) return uint8_t(MotionSource::Pattern);
    if (channel_id == ch::pattern_advanced_cmd) return uint8_t(MotionSource::Advanced);
    return std::nullopt;
}

// §11.2 (b): the machine-domain precondition the library cannot see. The
// emitter's steering word IS that answer -- 0 means parked, and the engine
// resets itself one motion tick after the latch, so busy falls too.
// This is also the ONE hook the hub calls on the release path and the hub
// guarantees the release proceeds iff it returns true, so dropping the
// arbiter's latch here keeps both sides in lockstep: both land in PAUSE.
// Release never rehomes: a power-cutting ESTOP left homed false, and motion
// stays refused until force_home and then resume.
// The release asks for motor power back (the enable sequence, val-091.24).
// The switch's own fault line still low is the latched cause unresolved:
// refused. A self-check that holds power off is not the latch's cause: the
// release lands in PAUSE with the switch off, and the arbiter's power gate
// refuses motion until it is on.
// The e-stop contacts are checked FIRST, whatever latched the ESTOP: a button
// still pressed at the machine is a cause unresolved (§11.2 (a)), and its
// refusal names the act that resolves it.
bool ValenceDevice::canClearEstop() {
    _nackDetail[0] = '\0';
    if (const char* held = estopRefusal(estopInputRead())) {
        GLOGW(kTag, "release refused CLEAR_REFUSED: %s", held);
        noteDetail(held);
        return false;
    }
    const MotionCensus c = motionCensus();
    if (c.busy || c.step_q8 != 0) {
        noteDetail("motion not at rest");
        return false;
    }
    const motorswitch::Refusal why = motorSwitchRequestEnable();
    if (why == motorswitch::Refusal::fault_line) {
        GLOGW(kTag, "release refused CLEAR_REFUSED: %s", motorswitch::refusalName(why));
        noteDetail(motorswitch::refusalName(why));
        return false;
    }
    if (why != motorswitch::Refusal::none)
        GLOGW(kTag, "release: motor power stays off, %s", motorswitch::refusalName(why));
    motionEstopClear();
    return true;
}

// §11.2: motion stops before protocol bookkeeping. The emitter is parked on
// THIS task inside motionEstop(), before this returns, so the stop precedes
// the latch as the spec requires.
void ValenceDevice::onEstop(uint8_t cause, uint8_t origin) {
    motionEstop();
    _returnPending = false;   // the arbiter dropped override and the return
    // The generators stop with the machine and stay stopped: clearing the
    // latch never restarts one (the arbiter's rail stays closed until a
    // start). This makes each running flag tell the same truth.
    haltGenerator();
    GLOGW(kTag, "ESTOP latched: cause=%u origin=%u", unsigned(cause), unsigned(origin));
    (void)cause;
    (void)origin;
}

// ---- 0x2100 / 0x2101 stream ingress --------------------------------------------
// Runs on the hub task, synchronously inside Hub::update(). The hub has
// already validated the §5.4 caps, the granted rate, ownership and the
// deadman (hub.hpp's contract on this method); this decodes and submits and
// re-checks none of it. Decoding is BY FIXED OFFSET against the catalog's
// own 0x2100 (4 B point) / 0x2101 (6 B timed segment) field order -- the
// same convention the publishers above encode with.
void ValenceDevice::onStreamBundle(uint16_t channel_id, uint32_t session_id,
                                   const BundleView& bundle) {
    const bool isSegment = (channel_id == ch::motion_segment);
    if (channel_id != ch::motion_input && !isSegment) return;

    // SPEC 11.1 PAUSE: the hub drops a source-mapped bundle whole while PAUSE
    // is latched and never calls this. A sample queued before the latch is
    // refused at accept() by the arbiter's own pause gate.
    const uint8_t n = bundle.sampleCount();

    // RFC-030: the session's GRANTED (post-curve-policy) family, looked up
    // once per bundle. Chase points never carry one -- the family is a
    // waveform-reconstruction concept.
    const uint8_t curveFamily =
        (isSegment && _hub != nullptr) ? _hub->publishCurveFamily(session_id, channel_id) : 0;

    // t_base/t_off are u32 HUB-us, the same wrapping domain the hub clock reads
    // (§7.2); BundleView already scaled a segments t_off from its 100 us unit.
    // now64 stays the FULL 64-bit reading so the anchor never wraps itself;
    // only the WIRE stamp being resolved against it does.
    const int64_t now64 = int64_t(deviceNowUs());
    const uint32_t now32 = uint32_t(uint64_t(now64) & 0xFFFFFFFFull);
    // RFC-084 lead cap, one per kind: a sample stamped further ahead than its
    // cap is CLAMPED to it, never dropped. Segments ride the grant's horizon
    // (RFC-087); samples the registry's max_future_schedule_ms.
    const int32_t leadCapUs = int32_t(isSegment ? scheduleHorizonMs(channel_id)
                                                : limits::max_future_schedule_ms) * 1000;

    // Normalized samples map onto the client-frame window (RFC-088); the
    // arbiter mirrors the result to the physical rail.
    const Window w = clientWindow(motionCensus().rail_mm);
    const float span = w.hi - w.lo;
    uint32_t dropped = 0;
    uint32_t farClamped = 0;
    for (uint8_t i = 0; i < n; ++i) {
        // Nearest-window resolve (§7.2): a wrap-aware signed subtract, safe
        // because the wire stamp is near now by construction (the bundle
        // span is capped far under the 32-bit wrap).
        int32_t delta = int32_t(bundle.sampleTimeUs(i) - now32);
        if (delta > leadCapUs) { delta = leadCapUs; ++farClamped; }
        if (delta < 0) delta = 0;

        // The field mapping is StreamIntent.h's, shared with the offline
        // planner; a zero-duration segment decodes to nullopt and is dropped.
        const auto sample = bundle.sample(i);
        const uint16_t pos = getU16(sample.subspan(0, 2));
        const uint64_t anchor = uint64_t(now64 + int64_t(delta));
        const std::optional<MotionIntent> in =
            isSegment ? segmentIntent(pos, getU16(sample.subspan(2, 2)),
                                      int16_t(getU16(sample.subspan(4, 2))), w.lo, span,
                                      curveFamily, anchor)
                      : pointIntent(pos, int16_t(getU16(sample.subspan(2, 2))), w.lo, span, anchor);
        if (!in || !motionSubmit(*in)) ++dropped;
    }

    motionNoteStream(1, n, dropped);
    if (farClamped) {
        GLOGW_EVERY_MS(2000, kTag,
                       "motion stream: %u sample(s) clamped from a far-future t_off "
                       "(missed CLOCK resync on the client?)", unsigned(farClamped));
    }
}

void ValenceDevice::onSessionJoined(uint32_t session_id) {
    GLOGI(kTag, "session %lu joined", static_cast<unsigned long>(session_id));
    (void)session_id;
}

void ValenceDevice::onSessionLeft(uint32_t session_id) {
    GLOGI(kTag, "session %lu left", static_cast<unsigned long>(session_id));
    (void)session_id;
}

// §11.4: every transition lands in _owner, the jog's SOURCE_CONFLICT answer.
// A release (owner 0) comes on a session's way out, whether it went STALE or
// was torn down, or when the source went quiet (RFC-098, a stopped generator
// is already off); the reason is deliberately not consulted. The generators
// are the hub-autonomous sources, and background_run is the operator's one
// standing answer, for both, to "keep going with nobody attached?".
void ValenceDevice::onSourceOwnership(uint8_t source_id, uint32_t owner_session, uint8_t reason) {
    if (source_id < _owner.size()) _owner[source_id] = owner_session;
    if (owner_session != 0) return;
    const auto gen = MotionSource(source_id);
    if (gen != MotionSource::Pattern && gen != MotionSource::Advanced) return;
    bool& run = gen == MotionSource::Advanced ? _pat.adv_running : _pat.running;
    const char* name = kMotionSourceNames[source_id];
    if (_pat.ownerReleased(run)) {
        _patDirty = true;
        motionReleaseRail(gen);
        GLOGI(kTag, "%s stopped: its session released it (reason %u), background_run off", name,
              unsigned(reason));
    } else if (run) {
        GLOGW(kTag, "%s running UNATTENDED: its session released it, background_run on", name);
    }
    (void)reason;
    (void)name;
}

// RFC-098 (SPEC 11.4): each source's source_kinds value on control-owner. The
// hub releases the stream on its own bundle clock.
uint8_t ValenceDevice::sourceKind(uint8_t source_id) {
    switch (MotionSource(source_id)) {
        case MotionSource::Manual:   return source_kinds::jog;
        case MotionSource::Stream:   return source_kinds::stream;
        case MotionSource::Pattern:  return source_kinds::classic;
        case MotionSource::Advanced: return source_kinds::advanced;
    }
    return source_kinds::reserved;
}

// RFC-098: a generator is quiet once stopped and no longer driving (its stop's
// brake done); the jog once the motion task took the move and the plan is at
// rest. A refused jog submitted nothing, so its slot is quiet at rest.
bool ValenceDevice::sourceQuiet(uint8_t source_id) {
    switch (MotionSource(source_id)) {
        case MotionSource::Manual: {
            const MotionCensus c = motionCensus();
            const bool taken = !_jogMark || c.intents + c.rejected != *_jogMark;
            return taken && !c.busy && c.step_q8 == 0;
        }
        case MotionSource::Pattern:  return !_pat.running && !patternActive();
        case MotionSource::Advanced: return !_pat.adv_running && !patternActive();
        case MotionSource::Stream:   return false;
    }
    return false;
}

// §8.7 store items over BLOB_REQ ns=1, each carrying its RFC-073 digest
// (PatternPresetStore::encodeItem). The hub has already enforced the declaring
// entry's access floor; an empty slot answers nullopt, which the hub NACKs
// CHUNK_UNAVAILABLE (honest and enumerable).
std::optional<HubDelegate::BlobView> ValenceDevice::readBlob(uint8_t ns, uint8_t store_id, uint8_t slot) {
    if (!boardFeatures().has_pattern || ns != blob_ns::store || store_id != kPresetStoreId)
        return std::nullopt;
    static_assert(PatternPresetStore::itemMaxBytes(std::string_view(kPresetKind).size()) <=
                      sizeof(_blobScratch),
                  "a preset item with its digest outgrew the blob scratch");
    const size_t n = _presets.encodeItem(slot, kPresetKind, _blobScratch);
    if (n == 0) return std::nullopt;
    BlobView v;
    v.bytes = std::span<const std::byte>(_blobScratch.data(), n);
    v.generation = _presets.generation();
    return v;
}

// ---- retained STATE --------------------------------------------------------------

void ValenceDevice::publishHubStatus() {
    // 4+4+1+1+4+1+1 = 16 B, matching the 0x0006 layout in ValenceCatalog.h.
    const int8_t rssi = _linkRssi.load(std::memory_order_relaxed);
    _mswSent = motorSwitchStatus();
    std::array<std::byte, 16> buf{};
    std::span<std::byte> s(buf);
    putU32(s.subspan(0, 4), deviceFreeHeapBytes());
    putU32(s.subspan(4, 4), uint32_t(deviceNowUs() / 1000000));
    putU8(s.subspan(8, 1), uint8_t(rssi));
    putU8(s.subspan(9, 1), uint8_t(_hub->sessionCount()));
    putU32(s.subspan(10, 4), _hub->logDropped());
    putU8(s.subspan(14, 1), uint8_t(_mswSent.state));
    putU8(s.subspan(15, 1), uint8_t(_mswSent.last_fault));
    _hub->publishState(channels::hub_status, s);
}

ValenceDevice::Window ValenceDevice::clientWindow(float rail) const {
    if (!_modes.flipped) return {_cfg.window_min, _cfg.window_max};
    return {std::max(0.0f, rail - _cfg.window_max), std::max(0.0f, rail - _cfg.window_min)};
}

// RFC-088's gate as the enabled_mask shows it: a homed rail at rest, owned by
// no source, not under override. applyModes() refuses each case by its own
// code.
bool ValenceDevice::flipOpen(const MotionCensus& c) const {
    const bool overrideOn = _hub != nullptr &&
                            (_hub->safetyModes() & safety_mode_bits::OVERRIDE) != 0;
    return c.homed && !c.estop && !railOwned() && !overrideOn && !c.override_mode &&
           !c.busy && c.step_q8 == 0 && !windowOnTrial();
}

bool ValenceDevice::windowOnTrial() const {
    return _hub != nullptr && (_hub->trialBaselineOf(ch::config_set, 1) || _hub->trialBaselineOf(ch::config_set, 2));
}

void ValenceDevice::publishMachineConfig() {
    // 38 B, matching the 0x1000 layout in ValenceCatalog.h. The window is the
    // client frame's (RFC-088).
    const StoredConfig& c = _cfg;
    const Window w = clientWindow(motionCensus().rail_mm);
    std::array<std::byte, 38> buf{};
    std::span<std::byte> s(buf);
    putF32(s.subspan(0, 4), w.lo);
    putF32(s.subspan(4, 4), w.hi);
    putF32(s.subspan(8, 4), c.jog_speed);
    putF32(s.subspan(12, 4), c.jog_accel);
    putF32(s.subspan(16, 4), c.input_speed);
    putF32(s.subspan(20, 4), c.input_accel);
    putF32(s.subspan(24, 4), c.max_rail);
    putF32(s.subspan(28, 4), c.input_jerk);
    // enabled_mask: all eight limits are writable at all times on this hub.
    // Nothing refuses a config-set; out-of-range values clamp, which is what
    // min/max is for. A bit held low here would gray a control the machine
    // would in fact accept.
    putU8(s.subspan(32, 1), 0xFF);
    // measured_stroke: 0 means NOT MEASURED. force_home ASSERTS a stroke that
    // nothing measured, and the arbiter's rail is where that assertion lives;
    // reporting it here would dress an assertion as a measurement. This board
    // has no way to measure a stroke, so the field is 0 forever.
    putF32(s.subspan(33, 4), 0.0f);
    putU8(s.subspan(37, 1), uint8_t(_hub->trialMask(ch::machine_config)));   // trial_mask (RFC-099)
    _hub->publishState(ch::machine_config, s);
    _lastPublishedCfg = c;
    _sentWindow = w;
    _cfgEverSent = true;
}

void ValenceDevice::pushConfigToMotion() const {
    motionSetCommissioned(commissioned(_modes));
    motionSetFlipped(_modes.flipped);
    motionSetWindow(_cfg.window_min, _cfg.window_max, _cfg.max_rail);
    motionSetJogLimits(_cfg.jog_speed, _cfg.jog_accel);
    motionSetInputLimits(_cfg.input_speed, _cfg.input_accel, _cfg.input_jerk);
}

// ---- persistence ----------------------------------------------------------------

// Decodes straight into the members: decodeConfig() assigns its outputs only
// once every check has passed, so a refused blob leaves the factory values.
std::expected<void, stored::ConfigReject> ValenceDevice::adoptConfigBlob(std::span<const std::byte> blob,
                                                                         uint16_t& cfgGen) {
    const auto adopted =
        stored::decodeConfig(blob, motionDefaultTuning().overshoot_guard, _cfg, _tune, _modes, cfgGen);
    if (!adopted) return adopted;
    _cfgDirty = false;
    // The engine adopts through the live write's own door, never a side path.
    motionSetTuning(_tune);
    return {};
}

// The stored values are the live ones with every trialed key put back to its
// pre-trial value (SPEC 9.3: a trial value is never persisted before its
// commit). The window baseline is in the client frame, mapped by the flip and
// rail of now; the flip is refused while a window key is on trial.
size_t ValenceDevice::encodeConfigBlob(std::span<std::byte> out, uint16_t cfgGen) const {
    StoredConfig c = _cfg;
    MotionTuning t = _tune;
    if (_hub != nullptr && _hub->trialCount() > 0) {
        const float rail = motionCensus().rail_mm;
        for (uint8_t k = 1; k <= 8; ++k)
            if (const auto b = _hub->trialBaselineOf(ch::config_set, k))
                setConfigKey(c, k, baselineNumber(k, *b), rail, _modes.flipped);
        if (const auto b = _hub->trialBaselineOf(ch::modes_set, 4))
            t.overshoot_guard = overshootGuardFor(baselineNumber(4, *b) != 0.0f, motionDefaultTuning().overshoot_guard);
        for (const uint8_t k : kTuningKeys)
            if (const auto b = _hub->trialBaselineOf(ch::kinetic_set, k)) (void)setTuningKey(t, k, baselineNumber(k, *b));
    }
    return stored::encodeConfig(out, c, t, _modes, cfgGen);
}

// ---- RFC-099 trial writes ---------------------------------------------------

std::optional<IntentValue> ValenceDevice::trialBaseline(uint16_t channel_id, uint8_t key) {
    if (channel_id == ch::config_set) {
        const Window w = clientWindow(motionCensus().rail_mm);
        switch (key) {
            case 1: return IntentValue::ofF32(w.lo);
            case 2: return IntentValue::ofF32(w.hi);
            case 3: return IntentValue::ofF32(_cfg.jog_speed);
            case 4: return IntentValue::ofF32(_cfg.jog_accel);
            case 5: return IntentValue::ofF32(_cfg.input_speed);
            case 6: return IntentValue::ofF32(_cfg.input_accel);
            case 7: return IntentValue::ofF32(_cfg.input_jerk);
            case 8: return IntentValue::ofF32(_cfg.max_rail);
            default: return std::nullopt;
        }
    }
    if (channel_id == ch::modes_set && key == 4) return IntentValue::ofU64(_tune.overshoot_guard > 0.0f ? 1 : 0);
    if (channel_id == ch::kinetic_set && key != kChaseDenseKey) return tuningKeyValue(_tune, key);
    return std::nullopt;
}

Ret ValenceDevice::applyTrialIntent(uint16_t channel_id, const IntentValueMap& requested, AccessLevel role,
                                    bool& cfgChanged) {
    _trialApply = true;
    Ret r = applyIntent(channel_id, requested, role, cfgChanged);
    _trialApply = false;
    return r;
}

// Through the writers, so the clamps, the ECHO-side copies and the republish
// are the live write's own. Revert MUST NOT refuse: a window bound the other
// bound no longer admits lands one step inside it.
void ValenceDevice::restoreTrial(uint16_t channel_id, const IntentValueMap& baselines, bool& cfgChanged) {
    IntentValueMap m = baselines;
    if (channel_id == ch::config_set) {
        const Window now = clientWindow(motionCensus().rail_mm);
        IntentValueField* lo = nullptr;
        IntentValueField* hi = nullptr;
        for (uint32_t i = 0; i < m.count; ++i) {
            if (m.fields[i].key == 1) lo = &m.fields[i];
            if (m.fields[i].key == 2) hi = &m.fields[i];
        }
        const float hiV = hi ? baselineNumber(2, hi->value) : now.hi;
        if (lo && baselineNumber(1, lo->value) >= hiV) lo->value = IntentValue::ofF32(std::max(0.0f, hiV - 1.0f));
        const float loV = lo ? baselineNumber(1, lo->value) : now.lo;
        if (hi && baselineNumber(2, hi->value) <= loV) hi->value = IntentValue::ofF32(loV + 1.0f);
    }
    _trialApply = true;
    const Ret r = channel_id == ch::config_set ? applyConfig(m, cfgChanged)
                : channel_id == ch::modes_set  ? applyModes(m, cfgChanged)
                : channel_id == ch::kinetic_set ? applyTuning(m, cfgChanged)
                                                : Ret::err(NackCode::UNSUPPORTED_OP);
    _trialApply = false;
    if (!r) GLOGW(kTag, "trial revert on %04x refused 0x%04x", unsigned(channel_id), unsigned(r.error()));
}

// Commit, or a durable write by the trial's own session: the live value is
// now the stored one. Armed even when nothing moved, since the stored blob
// held the baseline until now.
void ValenceDevice::onTrialCommit(uint16_t channel_id, const IntentValueMap& keys) {
    if (channel_id == ch::config_set) {
        uint8_t wrote = 0;
        for (uint32_t i = 0; i < keys.count; ++i)
            if (keys.fields[i].key >= 1 && keys.fields[i].key <= 8) wrote |= uint8_t(1u << (keys.fields[i].key - 1));
        noteSetupWritten(wrote);
    }
    _durableDirty = true;
    _cfgDirty = true;
}

bool ValenceDevice::adoptPresetsBlob(std::span<const std::byte> blob) {
    if (!_presets.decode(blob)) return false;
    _presetsGenSeen = _presets.generation();
    return true;
}

void ValenceDevice::attach(Hub& hub, const Catalog32& catalog) {
    _hub = &hub;
    _catalog = &catalog;
    // The declaration's one home is the composition's setEstopCutsPower().
    motionSetEstopCutsPower(hub.estopCutsPower());
    hub.publishControlOwnerStateIfPresent();
    publishMachineConfig();
    publishHubStatus();
    // _tune is the factory set or the adopted one, and the engine already holds
    // it either way (adoptConfigBlob pushed it), so nothing is pushed here.
    publishMachineModes(hub, _tune, _modes, !publishGrantLive(ch::motion_segment), flipOpen(motionCensus()));
    publishKineticCards(hub, _tune, kCardLimits | kCardChase | kCardWaveform);
    const MotionCensus mo = motionCensus();
    publishMotion(hub, mo, patternActive());
    publishPlanStrip(hub, mo);
    publishMotionDiag(hub, mo, _ingressDrops.total());
    publishOdometer(hub, mo);
    if (boardFeatures().has_pattern) {
        pushPattern();
        publishPatternPlane(mo);
    }
}

// ---- the board buttons -----------------------------------------------------------
// Hub task, from tick(): outside the hub's intent dispatch, so the hub calls
// below (openPresenceWindow, latchEstop) may publish and broadcast.

void ValenceDevice::serviceButtons(uint32_t nowMs) {
    const button::Gesture home = homeButtonTake();
    const button::Gesture pair = pairButtonTake();
    if (_reboot == Reboot::none) {
        if (home == button::Gesture::press) homeFromButton();
        else if (home == button::Gesture::hold) beginReboot(nowMs);
        if (pair == button::Gesture::press) {
            _hub->openPresenceWindow();
            GLOGW(kTag, "PAIR press: presence window open for %lu s (SPEC 12.3)",
                  static_cast<unsigned long>(limits::pairing_window_default_s));
        } else if (pair == button::Gesture::hold) {
            GLOGW(kTag, "PAIR hold: no binding");
        }
    }
    if (_reboot != Reboot::braking) return;
    const MotionCensus mo = motionCensus();
    const bool atRest = !mo.busy && mo.step_q8 == 0;
    if (!atRest && int32_t(nowMs - _rebootBrakeUntilMs) < 0) return;
    // The ESTOP is the power cut and the clients' truth before the GOODBYE:
    // the hub's own initiation, at its highest tier, like the fault latch.
    if (!_hub->estopLatched()) _hub->latchEstop(safety_causes::user, uint8_t(AccessLevel::configure));
    _reboot = Reboot::due;
    GLOGW(kTag, "HOME hold: %s, motor power cut, rebooting", atRest ? "at rest" : "brake timed out");
}

// The same door as the wire's home op 1, refusal included. force_home (op 2)
// is never reachable from the button: it asserts a stroke nothing measured
// (RFC-025).
void ValenceDevice::homeFromButton() {
    IntentValueMap m{};
    m.count = 1;
    m.fields[0] = IntentValueField{1, IntentValue::ofU64(1)};
    const Ret r = applyHome(m);
    if (r) GLOGW(kTag, "HOME press: homing");
    else GLOGW(kTag, "HOME press: home refused, NACK 0x%04x (no homing cycle on this board yet)",
               unsigned(r.error()));
}

// PAUSE's brake without PAUSE's latch: the hub's safety word moves once,
// to ESTOP, when the brake is done (serviceButtons()).
void ValenceDevice::beginReboot(uint32_t nowMs) {
    GLOGW(kTag, "HOME hold: graceful reboot: braking, then ESTOP, GOODBYE REBOOTING, NVS flush, restart");
    motionPause(true);
    _reboot = Reboot::braking;
    _rebootBrakeUntilMs = nowMs + kRebootBrakeMs;
}

uint8_t ValenceDevice::takePendingPersist() {
    uint8_t due = 0;
    if (_persistArmed) due |= kPersistConfig;
    if (_presetsArmed) due |= kPersistPresets;
    _persistArmed = false;
    _presetsArmed = false;
    return due;
}

uint8_t ValenceDevice::tick(uint32_t nowMs) {
    serviceButtons(nowMs);
    // force_home's two hub-side effects, one tick after applyIntent because
    // both publish and broadcast and applyIntent runs inside the hub's intent
    // dispatch. The release runs canClearEstop(), so the hub and the arbiter
    // land in PAUSE together or not at all.
    if (_clearLatch) {
        _clearLatch = false;
        if (_hub->estopLatched()) {
            if (_hub->releaseEstop()) GLOGW(kTag, "ESTOP released by force_home: PAUSE, resume to run");
            else GLOGW(kTag, "force_home could not release the ESTOP latch: %s", _nackDetail.data());
        }
    }
    if (_homeDone) {
        _homeDone = false;
        _hub->setHomeRequired(false);
    }
    // The e-stop at the machine (val-091.23): a reading that stops latches
    // ESTOP whenever none is latched, through the same hub-side door as the
    // fault latch below (SPEC 11.2: onEstop() cuts power and parks first).
    // LEVEL, not edge: canClearEstop() refuses the release while the reading
    // stops, so the latch outlives the press, and the button's own release
    // clears nothing. After the home blocks above, so a force_home in this
    // tick cannot clear the home_required this latch sets.
    const estop::Reading es = estopInputRead();
    if (es.stops() && !_hub->estopLatched()) {
        GLOGW(kTag, "e-stop %s at the machine: latching ESTOP, cause %s",
              estop::contactsName(es.state), es.state == estop::Contacts::pressed ? "user" : "fault");
        _hub->latchEstop(estopCause(es.state), uint8_t(AccessLevel::configure));
    }
    // A motor switch fault already cut power with no ESTOP behind it. Latch
    // one, cause fault (SPEC 11.2), so the release is the explicit re-enable
    // and a power-cutting hub lands unhomed. The hub is its own initiator, at
    // its highest tier. The seq is the hub's one SPEC 5.5 counter, never one
    // kept here: a count of this delegate's own initiations drifts from the
    // seq a raw 0xE5 frame or the `estop` op set.
    // An EN-node fault is also how the e-stop's hardware stop reaches the
    // switch, which sees it first: it waits kEstopNameMs for the block above
    // to latch it with the e-stop's own cause.
    const MotorSwitchStatus sw = motorSwitchStatus();
    if (sw.faults != _mswFaultsSeen) {
        if (!_mswFaultWaiting) {
            _mswFaultWaiting = true;
            _mswFaultSinceMs = nowMs;
        }
        const bool naming = sw.last_fault == motorswitch::Fault::en_node &&
                            uint32_t(nowMs - _mswFaultSinceMs) < kEstopNameMs;
        if (_hub->estopLatched() || !naming) {
            _mswFaultsSeen = sw.faults;
            _mswFaultWaiting = false;
            if (!_hub->estopLatched()) {
                GLOGE(kTag, "motor switch fault (%s): latching ESTOP, cause fault",
                      motorswitch::faultName(sw.last_fault));
                _hub->latchEstop(safety_causes::fault, uint8_t(AccessLevel::configure));
            }
        }
    }
    // DRV_ALM (val-091.29): the drive halted itself on an alarm. Latched like
    // the motor switch fault above: cause fault, the hub its own initiator at
    // its highest tier. A latch already held keeps its own cause.
    if (driveAlarmTake() && !_hub->estopLatched()) {
        GLOGE(kTag, "drive alarm (DRV_ALM): latching ESTOP, cause fault");
        _hub->latchEstop(safety_causes::fault, uint8_t(AccessLevel::configure));
    }
    // The motion plane, from ONE census so no two channels disagree about the
    // same instant. 0x1100 publishes at rate under its 60 Hz ceiling; 0x1110 is
    // a strip that is only news while a plan runs.
    const MotionCensus mo = motionCensus();

    // Every tick, not at the 1 Hz publish: a slot's counter dies with its
    // session, and a coarser read loses whatever it dropped since the last.
    for (size_t i = 0;; ++i) {
        const HubSession* s = _hub->sessionBySlot(i);
        if (s == nullptr) break;
        _ingressDrops.observe(i, s->occupied() ? s->session_id : 0,
                              _hub->streamIngressCounters(i).dropped);
    }

    // The schedule_horizon and flipped enabled_mask bits follow the segments
    // grants and the rail's state, which no write announces.
    const bool horizonOpen = !publishGrantLive(ch::motion_segment);
    const bool flipNowOpen = flipOpen(mo);
    if (horizonOpen != _horizonOpenSent || flipNowOpen != _flipOpenSent) {
        _horizonOpenSent = horizonOpen;
        _flipOpenSent = flipNowOpen;
        publishMachineModes(*_hub, _tune, _modes, horizonOpen, flipNowOpen);
    }

    // RETURN arrived (SPEC 11.1): override drops, plain PAUSE stays. A counter,
    // not the census flag, because the census lags the request by up to a
    // publish interval and would read "no override" before it ever started.
    if (_returnPending && mo.returns != _returnsAtRequest) {
        _returnPending = false;
        _hub->setOverride(false);
    }
    if (uint32_t(nowMs - _lastMotionMs) >= 33u) {
        _lastMotionMs = nowMs;
        publishMotion(*_hub, mo, patternActive());
    }
    if (uint32_t(nowMs - _lastPlanMs) >= 50u) {
        _lastPlanMs = nowMs;
        publishPlanStrip(*_hub, mo);
    }
    if (uint32_t(nowMs - _lastSlowMs) >= 1000u) {
        _lastSlowMs = nowMs;
        publishMotionDiag(*_hub, mo, _ingressDrops.total());
        publishOdometer(*_hub, mo);
    }

    if (_tuneDirty != 0) {
        motionSetTuning(_tune);
        if (_tuneDirty & kCardModes)
            publishMachineModes(*_hub, _tune, _modes, _horizonOpenSent, _flipOpenSent);
        publishKineticCards(*_hub, _tune, _tuneDirty);
        _tuneDirty = 0;
        // Same blob as 0x1000: the tuning write bumped cfg_gen too.
        if (_durableDirty) {
            _persistArmed = true;
            _persistDueMs = nowMs + kCfgPersistDebounceMs;
        }
    }
    if (_presets.generation() != _presetsGenSeen) {
        _presetsGenSeen = _presets.generation();
        _presetsArmed = true;
        _presetsDueMs = nowMs + kCfgPersistDebounceMs;
    }

    if (boardFeatures().has_pattern) {
        if (_patDirty) {
            _patDirty = false;
            pushPattern();
        }
        publishPatternPlane(mo);
    }

    const bool dirty = _cfgDirty;
    _cfgDirty = false;
    if (_cfgEverSent && clientWindow(mo.rail_mm) != _sentWindow) {
        // The frame moved (a flip, or the rail under one), not the setting.
        publishMachineConfig();
        _patDirty = true;
    }
    if (dirty || (_cfgEverSent && !(_lastPublishedCfg == _cfg))) {
        pushConfigToMotion();
        publishMachineConfig();
        _patDirty = true;   // the stroke frame is this config's window
        // Re-armed, not accumulated: the write lands only after the changes
        // stop. cfg_gen is read at write time, by which point Hub::update() has
        // already applied this tick's bump (§4.2). A trial alone arms nothing.
        if (_durableDirty) {
            _persistArmed = true;
            _persistDueMs = nowMs + kCfgPersistDebounceMs;
        }
    }
    _durableDirty = false;
    // RFC-099: the trial_mask on every settings card follows the hub's sets.
    if (_hub->trialGen() != _trialGenSent) {
        _trialGenSent = _hub->trialGen();
        publishMachineConfig();
        publishMachineModes(*_hub, _tune, _modes, _horizonOpenSent, _flipOpenSent);
        publishKineticCards(*_hub, _tune, kCardLimits | kCardChase | kCardWaveform);
    }

    // 1 Hz, and at once when the motor switch moved: a client watching a
    // release sees pre-charging, then on, without waiting out the second.
    const MotorSwitchStatus msw = motorSwitchStatus();
    if (uint32_t(nowMs - _lastStatusMs) >= 1000u || msw.state != _mswSent.state ||
        msw.last_fault != _mswSent.last_fault) {
        _lastStatusMs = nowMs;
        publishHubStatus();
    }

    uint8_t due = 0;
    if (_persistArmed && int32_t(nowMs - _persistDueMs) >= 0) {
        _persistArmed = false;
        due |= kPersistConfig;
    }
    if (_presetsArmed && int32_t(nowMs - _presetsDueMs) >= 0) {
        _presetsArmed = false;
        due |= kPersistPresets;
    }
    return due;
}

}  // namespace valence
