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
//   NVS pages; the arithmetic lives on ValenceHub.cpp's file header. A due
//   write then waits for a STILL WINDOW (kPersistStillUs, tick()): a flash
//   write turns the cache off, the motion tasks cannot renew the LP lease, and
//   the carriage would stop mid-motion (operator ruling 2026-10-09, bd
//   val-4rr). Settings apply in RAM at once; only the write waits. What a
//   blob holds is StoredState.h's and PatternPresetStore's; background_run is
//   in neither, on purpose (bd val-wcm, pending ruling).
// See: Valence SPEC.md §4.2, §6.3, §9.1, §9.3, §11.2, §11.4

#include "ValenceDevice.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <optional>
#include <string_view>

#include "ValenceEstopDatagram.h"
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
constexpr const char* kDetailPastStroke = "window past the measured stroke";

// 0x4100's ceiling: a burst of this many anomaly events, then one per
// kAnomalyEventMs. A planner in trouble records anomalies at its plan rate,
// and every EVENT channel a session holds shares one bounded queue, so an
// uncapped feed would push the safety and log events out of it. 0x1111's
// counters keep the exact count; the seq gap shows what the cap skipped.
constexpr uint32_t kAnomalyEventBurst = 8;
constexpr uint32_t kAnomalyEventMs = 100;

// Quiet time after the last applied change before a blob's persist is due,
// the same for both blobs. It coalesces a burst (a slider drag streams
// 0x3120 writes; a rename follows a save) into ONE flash write after the
// operator lets go: a write per INTENT would put an NVS commit, and now and
// then a sector erase, on the hub task at the intent rate.
constexpr uint32_t kCfgPersistDebounceMs = 2000;

// A due persist is written only while the published plan holds the carriage
// still for this long from now (motionStillFor()), under ESTOP, and never
// during a home cycle; until then it stays armed and coalesces, one write of
// the latest state. The worst-case NVS erase+write plus margin, inside the
// published strip (MotionArbiter.h kStripLen): the operator's starting
// value, NOT MEASURED on the P4.
// TODO(val-ggj): set it from the bench measurement.
constexpr uint32_t kPersistStillUs = 60000;

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
void packI32(std::span<std::byte> o, size_t& n, int32_t v)  { packU32(o, n, uint32_t(v)); }
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
int32_t wireI32(float v, float scale) {
    const float x = v * scale;
    if (!std::isfinite(x)) return 0;
    // The largest floats below 2^31 in magnitude: the conversion stays defined.
    if (x >= 2147483520.0f) return std::numeric_limits<int32_t>::max();
    if (x <= -2147483648.0f) return std::numeric_limits<int32_t>::min();
    return int32_t(x >= 0.0f ? x + 0.5f : x - 0.5f);
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

// The narrowest window the hub stores when it has to place one itself.
constexpr float kWindowFloorMm = 1.0f;

// SPEC 9.6 (RFC-101): the stored window lies inside [0, max_rail], held there
// by every write. A window wholly past max_rail has no nearest legal value: it
// collapses onto the rail's far end, kWindowFloorMm wide, where the arbiter's
// own clamp already held the carriage. True when it collapsed.
bool holdWindowInRail(StoredConfig& c) {
    if (c.window_min < c.max_rail) {
        c.window_max = std::min(c.window_max, c.max_rail);
        return false;
    }
    c.window_max = c.max_rail;
    c.window_min = std::max(0.0f, c.max_rail - kWindowFloorMm);
    return true;
}

// The 0x3120 schema's keys in wire order (ValenceCatalog.h, kinetic-set).
constexpr std::array<uint8_t, 8> kTuningKeys{1, 2, 3, 4, 5, 6, 7, 8};
// chase_dense: a live samples grant commits to it (applyTuning()).
constexpr uint8_t kChaseDenseKey = 7;

// One 0x3120 key into `t`, clamped into tuning_bounds; the value as the ECHO
// carries it. nullopt for a key outside the schema.
std::optional<IntentValue> setTuningKey(MotionTuning& t, uint8_t key, float v) {
    namespace b = tuning_bounds;
    switch (key) {
        case 1:  t.jmax_ovr = clampf(v, 0.0f, b::jmax_ovr_max);  return IntentValue::ofF32(t.jmax_ovr);
        case 2:  t.vmax_ovr = clampf(v, 0.0f, b::vmax_ovr_max);  return IntentValue::ofF32(t.vmax_ovr);
        case 3:  t.amax_ovr = clampf(v, 0.0f, b::amax_ovr_max);  return IntentValue::ofF32(t.amax_ovr);
        case 4:  t.smoothness = clampf(v, 0.0f, b::smoothness_max); return IntentValue::ofF32(t.smoothness);
        case 5:  t.handle_floor = clampf(v, b::handle_floor_min, b::handle_floor_max);
                 return IntentValue::ofF32(t.handle_floor);
        case 6:  t.trim_max = clampf(v, b::trim_max_min, b::trim_max_max); return IntentValue::ofF32(t.trim_max);
        case 7:  // ms on the wire, us in the engine
            t.chase_dense_us = uint32_t(clampf(v, b::dense_ms_min, b::dense_ms_max) * 1000.0f + 0.5f);
            return IntentValue::ofF32(float(t.chase_dense_us) / 1000.0f);
        case 8:  // ms on the wire, us in the engine
            t.react_us = uint32_t(clampf(v, 0.0f, b::react_ms_max) * 1000.0f + 0.5f);
            return IntentValue::ofF32(float(t.react_us) / 1000.0f);
        default: return std::nullopt;
    }
}

// The current value of one 0x3120 key, exactly as setTuningKey() echoes it.
std::optional<IntentValue> tuningKeyValue(const MotionTuning& t, uint8_t key) {
    switch (key) {
        case 1:  return IntentValue::ofF32(t.jmax_ovr);
        case 2:  return IntentValue::ofF32(t.vmax_ovr);
        case 3:  return IntentValue::ofF32(t.amax_ovr);
        case 4:  return IntentValue::ofF32(t.smoothness);
        case 5:  return IntentValue::ofF32(t.handle_floor);
        case 6:  return IntentValue::ofF32(t.trim_max);
        case 7:  return IntentValue::ofF32(float(t.chase_dense_us) / 1000.0f);
        case 8:  return IntentValue::ofF32(float(t.react_us) / 1000.0f);
        default: return std::nullopt;
    }
}

// A trial baseline as a number. Baselines are only ever F32 or U64 here.
float baselineNumber(uint8_t key, const IntentValue& v) {
    const IntentValueField f{key, v};
    return numberOf(&f).value_or(0.0f);
}

// The three tuning cards, one bit each in ValenceDevice::_tuneDirty.
constexpr uint8_t kCardModes    = 0x01;  // 0x1030
constexpr uint8_t kCardLimits   = 0x02;  // 0x1120
constexpr uint8_t kCardPlanner  = 0x08;  // 0x1122

// Which cards differ between two tuning sets. Field-to-card membership is the
// catalog's (ValenceCatalog.h, the kinetic-* and machine-modes entries).
uint8_t cardsChanged(const MotionTuning& a, const MotionTuning& b) {
    uint8_t m = 0;
    if (a.home_speed != b.home_speed)
        m |= kCardModes;
    if (a.jmax_ovr != b.jmax_ovr || a.vmax_ovr != b.vmax_ovr || a.amax_ovr != b.amax_ovr)
        m |= kCardLimits;
    if (a.smoothness != b.smoothness || a.handle_floor != b.handle_floor || a.trim_max != b.trim_max ||
        a.chase_dense_us != b.chase_dense_us || a.react_us != b.react_us)
        m |= kCardPlanner;
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
    // homing is a home op 1 cycle asked for or running. gen_running is the
    // generator DRIVING, not merely switched on (patternActive()).
    packU8(buf, n, uint8_t((m.homed ? 0x01u : 0u) | (m.homing ? 0x02u : 0u) | (genRunning ? 0x04u : 0u) |
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
    std::array<std::byte, 25> buf{};
    size_t n = 0;
    // flags: active, live_mode, grad_mode. live_mode and grad_mode named a
    // legacy interpolator split that has no counterpart in this engine.
    packU8(buf, n, m.busy ? 0x01u : 0u);
    // style: the planner's style (MotionArbiter.cpp PlanStyle), or `hold` for a hold segment, so a long dwell
    // reads as a live plan and never as a stall.
    packU8(buf, n, m.plan_hold ? kPlanStyleHold : m.mode);
    packI32(buf, n, wireI32(m.plan_start, 10000.0f));
    packI32(buf, n, wireI32(m.plan_end, 10000.0f));
    packI32(buf, n, wireI32(m.plan_cur, 10000.0f));
    packI16(buf, n, wireI16(m.plan_vel, 1000.0f));
    packU32(buf, n, m.plan_duration_us);
    packU32(buf, n, m.plan_elapsed_us);
    packU8(buf, n, m.plan_flags);   // RFC-100 plan.flags
    publishPacked(hub, ch::plan_strip, buf, n);
}

// hubDropped: bundles the hub dropped whole at ingress (IngressDropTally.h).
// segBundles: the motion-segment share of sync_bundles.
void publishMotionDiag(Hub& hub, const MotionCensus& m, uint32_t hubDropped, uint32_t segBundles) {
    // The layout ValenceCatalog.h's kinetic-diag entry sums: 48 B around one
    // u32 per kind, kind 0 (never counted) excepted.
    std::array<std::byte, 48 + 4 * (kAnomalyKinds - 1)> buf{};
    size_t n = 0;
    packU32(buf, n, m.plans);
    packU32(buf, n, m.failures);
    packU32(buf, n, m.anomalies);
    packU8(buf, n, m.mode);
    packU8(buf, n, m.plan_kind);
    for (size_t k = 1; k < m.anom.size(); ++k) packU32(buf, n, m.anom[k]);
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
    packU32(buf, n, segBundles);
    // reset_gen: no action resets this group (SPEC 9.3), so it stays 0; a
    // reboot reads as a new boot_id, never as a reset.
    packU16(buf, n, 0);
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

// Layout per ValenceCatalog.h's machine-modes entry: 13 B, plus home_style
// only where has_drive put it in the catalog.
void publishMachineModes(Hub& hub, const MotionTuning& t, const StoredModes& m, bool horizonOpen,
                         bool flipOpen) {
    std::array<std::byte, 14> buf{};
    const bool drive = boardFeatures().has_drive;
    const size_t len = drive ? 14 : 13;
    size_t n = 0;
    packU8(buf, n, 0);   // blend_mode_reserved
    packU8(buf, n, 0);   // stream_speed_reserved
    packU8(buf, n, 0);   // overshoot_clamp_reserved
    // enabled_mask: home_style (has_drive only, bit 0) stays low: neither of
    // its styles runs here. schedule_horizon drops while a segments grant is
    // live (applyModes()). flipped drops whenever applyModes() would refuse it
    // (flipOpen()). home_speed is accepted at all times; a cycle reads it at
    // its start. datagram_estop is accepted at all times.
    const uint8_t horizonBit = drive ? 0x02 : 0x01;
    const uint8_t flipBit = uint8_t(horizonBit << 1);
    const uint8_t homeSpeedBit = uint8_t(flipBit << 1);
    const uint8_t datagramBit = uint8_t(homeSpeedBit << 1);
    packU8(buf, n, uint8_t((horizonOpen ? horizonBit : 0) | (flipOpen ? flipBit : 0) | homeSpeedBit | datagramBit));
    packU8(buf, n, 2);   // motion_backend, read-only: quadrature, the LP-core emitter
    if (drive) packU8(buf, n, 0);   // home_style
    packU8(buf, n, m.horizon);      // schedule_horizon
    packU8(buf, n, m.flipped ? 1 : 0);   // flipped
    packU8(buf, n, uint8_t(hub.trialMask(ch::machine_modes)));   // trial_mask (RFC-099)
    packF32(buf, n, t.home_speed);   // home_speed
    packU8(buf, n, m.datagram_estop ? 1 : 0);   // datagram_estop
    publishPacked(hub, ch::machine_modes, std::span<const std::byte>(buf).first(len), n);
}

// The two kinetic cards, each only when `cards` names it. Every setting on
// them is applied (0x3120), so every enabled_mask bit is high but
// chase_dense_ms's, which drops while a samples grant is live (latencyOpen
// false; applyTuning() refuses it then).
void publishKineticCards(Hub& hub, const MotionTuning& t, uint8_t cards, bool latencyOpen) {
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
    if (cards & kCardPlanner) {
        std::array<std::byte, 22> buf{};
        size_t n = 0;
        packF32(buf, n, t.smoothness);
        packF32(buf, n, t.handle_floor);
        packF32(buf, n, t.trim_max);
        packU32(buf, n, t.chase_dense_us);     // scale 1000, unit ms: the wire carries us
        packU32(buf, n, t.react_us);           // scale 1000, unit ms: the wire carries us
        // enabled_mask: smoothness, handle_floor, trim_max, chase_dense_ms
        // (bit 3), react_ms.
        packU8(buf, n, uint8_t(latencyOpen ? 0x1F : 0x17));
        packU8(buf, n, uint8_t(hub.trialMask(ch::kinetic_planner)));
        publishPacked(hub, ch::kinetic_planner, buf, n);
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

// SPEC 5.4 (RFC-090): a c2h bundle stamped past now plus its cap moves earlier
// as a whole, every spacing kept; stamps are never clamped one by one. The hub
// moved it already against its own clock; this holds it against now32.
BundleView leadCapped(const BundleView& bundle, uint32_t now32, uint32_t capUs, bool& moved) {
    BundleView b = bundle;
    moved = b.clampLead(now32, capUs);
    return b;
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
    if (channel_id == ch::osc_set) return applyOsc(requested, cfgChanged);
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
    const MotionCensus mo = motionCensus();
    const float rail = mo.rail_mm;
    StoredConfig next = _cfg;
    for (size_t k = 0; k < keys.size(); ++k)
        if (keys[k] != nullptr) setConfigKey(next, uint8_t(k + 1), *numberOf(keys[k]), rail, _modes.flipped);

    // While a real cycle's measurement stands (force_home measures nothing),
    // max_rail never reaches past the stop it found (SPEC 9.6 RFC-101, bd
    // val-3kd).
    const bool measured = mo.homed && mo.home_rail_mm > 0.0f;
    const float stroke = measured ? std::clamp(mo.home_rail_mm, ceiling::rail_min, ceiling::rail_mm) : ceiling::rail_mm;
    next.max_rail = std::min(next.max_rail, stroke);

    // The window is held inside max_rail, so inside the stop too (SPEC 9.6
    // RFC-101, bd val-8yv). A window write wholly past it has no legal nearest
    // value and is refused; any other write collapses such a window, logged.
    if (next.window_min >= next.max_rail && (f1 != nullptr || f2 != nullptr))
        return refuse(NackCode::INVALID_VALUE,
                      measured && next.window_min >= stroke ? kDetailPastStroke : "window past max_rail");
    const StoredConfig asked = next;
    if (holdWindowInRail(next))
        GLOGW(kTag, "window %.1f..%.1f mm lies past max_rail %.1f mm: held at %.1f..%.1f mm, set the window again",
              double(asked.window_min), double(asked.window_max), double(next.max_rail), double(next.window_min),
              double(next.window_max));

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

// The same door as a client's config-set key 8: its clamp, its ECHO-side copy,
// its first-run bit, and tick()'s republish, push and persist. 0x1000 goes out
// even when max_rail held its value, because measured_stroke moved.
void ValenceDevice::adoptMeasuredRail(float rail_mm) {
    IntentValueMap m{};
    m.count = 1;
    m.fields[0] = IntentValueField{8, IntentValue::ofF32(rail_mm)};
    bool changed = false;
    if (!durable(applyConfig(m, changed))) {
        GLOGW(kTag, "HOME: homed, rail measured %.1f mm, but max_rail refused it", double(rail_mm));
        return;
    }
    _cfgDirty = true;
    _cfgGenOwed |= changed;
    GLOGW(kTag, "HOME: homed. Usable rail %.1f mm between the safety margins, max_rail %.1f mm",
          double(rail_mm), double(_cfg.max_rail));
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
// Key 7 (chase_dense) is refused INTERLOCK while a samples grant is live: it
// sets that grant's schedule_latency_us, a commitment for the grant's life
// (SPEC 5.4, RFC-059), and the library has no publish re-GRANT to move it.

void ValenceDevice::noteTuning(const MotionTuning& next, bool& cfgChanged) {
    const uint8_t cards = cardsChanged(_tune, next);
    cfgChanged = cards != 0;
    _tuneDirty |= cards;
    _tune = next;
}

// Keys 7 (schedule_horizon), 8 (flipped), 9 (home_speed, clamped to its
// catalog bounds) and 10 (datagram_estop, RFC-053 item 3, pushed to the
// datagram switch at once); the whole request is validated before anything
// is applied. A horizon change is refused INTERLOCK while a segments grant is
// live: the grant advertised the old value for its life (SPEC 5.4), and the
// library re-reads this one on every bundle.
// Key 8 (flipped, RFC-088) is gated in the spec's order: SOURCE_CONFLICT while
// a source owns the rail, NOT_HOMED while unhomed, INTERLOCK under override or
// in motion. An unchanged value is an ordinary no-op ECHO.
Ret ValenceDevice::applyModes(const IntentValueMap& requested, bool& cfgChanged) {
    const auto* f7 = findField(requested, 7);   // schedule_horizon
    const auto* f8 = findField(requested, 8);   // flipped
    const auto* f9 = findField(requested, 9);   // home_speed
    const auto* f10 = findField(requested, 10);   // datagram_estop
    if (!f7 && !f8 && !f9 && !f10) return Ret::err(NackCode::INVALID_VALUE);
    if (f7 && !numberOf(f7)) return refuseNotANumber(ch::modes_set, 7);
    if (f8 && !boolOf(f8)) return refuseNotANumber(ch::modes_set, 8);
    if (f9 && !numberOf(f9)) return refuseNotANumber(ch::modes_set, 9);
    if (f10 && !boolOf(f10)) return refuseNotANumber(ch::modes_set, 10);
    StoredModes nextModes = _modes;
    if (f7) nextModes.horizon = uint8_t(wholeIn(*numberOf(f7), 0.0f, float(kHorizonMs.size() - 1)));
    if (nextModes.horizon != _modes.horizon && publishGrantLive(ch::motion_segment))
        return Ret::err(NackCode::INTERLOCK);
    if (f8) nextModes.flipped = *boolOf(f8);
    if (f10) nextModes.datagram_estop = *boolOf(f10);
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
    MotionTuning next = _tune;
    if (f7) applied.fields[n++] = {7, IntentValue::ofU64(nextModes.horizon)};
    if (f8) applied.fields[n++] = {8, IntentValue::ofU64(nextModes.flipped ? 1 : 0)};
    if (f10) applied.fields[n++] = {10, IntentValue::ofU64(nextModes.datagram_estop ? 1 : 0)};
    if (f9) {
        next.home_speed = clampf(*numberOf(f9), tuning_bounds::home_speed_min, tuning_bounds::home_speed_max);
        applied.fields[n++] = {9, IntentValue::ofF32(next.home_speed)};
    }
    if (f9) noteTuning(next, tuneChanged);
    const bool modesChanged = !(nextModes == _modes);
    // At rest by the gate above, so the frame moves under a still carriage.
    if (nextModes.flipped != _modes.flipped) {
        motionSetFlipped(nextModes.flipped);
        GLOGW(kTag, "FLIP %s: position 0 is the %s end", nextModes.flipped ? "on" : "off",
              nextModes.flipped ? "far" : "home");
    }
    if (nextModes.datagram_estop != _modes.datagram_estop) {
        estopDatagramSetEnabled(nextModes.datagram_estop);
        GLOGW(kTag, "ESTOP datagrams %s", nextModes.datagram_estop ? "latch" : "are ignored");
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
    // SPEC 9.7: a driven point lands no nearer than the oscillator's lead.
    if (channel_id == ch::osc_drive) return kOscDriveLeadUs;
    if (channel_id == ch::motion_input) return sampleLatencyUs(_tune);
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
// index-align with. The per-verb argument set is RFC-089's: slot required but
// on save, where its absence picks the lowest free slot; name required by
// save and rename; a field the verb does not use is ignored and not echoed.
// save captures LIVE state, or imports the store-item document in `item`,
// whose slot and name must equal the request's and whose kind, payload size
// and digest are checked, the payload never decoded. load applies through the
// same clamps an intent takes into the advanced generator's knobs, starting
// nothing, and its truth arrives on the ordinary pattern-plane STATE. The
// echoed name is the STORE's copy; an imported item echoes as sent.
Ret ValenceDevice::applyPresets(const IntentValueMap& requested) {
    const auto* opF = findField(requested, 1);
    const auto* slotF = findField(requested, 2);
    if (opF && !numberOf(opF)) return refuseNotANumber(ch::pattern_presets_cmd, 1);
    if (slotF && !numberOf(slotF)) return refuseNotANumber(ch::pattern_presets_cmd, 2);
    const auto op = numberOf(opF);
    if (!op) return Ret::err(NackCode::INVALID_VALUE);
    const uint32_t verb = wholeIn(*op, 0.0f, 255.0f);
    std::optional<uint8_t> slot;
    if (const auto slotV = numberOf(slotF)) {
        if (*slotV < 0.0f || *slotV >= float(PatternPresetStore::kCapacity)) return Ret::err(NackCode::INVALID_VALUE);
        slot = uint8_t(wholeIn(*slotV, 0.0f, float(PatternPresetStore::kCapacity - 1)));
    } else if (verb != store_ops::save) {
        return refuse(NackCode::INVALID_VALUE, "slot required");
    }
    const auto* nameF = findField(requested, 3);
    const bool hasName = nameF && nameF->value.kind == IntentValue::Kind::Tstr;
    const std::string_view name = hasName ? nameF->value.tstr_val : std::string_view{};
    if (!hasName && (verb == store_ops::save || verb == store_ops::rename))
        return refuse(NackCode::INVALID_VALUE, "name required");
    const auto* itemF = verb == store_ops::save ? findField(requested, 4) : nullptr;

    switch (verb) {
        case store_ops::save: {
            if (!slot) slot = _presets.freeSlot();
            if (!slot) return refuse(NackCode::INVALID_VALUE, "pattern presets full");
            PatternPresetStore::Payload payload = _pat.capturePreset();
            if (itemF) {
                if (itemF->value.kind != IntentValue::Kind::Bstr) return refuse(NackCode::INVALID_VALUE, "item");
                const auto doc = decodeStoreItem(itemF->value.bstr_val);
                if (!doc) return refuse(NackCode::INVALID_VALUE, "item malformed");
                if (doc.value().slot != *slot || doc.value().name != name)
                    return refuse(NackCode::INVALID_VALUE, "item slot or name differs");
                if (doc.value().kind != std::string_view(kPresetKind)) return refuse(NackCode::INVALID_VALUE, "item kind");
                if (doc.value().payload.size() != payload.size()) return refuse(NackCode::INVALID_VALUE, "item size");
                if (storeItemDoneStatus(doc.value()) != BlobDoneStatus::VerifiedComplete)
                    return refuse(NackCode::INVALID_VALUE, "item digest");
                std::memcpy(payload.data(), doc.value().payload.data(), payload.size());
            }
            if (!_presets.save(*slot, name, payload)) return Ret::err(NackCode::INVALID_VALUE);
            GLOGI(kTag, "preset %s: slot %u", itemF ? "imported" : "saved", unsigned(*slot));
            break;
        }
        case store_ops::load: {
            if (motionCensus().estop) return refuse(NackCode::ESTOP_ACTIVE, kDetailEstop);
            const PatternPresetStore::Slot* s = _presets.slot(*slot);
            if (s == nullptr) return Ret::err(NackCode::INVALID_VALUE);
            _pat.applyPreset(s->payload);
            _patDirty = true;
            GLOGI(kTag, "preset loaded: slot %u", unsigned(*slot));
            break;
        }
        case store_ops::delete_item:
            if (!_presets.remove(*slot)) return Ret::err(NackCode::INVALID_VALUE);
            GLOGI(kTag, "preset deleted: slot %u", unsigned(*slot));
            break;
        case store_ops::rename:
            if (!_presets.rename(*slot, name)) return Ret::err(NackCode::INVALID_VALUE);
            break;
        default:
            return Ret::err(NackCode::INVALID_VALUE);
    }
    // The item view lives in the request frame, which outlives the ECHO's
    // encode (Hub::handleIntent). An import's ECHO is past the idempotency
    // ring's slot, so it is sent but not kept for a duplicate id.
    IntentValueMap applied{};
    applied.fields[0] = {1, IntentValue::ofU64(verb)};
    applied.fields[1] = {2, IntentValue::ofU64(*slot)};
    applied.count = 2;
    if (verb == store_ops::save || verb == store_ops::rename)
        applied.fields[applied.count++] = {3, IntentValue::ofTstr(_presets.slot(*slot)->nameView())};
    if (itemF) applied.fields[applied.count++] = {4, itemF->value};
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

// ---- 0x3140 osc-set, the oscillator (RFC-103, SPEC 9.7) --------------------------
// Every key is a parameter, accepted in every machine state: under ESTOP, PAUSE
// or unhomed the oscillator renders nothing whatever it is set to
// (MotionArbiter::setOscillator()), so a write there moves nothing until the
// machine may move again. Each key is checked before any is applied; the echo
// is the clamped value; a change moves cfg_gen (SPEC 9.7, a hub-side clear
// does too) and is never persisted.
Ret ValenceDevice::applyOsc(const IntentValueMap& requested, bool& cfgChanged) {
    const auto* f1 = findField(requested, 1);  // enabled
    const auto* f2 = findField(requested, 2);  // frequency
    const auto* f3 = findField(requested, 3);  // amplitude
    const auto* f4 = findField(requested, 4);  // shape
    const auto* f5 = findField(requested, 5);  // dwell_crest
    const auto* f6 = findField(requested, 6);  // dwell_trough
    // Keys 7..16: the two drives and their bounds (kOscDriveCards).
    std::array<const IntentValueField*, 10> fd{};
    bool any = f1 || f2 || f3 || f4 || f5 || f6;
    for (uint8_t k = 0; k < fd.size(); ++k) {
        fd[k] = findField(requested, uint8_t(7 + k));
        any = any || fd[k] != nullptr;
    }
    if (!any) return Ret::err(NackCode::INVALID_VALUE);
    if (f1 && !boolOf(f1)) return refuseNotANumber(ch::osc_set, 1);
    for (const auto* f : {f2, f3, f4, f5, f6})
        if (f && !numberOf(f)) return refuseNotANumber(ch::osc_set, f->key);
    for (const auto* f : fd)
        if (f && !numberOf(f)) return refuseNotANumber(ch::osc_set, f->key);
    // Two decimals (SPEC 9.7).
    auto dwell = [](float v) { return std::round(clampf(v, 0.0f, ceiling::osc_dwell_max) * 100.0f) / 100.0f; };
    const MotionOsc was = _osc;
    const uint32_t wasSession = _oscSession;
    MotionOsc& o = _osc;
    if (f1) {
        o.enabled = *boolOf(f1);
        _oscSession = _hub != nullptr ? _hub->intentSession() : 0;
    }
    if (f2) o.frequency_hz = clampf(*numberOf(f2), 0.0f, ceiling::osc_max_hz);
    if (f3) o.amplitude = clampf(*numberOf(f3), 0.0f, 1.0f);
    if (f4) o.shape = uint8_t(wholeIn(*numberOf(f4), 0.0f, 3.0f));
    if (f5) o.dwell_crest = dwell(*numberOf(f5));
    if (f6) o.dwell_trough = dwell(*numberOf(f6));
    // A drive's select, its input bounds as given, its output bounds clamped
    // into the parameter's range; equal input bounds have no map (SPEC 8.11).
    MotionOscDrive* drives[2] = {&o.frequency_drive, &o.amplitude_drive};
    const float tops[2] = {ceiling::osc_max_hz, 1.0f};
    for (int d = 0; d < 2; ++d) {
        const auto** f = &fd[size_t(5 * d)];
        MotionOscDrive& m = *drives[d];
        if (f[0]) m.drive = uint8_t(wholeIn(*numberOf(f[0]), 0.0f, 3.0f));
        if (f[1]) m.in_min = *numberOf(f[1]);
        if (f[2]) m.in_max = *numberOf(f[2]);
        if (f[3]) m.out_min = clampf(*numberOf(f[3]), 0.0f, tops[d]);
        if (f[4]) m.out_max = clampf(*numberOf(f[4]), 0.0f, tops[d]);
        if (!(m.in_min != m.in_max)) {
            o = was;
            _oscSession = wasSession;
            return refuse(NackCode::INVALID_VALUE, "drive in_min equals in_max");
        }
    }
    const bool driven = oscDriven(motionCensus().osc_stream_live);
    if (!(o == was)) {
        motionSetOscillator(o);
        _oscDirty = true;
        cfgChanged = true;
        _oscDrivenShown = driven;   // this write's cfg_gen covers what it reports
    }
    IntentValueMap applied{};
    uint32_t n = 0;
    if (f1) applied.fields[n++] = {1, IntentValue::ofBool(o.enabled)};
    if (f2) applied.fields[n++] = {2, IntentValue::ofF32(o.frequency_hz)};
    if (f3) applied.fields[n++] = {3, IntentValue::ofF32(o.amplitude)};
    if (f4) applied.fields[n++] = {4, IntentValue::ofU64(driven ? 0 : o.shape)};
    if (f5) applied.fields[n++] = {5, IntentValue::ofF32(driven ? 0.0f : o.dwell_crest)};
    if (f6) applied.fields[n++] = {6, IntentValue::ofF32(driven ? 0.0f : o.dwell_trough)};
    for (int d = 0; d < 2; ++d) {
        const MotionOscDrive& m = *drives[d];
        const uint8_t k0 = uint8_t(7 + 5 * d);
        if (fd[size_t(5 * d)]) applied.fields[n++] = {k0, IntentValue::ofU64(m.drive)};
        const float v[4] = {m.in_min, m.in_max, m.out_min, m.out_max};
        for (uint8_t i = 0; i < 4; ++i)
            if (fd[size_t(5 * d + 1 + i)]) applied.fields[n++] = {uint8_t(k0 + 1 + i), IntentValue::ofF32(v[i])};
    }
    applied.count = n;
    return Ret::ok(applied);
}

// Driven by a live input (a speed or position drive, or an axis drive with a
// live osc-drive stream) the oscillator is a sine with no dwells, Kinetic's
// driven mode; an axis drive with no live stream is fixed (bd val-o9r,
// Valence RFC-110 item 3, ahead of the pinned SPEC 9.7).
bool ValenceDevice::oscDriven(bool streamLive) const {
    auto live = [&](uint8_t d) {
        return d == osc_drives::speed || d == osc_drives::position || (d == osc_drives::axis && streamLive);
    };
    return live(_osc.frequency_drive.drive) || live(_osc.amplitude_drive.drive);
}

// The parameters as applied, shape and dwells as they play (the written ones
// kept), then what renders: osc.active and osc.amplitude_effective from the
// census.
// Every anomaly is taken, so the handoff never fills behind the bucket; one
// the bucket has no token for is dropped here. The kind rides event_kind and
// is mirrored into body key 1 (SPEC 9.4); t_us is the low 32 bits (anom_body).
void ValenceDevice::publishAnomalies(uint32_t nowMs) {
    const uint32_t refill = uint32_t(nowMs - _anomRefillMs) / kAnomalyEventMs;
    if (refill > 0) {
        _anomTokens = std::min(kAnomalyEventBurst, _anomTokens + refill);
        _anomRefillMs += refill * kAnomalyEventMs;
    }
    MotionAnomaly a;
    while (motionTakeAnomaly(a)) {
        if (_anomTokens == 0) continue;
        --_anomTokens;
        EventMsg ev{};
        ev.channel_id = ch::motion_anomaly;
        ev.timestamp = nowMs;
        ev.event_kind = a.kind;
        ev.has_body = true;
        ev.body_count = 5;
        ev.body[0] = IntentValueField{anom_body::kind, IntentValue::ofU64(a.kind)};
        ev.body[1] = IntentValueField{anom_body::seq, IntentValue::ofU64(a.seq)};
        ev.body[2] = IntentValueField{anom_body::target, IntentValue::ofF32(a.target)};
        ev.body[3] = IntentValueField{anom_body::detail, IntentValue::ofF32(a.detail)};
        ev.body[4] = IntentValueField{anom_body::t_us, IntentValue::ofU64(uint32_t(a.t_us))};
        std::array<std::byte, 96> buf{};
        const size_t n = encodeEvent(ev, std::span<std::byte>(buf));
        if (n > 0) _hub->publishEvent(ch::motion_anomaly, std::span<const std::byte>(buf.data(), n));
    }
}

void ValenceDevice::publishOscillator(const MotionCensus& mo, bool force) {
    const bool driven = oscDriven(mo.osc_stream_live);
    // SPEC 4.2: the reported shape or dwells moving with a stream's liveness
    // is a hub-side change; a kept sine with no dwells reports no change.
    if (driven != _oscDrivenShown) {
        _oscDrivenShown = driven;
        if (_osc.shape != 0 || _osc.dwell_crest != 0.0f || _osc.dwell_trough != 0.0f) _oscHubChange = true;
    }
    std::array<std::byte, 57> buf{};
    size_t n = 0;
    packU8(buf, n, _osc.enabled ? 1 : 0);
    packF32(buf, n, _osc.frequency_hz);
    packF32(buf, n, _osc.amplitude);
    packU8(buf, n, driven ? 0 : _osc.shape);
    packF32(buf, n, driven ? 0.0f : _osc.dwell_crest);
    packF32(buf, n, driven ? 0.0f : _osc.dwell_trough);
    packU8(buf, n, mo.osc_active ? 1 : 0);
    packF32(buf, n, mo.osc_amplitude);
    for (const MotionOscDrive* d : {&_osc.frequency_drive, &_osc.amplitude_drive}) {
        packU8(buf, n, d->drive);
        packF32(buf, n, d->in_min);
        packF32(buf, n, d->in_max);
        packF32(buf, n, d->out_min);
        packF32(buf, n, d->out_max);
    }
    publishIfChanged(*_hub, ch::oscillator, buf, n, _sentOsc, force);
    // RFC-011: a hub-side change bumps after its STATE is out.
    if (_oscHubChange) {
        _oscHubChange = false;
        _hub->bumpConfigGeneration();
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
        case 1: {  // home: a real cycle against the home sense (MotionArbiter.h, homing)
            // The ECHO answers the start. The outcome is the 0x1100 homing
            // and homed bits and the motion log; a failed cycle cannot NACK
            // an intent already answered (RFC-101 drafts the deferred answer).
            const MotionCensus c = motionCensus();
            if (c.estop) return refuse(NackCode::ESTOP_ACTIVE, kDetailEstop);
            // Under PAUSE an owning source is suspended (SPEC 11.1 admits the
            // home verb there); outside it, an owner is driving the rail.
            if (railOwned() && !c.paused) return refuse(NackCode::SOURCE_CONFLICT, "a source owns the rail");
            const bool overrideOn = _hub != nullptr &&
                                    (_hub->safetyModes() & safety_mode_bits::OVERRIDE) != 0;
            if (overrideOn || _returnPending) return refuse(NackCode::INTERLOCK, "override latched: return first");
            if (c.busy && !c.homing) return refuse(NackCode::INTERLOCK, "moving: pause first");
            // Operator ruling 2026-10-09 (Valence rfc-ns5c item 3): a home
            // cycle is the one thing an oscillating machine refuses.
            // A live stream renders it while an axis drive is bound, osc.enabled
            // or not (RFC-110 item 2).
            const bool streamed = c.osc_stream_live && (_osc.frequency_drive.drive == osc_drives::axis ||
                                                        _osc.amplitude_drive.drive == osc_drives::axis);
            if (_osc.enabled || streamed || c.osc_active)
                return refuse(NackCode::INTERLOCK, "oscillating: disable the oscillator first");
            switch (motionHome()) {
                case HomeStart::started:
                    break;
                case HomeStart::no_sense:
                    GLOGW_EVERY_MS(60000, kTag, "home op 1 refused: no home sense on this board "
                                   "-- force_home (op 2) on a motorless rig");
                    return refuse(NackCode::UNSUPPORTED_OP, "no home sense on this board");
                case HomeStart::undriven:
                    return refuse(NackCode::INTERLOCK, "home sense not driven: sensor unwired or down");
                case HomeStart::sense_high:
                    return refuse(NackCode::INTERLOCK, "home sense already reads a stall");
                case HomeStart::estop:
                    return refuse(NackCode::ESTOP_ACTIVE, kDetailEstop);
                case HomeStart::unpowered:
                    return refuseUnpowered("home");
            }
            applied.count = 1;
            applied.fields[0] = {1, IntentValue::ofU64(1)};
            return Ret::ok(applied);
        }

        case 2: {  // force_home {stroke}
            // *** HAZARD, RFC-025. The hazard note lives on motionForceHome()
            // in ValenceMotion.h; do not restate it (C-1). A completed home
            // clears home_required, and on this bench op it also releases a
            // held ESTOP latch into PAUSE. Both are tick()'s, on the next hub
            // tick: releaseEstop() and setHomeRequired() publish and
            // broadcast, and this runs inside the hub's own intent dispatch.
            if (motionCensus().homing) return refuse(NackCode::INTERLOCK, "homing");
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

// ---- 0x2140 osc-drive ingress ---------------------------------------------------
// SPEC 9.7: each sample is {amplitude f32, frequency f32}, each 0 .. 1, timed
// as any samples-kind bundle (resolved, and moved earlier whole past the lead
// cap, as 0x2100's) and handed to the motion task at once: the driven
// oscillator places it at its stamp, no nearer than kOscDriveLeadUs ahead, the
// grant's schedule latency. A stamp already past plays at once.
void ValenceDevice::takeOscDrive(const BundleView& bundle) {
    const int64_t now64 = int64_t(deviceNowUs());
    const uint32_t now32 = uint32_t(uint64_t(now64) & 0xFFFFFFFFull);
    bool moved = false;
    const BundleView b = leadCapped(bundle, now32, limits::max_future_schedule_ms * 1000u, moved);
    auto unit = [](float v) { return !(v > 0.0f) ? 0.0f : std::fmin(v, 1.0f); };
    for (uint8_t i = 0; i < b.sampleCount(); ++i) {
        const int32_t delta = std::max<int32_t>(int32_t(b.sampleTimeUs(i) - now32), 0);
        const auto sample = b.sample(i);
        if (sample.size() < 8) continue;
        motionOscDrive(unit(getF32(sample.subspan(0, 4))), unit(getF32(sample.subspan(4, 4))),
                       uint64_t(now64 + delta));
    }
}

// ---- 0x2100 / 0x2101 stream ingress --------------------------------------------
// Runs on the hub task, synchronously inside Hub::update(). The hub has
// already validated the §5.4 caps, the granted rate, ownership and the
// deadman (hub.hpp's contract on this method); this decodes and submits and
// re-checks none of it. Decoding is BY FIXED OFFSET against the catalog's
// own 0x2100 (4 B point) / 0x2101 (6 B timed segment) field order -- the
// same convention the publishers above encode with.
void ValenceDevice::onStreamBundle(uint16_t channel_id, uint32_t /*session_id*/,
                                   const BundleView& bundle) {
    if (channel_id == ch::osc_drive) {
        takeOscDrive(bundle);
        return;
    }
    const bool isSegment = (channel_id == ch::motion_segment);
    if (channel_id != ch::motion_input && !isSegment) return;

    // SPEC 11.1 PAUSE: the hub drops a source-mapped bundle whole while PAUSE
    // is latched and never calls this. A sample queued before the latch is
    // refused at accept() by the arbiter's own pause gate.
    const uint8_t n = bundle.sampleCount();

    // t_base/t_off are u32 HUB-us, the same wrapping domain the hub clock reads
    // (§7.2); BundleView already scaled a segments t_off from its 100 us unit.
    // now64 stays the FULL 64-bit reading so the anchor never wraps itself;
    // only the WIRE stamp being resolved against it does.
    const int64_t now64 = int64_t(deviceNowUs());
    const uint32_t now32 = uint32_t(uint64_t(now64) & 0xFFFFFFFFull);
    // RFC-084 lead cap, one per kind: a bundle stamped further ahead than its
    // cap moves earlier whole (leadCapped()), never dropped. Segments ride the
    // grant's horizon (RFC-087); samples the registry's max_future_schedule_ms.
    const uint32_t leadCapUs = uint32_t(isSegment ? scheduleHorizonMs(channel_id)
                                                  : limits::max_future_schedule_ms) * 1000u;
    bool farMoved = false;
    const BundleView b = leadCapped(bundle, now32, leadCapUs, farMoved);
    // A segment's expectation (MotionIntent::expect_us) is the hub's quiet
    // window for this stream, the same max the library releases it on.
    const uint32_t expectUs =
        std::max<uint32_t>(limits::stream_quiet_release_ms, scheduleHorizonMs(channel_id)) * 1000u;

    // Normalized samples map onto the client-frame window (RFC-088); the
    // arbiter mirrors the result to the physical rail.
    const Window w = clientWindow(motionCensus().rail_mm);
    const float span = w.hi - w.lo;
    uint32_t dropped = 0;
    // Samples of either kind whose time had passed at ingress, the count
    // Valence RFC-109 drafts as stream.late; worstUs is the bundle's arrival
    // lead when it is negative.
    uint32_t late = 0;
    int32_t worstUs = 0;
    // RFC-087 supersede: the bundle's first segment the motion path takes
    // flushes what is queued from its start (MotionArbiter, the Kinetic²
    // boundary). A samples bundle never flushes.
    bool flushed = !isSegment;
    for (uint8_t i = 0; i < n; ++i) {
        // Nearest-window resolve (§7.2): a wrap-aware signed subtract, safe
        // because the wire stamp is near now by construction (the bundle
        // span is capped far under the 32-bit wrap).
        int32_t delta = int32_t(b.sampleTimeUs(i) - now32);
        if (delta < 0) {
            ++late;
            worstUs = std::min(worstUs, delta);
            delta = 0;
        }

        // The field mapping is StreamIntent.h's, shared with the offline
        // planner; a zero-duration segment decodes to nullopt and is dropped.
        const auto sample = b.sample(i);
        const uint16_t pos = getU16(sample.subspan(0, 2));
        const uint64_t anchor = uint64_t(now64 + int64_t(delta));
        std::optional<MotionIntent> in =
            isSegment ? segmentIntent(pos, getU16(sample.subspan(2, 2)),
                                      int16_t(getU16(sample.subspan(4, 2))), w.lo, span, anchor)
                      : pointIntent(pos, int16_t(getU16(sample.subspan(2, 2))), w.lo, span, anchor);
        if (in && !flushed) in->supersede = true;
        if (in && isSegment) in->expect_us = expectUs;
        if (!in || !motionSubmit(*in)) {
            ++dropped;
            continue;
        }
        flushed = true;
    }

    motionNoteStream(1, n, dropped);
    if (isSegment) ++_segBundles;
    if (late) {
        _lateSamples.store(_lateSamples.load(std::memory_order_relaxed) + late, std::memory_order_relaxed);
        GLOGI_EVERY_MS(1000, kTag, "motion stream: %u of %u sample(s) arrived past their time, by up to %.1f ms",
                       unsigned(late), unsigned(n), double(-worstUs) * 1e-3);
    }
    if (farMoved) {
        GLOGW_EVERY_MS(2000, kTag,
                       "motion stream: a bundle of %u sample(s) stamped past the lead cap moved earlier whole "
                       "(missed CLOCK resync on the client?)", unsigned(n));
    }
}

void ValenceDevice::onSessionJoined(uint32_t session_id) {
    GLOGI(kTag, "session %lu joined", static_cast<unsigned long>(session_id));
    (void)session_id;
}

void ValenceDevice::onSessionLeft(uint32_t session_id) {
    GLOGI(kTag, "session %lu left", static_cast<unsigned long>(session_id));
    oscSessionEnded(session_id, "left");
}

void ValenceDevice::onSessionStale(uint32_t session_id) { oscSessionEnded(session_id, "went stale"); }

// SPEC 9.7: no oscillation outlives the hand that enabled it. A writer the hub
// did not name (0) is cleared by any session's end, never late.
void ValenceDevice::oscSessionEnded(uint32_t session_id, const char* how) {
    if (!_osc.enabled || (_oscSession != 0 && _oscSession != session_id)) return;
    _osc.enabled = false;
    motionSetOscillator(_osc);
    _oscDirty = true;
    _oscHubChange = true;
    GLOGI(kTag, "oscillator off: session %lu %s", static_cast<unsigned long>(session_id), how);
    (void)how;
}

float ValenceDevice::oscMaxHz() { return OSC_MAX_HZ; }

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
    // The stream's release ends its expectation (MotionArbiter::releaseRail).
    if (gen == MotionSource::Stream) motionReleaseRail(gen);
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
    const MotionCensus mo = motionCensus();
    const Window w = clientWindow(mo.rail_mm);
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
    // measured_stroke: 0 means NOT MEASURED. Only a completed home cycle
    // measures (census.home_rail_mm); force_home's stroke is an assertion and
    // never lands here.
    putF32(s.subspan(33, 4), mo.home_rail_mm);
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
        stored::decodeConfig(blob, motionDefaultTuning(), _cfg, _tune, _modes, cfgGen);
    if (!adopted) return adopted;
    _cfgDirty = false;
    // A blob written before the window was held inside max_rail (bd val-8yv).
    if (holdWindowInRail(_cfg))
        GLOGW(kTag, "stored window lies past max_rail %.1f mm: held at %.1f..%.1f mm, set the window again",
              double(_cfg.max_rail), double(_cfg.window_min), double(_cfg.window_max));
    // The engine adopts through the live write's own door, never a side path.
    motionSetTuning(_tune);
    estopDatagramSetEnabled(_modes.datagram_estop);
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
    _anomTokens = kAnomalyEventBurst;
    // The declaration's one home is the composition's setEstopCutsPower().
    motionSetEstopCutsPower(hub.estopCutsPower());
    hub.publishControlOwnerStateIfPresent();
    publishMachineConfig();
    publishHubStatus();
    // _tune is the factory set or the adopted one, and the engine already holds
    // it either way (adoptConfigBlob pushed it), so nothing is pushed here.
    publishMachineModes(hub, _tune, _modes, !publishGrantLive(ch::motion_segment), flipOpen(motionCensus()));
    publishKineticCards(hub, _tune, kCardLimits | kCardPlanner, !publishGrantLive(ch::motion_input));
    const MotionCensus mo = motionCensus();
    publishMotion(hub, mo, patternActive());
    publishPlanStrip(hub, mo);
    publishMotionDiag(hub, mo, _ingressDrops.total(), _segBundles);
    publishOdometer(hub, mo);
    motionSetOscillator(_osc);
    publishOscillator(mo, true);
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
    else GLOGW(kTag, "HOME press: home refused, NACK 0x%04x: %s", unsigned(r.error()), _nackDetail.data());
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

    // The schedule_horizon, flipped and chase_dense_ms enabled_mask bits
    // follow the publish grants and the rail's state, which no write announces.
    const bool horizonOpen = !publishGrantLive(ch::motion_segment);
    const bool flipNowOpen = flipOpen(mo);
    if (horizonOpen != _horizonOpenSent || flipNowOpen != _flipOpenSent) {
        _horizonOpenSent = horizonOpen;
        _flipOpenSent = flipNowOpen;
        publishMachineModes(*_hub, _tune, _modes, horizonOpen, flipNowOpen);
    }
    const bool latencyOpen = !publishGrantLive(ch::motion_input);
    if (latencyOpen != _latencyOpenSent) {
        _latencyOpenSent = latencyOpen;
        publishKineticCards(*_hub, _tune, kCardPlanner, latencyOpen);
    }

    // RETURN arrived (SPEC 11.1): override drops, plain PAUSE stays. A counter,
    // not the census flag, because the census lags the request by up to a
    // publish interval and would read "no override" before it ever started.
    if (_returnPending && mo.returns != _returnsAtRequest) {
        _returnPending = false;
        _hub->setOverride(false);
    }
    // A completed home cycle clears home_required (SPEC 11.2). A counter for
    // the same reason as the return's; an ESTOP since it left the machine
    // unhomed, and that latch's home_required stands. Its measured rail
    // becomes max_rail.
    if (mo.homes != _homesSeen) {
        _homesSeen = mo.homes;
        if (mo.homed && !mo.estop) _hub->setHomeRequired(false);
        if (mo.home_rail_mm > 0.0f) adoptMeasuredRail(mo.home_rail_mm);
    }
    // The log channel (Warn, on this task: system/ValenceLogBridge) carries
    // why a cycle ended unhomed; the arbiter's own line never reaches it.
    if (mo.home_fails != _homeFailsSeen) {
        _homeFailsSeen = mo.home_fails;
        GLOGW(kTag, "HOME failed at the %s: %s%s", mo.home_fail_leg == 0 ? "home end" : "far end",
              mo.home_fail_why != nullptr ? mo.home_fail_why : "no reason recorded",
              mo.homed ? "" : ", unhomed");
    }
    if (uint32_t(nowMs - _lastMotionMs) >= 33u) {
        _lastMotionMs = nowMs;
        publishMotion(*_hub, mo, patternActive());
    }
    if (uint32_t(nowMs - _lastPlanMs) >= 50u) {
        _lastPlanMs = nowMs;
        publishPlanStrip(*_hub, mo);
        publishOscillator(mo, false);   // what renders moves with the plan
    }
    if (_oscDirty) {
        _oscDirty = false;
        publishOscillator(mo, false);
    }
    publishAnomalies(nowMs);
    if (uint32_t(nowMs - _lastSlowMs) >= 1000u) {
        _lastSlowMs = nowMs;
        publishMotionDiag(*_hub, mo, _ingressDrops.total(), _segBundles);
        publishOdometer(*_hub, mo);
    }

    if (_tuneDirty != 0) {
        motionSetTuning(_tune);
        if (_tuneDirty & kCardModes)
            publishMachineModes(*_hub, _tune, _modes, _horizonOpenSent, _flipOpenSent);
        publishKineticCards(*_hub, _tune, _tuneDirty, _latencyOpenSent);
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
        // RFC-011: a hub-side change bumps after its STATE is out. A client
        // write's bump is the hub library's own, from cfgChanged.
        if (_cfgGenOwed) {
            _cfgGenOwed = false;
            _hub->bumpConfigGeneration();
        }
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
        publishKineticCards(*_hub, _tune, kCardLimits | kCardPlanner, _latencyOpenSent);
    }

    // 1 Hz, and at once when the motor switch moved: a client watching a
    // release sees pre-charging, then on, without waiting out the second.
    const MotorSwitchStatus msw = motorSwitchStatus();
    if (uint32_t(nowMs - _lastStatusMs) >= 1000u || msw.state != _mswSent.state ||
        msw.last_fault != _mswSent.last_fault) {
        _lastStatusMs = nowMs;
        publishHubStatus();
    }

    const bool cfgDue = _persistArmed && int32_t(nowMs - _persistDueMs) >= 0;
    const bool presetsDue = _presetsArmed && int32_t(nowMs - _presetsDueMs) >= 0;
    if (!(cfgDue || presetsDue)) return 0;
    if (!mo.estop && (mo.homing || !motionStillFor(kPersistStillUs))) return 0;
    uint8_t due = 0;
    if (cfgDue) {
        _persistArmed = false;
        due |= kPersistConfig;
    }
    if (presetsDue) {
        _presetsArmed = false;
        due |= kPersistPresets;
    }
    return due;
}

}  // namespace valence
