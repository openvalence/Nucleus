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
//   NVS page; the arithmetic lives on ValenceHub.cpp's file header.
// See: Valence SPEC.md §4.2, §6.3, §9.1, §9.3, §11.2, §11.4

#include "ValenceDevice.h"

#include <cmath>

#include "geiger/geiger.h"
#include "motion/ValenceMotion.h"

#include "valence/util/byte_io.hpp"

namespace valence {

namespace {

using Ret = Result<IntentValueMap, NackCode>;

constexpr const char* kTag = "hub";

// Quiet time after the last applied change before the persist is due.
constexpr uint32_t kCfgPersistDebounceMs = 2000;

// 0x2101 field 3's "no end velocity" sentinel. 0 is a legitimate slope, so it
// cannot mean absent; INT16_MIN is the value the catalog reserves.
constexpr int16_t kSegNoEndVel = -32768;

// How far ahead of now a stream sample's t_off may resolve before it is treated
// as a client clock that lost sync. Past this the sample is pulled back rather
// than parked: a quarter second of runway is already far more than any bundle
// span, so a larger lead is a resync failure, not a schedule.
constexpr int32_t kStreamFarFutureUs = 250000;

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

float fieldF32(const IntentValueField* f, float dflt) {
    if (!f) return dflt;
    switch (f->value.kind) {
        case IntentValue::Kind::F32:  return f->value.f32_val;
        case IntentValue::Kind::U64:  return float(f->value.u64_val);
        case IntentValue::Kind::I64:  return float(f->value.i64_val);
        default:                      return dflt;
    }
}

uint64_t fieldU64(const IntentValueField* f, uint64_t dflt) {
    if (!f) return dflt;
    switch (f->value.kind) {
        case IntentValue::Kind::U64: return f->value.u64_val;
        case IntentValue::Kind::I64: return f->value.i64_val < 0 ? dflt : uint64_t(f->value.i64_val);
        default:                     return dflt;
    }
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

void publishControlOwner(Hub& hub) {
    // 4 x {source u8, owner u32} ascending, exactly Hub::buildControlOwnerPayload.
    // Every source unowned at boot, which is the truth. The hub republishes
    // this channel itself on every ownership transition; this call is the SEED
    // that keeps a subscriber from holding "no idea" before the first one.
    std::array<std::byte, 20> buf{};
    std::span<std::byte> s(buf);
    for (uint8_t i = 0; i < 4; ++i) {
        putU8(s.subspan(size_t(i) * 5, 1), i);
        putU32(s.subspan(size_t(i) * 5 + 1, 4), 0);
    }
    hub.publishState(channels::control_owner, s);
}

// ---- the motion plane's retained STATE ---------------------------------------
// Every channel below reads ONE motionCensus(), so no two of them can disagree
// about the same instant. The byte layouts mirror ValenceCatalog.h's field
// order for each id exactly; that mirroring is the whole contract (there is no
// packed struct to static_assert against).

void publishMotion(Hub& hub, const MotionCensus& m) {
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
    publishPacked(hub, ch::motion, buf, n);
}

void publishPlanStrip(Hub& hub, const MotionCensus& m) {
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
    publishPacked(hub, ch::plan_strip, buf, n);
}

void publishMotionDiag(Hub& hub, const MotionCensus& m) {
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

// Published ONCE at attach(): nothing on this board changes any of it, and
// 0x3030 NACKs every write, so a republish would carry no news.
void publishMachineModes(Hub& hub) {
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
    publishPacked(hub, ch::machine_modes, buf, n);
}

// The three kinetic cards. Read-only on this board (0x3120 NACKs), so their
// enabled_mask is 0 and they publish once at attach().
void publishKineticCards(Hub& hub) {
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
        publishPacked(hub, ch::kinetic_limits, buf, n);
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
        publishPacked(hub, ch::kinetic_chase, buf, n);
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
        publishPacked(hub, ch::kinetic_waveform, buf, n);
    }
}

}  // namespace

// ---- HubDelegate ---------------------------------------------------------------

// §12.2. A HELLO nothing vouches for is a WORKING STATE, not a failure: at
// WATCH it can subscribe to everything and use the role-exempt stop and estop
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

// ---- 0x3100 move ---------------------------------------------------------------
// MANUAL source: the wire operator is the one driving. The arbiter lets a
// Manual intent through an unhomed machine (the push-to-home case a local
// button would use); this board has no such button, so the WIRE door is
// gated on homed here. force_home is that door (operator ruling
// 2026-09-21), and a refusal carries its reason rather than a count.
Ret ValenceDevice::applyMove(const IntentValueMap& requested) {
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

// ---- 0x3101 home ---------------------------------------------------------------
Ret ValenceDevice::applyHome(const IntentValueMap& requested) {
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
            // call, and the hub's own ESTOP bit is dropped by tick() on the
            // next hub tick, because clearEstop() broadcasts and this runs
            // inside the hub's own intent dispatch.
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
std::optional<uint8_t> ValenceDevice::sourceForChannel(uint16_t channel_id) {
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
bool ValenceDevice::canClearEstop() {
    const MotionCensus c = motionCensus();
    if (c.busy || c.step_q8 != 0) return false;
    motionEstopClear();
    return true;
}

// §11.2: motion stops before protocol bookkeeping. The emitter is parked on
// THIS task inside motionEstop(), before this returns, so the stop precedes
// the latch as the spec requires.
void ValenceDevice::onEstop(uint8_t cause, uint8_t origin) {
    motionEstop();
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

    // RFC-030: the session's GRANTED (post-curve-policy) family, looked up
    // once per bundle. Chase points never carry one -- the family is a
    // waveform-reconstruction concept.
    const uint8_t curveFamily =
        (isSegment && _hub != nullptr) ? _hub->publishCurveFamily(session_id, channel_id) : 0;

    // t_base/t_off are u32 HUB-us, the same wrapping domain the hub clock reads
    // (§7.2). now64 stays the FULL 64-bit reading so the anchor never wraps
    // itself; only the WIRE stamp being resolved against it does.
    const int64_t now64 = int64_t(deviceNowUs());
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
        const float norm = float(getU16(sample.subspan(0, 2))) / 10000.0f;

        MotionIntent in;
        in.source    = MotionSource::Stream;
        in.target_mm = _cfg.window_min + norm * (_cfg.window_max - _cfg.window_min);
        in.anchor_us = uint64_t(now64 + int64_t(delta));

        if (isSegment) {
            const uint16_t durMs = getU16(sample.subspan(2, 2));
            const int16_t endV = int16_t(getU16(sample.subspan(4, 2)));
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
            const int16_t vel = int16_t(getU16(sample.subspan(2, 2)));
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

void ValenceDevice::onSessionJoined(uint32_t session_id) {
    GLOGI(kTag, "session %lu joined", static_cast<unsigned long>(session_id));
    (void)session_id;
}

void ValenceDevice::onSessionLeft(uint32_t session_id) {
    GLOGI(kTag, "session %lu left", static_cast<unsigned long>(session_id));
    (void)session_id;
}

// ---- retained STATE --------------------------------------------------------------

void ValenceDevice::publishHubStatus() {
    // 4+4+1+1+4 = 14 B, matching the 0x0007 layout in ValenceCatalog.h.
    const int8_t rssi = _linkRssi.load(std::memory_order_relaxed);
    std::array<std::byte, 14> buf{};
    std::span<std::byte> s(buf);
    putU32(s.subspan(0, 4), deviceFreeHeapBytes());
    putU32(s.subspan(4, 4), uint32_t(deviceNowUs() / 1000000));
    putU8(s.subspan(8, 1), uint8_t(rssi));
    putU8(s.subspan(9, 1), uint8_t(_hub->sessionCount()));
    putU32(s.subspan(10, 4), _hub->logDropped());
    _hub->publishState(channels::hub_status, s);
}

void ValenceDevice::publishMachineConfig() {
    // 37 B, matching the 0x1000 layout in ValenceCatalog.h.
    const StoredConfig& c = _cfg;
    std::array<std::byte, 37> buf{};
    std::span<std::byte> s(buf);
    putF32(s.subspan(0, 4), c.window_min);
    putF32(s.subspan(4, 4), c.window_max);
    putF32(s.subspan(8, 4), c.user_speed);
    putF32(s.subspan(12, 4), c.user_accel);
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
    _hub->publishState(ch::machine_config, s);
    _lastPublishedCfg = c;
    _cfgEverSent = true;
}

void ValenceDevice::pushConfigToMotion() const {
    motionSetWindow(_cfg.window_min, _cfg.window_max, _cfg.max_rail);
    motionSetUserLimits(_cfg.user_speed, _cfg.user_accel);
    motionSetInputLimits(_cfg.input_speed, _cfg.input_accel, _cfg.input_jerk);
}

void ValenceDevice::attach(Hub& hub) {
    _hub = &hub;
    publishControlOwner(hub);
    publishMachineConfig();
    publishHubStatus();
    publishMachineModes(hub);
    publishKineticCards(hub);
    const MotionCensus mo = motionCensus();
    publishMotion(hub, mo);
    publishPlanStrip(hub, mo);
    publishMotionDiag(hub, mo);
    publishOdometer(hub, mo);
}

bool ValenceDevice::tick(uint32_t nowMs) {
    // force_home dropped the arbiter's latch inside applyIntent; the hub's own
    // ESTOP bit drops HERE, one tick later, because clearEstop() publishes and
    // broadcasts and applyIntent runs inside the hub's intent dispatch.
    // canClearEstop() still gates it, so the two never disagree.
    if (_clearLatch) {
        _clearLatch = false;
        if (_hub->estopLatched()) {
            if (_hub->clearEstop()) GLOGW(kTag, "ESTOP latch cleared by force_home");
            else GLOGW(kTag, "force_home could not clear the ESTOP latch: motion is not parked");
        }
    }

    // The motion plane, from ONE census so no two channels disagree about the
    // same instant. 0x1100 publishes at rate under its 60 Hz ceiling; 0x1110 is
    // a strip that is only news while a plan runs.
    const MotionCensus mo = motionCensus();
    if (uint32_t(nowMs - _lastMotionMs) >= 33u) {
        _lastMotionMs = nowMs;
        publishMotion(*_hub, mo);
    }
    if (uint32_t(nowMs - _lastPlanMs) >= 50u) {
        _lastPlanMs = nowMs;
        publishPlanStrip(*_hub, mo);
    }
    if (uint32_t(nowMs - _lastSlowMs) >= 1000u) {
        _lastSlowMs = nowMs;
        publishMotionDiag(*_hub, mo);
        publishOdometer(*_hub, mo);
    }

    const bool dirty = _cfgDirty;
    _cfgDirty = false;
    if (dirty || (_cfgEverSent && !(_lastPublishedCfg == _cfg))) {
        pushConfigToMotion();
        publishMachineConfig();
        // Re-armed, not accumulated: the write lands only after the changes
        // stop. cfg_gen is read at write time, by which point Hub::update() has
        // already applied this tick's bump (§4.2).
        _persistArmed = true;
        _persistDueMs = nowMs + kCfgPersistDebounceMs;
    }

    if (uint32_t(nowMs - _lastStatusMs) >= 1000u) {
        _lastStatusMs = nowMs;
        publishHubStatus();
    }

    if (_persistArmed && int32_t(nowMs - _persistDueMs) >= 0) {
        _persistArmed = false;
        return true;
    }
    return false;
}

}  // namespace valence
