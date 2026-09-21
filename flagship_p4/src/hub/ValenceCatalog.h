#pragma once

// ValenceCatalog -- builds this device's SlopSync channel catalog (SPEC §8.1).
//
// Constraints:
//   Hardware-free and library-only: nothing here may include an IDF or board
//   header. The constants it mirrors from valence_config.h are pinned by
//   static_asserts in ValenceHub.cpp, which is the one TU that sees both.
//   DeviceFeatures::has_motion gates EVERY motion and pattern channel. A
//   feature exists IF AND ONLY IF its channels exist (SPEC §6.3): a hub with
//   no motion plane advertising a motion channel that publishes zeros is
//   indistinguishable from an idle machine, the dead-gauge lie
//   has_current_sensor already exists to prevent.
//   Every field, unit, scale, and bit label below is wire-visible and part
//   of the client-invariant etag (SPEC §8.3) — author with the same care as
//   the FROZEN conformance fixture (conformance/mini_catalog.hpp).
//   Entries MUST be added in ASCENDING id order (etag is order-sensitive).
//   Layout entries (STATE/STREAM) must fit limits::min_transport_payload
//   (242 B) unfragmented; largest entry here is 88 B.
//   Names ≤32 B, field names ≤24 B, units ≤8 B (schema/catalog.cddl).
//   A fully-annotated entry must fit limits::catalog_max_entry_bytes (4096);
//   `desc` strings are the only unbounded cost and must stay tight.
//   0x0003/0x0004/0x0005/0x0007 layouts are pinned by the hub's own encoders
//   (buildSafetyPayload / buildControlOwnerPayload / handleIntent's
//   ESTOP_CLEAR path / emitTakeoverEvent) — not free to reshape here without
//   changing the hub in lockstep.
//   Wire sizes are noted per entry so a budget overrun is caught by eye;
//   there is no packed struct to static_assert against.
// See: SlopSync spec/AUTHORING.md ("you want X on screen -> you author Y" —
// read it BEFORE editing annotations here), SPEC.md and registry.yaml
// (SlopSync repo), docs/slopsync/CHANNEL-MAP.md (channel id grid /
// renumber history).

#include <cstdint>

#include "slopsync/channel/catalog.hpp"
#include "slopsync/channel/log_channel.hpp"
#include "slopsync/channel/safety_events_channel.hpp"
#include "slopsync/channel/trust_channels.hpp"

namespace valence {

// Device-catalog channel ids. The reserved 0x0001-0x0007 range is owned by
// the registry (slopsync::channels::); everything >=0x0080 is this device's
// own allocation. Named here so buildValenceCatalog() AND the telemetry
// publisher in SlopSyncHubService reference ONE definition — a literal in
// only one of the two would be a silent wire mismatch.
// Device ids follow the 0xCDSS grid: C=class (1 STATE/2 STREAM/3 INTENT/
// 4 EVENT/5 STORE), D=domain (0 machine/1 motion/2 pattern), S=family,
// S=member (member 0 = family master; a twin channel across class bands
// sharing domain+family+member is a MIRROR; family 0xF = admin/meta).
// Per-line `(was 0xXXXX)` names the immediately preceding id — full renumber
// history lives in docs/slopsync/CHANNEL-MAP.md's generated table, not here.
// A renumber moves the etag, which is the designed re-fetch mechanism, not a
// break; 0x0080-0x7FFF is device-allocated space per the registry.
namespace ch {
inline constexpr uint16_t motion         = 0x1100;  // STATE·motion, family 0 member 0 (master)
inline constexpr uint16_t machine_config = 0x1000;  // STATE·machine, family 0 member 0 (master)
inline constexpr uint16_t pattern_state  = 0x1200;  // STATE·pattern, family 0 member 0 (master); background_run field rides here
inline constexpr uint16_t odometer       = 0x1020;  // STATE·machine, family 2 member 0 (was 0x1002)
inline constexpr uint16_t motion_input   = 0x2100;  // STREAM·motion, family 0 member 0 (master)
inline constexpr uint16_t motion_segment = 0x2101;  // STREAM·motion, family 0 member 1
// ---- telemetry channels the legacy :81 plane owned --------------------------
inline constexpr uint16_t plan_strip     = 0x1110;  // STATE·motion, family 1 member 0 (master; was 0x1101)
inline constexpr uint16_t power          = 0x1010;  // STATE·machine, family 1 member 0 (was 0x1001)
inline constexpr uint16_t motion_diag    = 0x1111;  // STATE·motion, family 1 member 1 (was 0x1102)
inline constexpr uint16_t motion_anomaly = 0x4100;  // EVENT·motion, family 0 member 0 (master)
// ---- MODE settings the legacy :81/HTTP plane owned --------------------------
// A SECOND settings category, not more fields on 0x0081 — the reason is
// structural. 0x0081's `enabled_mask` is a bitfield8 whose bit i gates its
// i-th setting-annotated field, and seven of eight bits are already spoken
// for; widening the mask to fit these four would change an existing field's
// type, which is a protocol break, not append-only evolution. A settings
// category that outgrows its channel SPLITS into a new STATE+INTENT pair.
inline constexpr uint16_t machine_modes  = 0x1030;  // STATE·machine, family 3 member 0 (master; was 0x1003)
// ---- VMotion live tuning, off HTTP and onto the protocol -----------------
// THREE state cards, ONE shared writer (0x0105). `settingChannel` is per-entry
// and `setting_key` is a key WITHIN that writer, so several STATE channels may
// name the same INTENT channel as long as their keys do not collide. That is
// what lets 17 knobs -- more than any single channel's bitfield8 enabled_mask
// can gate -- stay one coherent write path instead of three.
inline constexpr uint16_t sm_limits      = 0x1120;  // STATE·motion, family 2 member 0 (master; was 0x1103)
inline constexpr uint16_t sm_chase       = 0x1121;  // STATE·motion, family 2 member 1 (was 0x1104)
inline constexpr uint16_t sm_waveform    = 0x1122;  // STATE·motion, family 2 member 2 (was 0x1105)
// ---- Servo drive registers, its own family: these configure the DRIVE, not --
// the planner. Family 2 is vmotion's; a drive register that happens to be
// spelled "acceleration" is a different subsystem and gets its own writer.
inline constexpr uint16_t drive_tune     = 0x1130;  // STATE·motion, family 3 member 0 (master)
// ---- Advanced pattern — off the dead /api/pattern HTTP surface, onto SlopSync
// Same flattened-entry budget split as 0x008B/C/D. AdvancedPattern.h's real
// parameter set is 8 base controls (advpat::Settings) plus a 6-field cyclic
// Modifier per base control (advpat::BASE_COUNT = 6), 44 settings total. A
// fully-annotated 50-field entry encodes to ~8-10 KB (catalog_max_entry_bytes);
// fitting in kMaxFields (64) and being affordable in one entry are different
// constraints, so this splits by subsystem — one channel per base control's
// modifier (6 fields each, well under the 8-bit enabled_mask) — same
// principle as the sm_limits/sm_chase/sm_waveform split.
inline constexpr uint16_t pattern_advanced          = 0x1210;  // STATE·pattern, family 1 member 0 (master; was 0x1201) — ap_mode + 7 base controls
// The six fray-d modifier lanes: ONE family (domain=pattern, family=1),
// members 1-6. Member order is speed-in/out, accel-in/out, depth-1/2 — NOT
// advpat::BaseId order (depth,depth,speedin,speedout,accelin,accelout) — see
// kModChannels in SlopSyncHubService.cpp, which maps between the two.
inline constexpr uint16_t pattern_adv_mod_speedin   = 0x1211;  // STATE·pattern, family 1 member 1 (was 0x1204)
inline constexpr uint16_t pattern_adv_mod_speedout  = 0x1212;  // STATE·pattern, family 1 member 2 (was 0x1205)
inline constexpr uint16_t pattern_adv_mod_accelin   = 0x1213;  // STATE·pattern, family 1 member 3 (was 0x1206)
inline constexpr uint16_t pattern_adv_mod_accelout  = 0x1214;  // STATE·pattern, family 1 member 4 (was 0x1207)
inline constexpr uint16_t pattern_adv_mod_depth1    = 0x1215;  // STATE·pattern, family 1 member 5 (was 0x1202) — advpat::DEPTH_MAX modifier
inline constexpr uint16_t pattern_adv_mod_depth2    = 0x1216;  // STATE·pattern, family 1 member 6 (was 0x1203) — advpat::DEPTH_MIN modifier
inline constexpr uint16_t move           = 0x3100;  // INTENT·motion, family 0 member 0 (master)
inline constexpr uint16_t config_set     = 0x3000;  // INTENT·machine, family 0 member 0 (master), MIRROR of machine_config
inline constexpr uint16_t pattern_cmd    = 0x3200;  // INTENT·pattern, family 0 member 0 (master), MIRROR of pattern_state
inline constexpr uint16_t home           = 0x3101;  // INTENT·motion, family 0 member 1
inline constexpr uint16_t modes_set      = 0x3030;  // INTENT·machine, family 3 member 0, MIRROR of machine_modes (was 0x3001)
inline constexpr uint16_t sm_set         = 0x3120;  // INTENT·motion, family 2 member 0, MIRROR of the sm_* family (was 0x3102)
inline constexpr uint16_t machine_admin  = 0x30F0;  // INTENT·machine, family F member 0 = admin (was 0x3002)
inline constexpr uint16_t drive_set      = 0x3130;  // INTENT·motion, family 3 member 0, MIRROR of drive_tune
// Shared writer behind ALL SEVEN pattern-advanced STATE channels — same
// "one settingChannel, many cards" pattern as sm_set. MIRROR of
// pattern_advanced (family 1 member 0 on both sides).
inline constexpr uint16_t pattern_advanced_cmd = 0x3210;  // INTENT·pattern, family 1 member 0 (was 0x3201)
// RFC-021 `pattern.frayd` preset store — retires POST /api/pattern/presets.
// See PatternPresetStore.h for the backend and the STORE/roster entries below.
// All three ride domain=pattern, family=2, member=0 (roster is the STATE
// master; presets/presets_cmd MIRROR it on the STORE and INTENT bands).
inline constexpr uint16_t pattern_presets_roster = 0x1220;  // STATE·pattern, family 2 member 0 (master; was 0x1208)
inline constexpr uint16_t pattern_presets_cmd    = 0x3220;  // INTENT·pattern, family 2 member 0 (was 0x3202)
inline constexpr uint16_t pattern_presets        = 0x5220;  // STORE·pattern, family 2 member 0 (was 0x5200)
}  // namespace ch

// MIRROR of PatternPresetStore::{kCapacity,kNameMax,kPayloadBytes}
// (include/comms/PatternPresetStore.h), same forced-duplication rule as
// kApBaseCount above (this header stays library-only; PatternPresetStore.h is
// itself hardware-free but still a cross-module include this header has never
// taken). SlopSyncHubService.cpp carries a static_assert pinning these
// together, so drift fails the FIRMWARE build, not a silent wire mismatch.
inline constexpr uint8_t kPresetCapacity = 24;
inline constexpr uint8_t kPresetNameMax = 32;
inline constexpr uint8_t kPresetPayloadBytes = 40;

// MIRROR of advpat::BASE_COUNT (include/motion/AdvancedPattern.h), same forced-
// duplication rule as `factory`/`ceiling` below: this header must stay
// buildable with nothing but the library (native tests, the sim), and
// AdvancedPattern.h — though itself hardware-free — is still a cross-module
// dependency this header has never taken. SlopSyncHubService.cpp DOES include
// PatternEngine.h (and therefore AdvancedPattern.h) and carries a static_assert
// pinning this to advpat::BASE_COUNT, so drift fails the FIRMWARE build, not a
// silent wire mismatch.
inline constexpr uint8_t kApBaseCount = 6;

// ---- motion-anomaly EVENT: the `body` (40) sub-map keys ---------------------
// These are the CHANNEL'S OWN schema keys, exactly as slopsync::safety_body is
// for 0x000E — that is the v1.0 EVENT grammar (registry key 40's own note: with
// kind-specific fields at the TOP level, every device-authored EVENT channel
// would need a registry PR to name its own fields). This channel is the first
// DEVICE-authored EVENT channel in the ecosystem and therefore the proof that
// the grammar fix works: nothing below required a registry change.
namespace anom_body {
inline constexpr uint8_t kind   = 1;  // vmotion::AnomalyType, MIRRORS event_kind (see the entry)
inline constexpr uint8_t seq    = 2;  // engine's rolling event id (wraps)
inline constexpr uint8_t target = 3;  // the command target that provoked it, 0..1 normalized
inline constexpr uint8_t detail = 4;  // KIND-SPECIFIC scalar — see the option labels
inline constexpr uint8_t t_us   = 5;  // engine time at record, µs (low 32 bits)
}  // namespace anom_body

// ---- Machine FEATURES that gate whether a channel is advertised AT ALL ------
// RFC-016 in practice: "capability discovery IS catalog introspection". A hub
// with no INA228 must not advertise a power channel that would publish zeros
// forever — a client cannot tell "0.0 A" from "no sensor", and a UI that shows
// a dead gauge is the same ground-truth violation as the WebUI's dead anomaly
// panel this milestone exists to kill. So the channel is ABSENT, and its
// absence IS the answer to "does this machine measure current?".
//
// Defaults are all-false so the HOST tests and any future non-motorized build
// get the minimal catalog unless they say otherwise; the firmware fills these
// in from MotorDriver::hasCurrentSensor()/hasPowerMonitor() at construction.
struct DeviceFeatures {
    bool has_current_sensor = false;  // MotorDriver::hasCurrentSensor()
    bool has_power_monitor  = false;  // MotorDriver::hasPowerMonitor() (die temp)
    // Gates EVERY motion and pattern channel, plus the three machine-domain
    // channels whose content is motion: odometer (0x1020, distance and stroke
    // totals), machine-modes (0x1030, motion_backend / home_style /
    // overshoot_clamp) with its writer 0x3030, and machine-admin (0x30F0,
    // whose ops are clear_fault / servo_scan on a drive that is absent).
    // machine-config (0x1000) and config-set (0x3000) SURVIVE: they are
    // configuration STORAGE, and a stored limit is truthfully what the hub
    // holds, not a gauge reading zero forever.
    bool has_motion         = false;
};

// ---- Factory DEFAULTS advertised as RFC-009 `default` annotations -----------
// MIRROR of getDefaultConfig() in include/system/config_api.h, which cannot be
// included here: it pulls in <Arduino.h>, and this header must stay hardware-
// free (the native test suite and the sim both build it with nothing but the
// library). Duplication is forced so drift is caught, not tolerated:
// SlopSyncHubService.cpp, which DOES include config_api.h, carries a
// static_assert per constant below — change a factory default there and the
// FIRMWARE fails to compile until this table follows.
namespace factory {
inline constexpr float window_min  = 0.0f;        // getDefaultConfig().min_position_mm
inline constexpr float window_max  = 500.0f;      // DEFAULT_MAX_RAIL_MM
inline constexpr float user_speed  = 50.0f;       // DEFAULT_USER_MAX_SPEED_MM_S
inline constexpr float user_accel  = 200.0f;      // DEFAULT_USER_ACCEL_MM_S2
inline constexpr float input_speed = 950.0f;      // DEFAULT_MAX_SPEED_MM_S
inline constexpr float input_accel = 50000.0f;    // DEFAULT_ACCEL_MM_S2
inline constexpr float input_jerk  = 2000000.0f;  // DEFAULT_INPUT_MAX_JERK_MM_S3
// max_rail is a real savable setting, not derived truth — see the field
// comment on 0x0081 below. Same mirror rule as its siblings above.
inline constexpr float max_rail    = 500.0f;      // DEFAULT_MAX_RAIL_MM
// Mode defaults (0x008A). Same forced-duplication rule as above — each one
// is static_assert'd against its real source in SlopSyncHubService.cpp.
// `blend_mode` and `stream_speed_mode` have no `.dflt` here: the settings they
// defaulted were retired from 0x008A (see the field comments there). Do not
// re-add either without re-adding the field's setting_key first.
inline constexpr uint8_t overshoot_clamp   = 0;   // SystemState::interp_clamp_overshoot = false
}  // namespace factory

// ---- Hard firmware ceilings advertised as `min`/`max` -----------------------
// The bounds WebUI::applySettings actually clamps to (src/ui/WebUI.cpp), NOT
// the NORMAL/EXPERT UI guardrails — those are a client-side affordance and a
// static catalog must advertise what the hub will really accept. Same
// static_assert treatment as `factory` above.
namespace ceiling {
inline constexpr float rail_mm    = 2000.0f;      // applySettings' max_rail sanity bound
inline constexpr float rail_min   = 10.0f;        // applySettings' max_rail sanity floor
inline constexpr float speed_min  = 1.0f;
inline constexpr float speed_max  = 10000.0f;     // MAX_SPEED_MM_S
inline constexpr float accel_min  = 10.0f;
inline constexpr float accel_max  = 100000.0f;    // MAX_ACCEL_MM_S2
inline constexpr float jerk_min   = 1000.0f;
inline constexpr float jerk_max   = 50000000.0f;  // MAX_JERK_MM_S3
}  // namespace ceiling

// Fills `c` with this device's catalog. OUT-PARAM, never a return value: a
// Catalog32 is ~22 KB of pooled field storage, so returning one by value
// would put that on the caller's stack — the bug class that has already blown
// a FreeRTOS task stack on this firmware once. Returns c.ok(): false means a
// capacity in Catalog32 was exceeded while building (entries or field pools),
// which is a build-time authoring error, not a runtime condition.
//
// `feat` gates the FEATURE-DEPENDENT entries (see DeviceFeatures). It defaults
// to all-false, so an existing call site keeps the minimal catalog.
inline bool buildValenceCatalog(slopsync::Catalog32& c, DeviceFeatures feat = {}) {
    using slopsync::AccessLevel;
    using slopsync::CborFieldType;
    using slopsync::ChannelClass;
    using slopsync::Direction;
    using slopsync::PackedFieldType;
    using slopsync::Priority;
    using slopsync::SettingDefault;
    namespace roles = slopsync::field_roles;

    c.clear();

    // ---- "safety" — STATE, critical, on-change ------------------------------
    // VERBATIM copy of conformance/mini_catalog.hpp's safety entry: the hub's
    // buildSafetyPayload() hardcodes exactly this 9-byte layout (word
    // bitfield8, cause u8, owner_session u32, estop_seq u16, modes bitfield8).
    // Do NOT reshape it.  [9 B]
    //
    // `modes` (manual_override + bypass_limits): SAFETY-domain state — they
    // render near the rail in a UI, but they change what the machine does
    // with a motion command, so every surface needs them on the retained,
    // critical-priority snapshot rather than a legacy HTTP endpoint. Written
    // via 0x0005 ops override_on/off + bypass_on/off. Append-only: bytes 0..7
    // keep their meaning and offsets exactly.
    c.addEntry({.id = slopsync::channels::safety, .name = "safety",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::critical});
    c.addBitfieldField({.name = "word", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f},
                       {"estop", "stop", "hold", "pause"});
    c.addLayoutField({.name = "cause", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "owner_session", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "estop_seq", .type = PackedFieldType::u16, .unit = "count", .scale = 1.0f});
    c.addBitfieldField({.name = "modes", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f},
                       {"override", "bypass"});

    // ---- "control-owner" — STATE, critical, on-change -----------------------
    // Matches Hub::buildControlOwnerPayload(): 4 × {source u8, owner u32}, in
    // ascending source order, 20 bytes total. Each pair is one arbiter source
    // (0 manual, 1 tcode, 2 pattern, 3 ossm) and the session id that owns it
    // (0 = unowned).  [20 B]
    c.addEntry({.id = slopsync::channels::control_owner, .name = "control-owner",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::critical});
    c.addLayoutField({.name = "src0",   .type = PackedFieldType::u8,  .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "owner0", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "src1",   .type = PackedFieldType::u8,  .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "owner1", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "src2",   .type = PackedFieldType::u8,  .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "owner2", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "src3",   .type = PackedFieldType::u8,  .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "owner3", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});

    // ---- "safety-intents" — INTENT, critical, modest rate -------------------
    // The client sends {1:"op"} where op is a safety_ops:: value (estop=6 and
    // estop_clear=1 are hub-handled; the rest reach the delegate and the hub
    // latches the result — RFC-025a).
    //
    // *** THE ACCESS FLOOR IS `watch`, AND THAT IS THE POINT (RFC-025b). ***
    // `estop` and `stop` are ROLE-EXEMPT: anyone connected — including a
    // watch-only session — may stop this machine. §11.2's "safety outranks
    // authorization", generalized. The failure mode of getting this backwards
    // is "the person standing in the room cannot stop the machine", which is
    // not a permissions bug, it is an injury. Loop-stop spam by a viewer is
    // bounded by the §9.3 intent rate limiter above (20 Hz here) and is a
    // named, accepted risk in §12.1.
    //
    // Everything else — hold/pause/resume/estop_clear/override/bypass —
    // requires `control`, expressed as index-aligned `option_access` (catalog
    // key 17) on the enum-valued `op` field rather than as hub-side code,
    // because a GENERIC client renders this channel from the catalog and must
    // know which ops it may offer. A client that honors key 17 grays the
    // rest correctly (RFC-009's gray-never-hide); one that ignores it
    // discovers the same truth by NACK. Encoding it only in hub code would
    // make the honest client impossible.
    //
    // Index alignment is LITERAL: element i of both lists describes wire value
    // i. safety_ops starts at 1, so index 0 is a PLACEHOLDER — and it carries
    // the label "reserved" rather than "", because an option label may never
    // be empty (an unnamed choice is unrenderable, and the decoder rejects
    // one outright). Its access is `control`, the strict side, so wire value 0
    // is never the cheapest thing on this channel to reach; it NACKs
    // UNSUPPORTED_OP at the delegate regardless.
    c.addEntry({.id = slopsync::channels::safety_intents, .name = "safety-intents",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::watch, .maxRateHz = 20.0f,
                .defaultPriority = Priority::critical});
    c.addSelectSchemaField({.key = 1, .name = "op", .type = CborFieldType::uint_t, .unit = "",
                            .role = "action.safety"},
                           {"reserved", "estop_clear", "stop", "hold", "pause", "resume",
                            "estop", "override_on", "override_off", "bypass_on", "bypass_off"},
                           {AccessLevel::control,  // 0  (placeholder, never an op)
                            AccessLevel::control,  // 1  estop_clear
                            AccessLevel::watch,    // 2  stop          ROLE-EXEMPT
                            AccessLevel::control,  // 3  hold
                            AccessLevel::control,  // 4  pause
                            AccessLevel::control,  // 5  resume
                            AccessLevel::watch,    // 6  estop         ROLE-EXEMPT
                            AccessLevel::control,  // 7  override_on
                            AccessLevel::control,  // 8  override_off
                            AccessLevel::control,  // 9  bypass_on
                            AccessLevel::control});// 10 bypass_off

    // ---- "hub-status" — STATE, background, 1 Hz -----------------------------
    // Slow health telemetry.  [4+4+1+1 = 10 B]
    c.addEntry({.id = slopsync::channels::hub_status, .name = "hub-status",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 1.0f,
                .defaultPriority = Priority::background});
    c.addLayoutField({.name = "heap_free", .type = PackedFieldType::u32, .unit = "B",     .scale = 1.0f});
    c.addLayoutField({.name = "uptime_s",  .type = PackedFieldType::u32, .unit = "s",     .scale = 1.0f,
                      .role = roles::telemetry_uptime});
    c.addLayoutField({.name = "rssi",      .type = PackedFieldType::i8,  .unit = "dBm",   .scale = 1.0f});
    c.addLayoutField({.name = "sessions",  .type = PackedFieldType::u8,  .unit = "count", .scale = 1.0f});
    // `log_dropped` (field 5, appended 10 -> 14 B): SPEC §9.4's VISIBLE drop
    // counter for the log plane. A bounded log that silently eats lines under
    // load is a log you cannot reason about, so the number must be reachable
    // on the wire. Sums both places a line can be lost: the hub's replay ring
    // (Hub::logDropped) and the firmware's httpTask->hub hand-off ring.
    // Append-only — bytes 0..9 keep their offsets.
    c.addLayoutField({.name = "log_dropped", .type = PackedFieldType::u32, .unit = "count", .scale = 1.0f,
                      .desc = "Log lines dropped since boot (replay ring + cross-task bridge)."});

    // ---- "session-events" — EVENT, watch ------------------------------------
    // Payload keys match Hub::emitTakeoverEvent(): {1:"source", 2:"session"}.
    c.addEntry({.id = slopsync::channels::session_events, .name = "session-events",
                .cls = ChannelClass::EVENT, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::normal});
    c.addSchemaField({.key = 1, .name = "source",  .type = CborFieldType::uint_t, .unit = ""});
    c.addSchemaField({.key = 2, .name = "session", .type = CborFieldType::uint_t, .unit = ""});

    // ---- "log" — EVENT, watch, background, replay_depth 32 ------------------
    // The device log, in band. Declared by the library's own builder for the
    // same reason the trust channels are — it is a SPEC-CORE channel whose
    // shape hub and client cannot negotiate, so a hand-authored near-copy
    // would be quietly non-conforming.
    //
    // Declaring it is what makes the VLog bridge REACHABLE: publishLog()
    // returns false on a hub whose catalog has no 0x0008, so without this line
    // the whole bridge is a no-op. The replay depth is the registry default
    // (limits::log_replay_depth_default = 32) — a client that connects AFTER a
    // fault still sees what happened, which is the one sanctioned exception to
    // §9.4's no-replay rule.
    if (!slopsync::addLogChannel(c)) return false;

    // ---- the TRUST ADMINISTRATION surface -----------------------------------
    // session-admin, pending-pairing, pairing-events, and the paired-devices
    // store + its roster, all in the canonical shapes the library declares
    // (lib/slopsync/include/slopsync/channel/trust_channels.hpp). Declared as a
    // group and by the library's own builders rather than hand-authored here,
    // because these are SPEC-CORE channels: hub and client cannot negotiate
    // their shapes, so a device that re-authored them slightly differently
    // would be quietly non-conforming with a perfectly valid-looking catalog.
    //
    // Declaring them is what makes pairing REACHABLE on this machine: the
    // approval verb is an ordinary INTENT and the hub resolves it through the
    // catalog like any other, so an undeclared 0x0009 means an operator has no
    // way to approve anything. The trust ledger's store_id (1) is discovered by
    // the hub FROM this descriptor — the catalog is self-describing, and a
    // store number is agreed by being published rather than legislated.
    if (!slopsync::addTrustChannels(c)) return false;

    // ---- "safety-events" — EVENT, critical, watch ---------------------------
    // The §9.4 EVENT TWIN of the safety latch (0x0003). §5.5/§11.2 require the
    // hub to emit it. Same access and priority as its STATE twin: an edge
    // nobody may be denied and nobody's may be shed.
    if (!slopsync::addSafetyEventsChannel(c)) return false;

    // ---- "motion" — STATE, elevated, 60 Hz ----------------------------------
    // The live carriage snapshot. scale 100 on positions = 10µm wire units;
    // scale 10 on speed = 0.1 mm/s wire units.  [2+2+2+1+2 = 9 B]
    //
    // raw_10um (field 5) is the pre-planning demand — do not split it into
    // its own channel. raw/target/actual are ONE measurement of ONE quantity
    // at three pipeline stages (asked, planned, achieved); splitting them
    // would give the diagnostic CLI independently-paced STATE streams to
    // re-correlate, reintroducing the sampling skew the plot exists to
    // measure. Appending keeps ONE frame/seq/timestamp. Append-only: bytes
    // 0..6 keep their offsets; the etag moves on append, which is the
    // designed re-fetch mechanism.
    //
    // category = motion, rank = hero: THE live motion feed, the machine's
    // face. provenance on pos/tgt/raw marks one quantity at three pipeline
    // stages (demand/planned/actual) for the CLI, which combines it with
    // `window.min|max` from 0x0081 to convert normalized intent into mm.
    auto addMotion = [&]() {
    c.addEntry({.id = ch::motion, .name = "motion",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 60.0f,
                .defaultPriority = Priority::elevated,
                .hasCategory = true, .category = slopsync::ui_categories::motion,
                .hasRank = true, .rank = slopsync::ui_ranks::hero});
    // PLANNED, not actual, since the RP2350 took the plan (sd-4k1.4): this is
    // the coprocessor's RENDERED position, which IS the machine's position
    // truth (docs/rp-motion-port.md). The drive encoder is its AUDITOR and
    // reaches clients through the encoder-deviation channel, not this field.
    c.addLayoutField({.name = "pos_10um", .type = PackedFieldType::u16, .unit = "mm",   .scale = 100.0f,
                      .desc = "Where the carriage is, as the motion processor rendered it.",
                      .role = roles::telemetry_position,
                      .hasRank = true, .rank = slopsync::ui_ranks::hero,
                      .hasProvenance = true, .provenance = slopsync::value_provenance::planned,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm});
    c.addLayoutField({.name = "tgt_10um", .type = PackedFieldType::u16, .unit = "mm",   .scale = 100.0f,
                      .desc = "Where the motion planner is currently driving to.",
                      .role = roles::telemetry_target,
                      .hasRank = true, .rank = slopsync::ui_ranks::hero,
                      .hasProvenance = true, .provenance = slopsync::value_provenance::planned,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm});
    c.addLayoutField({.name = "speed",    .type = PackedFieldType::i16, .unit = "mm/s", .scale = 10.0f,
                      .desc = "Live carriage speed; sign is the direction of travel.",
                      .role = roles::telemetry_velocity,
                      .hasRank = true, .rank = slopsync::ui_ranks::hero,
                      .hasProvenance = true, .provenance = slopsync::value_provenance::actual,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm_s});
    c.addBitfieldField({.name = "flags", .type = PackedFieldType::bitfield8, .unit = "flag", .scale = 1.0f,
                        .desc = "Live machine mode bits.",
                        .hasRank = true, .rank = slopsync::ui_ranks::detail},
                       {"homed", "homing", "gen_running", "paused", "override", "estop", "stream"});
    c.addLayoutField({.name = "raw_10um", .type = PackedFieldType::u16, .unit = "mm",   .scale = 100.0f,
                      // No registry role fits a demand-provenance position on a
                      // STATE channel (command.position is an INTENT role), so
                      // this desc's LEADING CLAUSE is the field's human label:
                      // clients read it instead of the wire name (webui
                      // model/format.js labelFor).
                      .desc = "Asked position, as the controlling input sent it, mapped into "
                              "the stroke window before the planner shaped it.",
                      .hasRank = true, .rank = slopsync::ui_ranks::diagnostic,
                      .hasProvenance = true, .provenance = slopsync::value_provenance::demand,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm});
    };

    // ---- "machine-config" — STATE, normal, on-change ------------------------
    // The full geometry + dual-limit-set snapshot in physical units (f32).
    // Append-only layout: every existing field keeps its offset; the etag
    // changes on append, which is the designed re-fetch mechanism, not a
    // break. Fields live in the catalog's shared layout pool; CatalogEntry::
    // kMaxFields (the codec's per-entry bound) is 64, so appending a field
    // here is an etag bump, not a library change.
    //
    // Every field below carries the paired INTENT key (`setting_key` ->
    // 0x0101), a factory `default`, min/max/step, a `group` card heading, a
    // USER-FACING `desc`, and a registry `role`. `settingChannel` = 0x0101,
    // `category` = limits — a generic client renders a full settings page
    // from the catalog alone.
    //
    // `max_rail` is a REAL SAVABLE SETTING, not derived truth: the
    // user-configured ceiling that bounds the sensorless-homing search sweep
    // and serves as the position ceiling before homing has measured the real
    // stroke (see config_api.h's DEFAULT_MAX_RAIL_MM doc) — on a 2 m rail,
    // set it above 2000 mm so homing's search reaches both hard stops.
    // `measured_stroke` (field 10, below) is the SEPARATE, read-only quantity
    // — what homing actually measured. The two must never be conflated: a
    // client must not adopt one as a stand-in for the other.
    auto addMachineConfig = [&]() {
    c.addEntry({.id = ch::machine_config, .name = "machine-config",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = slopsync::ui_categories::limits,
                .hasSettingChannel = true, .settingChannel = ch::config_set,
                .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addLayoutField({.name = "window_min",  .type = PackedFieldType::f32, .unit = "mm",    .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::rail_mm,
                      .dflt = SettingDefault::ofFloat(factory::window_min),
                      .group = "Stroke window",
                      .desc = "Rearmost point of travel. Everything the machine is told to do is "
                              "mapped into the window between this and the front limit.",
                      .role = roles::window_min, .step = 1.0f,
                      .settingKey = 1, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm});
    c.addLayoutField({.name = "window_max",  .type = PackedFieldType::f32, .unit = "mm",    .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::rail_mm,
                      .dflt = SettingDefault::ofFloat(factory::window_max),
                      .group = "Stroke window",
                      .desc = "Frontmost point of travel. Must be greater than the rear limit; the "
                              "machine never moves past it.",
                      .role = roles::window_max, .step = 1.0f,
                      .settingKey = 2, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm});
    c.addLayoutField({.name = "user_speed",  .type = PackedFieldType::f32, .unit = "mm/s",  .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::speed_min, .max = ceiling::speed_max,
                      .dflt = SettingDefault::ofFloat(factory::user_speed),
                      .group = "Manual limits",
                      .desc = "Speed ceiling for moves YOU drive by hand. Kept gentle by default: "
                              "it is a ceiling, not a target.",
                      .role = roles::limit_user_speed, .step = 1.0f,
                      .settingKey = 3, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm_s});
    c.addLayoutField({.name = "user_accel",  .type = PackedFieldType::f32, .unit = "mm/s2", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::accel_min, .max = ceiling::accel_max,
                      .dflt = SettingDefault::ofFloat(factory::user_accel),
                      .group = "Manual limits",
                      .desc = "How hard a hand-driven move is allowed to pick up speed. Lower "
                              "feels softer at the start and end of every move.",
                      .role = roles::limit_user_accel, .step = 10.0f,
                      .settingKey = 4, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm_s2});
    c.addLayoutField({.name = "input_speed", .type = PackedFieldType::f32, .unit = "mm/s",  .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::speed_min, .max = ceiling::speed_max,
                      .dflt = SettingDefault::ofFloat(factory::input_speed),
                      .group = "Machine-driven limits",
                      .desc = "Speed ceiling for everything the machine drives itself: patterns, "
                              "scripts and live streams. This is your top-speed safety limit.",
                      .role = roles::limit_input_speed, .step = 10.0f,
                      .settingKey = 5, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm_s});
    c.addLayoutField({.name = "input_accel", .type = PackedFieldType::f32, .unit = "mm/s2", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::accel_min, .max = ceiling::accel_max,
                      .dflt = SettingDefault::ofFloat(factory::input_accel),
                      .group = "Machine-driven limits",
                      .desc = "How hard patterns and scripts may change speed. Raise it for snappy "
                              "content, lower it if the machine feels harsh.",
                      .role = roles::limit_input_accel, .step = 100.0f,
                      .settingKey = 6, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm_s2});
    c.addLayoutField({.name = "max_rail",    .type = PackedFieldType::f32, .unit = "mm",    .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::rail_min, .max = ceiling::rail_mm,
                      .dflt = SettingDefault::ofFloat(factory::max_rail),
                      .group = "Rail geometry",
                      .desc = "How far sensorless homing searches for the hard stops. Set it above "
                              "your rail's real length (e.g. 2000mm+ for a 2m rail).",
                      .role = roles::geometry_max_travel, .step = 1.0f,
                      .settingKey = 8, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm});
    c.addLayoutField({.name = "input_jerk",  .type = PackedFieldType::f32, .unit = "mm/s3", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::jerk_min, .max = ceiling::jerk_max,
                      .dflt = SettingDefault::ofFloat(factory::input_jerk),
                      .group = "Machine-driven limits",
                      .desc = "How abruptly machine-driven motion may change its acceleration. "
                              "Protects the mechanics; it is not a smoothing knob.",
                      .role = roles::limit_input_jerk, .step = 1000.0f,
                      .settingKey = 7, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::advanced,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm_s3});
    // DYNAMIC ENABLED STATE. Bit i gates the i-th SETTING-ANNOTATED field of
    // this layout, in layout order:
    //   0 window_min  1 window_max  2 user_speed  3 user_accel
    //   4 input_speed 5 input_accel 6 max_rail     7 input_jerk
    // max_rail sits at bit 6 (declared before input_jerk, which takes bit 7)
    // — a relabeling of what a bit means, never a byte-offset reshuffle
    // (enabled_mask is metadata about the layout, not part of it). All 8
    // bits are spoken for; a 9th setting on this entry must split into a new
    // channel (same precedent as 0x008B/C/D). `enabled_mask` is never itself
    // setting-annotated. Disabled means GRAY, NEVER HIDE. Bit labels name the
    // field each bit gates so the mapping survives encode/decode.
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f,
                        .desc = "Which of these settings the machine will accept right now.",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = slopsync::ui_ranks::detail},
                       {"window_min", "window_max", "user_speed", "user_accel",
                        "input_speed", "input_accel", "max_rail", "input_jerk"});
    // measured_stroke (field 10, byte 33): THE REAL HOMING MEASUREMENT,
    // distinct from max_rail (the configured ceiling above). 0 until the
    // first successful home this boot; then the usable stroke sensorless
    // homing actually felt out between the two hard stops. No setting_key —
    // derived machine truth. The publisher (SlopSyncHubService.cpp) clamps
    // the PRE-HOME value to max_rail (a stale NVS-restored measurement from a
    // prior boot must never overstate the configured ceiling), but a
    // measurement earned by a fresh home this session is trusted even past
    // max_rail — the search sweep bounds hunting, not the result.
    // Append-only: added after enabled_mask, bytes 0..32 keep their offsets.
    c.addLayoutField({.name = "measured_stroke", .type = PackedFieldType::f32, .unit = "mm", .scale = 1.0f,
                      .desc = "Usable stroke length sensorless homing actually measured between the "
                              "two hard stops. Zero until the first successful home.",
                      .role = roles::geometry_measured_travel,
                      .hasRank = true, .rank = slopsync::ui_ranks::detail,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm});
    };

    // ---- "pattern-state" — STATE, normal, on-change -------------------------
    // PatternEngine live snapshot.  [1+1+4+4+4+4+1+1 = 20 B]. Append-only:
    // enabled_mask (field 7) and background_run (field 8, settingKey 7 on the
    // paired 0x3200 pattern-cmd intent) keep bytes 0..18 at their offsets.
    // category = user; settingChannel = 0x0102 pattern-cmd.
    //
    // `pattern` option labels are PatternEngine::patternName()'s own strings,
    // index-aligned with the wire value exactly as setPattern(idx) consumes
    // it — must stay in sync with that function.
    //
    // Fields carry `pattern.*` roles (registry field_roles) so a generic
    // client can draw a proper generator card instead of unrelated sliders —
    // a hint a client MAY upgrade a widget on, never a requirement.
    //
    // Only PatternEngine::CORE_PATTERN_COUNT (7) names are advertised, never
    // the build-flagged extended patterns (PATTERN_EXT_TESTPATTERN1/2): this
    // header is hardware-free and cannot see those flags, and advertising a
    // choice the running firmware might clamp away would violate the
    // ground-truth doctrine. A build that ships the extended patterns can
    // append their labels here — index-aligned and append-only, an etag
    // bump and nothing more.
    auto addPatternState = [&]() {
    c.addEntry({.id = ch::pattern_state, .name = "pattern-state",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = slopsync::ui_categories::control,
                .hasSettingChannel = true, .settingChannel = ch::pattern_cmd,
                .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addLayoutField({.name = "running",   .type = PackedFieldType::u8,  .unit = "",  .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofBool(false),
                      .group = "Pattern",
                      .desc = "Whether the built-in pattern generator is currently driving the "
                              "machine.",
                      .role = roles::pattern_running,
                      .step = 1.0f, .settingKey = 1, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addSelectField({.name = "pattern",   .type = PackedFieldType::u8,  .unit = "",  .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 6.0f,
                      .dflt = SettingDefault::ofInt(0),
                      .group = "Pattern",
                      .desc = "Which stroke pattern the generator plays.",
                      .role = roles::pattern_select,
                      .step = 1.0f, .settingKey = 2, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control},
                     {"Simple Stroke", "Teasing Pounding", "Robo Stroke", "Half'n'Half",
                      "Deeper", "Stop'n'Go", "Insist"});
    c.addLayoutField({.name = "speed",     .type = PackedFieldType::f32, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofFloat(0.0f),
                      .group = "Pattern",
                      .desc = "How fast the pattern strokes, as a percentage of its own range. "
                              "Bounded by the machine-driven speed limit.",
                      .role = roles::pattern_speed,
                      .step = 1.0f, .settingKey = 3, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
    c.addLayoutField({.name = "depth",     .type = PackedFieldType::f32, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofFloat(0.0f),
                      .group = "Pattern",
                      .desc = "How far into the stroke window the pattern reaches.",
                      .role = roles::pattern_depth,
                      .step = 1.0f, .settingKey = 4, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
    c.addLayoutField({.name = "stroke",    .type = PackedFieldType::f32, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofFloat(0.0f),
                      .group = "Pattern",
                      .desc = "Length of each stroke, as a percentage of the available depth.",
                      .role = roles::pattern_stroke,
                      .step = 1.0f, .settingKey = 5, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
    c.addLayoutField({.name = "sensation", .type = PackedFieldType::f32, .unit = "",  .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofFloat(50.0f),
                      .group = "Pattern",
                      .desc = "Pattern character knob. 50 is neutral; what it changes depends on "
                              "the pattern you picked.",
                      .role = roles::pattern_sensation,
                      .step = 1.0f, .settingKey = 6, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control});
    // Bit i gates the i-th setting-annotated field above:
    //   0 running  1 pattern  2 speed  3 depth  4 stroke  5 sensation
    //   6 background_run (see its own field comment for why its bit is
    //     unconditionally 1, unlike bits 0-5)
    // Bits 0-5 are genuinely DYNAMIC: the delegate refuses 0x0102 outright
    // while the e-stop is latched (ESTOP_ACTIVE) or before homing
    // (NOT_HOMED), so all six drop together and a client grays the whole
    // pattern card from one ground truth instead of discovering it one NACK
    // at a time.
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f,
                        .desc = "Which pattern controls the machine will accept right now.",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = slopsync::ui_ranks::detail},
                       {"running", "pattern", "speed", "depth", "stroke", "sensation", "background_run"});
    // `source.background_run` — appended after enabled_mask, settingKey 7
    // (append-only, never inserted before an existing field). Bit 6 of the
    // mask above is UNCONDITIONALLY 1: this is a standing policy choice
    // ("should the generator keep going if I disconnect"), not a live motion
    // command, so unlike bits 0-5 it is never gated by homed/estop — it
    // still needs a bit (every setting-annotated field does), just one that
    // never drops.
    c.addLayoutField({.name = "background_run", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofBool(false),
                      .group = "Pattern",
                      .desc = "Keep the pattern running after its session disconnects. Off stops "
                              "it; on leaves it running, stoppable via Stop/E-Stop.",
                      .role = roles::source_background_run,
                      .step = 1.0f, .settingKey = 7, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control});
    };

    // ---- "odometer" — STATE, background, 1 Hz -------------------------------
    // Session totals.  [4+4+4+4+4 = 20 B]. Append-only: energy_wh/session_ms
    // (fields 4/5) keep strokes/distance_m/peak_mm_s at their offsets.
    // energy_wh is Wh as a float, not a fixed-point milli-Wh u32: the wire is
    // self-describing (unit + scale), so there is no reason to carry an
    // encoding a client has to know about. Reads 0.0 forever on a machine
    // with no power monitor — honest; the capability question is answered
    // by 0x0087's presence, not by this field.
    auto addOdometer = [&]() {
    c.addEntry({.id = ch::odometer, .name = "odometer",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 1.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = slopsync::ui_categories::system,
                .hasRank = true, .rank = slopsync::ui_ranks::diagnostic});
    // aspect/scope: every field here is a session-scope figure (RENDERING.md
    // §5.2 scope=session is the default, set explicitly per the honesty rule
    // — §5.4 "scope MUST always be displayed or unambiguously implied").
    // aspect is total(4) for the cumulative counters and peak(1) for
    // peak_mm_s, its companion-instrument tag (§5.4).
    c.addLayoutField({.name = "strokes",    .type = PackedFieldType::u32, .unit = "",     .scale = 1.0f,
                      .group = "Session", .desc = "Direction reversals counted this session.",
                      .hasAspect = true, .aspect = slopsync::value_aspects::total,
                      .hasScope = true, .scope = slopsync::value_scopes::session,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::count});
    c.addLayoutField({.name = "distance_m", .type = PackedFieldType::f32, .unit = "m",    .scale = 1.0f,
                      .group = "Session", .desc = "Total distance the carriage has traveled this session.",
                      .hasAspect = true, .aspect = slopsync::value_aspects::total,
                      .hasScope = true, .scope = slopsync::value_scopes::session});
                      // unit_id deliberately absent: unit_ids has no meters (only mm, id 0) and
                      // reporting mm here would misstate the physical unit — falls back to the
                      // "m" string label (the honest, documented unit_ids gap).
    c.addLayoutField({.name = "peak_mm_s",  .type = PackedFieldType::f32, .unit = "mm/s", .scale = 1.0f,
                      .group = "Session", .desc = "Fastest the carriage moved this session.",
                      .hasAspect = true, .aspect = slopsync::value_aspects::peak,
                      .hasScope = true, .scope = slopsync::value_scopes::session,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm_s});
    c.addLayoutField({.name = "energy_wh",  .type = PackedFieldType::f32, .unit = "Wh",   .scale = 1.0f,
                      .group = "Session", .desc = "Electrical energy drawn this session. Zero if this "
                                                  "machine has no power monitor.",
                      .hasAspect = true, .aspect = slopsync::value_aspects::total,
                      .hasScope = true, .scope = slopsync::value_scopes::session,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::wh});
    c.addLayoutField({.name = "session_ms", .type = PackedFieldType::u32, .unit = "ms",   .scale = 1.0f,
                      .group = "Session", .desc = "Time since boot, or since the session counters were "
                                                  "last reset.",
                      .hasAspect = true, .aspect = slopsync::value_aspects::total,
                      .hasScope = true, .scope = slopsync::value_scopes::session,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::ms});
    };

    // ---- "motion-input" — STREAM, c2h, control, ≤333 Hz ---------------------
    // Continuous stroke-window targets + optional signed handoff velocity,
    // decoded straight off BundleView by the hub delegate's onStreamBundle()
    // into the VMotion pacing ring; maps to arbiter source 1
    // (MotionSource::TCODE_STREAM), the same source id legacy TCode uses.
    // scale 10000 on target = 1e-4 resolution over the 0..1 stroke window;
    // scale 1000 on vel = 1e-3 resolution, i16 signed (0 = no handoff
    // velocity). SPEC Appendix D sketches 0x0081 as "motion-input", but this
    // firmware spent 0x0081 on machine-config — 0x0084 is this device's
    // actual allocation; the catalog is self-describing and authoritative
    // per Appendix D's own disclaimer.  [2+2 = 4 B]
    auto addMotionInput = [&]() {
    c.addEntry({.id = ch::motion_input, .name = "motion-input",
                .cls = ChannelClass::STREAM, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 333.0f,
                .defaultPriority = Priority::elevated,
                // ui_categories::control's own note names "streams" explicitly.
                .hasCategory = true, .category = slopsync::ui_categories::control,
                .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addLayoutField({.name = "target_norm", .type = PackedFieldType::u16, .unit = "norm",   .scale = 10000.0f,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::normalized});
    c.addLayoutField({.name = "vel_norm",    .type = PackedFieldType::i16, .unit = "norm/s", .scale = 1000.0f});
                      // unit_id left absent for vel_norm: unit_ids has no "normalized/s" variant
                      // (a documented gap, same class as the sm_limits override fields below).
    };

    // ---- "motion-segment" — STREAM, c2h, control, ≤50 Hz --------------------
    // TIMED-SEGMENT motion streaming, the WAVEFORM-mode companion to 0x0084.
    // Carries the sender's native segments — ONE {target, duration, end_vel}
    // per stroke leg — which the VMotion engine renders as a C2 quintic
    // over EXACTLY the commanded duration. Decoded by FIXED OFFSET in the
    // delegate's onStreamBundle() (same convention as 0x0084), enqueued into
    // the SAME VMotion pacing ring, mapped to arbiter source 1
    // (TCODE_STREAM) — a client uses 0x0084 OR 0x0085, both ARE "the stream
    // input".
    //   * duration_ms is the commanded segment duration and MUST be ≥1;
    //     durationless points belong on 0x0084 (a 0 here is skipped and
    //     counted dropped, never sent to the engine).
    //   * end_vel_norm == -32768 (INT16_MIN) is the "NO end velocity"
    //     SENTINEL: 0 is a legitimate slope (a reversal ends AT rest), so 0
    //     cannot mean "absent". On the sentinel the engine estimates the
    //     boundary accel/vel itself (backward-difference af + stream vf);
    //     otherwise it honors the wire handoff velocity verbatim.
    //   * bundle sample timestamps (§5.4 t_off) are the intended segment
    //     START in hub time, resolved through the same nearest-window pacing
    //     as 0x0084.
    // scale 10000 on target = 1e-4 over the 0..1 window; scale 1000 on
    // end_vel = 1e-3 units/s, i16 signed.  [2+2+2 = 6 B]
    // streamKind = segments: EACH SAMPLE CARRIES ITS OWN duration_ms, so it
    // commands a time extent, not an instant. A dropped segment is a
    // permanently lost motion command (not a recoverable interpolation gap
    // like 0x0084's points) — the hub's shedding table must NEVER decimate
    // this channel. 0x0084 stays at the stream_kind DEFAULT (samples)
    // deliberately: absent-means-samples is the rule.
    auto addMotionSegment = [&]() {
    c.addEntry({.id = ch::motion_segment, .name = "motion-segment",
                .cls = ChannelClass::STREAM, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 50.0f,
                .defaultPriority = Priority::elevated,
                .hasCategory = true, .category = slopsync::ui_categories::control,
                .streamKind = slopsync::stream_kinds::segments,
                .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addLayoutField({.name = "target_norm",  .type = PackedFieldType::u16, .unit = "norm",   .scale = 10000.0f,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::normalized});
    c.addLayoutField({.name = "duration_ms",  .type = PackedFieldType::u16, .unit = "ms",     .scale = 1.0f,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::ms});
    c.addLayoutField({.name = "end_vel_norm", .type = PackedFieldType::i16, .unit = "norm/s", .scale = 1000.0f});
    };

    // ---- "plan-strip" — STATE, elevated, 45 Hz ------------------------------
    // THE PLANNER'S CURRENT SEGMENT: what VMotion is executing right now,
    // as a strip you can draw. Together with 0x0080's raw/tgt/pos triple it
    // is the whole input for the diagnostic graphing CLI: raw demand in,
    // planner shape out, carriage response.
    //
    // STATE, not STREAM: a SNAPSHOT of a thing that is continuously true (the
    // active plan), not a series of commands or timed samples — a subscriber
    // that falls behind wants the CURRENT segment, never a backlog of stale
    // ones, so conflation is the right loss behavior. Stays at the
    // stream_kind DEFAULT and declares nothing: `streamKind` is read only for
    // STREAM-class entries (isSegmentClass), so marking a STATE channel
    // `samples` would imply a classification that does not apply.
    //
    // Normalized units, matching the engine's own domain (1.0 == the full
    // stroke window): positions scale 10000, velocity scale 1000.
    // durationUs/elapsedUs stay µs u32.  [1+1+2+2+2+2+4+4 = 18 B]
    auto addPlanStrip = [&]() {
    c.addEntry({.id = ch::plan_strip, .name = "plan-strip",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 45.0f,
                .defaultPriority = Priority::elevated,
                .hasCategory = true, .category = slopsync::ui_categories::tuning,
                .hasRank = true, .rank = slopsync::ui_ranks::diagnostic});
    c.addBitfieldField({.name = "flags", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f,
                        .group = "Active plan",
                        .desc = "Whether a plan is running, and which planner produced it."},
                       {"active", "live_mode", "grad_mode"});
    // RFC-035: the plan.* role family — a generic plan-strip widget finds this
    // channel BY ROLE on any machine, replacing the reference client's
    // documented /plan/i entry-name regex (which silently fails on a hub that
    // names the concept differently).
    c.addSelectField({.name = "style", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .group = "Active plan",
                      .desc = "Which planning mode the motion core is in.",
                      .role = roles::plan_style},
                     {"idle", "waveform", "chase", "settle"});
    c.addLayoutField({.name = "start_norm", .type = PackedFieldType::u16, .unit = "norm",   .scale = 10000.0f,
                      .group = "Active plan", .desc = "Where the current plan started.",
                      .role = roles::plan_start});
    c.addLayoutField({.name = "end_norm",   .type = PackedFieldType::u16, .unit = "norm",   .scale = 10000.0f,
                      .group = "Active plan", .desc = "Where the current plan ends.",
                      .role = roles::plan_end});
    c.addLayoutField({.name = "cur_norm",   .type = PackedFieldType::u16, .unit = "norm",   .scale = 10000.0f,
                      .group = "Active plan", .desc = "The setpoint the plan is producing right now.",
                      .role = roles::plan_current});
    c.addLayoutField({.name = "cur_vel",    .type = PackedFieldType::i16, .unit = "norm/s", .scale = 1000.0f,
                      .group = "Active plan", .desc = "The plan's velocity right now, signed.",
                      .role = roles::plan_velocity});
    c.addLayoutField({.name = "duration_us", .type = PackedFieldType::u32, .unit = "us",    .scale = 1.0f,
                      .group = "Active plan", .desc = "How long the current plan runs in total.",
                      .role = roles::plan_duration});
    c.addLayoutField({.name = "elapsed_us",  .type = PackedFieldType::u32, .unit = "us",    .scale = 1.0f,
                      .group = "Active plan", .desc = "How far into the current plan we are.",
                      .role = roles::plan_elapsed});
    };

    // ---- "power" — STATE, background, 10 Hz ---------------------------------
    // Bus voltage / current / die temperature, feeding the BUS A/V and DIE
    // degC meter tiles.
    //
    // *** DECLARED ONLY WHEN THE HARDWARE EXISTS. *** A machine with no
    // INA228 does not advertise this channel at all — its ABSENCE answers
    // "can this hub measure power?". Publishing zeros instead would be
    // indistinguishable from an idle machine, the same class of lie as a
    // dead gauge.
    //
    // hasCurrentSensor() gates bus V/A, hasPowerMonitor() adds die temp
    // separately, so a rig with a shunt but no thermal sensor advertises a
    // 3-field entry rather than a 4-field one with a permanently-zero
    // column. Byte offsets differ between those builds; that is fine and is
    // precisely why the catalog is fetched per firmware and etag-keyed
    // rather than assumed.
    //
    // i_bus_mA lives HERE, not on 0x0080, deliberately: it is a slow,
    // background diagnostic, and putting it on the 60 Hz motion snapshot
    // would grow the highest-rate channel to carry a value nothing on the
    // motion path reads.  [2+2+2 = 6 B, or +2 = 8 B with a power monitor]
    auto addPower = [&]() {
    if (feat.has_current_sensor) {
        c.addEntry({.id = ch::power, .name = "power",
                    .cls = ChannelClass::STATE, .dir = Direction::h2c,
                    .access = AccessLevel::watch, .maxRateHz = 10.0f,
                    .defaultPriority = Priority::background,
                    .hasCategory = true, .category = slopsync::ui_categories::system,
                    .hasRank = true, .rank = slopsync::ui_ranks::diagnostic});
        c.addLayoutField({.name = "bus_mV",  .type = PackedFieldType::u16, .unit = "V", .scale = 1000.0f,
                          .group = "Power", .desc = "DC bus voltage feeding the motor drive.",
                          .role = roles::telemetry_power_bus});
        c.addLayoutField({.name = "peak_mA", .type = PackedFieldType::u16, .unit = "A", .scale = 1000.0f,
                          .group = "Power",
                          .desc = "Largest bus current seen since the counters were last reset."});
        c.addLayoutField({.name = "i_bus_mA", .type = PackedFieldType::i16, .unit = "A", .scale = 1000.0f,
                          .group = "Power", .desc = "Bus current right now; sign follows the drive.",
                          .role = roles::telemetry_current});
        if (feat.has_power_monitor) {
            c.addLayoutField({.name = "die_c10", .type = PackedFieldType::i16, .unit = "C", .scale = 10.0f,
                              .group = "Power", .desc = "Power-monitor die temperature.",
                              .role = roles::telemetry_temp});
        }
    }
    };

    // ---- "vmotion-diag" — STATE, background, 1 Hz ------------------------
    // Plan counts, the per-kind anomaly breakdown, the on-device plan-time
    // bench, and the SlopSync stream-ingress counters.
    //
    // The per-kind counters are eleven NAMED fields rather than one array: a
    // generic client renders named fields with no per-device knowledge.
    // Their order is vmotion::AnomalyType's own, which is APPEND-ONLY
    // upstream, so a new engine kind appends a field to the END OF THIS
    // BLOCK — shifting every offset after it. The catalog's own layout is
    // what a client decodes against and the etag moves with it, so that is
    // a resync, not a break — before the v1.0 tag. After it, a new kind
    // wants its own channel rather than a reshuffled 0x0088.
    //
    // reset_gen is the observable-reset half: every applied counter reset
    // increments it, so EVERY subscriber sees the reset happened, not only
    // the session that asked for it. Without it a client watching the
    // counters cannot tell a reset from a reboot from a wrap.
    //   [3*4 + 11*4 + 12 + 5*4 + 2 + 1 + 1 = 92 B]
    auto addMotionDiag = [&]() {
    c.addEntry({.id = ch::motion_diag, .name = "vmotion-diag",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 1.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = slopsync::ui_categories::tuning,
                .hasRank = true, .rank = slopsync::ui_ranks::diagnostic});
    c.addLayoutField({.name = "plans",    .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Planner", .desc = "Motion plans computed successfully."});
    c.addLayoutField({.name = "failures", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Planner", .desc = "Commands the planner rejected; the previous plan kept running."});
    c.addLayoutField({.name = "anomalies", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Planner", .desc = "Total planner anomalies of every kind."});
    c.addSelectField({.name = "mode",      .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .group = "Planner", .desc = "Which planning mode the motion core is in."},
                     {"idle", "waveform", "chase", "settle"});
    // Options are indexed by vmotion::PlanKind and the enum is APPEND-ONLY.
    // "cubic" (=3) arrived with curve_policy/ForceC1: a C1 cubic and a C2 quintic
    // are different curves and the client must be able to tell them apart, so
    // this list grows rather than collapsing both into "hermite".
    c.addSelectField({.name = "plan_kind", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .group = "Planner", .desc = "Which curve the active plan is."},
                     {"none", "quintic", "ruckig", "cubic"});
    // Per-kind breakdown — names are vmotion::AnomalyType's, index 0 is
    // the engine's own "none" placeholder and is never counted.
    c.addLayoutField({.name = "anom_none",        .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies", .desc = "Placeholder slot; never counts."});
    c.addLayoutField({.name = "anom_plan_failed", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies", .desc = "A command could not be planned at all."});
    c.addLayoutField({.name = "anom_settle",      .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies", .desc = "The stream stopped sending mid-move; the machine braked to rest."});
    c.addLayoutField({.name = "anom_endvel_clamped", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies", .desc = "A handoff speed was cut back to stay inside the window."});
    c.addLayoutField({.name = "anom_deadline_stretched", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies", .desc = "A move needed longer than the time it was given."});
    c.addLayoutField({.name = "anom_waveform_fallback",  .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies", .desc = "The sender's curve broke a limit; the machine reshaped it."});
    c.addLayoutField({.name = "anom_waveform_scaled",    .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies", .desc = "A stroke was shortened to finish on time."});
    // Retired kind (centering left the engine 2026-09-03); the counter stays
    // in the layout so the per-kind table keeps its positions, and hidden so
    // no renderer draws a permanent zero (sd-djg).
    c.addLayoutField({.name = "anom_waveform_centered",   .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies", .desc = "A shortened stroke was re-centered on its midpoint.",
                      .hasRank = true, .rank = slopsync::ui_ranks::hidden});
    c.addLayoutField({.name = "anom_handoff_bounded",    .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies",
                      .desc = "A sender asked to arrive at a speed the next segment could not "
                              "absorb; the machine bounded it."});
    c.addLayoutField({.name = "anom_waveform_smoothed", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies",
                      .desc = "A curve was flattened toward a straight line so the machine "
                              "could keep the timing without losing the stroke."});
    c.addLayoutField({.name = "anom_dwell_zeroed", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Anomalies",
                      .desc = "A hold was re-sent carrying a stale arrival speed; the machine "
                              "ignored it and stayed put."});
    c.addLayoutField({.name = "plan_us_last", .type = PackedFieldType::u32, .unit = "us", .scale = 1.0f,
                      .group = "Plan time", .desc = "Time the most recent plan took to compute."});
    c.addLayoutField({.name = "plan_us_max",  .type = PackedFieldType::u32, .unit = "us", .scale = 1.0f,
                      .group = "Plan time", .desc = "Worst plan time since the counters were reset."});
    c.addLayoutField({.name = "plan_us_avg",  .type = PackedFieldType::f32, .unit = "us", .scale = 1.0f,
                      .group = "Plan time", .desc = "Smoothed average plan time."});
    c.addLayoutField({.name = "sync_bundles",  .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Stream ingress", .desc = "Motion bundles accepted over SlopSync."});
    c.addLayoutField({.name = "sync_samples",  .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Stream ingress", .desc = "Motion samples decoded from those bundles."});
    c.addLayoutField({.name = "sync_enqueued", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Stream ingress", .desc = "Samples that reached the motion core."});
    c.addLayoutField({.name = "sync_dropped",  .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Stream ingress",
                      .desc = "Samples discarded before the motion core: too late, unusable, or "
                              "refused because the machine was not ready."});
    c.addLayoutField({.name = "sync_seg_bundles", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = "Stream ingress", .desc = "How many of those bundles were timed segments."});
    c.addLayoutField({.name = "reset_gen", .type = PackedFieldType::u16, .unit = "", .scale = 1.0f,
                      .group = "Planner",
                      .desc = "Counts up every time these counters are reset, so every viewer "
                              "sees the reset and not just whoever asked for it.",
                      .role = roles::meta_reset_gen});
    };

    // ---- "motion-anomaly" — EVENT, watch, normal ----------------------------
    // VMotion's anomaly feed, as EDGES.
    //
    // FIRST DEVICE-AUTHORED EVENT CHANNEL: every field below is keyed by
    // THIS CHANNEL'S OWN schema (valence::anom_body), naming them costs no
    // registry PR — under a kind-specific-fields-at-top-level grammar this
    // channel could not exist without one, which the self-describing
    // catalog's body-map grammar exists to prevent.
    //
    // `kind` appears BOTH as the frame's event_kind (33) — the protocol's own
    // discriminator a client switches on — and as body key 1 carrying the
    // identical value. Not redundancy for its own sake: the catalog has no
    // vocabulary for LABELING event kinds, and `options` on a schema field is
    // the one registered mechanism for turning a number into a name.
    // Mirroring it into the body lets a generic client print
    // "waveform_scaled" instead of "6".
    //
    // NO replay depth: an anomaly is an edge, and §9.4's default (edges are
    // never replayed) is right for it. The counters on 0x0088 are the
    // durable record — the event/state duality doing its job.
    auto addMotionAnomaly = [&]() {
    c.addEntry({.id = ch::motion_anomaly, .name = "motion-anomaly",
                .cls = ChannelClass::EVENT, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = slopsync::ui_categories::tuning,
                .hasRank = true, .rank = slopsync::ui_ranks::diagnostic});
    c.addSelectSchemaField({.key = anom_body::kind, .name = "kind", .type = CborFieldType::uint_t,
                            .unit = "",
                            .desc = "What the motion core had to do differently, and why."},
                           {"none", "plan_failed", "settle", "endvel_clamped", "deadline_stretched",
                            "waveform_fallback", "waveform_scaled", "waveform_centered",
                            "handoff_bounded", "waveform_smoothed", "dwell_zeroed"});
    c.addSchemaField({.key = anom_body::seq, .name = "seq", .type = CborFieldType::uint_t, .unit = "",
                      .desc = "Rolling event id from the motion core; wraps."});
    c.addSchemaField({.key = anom_body::target, .name = "target", .type = CborFieldType::f32_t,
                      .unit = "norm",
                      .desc = "The commanded position that provoked it, 0..1 across the stroke window."});
    c.addSchemaField({.key = anom_body::detail, .name = "detail", .type = CborFieldType::f32_t, .unit = "",
                      .desc = "Kind-specific number: the clamped speed, the stretched duration, or "
                              "the fraction of the stroke actually achieved."});
    c.addSchemaField({.key = anom_body::t_us, .name = "t_us", .type = CborFieldType::uint_t, .unit = "us",
                      .desc = "Motion-core time when it happened."});
    };

    // ---- "machine-modes" — STATE, elevated, on-change -----------------------
    // One MODE setting left: overshoot_clamp. blend_mode_reserved and
    // stream_speed_reserved are retired bytes (see below), and
    // `transport` (WS_OP_MODE) is a PERMANENT GAP at INTENT key 2 — see
    // ch::modes_set's note.
    //
    // `blend_mode` is RETIRED: MotionArbiter::setBlendMode() aliases every
    // mode to "allow", and the driver-level stream dispatch
    // (AIMServoDriver::streamTo/streamToSteps) never reads _blend_mode —
    // there is no live motion behavior behind this control. The BYTE STAYS
    // (renamed `blend_mode_reserved`, still occupies byte 0 so bytes 1..3
    // keep their offsets — packed layouts are append-only, deleting the byte
    // would be a wire break) but carries NO setting_key, so no generic
    // client renders a control for it. The paired INTENT key (0x0104 key 1)
    // is retired too — see the modes_set case in SlopSyncHubService.cpp —
    // a SECOND permanent gap alongside key 2's `transport`.
    //
    // `stream_speed_mode` is RETIRED the same way and for the same reason:
    // the S3-side stream speed feed it chose between went with the motion
    // port (docs/rp-motion-port.md), so nothing reads it to make a decision.
    // BYTE STAYS as `stream_speed_reserved` at byte 1 so bytes 2..3 keep
    // their offsets; INTENT key 3 is retired with a permanent gap.
    //
    // They are MODES, not limits: each one changes what the machine DOES
    // with a command rather than how far or how fast it may go — their own
    // category rather than more fields on 0x0081 (see ch::machine_modes for
    // the enabled_mask arithmetic that makes the split structural).
    //
    // Layout [1+1+1+1 = 4 B], all u8 — small enough that the on-change
    // cadence costs nothing, and every live value is an enum the catalog
    // names, so a generic client renders two dropdowns without knowing this
    // device exists.
    auto addMachineModes = [&]() {
    c.addEntry({.id = ch::machine_modes, .name = "machine-modes",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::elevated,
                .hasCategory = true, .category = slopsync::ui_categories::tuning,
                .hasSettingChannel = true, .settingChannel = ch::modes_set,
                .hasRank = true, .rank = slopsync::ui_ranks::advanced});
    // RETIRED — see the entry comment above. Plain reserved byte, no
    // options/group/default/setting_key: nothing should render this. The
    // publisher still writes the driver's (inert) getBlendMode() value here
    // rather than a hardcoded 0 — the byte's CONTENT is no longer meaningful
    // either way.
    c.addLayoutField({.name = "blend_mode_reserved", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .desc = "Retired. Unused padding now, the motion policy it once set is gone. "
                              "Motion always behaves as 'allow'."});
    // RETIRED — see the entry comment above. Plain reserved byte, no
    // options/group/default/setting_key: nothing should render this.
    c.addLayoutField({.name = "stream_speed_reserved", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .desc = "Retired. Unused padding now, the speed feed it selected between "
                              "is gone. A streamed point takes the speed its plan derives."});
    // rank = hidden. INERT: `interp_clamp_overshoot` is consumed by nothing
    // on the live engine; this is the released-but-inert-field case
    // ui_ranks::hidden exists for (RENDERING.md §4), overriding the
    // `advanced` setting_flag rather than stacking with it — hidden is the
    // stronger, terminal statement.
    c.addSelectField({.name = "overshoot_clamp", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(factory::overshoot_clamp),
                      .group = "Motion behavior",
                      .desc = "Stops a smoothed curve from bulging past the points it was given. "
                              "Costs a little smoothness to remove overshoot micromotion.",
                      .settingKey = 4, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::hidden},
                     {"off", "on"});
    // Bit i gates the i-th setting-annotated field, same rule as 0x0081.
    // Neither reserved byte carries a setting_key, so overshoot_clamp,
    // motion_backend and home_style (the last two appended after this mask
    // byte, packed layouts being append-only) are bits 0, 1 and 2.
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f,
                        .desc = "Which of these the machine will accept right now.",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = slopsync::ui_ranks::detail},
                       {"overshoot_clamp", "motion_backend", "home_style"});
    // Which path actually drives the motor. restart_required is the whole
    // contract: the NVS key is read once in setup() before anything touches
    // the motor reference, so a live switch is not expressible. Applying it
    // stores the choice and changes nothing until the next boot.
    c.addSelectField({.name = "motion_backend", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(0),
                      .group = "Motion behavior",
                      .desc = "Which path drives the motor: a step/dir pulse train, or absolute "
                              "setpoints over RS485. Takes effect at the next boot.",
                      .settingKey = 5,
                      .flags = uint8_t(slopsync::setting_flags::advanced |
                                       slopsync::setting_flags::restart_required),
                      .hasSettingKey = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::advanced},
                     {"step-dir", "modbus"});
    // Live-applied, no restart: read fresh at the start of every homing cycle.
    // Only the Modbus backend honors it; step/dir mode always runs its own
    // current-stall sweep.
    c.addSelectField({.name = "home_style", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(0),
                      .group = "Motion behavior",
                      .desc = "How the machine finds home: feel for the hard stops itself, or "
                              "hand the whole cycle to the drive.",
                      .settingKey = 6, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::advanced},
                     {"sensorless sweep", "drive built-in"});
    };

    // ---- "vmotion-*" — STATE, tuning -------------------------------------
    // The motion engine's live-tune surface. No controls outside SlopSync.
    //
    // THREE CHANNELS, ONE TAB. A settings channel is capped at 8 settings
    // because its enabled_mask is a bitfield8 and bit i gates the i-th
    // setting of ITS layout — a WIRE limit the user never sees: SPEC §8.8
    // ("a category spans channels; two channels in the same category merge
    // into one tab") lets all three carry category = tuning and differ only
    // by `group`. One Tuning tab, three cards, nothing dropped.
    //
    // ONE SHARED WRITER (0x0105). `settingChannel` is per-entry and
    // `setting_key` is a key WITHIN that writer, so several STATE channels
    // may name the same INTENT channel provided their keys never collide.
    // Keys are allocated 1..20 across the three cards and are never reused.
    //
    // PERSISTED to NVS: these are real settings and survive a reboot. That is
    // also why they carry `default` annotations — a generic client needs to
    // offer "reset to factory" for a value that sticks.
    auto addSmLimits = [&]() {
    c.addEntry({.id = ch::sm_limits, .name = "vmotion-limits",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = slopsync::ui_categories::tuning,
                .hasSettingChannel = true, .settingChannel = ch::sm_set,
                .hasRank = true, .rank = slopsync::ui_ranks::advanced});
    c.addLayoutField({.name = "jmax_ovr", .type = PackedFieldType::f32, .unit = "1/s3", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 2000000.0f,
                      .dflt = SettingDefault::ofFloat(0.0f), .group = "Ceiling overrides",
                      .desc = "Jerk ceiling for the planner. 0 derives it from the machine limits.",
                      .role = "", .step = 1000.0f,
                      .settingKey = 1, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true, .hasStep = true});
    c.addLayoutField({.name = "vmax_ovr", .type = PackedFieldType::f32, .unit = "1/s", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 20.0f,
                      .dflt = SettingDefault::ofFloat(0.0f), .group = "Ceiling overrides",
                      .desc = "Speed ceiling override, normalized. 0 derives it from the mm limits.",
                      .settingKey = 2, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true});
    c.addLayoutField({.name = "amax_ovr", .type = PackedFieldType::f32, .unit = "1/s2", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 500.0f,
                      .dflt = SettingDefault::ofFloat(0.0f), .group = "Ceiling overrides",
                      .desc = "Acceleration ceiling override, normalized. 0 derives it from the mm limits.",
                      .settingKey = 3, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true});
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Which of these the machine will accept right now.",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = slopsync::ui_ranks::detail},
                       {"jmax_ovr", "vmax_ovr", "amax_ovr"});
    };

    // BOTH INPUT PATHS ARE LIVE AND IN USE. These knobs steer the CHASE path
    // (dense sample streams — MFP's Samples mode); the waveform path
    // (timed segments — MFP's Segments mode) has its own card below. This is a
    // per-path split, NOT a legacy one: neither path is deprecated and the
    // plugin ships both.
    //
    // All of them are wired — every one reaches the engine config on the
    // per-tick push — so the mask reports them ENABLED, which is the truth. A
    // knob that is accepted but whose path is not currently active is a
    // different statement from a knob the machine refuses, and graying it would
    // be exactly the lie enabled_mask exists to prevent.
    auto addSmChase = [&]() {
    c.addEntry({.id = ch::sm_chase, .name = "vmotion-chase",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = slopsync::ui_categories::tuning,
                .hasSettingChannel = true, .settingChannel = ch::sm_set,
                .hasRank = true, .rank = slopsync::ui_ranks::advanced});
    c.addSelectField({.name = "chase_ff", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(1), .group = "Sample streams",
                      .desc = "Aim at where the sender is heading, not just where it last was.",
                      .settingKey = 6, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true},
                     {"off", "on"});
    c.addSelectField({.name = "chase_accel_ff", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(1), .group = "Sample streams",
                      .desc = "Also match how the sender's speed is changing, not just its speed.",
                      .settingKey = 7, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true},
                     {"off", "on"});
    c.addLayoutField({.name = "chase_gain", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.5f,
                      .dflt = SettingDefault::ofFloat(0.9f), .group = "Sample streams",
                      .desc = "Damping on the speed estimate. Lower is steadier, higher is more responsive.",
                      .step = 0.05f, .settingKey = 8, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true, .hasStep = true});
    c.addLayoutField({.name = "chase_lookahead", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 8.0f,
                      .dflt = SettingDefault::ofFloat(3.0f), .group = "Sample streams",
                      .desc = "How far ahead to aim, in stream intervals. Too far overshoots at turns.",
                      .step = 0.5f, .settingKey = 9, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true, .hasStep = true});
    c.addLayoutField({.name = "chase_dense_ms", .type = PackedFieldType::u32, .unit = "ms", .scale = 1000.0f,
                      .hasMin = true, .hasMax = true, .min = 10.0f, .max = 500.0f,
                      .dflt = SettingDefault::ofFloat(60.0f), .group = "Sample streams",
                      .desc = "Streams faster than this count as dense and get predictive aiming.",
                      .settingKey = 10, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true});
    c.addSelectField({.name = "chase_aim_extrap", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(1), .group = "Sample streams",
                      .desc = "Second-order aiming. Sharper tracking, but can overshoot at turn points.",
                      .settingKey = 11, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true},
                     {"off", "on"});
    c.addLayoutField({.name = "handoff_k", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 8.0f,
                      .dflt = SettingDefault::ofFloat(1.5f), .group = "Sample streams",
                      .desc = "Bound on handoff speed between moves, as a multiple of the chord.",
                      .step = 0.1f, .settingKey = 12, .flags = slopsync::setting_flags::advanced,
                      .hasSettingKey = true, .hasStep = true});
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Which of these the machine will accept right now.",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = slopsync::ui_ranks::detail},
                       {"chase_ff", "chase_accel_ff", "chase_gain", "chase_lookahead",
                        "chase_dense_ms", "chase_aim_extrap", "handoff_k"});
    };

    // The WAVEFORM path (timed segments — MFP's Segments mode). Equally live.
    auto addSmWaveform = [&]() {
    c.addEntry({.id = ch::sm_waveform, .name = "vmotion-waveform",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = slopsync::ui_categories::tuning,
                .hasSettingChannel = true, .settingChannel = ch::sm_set,
                // Unlike its sm_limits/sm_chase siblings, none of these fields carry
                // setting_flags::advanced in code — rank matches that: control, not
                // advanced, so it stays visible without an advanced-affordance gate.
                .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addSelectField({.name = "curve_policy", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(0), .group = "Curve",
                      .desc = "Rebuild the sender's curve as sent, or force one smoothness family.",
                      .settingKey = 13, .hasSettingKey = true},
                     {"follow client", "force C1", "force C2"});
    // A SELECT'S WIRE VALUE IS ITS INDEX (SPEC, catalog.hpp addSelectField), so
    // this list is the stored ordinal: 0 = stretch, 1 = blend. The engine's own
    // enum keeps Blend = 5 and the host maps between them
    // (include/motion/EngineConfigMap.h), which is also what runs the four
    // ordinals of policies deleted 2026-09-02 as blend.
    c.addSelectField({.name = "infeasible_policy", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(0), .group = "Infeasible moves",
                      .desc = "What to do when a move cannot be finished in the time it was given.",
                      .settingKey = 14, .hasSettingKey = true},
                     {"stretch", "blend"});
    c.addLayoutField({.name = "smooth_budget", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofFloat(0.5f), .group = "Infeasible moves",
                      .desc = "How much smoothness may be spent before amplitude is touched.",
                      .step = 0.05f, .settingKey = 16, .hasSettingKey = true, .hasStep = true});
    c.addLayoutField({.name = "amplitude_budget", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofFloat(0.5f), .group = "Infeasible moves",
                      .desc = "How much stroke length may be spent before smoothness is touched.",
                      .step = 0.05f, .settingKey = 17, .hasSettingKey = true, .hasStep = true});
    c.addLayoutField({.name = "blend_steps", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 1.0f, .max = 10.0f,
                      .dflt = SettingDefault::ofInt(6), .group = "Infeasible moves",
                      .desc = "How gradually a budget is spent. More steps is smoother, slower to settle.",
                      .settingKey = 18, .hasSettingKey = true});
    // TODO(sd-6b2.4): `infeasible_blend` (SystemState::sm_tune_infeas_blend,
    // f32, 0..1, default 0.5, group "Infeasible moves") belongs here and in the
    // schema block below on the next free setting key. It is Blend's ONE
    // slider and Blend is the shipped policy, so it is the last unreachable
    // knob. Keys 4, 5, 15 and 19 are FREE but are not reused for it: released
    // keys stay released.
    c.addLayoutField({.name = "settle_grace_ms", .type = PackedFieldType::u32, .unit = "ms", .scale = 1000.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 200.0f,
                      .dflt = SettingDefault::ofFloat(30.0f), .group = "Settling",
                      .desc = "Grace period after a stream stops before the machine brakes to rest.",
                      .settingKey = 20, .hasSettingKey = true});
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Which of these the machine will accept right now.",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = slopsync::ui_ranks::detail},
                       {"curve_policy", "infeasible_policy", "smooth_budget",
                        "amplitude_budget", "blend_steps", "settle_grace_ms"});
    };

    // ---- "drive-tune" -- STATE, the AIM drive's own registers ---------------
    // Category `hardware`, not `tuning`: this writes the DRIVE, and its unit is
    // the drive's ((r/min)/s), not the machine's mm/s2. Filing it beside the
    // planner knobs would invite reading one as the other. Rank `control` and
    // no advanced flag, so it is reachable without an advanced affordance.
    //   [4 + 1 mask = 5 B]
    auto addDriveTune = [&]() {
    c.addEntry({.id = ch::drive_tune, .name = "drive-tune",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = slopsync::ui_categories::hardware,
                .hasSettingChannel = true, .settingChannel = ch::drive_set,
                .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addLayoutField({.name = "accel_reg", .type = PackedFieldType::u32, .unit = "rpm/s",
                      .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 60098.0f,
                      .dflt = SettingDefault::ofInt(0), .group = "Servo drive",
                      // 128 bytes exactly, which is limits::desc_max_bytes.
                      .desc = "Drive ramp register. 0 = auto. Under 60000 the drive ramps on "
                              "its own and lags; 60000 removes it; 60001-60098 add feedforward %.",
                      .step = 1.0f, .settingKey = 1,
                      .hasSettingKey = true, .hasStep = true});
    // READBACK, no setting_key, so these render as readouts. What the DRIVE
    // reports, never what was asked for: the request and the register can
    // disagree, and only the drive's own answer settles it.
    c.addLayoutField({.name = "accel_reg_actual", .type = PackedFieldType::u32, .unit = "rpm/s",
                      .scale = 1.0f, .group = "Servo drive",
                      .desc = "What the drive reports for 0x03 right now, read back off the bus."});
    c.addLayoutField({.name = "ramp_limit", .type = PackedFieldType::f32, .unit = "mm/s2",
                      .scale = 1.0f, .group = "Servo drive",
                      .desc = "That same setting in machine units. Below your accel ceiling the "
                              "drive, not the planner, is what limits the stroke."});
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Which of these the machine will accept right now.",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = slopsync::ui_ranks::detail},
                       {"accel_reg"});
    };

    // ---- "pattern-advanced" — STATE, normal, on-change ----------------------
    // Advanced mode's 8 BASE controls (advpat::Settings, everything except
    // the per-control cyclic Modifier — see 0x008F..0x0094 for those).
    // Replaces the ad-hoc JSON keys POST /api/pattern used to carry
    // (ap_mode/ap_speed/ap_max_depth/ap_min_depth/ap_in_speed/ap_out_speed/
    // ap_in_accel/ap_out_accel, undiscoverable by a generic client) with 8
    // settings a generic client renders without knowing this firmware
    // exists. That endpoint answers 410 today; this channel is what makes
    // Advanced mode reachable at all.
    //
    // SAME CATEGORY AS 0x0082 (`user`), DIFFERENT settingChannel (0x0107, not
    // 0x0102): Advanced is a separate sub-mode of the SAME pattern generator,
    // not a seventh classic-pattern field, so it earns its own writer while
    // sharing the category so both render as ONE tab (SPEC §8.8 — "a category
    // spans channels").
    //   [1*8 fields + 1 mask = 9 B]
    auto addPatternAdvanced = [&]() {
    c.addEntry({.id = ch::pattern_advanced, .name = "pattern-advanced",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = slopsync::ui_categories::control,
                .hasSettingChannel = true, .settingChannel = ch::pattern_advanced_cmd,
                .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addLayoutField({.name = "ap_mode", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofBool(false),
                      .group = "Advanced pattern",
                      .desc = "Drive the generator with Advanced mode instead of the classic patterns.",
                      .step = 1.0f, .settingKey = 1, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addLayoutField({.name = "master", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(0),
                      .group = "Advanced pattern",
                      .desc = "Overall stroke speed. 0 holds position.",
                      .step = 1.0f, .settingKey = 2, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
    c.addLayoutField({.name = "max_depth", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(10),
                      .group = "Depth window",
                      .desc = "Deepest point of the stroke (the in-stroke target).",
                      .step = 1.0f, .settingKey = 3, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
    c.addLayoutField({.name = "min_depth", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(0),
                      .group = "Depth window",
                      .desc = "Shallowest point of the stroke (the out-stroke target).",
                      .step = 1.0f, .settingKey = 4, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
    c.addLayoutField({.name = "in_speed", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 1.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(100),
                      .group = "Speed",
                      .desc = "In-stroke speed, as a percentage of master speed.",
                      .step = 1.0f, .settingKey = 5, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
    c.addLayoutField({.name = "out_speed", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 1.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(100),
                      .group = "Speed",
                      .desc = "Out-stroke speed, as a percentage of master speed.",
                      .step = 1.0f, .settingKey = 6, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
    c.addLayoutField({.name = "in_accel", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(40),
                      .group = "Acceleration",
                      .desc = "How hard the in-stroke accelerates.",
                      .step = 1.0f, .settingKey = 7, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
    c.addLayoutField({.name = "out_accel", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(40),
                      .group = "Acceleration",
                      .desc = "How hard the out-stroke accelerates.",
                      .step = 1.0f, .settingKey = 8, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = slopsync::ui_ranks::control,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
    // Bit i gates the i-th setting-annotated field above, same rule as 0x0082.
    // GENUINELY dynamic, and genuinely NARROWER than 0x0082's: unlike `running`
    // on 0x0102, none of these 8 setters is gated on `homed` (PatternEngine::
    // setAdvancedMode/setApMaster/setApBase have no homed check — only start()
    // does), so mirroring 0x0082's mask formula here would be a DISHONEST
    // refusal the delegate never actually makes. The mask therefore tracks
    // e-stop alone; the fields simply have no effect on the machine until it is
    // homed and running, same as dialing in a pattern before pressing start.
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f,
                        .desc = "Which of these the machine will accept right now.",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = slopsync::ui_ranks::detail},
                       {"ap_mode", "master", "max_depth", "min_depth", "in_speed", "out_speed",
                        "in_accel", "out_accel"});
    };

    // ---- "pattern-adv-mod-*" — STATE, background ----------------------------
    // The 6-field cyclic Modifier (advpat::Modifier) that rides EACH of the 6
    // base controls (advpat::BASE_COUNT): amplitude ramps a control's swing
    // in over `in_step` strokes, holds `in_wait`, ramps back over
    // `out_step`, rests `out_wait`, and `offset` phase-shifts the whole
    // cycle. ONE CHANNEL PER BASE CONTROL rather than binpacking 36 fields
    // into the fewest possible 8-field channels: each is a real, separate
    // concept (a different breathing pattern on depth vs. speed vs. accel),
    // same judgment 0x008B/C/D made splitting by subsystem, not by
    // bit-count.
    //
    // ALL SIX SHARE ch::pattern_advanced_cmd as settingChannel (same writer
    // as ch::pattern_advanced) and `user` as category, so all seven
    // advanced-pattern cards merge into ONE tab. setting_keys are allocated
    // 9..44 across the six, 6 keys apiece, matching the wire layout below
    // exactly: keyBase+0 amplitude, +1 in_step, +2 in_wait, +3 out_step,
    // +4 out_wait, +5 offset — the SAME formula as SlopSyncHubService's
    // applyIntent(ch::pattern_advanced_cmd) grouping (base = 9 +
    // 6*advpat::BaseId); the two must be kept in sync. `advanced`-flagged:
    // this is the deep-customization layer under the 8 base controls, not
    // the everyday knobs.
    //   [1*6 fields + 1 mask = 7 B, x6 channels]
    auto addApModifierChannel = [&](uint16_t id, const char* wireName, const char* group,
                                    uint8_t keyBase) {
        c.addEntry({.id = id, .name = wireName,
                    .cls = ChannelClass::STATE, .dir = Direction::h2c,
                    .access = AccessLevel::watch, .maxRateHz = 0.0f,
                    .defaultPriority = Priority::background,
                    .hasCategory = true, .category = slopsync::ui_categories::control,
                    .hasSettingChannel = true, .settingChannel = ch::pattern_advanced_cmd,
                    // rank = advanced at BOTH entry and field level — this whole
                    // channel IS the deep-customization layer under the 8 base
                    // controls (see the comment above), and every field it declares
                    // already carries setting_flags::advanced.
                    .hasRank = true, .rank = slopsync::ui_ranks::advanced});
        c.addLayoutField({.name = "amplitude", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                          .dflt = SettingDefault::ofInt(100), .group = group,
                          .desc = "Modulation strength; 100 = off.",
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 0),
                          .flags = slopsync::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = slopsync::ui_ranks::advanced,
                          .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
        c.addLayoutField({.name = "in_step", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 1.0f, .max = 25.0f,
                          .dflt = SettingDefault::ofInt(1), .group = group,
                          .desc = "Strokes ramping into the modulation.",
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 1),
                          .flags = slopsync::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = slopsync::ui_ranks::advanced});
        c.addLayoutField({.name = "in_wait", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = 25.0f,
                          .dflt = SettingDefault::ofInt(0), .group = group,
                          .desc = "Strokes held at full modulation.",
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 2),
                          .flags = slopsync::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = slopsync::ui_ranks::advanced});
        c.addLayoutField({.name = "out_step", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 1.0f, .max = 25.0f,
                          .dflt = SettingDefault::ofInt(1), .group = group,
                          .desc = "Strokes ramping back out.",
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 3),
                          .flags = slopsync::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = slopsync::ui_ranks::advanced});
        c.addLayoutField({.name = "out_wait", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = 25.0f,
                          .dflt = SettingDefault::ofInt(0), .group = group,
                          .desc = "Strokes resting before the cycle repeats.",
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 4),
                          .flags = slopsync::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = slopsync::ui_ranks::advanced});
        c.addLayoutField({.name = "offset", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                          .dflt = SettingDefault::ofInt(0), .group = group,
                          .desc = "Phase shift of the cycle.",
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 5),
                          .flags = slopsync::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = slopsync::ui_ranks::advanced,
                          .hasUnitId = true, .unitId = slopsync::unit_ids::percent});
        // Same honesty note as 0x008E: no setter here checks `homed` either
        // (setApModifier has no gate beyond the delegate's e-stop check), so
        // the mask tracks e-stop alone.
        c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                            .scale = 1.0f,
                            .desc = "Which of these the machine will accept right now.",
                            .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = slopsync::ui_ranks::detail},
                           {"amplitude", "in_step", "in_wait", "out_step", "out_wait", "offset"});
    };
    // The six invocations move to the final ascending-id call sequence below
    // — see the end of this function.

    // ---- "pattern-presets" — STORE, control ---------------------------------
    // RFC-021's `pattern.frayd` worked example, landed: retires the last HTTP
    // writer, POST /api/pattern/presets (NVS "advpreset", 24 x {name, def}
    // opaque JSON). `access = control` matches this store's CRUD writer
    // (0x0108) — same tier as every other pattern control, not `configure`
    // (unlike 0x000C paired-devices, this is not an admin/security surface).
    // storeId 2 (1 is the trust ledger, RFC-027/029) — self-describing, agreed
    // by being published here rather than legislated.
    //
    // perItemMax/nameMax are declared, not measured against the wire: the
    // payload is opaque device-defined bytes (in/out speed, in/out accel, six
    // modifier blocks — the SAME fields the retired handler's `def` carried,
    // "never depths or master speed"). See PatternPresetStore.h for the
    // 40-byte layout and SlopDriveHubDelegate::applyIntent's 0x0108 case for
    // the encode/decode.
    auto addPatternPresets = [&]() {
    c.addEntry({.id = ch::pattern_presets, .name = "pattern-presets",
                .cls = ChannelClass::STORE, .dir = Direction::h2c,
                .access = AccessLevel::control, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = slopsync::ui_categories::library,
                .hasRank = true, .rank = slopsync::ui_ranks::detail});
    c.addStoreDescriptor({.storeId = 2, .kind = "pattern.frayd",
                          .capacity = kPresetCapacity,
                          .perItemMax = kPresetPayloadBytes,
                          .nameMax = kPresetNameMax});
    };

    // ---- "pattern-presets-roster" — STATE, watch, on-change -----------------
    // {generation u16, count u8, capacity u8} — BARE, deliberately, same shape
    // as 0x000D paired-devices-roster. An embedded str16 name preview per slot
    // was the original plan (see PatternPresetStore.h's earlier revision) and
    // was cut for a real, measured reason, not a preference: Catalog32's
    // layout-field pool (channel/catalog.hpp, capacity 200) had only 11 free
    // slots left on this device before this channel existed (189/200 used),
    // and 3 header fields + 14 name fields needs 17. A client enumerates names
    // the same way it already does for the trust ledger: BLOB_REQ each slot
    // (kPayloadBytes is tiny — 40 B — so kPresetCapacity fetches is cheap) or
    // read the name back from a save/rename ECHO it sent itself. A generation
    // bump means "re-enumerate", exactly like 0x000D.
    auto addPatternPresetsRoster = [&]() {
    c.addEntry({.id = ch::pattern_presets_roster, .name = "pattern-presets-roster",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = slopsync::ui_categories::library,
                .hasSettingChannel = true, .settingChannel = ch::pattern_presets_cmd,
                .hasRank = true, .rank = slopsync::ui_ranks::detail});
    c.addLayoutField({.name = "generation", .type = PackedFieldType::u16, .unit = "count", .scale = 1.0f});
    c.addLayoutField({.name = "count",      .type = PackedFieldType::u8,  .unit = "count", .scale = 1.0f});
    c.addLayoutField({.name = "capacity",   .type = PackedFieldType::u8,  .unit = "count", .scale = 1.0f});
    };

    // ---- "move" — INTENT, control, 20 Hz, critical --------------------------
    // {1:"position" f32 mm, 2:"bypass" bool}. This channel maps to arbiter
    // source 0 (MANUAL) in the delegate.
    //
    // NO `action.*` ROLE HERE, DELIBERATELY — and that refusal became RFC-032:
    // RFC-019's action roles mark a schema field as a VERB ("do this");
    // `position` is a VALUE (where to go), and tagging it action.move would
    // tell a generic client to render a button where a slider belongs. The
    // honest annotation is the value-role `command.position`, which is what
    // lets ANY client (the rail widget's tap-to-move tape first among them)
    // find "put the carriage there" without hardcoding 0x0100.
    auto addMove = [&]() {
    c.addEntry({.id = ch::move, .name = "move",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 20.0f,
                .defaultPriority = Priority::critical,
                // THE primary positional command — the machine's face, same rank
                // as motion's own hero fields it commands.
                .hasCategory = true, .category = slopsync::ui_categories::control,
                .hasRank = true, .rank = slopsync::ui_ranks::hero});
    c.addSchemaField({.key = 1, .name = "position", .type = CborFieldType::f32_t, .unit = "mm",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 2000.0f,
                      .role = roles::command_position,
                      .hasRank = true, .rank = slopsync::ui_ranks::hero,
                      .hasUnitId = true, .unitId = slopsync::unit_ids::mm});
    c.addSchemaField({.key = 2, .name = "bypass", .type = CborFieldType::bool_t, .unit = ""});
    };

    // ---- "config-set" — INTENT, control, 10 Hz ------------------------------
    // Every field optional; present keys are applied. cfg_gen bumps on
    // success. Key 7 "input_jerk" and key 8 "max_rail" (below) are
    // append-only additions — keys 1-6 keep their meaning exactly, and
    // released key numbers are never reused.
    auto addConfigSet = [&]() {
    c.addEntry({.id = ch::config_set, .name = "config-set",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 10.0f,
                .defaultPriority = Priority::normal});
    //
    // Each key advertises the SAME bounds its 0x0081 twin does, so a client
    // that validates before sending gets the same answer the hub would NACK
    // with. The user-facing text (desc/group/default/role) lives ONCE, on
    // the STATE side — RFC-009 renders settings from the snapshot and resolves
    // the write key through `settingChannel`, so duplicating 128-byte descs
    // here would double the flash cost of every tooltip for no new meaning.
    c.addSchemaField({.key = 1, .name = "window_min",  .type = CborFieldType::f32_t, .unit = "mm",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::rail_mm});
    c.addSchemaField({.key = 2, .name = "window_max",  .type = CborFieldType::f32_t, .unit = "mm",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::rail_mm});
    c.addSchemaField({.key = 3, .name = "user_speed",  .type = CborFieldType::f32_t, .unit = "mm/s",
                      .hasMin = true, .hasMax = true, .min = ceiling::speed_min, .max = ceiling::speed_max});
    c.addSchemaField({.key = 4, .name = "user_accel",  .type = CborFieldType::f32_t, .unit = "mm/s2",
                      .hasMin = true, .hasMax = true, .min = ceiling::accel_min, .max = ceiling::accel_max});
    c.addSchemaField({.key = 5, .name = "input_speed", .type = CborFieldType::f32_t, .unit = "mm/s",
                      .hasMin = true, .hasMax = true, .min = ceiling::speed_min, .max = ceiling::speed_max});
    c.addSchemaField({.key = 6, .name = "input_accel", .type = CborFieldType::f32_t, .unit = "mm/s2",
                      .hasMin = true, .hasMax = true, .min = ceiling::accel_min, .max = ceiling::accel_max});
    c.addSchemaField({.key = 7, .name = "input_jerk",  .type = CborFieldType::f32_t, .unit = "mm/s3",
                      .hasMin = true, .hasMax = true, .min = ceiling::jerk_min, .max = ceiling::jerk_max});
    // key 8 "max_rail": promoted from read-only derived truth to a real
    // setting — see the field comment on 0x0081's `max_rail` for the full
    // rationale.
    c.addSchemaField({.key = 8, .name = "max_rail",    .type = CborFieldType::f32_t, .unit = "mm",
                      .hasMin = true, .hasMax = true, .min = ceiling::rail_min, .max = ceiling::rail_mm});
    };

    // ---- "pattern-cmd" — INTENT, control, 20 Hz -----------------------------
    // Session-volatile (cfg_gen does NOT bump). Maps to arbiter source 2
    // (PATTERN) via the delegate; running drives start/stop.
    auto addPatternCmd = [&]() {
    c.addEntry({.id = ch::pattern_cmd, .name = "pattern-cmd",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 20.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = slopsync::ui_categories::control,
                .hasRank = true, .rank = slopsync::ui_ranks::control});
    // Bounds mirror the 0x0082 twin (see the config-set note above for why the
    // prose lives only on the STATE side).
    c.addSchemaField({.key = 1, .name = "running",   .type = CborFieldType::bool_t, .unit = ""});
    c.addSchemaField({.key = 2, .name = "pattern",   .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 6.0f});
    c.addSchemaField({.key = 3, .name = "speed",     .type = CborFieldType::f32_t,  .unit = "%",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    c.addSchemaField({.key = 4, .name = "depth",     .type = CborFieldType::f32_t,  .unit = "%",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    c.addSchemaField({.key = 5, .name = "stroke",    .type = CborFieldType::f32_t,  .unit = "%",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    c.addSchemaField({.key = 6, .name = "sensation", .type = CborFieldType::f32_t,  .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    // key 7 `source.background_run`, appended, pairs 0x1200 pattern-state's
    // field of the same settingKey. Unlike keys 1-6, NOT session-volatile —
    // applyIntent persists it (coalesced, like max_rail) because it is a
    // standing policy, not a live pattern param.
    c.addSchemaField({.key = 7, .name = "background_run", .type = CborFieldType::bool_t, .unit = ""});
    };

    // ---- "home" — INTENT, control -------------------------------------------
    // {1:"op", 2:"stroke"} — op 1 starts sensorless homing; ops 2/3 are the
    // BENCH ops (RFC-025, safety-reviewed) that make motorless dev work
    // possible at all.
    //
    // *** OP 2 (force_home) CLEARS AN E-STOP LATCH. *** That is exactly why
    // RFC-025 placed these under safety review rather than in a convenience
    // bucket, and why they are `control` and rate-capped like every other op
    // here. Op 2 declares the machine homed WITHOUT a homing cycle, so the
    // stroke window it hands the arbiter is an ASSERTION, not a measurement —
    // on a machine with a motor attached that is a real collision hazard, and
    // the call site in SlopSyncHubService says so again.
    //
    // Op values are DEVICE-defined: 0x0103 is in this device's own >=0x0100
    // allocation, so unlike 0x0005's registry-governed `safety_ops` these
    // numbers live in this catalog and nowhere else — which is precisely why
    // they carry option labels, so a generic client can name them.
    auto addHome = [&]() {
    c.addEntry({.id = ch::home, .name = "home",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 5.0f,
                .defaultPriority = Priority::normal,
                // ui_categories::hardware's own note names "homing" explicitly.
                .hasCategory = true, .category = slopsync::ui_categories::hardware,
                .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addSelectSchemaField({.key = 1, .name = "op", .type = CborFieldType::uint_t, .unit = "",
                            .role = "action.home"},
                           {"reserved", "home", "force_home", "clear_override"},
                           {AccessLevel::control,   // 0 (placeholder, never an op)
                            AccessLevel::control,   // 1 home
                            AccessLevel::control,   // 2 force_home  — CLEARS THE E-STOP LATCH
                            AccessLevel::control}); // 3 clear_override
    c.addSchemaField({.key = 2, .name = "stroke", .type = CborFieldType::f32_t, .unit = "mm",
                      .hasMin = true, .hasMax = true, .min = 1.0f, .max = 2000.0f});
    };

    // ---- "modes-set" — INTENT, control, 5 Hz --------------------------------
    // The write half of 0x008A. Every key optional; present keys applied,
    // and the ECHO carries the POST-CLAMP value the handler actually took.
    //
    // NOT cfg_gen-bumping and NOT persisted here — each op routes to the same
    // WebUI::handleCommand path the legacy plane used, which owns whatever
    // persistence each mode has. Routing them anywhere else would give
    // SlopSync a second, divergent idea of what "blend mode" means.
    //
    // 5 Hz because these are human dropdown changes, not a control loop. The
    // bounds are the enum ranges the catalog's own option arrays declare, so a
    // client that validates locally gets the same answer the hub would NACK.
    auto addModesSet = [&]() {
    c.addEntry({.id = ch::modes_set, .name = "modes-set",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 5.0f,
                .defaultPriority = Priority::normal});
    // KEY 1 IS DELIBERATELY UNUSED. It held "blend_mode" until
    // MotionArbiter's alias-everything-to-"allow" behavior (see
    // setBlendMode()) made clear there was no live setting left to write —
    // see the field comment on 0x008A's `blend_mode_reserved`.
    // SlopSyncHubService.cpp's modes_set case no longer recognizes this key;
    // a client that still sends it gets NACK(INVALID_VALUE) same as any
    // other unrecognized key would.
    //
    // KEY 2 IS ALSO DELIBERATELY UNUSED. It briefly held "transport" (the WS/
    // SER/BT/DONGLE/OSSM input-source selector) before that setting was
    // retired: SlopSync is now the only way in, the hub listens on WebSocket
    // and BLE by default, and OSSM-BLE is gone. The C5 dongle may return one
    // day, but as a transport the hub simply HAS, not a mode an operator picks.
    //
    // KEY 3 IS NOW A PERMANENT GAP TOO. It held "stream_speed_mode" until the
    // motion port took the S3-side speed feed it chose between; see the field
    // comment on 0x008A's `stream_speed_reserved`. The modes_set case in
    // SlopSyncHubService.cpp no longer recognizes it, so a client that still
    // sends it gets NACK(INVALID_VALUE).
    //
    // All three numbers are skipped rather than recycled. This channel never
    // left the branch so reuse would technically be safe, but "released keys
    // are never reused" is only a reliable habit if it does not get
    // relitigated per case, and a gap costs nothing.
    c.addSchemaField({.key = 4, .name = "overshoot_clamp", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    c.addSchemaField({.key = 5, .name = "motion_backend", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    c.addSchemaField({.key = 6, .name = "home_style", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    };

    // ---- "vmotion-set" — INTENT, control, 5 Hz ---------------------------
    // The single writer behind all three vmotion-* cards. Keys 1..20 are
    // allocated across those cards and never collide; every key optional, only
    // the keys PRESENT are applied, and each echoes the value the machine
    // actually took after its own clamp. Keys 4, 5, 15 and 19 were RELEASED
    // 2026-09-02 with the knobs they wrote and are never reused.
    //
    // Bounds mirror the engine's own clamps exactly, so a client that validates
    // locally gets the same answer the hub would NACK with. Times are
    // MILLISECONDS on the wire; the engine stores microseconds.
    auto addSmSet = [&]() {
    c.addEntry({.id = ch::sm_set, .name = "vmotion-set",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 5.0f,
                .defaultPriority = Priority::normal});
    c.addSchemaField({.key = 1, .name = "jmax_ovr", .type = CborFieldType::f32_t, .unit = "1/s3",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 2000000.0f});
    c.addSchemaField({.key = 2, .name = "vmax_ovr", .type = CborFieldType::f32_t, .unit = "1/s",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 20.0f});
    c.addSchemaField({.key = 3, .name = "amax_ovr", .type = CborFieldType::f32_t, .unit = "1/s2",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 500.0f});
    c.addSchemaField({.key = 6, .name = "chase_ff", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    c.addSchemaField({.key = 7, .name = "chase_accel_ff", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    c.addSchemaField({.key = 8, .name = "chase_gain", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.5f});
    c.addSchemaField({.key = 9, .name = "chase_lookahead", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 8.0f});
    c.addSchemaField({.key = 10, .name = "chase_dense_ms", .type = CborFieldType::f32_t, .unit = "ms",
                      .hasMin = true, .hasMax = true, .min = 10.0f, .max = 500.0f});
    c.addSchemaField({.key = 11, .name = "chase_aim_extrap", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    c.addSchemaField({.key = 12, .name = "handoff_k", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 8.0f});
    c.addSchemaField({.key = 13, .name = "curve_policy", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 2.0f});
    c.addSchemaField({.key = 14, .name = "infeasible_policy", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    c.addSchemaField({.key = 16, .name = "smooth_budget", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    c.addSchemaField({.key = 17, .name = "amplitude_budget", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    c.addSchemaField({.key = 18, .name = "blend_steps", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 1.0f, .max = 10.0f});
    c.addSchemaField({.key = 20, .name = "settle_grace_ms", .type = CborFieldType::f32_t, .unit = "ms",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 200.0f});
    };

    // ---- "drive-set" -- INTENT, the writer behind drive-tune ----------------
    // 1 Hz, not 5: every accepted write reaches the drive over a shared 19200
    // Modbus wire that the setpoint stream also uses, and in step/dir mode it
    // costs a four-write enable/save/disable sequence. A human turning a knob
    // is the only intended caller. The hub refuses it outright while moving.
    auto addDriveSet = [&]() {
    c.addEntry({.id = ch::drive_set, .name = "drive-set",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 1.0f,
                .defaultPriority = Priority::normal});
    c.addSchemaField({.key = 1, .name = "accel_reg", .type = CborFieldType::uint_t,
                      .unit = "rpm/s",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 60098.0f});
    };

    // ---- "machine-admin" — INTENT, control ----------------------------------
    // The device ACTIONS that are not settings and not motion: clear a driver
    // fault, persist config, kick off a servo register scan. They were HTTP
    // writers (/api/clearfault, WS_OP_SAVE, POST /api/servo {"scan":true});
    // "no controls outside SlopSync" retires all three.
    //
    // An op SELECT rather than one channel per verb, exactly like 0x0103 home:
    // these are rare, human-initiated, and share a shape. 2 Hz because a human
    // presses them; a client that needs to press one faster than twice a second
    // is doing something the machine should not help with.
    auto addMachineAdmin = [&]() {
    c.addEntry({.id = ch::machine_admin, .name = "machine-admin",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 2.0f,
                .defaultPriority = Priority::normal,
                // clear_fault/servo_scan are hardware-adjacent (2 of 3 ops);
                // save_config rides along on the same rare-admin-action channel.
                .hasCategory = true, .category = slopsync::ui_categories::hardware,
                .hasRank = true, .rank = slopsync::ui_ranks::control});
    c.addSelectSchemaField({.key = 1, .name = "op", .type = CborFieldType::uint_t, .unit = "",
                            .role = "action.admin"},
                           {"reserved", "clear_fault", "save_config", "servo_scan"},
                           {AccessLevel::control,   // 0 placeholder, never an op
                            AccessLevel::control,   // 1 clear_fault
                            AccessLevel::control,   // 2 save_config
                            AccessLevel::control}); // 3 servo_scan
    };

    // ---- "pattern-advanced-cmd" — INTENT, control, 20 Hz --------------------
    // The single writer behind ALL SEVEN 0x008E..0x0094 advanced-pattern
    // cards. Same lean-schema convention as every other settings writer in
    // this catalog (config_set, pattern_cmd, modes_set, sm_set): the
    // user-facing text (desc/group/default/role) lives ONCE, on the STATE
    // side, so this channel carries only what a client needs to validate
    // before sending — name, type, unit, bounds.
    //
    // Keys 1..8 mirror 0x008E's layout exactly. Keys 9..44 are 6-per-control
    // blocks, base = 9 + 6*id with id in advpat::BaseId order (DEPTH_MAX=0
    // .. ACCEL_OUT=5), matching 0x008F..0x0094 exactly — see
    // SlopSyncHubService's applyIntent(0x0107) for the same arithmetic run
    // in reverse to decode a wire frame back into a control + sub-field.
    //
    // Session-volatile, same as 0x0102 pattern-cmd: cfg_gen does not bump.
    auto addPatternAdvancedCmd = [&]() {
    c.addEntry({.id = ch::pattern_advanced_cmd, .name = "pattern-advanced-cmd",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 20.0f,
                .defaultPriority = Priority::normal});
    c.addSchemaField({.key = 1, .name = "ap_mode",   .type = CborFieldType::bool_t, .unit = ""});
    c.addSchemaField({.key = 2, .name = "master",    .type = CborFieldType::uint_t, .unit = "%",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    c.addSchemaField({.key = 3, .name = "max_depth", .type = CborFieldType::uint_t, .unit = "%",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    c.addSchemaField({.key = 4, .name = "min_depth", .type = CborFieldType::uint_t, .unit = "%",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    c.addSchemaField({.key = 5, .name = "in_speed",  .type = CborFieldType::uint_t, .unit = "%",
                      .hasMin = true, .hasMax = true, .min = 1.0f, .max = 100.0f});
    c.addSchemaField({.key = 6, .name = "out_speed", .type = CborFieldType::uint_t, .unit = "%",
                      .hasMin = true, .hasMax = true, .min = 1.0f, .max = 100.0f});
    c.addSchemaField({.key = 7, .name = "in_accel",  .type = CborFieldType::uint_t, .unit = "%",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    c.addSchemaField({.key = 8, .name = "out_accel", .type = CborFieldType::uint_t, .unit = "%",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    // 6 keys per base control (advpat::BaseId order): amplitude, in_step,
    // in_wait, out_step, out_wait, offset — base = 9 + 6*id. Authoring order
    // doesn't need to be ascending here (the encoder sorts schema fields by
    // key before emitting), so a loop is safe where it would not be for the
    // addEntry() ordering above.
    for (uint8_t id = 0; id < kApBaseCount; ++id) {
        const uint8_t base = uint8_t(9 + 6 * id);
        c.addSchemaField({.key = uint8_t(base + 0), .name = "amplitude", .type = CborFieldType::uint_t,
                          .unit = "%", .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
        c.addSchemaField({.key = uint8_t(base + 1), .name = "in_step",   .type = CborFieldType::uint_t,
                          .unit = "", .hasMin = true, .hasMax = true, .min = 1.0f, .max = 25.0f});
        c.addSchemaField({.key = uint8_t(base + 2), .name = "in_wait",   .type = CborFieldType::uint_t,
                          .unit = "", .hasMin = true, .hasMax = true, .min = 0.0f, .max = 25.0f});
        c.addSchemaField({.key = uint8_t(base + 3), .name = "out_step",  .type = CborFieldType::uint_t,
                          .unit = "", .hasMin = true, .hasMax = true, .min = 1.0f, .max = 25.0f});
        c.addSchemaField({.key = uint8_t(base + 4), .name = "out_wait",  .type = CborFieldType::uint_t,
                          .unit = "", .hasMin = true, .hasMax = true, .min = 0.0f, .max = 25.0f});
        c.addSchemaField({.key = uint8_t(base + 5), .name = "offset",    .type = CborFieldType::uint_t,
                          .unit = "", .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    }
    };

    // ---- "pattern-presets-cmd" — INTENT, control ----------------------------
    // The CRUD writer behind the 0x0095 store / 0x0096 roster pair (RFC-021).
    // {1:"op", 2:"slot", 3:"name"}. `op` is RFC-019's OPEN `action.<name>`
    // convention (no registry change needed, same as 0x0005's action.safety):
    // index 0 is the mandatory non-empty placeholder, never a real op.
    //
    // `slot` addresses directly — the client picks it (normally the roster's
    // first free entry), there is no name-keyed dedup the way the retired
    // HTTP handler had. `name` is required for save/rename, ignored for
    // load/delete. See SlopDriveHubDelegate::applyIntent's 0x0108 case for
    // exactly what each op does and PatternPresetStore.h for the backend.
    auto addPatternPresetsCmd = [&]() {
    c.addEntry({.id = ch::pattern_presets_cmd, .name = "pattern-presets-cmd",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 5.0f,
                .defaultPriority = Priority::normal});
    c.addSelectSchemaField({.key = 1, .name = "op", .type = CborFieldType::uint_t, .unit = "",
                            .role = "action.preset"},
                           {"reserved", "save", "load", "delete", "rename"});
    c.addSchemaField({.key = 2, .name = "slot", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = float(kPresetCapacity - 1)});
    c.addSchemaField({.key = 3, .name = "name", .type = CborFieldType::tstr_t, .unit = ""});
    };

    // ---- invoke every device-channel builder above in ASCENDING NEW-ID ORDER --
    // The authoring order above does not match wire order, so each entry's
    // build logic is wrapped in a lambda (`add*`) and the calls below are the
    // one place that has to stay ascending (encodeCatalog/etag require it,
    // §8.3). Core channels (0x0003-0x000E) are unaffected: they were already
    // emitted above, in order, before any device channel. The six AP-modifier
    // calls are in MEMBER order (speedin/out, accelin/out, depth1/2), NOT
    // advpat::BaseId order — see the ch:: namespace comment on those
    // constants.
    //
    // `feat.has_motion` gates every call below that describes motion. The gate
    // is HERE and not inside each lambda so the surviving set is readable as a
    // list: an id that is not in this block's unconditional half does not exist
    // on a motionless hub, and its absence IS the capability answer (SPEC §6.3).
    // Ascending order is preserved either way -- skipping entries never
    // reorders the rest.
    addMachineConfig();          // 0x1000 STATE·machine, family 0 member 0
    addPower();                  // 0x1010 STATE·machine, family 1 member 0 (no-ops without feat.has_current_sensor)
    if (feat.has_motion) {
        addOdometer();           // 0x1020 STATE·machine, family 2 member 0
        addMachineModes();       // 0x1030 STATE·machine, family 3 member 0
        addMotion();             // 0x1100 STATE·motion, family 0 member 0
        addPlanStrip();          // 0x1110 STATE·motion, family 1 member 0
        addMotionDiag();         // 0x1111 STATE·motion, family 1 member 1
        addSmLimits();           // 0x1120 STATE·motion, family 2 member 0
        addSmChase();            // 0x1121 STATE·motion, family 2 member 1
        addSmWaveform();         // 0x1122 STATE·motion, family 2 member 2
        addDriveTune();          // 0x1130 STATE·motion, family 3 member 0
        addPatternState();       // 0x1200 STATE·pattern, family 0 member 0
        addPatternAdvanced();    // 0x1210 STATE·pattern, family 1 member 0
        addApModifierChannel(ch::pattern_adv_mod_speedin,  "pattern-adv-mod-speedin",  "Speed in modifier",  21);  // 0x1211
        addApModifierChannel(ch::pattern_adv_mod_speedout, "pattern-adv-mod-speedout", "Speed out modifier", 27);  // 0x1212
        addApModifierChannel(ch::pattern_adv_mod_accelin,  "pattern-adv-mod-accelin",  "Accel in modifier",  33);  // 0x1213
        addApModifierChannel(ch::pattern_adv_mod_accelout, "pattern-adv-mod-accelout", "Accel out modifier", 39);  // 0x1214
        addApModifierChannel(ch::pattern_adv_mod_depth1,   "pattern-adv-mod-depth1",   "Depth 1 modifier",   9);   // 0x1215
        addApModifierChannel(ch::pattern_adv_mod_depth2,   "pattern-adv-mod-depth2",   "Depth 2 modifier",   15);  // 0x1216
        addPatternPresetsRoster();  // 0x1220 STATE·pattern, family 2 member 0
        addMotionInput();        // 0x2100 STREAM·motion, family 0 member 0
        addMotionSegment();      // 0x2101 STREAM·motion, family 0 member 1
    }
    addConfigSet();              // 0x3000 INTENT·machine, family 0 member 0
    if (feat.has_motion) {
        addModesSet();           // 0x3030 INTENT·machine, family 3 member 0
        addMachineAdmin();       // 0x30F0 INTENT·machine, family F member 0
        addMove();               // 0x3100 INTENT·motion, family 0 member 0
        addHome();               // 0x3101 INTENT·motion, family 0 member 1
        addSmSet();              // 0x3120 INTENT·motion, family 2 member 0
        addDriveSet();           // 0x3130 INTENT·motion, family 3 member 0
        addPatternCmd();         // 0x3200 INTENT·pattern, family 0 member 0
        addPatternAdvancedCmd(); // 0x3210 INTENT·pattern, family 1 member 0
        addPatternPresetsCmd();  // 0x3220 INTENT·pattern, family 2 member 0
        addMotionAnomaly();      // 0x4100 EVENT·motion, family 0 member 0
        addPatternPresets();     // 0x5220 STORE·pattern, family 2 member 0
    }

    return c.ok();
}

}  // namespace valence
