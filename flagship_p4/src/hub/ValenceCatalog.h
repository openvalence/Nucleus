#pragma once

// ValenceCatalog -- builds this device's Valence channel catalog (SPEC §8.1).
//
// Constraints:
//   Hardware-free and library-only: nothing here may include an IDF or board
//   header. The constants it mirrors from valence_config.h are pinned by
//   static_asserts in ValenceHub.cpp, which is the one TU that sees both.
//   DeviceFeatures gates whole planes: has_motion the motion plane, has_drive
//   the servo drive's own surface, has_pattern the generator. A feature exists
//   IF AND ONLY IF its channels exist (SPEC §6.3): a hub advertising a channel
//   that publishes zeros is indistinguishable from an idle machine, the
//   dead-gauge lie has_current_sensor already exists to prevent.
//   Every field, unit, scale, and bit label below is wire-visible and part
//   of the client-invariant etag (SPEC §8.3) — author with the same care as
//   the FROZEN conformance fixture (conformance/mini_catalog.hpp).
//   Entries MUST be added in ASCENDING id order (etag is order-sensitive).
//   Layout entries (STATE/STREAM) must fit limits::min_transport_payload
//   (242 B) unfragmented; largest entry here is 88 B.
//   Names ≤32 B, field names ≤24 B, units ≤8 B (schema/catalog.cddl).
//   A fully-annotated entry must fit limits::catalog_max_entry_bytes (4096);
//   `desc` strings are the only unbounded cost and must stay tight.
//   `desc` reads like a Blender tooltip: one fragment, no period, no rationale.
//   0x0003/0x0004/0x0005/0x0007 layouts are pinned by the hub's own encoders
//   (buildSafetyPayload / buildControlOwnerPayload / handleIntent's
//   release path / emitTakeoverEvent) — not free to reshape here without
//   changing the hub in lockstep.
//   Wire sizes are noted per entry so a budget overrun is caught by eye;
//   there is no packed struct to static_assert against.
// See: Valence spec/AUTHORING.md ("you want X on screen -> you author Y" —
// read it BEFORE editing annotations here), SPEC.md and registry.yaml
// (Valence repo).

#include <array>
#include <cstdint>
#include <string_view>

#include "valence/channel/catalog.hpp"
#include "valence/channel/log_channel.hpp"
#include "valence/channel/safety_events_channel.hpp"
#include "valence/channel/settings_trial_channel.hpp"
#include "valence/channel/trust_channels.hpp"

#include "motion/ValenceMotion.h"

namespace valence {

// Device-catalog channel ids. The reserved 0x0001-0x0007 range is owned by
// the registry (valence::channels::); everything >=0x0080 is this device's
// own allocation. Named here so buildValenceCatalog() AND the publishers in
// ValenceDevice.cpp reference ONE definition — a literal in only one of the
// two would be a silent wire mismatch.
// Device ids follow the 0xCDSS grid: C=class (1 STATE/2 STREAM/3 INTENT/
// 4 EVENT/5 STORE), D=domain (0 machine/1 motion/2 pattern), S=family,
// S=member (member 0 = family master; a twin channel across class bands
// sharing domain+family+member is a MIRROR; family 0xF = admin/meta).
// Per-line `(was 0xXXXX)` names the immediately preceding id; older history
// lives in git, not here.
// A renumber moves the etag, which is the designed re-fetch mechanism, not a
// break; 0x0080-0x7FFF is device-allocated space per the registry.
namespace ch {
inline constexpr uint16_t motion           = 0x1100;  // STATE·motion, family 0 member 0 (master)
inline constexpr uint16_t machine_config   = 0x1000;  // STATE·machine, family 0 member 0 (master)
inline constexpr uint16_t pattern_state    = 0x1200;  // STATE·pattern, family 0 member 0 (master); background_run field rides here
inline constexpr uint16_t odometer         = 0x1020;  // STATE·machine, family 2 member 0 (was 0x1002)
inline constexpr uint16_t motion_input     = 0x2100;  // STREAM·motion, family 0 member 0 (master)
inline constexpr uint16_t motion_segment   = 0x2101;  // STREAM·motion, family 0 member 1
inline constexpr uint16_t osc_drive        = 0x2140;  // STREAM·motion, family 4 member 0, MIRROR of oscillator
// ---- telemetry channels the legacy :81 plane owned --------------------------
inline constexpr uint16_t plan_strip       = 0x1110;  // STATE·motion, family 1 member 0 (master; was 0x1101)
inline constexpr uint16_t power            = 0x1010;  // STATE·machine, family 1 member 0 (was 0x1001)
inline constexpr uint16_t motion_diag      = 0x1111;  // STATE·motion, family 1 member 1 (was 0x1102)
inline constexpr uint16_t motion_anomaly   = 0x4100;  // EVENT·motion, family 0 member 0 (master)
// ---- MODE settings the legacy :81/HTTP plane owned --------------------------
// A SECOND settings category, not more fields on 0x0081 — the reason is
// structural. 0x0081's `enabled_mask` is a bitfield8 whose bit i gates its
// i-th setting-annotated field, and seven of eight bits are already spoken
// for; widening the mask to fit these four would change an existing field's
// type, which is a protocol break, not append-only evolution. A settings
// category that outgrows its channel SPLITS into a new STATE+INTENT pair.
inline constexpr uint16_t machine_modes    = 0x1030;  // STATE·machine, family 3 member 0 (master; was 0x1003)
// ---- Kinetic live tuning, off HTTP and onto the protocol -----------------
// TWO state cards, ONE shared writer (kinetic_set). `settingChannel` is
// per-entry and `setting_key` is a key WITHIN that writer, so several STATE
// channels may name the same INTENT channel as long as their keys do not
// collide: one coherent write path however many cards the knobs need.
// 0x1121 (kinetic-chase) is RETIRED and its id is never re-used (SPEC 5.4,
// 4.4).
inline constexpr uint16_t kinetic_limits   = 0x1120;  // STATE·motion, family 2 member 0 (master; was 0x1103)
inline constexpr uint16_t kinetic_planner  = 0x1122;  // STATE·motion, family 2 member 2, kinetic-planner (was 0x1105)
// ---- Servo drive registers, its own family: these configure the DRIVE, not --
// the planner. Family 2 is kinetic's; a drive register that happens to be
// spelled "acceleration" is a different subsystem and gets its own writer.
inline constexpr uint16_t drive_tune       = 0x1130;  // STATE·motion, family 3 member 0 (master)
// ---- The oscillator (RFC-103, SPEC 9.7): its own family on the motion plane --
inline constexpr uint16_t oscillator       = 0x1140;  // STATE·motion, family 4 member 0 (master)
// ---- Advanced pattern — off the dead /api/pattern HTTP surface, onto Valence
// Same flattened-entry budget split as 0x008B/C/D. AdvancedPattern.h's real
// parameter set is the master knob, 8 base controls (advpat::BASE_COUNT) and
// a 6-field cyclic Modifier per base control, 58 settings with `running`. A
// fully-annotated 50-field entry encodes to ~8-10 KB (catalog_max_entry_bytes);
// fitting in kMaxFields (64) and being affordable in one entry are different
// constraints, so this splits by subsystem — one channel per base control's
// modifier (6 fields each) — same principle as the kinetic_limits /
// kinetic_planner split.
inline constexpr uint16_t pattern_advanced          = 0x1210;  // STATE·pattern, family 1 member 0 (master; was 0x1201) — 9 base controls + running
// The eight fray-d modulators (RFC-066): ONE family (domain=pattern, family=1),
// members 1-8. Member order is speed-in/out, accel-in/out, depth-1/2, then the
// two dwells (RFC-095) — NOT advpat::BaseId order (depth,depth,speedin,
// speedout,accelin,accelout,crest,trough) — see kModChannels in
// ValenceDevice.cpp, which maps between the two.
inline constexpr uint16_t pattern_adv_mod_speedin   = 0x1211;  // STATE·pattern, family 1 member 1 (was 0x1204)
inline constexpr uint16_t pattern_adv_mod_speedout  = 0x1212;  // STATE·pattern, family 1 member 2 (was 0x1205)
inline constexpr uint16_t pattern_adv_mod_accelin   = 0x1213;  // STATE·pattern, family 1 member 3 (was 0x1206)
inline constexpr uint16_t pattern_adv_mod_accelout  = 0x1214;  // STATE·pattern, family 1 member 4 (was 0x1207)
inline constexpr uint16_t pattern_adv_mod_depth1    = 0x1215;  // STATE·pattern, family 1 member 5 (was 0x1202) — advpat::DEPTH_MAX modifier
inline constexpr uint16_t pattern_adv_mod_depth2    = 0x1216;  // STATE·pattern, family 1 member 6 (was 0x1203) — advpat::DEPTH_MIN modifier
inline constexpr uint16_t pattern_adv_mod_crest     = 0x1217;  // STATE·pattern, family 1 member 7 — advpat::DWELL_CREST modifier (RFC-095)
inline constexpr uint16_t pattern_adv_mod_trough    = 0x1218;  // STATE·pattern, family 1 member 8 — advpat::DWELL_TROUGH modifier (RFC-095)
inline constexpr uint16_t move             = 0x3100;  // INTENT·motion, family 0 member 0 (master)
inline constexpr uint16_t config_set       = 0x3000;  // INTENT·machine, family 0 member 0 (master), MIRROR of machine_config
inline constexpr uint16_t pattern_cmd      = 0x3200;  // INTENT·pattern, family 0 member 0 (master), MIRROR of pattern_state
inline constexpr uint16_t home             = 0x3101;  // INTENT·motion, family 0 member 1
inline constexpr uint16_t modes_set        = 0x3030;  // INTENT·machine, family 3 member 0, MIRROR of machine_modes (was 0x3001)
inline constexpr uint16_t kinetic_set      = 0x3120;  // INTENT·motion, family 2 member 0, MIRROR of the kinetic_* family (was 0x3102)
inline constexpr uint16_t machine_admin    = 0x30F0;  // INTENT·machine, family F member 0 = admin (was 0x3002)
inline constexpr uint16_t drive_set        = 0x3130;  // INTENT·motion, family 3 member 0, MIRROR of drive_tune
inline constexpr uint16_t osc_set          = 0x3140;  // INTENT·motion, family 4 member 0, MIRROR of oscillator
// Shared writer behind ALL NINE pattern-advanced STATE channels — same
// "one settingChannel, many cards" pattern as kinetic_set. MIRROR of
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
// (flagship_p4/src/patterns/PatternPresetStore.h), same forced-duplication
// rule as `factory`/`ceiling` below: this header stays library-only.
// ValenceDevice.cpp sees both and static_asserts them together, so drift
// fails the build, not the wire.
inline constexpr uint8_t kPresetCapacity = 24;
inline constexpr uint8_t kPresetNameMax = 32;
inline constexpr uint8_t kPresetPayloadBytes = 56;
// The preset store's identity, published in its STORE descriptor, carried as
// store_id (RFC-070) by its roster and writer, and answered by
// ValenceDevice::readBlob(). store_id 1 is the trust ledger.
inline constexpr uint8_t kPresetStoreId = 2;
inline constexpr const char* kPresetKind = "pattern.frayd";

// MIRROR of advpat::BASE_COUNT and advpat::PERCENT_BASE_COUNT
// (flagship_p4/src/patterns/advanced/AdvancedPattern.h), same rule and the same
// static_assert home as the preset mirror above.
inline constexpr uint8_t kApBaseCount = 8;
inline constexpr uint8_t kApPercentBaseCount = 6;

// Where base control `base` (advpat::BaseId) lives on the wire. The six
// percent knobs sit before 0x1210's first enabled_mask and `running`; the two
// dwells (RFC-095) are appended after them at the tail (SPEC 5.4), so every
// older offset and key holds. The catalog builders below and
// ValenceDevice::applyPatternAdvanced both read these; nothing restates them.
// 0x1210 layout index of the base control.
constexpr uint16_t apBaseLayoutIndex(uint8_t base) {
    return base < kApPercentBaseCount ? uint16_t(2 + base) : uint16_t(10 + (base - kApPercentBaseCount));
}
// 0x3210 key writing the base control.
constexpr uint8_t apBaseKey(uint8_t base) {
    return base < kApPercentBaseCount ? uint8_t(3 + base) : uint8_t(46 + (base - kApPercentBaseCount));
}
// 0x3210 key of the base control's modulator's first field; six in a row.
constexpr uint8_t apModKeyBase(uint8_t base) {
    return base < kApPercentBaseCount ? uint8_t(9 + 6 * base)
                                      : uint8_t(48 + 6 * (base - kApPercentBaseCount));
}
inline constexpr uint8_t kApRunKey = 45;
inline constexpr uint8_t kApLastKey = uint8_t(apModKeyBase(kApBaseCount - 1) + 5);

// plan-strip `style` option for a hold: a timed plan whose start is its end
// (SPEC 9.6 hold segment, an RFC-095 dwell among them). Indices below it are
// the planner's styles (MotionArbiter.cpp PlanStyle).
inline constexpr uint8_t kPlanStyleHold = 4;

// MIRROR of kMotionSourceNames (flagship_p4/src/motion/ValenceMotion.h),
// indexed by source id: control-owner's option labels. Same rule and the
// same static_assert home as the preset mirror above.
inline constexpr std::array<const char*, 4> kSourceLabels{"Jog", "Stream", "Classic", "Advanced"};

// Card headings under the subgroups the folded categories became (RFC-094,
// RENDERING §3). "<subgroup> / <card>": the first " / " names the section the
// card sits under (RFC-096, RENDERING 3; the registry pins the spelling as
// limits::group_section_separator; on the wire it stays one free-text
// `group`, SPEC §8.8). Every sectioned heading lives here, checked against
// the registry below. Every drawn field of a
// `motion` entry that was `tuning`, and of a `system` entry that was
// `library`, carries one. Masks and retired padding are never drawn and stay
// ungrouped.
namespace card {
inline constexpr std::string_view active_plan      = "Tuning / Active plan";
inline constexpr std::string_view planner          = "Tuning / Planner";
inline constexpr std::string_view anomalies        = "Tuning / Anomalies";
inline constexpr std::string_view plan_time        = "Tuning / Plan time";
inline constexpr std::string_view stream_ingress   = "Tuning / Stream ingress";
inline constexpr std::string_view motion_behavior  = "Tuning / Motion behavior";
inline constexpr std::string_view streaming        = "Tuning / Streaming";
inline constexpr std::string_view sample_streams   = "Tuning / Sample streams";
inline constexpr std::string_view curve            = "Tuning / Curve";
inline constexpr std::string_view ceilings         = "Tuning / Ceilings";
inline constexpr std::string_view replanning       = "Tuning / Re-planning";
inline constexpr std::string_view pattern_presets  = "Library / Pattern presets";
inline constexpr std::string_view safety           = "Tuning / Safety";
}  // namespace card

// RFC-096: true when a group string has no section, or its first '/' sits in
// the registry's separator, spelled exactly.
constexpr bool sectionedByRegistry(std::string_view group) {
    const size_t at = group.find('/');
    if (at == std::string_view::npos) return true;
    constexpr std::string_view sep = valence::limits::group_section_separator;
    const size_t lead = sep.find('/');
    return at >= lead && group.substr(at - lead, sep.size()) == sep;
}
static_assert([] {
    for (const std::string_view g : {card::active_plan, card::planner, card::anomalies, card::plan_time,
                                     card::stream_ingress, card::motion_behavior, card::streaming,
                                     card::sample_streams, card::curve, card::ceilings, card::replanning,
                                     card::pattern_presets, card::safety})
        if (!sectionedByRegistry(g)) return false;
    return true;
}(), "a card heading's section separator is the registry's (RFC-096)");

// ---- motion-anomaly EVENT: the `body` (40) sub-map keys ---------------------
// These are the CHANNEL'S OWN schema keys, exactly as valence::safety_body is
// for 0x000E — that is the v1.0 EVENT grammar (registry key 40's own note: with
// kind-specific fields at the TOP level, every device-authored EVENT channel
// would need a registry PR to name its own fields). This channel is the first
// DEVICE-authored EVENT channel in the ecosystem and therefore the proof that
// the grammar fix works: nothing below required a registry change.
namespace anom_body {
inline constexpr uint8_t kind   = 1;  // kinetic2::AnomalyKind, mirrors event_kind; labels live in event_kinds
inline constexpr uint8_t seq    = 2;  // engine's rolling event id (wraps)
inline constexpr uint8_t target = 3;  // the command target that provoked it, 0..1 normalized
inline constexpr uint8_t detail = 4;  // KIND-SPECIFIC scalar — see the option labels
inline constexpr uint8_t t_us   = 5;  // engine time at record, µs (low 32 bits)
}  // namespace anom_body

// ---- Machine FEATURES that gate whether a channel is advertised AT ALL ------
// RFC-016 in practice: "capability discovery IS catalog introspection". A hub
// with no INA228 must not advertise a power channel that would publish zeros
// forever — a client cannot tell "0.0 A" from "no sensor", and a UI that shows
// a dead gauge violates the ground-truth doctrine. So the channel is ABSENT,
// and its absence IS the answer to "does this machine measure current?".
//
// Defaults are all-false so the HOST tests and any future non-motorized build
// get the minimal catalog unless they say otherwise; the board's values come
// from boardFeatures() (ValenceDevice.h).
struct DeviceFeatures {
    bool has_current_sensor = false;  // bus current measured
    bool has_power_monitor  = false;  // power monitor die temperature
    // Gates the MOTION PLANE: the motion/plan/diag/tuning STATE channels, both
    // c2h motion streams, move/home, the anomaly event, and the two
    // machine-domain channels whose content is motion (odometer 0x1020,
    // machine-modes 0x1030 with its writer 0x3030).
    // machine-config (0x1000) and config-set (0x3000) SURVIVE it: they are
    // configuration STORAGE, and a stored limit is truthfully what the hub
    // holds, not a gauge reading zero forever.
    bool has_motion         = false;
    // Gates the SERVO DRIVE's own surface: drive-tune (0x1130), its writer
    // drive-set (0x3130), and machine-admin (0x30F0, whose ops are
    // clear_fault / servo_scan on that drive). A machine can plan and render
    // motion with no programmable drive on a bus, which is why this is its own
    // flag and not a corner of has_motion.
    bool has_drive          = false;
    // Gates the PATTERN GENERATOR: every pattern-* channel and the 0x5220
    // preset store. A hub with no generator advertising pattern-state would
    // publish a permanently stopped generator, which is the dead-gauge lie.
    bool has_pattern        = false;
};

// ---- Factory DEFAULTS advertised as RFC-009 `default` annotations -----------
// MIRROR of the DEFAULT_* constants in valence_config.h, which this header
// does not include: it must stay hardware-free (the native suites and the sim
// build it with nothing but the library). ValenceHub.cpp static_asserts each
// mirrored constant against its source, so a drifted default fails the
// firmware build until this table follows.
namespace factory {
inline constexpr float window_min  = 0.0f;
inline constexpr float window_max  = 500.0f;      // DEFAULT_MAX_RAIL_MM
inline constexpr float jog_speed  = 50.0f;       // DEFAULT_JOG_MAX_SPEED_MM_S
inline constexpr float jog_accel  = 200.0f;      // DEFAULT_JOG_ACCEL_MM_S2
inline constexpr float input_speed = 1200.0f;     // DEFAULT_MAX_SPEED_MM_S
inline constexpr float input_accel = 100000.0f;   // DEFAULT_ACCEL_MM_S2
inline constexpr float input_jerk  = 20000000.0f; // DEFAULT_INPUT_MAX_JERK_MM_S3
// max_rail is a real savable setting, not derived truth — see the field
// comment on 0x0081 below. Same mirror rule as its siblings above.
inline constexpr float max_rail    = 500.0f;      // DEFAULT_MAX_RAIL_MM
inline constexpr float home_speed  = 40.0f;       // DEFAULT_HOME_SPEED_MM_S
// Mode defaults (ch::machine_modes). `blend_mode`, `stream_speed_mode` and
// `overshoot_clamp` have no `.dflt` here: their settings are retired bytes
// (see the field comments there). Do not re-add one without re-adding the
// field's setting_key first.
// Kinetic² planner options, MIRRORS of kinetic2::Config's defaults
// (static_asserted in ValenceHub.cpp). Times in the wire's milliseconds.
inline constexpr float   smoothness        = 0.0f;   // Config::smoothness
inline constexpr float   handle_floor      = 0.15f;  // Config::handle_floor
inline constexpr float   trim_max          = 1.0f;   // Config::trim_max
inline constexpr float   react_ms          = 4.0f;   // Config::react_us
}  // namespace factory

// ---- The schedule horizon (RFC-087, SPEC 5.4) ---------------------------------
// The three steps a segments grant may advertise, indexed by 0x1030's
// schedule_horizon ordinal. 500 has no registry name; SPEC 5.4 lists it.
inline constexpr std::array<uint16_t, 3> kHorizonMs{
    uint16_t(limits::max_future_schedule_ms), 500, uint16_t(limits::schedule_horizon_max_ms)};

// ---- Hard firmware ceilings advertised as `min`/`max` -----------------------
// The bounds the hub enforces (StoredState.h configValid, ValenceDevice.cpp's
// window clamp), never a client's guardrails: a static catalog advertises what
// the hub will really accept. ValenceHub.cpp static_asserts the speed, accel
// and jerk maxima against valence_config.h.
namespace ceiling {
inline constexpr float rail_mm    = 2000.0f;      // configValid's max_rail bound
inline constexpr float rail_min   = 10.0f;        // configValid's max_rail floor, MIN_RAIL_MM
inline constexpr float speed_min  = 1.0f;
inline constexpr float speed_max  = 2000.0f;      // MAX_SPEED_MM_S, the LP emitter's measured rate
inline constexpr float accel_min  = 10.0f;
inline constexpr float accel_max  = 100000.0f;    // MAX_ACCEL_MM_S2
inline constexpr float jerk_min   = 1000.0f;
inline constexpr float jerk_max   = 50000000.0f;  // MAX_JERK_MM_S3
inline constexpr float home_speed_min = 5.0f;     // MIN_HOME_SPEED_MM_S; its max is speed_max
inline constexpr float osc_max_hz = 100.0f;       // OSC_MAX_HZ, WELCOME limits osc_max_hz
inline constexpr float osc_dwell_max = 4.0f;      // the longest hold, four moving cycles
}  // namespace ceiling

// The two driven oscillator parameters (SPEC 9.7) as 0x3140 keys them, from
// key0: drive, in_min, in_max, out_min, out_max; 0x1140 mirrors them.
struct OscDriveCard {
    uint8_t key0;
    std::string_view drive, in_min, in_max, out_min, out_max, unit;
    float top;
    std::string_view drive_desc, out_min_desc, out_max_desc;
    std::array<std::string_view, 5> roles;
};
inline constexpr std::array<OscDriveCard, 2> kOscDriveCards{{
    {7, "frequency_drive", "frequency_in_min", "frequency_in_max", "frequency_out_min", "frequency_out_max", "Hz",
     ceiling::osc_max_hz, "What sets the frequency", "Frequency at the low input", "Frequency at the high input",
     {valence::field_roles::osc_frequency_drive, valence::field_roles::osc_frequency_in_min,
      valence::field_roles::osc_frequency_in_max, valence::field_roles::osc_frequency_out_min,
      valence::field_roles::osc_frequency_out_max}},
    {12, "amplitude_drive", "amplitude_in_min", "amplitude_in_max", "amplitude_out_min", "amplitude_out_max", "",
     1.0f, "What sets the amplitude", "Amplitude at the low input", "Amplitude at the high input",
     {valence::field_roles::osc_amplitude_drive, valence::field_roles::osc_amplitude_in_min,
      valence::field_roles::osc_amplitude_in_max, valence::field_roles::osc_amplitude_out_min,
      valence::field_roles::osc_amplitude_out_max}},
}};

// Fills `c` with this device's catalog. OUT-PARAM, never a return value: a
// Catalog32 is ~22 KB of pooled field storage, so returning one by value
// would put that on the caller's stack — the bug class that has already blown
// a FreeRTOS task stack on this firmware once. Returns c.ok(): false means a
// capacity in Catalog32 was exceeded while building (entries or field pools),
// which is a build-time authoring error, not a runtime condition.
//
// `feat` gates the FEATURE-DEPENDENT entries (see DeviceFeatures). It defaults
// to all-false, so an existing call site keeps the minimal catalog.
inline bool buildValenceCatalog(valence::Catalog32& c, DeviceFeatures feat = {}) {
    using valence::AccessLevel;
    using valence::CborFieldType;
    using valence::ChannelClass;
    using valence::Direction;
    using valence::PackedFieldType;
    using valence::Priority;
    using valence::SettingDefault;
    namespace roles = valence::field_roles;

    c.clear();

    // ---- "safety" — STATE, critical, on-change ------------------------------
    // VERBATIM copy of conformance/mini_catalog.hpp's safety entry: the hub's
    // buildSafetyPayload() hardcodes exactly this 9-byte layout (word
    // bitfield8, cause u8, owner_session u32, estop_seq u16, modes bitfield8).
    // Do NOT reshape it.  [9 B]
    //
    // `word` bits 1 and 2 (STOP, HOLD) are retired and always zero (RFC-085);
    // their labels say so, because a label is all a generic client can show.
    // `modes` (override + home_required, RFC-085): latched MODES, not stop
    // edges. override is written by the 0x0005 override/return pair;
    // home_required by the hub on a power-cutting ESTOP, cleared by a
    // completed home. Append-only: bytes 0..7 keep their meaning and offsets.
    c.addEntry({.id = valence::channels::safety, .name = "safety",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::critical});
    c.addBitfieldField({.name = "word", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f},
                       {"estop", "retired", "retired", "pause"});
    c.addLayoutField({.name = "cause", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "owner_session", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "estop_seq", .type = PackedFieldType::u16, .unit = "count", .scale = 1.0f});
    c.addBitfieldField({.name = "modes", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f},
                       {"override", "home_required"});

    // ---- "control-owner" — STATE, critical, on-change -----------------------
    // Matches Hub::buildControlOwnerPayload(): 4 × {source u8, owner u32}, in
    // ascending source order. Each pair is one arbiter source and the session
    // id that owns it (0 = unowned). The src options name each source id;
    // index 0 is the jog, a real source, not a none label. Then per slot
    // (RFC-098) the source_kinds value and the owner's HELLO client_kind and
    // client_name, zero-filled when unowned.  [216 B]
    c.addEntry({.id = valence::channels::control_owner, .name = "control-owner",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::critical});
    c.addSelectField({.name = "src0",   .type = PackedFieldType::u8,  .unit = "", .scale = 1.0f},
                     {kSourceLabels[0], kSourceLabels[1], kSourceLabels[2], kSourceLabels[3]});
    c.addLayoutField({.name = "owner0", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});
    c.addSelectField({.name = "src1",   .type = PackedFieldType::u8,  .unit = "", .scale = 1.0f},
                     {kSourceLabels[0], kSourceLabels[1], kSourceLabels[2], kSourceLabels[3]});
    c.addLayoutField({.name = "owner1", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});
    c.addSelectField({.name = "src2",   .type = PackedFieldType::u8,  .unit = "", .scale = 1.0f},
                     {kSourceLabels[0], kSourceLabels[1], kSourceLabels[2], kSourceLabels[3]});
    c.addLayoutField({.name = "owner2", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});
    c.addSelectField({.name = "src3",   .type = PackedFieldType::u8,  .unit = "", .scale = 1.0f},
                     {kSourceLabels[0], kSourceLabels[1], kSourceLabels[2], kSourceLabels[3]});
    c.addLayoutField({.name = "owner3", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "kind0", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "client_kind0", .type = PackedFieldType::str16, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "client_name0", .type = PackedFieldType::str32, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "kind1", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "client_kind1", .type = PackedFieldType::str16, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "client_name1", .type = PackedFieldType::str32, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "kind2", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "client_kind2", .type = PackedFieldType::str16, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "client_name2", .type = PackedFieldType::str32, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "kind3", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "client_kind3", .type = PackedFieldType::str16, .unit = "", .scale = 1.0f});
    c.addLayoutField({.name = "client_name3", .type = PackedFieldType::str32, .unit = "", .scale = 1.0f});

    // ---- "safety-intents" — INTENT, critical, modest rate -------------------
    // The client sends {1:"op"} where op is a safety_ops:: value (estop=6 and
    // release=1 are hub-handled; the rest reach the delegate and the hub
    // latches the result — RFC-025a, RFC-085).
    //
    // *** THE ACCESS FLOOR IS `watch`, AND THAT IS THE POINT (RFC-025b). ***
    // `estop` and `pause` are ROLE-EXEMPT: anyone connected — including a
    // watch-only session — may stop this machine. §11.2's "safety outranks
    // authorization", generalized. The failure mode of getting this backwards
    // is "the person standing in the room cannot stop the machine", which is
    // not a permissions bug, it is an injury. Loop-stop spam by a viewer is
    // bounded by the §9.3 intent rate limiter above (20 Hz here) and is a
    // named, accepted risk in §12.1.
    //
    // Everything else — release/resume/override/return — requires
    // `control`, expressed as index-aligned `option_access` (catalog
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
    // UNSUPPORTED_OP at the delegate regardless. The four numbers RFC-085
    // retired (2 stop, 3 hold, 9 and 10 the bypass pair) keep their index as
    // "retired" at `control` and NACK UNSUPPORTED_OP the same way.
    c.addEntry({.id = valence::channels::safety_intents, .name = "safety-intents",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::watch, .maxRateHz = 20.0f,
                .defaultPriority = Priority::critical});
    c.addSelectSchemaField({.key = 1, .name = "op", .type = CborFieldType::uint_t, .unit = "",
                            .role = "action.safety"},
                           {"reserved", "release", "retired", "retired", "pause", "resume",
                            "estop", "override", "return", "retired", "retired"},
                           {AccessLevel::control,  // 0  (placeholder, never an op)
                            AccessLevel::control,  // 1  release
                            AccessLevel::control,  // 2  retired (stop)
                            AccessLevel::control,  // 3  retired (hold)
                            AccessLevel::watch,    // 4  pause         ROLE-EXEMPT
                            AccessLevel::control,  // 5  resume
                            AccessLevel::watch,    // 6  estop         ROLE-EXEMPT
                            AccessLevel::control,  // 7  override
                            AccessLevel::control,  // 8  return
                            AccessLevel::control,  // 9  retired (bypass_on)
                            AccessLevel::control});// 10 retired (bypass_off)

    // ---- "hub-status" — STATE, background, 1 Hz -----------------------------
    // Slow health telemetry.  [4+4+1+1+4+1+1 = 16 B]
    c.addEntry({.id = valence::channels::hub_status, .name = "hub-status",
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
                      .desc = "Log lines dropped since boot"});
    // `motor_switch` and `motor_fault` (fields 6-7, appended 14 -> 16 B): the
    // motor switch's state and the reason it last latched faulted. Their
    // ordinals are motorswitch::State and motorswitch::Fault
    // (system/MotorSwitch.h): the labels move with those enums, append-only.
    // A machine with no switch publishes `on` and `none` (the sim twin).
    // motor_fault keeps reporting the last fault after a recovery.
    c.addSelectField({.name = "motor_switch", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .desc = "Motor power switch state"},
                     {"off", "precharging", "on", "faulted"});
    c.addSelectField({.name = "motor_fault", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .desc = "Reason the motor switch last latched off"},
                     {"none", "switch_fault", "en_node", "inrush", "precharge"});

    // ---- "session-events" — EVENT, watch ------------------------------------
    // Payload keys match Hub::emitTakeoverEvent(): {1:"source", 2:"session"}.
    c.addEntry({.id = valence::channels::session_events, .name = "session-events",
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
    // Declaring it is what makes the Geiger bridge REACHABLE: publishLog()
    // returns false on a hub whose catalog has no 0x0008, so without this line
    // the whole bridge is a no-op. The replay depth is the registry default
    // (limits::log_replay_depth_default = 32) — a client that connects AFTER a
    // fault still sees what happened, which is the one sanctioned exception to
    // §9.4's no-replay rule.
    if (!valence::addLogChannel(c)) return false;

    // ---- the TRUST ADMINISTRATION surface -----------------------------------
    // session-admin, pending-pairing, pairing-events, and the paired-devices
    // store + its roster, all in the canonical shapes the library declares
    // (lib/valence/include/valence/channel/trust_channels.hpp). Declared as a
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
    if (!valence::addTrustChannels(c)) return false;

    // ---- "safety-events" — EVENT, critical, watch ---------------------------
    // The §9.4 EVENT TWIN of the safety latch (0x0003). §5.5/§11.2 require the
    // hub to emit it. Same access and priority as its STATE twin: an edge
    // nobody may be denied and nobody's may be shed.
    if (!valence::addSafetyEventsChannel(c)) return false;

    // ---- "settings-trial" -- INTENT, control (RFC-099) ----------------------
    // Declaring it is what tells a client this hub keeps a `trial` write
    // unpersisted; the hub answers commit and revert itself. Each settings
    // card below carries a trial_mask (meta.trial_pending), bit i the i-th
    // setting of its layout, as enabled_mask indexes it.
    if (!valence::addSettingsTrialChannel(c)) return false;

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
                .hasCategory = true, .category = valence::ui_categories::motion,
                .hasRank = true, .rank = valence::ui_ranks::hero});
    // PLANNED, not actual: the LP core's RENDERED position, which IS the
    // machine's position truth (.claude/rules/architecture.md section 2).
    // The drive encoder is its AUDITOR and
    // reaches clients through the encoder-deviation channel, not this field.
    c.addLayoutField({.name = "pos_10um", .type = PackedFieldType::u16, .unit = "mm",   .scale = 100.0f,
                      .desc = "Carriage position as rendered",
                      .role = roles::telemetry_position,
                      .hasRank = true, .rank = valence::ui_ranks::hero,
                      .hasProvenance = true, .provenance = valence::value_provenance::planned,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm});
    c.addLayoutField({.name = "tgt_10um", .type = PackedFieldType::u16, .unit = "mm",   .scale = 100.0f,
                      .desc = "Position the planner is driving to",
                      .role = roles::telemetry_target,
                      .hasRank = true, .rank = valence::ui_ranks::hero,
                      .hasProvenance = true, .provenance = valence::value_provenance::planned,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm});
    c.addLayoutField({.name = "speed",    .type = PackedFieldType::i16, .unit = "mm/s", .scale = 10.0f,
                      .desc = "Carriage speed, signed by direction",
                      .role = roles::telemetry_velocity,
                      .hasRank = true, .rank = valence::ui_ranks::hero,
                      .hasProvenance = true, .provenance = valence::value_provenance::actual,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm_s});
    c.addBitfieldField({.name = "flags", .type = PackedFieldType::bitfield8, .unit = "flag", .scale = 1.0f,
                        .desc = "Live machine mode bits",
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"homed", "homing", "gen_running", "paused", "override", "estop", "stream"});
    c.addLayoutField({.name = "raw_10um", .type = PackedFieldType::u16, .unit = "mm",   .scale = 100.0f,
                      // No registry role fits a demand-provenance position on a
                      // STATE channel (command.position is an INTENT role), so
                      // this desc's LEADING CLAUSE is the field's human label:
                      // clients read it instead of the wire name (Phosphor
                      // src/model/format.js labelFor).
                      .desc = "Asked position, mapped into the window, before planning",
                      .hasRank = true, .rank = valence::ui_ranks::diagnostic,
                      .hasProvenance = true, .provenance = valence::value_provenance::demand,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm});
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
    // USER-FACING `desc`, and a registry `role`. `settingChannel` = 0x3000, so
    // a generic client renders a full settings page from the catalog alone.
    //
    // Category setup (RFC-079): the machine's geometry and ceilings are the
    // owner's commissioning, entered through the setup wizard, never firmware
    // knowledge. Categories are static per entry, so it is setup in every mode;
    // the wizard steps the setup entries in authoring (ascending id) order,
    // this one before the kinetic ceilings on 0x1120.
    //
    // `max_rail` is a REAL SAVABLE SETTING: the usable rail length, stop to
    // stop minus a safety margin at each end, and the home cycle's search
    // distance per leg (plus both margins and a search margin, MotionArbiter.h
    // homing). Before the first home an owner sets it at or above the real
    // rail; a completed home writes the usable length it measured into it
    // through config-set key 8's
    // own writer (operator ruling 2026-10-03, Valence RFC-101), so after a
    // home the setting IS the measurement. While that home stands, max_rail
    // and the window clamp to the measurement (ValenceDevice::applyConfig(),
    // bd val-3kd); unhomed, max_rail can be raised for a longer rail's search.
    // `measured_stroke` (field 10, below)
    // is this boot's raw measurement, read-only; a client never writes one
    // from the other.
    auto addMachineConfig = [&]() {
    c.addEntry({.id = ch::machine_config, .name = "machine-config",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = valence::ui_categories::setup,
                .hasSettingChannel = true, .settingChannel = ch::config_set,
                .hasRank = true, .rank = valence::ui_ranks::control});
    // Commands map into [window_min, window_max]; window_max > window_min.
    c.addLayoutField({.name = "window_min",  .type = PackedFieldType::f32, .unit = "mm",    .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::rail_mm,
                      .dflt = SettingDefault::ofFloat(factory::window_min),
                      .group = "Stroke window",
                      .desc = "Rear travel limit of the window",
                      .role = roles::window_min, .step = 1.0f,
                      .settingKey = 1, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm});
    c.addLayoutField({.name = "window_max",  .type = PackedFieldType::f32, .unit = "mm",    .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::rail_mm,
                      .dflt = SettingDefault::ofFloat(factory::window_max),
                      .group = "Stroke window",
                      .desc = "Front travel limit of the window",
                      .role = roles::window_max, .step = 1.0f,
                      .settingKey = 2, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm});
    // A ceiling, not a target; the factory value is deliberately gentle.
    c.addLayoutField({.name = "jog_speed",  .type = PackedFieldType::f32, .unit = "mm/s",  .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::speed_min, .max = ceiling::speed_max,
                      .dflt = SettingDefault::ofFloat(factory::jog_speed),
                      .group = "Jog limits",
                      .desc = "Speed ceiling for manual jogs",
                      .role = roles::limit_jog_speed, .step = 1.0f,
                      .settingKey = 3, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm_s});
    c.addLayoutField({.name = "jog_accel",  .type = PackedFieldType::f32, .unit = "mm/s2", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::accel_min, .max = ceiling::accel_max,
                      .dflt = SettingDefault::ofFloat(factory::jog_accel),
                      .group = "Jog limits",
                      .desc = "Acceleration ceiling for manual jogs",
                      .role = roles::limit_jog_accel, .step = 10.0f,
                      .settingKey = 4, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm_s2});
    c.addLayoutField({.name = "input_speed", .type = PackedFieldType::f32, .unit = "mm/s",  .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::speed_min, .max = ceiling::speed_max,
                      .dflt = SettingDefault::ofFloat(factory::input_speed),
                      .group = "Machine-driven limits",
                      .desc = "Top speed for patterns, scripts and streams",
                      .role = roles::limit_input_speed, .step = 10.0f,
                      .settingKey = 5, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm_s});
    c.addLayoutField({.name = "input_accel", .type = PackedFieldType::f32, .unit = "mm/s2", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::accel_min, .max = ceiling::accel_max,
                      .dflt = SettingDefault::ofFloat(factory::input_accel),
                      .group = "Machine-driven limits",
                      .desc = "Acceleration ceiling for patterns, scripts and streams",
                      .role = roles::limit_input_accel, .step = 100.0f,
                      .settingKey = 6, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm_s2});
    c.addLayoutField({.name = "max_rail",    .type = PackedFieldType::f32, .unit = "mm",    .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::rail_min, .max = ceiling::rail_mm,
                      .dflt = SettingDefault::ofFloat(factory::max_rail),
                      .group = "Rail geometry",
                      .desc = "Rail length, measured and stored by homing",
                      .role = roles::geometry_max_travel, .step = 1.0f,
                      .settingKey = 8, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm});
    // Protects the mechanics; not a smoothing knob.
    c.addLayoutField({.name = "input_jerk",  .type = PackedFieldType::f32, .unit = "mm/s3", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::jerk_min, .max = ceiling::jerk_max,
                      .dflt = SettingDefault::ofFloat(factory::input_jerk),
                      .group = "Machine-driven limits",
                      .desc = "Jerk ceiling for patterns, scripts and streams",
                      .role = roles::limit_input_jerk, .step = 1000.0f,
                      .settingKey = 7, .flags = valence::setting_flags::advanced,
                      .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::advanced,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm_s3});
    // DYNAMIC ENABLED STATE. Bit i gates the i-th SETTING-ANNOTATED field of
    // this layout, in layout order:
    //   0 window_min  1 window_max  2 jog_speed  3 jog_accel
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
                        .desc = "Settings the machine accepts right now",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"window_min", "window_max", "jog_speed", "jog_accel",
                        "input_speed", "input_accel", "max_rail", "input_jerk"});
    // measured_stroke (field 10, byte 33): THE REAL HOMING MEASUREMENT. 0
    // until the first completed home cycle this boot; then the usable rail,
    // the far datum minus the home datum minus both safety margins, unclamped
    // (max_rail holds it clamped to its bounds). No
    // setting_key: derived machine truth, never an assertion, so force_home's
    // stroke never lands here.
    // Append-only: added after enabled_mask, bytes 0..32 keep their offsets.
    c.addLayoutField({.name = "measured_stroke", .type = PackedFieldType::f32, .unit = "mm", .scale = 1.0f,
                      .desc = "Stroke measured by homing, 0 until homed",
                      .role = roles::geometry_measured_travel,
                      .hasRank = true, .rank = valence::ui_ranks::detail,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm});
    // RFC-099, append-only (byte 37): bits as enabled_mask's.
    c.addBitfieldField({.name = "trial_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Settings on trial, not stored yet",
                        .role = roles::meta_trial_pending,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"window_min", "window_max", "jog_speed", "jog_accel",
                        "input_speed", "input_accel", "max_rail", "input_jerk"});
    };

    // ---- "pattern-state" — STATE, normal, on-change -------------------------
    // PatternEngine live snapshot.  [1+1+4+4+4+4+1+1 = 20 B]. Append-only:
    // enabled_mask (field 7) and background_run (field 8, settingKey 7 on the
    // paired 0x3200 pattern-cmd intent) keep bytes 0..18 at their offsets.
    // category = generator; settingChannel = ch::pattern_cmd.
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
                .hasCategory = true, .category = valence::ui_categories::generator,
                .hasSettingChannel = true, .settingChannel = ch::pattern_cmd,
                .hasRank = true, .rank = valence::ui_ranks::control});
    c.addLayoutField({.name = "running",   .type = PackedFieldType::u8,  .unit = "",  .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofBool(false),
                      .group = "Pattern",
                      .desc = "Run the classic generator",
                      .role = roles::pattern_running,
                      .step = 1.0f, .settingKey = 1, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control});
    c.addSelectField({.name = "pattern",   .type = PackedFieldType::u8,  .unit = "",  .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 6.0f,
                      .dflt = SettingDefault::ofInt(0),
                      .group = "Pattern",
                      .desc = "Stroke pattern to play",
                      .role = roles::pattern_select,
                      .step = 1.0f, .settingKey = 2, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control},
                     {"Simple Stroke", "Teasing Pounding", "Robo Stroke", "Half'n'Half",
                      "Deeper", "Stop'n'Go", "Insist"});
    // Also bounded by input_speed on 0x1000.
    c.addLayoutField({.name = "speed",     .type = PackedFieldType::f32, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofFloat(0.0f),
                      .group = "Pattern",
                      .desc = "Pattern speed, percent of its range",
                      .role = roles::pattern_speed,
                      .step = 1.0f, .settingKey = 3, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::percent});
    c.addLayoutField({.name = "depth",     .type = PackedFieldType::f32, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofFloat(0.0f),
                      .group = "Pattern",
                      .desc = "How far into the window the pattern reaches",
                      .role = roles::pattern_depth,
                      .step = 1.0f, .settingKey = 4, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::percent});
    c.addLayoutField({.name = "stroke",    .type = PackedFieldType::f32, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofFloat(0.0f),
                      .group = "Pattern",
                      .desc = "Stroke length, percent of depth",
                      .role = roles::pattern_stroke,
                      .step = 1.0f, .settingKey = 5, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::percent});
    // What it changes depends on the selected pattern.
    c.addLayoutField({.name = "sensation", .type = PackedFieldType::f32, .unit = "",  .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofFloat(50.0f),
                      .group = "Pattern",
                      .desc = "Pattern character, 50 is neutral",
                      .role = roles::pattern_sensation,
                      .step = 1.0f, .settingKey = 6, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control});
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
                        .desc = "Settings the machine accepts right now",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"running", "pattern", "speed", "depth", "stroke", "sensation", "background_run"});
    // `source.background_run` — appended after enabled_mask, settingKey 7
    // (append-only, never inserted before an existing field). Bit 6 of the
    // mask above is UNCONDITIONALLY 1: this is a standing policy choice
    // ("should the generator keep going if I disconnect"), not a live motion
    // command, so unlike bits 0-5 it is never gated by homed/estop — it
    // still needs a bit (every setting-annotated field does), just one that
    // never drops. One policy for both generators: a released session stops
    // the classic and the advanced alike unless this is on (RFC-093).
    c.addLayoutField({.name = "background_run", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofBool(false),
                      .group = "Pattern",
                      .desc = "Keep running after the session disconnects",
                      .role = roles::source_background_run,
                      .step = 1.0f, .settingKey = 7, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control});
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
                .hasCategory = true, .category = valence::ui_categories::system,
                .hasRank = true, .rank = valence::ui_ranks::diagnostic});
    // aspect/scope: every field here is a session-scope figure (RENDERING.md
    // §5.2 scope=session is the default, set explicitly per the honesty rule
    // — §5.4 "scope MUST always be displayed or unambiguously implied").
    // aspect is total(4) for the cumulative counters and peak(1) for
    // peak_mm_s, its companion-instrument tag (§5.4).
    c.addLayoutField({.name = "strokes",    .type = PackedFieldType::u32, .unit = "",     .scale = 1.0f,
                      .group = "Session", .desc = "Direction reversals this session",
                      .hasAspect = true, .aspect = valence::value_aspects::total,
                      .hasScope = true, .scope = valence::value_scopes::session,
                      .hasUnitId = true, .unitId = valence::unit_ids::count});
    c.addLayoutField({.name = "distance_m", .type = PackedFieldType::f32, .unit = "m",    .scale = 1.0f,
                      .group = "Session", .desc = "Distance traveled this session",
                      .hasAspect = true, .aspect = valence::value_aspects::total,
                      .hasScope = true, .scope = valence::value_scopes::session});
                      // unit_id deliberately absent: unit_ids has no meters (only mm, id 0) and
                      // reporting mm here would misstate the physical unit — falls back to the
                      // "m" string label (the honest, documented unit_ids gap).
    c.addLayoutField({.name = "peak_mm_s",  .type = PackedFieldType::f32, .unit = "mm/s", .scale = 1.0f,
                      .group = "Session", .desc = "Top carriage speed this session",
                      .hasAspect = true, .aspect = valence::value_aspects::peak,
                      .hasScope = true, .scope = valence::value_scopes::session,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm_s});
    c.addLayoutField({.name = "energy_wh",  .type = PackedFieldType::f32, .unit = "Wh",   .scale = 1.0f,
                      .group = "Session", .desc = "Energy drawn this session",
                      .hasAspect = true, .aspect = valence::value_aspects::total,
                      .hasScope = true, .scope = valence::value_scopes::session,
                      .hasUnitId = true, .unitId = valence::unit_ids::wh});
    c.addLayoutField({.name = "session_ms", .type = PackedFieldType::u32, .unit = "ms",   .scale = 1.0f,
                      .group = "Session", .desc = "Session time, since boot or counter reset",
                      .hasAspect = true, .aspect = valence::value_aspects::total,
                      .hasScope = true, .scope = valence::value_scopes::session,
                      .hasUnitId = true, .unitId = valence::unit_ids::ms});
    };

    // ---- "motion-input" — STREAM, c2h, control, ≤333 Hz ---------------------
    // Continuous stroke-window targets + optional signed handoff velocity,
    // decoded straight off BundleView by the hub delegate's onStreamBundle()
    // into the motion queue (motionSubmit()); maps to arbiter source 1
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
                // ui_categories::generator's own note names "streams" explicitly.
                .hasCategory = true, .category = valence::ui_categories::generator,
                .hasRank = true, .rank = valence::ui_ranks::control});
    // input.* (RFC-071): how a client finds the motion input without a name.
    c.addLayoutField({.name = "target_norm", .type = PackedFieldType::u16, .unit = "norm",   .scale = 10000.0f,
                      .role = roles::input_target,
                      .hasUnitId = true, .unitId = valence::unit_ids::normalized});
    c.addLayoutField({.name = "vel_norm",    .type = PackedFieldType::i16, .unit = "norm/s", .scale = 1000.0f,
                      .role = roles::input_velocity});
                      // unit_id left absent for vel_norm: unit_ids has no "normalized/s" variant
                      // (a documented gap, same class as the kinetic_limits override fields below).
    };

    // ---- "motion-segment" — STREAM, c2h, control, ≤50 Hz --------------------
    // TIMED-SEGMENT motion streaming, the WAVEFORM-mode companion to 0x0084.
    // Carries the sender's native segments — ONE {target, duration, end_vel}
    // per stroke leg — which the planner renders as a knot at the segment's
    // start plus its duration (MotionArbiter.cpp, the Kinetic² boundary).
    // Decoded by FIXED OFFSET in the delegate's onStreamBundle() (same
    // convention as 0x0084), enqueued into the SAME motion queue, mapped to arbiter source 1
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
                .hasCategory = true, .category = valence::ui_categories::generator,
                .streamKind = valence::stream_kinds::segments,
                .hasRank = true, .rank = valence::ui_ranks::control});
    c.addLayoutField({.name = "target_norm",  .type = PackedFieldType::u16, .unit = "norm",   .scale = 10000.0f,
                      .role = roles::input_target,
                      .hasUnitId = true, .unitId = valence::unit_ids::normalized});
    c.addLayoutField({.name = "duration_ms",  .type = PackedFieldType::u16, .unit = "ms",     .scale = 1.0f,
                      .role = roles::input_duration,
                      .hasUnitId = true, .unitId = valence::unit_ids::ms});
    c.addLayoutField({.name = "end_vel_norm", .type = PackedFieldType::i16, .unit = "norm/s", .scale = 1000.0f,
                      .role = roles::input_end_velocity});
    };

    // ---- "osc-drive" -- STREAM, c2h, control: the oscillator's axis ---------
    // SPEC 9.7 (RFC-103): samples-kind, the channel role osc.drive fixing the
    // layout, amplitude then frequency, each 0 .. 1 and mapped through its
    // parameter's bounds where that parameter's drive is axis. No input.*
    // role: never motion input, never a source, never the rail
    // (ValenceDevice::sourceForChannel()). A script's V8 and V9.  [4 + 4 = 8 B]
    auto addOscDrive = [&]() {
    c.addEntry({.id = ch::osc_drive, .name = "osc-drive",
                .cls = ChannelClass::STREAM, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 50.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = valence::ui_categories::generator,
                .hasRank = true, .rank = valence::ui_ranks::control,
                .role = valence::channel_roles::osc_drive});
    c.addLayoutField({.name = "amplitude", .type = PackedFieldType::f32, .unit = "norm", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .hasUnitId = true, .unitId = valence::unit_ids::normalized});
    c.addLayoutField({.name = "frequency", .type = PackedFieldType::f32, .unit = "norm", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .hasUnitId = true, .unitId = valence::unit_ids::normalized});
    };

    // ---- "plan-strip" — STATE, elevated, 45 Hz ------------------------------
    // THE PLANNER'S CURRENT SEGMENT: what the planner is executing right now,
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
    // durationUs/elapsedUs stay µs u32. RFC-100 appends the plan.flags byte.
    // Positions are i32: a plan outside the window (an override jog, a window
    // written away from the carriage) is a share below 0 or far above 1, which
    // a u16 or an i16 at this scale cannot carry (bd val-vik).
    // [1+1+4+4+4+2+4+4+1 = 25 B]
    auto addPlanStrip = [&]() {
    c.addEntry({.id = ch::plan_strip, .name = "plan-strip",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 45.0f,
                .defaultPriority = Priority::elevated,
                .hasCategory = true, .category = valence::ui_categories::motion,
                .hasRank = true, .rank = valence::ui_ranks::diagnostic});
    c.addBitfieldField({.name = "flags", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f,
                        .group = card::active_plan,
                        .desc = "Active plan and planner mode bits"},
                       {"active", "live_mode", "grad_mode"});
    // RFC-035: the plan.* role family — a generic plan-strip widget finds this
    // channel BY ROLE on any machine, replacing the reference client's
    // documented /plan/i entry-name regex (which silently fails on a hub that
    // names the concept differently).
    c.addSelectField({.name = "style", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .group = card::active_plan,
                      .desc = "Planning mode of the motion core",
                      .role = roles::plan_style},
                     {"idle", "waveform", "chase", "settle", "hold"});
    c.addLayoutField({.name = "start_norm", .type = PackedFieldType::i32, .unit = "norm",   .scale = 10000.0f,
                      .group = card::active_plan, .desc = "Start of the current plan",
                      .role = roles::plan_start});
    c.addLayoutField({.name = "end_norm",   .type = PackedFieldType::i32, .unit = "norm",   .scale = 10000.0f,
                      .group = card::active_plan, .desc = "End of the current plan",
                      .role = roles::plan_end});
    c.addLayoutField({.name = "cur_norm",   .type = PackedFieldType::i32, .unit = "norm",   .scale = 10000.0f,
                      .group = card::active_plan, .desc = "Setpoint the plan is producing now",
                      .role = roles::plan_current});
    c.addLayoutField({.name = "cur_vel",    .type = PackedFieldType::i16, .unit = "norm/s", .scale = 1000.0f,
                      .group = card::active_plan, .desc = "Plan velocity right now, signed",
                      .role = roles::plan_velocity});
    c.addLayoutField({.name = "duration_us", .type = PackedFieldType::u32, .unit = "us",    .scale = 1.0f,
                      .group = card::active_plan, .desc = "Total duration of the current plan",
                      .role = roles::plan_duration,
                      .hasUnitId = true, .unitId = valence::unit_ids::us});
    c.addLayoutField({.name = "elapsed_us",  .type = PackedFieldType::u32, .unit = "us",    .scale = 1.0f,
                      .group = card::active_plan, .desc = "Time elapsed in the current plan",
                      .role = roles::plan_elapsed,
                      .hasUnitId = true, .unitId = valence::unit_ids::us});
    // Labels in registry plan_flags bit order.
    c.addBitfieldField({.name = "feasibility", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f,
                        .group = card::active_plan,
                        .desc = "How the planner bent the current plan",
                        .role = roles::plan_flags},
                       {"shaped", "stretched", "clamped"});
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
                    .hasCategory = true, .category = valence::ui_categories::system,
                    .hasRank = true, .rank = valence::ui_ranks::diagnostic});
        c.addLayoutField({.name = "bus_mV",  .type = PackedFieldType::u16, .unit = "V", .scale = 1000.0f,
                          .group = "Power", .desc = "DC bus voltage at the motor drive",
                          .role = roles::telemetry_power_bus});
        c.addLayoutField({.name = "peak_mA", .type = PackedFieldType::u16, .unit = "A", .scale = 1000.0f,
                          .group = "Power",
                          .desc = "Peak bus current since last reset"});
        c.addLayoutField({.name = "i_bus_mA", .type = PackedFieldType::i16, .unit = "A", .scale = 1000.0f,
                          .group = "Power", .desc = "Bus current, signed by the drive",
                          .role = roles::telemetry_current});
        if (feat.has_power_monitor) {
            c.addLayoutField({.name = "die_c10", .type = PackedFieldType::i16, .unit = "C", .scale = 10.0f,
                              .group = "Power", .desc = "Power monitor die temperature",
                              .role = roles::telemetry_temp});
        }
    }
    };

    // ---- "kinetic-diag" — STATE, background, 1 Hz ------------------------
    // Plan counts, the per-kind anomaly breakdown, the on-device plan-time
    // bench, and the Valence stream-ingress counters.
    //
    // The per-kind counters are six NAMED fields rather than one array: a
    // generic client renders named fields with no per-device knowledge.
    // Their order is kinetic2::AnomalyKind's own from kind 1 (kind 0 is
    // never counted), so a new engine kind appends a field to the END OF
    // THIS BLOCK, shifting every offset after it. Before the v1.0 tag that
    // is a resync (the etag moves); after it, a new kind wants its own
    // channel (SPEC 5.4).
    //
    // Channel role anomaly.summary (RFC-065): the latched counters twin of
    // motion-anomaly's events.anomaly, so a client binds log and counters
    // together.
    //
    // reset_gen is the observable-reset half: every applied counter reset
    // increments it, so EVERY subscriber sees the reset happened, not only
    // the session that asked for it. Without it a client watching the
    // counters cannot tell a reset from a reboot from a wrap.
    //   [3*4 + 1 + 1 + 6*4 + 3*4 + 5*4 + 2 = 72 B]
    auto addMotionDiag = [&]() {
    c.addEntry({.id = ch::motion_diag, .name = "kinetic-diag",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 1.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = valence::ui_categories::motion,
                .hasRank = true, .rank = valence::ui_ranks::diagnostic,
                .role = valence::channel_roles::anomaly_summary});
    c.addLayoutField({.name = "plans",    .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::planner, .desc = "Motion plans computed successfully"});
    // On a rejection the previous plan keeps running.
    c.addLayoutField({.name = "failures", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::planner, .desc = "Commands the planner rejected"});
    c.addLayoutField({.name = "anomalies", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::planner, .desc = "Planner anomalies of every kind"});
    c.addSelectField({.name = "mode",      .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .group = card::planner, .desc = "Planning mode of the motion core"},
                     {"idle", "waveform", "chase", "settle"});
    // Option ordinals are wire values (MotionArbiter.cpp kPlanKindBezier).
    c.addSelectField({.name = "plan_kind", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .group = card::planner, .desc = "Curve type of the active plan"},
                     {"none", "bezier"});
    // Per-kind breakdown, kinetic2::AnomalyKind 1..6 in order.
    c.addLayoutField({.name = "anom_settle",      .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::anomalies, .desc = "Moves braked to rest after the stream stopped"});
    c.addLayoutField({.name = "anom_endvel_clamped", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::anomalies, .desc = "Authored speeds held to the ceiling or the wall"});
    c.addLayoutField({.name = "anom_knot_trimmed", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::anomalies, .desc = "Knots trimmed to fit the ceilings"});
    c.addLayoutField({.name = "anom_dwell_zeroed", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::anomalies,
                      .desc = "Stale arrival speeds ignored on held positions"});
    c.addLayoutField({.name = "anom_knot_refused", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::anomalies,
                      .desc = "Points refused as late or out of order"});
    c.addLayoutField({.name = "anom_piece_over_ceiling", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::anomalies,
                      .desc = "Spans no trim could keep inside a limit"});
    c.addLayoutField({.name = "plan_us_last", .type = PackedFieldType::u32, .unit = "us", .scale = 1.0f,
                      .group = card::plan_time, .desc = "Compute time of the latest plan",
                      .hasUnitId = true, .unitId = valence::unit_ids::us});
    c.addLayoutField({.name = "plan_us_max",  .type = PackedFieldType::u32, .unit = "us", .scale = 1.0f,
                      .group = card::plan_time, .desc = "Worst plan compute time since reset",
                      .hasUnitId = true, .unitId = valence::unit_ids::us});
    c.addLayoutField({.name = "plan_us_avg",  .type = PackedFieldType::f32, .unit = "us", .scale = 1.0f,
                      .group = card::plan_time, .desc = "Smoothed average plan compute time",
                      .hasUnitId = true, .unitId = valence::unit_ids::us});
    c.addLayoutField({.name = "sync_bundles",  .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::stream_ingress, .desc = "Motion bundles accepted over Valence"});
    c.addLayoutField({.name = "sync_samples",  .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::stream_ingress, .desc = "Motion samples decoded from bundles"});
    c.addLayoutField({.name = "sync_enqueued", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::stream_ingress, .desc = "Samples that reached the motion core"});
    c.addLayoutField({.name = "sync_dropped",  .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::stream_ingress,
                      .desc = "Samples dropped as late, unusable or refused"});
    c.addLayoutField({.name = "sync_seg_bundles", .type = PackedFieldType::u32, .unit = "", .scale = 1.0f,
                      .group = card::stream_ingress, .desc = "Bundles that carried timed segments"});
    c.addLayoutField({.name = "reset_gen", .type = PackedFieldType::u16, .unit = "", .scale = 1.0f,
                      .group = card::planner,
                      .desc = "Increments on every counter reset",
                      .role = roles::meta_reset_gen});
    };

    // ---- "motion-anomaly" — EVENT, watch, normal ----------------------------
    // The planner's anomaly feed, as EDGES.
    //
    // FIRST DEVICE-AUTHORED EVENT CHANNEL: every field below is keyed by
    // THIS CHANNEL'S OWN schema (valence::anom_body), naming them costs no
    // registry PR — under a kind-specific-fields-at-top-level grammar this
    // channel could not exist without one, which the self-describing
    // catalog's body-map grammar exists to prevent.
    //
    // The kind rides the frame's event_kind (33) and is labeled by this entry's
    // event_kinds table (RFC-065), index-aligned with kinetic2::AnomalyKind;
    // a new kind appends. Body key 1 mirrors the value (SPEC 9.4 MAY) and carries no labels,
    // because the table is their one home. The entry's events.anomaly role is
    // how a client finds the anomaly feed without a name.
    //
    // NO replay depth: an anomaly is an edge, and §9.4's default (edges are
    // never replayed) is right for it. The counters on kinetic-diag (0x1111,
    // channel role anomaly.summary) are the durable record: the event/state
    // duality doing its job.
    auto addMotionAnomaly = [&]() {
    c.addEntry({.id = ch::motion_anomaly, .name = "motion-anomaly",
                .cls = ChannelClass::EVENT, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = valence::ui_categories::motion,
                .hasRank = true, .rank = valence::ui_ranks::diagnostic,
                .role = valence::channel_roles::events_anomaly});
    c.setEventKinds({"none", "settle", "endvel_clamped", "knot_trimmed", "dwell_zeroed", "knot_refused",
                     "piece_over_ceiling"});
    c.addSchemaField({.key = anom_body::kind, .name = "kind", .type = CborFieldType::uint_t, .unit = "",
                      .group = card::anomalies, .desc = "Anomaly kind, same as the event kind"});
    c.addSchemaField({.key = anom_body::seq, .name = "seq", .type = CborFieldType::uint_t, .unit = "",
                      .group = card::anomalies, .desc = "Rolling event id, wraps"});
    c.addSchemaField({.key = anom_body::target, .name = "target", .type = CborFieldType::f32_t,
                      .unit = "norm", .group = card::anomalies,
                      .desc = "Commanded position, 0 to 1 across the window"});
    c.addSchemaField({.key = anom_body::detail, .name = "detail", .type = CborFieldType::f32_t, .unit = "",
                      .group = card::anomalies, .desc = "Speed, duration or stroke fraction, per kind"});
    // unit_ids us, not hub_s: a schema field carries no scale and the registry
    // has no microsecond hub-time unit (RFC-086), so the wire states this
    // stamp's magnitude only, never which clock it was read from.
    c.addSchemaField({.key = anom_body::t_us, .name = "t_us", .type = CborFieldType::uint_t, .unit = "us",
                      .group = card::anomalies, .desc = "Motion core time of the event",
                      .hasUnitId = true, .unitId = valence::unit_ids::us});
    };

    // ---- "machine-modes" — STATE, elevated, on-change -----------------------
    // Live MODE settings: schedule_horizon (RFC-087), the segments grants'
    // horizon, the flip and the home speed.
    // motion_backend is READ-ONLY (no setting_key): this board has one
    // backend, soldered, and a select it could not honor would be a control
    // that drives nothing. home_style exists only with has_drive, because
    // both homing cycles it picks between need a drive. blend_mode_reserved,
    // stream_speed_reserved and overshoot_clamp_reserved are retired bytes
    // (see below), and `transport` (WS_OP_MODE) is a PERMANENT GAP at INTENT
    // key 2 — see ch::modes_set's note.
    //
    // `blend_mode` is RETIRED: no motion behavior on this board reads it. The
    // BYTE STAYS (renamed `blend_mode_reserved`, still byte 0 so bytes 1..3
    // keep their offsets — packed layouts are append-only, deleting the byte
    // would be a wire break) but carries NO setting_key and rank hidden
    // (RENDERING section 4: hidden never renders), so no client offers it as a
    // setting or shows it at all. Its INTENT key (ch::modes_set key 1) is a
    // permanent gap alongside key 2's `transport`; ValenceDevice::applyModes
    // never reads it.
    //
    // `stream_speed_mode` is RETIRED the same way: nothing reads it to make a
    // decision. BYTE STAYS as `stream_speed_reserved` at byte 1 so bytes 2..3
    // keep their offsets; INTENT key 3 is retired with a permanent gap.
    //
    // `overshoot_clamp` is RETIRED the same way: the planner has no overshoot
    // guard to arm (its solver never accepts an excursion). BYTE STAYS as
    // `overshoot_clamp_reserved` at byte 2; INTENT key 4 is a permanent gap.
    //
    // They are MODES, not limits: each one changes what the machine DOES
    // with a command rather than how far or how fast it may go — their own
    // category rather than more fields on 0x0081 (see ch::machine_modes for
    // the enabled_mask arithmetic that makes the split structural).
    //
    // Layout [13 B, 14 B with has_drive]: u8 enums the catalog names, then
    // the f32 home_speed and the u8 datagram_estop appended at the tail, so a
    // generic client renders them without knowing this device exists.
    // ValenceDevice.cpp's publishMachineModes() packs the same bytes.
    auto addMachineModes = [&]() {
    c.addEntry({.id = ch::machine_modes, .name = "machine-modes",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::elevated,
                .hasCategory = true, .category = valence::ui_categories::motion,
                .hasSettingChannel = true, .settingChannel = ch::modes_set,
                .hasRank = true, .rank = valence::ui_ranks::advanced});
    // RETIRED padding, see the entry comment above. Rank hidden and no
    // options/group/default/setting_key: never rendered, never a setting.
    // publishMachineModes() writes 0 to all three bytes.
    c.addLayoutField({.name = "blend_mode_reserved", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .desc = "Retired padding, always 0",
                      .hasRank = true, .rank = valence::ui_ranks::hidden});
    c.addLayoutField({.name = "stream_speed_reserved", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .desc = "Retired padding, always 0",
                      .hasRank = true, .rank = valence::ui_ranks::hidden});
    c.addLayoutField({.name = "overshoot_clamp_reserved", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .desc = "Retired padding, always 0",
                      .hasRank = true, .rank = valence::ui_ranks::hidden});
    // Bit i gates the i-th setting-annotated field, same rule as 0x0081.
    // No reserved byte and not motion_backend carries a setting_key, so
    // home_style (where it exists) is bit 0, then schedule_horizon, flipped,
    // home_speed and datagram_estop.
    if (feat.has_drive) {
        c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                            .scale = 1.0f,
                            .desc = "Settings the machine accepts right now",
                            .role = roles::meta_enabled_mask,
                            .hasRank = true, .rank = valence::ui_ranks::detail},
                           {"home_style", "schedule_horizon", "flipped", "home_speed", "datagram_estop"});
    } else {
        c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                            .scale = 1.0f,
                            .desc = "Settings the machine accepts right now",
                            .role = roles::meta_enabled_mask,
                            .hasRank = true, .rank = valence::ui_ranks::detail},
                           {"schedule_horizon", "flipped", "home_speed", "datagram_estop"});
    }
    // Which path actually drives the motor. READ-ONLY: the backend is what is
    // soldered, so there is no choice for a setting to make.
    // "quadrature" is ordinal 2, APPENDED rather than substituted: a select's
    // wire value is its index, so re-pointing 0 or 1 would silently re-label a
    // value another machine in this ecosystem already publishes.
    c.addSelectField({.name = "motion_backend", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .group = card::motion_behavior,
                      .desc = "Signal path that drives the motor",
                      .flags = valence::setting_flags::advanced,
                      .hasRank = true, .rank = valence::ui_ranks::advanced},
                     {"step-dir", "modbus", "quadrature"});
    // Needs a drive: one cycle feels for the hard stops through it, the other
    // hands it the whole job.
    if (feat.has_drive) {
        c.addSelectField({.name = "home_style", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                          .dflt = SettingDefault::ofInt(0),
                          .group = card::motion_behavior,
                          .desc = "How the machine finds home",
                          .settingKey = 6, .flags = valence::setting_flags::advanced,
                          .hasSettingKey = true,
                          .hasRank = true, .rank = valence::ui_ranks::advanced},
                         {"sensorless sweep", "drive built-in"});
    }
    // RFC-087: the schedule horizon a segments grant advertises (kHorizonMs).
    // Refused INTERLOCK while any session holds a segments grant, because the
    // grant's horizon is a commitment for its life (SPEC 5.4); enabled_mask grays
    // it while a grant is live.
    c.addSelectField({.name = "schedule_horizon", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(0),
                      .group = card::streaming,
                      .desc = "How far ahead segment players may schedule",
                      .settingKey = 7, .flags = valence::setting_flags::advanced,
                      .hasSettingKey = true,
                      .hasRank = true, .rank = valence::ui_ranks::advanced},
                     {"250 ms", "500 ms", "1000 ms"});
    // RFC-088 (SPEC 9.6): how the rail is mounted. On, position 0 is the far
    // end and the hub mirrors positions, the window and every target. Stored;
    // gated to a homed rail at rest with no source and no override.
    c.addSelectField({.name = "flipped", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(0),
                      .group = card::motion_behavior,
                      .desc = "Position 0 at the far end",
                      .role = roles::axis_flipped,
                      .settingKey = 8, .hasSettingKey = true,
                      .hasRank = true, .rank = valence::ui_ranks::control},
                     {"off", "on"});
    // RFC-099, append-only: bits as enabled_mask's. None is trialable: each is
    // gated on live state (ValenceDevice.cpp), so this mask reads 0.
    if (feat.has_drive) {
    c.addBitfieldField({.name = "trial_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Settings on trial, not stored yet",
                        .role = roles::meta_trial_pending,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"home_style", "schedule_horizon", "flipped", "home_speed", "datagram_estop"});
    } else {
    c.addBitfieldField({.name = "trial_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Settings on trial, not stored yet",
                        .role = roles::meta_trial_pending,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"schedule_horizon", "flipped", "home_speed", "datagram_estop"});
    }
    // The home cycle's approach speed (MotionArbiter.h, homing): the re-touch
    // runs at a quarter of it, and the arbiter holds it to jog_speed. Applied
    // at the next cycle's start. Append-only, after trial_mask.
    c.addLayoutField({.name = "home_speed", .type = PackedFieldType::f32, .unit = "mm/s", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = ceiling::home_speed_min, .max = ceiling::speed_max,
                      .dflt = SettingDefault::ofFloat(factory::home_speed),
                      .group = card::motion_behavior,
                      .desc = "Approach speed for homing",
                      .step = 1.0f,
                      .settingKey = 9, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::advanced,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm_s});
    // RFC-053 item 3: whether an ESTOP datagram on the SPEC 13.8 port latches
    // (ValenceEstopDatagram.h); DISCOVER_REPLY bit1 follows it. Default on;
    // its writer key is configure tier (modes-set key 10). Stored. Append-
    // only, after home_speed.
    c.addSelectField({.name = "datagram_estop", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .dflt = SettingDefault::ofInt(1),
                      .group = card::safety,
                      .desc = "E-stop from any device on the network",
                      .settingKey = 10, .hasSettingKey = true,
                      .hasRank = true, .rank = valence::ui_ranks::control},
                     {"off", "on"});
    };

    // ---- "kinetic-*" — STATE, motion, section Tuning ----------------------
    // The motion engine's live-tune surface. No controls outside Valence.
    //
    // TWO CHANNELS, TWO TABS, split by category, not by size: bit i of an
    // enabled_mask gates the i-th setting of ITS layout, and a channel past
    // eight settings carries a second mask pair (SPEC §8.8). kinetic-limits
    // holds the planner CEILINGS, which are commissioning (RFC-079), so it
    // alone carries category setup; kinetic-planner is category motion,
    // section Tuning (RFC-094, `card::`).
    //
    // ONE SHARED WRITER (kinetic-set). `settingChannel` is per-entry and
    // `setting_key` is a key WITHIN that writer, so several STATE channels
    // may name the same INTENT channel provided their keys never collide.
    // Keys run 1..8 across the two cards, contiguous.
    //
    // Applied live and persisted with 0x1000 and cfg_gen in one blob
    // (StoredState.h). The `default` annotations stay the FACTORY values
    // (motionDefaultTuning()), never the stored ones: SPEC distinguishes default
    // from current, and current is what the STATE carries.
    auto addKineticLimits = [&]() {
    c.addEntry({.id = ch::kinetic_limits, .name = "kinetic-limits",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = valence::ui_categories::setup,
                .hasSettingChannel = true, .settingChannel = ch::kinetic_set,
                .hasRank = true, .rank = valence::ui_ranks::advanced});
    // The three overrides: 0 derives the ceiling from the mm limits on 0x1000.
    c.addLayoutField({.name = "jmax_ovr", .type = PackedFieldType::f32, .unit = "1/s3", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 2000000.0f,
                      .dflt = SettingDefault::ofFloat(0.0f), .group = "Ceiling overrides",
                      .desc = "Jerk ceiling override, 0 for automatic",
                      .role = "", .step = 1000.0f,
                      .settingKey = 1, .flags = valence::setting_flags::advanced,
                      .hasSettingKey = true, .hasStep = true});
    c.addLayoutField({.name = "vmax_ovr", .type = PackedFieldType::f32, .unit = "1/s", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 20.0f,
                      .dflt = SettingDefault::ofFloat(0.0f), .group = "Ceiling overrides",
                      .desc = "Speed ceiling override, 0 for automatic",
                      .settingKey = 2, .flags = valence::setting_flags::advanced,
                      .hasSettingKey = true});
    c.addLayoutField({.name = "amax_ovr", .type = PackedFieldType::f32, .unit = "1/s2", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 500.0f,
                      .dflt = SettingDefault::ofFloat(0.0f), .group = "Ceiling overrides",
                      .desc = "Acceleration ceiling override, 0 for automatic",
                      .settingKey = 3, .flags = valence::setting_flags::advanced,
                      .hasSettingKey = true});
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Settings the machine accepts right now",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"jmax_ovr", "vmax_ovr", "amax_ovr"});
    // RFC-099, append-only.
    c.addBitfieldField({.name = "trial_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Settings on trial, not stored yet",
                        .role = roles::meta_trial_pending,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"jmax_ovr", "vmax_ovr", "amax_ovr"});
    };

    // Kinetic²'s planner options (RFC-108), for segments and samples alike:
    // the engine reads every setting here (kinetic2::Config of the same
    // names), chase_dense_ms excepted, which the arbiter applies at the knot
    // boundary (sampleLatencyUs()).
    //   [4 + 4 + 4 + 4 + 4 + 1 + 1 = 22 B]
    auto addKineticPlanner = [&]() {
    c.addEntry({.id = ch::kinetic_planner, .name = "kinetic-planner",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = valence::ui_categories::motion,
                .hasSettingChannel = true, .settingChannel = ch::kinetic_set,
                // Rank control: smoothness is an everyday setting, so the card
                // stays visible without an advanced-affordance gate; the other
                // options carry setting_flags::advanced field by field.
                .hasRank = true, .rank = valence::ui_ranks::control});
    c.addLayoutField({.name = "smoothness", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofFloat(factory::smoothness), .group = card::curve,
                      .desc = "Crisp at 0, smooth at 1",
                      .step = 0.05f, .settingKey = 4, .hasSettingKey = true, .hasStep = true});
    c.addLayoutField({.name = "handle_floor", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.05f, .max = 0.33f,
                      .dflt = SettingDefault::ofFloat(factory::handle_floor), .group = card::curve,
                      .desc = "Shortest handle the fit may use",
                      .step = 0.01f, .settingKey = 5, .flags = valence::setting_flags::advanced,
                      .hasSettingKey = true, .hasStep = true});
    // Below 1 a piece no trim within it makes legal renders over a ceiling
    // (piece_over_ceiling); the 0.1 floor keeps a slider from switching trims off.
    c.addLayoutField({.name = "trim_max", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.1f, .max = 1.0f,
                      .dflt = SettingDefault::ofFloat(factory::trim_max), .group = card::ceilings,
                      .desc = "Farthest a knot moves to fit the ceilings",
                      .step = 0.05f, .settingKey = 6, .flags = valence::setting_flags::advanced,
                      .hasSettingKey = true, .hasStep = true});
    // The samples grant's schedule_latency_us less the motion tick: a sample
    // becomes a knot this long after it arrives (RFC-105 promise 1). Refused
    // INTERLOCK, and its enabled_mask bit low, while a samples grant is live;
    // never trialable.
    c.addLayoutField({.name = "chase_dense_ms", .type = PackedFieldType::u32, .unit = "ms", .scale = 1000.0f,
                      .hasMin = true, .hasMax = true, .min = 10.0f, .max = 500.0f,
                      .dflt = SettingDefault::ofFloat(60.0f), .group = card::sample_streams,
                      .desc = "Delay a streamed sample renders behind",
                      .settingKey = 7, .flags = valence::setting_flags::advanced,
                      .hasSettingKey = true});
    // RFC-105 (bb): the curve this far ahead of now is committed when a knot
    // arrives mid-motion; the re-plan starts there (kinetic2::Config::react_us).
    c.addLayoutField({.name = "react_ms", .type = PackedFieldType::u32, .unit = "ms", .scale = 1000.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofFloat(factory::react_ms), .group = card::replanning,
                      .desc = "Committed curve ahead of a re-plan",
                      .step = 0.5f, .settingKey = 8, .flags = valence::setting_flags::advanced,
                      .hasSettingKey = true, .hasStep = true});
    // Bit i gates the i-th setting-annotated field.
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Settings the machine accepts right now",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"smoothness", "handle_floor", "trim_max", "chase_dense_ms", "react_ms"});
    // RFC-099. chase_dense_ms is never trialable (ValenceDevice.cpp).
    c.addBitfieldField({.name = "trial_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Settings on trial, not stored yet",
                        .role = roles::meta_trial_pending,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"smoothness", "handle_floor", "trim_max", "chase_dense_ms", "react_ms"});
    };

    // ---- "drive-tune" -- STATE, the AIM drive's own registers ---------------
    // Category `hardware`, not motion's Tuning: this writes the DRIVE, its unit is
    // the drive's ((r/min)/s), not the machine's mm/s2. Filing it beside the
    // planner knobs would invite reading one as the other. Rank `control` and
    // no advanced flag, so it is reachable without an advanced affordance.
    //   [4 + 1 mask = 5 B]
    auto addDriveTune = [&]() {
    c.addEntry({.id = ch::drive_tune, .name = "drive-tune",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = valence::ui_categories::hardware,
                .hasSettingChannel = true, .settingChannel = ch::drive_set,
                .hasRank = true, .rank = valence::ui_ranks::control});
    c.addLayoutField({.name = "accel_reg", .type = PackedFieldType::u32, .unit = "rpm/s",
                      .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 60098.0f,
                      .dflt = SettingDefault::ofInt(0), .group = "Servo drive",
                      // Below 60000 the drive ramps on its own and lags the planner.
                      .desc = "Ramp: 0 auto, 60000 none, 60001+ feedforward %",
                      .step = 1.0f, .settingKey = 1,
                      .hasSettingKey = true, .hasStep = true});
    // READBACK, no setting_key, so these render as readouts. What the DRIVE
    // reports, never what was asked for: the request and the register can
    // disagree, and only the drive's own answer settles it.
    c.addLayoutField({.name = "accel_reg_actual", .type = PackedFieldType::u32, .unit = "rpm/s",
                      .scale = 1.0f, .group = "Servo drive",
                      .desc = "Ramp register as read back from the drive"});
    // Below the planner's accel ceiling, the drive limits the stroke.
    c.addLayoutField({.name = "ramp_limit", .type = PackedFieldType::f32, .unit = "mm/s2",
                      .scale = 1.0f, .group = "Servo drive",
                      .desc = "Drive ramp in machine units"});
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f, .desc = "Settings the machine accepts right now",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"accel_reg"});
    };

    // ---- "oscillator" -- STATE, motion: the oscillator's twin (RFC-103) -----
    // What 0x3140 osc-set last applied, post-clamp, and what renders now:
    // osc.active and osc.amplitude_effective, the amplitude the ceilings and
    // the window leave (it yields first). The parameters' osc.* roles live on
    // the writer (SPEC 9.7, once per catalog); these mirror it by setting_key.
    // Session-volatile: nothing persists, enabled boots false, and a session
    // ending clears it (ValenceDevice.cpp onSessionLeft()). The two drives and
    // their bounds (SPEC 9.7) are appended, keys 7..16, frequency's then
    // amplitude's (kOscDriveCards).
    //   [1 + 4 + 4 + 1 + 4 + 4 + 1 + 4 + 2 x (1 + 4 x 4) = 57 B]
    auto addOscillator = [&]() {
    c.addEntry({.id = ch::oscillator, .name = "oscillator",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = valence::ui_categories::motion,
                .hasSettingChannel = true, .settingChannel = ch::osc_set,
                .hasRank = true, .rank = valence::ui_ranks::control});
    c.addLayoutField({.name = "enabled", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofBool(false), .group = "Oscillator",
                      .desc = "Oscillate on top of whatever moves the rail",
                      .step = 1.0f, .settingKey = 1, .hasSettingKey = true, .hasStep = true});
    c.addLayoutField({.name = "frequency", .type = PackedFieldType::f32, .unit = "Hz", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::osc_max_hz,
                      .dflt = SettingDefault::ofFloat(MotionOsc{}.frequency_hz), .group = "Oscillator",
                      .desc = "Cycles a second, holds aside",
                      .step = 0.1f, .settingKey = 2, .hasSettingKey = true, .hasStep = true,
                      .hasUnitId = true, .unitId = valence::unit_ids::hz});
    c.addLayoutField({.name = "amplitude", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofFloat(MotionOsc{}.amplitude), .group = "Oscillator",
                      .desc = "Peak displacement, share of the window",
                      .step = 0.001f, .settingKey = 3, .hasSettingKey = true, .hasStep = true});
    c.addSelectField({.name = "shape", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 3.0f,
                      .dflt = SettingDefault::ofInt(0), .group = "Oscillator",
                      .desc = "Waveform",
                      .step = 1.0f, .settingKey = 4, .hasSettingKey = true, .hasStep = true},
                     {"sine", "square", "saw", "saw_reverse"});
    c.addLayoutField({.name = "dwell_crest", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::osc_dwell_max,
                      .dflt = SettingDefault::ofFloat(0.0f), .group = "Oscillator",
                      .desc = "Hold at the crest, share of a cycle",
                      .step = 0.01f, .settingKey = 5, .hasSettingKey = true, .hasStep = true});
    c.addLayoutField({.name = "dwell_trough", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::osc_dwell_max,
                      .dflt = SettingDefault::ofFloat(0.0f), .group = "Oscillator",
                      .desc = "Hold at the trough, share of a cycle",
                      .step = 0.01f, .settingKey = 6, .hasSettingKey = true, .hasStep = true});
    c.addLayoutField({.name = "active", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .group = "Oscillator", .desc = "Rendering now",
                      .role = roles::osc_active});
    c.addLayoutField({.name = "amplitude_effective", .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                      .group = "Oscillator", .desc = "Peak displacement the ceilings and window leave",
                      .role = roles::osc_amplitude_effective});
    for (const OscDriveCard& d : kOscDriveCards) {
        const MotionOscDrive dflt = d.key0 == 7 ? MotionOsc{}.frequency_drive : MotionOsc{}.amplitude_drive;
        c.addSelectField({.name = d.drive, .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = 3.0f,
                          .dflt = SettingDefault::ofInt(dflt.drive), .group = "Oscillator",
                          .desc = d.drive_desc,
                          .step = 1.0f, .settingKey = d.key0, .hasSettingKey = true, .hasStep = true},
                         {"fixed", "speed", "position", "axis"});
        c.addLayoutField({.name = d.in_min, .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                          .dflt = SettingDefault::ofFloat(dflt.in_min), .group = "Oscillator",
                          .desc = "Drive input at the low end",
                          .settingKey = uint8_t(d.key0 + 1), .hasSettingKey = true});
        c.addLayoutField({.name = d.in_max, .type = PackedFieldType::f32, .unit = "", .scale = 1.0f,
                          .dflt = SettingDefault::ofFloat(dflt.in_max), .group = "Oscillator",
                          .desc = "Drive input at the high end",
                          .settingKey = uint8_t(d.key0 + 2), .hasSettingKey = true});
        c.addLayoutField({.name = d.out_min, .type = PackedFieldType::f32, .unit = d.unit, .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = d.top,
                          .dflt = SettingDefault::ofFloat(dflt.out_min), .group = "Oscillator",
                          .desc = d.out_min_desc,
                          .settingKey = uint8_t(d.key0 + 3), .hasSettingKey = true});
        c.addLayoutField({.name = d.out_max, .type = PackedFieldType::f32, .unit = d.unit, .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = d.top,
                          .dflt = SettingDefault::ofFloat(dflt.out_max), .group = "Oscillator",
                          .desc = d.out_max_desc,
                          .settingKey = uint8_t(d.key0 + 4), .hasSettingKey = true});
    }
    };

    // ---- "pattern-advanced" — STATE, normal, on-change ----------------------
    // The advanced generator's 7 BASE controls and its own run/stop
    // (advpat::Settings, everything except the per-control modulators,
    // 0x1211..0x1216). Their advgen.* roles (RFC-081, RFC-093) are the
    // advanced generator's binding: one each per catalog (SPEC 8.8), so the
    // 0x3210 writer's mirror fields carry none.
    //
    // RFC-093: the advanced generator is its own SPEC 11.4 source (id 3),
    // never a mode of the classic one: `running` here and 0x1200's `running`
    // are independent, and starting one while the other owns the rail is
    // refused SOURCE_CONFLICT. Same category as 0x1200, its own writer.
    //
    // Byte 0 is the retired mode switch, kept as hidden padding so bytes 1..8
    // keep their offsets; its writer key 1 on 0x3210 is a permanent gap.
    // `running` is appended after enabled_mask, writer key 45. The two dwells
    // (RFC-095, u16 at 0.01 strokes, writer keys 46 and 47) follow, gated by a
    // second enabled_mask whose bits continue the first's numbering (SPEC
    // 8.8: bit 0 is the ninth setting field).
    //   [1 pad + 7 fields + 1 mask + 1 running + 2 x 2 dwells + 1 mask = 15 B]
    auto addPatternAdvanced = [&]() {
    c.addEntry({.id = ch::pattern_advanced, .name = "pattern-advanced",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = valence::ui_categories::generator,
                .hasSettingChannel = true, .settingChannel = ch::pattern_advanced_cmd,
                .hasRank = true, .rank = valence::ui_ranks::control});
    // RETIRED padding (RFC-093). Rank hidden and no setting_key: never
    // rendered, never a setting. publishPatternPlane() writes 0.
    c.addLayoutField({.name = "ap_mode_reserved", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .desc = "Retired padding, always 0",
                      .hasRank = true, .rank = valence::ui_ranks::hidden});
    c.addLayoutField({.name = "master", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(0),
                      .group = "Advanced pattern",
                      .desc = "Overall stroke speed, 0 holds position",
                      .role = roles::advgen_master,
                      .step = 1.0f, .settingKey = 2, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::percent});
    c.addLayoutField({.name = "max_depth", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(10),
                      .group = "Depth window",
                      .desc = "Deepest point of the stroke",
                      .role = roles::advgen_depth_max,
                      .step = 1.0f, .settingKey = 3, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::percent});
    c.addLayoutField({.name = "min_depth", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(0),
                      .group = "Depth window",
                      .desc = "Shallowest point of the stroke",
                      .role = roles::advgen_depth_min,
                      .step = 1.0f, .settingKey = 4, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::percent});
    c.addLayoutField({.name = "in_speed", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 1.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(100),
                      .group = "Speed",
                      .desc = "In-stroke speed, percent of master",
                      .role = roles::advgen_speed_in,
                      .step = 1.0f, .settingKey = 5, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::percent});
    c.addLayoutField({.name = "out_speed", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 1.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(100),
                      .group = "Speed",
                      .desc = "Out-stroke speed, percent of master",
                      .role = roles::advgen_speed_out,
                      .step = 1.0f, .settingKey = 6, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::percent});
    c.addLayoutField({.name = "in_accel", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(40),
                      .group = "Acceleration",
                      .desc = "In-stroke acceleration",
                      .role = roles::advgen_accel_in,
                      .step = 1.0f, .settingKey = 7, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::percent});
    c.addLayoutField({.name = "out_accel", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                      .dflt = SettingDefault::ofInt(40),
                      .group = "Acceleration",
                      .desc = "Out-stroke acceleration",
                      .role = roles::advgen_accel_out,
                      .step = 1.0f, .settingKey = 8, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::percent});
    // Bit i gates the i-th setting-annotated field of this layout, same rule
    // as 0x1200: bits 0-6 the knobs, bit 7 `running`. The knobs are refused
    // only under e-stop (they move nothing until a start), so bits 0-6 track
    // e-stop alone; bit 7 also drops unhomed, as 0x1200's running bit does.
    // Neither drops while the classic generator owns the rail: the start is
    // refused SOURCE_CONFLICT and a client shows that refusal (RFC-093).
    c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f,
                        .desc = "Settings the machine accepts right now",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"master", "max_depth", "min_depth", "in_speed", "out_speed",
                        "in_accel", "out_accel", "running"});
    c.addLayoutField({.name = "running", .type = PackedFieldType::u8, .unit = "", .scale = 1.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .dflt = SettingDefault::ofBool(false),
                      .group = "Advanced pattern",
                      .desc = "Run the advanced generator",
                      .role = roles::advgen_running,
                      .step = 1.0f, .settingKey = kApRunKey, .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control});
    // Unit "strokes" with unit_id count, as the modulators' timing fields: the
    // registry has no stroke unit.
    c.addLayoutField({.name = "dwell_crest", .type = PackedFieldType::u16, .unit = "strokes", .scale = 100.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 655.35f,
                      .dflt = SettingDefault::ofFloat(0.0f),
                      .group = "Dwell",
                      .desc = "Hold at the deepest point, in strokes",
                      .role = roles::advgen_dwell_crest,
                      .step = 0.01f, .settingKey = apBaseKey(6), .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::count});
    c.addLayoutField({.name = "dwell_trough", .type = PackedFieldType::u16, .unit = "strokes", .scale = 100.0f,
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 655.35f,
                      .dflt = SettingDefault::ofFloat(0.0f),
                      .group = "Dwell",
                      .desc = "Hold at the shallowest point, in strokes",
                      .role = roles::advgen_dwell_trough,
                      .step = 0.01f, .settingKey = apBaseKey(7), .hasSettingKey = true, .hasStep = true,
                      .hasRank = true, .rank = valence::ui_ranks::control,
                      .hasUnitId = true, .unitId = valence::unit_ids::count});
    // Bits 0-1 are setting fields 8 and 9, the dwells: knobs, so e-stop alone
    // drops them, as bits 0-6 of the first mask.
    c.addBitfieldField({.name = "enabled_mask2", .type = PackedFieldType::bitfield8, .unit = "flag",
                        .scale = 1.0f,
                        .desc = "Settings the machine accepts right now",
                        .role = roles::meta_enabled_mask,
                        .hasRank = true, .rank = valence::ui_ranks::detail},
                       {"dwell_crest", "dwell_trough"});
    };

    // ---- "pattern-adv-mod-*" — STATE, background ----------------------------
    // The eight MODULATORS (RFC-066, SPEC 8.8 `mod.*`), one per base control
    // (advpat::Modifier x advpat::BASE_COUNT). A modulator swings its control
    // by `amount` through a cycle: `in_step` strokes rising, `in_wait` held,
    // `out_step` falling, `out_wait` at rest, the whole cycle shifted by
    // `offset` strokes. One modulator per entry, attached by the entry-level
    // mod_target to the 0x1210 field it rides; its mod.* roles bind within the
    // entry and repeat across the eight by design (SPEC 8.8 cardinality).
    //
    // The five timing fields carry unit "strokes" on both the layout and the
    // writer's schema (SPEC 8.8: a time-like mod field carries its clock in
    // its unit) with unit_id count, because the registry has no stroke unit.
    //
    // `amount` 0 = no modulation, 100 = the full swing (RFC-066). Presets
    // stored under the retired 100 = off meaning are migrated on load
    // (PatternPresetStore blob version 1), never read under this one.
    //
    // `base` is the advpat::BaseId. 0x1210 lays the base control out at
    // apBaseLayoutIndex(base), and the shared writer ch::pattern_advanced_cmd
    // keys this entry's six fields apModKeyBase(base) onward in field order,
    // the arithmetic ValenceDevice::applyPatternAdvanced runs in reverse.
    // Category generator, so all nine advanced-pattern cards merge into one
    // tab; `advanced`-flagged, the deep-customization layer under the base
    // controls.
    //   [1*6 fields + 1 mask = 7 B, x8 channels]
    auto addApModifierChannel = [&](uint16_t id, const char* wireName, const char* group,
                                    uint8_t base) {
        const uint8_t keyBase = apModKeyBase(base);
        c.addEntry({.id = id, .name = wireName,
                    .cls = ChannelClass::STATE, .dir = Direction::h2c,
                    .access = AccessLevel::watch, .maxRateHz = 0.0f,
                    .defaultPriority = Priority::background,
                    .hasCategory = true, .category = valence::ui_categories::generator,
                    .hasSettingChannel = true, .settingChannel = ch::pattern_advanced_cmd,
                    .hasRank = true, .rank = valence::ui_ranks::advanced,
                    .hasModTarget = true, .modTargetChannel = ch::pattern_advanced,
                    .modTargetField = apBaseLayoutIndex(base)});
        c.addLayoutField({.name = "amount", .type = PackedFieldType::u8, .unit = "%", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                          .dflt = SettingDefault::ofInt(0), .group = group,
                          .desc = "Modulation amount, 0 none, 100 full swing",
                          .role = roles::mod_amount,
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 0),
                          .flags = valence::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = valence::ui_ranks::advanced,
                          .hasUnitId = true, .unitId = valence::unit_ids::percent});
        c.addLayoutField({.name = "in_step", .type = PackedFieldType::u8, .unit = "strokes", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 1.0f, .max = 25.0f,
                          .dflt = SettingDefault::ofInt(1), .group = group,
                          .desc = "Strokes rising to full swing",
                          .role = roles::mod_rise,
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 1),
                          .flags = valence::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = valence::ui_ranks::advanced,
                          .hasUnitId = true, .unitId = valence::unit_ids::count});
        c.addLayoutField({.name = "in_wait", .type = PackedFieldType::u8, .unit = "strokes", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = 25.0f,
                          .dflt = SettingDefault::ofInt(0), .group = group,
                          .desc = "Strokes held at full swing",
                          .role = roles::mod_hold,
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 2),
                          .flags = valence::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = valence::ui_ranks::advanced,
                          .hasUnitId = true, .unitId = valence::unit_ids::count});
        c.addLayoutField({.name = "out_step", .type = PackedFieldType::u8, .unit = "strokes", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 1.0f, .max = 25.0f,
                          .dflt = SettingDefault::ofInt(1), .group = group,
                          .desc = "Strokes falling back to base",
                          .role = roles::mod_fall,
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 3),
                          .flags = valence::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = valence::ui_ranks::advanced,
                          .hasUnitId = true, .unitId = valence::unit_ids::count});
        c.addLayoutField({.name = "out_wait", .type = PackedFieldType::u8, .unit = "strokes", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = 25.0f,
                          .dflt = SettingDefault::ofInt(0), .group = group,
                          .desc = "Strokes resting at base before repeating",
                          .role = roles::mod_rest,
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 4),
                          .flags = valence::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = valence::ui_ranks::advanced,
                          .hasUnitId = true, .unitId = valence::unit_ids::count});
        c.addLayoutField({.name = "offset", .type = PackedFieldType::u8, .unit = "strokes", .scale = 1.0f,
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f,
                          .dflt = SettingDefault::ofInt(0), .group = group,
                          .desc = "Cycle offset against the other modulators",
                          .role = roles::mod_phase,
                          .step = 1.0f, .settingKey = uint8_t(keyBase + 5),
                          .flags = valence::setting_flags::advanced,
                          .hasSettingKey = true, .hasStep = true,
                          .hasRank = true, .rank = valence::ui_ranks::advanced,
                          .hasUnitId = true, .unitId = valence::unit_ids::count});
        // Same honesty note as 0x1210: no setter here checks `homed` either
        // (Modifier::set has no gate beyond the delegate's e-stop check), so
        // the mask tracks e-stop alone.
        c.addBitfieldField({.name = "enabled_mask", .type = PackedFieldType::bitfield8, .unit = "flag",
                            .scale = 1.0f,
                            .desc = "Settings the machine accepts right now",
                            .role = roles::meta_enabled_mask,
                            .hasRank = true, .rank = valence::ui_ranks::detail},
                           {"amount", "in_step", "in_wait", "out_step", "out_wait", "offset"});
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
    // payload is opaque device-defined bytes (in/out speed, in/out accel, the
    // two dwells, eight modifier blocks — "never depths or master speed").
    // See PatternSettings.h for the 56-byte layout and
    // ValenceDevice::applyPresets for the encode/decode.
    auto addPatternPresets = [&]() {
    c.addEntry({.id = ch::pattern_presets, .name = "pattern-presets",
                .cls = ChannelClass::STORE, .dir = Direction::h2c,
                .access = AccessLevel::control, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = valence::ui_categories::system,
                .hasRank = true, .rank = valence::ui_ranks::detail});
    c.addStoreDescriptor({.storeId = kPresetStoreId, .kind = kPresetKind,
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
    // (kPayloadBytes is tiny — 56 B — so kPresetCapacity fetches is cheap) or
    // read the name back from a save/rename ECHO it sent itself. A generation
    // bump means "re-enumerate", exactly like 0x000D. store_id (RFC-070)
    // joins this roster, the 0x5220 store and the 0x3220 writer by one key.
    auto addPatternPresetsRoster = [&]() {
    c.addEntry({.id = ch::pattern_presets_roster, .name = "pattern-presets-roster",
                .cls = ChannelClass::STATE, .dir = Direction::h2c,
                .access = AccessLevel::watch, .maxRateHz = 0.0f,
                .defaultPriority = Priority::background,
                .hasCategory = true, .category = valence::ui_categories::system,
                .hasSettingChannel = true, .settingChannel = ch::pattern_presets_cmd,
                .hasRank = true, .rank = valence::ui_ranks::detail,
                .hasStoreId = true, .storeId = kPresetStoreId});
    c.addLayoutField({.name = "generation", .type = PackedFieldType::u16, .unit = "count", .scale = 1.0f,
                      .group = card::pattern_presets});
    c.addLayoutField({.name = "count",      .type = PackedFieldType::u8,  .unit = "count", .scale = 1.0f,
                      .group = card::pattern_presets});
    c.addLayoutField({.name = "capacity",   .type = PackedFieldType::u8,  .unit = "count", .scale = 1.0f,
                      .group = card::pattern_presets});
    };

    // ---- "move" — INTENT, control, 20 Hz, critical --------------------------
    // {1:"position" f32 mm}: the JOG (SPEC 11.1, 11.4). This channel maps to
    // arbiter source 0 (MANUAL) in the delegate. Key 2 (the per-move bypass)
    // is retired by RFC-085 and never reused: a jog under override is already
    // outside the limits by mode.
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
                .hasCategory = true, .category = valence::ui_categories::generator,
                .hasRank = true, .rank = valence::ui_ranks::hero});
    c.addSchemaField({.key = 1, .name = "position", .type = CborFieldType::f32_t, .unit = "mm",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 2000.0f,
                      .role = roles::command_position,
                      .hasRank = true, .rank = valence::ui_ranks::hero,
                      .hasUnitId = true, .unitId = valence::unit_ids::mm});
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
    c.addSchemaField({.key = 3, .name = "jog_speed",  .type = CborFieldType::f32_t, .unit = "mm/s",
                      .hasMin = true, .hasMax = true, .min = ceiling::speed_min, .max = ceiling::speed_max});
    c.addSchemaField({.key = 4, .name = "jog_accel",  .type = CborFieldType::f32_t, .unit = "mm/s2",
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
                .hasCategory = true, .category = valence::ui_categories::generator,
                .hasRank = true, .rank = valence::ui_ranks::control});
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
    // field of the same settingKey. A standing policy, not a live pattern
    // param, but RAM-ONLY: persisting it would let a rebooted machine start
    // moving unattended, which waits on an operator ruling (bd val-wcm).
    c.addSchemaField({.key = 7, .name = "background_run", .type = CborFieldType::bool_t, .unit = ""});
    };

    // ---- "home" — INTENT, control -------------------------------------------
    // {1:"op", 2:"stroke"} — op 1 starts sensorless homing; op 2 is the
    // BENCH op (RFC-025, safety-reviewed) that makes motorless dev work
    // possible at all. No op here clears override: SPEC §11.1 leaves it only
    // by the safety-intents `return` op (RFC-085).
    //
    // *** OP 2 (force_home) RELEASES AN E-STOP LATCH INTO PAUSE. *** That is exactly why
    // RFC-025 placed these under safety review rather than in a convenience
    // bucket, and why they are `control` and rate-capped like every other op
    // here. Op 2 declares the machine homed WITHOUT a homing cycle, so the
    // stroke window it hands the arbiter is an ASSERTION, not a measurement —
    // on a machine with a motor attached that is a real collision hazard; the
    // hazard note's one home is motionForceHome() in ValenceMotion.h.
    //
    // Op values are DEVICE-defined: 0x3101 is in this device's own >=0x0080
    // allocation, so unlike 0x0005's registry-governed `safety_ops` these
    // numbers live in this catalog and nowhere else — which is precisely why
    // they carry option labels, so a generic client can name them.
    auto addHome = [&]() {
    c.addEntry({.id = ch::home, .name = "home",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 5.0f,
                .defaultPriority = Priority::normal,
                // ui_categories::hardware's own note names "homing" explicitly.
                .hasCategory = true, .category = valence::ui_categories::hardware,
                .hasRank = true, .rank = valence::ui_ranks::control});
    c.addSelectSchemaField({.key = 1, .name = "op", .type = CborFieldType::uint_t, .unit = "",
                            .role = "action.home"},
                           {"reserved", "home", "force_home"},
                           {AccessLevel::control,   // 0 (placeholder, never an op)
                            AccessLevel::control,   // 1 home
                            AccessLevel::control}); // 2 force_home  — RELEASES THE E-STOP LATCH
    c.addSchemaField({.key = 2, .name = "stroke", .type = CborFieldType::f32_t, .unit = "mm",
                      .hasMin = true, .hasMax = true, .min = 1.0f, .max = 2000.0f});
    };

    // ---- "modes-set" — INTENT, control, 5 Hz --------------------------------
    // The write half of 0x008A. Every key optional; present keys applied,
    // and the ECHO carries the POST-CLAMP value the handler actually took.
    //
    // cfg_gen bumps on a real change (SPEC §4.2); persisted with it
    // (StoredState.h).
    //
    // 5 Hz because these are human dropdown changes, not a control loop. The
    // bounds are the enum ranges the catalog's own option arrays declare, so a
    // client that validates locally gets the same answer the hub would NACK.
    auto addModesSet = [&]() {
    c.addEntry({.id = ch::modes_set, .name = "modes-set",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 5.0f,
                .defaultPriority = Priority::normal});
    // KEY 1 IS DELIBERATELY UNUSED. It held "blend_mode"; see the field
    // comment on machine-modes' `blend_mode_reserved`. ValenceDevice::
    // applyModes reads only keys 7, 8, 9 and 10, and refuses a request
    // carrying none of them with NACK(INVALID_VALUE).
    //
    // KEY 2 IS ALSO DELIBERATELY UNUSED. It briefly held "transport" (the WS/
    // SER/BT/DONGLE/OSSM input-source selector) before that setting was
    // retired: Valence is now the only way in, the hub listens on WebSocket
    // and BLE by default, and OSSM-BLE is gone. The C5 dongle may return one
    // day, but as a transport the hub simply HAS, not a mode an operator picks.
    //
    // KEY 3 IS NOW A PERMANENT GAP TOO. It held "stream_speed_mode"; see the
    // field comment on machine-modes' `stream_speed_reserved`. applyModes
    // never reads it.
    //
    // KEY 4 IS A PERMANENT GAP. It held "overshoot_clamp"; see the field
    // comment on machine-modes' `overshoot_clamp_reserved`.
    //
    // KEY 5 IS RELEASED on this board: motion_backend is read-only here (see
    // 0x1030), so there is nothing for it to write.
    //
    // Every number is skipped rather than recycled. "Released keys are never
    // reused" is only a reliable habit if it does not get relitigated per
    // case, and a gap costs nothing.
    if (feat.has_drive) {
        c.addSchemaField({.key = 6, .name = "home_style", .type = CborFieldType::uint_t, .unit = "",
                          .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    }
    c.addSchemaField({.key = 7, .name = "schedule_horizon", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = float(kHorizonMs.size() - 1)});
    c.addSchemaField({.key = 8, .name = "flipped", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    c.addSchemaField({.key = 9, .name = "home_speed", .type = CborFieldType::f32_t, .unit = "mm/s",
                      .hasMin = true, .hasMax = true, .min = ceiling::home_speed_min, .max = ceiling::speed_max});
    // RFC-053 item 3: configure tier to change, above the entry's control.
    c.addSchemaField({.key = 10, .name = "datagram_estop", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .access = AccessLevel::configure, .hasAccess = true});
    };

    // ---- "kinetic-set" — INTENT, control, 5 Hz ---------------------------
    // The single writer behind both kinetic-* cards: keys 1..8, contiguous,
    // 1-3 kinetic-limits' and 4-8 kinetic-planner's (RFC-108). Every key
    // optional, only the keys PRESENT are applied, and each echoes the value
    // the machine actually took after its own clamp. A request carrying none
    // of them NACKs INVALID_VALUE.
    //
    // Bounds mirror the engine's own clamps exactly, so a client that validates
    // locally gets the same answer the hub would NACK with. Times are
    // MILLISECONDS on the wire; the engine stores microseconds.
    auto addKineticSet = [&]() {
    c.addEntry({.id = ch::kinetic_set, .name = "kinetic-set",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 5.0f,
                .defaultPriority = Priority::normal});
    c.addSchemaField({.key = 1, .name = "jmax_ovr", .type = CborFieldType::f32_t, .unit = "1/s3",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 2000000.0f});
    c.addSchemaField({.key = 2, .name = "vmax_ovr", .type = CborFieldType::f32_t, .unit = "1/s",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 20.0f});
    c.addSchemaField({.key = 3, .name = "amax_ovr", .type = CborFieldType::f32_t, .unit = "1/s2",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 500.0f});
    c.addSchemaField({.key = 4, .name = "smoothness", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f});
    c.addSchemaField({.key = 5, .name = "handle_floor", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.05f, .max = 0.33f});
    c.addSchemaField({.key = 6, .name = "trim_max", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.1f, .max = 1.0f});
    c.addSchemaField({.key = 7, .name = "chase_dense_ms", .type = CborFieldType::f32_t, .unit = "ms",
                      .hasMin = true, .hasMax = true, .min = 10.0f, .max = 500.0f});
    c.addSchemaField({.key = 8, .name = "react_ms", .type = CborFieldType::f32_t, .unit = "ms",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
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

    // ---- "osc-set" -- INTENT, control, 20 Hz: the oscillator (RFC-103) -----
    // Every key optional; present keys applied, clamped, echoed post-clamp
    // (SPEC 9.7). The osc.* roles live here, the prose on 0x1140.
    auto addOscSet = [&]() {
    c.addEntry({.id = ch::osc_set, .name = "osc-set",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 20.0f,
                .defaultPriority = Priority::normal,
                .hasCategory = true, .category = valence::ui_categories::motion,
                .hasRank = true, .rank = valence::ui_ranks::control});
    c.addSchemaField({.key = 1, .name = "enabled", .type = CborFieldType::bool_t, .unit = "",
                      .role = roles::osc_enabled});
    c.addSchemaField({.key = 2, .name = "frequency", .type = CborFieldType::f32_t, .unit = "Hz",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::osc_max_hz,
                      .role = roles::osc_frequency, .hasUnitId = true, .unitId = valence::unit_ids::hz});
    c.addSchemaField({.key = 3, .name = "amplitude", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = 1.0f,
                      .role = roles::osc_amplitude});
    c.addSelectSchemaField({.key = 4, .name = "shape", .type = CborFieldType::uint_t, .unit = "",
                            .hasMin = true, .hasMax = true, .min = 0.0f, .max = 3.0f,
                            .role = roles::osc_shape},
                           {"sine", "square", "saw", "saw_reverse"});
    c.addSchemaField({.key = 5, .name = "dwell_crest", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::osc_dwell_max,
                      .role = roles::osc_dwell_crest});
    c.addSchemaField({.key = 6, .name = "dwell_trough", .type = CborFieldType::f32_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = ceiling::osc_dwell_max,
                      .role = roles::osc_dwell_trough});
    // SPEC 9.7 driven parameters: keys 7..11 frequency's, 12..16 amplitude's.
    for (const OscDriveCard& d : kOscDriveCards) {
        c.addSelectSchemaField({.key = d.key0, .name = d.drive, .type = CborFieldType::uint_t, .unit = "",
                                .hasMin = true, .hasMax = true, .min = 0.0f, .max = 3.0f,
                                .role = d.roles[0]},
                               {"fixed", "speed", "position", "axis"});
        c.addSchemaField({.key = uint8_t(d.key0 + 1), .name = d.in_min, .type = CborFieldType::f32_t, .unit = "",
                          .role = d.roles[1]});
        c.addSchemaField({.key = uint8_t(d.key0 + 2), .name = d.in_max, .type = CborFieldType::f32_t, .unit = "",
                          .role = d.roles[2]});
        c.addSchemaField({.key = uint8_t(d.key0 + 3), .name = d.out_min, .type = CborFieldType::f32_t,
                          .unit = d.unit, .hasMin = true, .hasMax = true, .min = 0.0f, .max = d.top,
                          .role = d.roles[3]});
        c.addSchemaField({.key = uint8_t(d.key0 + 4), .name = d.out_max, .type = CborFieldType::f32_t,
                          .unit = d.unit, .hasMin = true, .hasMax = true, .min = 0.0f, .max = d.top,
                          .role = d.roles[4]});
    }
    };

    // ---- "machine-admin" — INTENT, control ----------------------------------
    // The device ACTIONS that are not settings and not motion: clear a driver
    // fault, persist config, kick off a servo register scan. They were HTTP
    // writers (/api/clearfault, WS_OP_SAVE, POST /api/servo {"scan":true});
    // "no controls outside Valence" retires all three.
    //
    // An op SELECT rather than one channel per verb, exactly like 0x0103 home:
    // these are rare, human-initiated, and share a shape. 2 Hz because a human
    // presses them; a client that needs to press one faster than twice a second
    // is doing something the machine should not help with.
    //
    // No op here is destructive (RFC-063, SPEC 8.8), so destructive_options
    // stays absent: clear_fault drops a latch that re-asserts if the fault is
    // still there, save_config commits values every one of which stays
    // writable from a client, and servo_scan only queues register reads. An op
    // that loses state a client cannot restore (a factory reset, a drive
    // re-address) sets its bit in destructive_options when it is added.
    auto addMachineAdmin = [&]() {
    c.addEntry({.id = ch::machine_admin, .name = "machine-admin",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 2.0f,
                .defaultPriority = Priority::normal,
                // clear_fault/servo_scan are hardware-adjacent (2 of 3 ops);
                // save_config rides along on the same rare-admin-action channel.
                .hasCategory = true, .category = valence::ui_categories::hardware,
                .hasRank = true, .rank = valence::ui_ranks::control});
    c.addSelectSchemaField({.key = 1, .name = "op", .type = CborFieldType::uint_t, .unit = "",
                            .role = "action.admin"},
                           {"reserved", "clear_fault", "save_config", "servo_scan"},
                           {AccessLevel::control,   // 0 placeholder, never an op
                            AccessLevel::control,   // 1 clear_fault
                            AccessLevel::control,   // 2 save_config
                            AccessLevel::control}); // 3 servo_scan
    };

    // ---- "pattern-advanced-cmd" — INTENT, control, 20 Hz --------------------
    // The single writer behind ALL NINE 0x1210..0x1218 advanced-pattern
    // cards. Same lean-schema convention as every other settings writer in
    // this catalog (config_set, pattern_cmd, modes_set, kinetic_set): the
    // user-facing text (desc/group/default/role) lives ONCE, on the STATE
    // side, so this channel carries only what a client needs to validate
    // before sending — name, type, unit, bounds.
    //
    // Keys 2..8 mirror 0x1210's master and percent knobs, 46..47 its dwells
    // (RFC-095). Keys 9..44 and 48..59 are 6-per-control modulator blocks at
    // apModKeyBase(id), id in advpat::BaseId order, matching 0x1211..0x1218;
    // ValenceDevice.cpp's applyPatternAdvanced() runs the same arithmetic in
    // reverse. Key 1 (the retired mode switch, RFC-093) is a permanent gap;
    // key 45 is `running`.
    //
    // Session-volatile, same as 0x0102 pattern-cmd: cfg_gen does not bump.
    auto addPatternAdvancedCmd = [&]() {
    c.addEntry({.id = ch::pattern_advanced_cmd, .name = "pattern-advanced-cmd",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 20.0f,
                .defaultPriority = Priority::normal});
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
    c.addSchemaField({.key = apBaseKey(6), .name = "dwell_crest", .type = CborFieldType::f32_t,
                      .unit = "strokes", .hasMin = true, .hasMax = true, .min = 0.0f, .max = 655.35f});
    c.addSchemaField({.key = apBaseKey(7), .name = "dwell_trough", .type = CborFieldType::f32_t,
                      .unit = "strokes", .hasMin = true, .hasMax = true, .min = 0.0f, .max = 655.35f});
    // 6 keys per modulator (advpat::BaseId order): amount, in_step, in_wait,
    // out_step, out_wait, offset, from apModKeyBase(id). Authoring order
    // doesn't need to be ascending here (the encoder sorts schema fields by
    // key before emitting), so a loop is safe where it would not be for the
    // addEntry() ordering above.
    for (uint8_t id = 0; id < kApBaseCount; ++id) {
        const uint8_t base = apModKeyBase(id);
        c.addSchemaField({.key = uint8_t(base + 0), .name = "amount",    .type = CborFieldType::uint_t,
                          .unit = "%", .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
        c.addSchemaField({.key = uint8_t(base + 1), .name = "in_step",   .type = CborFieldType::uint_t,
                          .unit = "strokes", .hasMin = true, .hasMax = true, .min = 1.0f, .max = 25.0f});
        c.addSchemaField({.key = uint8_t(base + 2), .name = "in_wait",   .type = CborFieldType::uint_t,
                          .unit = "strokes", .hasMin = true, .hasMax = true, .min = 0.0f, .max = 25.0f});
        c.addSchemaField({.key = uint8_t(base + 3), .name = "out_step",  .type = CborFieldType::uint_t,
                          .unit = "strokes", .hasMin = true, .hasMax = true, .min = 1.0f, .max = 25.0f});
        c.addSchemaField({.key = uint8_t(base + 4), .name = "out_wait",  .type = CborFieldType::uint_t,
                          .unit = "strokes", .hasMin = true, .hasMax = true, .min = 0.0f, .max = 25.0f});
        c.addSchemaField({.key = uint8_t(base + 5), .name = "offset",    .type = CborFieldType::uint_t,
                          .unit = "strokes", .hasMin = true, .hasMax = true, .min = 0.0f, .max = 100.0f});
    }
    c.addSchemaField({.key = kApRunKey, .name = "running", .type = CborFieldType::bool_t, .unit = ""});
    };

    // ---- "pattern-presets-cmd" — INTENT, control ----------------------------
    // The CRUD writer behind the 0x5220 store / 0x1220 roster pair (RFC-021).
    // {1:"op", 2:"slot", 3:"name", 4:"item"}. `op` is the registered
    // action.store op select (RFC-067): options index-aligned with registry
    // store_ops, no option beyond them, index 0 the mandatory non-empty
    // filler (SPEC 8.9). delete is destructive by registration (SPEC 8.8), so
    // no mask restates it. slot, name and item carry the store.* roles and
    // the per-verb set of SPEC 8.7 (RFC-089); ValenceDevice::applyPresets
    // enforces it and PatternPresetStore.h is the backend.
    auto addPatternPresetsCmd = [&]() {
    c.addEntry({.id = ch::pattern_presets_cmd, .name = "pattern-presets-cmd",
                .cls = ChannelClass::INTENT, .dir = Direction::c2h,
                .access = AccessLevel::control, .maxRateHz = 5.0f,
                .defaultPriority = Priority::normal,
                .hasStoreId = true, .storeId = kPresetStoreId});
    c.addSelectSchemaField({.key = 1, .name = "op", .type = CborFieldType::uint_t, .unit = "",
                            .role = "action.store"},
                           {"reserved", "save", "load", "delete", "rename"});
    c.addSchemaField({.key = 2, .name = "slot", .type = CborFieldType::uint_t, .unit = "",
                      .hasMin = true, .hasMax = true, .min = 0.0f, .max = float(kPresetCapacity - 1),
                      .role = roles::store_slot});
    c.addSchemaField({.key = 3, .name = "name", .type = CborFieldType::tstr_t, .unit = "",
                      .role = roles::store_name});
    c.addSchemaField({.key = 4, .name = "item", .type = CborFieldType::bstr_t, .unit = "",
                      .role = roles::store_item});
    };

    // ---- invoke every device-channel builder above in ASCENDING NEW-ID ORDER --
    // The authoring order above does not match wire order, so each entry's
    // build logic is wrapped in a lambda (`add*`) and the calls below are the
    // one place that has to stay ascending (encodeCatalog/etag require it,
    // §8.3). Core channels (0x0003-0x000E) are unaffected: they were already
    // emitted above, in order, before any device channel. The eight AP-modifier
    // calls are in MEMBER order (speedin/out, accelin/out, depth1/2,
    // crest/trough), NOT
    // advpat::BaseId order — see the ch:: namespace comment on those
    // constants.
    //
    // The three feature flags gate every call below. The gates are HERE and
    // not inside each lambda so the surviving set is readable as a list: an id
    // this block skips does not exist on that hub, and its absence IS the
    // capability answer (SPEC §6.3). Ascending order is preserved either way --
    // skipping entries never reorders the rest.
    addMachineConfig();          // 0x1000 STATE·machine, family 0 member 0
    addPower();                  // 0x1010 STATE·machine, family 1 member 0 (no-ops without feat.has_current_sensor)
    if (feat.has_motion) {
        addOdometer();           // 0x1020 STATE·machine, family 2 member 0
        addMachineModes();       // 0x1030 STATE·machine, family 3 member 0
        addMotion();             // 0x1100 STATE·motion, family 0 member 0
        addPlanStrip();          // 0x1110 STATE·motion, family 1 member 0
        addMotionDiag();         // 0x1111 STATE·motion, family 1 member 1
        addKineticLimits();      // 0x1120 STATE·motion, family 2 member 0
        addKineticPlanner();     // 0x1122 STATE·motion, family 2 member 2
    }
    if (feat.has_drive) addDriveTune();   // 0x1130 STATE·motion, family 3 member 0
    if (feat.has_motion) addOscillator(); // 0x1140 STATE·motion, family 4 member 0
    if (feat.has_pattern) {
        addPatternState();       // 0x1200 STATE·pattern, family 0 member 0
        addPatternAdvanced();    // 0x1210 STATE·pattern, family 1 member 0
        addApModifierChannel(ch::pattern_adv_mod_speedin,  "pattern-adv-mod-speedin",  "Speed in modifier",  2);  // 0x1211
        addApModifierChannel(ch::pattern_adv_mod_speedout, "pattern-adv-mod-speedout", "Speed out modifier", 3);  // 0x1212
        addApModifierChannel(ch::pattern_adv_mod_accelin,  "pattern-adv-mod-accelin",  "Accel in modifier",  4);  // 0x1213
        addApModifierChannel(ch::pattern_adv_mod_accelout, "pattern-adv-mod-accelout", "Accel out modifier", 5);  // 0x1214
        addApModifierChannel(ch::pattern_adv_mod_depth1,   "pattern-adv-mod-depth1",   "Depth 1 modifier",   0);   // 0x1215
        addApModifierChannel(ch::pattern_adv_mod_depth2,   "pattern-adv-mod-depth2",   "Depth 2 modifier",   1);   // 0x1216
        addApModifierChannel(ch::pattern_adv_mod_crest,    "pattern-adv-mod-crest",    "Crest dwell modifier",  6);  // 0x1217
        addApModifierChannel(ch::pattern_adv_mod_trough,   "pattern-adv-mod-trough",   "Trough dwell modifier", 7);  // 0x1218
        addPatternPresetsRoster();  // 0x1220 STATE·pattern, family 2 member 0
    }
    if (feat.has_motion) {
        addMotionInput();        // 0x2100 STREAM·motion, family 0 member 0
        addMotionSegment();      // 0x2101 STREAM·motion, family 0 member 1
        addOscDrive();           // 0x2140 STREAM·motion, family 4 member 0
    }
    addConfigSet();              // 0x3000 INTENT·machine, family 0 member 0
    if (feat.has_motion) addModesSet();      // 0x3030 INTENT·machine, family 3 member 0
    if (feat.has_drive)  addMachineAdmin();  // 0x30F0 INTENT·machine, family F member 0
    if (feat.has_motion) {
        addMove();               // 0x3100 INTENT·motion, family 0 member 0
        addHome();               // 0x3101 INTENT·motion, family 0 member 1
        addKineticSet();         // 0x3120 INTENT·motion, family 2 member 0
    }
    if (feat.has_drive) addDriveSet();   // 0x3130 INTENT·motion, family 3 member 0
    if (feat.has_motion) addOscSet();    // 0x3140 INTENT·motion, family 4 member 0
    if (feat.has_pattern) {
        addPatternCmd();         // 0x3200 INTENT·pattern, family 0 member 0
        addPatternAdvancedCmd(); // 0x3210 INTENT·pattern, family 1 member 0
        addPatternPresetsCmd();  // 0x3220 INTENT·pattern, family 2 member 0
    }
    if (feat.has_motion) addMotionAnomaly();    // 0x4100 EVENT·motion, family 0 member 0
    if (feat.has_pattern) addPatternPresets();  // 0x5220 STORE·pattern, family 2 member 0

    return c.ok();
}

}  // namespace valence
