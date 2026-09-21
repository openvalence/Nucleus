// ValenceHub -- composition root for the SlopSync hub on the P4
// Constraints:
// - See ValenceHub.h for the single-task, PSRAM and construction-order rules.
// - Every static_assert below pins ValenceCatalog.h's hand-mirrored defaults
//   to valence_config.h. The catalog is library-only and cannot include the
//   config header; this TU sees both, so it is where the mirror is nailed.
// - The delegate is HONEST OR ABSENT: an intent it cannot really apply is
//   NACKed UNSUPPORTED_OP (0x0303, registry.yaml:609), never NOT_HOMED --
//   there is no homing on this board to be waiting for.
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
// See: SlopSync SPEC.md §4.2, §6.3, §9.1, §9.3

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
#include "vlog/vlog.h"
#include "ValencePlatform.h"
#include "ValenceUiToken.h"
#include "ValenceWsPort.h"
#include "motion/ValenceMotion.h"
#include "valence_config.h"

#include "slopsync/util/byte_io.hpp"

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

using slopsync::AccessLevel;
using slopsync::IntentValue;
using slopsync::IntentValueMap;
using slopsync::NackCode;
using Ret = slopsync::Result<IntentValueMap, NackCode>;

constexpr const char* kTag = "hub";

// The SlopSync socket. 82 on every machine in this ecosystem; /uitoken rides
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

const slopsync::IntentValueField* findField(const IntentValueMap& m, uint8_t key) {
    for (uint32_t i = 0; i < m.count; ++i) {
        if (m.fields[i].key == key) return &m.fields[i];
    }
    return nullptr;
}

float fieldF32(const slopsync::IntentValueField* f, float dflt) {
    if (!f) return dflt;
    switch (f->value.kind) {
        case IntentValue::Kind::F32:  return f->value.f32_val;
        case IntentValue::Kind::U64:  return float(f->value.u64_val);
        case IntentValue::Kind::I64:  return float(f->value.i64_val);
        default:                      return dflt;
    }
}

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
        SLOGW(kTag, "stored config rejected (magic/version/range) -- factory defaults stand");
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
        SLOGW(kTag, "config persist: nvs_open failed");
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
    if (err != ESP_OK) SLOGW(kTag, "config persist failed: %s", esp_err_to_name(err));
    else SLOGI(kTag, "config persisted, cfg_gen=%u, %lu us on the hub task",
               unsigned(gen), static_cast<unsigned long>(us));
}

// ---- the delegate ------------------------------------------------------------
// Two pure virtuals and nothing else is overridden that this board cannot back
// with real behavior. Every other channel the catalog advertises is either
// hub-owned (safety latch, pairing, session admin) or gated out by
// has_motion=false.
class ValenceDelegate final : public slopsync::HubDelegate {
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
        if (channel_id != ch::config_set) {
            // THE SEAM FOR ch::move (0x3100), and it is deliberately not code
            // yet. The motion path exists and works (motion/ValenceMotion.h),
            // but DeviceFeatures::has_motion is false, so ch::move is not in
            // the catalog and an intent on it cannot arrive: a branch here
            // would be unreachable by construction. When a real drive and
            // encoder land and has_motion flips on, this becomes
            //   if (channel_id == ch::move) {
            //       MotionIntent in{MotionSource::Stream, <target mm>, ...};
            //       return motionSubmit(in) ? Ret::ok(...)
            //                               : Ret::err(NackCode::...);
            //   }
            // and nothing else here changes -- the arbiter already owns the
            // gates, the clamp and the limit set.
            //
            // Includes every op on 0x0005 that the hub does not handle itself
            // (stop / hold / pause / resume / override / bypass): each one names
            // a motion-plane behavior this board does not yet expose, and the
            // library's own contract is that an unimplemented op returns
            // UNSUPPORTED_OP so the hub latches NOTHING. estop and estop_clear
            // never reach here -- the hub owns both -- so the red button works
            // regardless.
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

    // §11.2: motion stops before protocol bookkeeping. The emitter is parked on
    // THIS task inside motionEstop(), before this returns, so the stop precedes
    // the latch as the spec requires -- and it does so whether or not the
    // catalog advertises a motion channel, because e-stop is never gated by a
    // capability flag.
    void onEstop(uint8_t cause, uint8_t origin) override {
        motionEstop();
        SLOGW(kTag, "ESTOP latched: cause=%u origin=%u", unsigned(cause), unsigned(origin));
    }

    void onSessionJoined(uint32_t session_id) override {
        SLOGI(kTag, "session %lu joined", static_cast<unsigned long>(session_id));
    }
    void onSessionLeft(uint32_t session_id) override {
        SLOGI(kTag, "session %lu left", static_cast<unsigned long>(session_id));
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
    ValenceUiTokenMinter* _minter = nullptr;
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
    slopsync::Catalog32 catalog{};
    EspClock clock{};
    EspRandom rng{};
    ValenceDelegate delegate{};
    std::optional<slopsync::Hub> hub{};
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
// value, and SlopDeck sits at 'syncing' forever with nothing to report.
// The hub seeds 0x0003 safety, 0x000A pending-pairing and 0x000D roster
// itself (hub_impl.hpp's constructor); these are the rest.

void publishControlOwner() {
    // 4 x {source u8, owner u32} ascending, exactly Hub::buildControlOwnerPayload.
    // Every source unowned at boot, which is the truth: with no motion plane
    // there is nothing for a session to own, and the delegate maps no channel
    // to a source, so this value never changes.
    std::array<std::byte, 20> buf{};
    std::span<std::byte> s(buf);
    for (uint8_t i = 0; i < 4; ++i) {
        slopsync::putU8(s.subspan(size_t(i) * 5, 1), i);
        slopsync::putU32(s.subspan(size_t(i) * 5 + 1, 4), 0);
    }
    g_box->hub->publishState(slopsync::channels::control_owner, s);
}

void publishHubStatus() {
    // 4+4+1+1+4 = 14 B, matching the 0x0007 layout in ValenceCatalog.h.
    // RSSI is PUSHED IN from main's slow loop, never read here -- see
    // hubSetLinkRssi() in ValenceHub.h for the 151 ms that buys.
    const int8_t rssi = g_linkRssi.load(std::memory_order_relaxed);

    std::array<std::byte, 14> buf{};
    std::span<std::byte> s(buf);
    slopsync::putU32(s.subspan(0, 4), uint32_t(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    slopsync::putU32(s.subspan(4, 4), uint32_t(esp_timer_get_time() / 1000000));
    slopsync::putU8(s.subspan(8, 1), uint8_t(rssi));
    slopsync::putU8(s.subspan(9, 1), uint8_t(g_box->hub->sessionCount()));
    slopsync::putU32(s.subspan(10, 4), g_box->hub->logDropped());
    g_box->hub->publishState(slopsync::channels::hub_status, s);
}

void publishMachineConfig() {
    // 37 B, matching the 0x1000 layout in ValenceCatalog.h.
    const StoredConfig& c = g_box->delegate.config();
    std::array<std::byte, 37> buf{};
    std::span<std::byte> s(buf);
    slopsync::putF32(s.subspan(0, 4), c.window_min);
    slopsync::putF32(s.subspan(4, 4), c.window_max);
    slopsync::putF32(s.subspan(8, 4), c.user_speed);
    slopsync::putF32(s.subspan(12, 4), c.user_accel);
    slopsync::putF32(s.subspan(16, 4), c.input_speed);
    slopsync::putF32(s.subspan(20, 4), c.input_accel);
    slopsync::putF32(s.subspan(24, 4), c.max_rail);
    slopsync::putF32(s.subspan(28, 4), c.input_jerk);
    // enabled_mask: all eight limits are writable at all times on this hub.
    // Nothing refuses a config-set; out-of-range values clamp, which is what
    // min/max is for. A bit held low here would gray a control the machine
    // would in fact accept.
    slopsync::putU8(s.subspan(32, 1), 0xFF);
    // measured_stroke: 0 means NOT MEASURED, and on a board with no motion
    // plane that is the whole truth. Never report the configured rail here.
    slopsync::putF32(s.subspan(33, 4), 0.0f);
    g_box->hub->publishState(ch::machine_config, s);
    g_lastPublishedCfg = c;
    g_cfgEverSent = true;
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
        SLOGI(kTag, "minted hub_instance_id %08lx%08lx",
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

        if (g_box->delegate.takeConfigDirty() ||
            (g_cfgEverSent && !(g_lastPublishedCfg == g_box->delegate.config()))) {
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

        vlog::drainToSinks();
    }
}

}  // namespace

// ---- public surface ----------------------------------------------------------

slopsync::Hub* hub() { return (g_box && g_box->hub) ? &*g_box->hub : nullptr; }

void hubSetLinkRssi(int8_t rssi) { g_linkRssi.store(rssi, std::memory_order_relaxed); }

HubCensus hubCensus() {
    HubCensus c{};
    if (!g_box || !g_box->hub) return c;
    c.sessions = uint32_t(g_box->hub->sessionCount());
    c.ticks = g_ticks;
    c.wsFrames = g_box->port.framesRx();
    c.wsDrops = g_box->port.drops();
    c.stackFree = g_hubTask ? uint32_t(uxTaskGetStackHighWaterMark(g_hubTask)) : 0;
    return c;
}

bool hubBegin() {
    vlog::logBegin();

    // PSRAM, via placement-new (T2). The catalog alone is ~22 KB of pooled
    // field storage and the hub carries the session table and the retained
    // store; in .bss that is internal RAM the network stack allocates from at
    // runtime. The TASK STACK below stays internal on purpose.
    void* mem = heap_caps_malloc(sizeof(HubBox), MALLOC_CAP_SPIRAM);
    if (mem == nullptr) {
        SLOGE(kTag, "PSRAM alloc of %u bytes for the hub failed", unsigned(sizeof(HubBox)));
        vlog::drainToSinks();
        return false;
    }
    g_box = new (mem) HubBox();
    g_box->minter.begin();
    g_box->delegate.bindMinter(&g_box->minter);

    DeviceFeatures feat{};
    feat.has_motion = false;  // no motion plane on this board yet (sd/val-091.3)
    if (!buildValenceCatalog(g_box->catalog, feat)) {
        SLOGE(kTag, "catalog build overflowed a Catalog32 pool");
        vlog::drainToSinks();
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

    g_box->hub.emplace(g_box->catalog, g_box->clock, g_box->rng, g_box->delegate);
    // cfg_gen survives the reboot with the values it belongs to (§4.2). The
    // library exposes advance-only (bumpConfigGeneration), which is correct for
    // its RFC-011 job, so the restore walks the u16 up to the stored value; it
    // terminates by wrapping and costs one increment per step, nothing more.
    if (haveStored) {
        while (g_box->hub->cfgGen() != storedGen) g_box->hub->bumpConfigGeneration();
        SLOGI(kTag, "config adopted from NVS, cfg_gen=%u", unsigned(storedGen));
    }
    if (g_box->hub->catalogEncodedBytes() == 0) {
        SLOGE(kTag, "catalog encoded to ZERO bytes -- it did not fit the hub scratch (%u B)",
              unsigned(slopsync::Hub::catalogScratchCapacity()));
        vlog::drainToSinks();
        return false;
    }
    g_box->hub->setIdentity(VALENCE_PRODUCT, FIRMWARE_VERSION, VALENCE_HUB_NAME);
    g_box->hub->setHubInstanceId(loadOrMintInstanceId());
    refreshEndpoint();

    publishControlOwner();
    publishMachineConfig();
    publishHubStatus();

    auto etag = g_box->hub->catalogEtag();
    SLOGI(kTag, "catalog: %u entries, %u B encoded (scratch %u B)",
          unsigned(g_box->catalog.count), unsigned(g_box->hub->catalogEncodedBytes()),
          unsigned(slopsync::Hub::catalogScratchCapacity()));
    SLOGI(kTag, "catalog etag: %02x%02x%02x%02x%02x%02x%02x%02x",
          unsigned(etag[0]), unsigned(etag[1]), unsigned(etag[2]), unsigned(etag[3]),
          unsigned(etag[4]), unsigned(etag[5]), unsigned(etag[6]), unsigned(etag[7]));
    SLOGI(kTag, "hub box %u B in PSRAM, boot_id=%08lx, %s",
          unsigned(sizeof(HubBox)), static_cast<unsigned long>(g_box->hub->bootId()),
          FIRMWARE_VERSION);

    if (!g_box->port.begin(&*g_box->hub, kWsPort)) {
        SLOGE(kTag, "WS port failed to start on :%u", unsigned(kWsPort));
        vlog::drainToSinks();
        return false;
    }
    // Non-fatal: a hub with no /uitoken still serves every watch-tier client
    // and every paired one. Losing the mint costs the browser onramp, not the
    // machine.
    if (!g_box->minter.attachRoutes()) SLOGW(kTag, "/uitoken unavailable");

    // Stack: internal by construction (plain xTaskCreatePinnedToCore). The size
    // and the measurement that set it live on kHubTaskStackBytes in ValenceHub.h.
    if (xTaskCreatePinnedToCore(hubTask, "SlopHub", kHubTaskStackBytes, nullptr, 5,
                                &g_hubTask, 1) != pdPASS) {
        SLOGE(kTag, "hub task create failed");
        vlog::drainToSinks();
        return false;
    }
    vlog::drainToSinks();
    return true;
}

}  // namespace valence
