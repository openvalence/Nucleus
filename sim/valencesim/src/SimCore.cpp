// SimCore -- implementation. See SimCore.h; nothing is restated here.

#include "SimCore.h"

#include <cmath>
#include <cstdio>
#include <string_view>

#include "SimMotion.h"
#include "SimMotorSwitch.h"
#include "SimPattern.h"
#include "hub/valence_config.h"
#include "motion/MotionArbiter.h"
#include "motion/ValenceMotion.h"
#include "system/ValenceButtons.h"
#include "system/ValenceDriveLink.h"
#include "system/ValenceEstopInput.h"
#include "system/ValenceMotorSwitch.h"

namespace valence {

uint32_t deviceFreeHeapBytes() { return 0; }
// The twin has no buttons: the pairing_window option stands in for PAIR, and
// HOME's reboot has no meaning for a desktop process.
button::Gesture homeButtonTake() { return button::Gesture::none; }
button::Gesture pairButtonTake() { return button::Gesture::none; }
// The twin has no drive: DRV_ALM never asserts.
bool driveAlarmTake() { return false; }
// The twin has no e-stop wired: it reads present and released, always.
estop::Reading estopInputRead() {
    estop::Reading r;
    r.known = true;
    return r;
}

void SimCore::GeigerToSessionLog::write(const geiger::Record& r) {
    const auto s = static_cast<unsigned long>(r.ms / 1000u);
    const auto ms = static_cast<unsigned long>(r.ms % 1000u);
    if (r.lost != 0) {
        _log.logf(geiger::levelChar(r.level), "%7lu.%03lu %-10s %s  (+%u lost)", s, ms, r.tag, r.msg,
                  unsigned(r.lost));
    } else {
        _log.logf(geiger::levelChar(r.level), "%7lu.%03lu %-10s %s", s, ms, r.tag, r.msg);
    }
}

void SimCore::GeigerToLogChannel::write(const geiger::Record& r) {
    if (hub == nullptr) return;
    // geiger::Level and the registry's log_levels share one numbering.
    hub->publishLog(uint8_t(r.level), std::string_view(r.tag), std::string_view(r.msg));
}

// The P4's loadOrMintInstanceId(): minted once, never 0, kept across boots. A
// blob that is not exactly 8 nonzero bytes is replaced by a fresh mint.
uint64_t SimCore::loadOrMintInstanceId() {
    std::array<std::byte, 8> raw{};
    const std::span<const std::byte> got = _store->load(SimBlob::iid, raw);
    uint64_t id = 0;
    if (got.size() == raw.size()) {
        for (size_t i = 0; i < raw.size(); ++i) id |= uint64_t(got[i]) << (8 * i);
        if (id != 0) return id;
    }
    do {
        id = (uint64_t(_rng.nextU32()) << 32) | _rng.nextU32();
    } while (id == 0);
    for (size_t i = 0; i < raw.size(); ++i) raw[i] = std::byte((id >> (8 * i)) & 0xFF);
    if (!_store->save(SimBlob::iid, raw))
        log.logf('W', "valencesim: persist %s identity failed -- it lasts this run only", _storeName);
    log.logf('I', "valencesim: %s held no identity -- minted a new one", _storeName);
    return id;
}

bool SimCore::begin(const SimConfig& opt, IClock& clock, ISimStore& store) {
    _store = &store;
    _storeName = opt.storeName;
    // Declared before the first GLOG can fire; both outlive every drain.
    geiger::logger().addSink(&_geigerSink);
    if (!geiger::logger().addSink(&_logChannel, geiger::Level::Warn))
        log.logf('W', "valencesim: log channel bridge not registered: Geiger sink table full");

    if (opt.homeSenseAtMm) {
        simMotionSetHomeSenseAt(*opt.homeSenseAtMm);
        log.logf('W', "valencesim: --home-sense-at: home stop at %.1f mm from the boot position",
                 double(*opt.homeSenseAtMm));
    }
    if (opt.planDelayMs) {
        simMotionSetPlanDelayMs(opt.planDelayMs);
        log.logf('W', "valencesim: --plan-delay-ms: every solve lands %lu ms after it starts",
                 static_cast<unsigned long>(opt.planDelayMs));
    }
    motionBegin();
    if (opt.motorSwitch) simMotorSwitchModel();
    motorSwitchBegin();
    patternBegin();

    if (!buildValenceCatalog(_catalog, boardFeatures())) {
        std::fprintf(stderr, "valencesim: catalog build overflowed a Catalog32 pool\n");
        return false;
    }
    _device.bindTokenGate(&_minter);
    // Stored state BEFORE the Hub, exactly as the P4's hubBegin orders it.
    std::span<std::byte> scratch(_scratch);
    uint16_t storedGen = 0;
    std::span<const std::byte> blob = store.load(SimBlob::cfg, scratch);
    const bool haveStored = !blob.empty() && _device.adoptConfigBlob(blob, storedGen);
    if (!blob.empty() && !haveStored) log.logf('W', "valencesim: stored cfg rejected -- factory values stand");
    blob = store.load(SimBlob::presets, scratch);
    if (!blob.empty() && !_device.adoptPresetsBlob(blob))
        log.logf('W', "valencesim: stored presets rejected -- preset store starts empty");
    // The twin is a COMMISSIONED machine whatever its stored state says, so
    // every client test can stream at once; uncommissioned is the first-run
    // hub (RFC-079), which refuses stream and pattern motion until config-set
    // writes have carried all eight keys.
    _device.setSetupWritten(opt.uncommissioned ? 0 : kSetupRequiredMask);
    if (opt.uncommissioned) log.logf('W', "valencesim: --uncommissioned: first-run hub, setup record cleared");
    // The far stop defaults to the stored max_rail plus both safety margins
    // from the home stop, on the other side of the boot position: a usable rail
    // exactly as long as the setting, so a cycle stores what it found.
    if (opt.homeSenseAtMm) {
        const float stops = _device.config().max_rail + 2.0f * kHomeSafetyMarginMm;
        const float farAt = opt.railEndAtMm.value_or(*opt.homeSenseAtMm < 0.0f ? *opt.homeSenseAtMm + stops
                                                                               : *opt.homeSenseAtMm - stops);
        simMotionSetRailEndAt(farAt);
        const float between = std::fabs(farAt - *opt.homeSenseAtMm);
        log.logf('W', "valencesim: --rail-end-at: far stop at %.1f mm from the boot position, %.1f mm stop to stop, "
                 "usable rail %.1f mm", double(farAt), double(between),
                 double(between - 2.0f * kHomeSafetyMarginMm));
    }
    _device.pushConfigToMotion();

    _hub.emplace(_catalog, clock, _rng, _device);
    Hub& hub = *_hub;
    if (hub.catalogEncodedBytes() == 0) {
        std::fprintf(stderr, "valencesim: catalog encoded to ZERO bytes (scratch %u B)\n",
                     unsigned(Hub::catalogScratchCapacity()));
        return false;
    }
    if (haveStored) {
        while (hub.cfgGen() != storedGen) hub.bumpConfigGeneration();
    }
    log.logf('I', "valencesim: state %s: config %s, cfg_gen %u", opt.storeName, haveStored ? "stored" : "factory",
             unsigned(hub.cfgGen()));
    hub.setIdentity(VALENCE_PRODUCT, FIRMWARE_MAJOR_MINOR, kHubName);
    // No motor switch on a desktop by default: ESTOP is a halt that keeps
    // home (SPEC 11.2). motorSwitch models the board's, which cuts power.
    hub.setEstopCutsPower(opt.motorSwitch);
    hub.setHubInstanceId(loadOrMintInstanceId());
    log.logf('I', "valencesim: hub_instance_id %016llx", static_cast<unsigned long long>(hub.hubInstanceId()));
    _device.attach(hub, _catalog);
    _logChannel.hub = &hub;

    const auto etag = hub.catalogEtag();
    std::array<char, 2 * 32 + 1> etagHex{};
    for (size_t i = 0; i < etag.size() && i < 32; ++i)
        std::snprintf(etagHex.data() + 2 * i, 3, "%02x", unsigned(etag[i]));
    log.logf('I', "valencesim: %s %s, catalog %u entries, %u B, etag %s", VALENCE_PRODUCT, FIRMWARE_VERSION,
             unsigned(_catalog.count), unsigned(hub.catalogEncodedBytes()), etagHex.data());
    const CatalogHeadroom room = catalogHeadroom(_catalog, hub.catalogEncodedBytes());
    log.logf('I', "valencesim: accessory headroom: %u accessories; free %u entries, %u layout, "
             "%u schema, %u safe, %lu B", unsigned(room.accessories), unsigned(room.entries),
             unsigned(room.layout), unsigned(room.schema), unsigned(room.safe),
             static_cast<unsigned long>(room.bytes));

    // The board's boot hands the switch the self-check's verdict; the twin's
    // board always passes it. Without the model this changes nothing.
    motorSwitchSetSelfCheck(true);
    if (opt.motorSwitch) log.logf('W', "valencesim: --motor-switch: switch modeled, estop_cuts_power true, enabling");
    if (opt.motorSwitch && opt.mswFaultS >= 0) {
        const uint64_t from = deviceNowUs() + uint64_t(opt.mswFaultS) * 1000000u;
        simMotorSwitchInjectFault(from, from + 2000000u);
        log.logf('W', "valencesim: --msw-fault: MSW_FLT_N low from +%d s for 2 s", opt.mswFaultS);
    }
    if (opt.homed) {
        const float stroke = motionForceHome(_device.config().max_rail);
        log.logf('W', "valencesim: --homed: force_home asserted, stroke %.1f mm", double(stroke));
    }
    if (opt.pairingWindow) {
        hub.openPresenceWindow();
        log.logf('W', "valencesim: --pairing-window: presence window open (the gesture's twin)");
    }
    return true;
}

void SimCore::pass(uint64_t nowUs, const std::function<void(uint32_t)>& pump) {
    const uint32_t nowMs = uint32_t(nowUs / 1000);
    // Generator first, so a stroke it emits is planned on this same pass, as
    // the P4's motion task plans at arrival.
    simPatternTick(nowUs);
    simMotorSwitchTick(nowUs);
    simMotionTick(nowUs);
    if (uint32_t(nowMs - _lastHubMs) < 5u) return;
    _lastHubMs = nowMs;
    Hub& hub = *_hub;
    pump(nowMs);
    hub.update(uint32_t(nowUs));
    const uint8_t due = _device.tick(nowMs);
    geiger::drainToSinks();
    std::span<std::byte> scratch(_scratch);
    if (due & kPersistConfig) {
        const size_t n = _device.encodeConfigBlob(scratch, hub.cfgGen());
        if (!_store->save(SimBlob::cfg, scratch.first(n))) log.logf('W', "valencesim: persist cfg failed");
    }
    if (due & kPersistPresets) {
        const size_t n = _device.encodePresetsBlob(scratch);
        if (!_store->save(SimBlob::presets, scratch.first(n))) log.logf('W', "valencesim: persist presets failed");
    }
}

}  // namespace valence
