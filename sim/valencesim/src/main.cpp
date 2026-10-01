// valencesim -- the Nucleus device twin on a desktop: the REAL valence::Hub,
// the REAL device catalog and delegate (flagship_p4/src/hub/ValenceDevice),
// and the REAL kinetic::Engine behind a WebSocket speaking valence.v1
//
//   valencesim [machine] [--port 82] [--http 80] [--homed] [--duration S]
//              [--pairing-window] [--enforce] [--state PREFIX]
//              [--headless] [--no-mdns]
//
// Constraints:
// - HOST-ONLY: never touches a device, never deploys, no pio.
// - ONE hub thread. The Hub, ValenceDevice, SimMotion and SimPattern are
//   called from the loop below and nowhere else (T5). IXWebSocket connection
//   threads only feed the port's RX rings and the /uitoken slot table.
// - DEVICE GLOG LINES: Geiger's host platform layer (GEIGER_HOST_PLATFORM in
//   CMakeLists.txt) is drained on the hub thread only, into the sim's own
//   SessionLog, so a run reads as one stream.
// - The loop's 5 ms hub tick matches the P4's hub task; motion is evaluated
//   every pass (~1 ms), matching the P4's 1 kHz motion tick as closely as a
//   desktop scheduler allows.
// - PERSISTENCE IS THE BOARD'S, WITH FILES FOR NVS KEYS: PREFIX.cfg and
//   PREFIX.presets hold the exact blobs the P4 writes, on the same debounce,
//   written to a temp file and renamed over the old one so a kill mid-write
//   leaves the previous blob whole. PREFIX defaults to valencesim-state
//   beside the exe.
// See: sim/valencesim/README.md

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <thread>

#include <ixwebsocket/IXNetSystem.h>

#ifdef _WIN32
#include <windows.h>
#include <timeapi.h>
#endif

#include "SimMotion.h"
#include "SimPattern.h"
#include "SimUiToken.h"
#include "common/HostPlatform.h"
#include "common/SessionLog.h"
#include "geiger/geiger.h"
#include "hub/ValenceCatalog.h"
#include "hub/ValenceDevice.h"
#include "hub/valence_config.h"
#include "motion/ValenceMotion.h"
#include "net/WsServerPort.h"
#include "patterns/ValencePattern.h"

namespace {

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop = true; }

// The one clock: the hub's u32 wire time, deviceNowUs() and the engine all read
// it, which is what keeps stream anchors honest (ValenceDevice.h).
bench::HostClock g_clock;

constexpr const char* kHubName = "valencesim";

class GeigerToSessionLog final : public geiger::ISink {
public:
    explicit GeigerToSessionLog(bench::SessionLog& log) : _log(log) {}

    void write(const geiger::Record& r) override {
        const auto s = static_cast<unsigned long>(r.ms / 1000u);
        const auto ms = static_cast<unsigned long>(r.ms % 1000u);
        if (r.lost != 0) {
            _log.logf(geiger::levelChar(r.level), "%7lu.%03lu %-10s %s  (+%u lost)", s, ms, r.tag,
                      r.msg, unsigned(r.lost));
        } else {
            _log.logf(geiger::levelChar(r.level), "%7lu.%03lu %-10s %s", s, ms, r.tag, r.msg);
        }
    }

private:
    bench::SessionLog& _log;
};

struct Options {
    uint16_t wsPort = 82;
    uint16_t httpPort = 80;
    bool homed = false;
    int durationS = 0;
    bool pairingWindow = false;
    bool enforce = false;
    std::string statePrefix;   // empty = valencesim-state beside the exe
};

bool parseArgs(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const bool hasNext = i + 1 < argc;
        if (!std::strcmp(a, "machine")) continue;
        if (!std::strcmp(a, "--port") && hasNext) o.wsPort = uint16_t(std::atoi(argv[++i]));
        else if (!std::strcmp(a, "--http") && hasNext) o.httpPort = uint16_t(std::atoi(argv[++i]));
        else if (!std::strcmp(a, "--duration") && hasNext) o.durationS = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "--homed")) o.homed = true;
        else if (!std::strcmp(a, "--pairing-window")) o.pairingWindow = true;
        else if (!std::strcmp(a, "--enforce")) o.enforce = true;
        else if (!std::strcmp(a, "--state") && hasNext) o.statePrefix = argv[++i];
        // No TUI and no mDNS responder exist; both flags are accepted so the
        // command lines the Phosphor harness uses run unchanged.
        else if (!std::strcmp(a, "--headless") || !std::strcmp(a, "--no-mdns")) continue;
        else {
            std::fprintf(stderr, "valencesim: unknown flag '%s'\n", a);
            return false;
        }
    }
    return true;
}

std::filesystem::path exeDir(const char* argv0) {
#ifdef _WIN32
    std::array<wchar_t, 1024> buf{};
    const DWORD n = GetModuleFileNameW(nullptr, buf.data(), DWORD(buf.size()));
    if (n > 0 && n < buf.size()) return std::filesystem::path(buf.data()).parent_path();
#endif
    return std::filesystem::absolute(argv0).parent_path();
}

// The file's bytes in `scratch`, or an empty span for absent or larger than
// the scratch (the same answer the P4's loadBlob gives).
std::span<const std::byte> loadBlob(const std::filesystem::path& p, std::span<std::byte> scratch) {
    std::ifstream f(p, std::ios::binary);
    if (!f) return {};
    f.read(reinterpret_cast<char*>(scratch.data()), std::streamsize(scratch.size()));
    const size_t got = size_t(f.gcount());
    if (got == 0 || f.peek() != std::ifstream::traits_type::eof()) return {};
    return scratch.first(got);
}

bool saveBlob(const std::filesystem::path& p, std::span<const std::byte> blob) {
    if (blob.empty()) return false;
    std::filesystem::path tmp = p;
    tmp += ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) return false;
        f.write(reinterpret_cast<const char*>(blob.data()), std::streamsize(blob.size()));
        if (!f) return false;
    }
    std::error_code ec;
    std::filesystem::rename(tmp, p, ec);
    return !ec;
}

// Heap, not stack: the catalog pools and the hub's session table are tens of
// KB, and the Hub sits in an optional so the catalog is FILLED before the Hub
// constructor encodes it (the same order the P4's HubBox keeps).
struct SimBox {
    valence::Catalog32 catalog{};
    bench::HostRandom rng{};
    valence::ValenceDevice device{};
    std::optional<valence::Hub> hub{};
    bench::ValenceBenchWsPort port{};
    valence::SimUiToken minter{};
    std::array<std::byte, valence::PatternPresetStore::kBlobBytes> blobScratch{};
};

}  // namespace

namespace valence {
uint64_t deviceNowUs() { return g_clock.nowUs64(); }
uint32_t deviceFreeHeapBytes() { return 0; }
}  // namespace valence

namespace geiger {
uint32_t hostNowMs() { return g_clock.nowMs32(); }
}  // namespace geiger

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        std::fprintf(stderr,
                     "usage: valencesim [machine] [--port 82] [--http 80] [--homed] [--duration S]\n"
                     "                  [--pairing-window] [--enforce] [--state PREFIX]\n"
                     "                  [--headless] [--no-mdns]\n");
        return 2;
    }

    ix::initNetSystem();
    std::signal(SIGINT, onSignal);
#ifdef _WIN32
    // Without this a 1 ms sleep lands at ~15.6 ms and the motion tick with it.
    timeBeginPeriod(1);
#endif

    bench::SessionLog log;
    log.setEcho(true);
    // Declared before the first GLOG can fire and outlives every drain below.
    GeigerToSessionLog geigerSink(log);
    geiger::logger().addSink(&geigerSink);

    auto box = std::make_unique<SimBox>();
    valence::motionBegin();
    valence::patternBegin();

    if (!valence::buildValenceCatalog(box->catalog, valence::boardFeatures())) {
        std::fprintf(stderr, "valencesim: catalog build overflowed a Catalog32 pool\n");
        return 1;
    }
    box->device.bindTokenGate(&box->minter);
    // The P4 lands an unvouched HELLO at watch. The twin floats it at control
    // unless --enforce, because the Phosphor device tests were written against
    // that floor; --enforce is the device-exact posture.
    box->device.setUnvouchedRole(opt.enforce ? valence::AccessLevel::watch
                                             : valence::AccessLevel::control);
    // Stored state BEFORE the Hub, exactly as the P4's hubBegin orders it.
    const std::filesystem::path prefix =
        opt.statePrefix.empty() ? exeDir(argv[0]) / "valencesim-state" : std::filesystem::path(opt.statePrefix);
    std::filesystem::path cfgPath = prefix;
    cfgPath += ".cfg";
    std::filesystem::path presetsPath = prefix;
    presetsPath += ".presets";
    std::span<std::byte> scratch(box->blobScratch);
    uint16_t storedGen = 0;
    std::span<const std::byte> blob = loadBlob(cfgPath, scratch);
    const bool haveStored = !blob.empty() && box->device.adoptConfigBlob(blob, storedGen);
    if (!blob.empty() && !haveStored)
        log.logf('W', "valencesim: %s rejected -- factory values stand", cfgPath.string().c_str());
    blob = loadBlob(presetsPath, scratch);
    if (!blob.empty() && !box->device.adoptPresetsBlob(blob))
        log.logf('W', "valencesim: %s rejected -- preset store starts empty", presetsPath.string().c_str());
    box->device.pushConfigToMotion();

    box->hub.emplace(box->catalog, g_clock, box->rng, box->device);
    valence::Hub& hub = *box->hub;
    if (hub.catalogEncodedBytes() == 0) {
        std::fprintf(stderr, "valencesim: catalog encoded to ZERO bytes (scratch %u B)\n",
                     unsigned(valence::Hub::catalogScratchCapacity()));
        return 1;
    }
    if (haveStored) {
        while (hub.cfgGen() != storedGen) hub.bumpConfigGeneration();
    }
    log.logf('I', "valencesim: state %s.{cfg,presets}: config %s, cfg_gen %u",
             prefix.string().c_str(), haveStored ? "stored" : "factory", unsigned(hub.cfgGen()));
    hub.setIdentity(VALENCE_PRODUCT, FIRMWARE_VERSION, kHubName);
    hub.setEndpoint(opt.wsPort, 0x7F000001u);
    box->device.attach(hub);

    const auto etag = hub.catalogEtag();
    std::array<char, 2 * 32 + 1> etagHex{};
    for (size_t i = 0; i < etag.size() && i < 32; ++i)
        std::snprintf(etagHex.data() + 2 * i, 3, "%02x", unsigned(etag[i]));
    log.logf('I', "valencesim: %s %s, catalog %u entries, %u B, etag %s", VALENCE_PRODUCT,
             FIRMWARE_VERSION, unsigned(box->catalog.count), unsigned(hub.catalogEncodedBytes()),
             etagHex.data());
    log.logf('I', "valencesim: unvouched HELLO lands at %s",
             opt.enforce ? "watch (--enforce, device-exact)" : "control (sim floor)");

    if (opt.homed) {
        const float stroke = valence::motionForceHome(box->device.config().max_rail);
        log.logf('W', "valencesim: --homed: force_home asserted, stroke %.1f mm", double(stroke));
    }
    if (opt.pairingWindow) {
        hub.openPresenceWindow();
        log.logf('W', "valencesim: --pairing-window: presence window open (the gesture's twin)");
    }

    if (!box->port.begin(&hub, opt.wsPort, &log)) {
        std::fprintf(stderr, "valencesim: WS listen failed on :%u\n", unsigned(opt.wsPort));
        return 1;
    }
    std::string err;
    if (!box->minter.begin(opt.httpPort, err)) {
        // Non-fatal, as on the P4: watch-tier and paired clients still work.
        log.logf('W', "valencesim: /uitoken unavailable on :%u -- %s", unsigned(opt.httpPort),
                 err.c_str());
    } else {
        log.logf('I', "valencesim: GET /uitoken on 127.0.0.1:%u", unsigned(opt.httpPort));
    }

    const auto start = std::chrono::steady_clock::now();
    uint32_t lastHubMs = 0;
    while (!g_stop) {
        const uint64_t nowUs = g_clock.nowUs64();
        const uint32_t nowMs = uint32_t(nowUs / 1000);
        // Generator first, so a stroke it emits is planned on this same pass,
        // as the P4's motion task plans at arrival.
        valence::simPatternTick(nowUs);
        valence::simMotionTick(nowUs);
        if (uint32_t(nowMs - lastHubMs) >= 5u) {
            lastHubMs = nowMs;
            box->port.loop(nowMs);
            hub.update(g_clock.nowUs());
            const uint8_t due = box->device.tick(nowMs);
            geiger::drainToSinks();
            if (due & valence::kPersistConfig) {
                const size_t n = box->device.encodeConfigBlob(scratch, hub.cfgGen());
                if (!saveBlob(cfgPath, scratch.first(n)))
                    log.logf('W', "valencesim: persist %s failed", cfgPath.string().c_str());
            }
            if (due & valence::kPersistPresets) {
                const size_t n = box->device.encodePresetsBlob(scratch);
                if (!saveBlob(presetsPath, scratch.first(n)))
                    log.logf('W', "valencesim: persist %s failed", presetsPath.string().c_str());
            }
        }
        if (opt.durationS > 0 &&
            std::chrono::steady_clock::now() - start > std::chrono::seconds(opt.durationS)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    geiger::drainToSinks();
    box->minter.stop();
    box->port.stop();
#ifdef _WIN32
    timeEndPeriod(1);
#endif
    ix::uninitNetSystem();
    return 0;
}
