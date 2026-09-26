// valencesim -- the Nucleus device twin on a desktop: the REAL valence::Hub,
// the REAL device catalog and delegate (flagship_p4/src/hub/ValenceDevice),
// and the REAL kinetic::Engine behind a WebSocket speaking valence.v1
//
//   valencesim [machine] [--port 82] [--http 80] [--homed] [--duration S]
//              [--pairing-window] [--enforce] [--headless] [--no-mdns]
//
// Constraints:
// - HOST-ONLY: never touches a device, never deploys, no pio.
// - ONE hub thread. The Hub, ValenceDevice and SimMotion are called from the
//   loop below and nowhere else (T5). IXWebSocket connection threads only feed
//   the port's RX rings and the /uitoken slot table.
// - The loop's 5 ms hub tick matches the P4's hub task; motion is evaluated
//   every pass (~1 ms), matching the P4's 1 kHz motion tick as closely as a
//   desktop scheduler allows.
// See: sim/valencesim/README.md

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <ixwebsocket/IXNetSystem.h>

#ifdef _WIN32
#include <windows.h>
#include <timeapi.h>
#endif

#include "SimMotion.h"
#include "SimUiToken.h"
#include "common/HostPlatform.h"
#include "common/SessionLog.h"
#include "hub/ValenceCatalog.h"
#include "hub/ValenceDevice.h"
#include "hub/valence_config.h"
#include "motion/ValenceMotion.h"
#include "net/WsServerPort.h"

namespace {

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop = true; }

// The one clock: the hub's u32 wire time, deviceNowUs() and the engine all read
// it, which is what keeps stream anchors honest (ValenceDevice.h).
bench::HostClock g_clock;

constexpr const char* kHubName = "valencesim";

struct Options {
    uint16_t wsPort = 82;
    uint16_t httpPort = 80;
    bool homed = false;
    int durationS = 0;
    bool pairingWindow = false;
    bool enforce = false;
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
};

}  // namespace

namespace valence {
uint64_t deviceNowUs() { return g_clock.nowUs64(); }
uint32_t deviceFreeHeapBytes() { return 0; }
}  // namespace valence

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        std::fprintf(stderr,
                     "usage: valencesim [machine] [--port 82] [--http 80] [--homed] [--duration S]\n"
                     "                  [--pairing-window] [--enforce] [--headless] [--no-mdns]\n");
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

    auto box = std::make_unique<SimBox>();
    valence::motionBegin();

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
    box->device.pushConfigToMotion();

    box->hub.emplace(box->catalog, g_clock, box->rng, box->device);
    valence::Hub& hub = *box->hub;
    if (hub.catalogEncodedBytes() == 0) {
        std::fprintf(stderr, "valencesim: catalog encoded to ZERO bytes (scratch %u B)\n",
                     unsigned(valence::Hub::catalogScratchCapacity()));
        return 1;
    }
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
        valence::simMotionTick(nowUs);
        if (uint32_t(nowMs - lastHubMs) >= 5u) {
            lastHubMs = nowMs;
            box->port.loop(nowMs);
            hub.update(g_clock.nowUs());
            // The persist-due answer is dropped: the twin keeps config in
            // memory for the life of the process, like a board with no NVS.
            (void)box->device.tick(nowMs);
        }
        if (opt.durationS > 0 &&
            std::chrono::steady_clock::now() - start > std::chrono::seconds(opt.durationS)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    box->minter.stop();
    box->port.stop();
#ifdef _WIN32
    timeEndPeriod(1);
#endif
    ix::uninitNetSystem();
    return 0;
}
