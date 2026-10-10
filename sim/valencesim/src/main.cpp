// valencesim -- the native front of Integral, the Nucleus device twin: the
// machine (src/SimCore.h) behind a WebSocket speaking valence.v1, /uitoken on
// HTTP, UDP discovery and a wall clock
//
//   valencesim [machine] [--port 82] [--bind 0.0.0.0] [--http 80] [--homed] [--duration S]
//              [--pairing-window] [--motor-switch [--msw-fault S]] [--state PREFIX]
//              [--no-estop-udp] [--home-sense-at MM [--rail-end-at MM]] [--plan-delay-ms N]
//              [--uncommissioned] [--no-discovery] [--discovery-port N]
//              [--headless] [--no-mdns] [--enforce]
//
// Constraints:
// - HOST-ONLY: never touches a device, never deploys, no pio.
// - ONE hub thread. SimCore is called from the loop below and nowhere else
//   (T5). IXWebSocket connection threads only feed the port's RX rings and
//   the /uitoken slot table.
// - DEVICE GLOG LINES: Geiger's host platform layer (GEIGER_HOST_PLATFORM in
//   CMakeLists.txt) is drained on the hub thread only, into the sim's own
//   SessionLog, so a run reads as one stream, and Warn and above onto the log
//   channel 0x0008 as the board's ValenceLogBridge does.
// - UDP DISCOVERY IS THE BOARD'S: flagship_p4/src/hub/ValenceDiscovery.cpp,
//   polled on the hub thread like the board's hub task. --no-discovery keeps a
//   test run off UDP; --discovery-port moves it off the registry port.
// - The loop's 5 ms hub tick matches the P4's hub task; motion is evaluated
//   every pass, paced to kMotionTickUs against a deadline (TickPacer). On
//   Windows the wait is a high-resolution waitable timer: a headless
//   process's timeBeginPeriod(1) is not honored, and a 1 ms sleep there
//   lands at ~15.6 ms.
// - PERSISTENCE IS THE BOARD'S, WITH FILES FOR NVS KEYS: PREFIX.cfg and
//   PREFIX.presets hold the exact blobs the P4 writes, on the same debounce,
//   and PREFIX.iid holds the P4's hub_iid (WELCOME identity key 5, SPEC §6.3)
//   as 8 bytes little-endian. Each is written to a temp file and renamed over
//   the old one so a kill mid-write leaves the previous blob whole. PREFIX
//   defaults to valencesim-state beside the exe.
// See: sim/valencesim/README.md

#include <algorithm>
#include <array>
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

#include <ixwebsocket/IXHttpServer.h>
#include <ixwebsocket/IXNetSystem.h>

#ifdef _WIN32
#include <windows.h>
#include <timeapi.h>
#endif

#include "SimCore.h"
#include "hub/ValenceDiscovery.h"
#include "hub/ValenceEstopDatagram.h"
#include "motion/ValenceMotion.h"
#include "net/WsServerPort.h"

namespace {

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop = true; }

// One pass per period from a running deadline. A pass that overran restarts
// the cadence from now: a late tick is a stall, never a burst of catch-up.
class TickPacer {
public:
    explicit TickPacer(std::chrono::microseconds period) : _period(period) {
#ifdef _WIN32
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
        _timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
#endif
    }
    ~TickPacer() {
#ifdef _WIN32
        if (_timer != nullptr) CloseHandle(_timer);
#endif
    }
    TickPacer(const TickPacer&) = delete;
    TickPacer& operator=(const TickPacer&) = delete;

    // false: the high-resolution timer is unavailable (before Windows 10
    // 1803) and the wait falls back to sleep_until at the default resolution.
    bool precise() const {
#ifdef _WIN32
        return _timer != nullptr;
#else
        return true;
#endif
    }

    void wait() {
        const auto now = std::chrono::steady_clock::now();
        _next = _next + _period < now ? now + _period : _next + _period;
#ifdef _WIN32
        if (_timer != nullptr) {
            const auto left = std::chrono::duration_cast<std::chrono::nanoseconds>(_next - now).count();
            LARGE_INTEGER due;
            due.QuadPart = -std::max<long long>(left / 100, 1);   // relative, 100 ns units
            if (SetWaitableTimer(_timer, &due, 0, nullptr, nullptr, FALSE)) {
                WaitForSingleObject(_timer, INFINITE);
                return;
            }
        }
#endif
        std::this_thread::sleep_until(_next);
    }

private:
    std::chrono::microseconds _period;
    std::chrono::steady_clock::time_point _next = std::chrono::steady_clock::now();
#ifdef _WIN32
    HANDLE _timer = nullptr;
#endif
};

// The one clock: the hub's u32 wire time, deviceNowUs() and the engine all read
// it, which is what keeps stream anchors honest (ValenceDevice.h).
bench::HostClock g_clock;

struct Options {
    uint16_t wsPort = 82;
    std::string bindHost = "0.0.0.0";   // --bind: the WS listen address
    uint16_t httpPort = 80;
    int durationS = 0;
    bool noEstopUdp = false;   // --no-estop-udp: RFC-053 datagrams never latch
    valence::SimConfig sim;    // the machine's own options
    std::string statePrefix;   // empty = valencesim-state beside the exe
    bool discovery = true;
    uint16_t discoveryPort = uint16_t(valence::udp_discovery::port);
};

bool parseArgs(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const bool hasNext = i + 1 < argc;
        if (!std::strcmp(a, "machine")) continue;
        if (!std::strcmp(a, "--port") && hasNext) o.wsPort = uint16_t(std::atoi(argv[++i]));
        else if (!std::strcmp(a, "--bind") && hasNext) o.bindHost = argv[++i];
        else if (!std::strcmp(a, "--http") && hasNext) o.httpPort = uint16_t(std::atoi(argv[++i]));
        else if (!std::strcmp(a, "--duration") && hasNext) o.durationS = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "--homed")) o.sim.homed = true;
        else if (!std::strcmp(a, "--uncommissioned")) o.sim.uncommissioned = true;
        else if (!std::strcmp(a, "--pairing-window")) o.sim.pairingWindow = true;
        else if (!std::strcmp(a, "--motor-switch")) o.sim.motorSwitch = true;
        else if (!std::strcmp(a, "--msw-fault") && hasNext) o.sim.mswFaultS = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "--no-estop-udp")) o.noEstopUdp = true;
        else if (!std::strcmp(a, "--home-sense-at") && hasNext) o.sim.homeSenseAtMm = float(std::atof(argv[++i]));
        else if (!std::strcmp(a, "--rail-end-at") && hasNext) o.sim.railEndAtMm = float(std::atof(argv[++i]));
        else if (!std::strcmp(a, "--plan-delay-ms") && hasNext) o.sim.planDelayMs = uint32_t(std::atoi(argv[++i]));
        else if (!std::strcmp(a, "--state") && hasNext) o.statePrefix = argv[++i];
        else if (!std::strcmp(a, "--no-discovery")) o.discovery = false;
        else if (!std::strcmp(a, "--discovery-port") && hasNext) o.discoveryPort = uint16_t(std::atoi(argv[++i]));
        // No TUI and no mDNS responder exist, and --enforce names what is now
        // the only posture; all three are accepted so older command lines run
        // unchanged.
        else if (!std::strcmp(a, "--headless") || !std::strcmp(a, "--no-mdns") ||
                 !std::strcmp(a, "--enforce"))
            continue;
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

// A file stands in for each NVS key: PREFIX.cfg, PREFIX.presets, PREFIX.iid.
class FileStore final : public valence::ISimStore {
public:
    explicit FileStore(std::filesystem::path prefix) : _prefix(std::move(prefix)) {}
    std::span<const std::byte> load(valence::SimBlob b, std::span<std::byte> scratch) override {
        return loadBlob(path(b), scratch);
    }
    bool save(valence::SimBlob b, std::span<const std::byte> blob) override { return saveBlob(path(b), blob); }
    std::string name() const { return _prefix.string() + ".{cfg,presets,iid}"; }

private:
    std::filesystem::path path(valence::SimBlob b) const {
        std::filesystem::path p = _prefix;
        p += b == valence::SimBlob::cfg ? ".cfg" : b == valence::SimBlob::presets ? ".presets" : ".iid";
        return p;
    }
    std::filesystem::path _prefix;
};

// GET /uitoken on 127.0.0.1, routed by SimUiToken::serve. Null if the port
// cannot bind (`err` says why).
std::unique_ptr<ix::HttpServer> startUiTokenHttp(valence::SimUiToken& minter, uint16_t port, std::string& err) {
    auto server = std::make_unique<ix::HttpServer>(port, "127.0.0.1");
    server->setOnConnectionCallback(
        [&minter](ix::HttpRequestPtr req, std::shared_ptr<ix::ConnectionState>) -> ix::HttpResponsePtr {
            // ---- DO NOT ADD CORS HEADERS: their ABSENCE is the mechanism ----
            ix::WebSocketHttpHeaders h;
            h["Connection"] = "close";
            std::string body;
            const int code = minter.serve(req->method, req->uri, body);
            if (code != 404) h["Content-Type"] = "application/json";
            if (code == 200) h["Cache-Control"] = "no-store";
            const char* reason = code == 200 ? "OK" : code == 429 ? "Too Many Requests" : "Not Found";
            return std::make_shared<ix::HttpResponse>(code, reason, ix::HttpErrorCode::Ok, h, body);
        });
    const auto res = server->listen();
    if (!res.first) {
        err = res.second;
        return nullptr;
    }
    server->start();
    return server;
}

}  // namespace

namespace valence {
uint64_t deviceNowUs() { return g_clock.nowUs64(); }
}  // namespace valence

namespace geiger {
uint32_t hostNowMs() { return g_clock.nowMs32(); }
}  // namespace geiger

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        std::fprintf(stderr,
                     "usage: valencesim [machine] [--port 82] [--bind 0.0.0.0] [--http 80] [--homed] [--duration S]\n"
                     "                  [--pairing-window] [--motor-switch [--msw-fault S]] [--state PREFIX]\n"
                     "                  [--no-estop-udp] [--home-sense-at MM [--rail-end-at MM]] [--plan-delay-ms N]\n"
                     "                  [--uncommissioned] [--no-discovery] [--discovery-port N]\n"
                     "                  [--headless] [--no-mdns] [--enforce]\n");
        return 2;
    }

    ix::initNetSystem();
    std::signal(SIGINT, onSignal);
#ifndef _WIN32
    // macOS has no MSG_NOSIGNAL and IXWebSocket sets SO_NOSIGPIPE only on
    // sockets it connects, never on accepted ones: a client that drops
    // mid-send would kill the twin. The failed send returns EPIPE instead.
    std::signal(SIGPIPE, SIG_IGN);
#endif
#ifdef _WIN32
    // The waits the motion tick does not own (IXWebSocket's polls), at 1 ms
    // where Windows honors it; TickPacer paces the tick itself.
    timeBeginPeriod(1);
#endif

    auto core = std::make_unique<valence::SimCore>();
    bench::SessionLog& log = core->log;
    log.setEcho(true);
    FileStore store(opt.statePrefix.empty() ? exeDir(argv[0]) / "valencesim-state"
                                            : std::filesystem::path(opt.statePrefix));
    const std::string storeName = store.name();
    opt.sim.storeName = storeName.c_str();
    if (!core->begin(opt.sim, g_clock, store)) return 1;
    valence::Hub& hub = core->hub();
    hub.setEndpoint(opt.wsPort, 0x7F000001u);
    // RFC-053: ESTOP datagrams share the §13.8 port, wired as the board's
    // hubBegin wires them: the hub they latch, and the port's hook.
    valence::ValenceDiscoveryPort discovery;
    valence::estopDatagramBind(&hub);
    discovery.setDatagramHook(&valence::estopDatagramHook);
    discovery.setReplyFlagsHook(&valence::estopDatagramReplyFlags);
    if (opt.noEstopUdp) {
        valence::estopDatagramSetEnabled(false);
        log.logf('W', "valencesim: --no-estop-udp: ESTOP datagrams are dropped, never latched");
    }

    bench::ValenceBenchWsPort port;
    if (!port.begin(&hub, opt.wsPort, &log, opt.bindHost)) {
        std::fprintf(stderr, "valencesim: WS listen failed on %s:%u\n", opt.bindHost.c_str(), unsigned(opt.wsPort));
        return 1;
    }
    // Non-fatal, as on the P4: a port another twin holds only costs discovery.
    if (opt.discovery) discovery.begin(opt.discoveryPort, valence::SimCore::kHubName, FIRMWARE_MAJOR_MINOR, opt.wsPort);
    std::string err;
    std::unique_ptr<ix::HttpServer> http = startUiTokenHttp(core->minter(), opt.httpPort, err);
    if (!http) {
        // Non-fatal, as on the P4: watch-tier and paired clients still work.
        log.logf('W', "valencesim: /uitoken unavailable on :%u -- %s", unsigned(opt.httpPort), err.c_str());
    } else {
        log.logf('I', "valencesim: GET /uitoken on 127.0.0.1:%u", unsigned(opt.httpPort));
    }

    const auto pump = [&](uint32_t nowMs) {
        port.loop(nowMs);
        discovery.poll(hub, nowMs);
    };
    TickPacer pacer(std::chrono::microseconds(valence::kMotionTickUs));
    if (!pacer.precise()) log.logf('W', "valencesim: no high-resolution timer, the motion tick runs coarse");
    const auto start = std::chrono::steady_clock::now();
    while (!g_stop) {
        core->pass(g_clock.nowUs64(), pump);
        if (opt.durationS > 0 &&
            std::chrono::steady_clock::now() - start > std::chrono::seconds(opt.durationS)) {
            break;
        }
        pacer.wait();
    }

    geiger::drainToSinks();
    if (http) http->stop();
    discovery.end();
    port.stop();
#ifdef _WIN32
    timeEndPeriod(1);
#endif
    ix::uninitNetSystem();
    return 0;
}
