// NeutrinoObserver -- implementation. See NeutrinoObserver.h; nothing is restated here.

#include "NeutrinoObserver.h"

#include <chrono>

#include "SimMotion.h"
#include "SimMotorSwitch.h"
#include "hub/valence_config.h"
#include "motion/MotionArbiter.h"
#include "motion/ValenceMotion.h"
#include "patterns/ValencePattern.h"

namespace valence {

static_assert(devtool::kStripLen == kStripLen, "the dev tool's strip is the arbiter's");

namespace {

constexpr size_t kSendBacklogBytes = 1u << 20;   // a dev tool this far behind skips frames

const char* sourceOf(const MotionCensus& c) {
    if (c.stream) return "stream";
    if (c.homing) return "home";
    if (c.returning) return "return";
    if (patternActive()) return "pattern";
    if (c.busy) return "move";
    return "idle";
}

const char* className(ChannelClass c) {
    switch (c) {
        case ChannelClass::STATE: return "state";
        case ChannelClass::STREAM: return "stream";
        case ChannelClass::INTENT: return "intent";
        case ChannelClass::EVENT: return "event";
        case ChannelClass::STORE: return "store";
    }
    return "?";
}

}  // namespace

bool NeutrinoObserver::begin(SimCore& core, NeutrinoWsPort& port, uint16_t devtoolPort, uint16_t wsPort,
                             uint16_t httpPort, std::string& err) {
    _core = &core;
    _port = &port;
    _log = &core.log;

    // The hello: fixed per boot, so built here, before any thread can read it.
    using devtool::appendf;
    using devtool::appendStr;
    const Hub& hub = core.hub();
    _hello = "{\"t\":\"hello\",\"name\":";
    appendStr(_hello, SimCore::kHubName);
    _hello += ",\"product\":", appendStr(_hello, VALENCE_PRODUCT);
    _hello += ",\"version\":", appendStr(_hello, FIRMWARE_VERSION);
    appendf(_hello, ",\"ws\":%u,\"http\":%u,\"devtool\":%u,\"slots\":%u,\"iid\":\"%016llx\",\"etag\":\"",
            unsigned(wsPort), unsigned(httpPort), unsigned(devtoolPort), unsigned(NeutrinoWsPort::kSlots),
            static_cast<unsigned long long>(hub.hubInstanceId()));
    for (const std::byte b : hub.catalogEtag()) appendf(_hello, "%02x", unsigned(b));
    appendf(_hello, "\",\"bytes\":%u,\"catalog\":[", unsigned(hub.catalogEncodedBytes()));
    const Catalog32& cat = core.catalog();
    for (size_t i = 0; i < cat.count; ++i) {
        const CatalogEntry& e = cat.entries[i];
        if (i) _hello += ',';
        appendf(_hello, "[%u,", unsigned(e.id));
        appendStr(_hello, e.name);
        appendf(_hello, ",\"%s\"]", className(e.cls));
    }
    _hello += "]}";

    _server = std::make_unique<ix::WebSocketServer>(devtoolPort, "127.0.0.1");
    // Connection threads: parse, queue, greet. Nothing else.
    _server->setOnClientMessageCallback(
        [this](std::shared_ptr<ix::ConnectionState>, ix::WebSocket& ws, const ix::WebSocketMessagePtr& msg) {
            if (msg->type == ix::WebSocketMessageType::Open) {
                ws.sendText(_hello);
                return;
            }
            if (msg->type != ix::WebSocketMessageType::Message || msg->binary) return;
            devtool::Input in;
            if (!devtool::parseInput(msg->str, in)) {
                _log->logf('W', "devtool: unknown input '%.60s'", msg->str.c_str());
                return;
            }
            if (!_inputs.push(in)) _log->logf('W', "devtool: input queue full, input dropped");
        });
    const auto res = _server->listen();
    if (!res.first) {
        err = res.second;
        _server.reset();
        return false;
    }
    _server->start();
    _broadcaster = std::thread([this] { broadcast(); });
    _log->logf('I', "devtool: observer channel on 127.0.0.1:%u", unsigned(devtoolPort));
    return true;
}

void NeutrinoObserver::stop() {
    _stop = true;
    if (_broadcaster.joinable()) _broadcaster.join();
    if (_server) _server->stop();
    _server.reset();
}

// ---- broadcaster thread ---------------------------------------------------------

void NeutrinoObserver::broadcast() {
    devtool::Snapshot snap;
    std::string json;
    while (!_stop) {
        std::this_thread::sleep_for(std::chrono::microseconds(kSnapshotPeriodUs));
        if (!_mailbox.take(snap)) continue;
        devtool::encodeSnapshot(snap, json);
        for (const auto& ws : _server->getClients()) {
            if (ws->bufferedAmount() < kSendBacklogBytes) ws->sendText(json);
        }
    }
}

// ---- hub thread -------------------------------------------------------------------

void NeutrinoObserver::apply(const devtool::Input& in, uint64_t nowUs) {
    using K = devtool::Input::Kind;
    switch (in.kind) {
        case K::pair: _pairDown = in.down; break;
        case K::home: _homeDown = in.down; break;
        case K::estop:
            if (in.down == _estopDown) break;
            _estopDown = in.down;
            simEstopPressed(in.down);
            _log->logf('W', "devtool: e-stop %s", in.down ? "pressed" : "released");
            break;
        case K::drive_alarm:
            simDriveAlarm();
            _log->logf('W', "devtool: DRV_ALM asserted");
            break;
        case K::switch_fault:
            simMotorSwitchInjectFault(nowUs, nowUs + 2000000u);
            _log->logf('W', "devtool: MSW_FLT_N low for 2 s (needs --motor-switch)");
            break;
        case K::faults:
            if (in.slot < 0) _port->setGlobalFaults(in.faults);
            else _port->setClientFaults(uint8_t(in.slot), in.faults);
            if (in.slot < 0)
                _log->logf('W', "devtool: faults, every client: %u ms, +%u ms jitter, %.1f%% loss",
                           unsigned(in.faults.latency_ms), unsigned(in.faults.jitter_ms), double(in.faults.loss_pct));
            else
                _log->logf('W', "devtool: faults, slot %d: %u ms, +%u ms jitter, %.1f%% loss", int(in.slot),
                           unsigned(in.faults.latency_ms), unsigned(in.faults.jitter_ms), double(in.faults.loss_pct));
            break;
        case K::wifi_drop:
            _wifiHeld = in.ms != 0;
            _wifiUntilUs = nowUs + uint64_t(in.ms) * 1000u;
            _port->setHeld(_wifiHeld);
            if (_wifiHeld) _log->logf('W', "devtool: Wi-Fi down for %.1f s", double(in.ms) / 1000.0);
            else _log->logf('W', "devtool: Wi-Fi back");
            break;
        case K::kick:
            _port->kick(uint8_t(in.slot));
            _log->logf('W', "devtool: slot %d disconnected", int(in.slot));
            break;
    }
}

void NeutrinoObserver::pass(uint64_t nowUs) {
    const uint32_t nowMs = uint32_t(nowUs / 1000u);
    const size_t n = _inputs.drain(_drained);
    for (size_t i = 0; i < n; ++i) apply(_drained[i], nowUs);

    // The board's BoardIo task samples both pads at least every 10 ms; every
    // pass is finer and changes no gesture's timing.
    if (const button::PairGesture g = _pairCore.sample(_pairDown, nowMs); g != button::PairGesture::none) {
        _log->logf('W', "button: PAIR %s", button::pairGestureName(g));
        simPairGesture(g);
    }
    if (const button::Gesture g = _homeCore.sample(_homeDown, nowMs); g != button::Gesture::none) {
        _log->logf('W', "button: HOME %s", button::gestureName(g));
        simHomeGesture(g);
    }
    if (_wifiHeld && nowUs >= _wifiUntilUs) {
        _wifiHeld = false;
        _port->setHeld(false);
        _log->logf('W', "devtool: Wi-Fi back");
    }

    const MotionCensus c = motionCensus();
    if (_build.samples.size() >= devtool::kMaxSamples) _build.samples.erase(_build.samples.begin());
    _build.samples.push_back({c.sample_us, c.position_mm, c.velocity_mm_s});

    if (nowUs < _nextSnapUs) return;
    _nextSnapUs = nowUs + kSnapshotPeriodUs;
    fillSnapshot(nowUs);
    if (_mailbox.publish(_build)) {
        _build.samples.clear();
        _build.log.clear();
    }
}

void NeutrinoObserver::fillSnapshot(uint64_t nowUs) {
    devtool::Snapshot& s = _build;
    Hub& hub = _core->hub();
    s.seq = ++_seq;
    s.nowUs = nowUs;
    s.m = motionCensus();
    s.source = sourceOf(s.m);
    s.intentSession = hub.intentSession();
    s.stripN = simMotionStrip(s.stripT0Us, s.strip);
    s.estopLatched = hub.estopLatched();
    s.pauseLatched = hub.pauseLatched();
    s.safetyWord = hub.safetyWord();
    s.presenceOpen = hub.presenceWindowOpen();
    s.pairingOpen = hub.pairingWindowOpen();
    s.configureAsks = uint32_t(hub.configureRequestsPending());
    s.pairLook = uint8_t(_core->device().pairLook());
    s.pairDown = _pairDown;
    s.homeDown = _homeDown;
    s.estopDown = _estopDown;
    s.wifiDropLeftMs = _wifiHeld ? uint32_t((_wifiUntilUs - nowUs) / 1000u) : 0;
    s.net = _port->globalFaults();
    s.inputsDropped = _inputs.dropped();

    // Every port slot with a connection, matched to its hub session by the
    // session id its link saw in WELCOME; then the parked sessions no
    // connection carries (slot 255).
    s.clients.clear();
    std::array<bool, 16> matched{};
    for (uint8_t i = 0; i < NeutrinoWsPort::kSlots; ++i) {
        const NeutrinoWsPort::SlotInfo info = _port->info(i);
        if (!info.inUse) continue;
        devtool::Client cl;
        cl.slot = i;
        cl.peer = info.peer;
        cl.muted = info.muted;
        cl.rxDrops = info.rxDrops;
        cl.link = info.link;
        cl.faults = info.faults;
        for (size_t j = 0; const HubSession* hs = hub.sessionBySlot(j); ++j) {
            if (j < matched.size() && !matched[j] && hs->occupied() && info.link.session_id != 0 &&
                hs->session_id == info.link.session_id) {
                matched[j] = true;
                cl.hasSession = true;
                cl.name.assign(hs->clientName.view());
                cl.kind.assign(hs->clientKind.view());
                cl.tier = uint8_t(hs->role);
                cl.state = uint8_t(hs->state);
                cl.ready = hs->ready;
                const auto ing = hub.streamIngressCounters(j);
                cl.streamAccepted = ing.accepted;
                cl.streamDropped = ing.dropped;
                break;
            }
        }
        s.clients.push_back(std::move(cl));
    }
    for (size_t j = 0; const HubSession* hs = hub.sessionBySlot(j); ++j) {
        if (j >= matched.size() || matched[j] || !hs->occupied()) continue;
        devtool::Client cl;
        cl.slot = 255;
        cl.hasSession = true;
        cl.name.assign(hs->clientName.view());
        cl.kind.assign(hs->clientKind.view());
        cl.tier = uint8_t(hs->role);
        cl.state = uint8_t(hs->state);
        cl.ready = hs->ready;
        cl.link.session_id = hs->session_id;
        s.clients.push_back(std::move(cl));
    }

    const size_t rev = _core->log.revision();
    if (rev != _logRev) {
        const size_t fresh = std::min<size_t>(rev - _logRev, devtool::kMaxLogLines);
        _logRev = rev;
        for (auto& line : _core->log.tail(fresh)) s.log.push_back({line.level, std::move(line.text)});
        if (s.log.size() > devtool::kMaxLogLines) s.log.erase(s.log.begin(), s.log.end() - devtool::kMaxLogLines);
    }
}

}  // namespace valence
