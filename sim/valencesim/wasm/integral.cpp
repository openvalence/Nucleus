// integral.cpp -- the in-process front of Integral, the Nucleus device twin:
// SimCore behind a C ABI a host (Phosphor's worker) drives. The host owns time
// and carries every byte; this file owns no clock, no socket and no thread.
// Constraints:
// - ONE instance per module: SimCore is a process singleton (SimCore.h).
//   integral_create succeeds once; a fresh machine is a fresh module.
// - One integral_send is one WebSocket message, which is one Valence frame
//   (the native port's rule); one integral_poll returns one. A pointer
//   integral_poll, integral_http or integral_state_get hands out stays valid
//   until the next call of that same function.
// - Time only moves in integral_tick, in 1 ms passes (the P4's motion tick).
// - Mirrors bench::ValenceBenchWsPort where a host can see it: the RX ring is
//   bounded and drops, a TX backlog over kMuteBytes mutes the client until
//   the host drains it, a close waits one hub tick for frames already received.
// See: sim/valencesim/README.md (the ABI and the host contract)

#include <emscripten/emscripten.h>

#include <cstdlib>
#include <cstring>
#include <deque>
#include <new>
#include <string>
#include <string_view>
#include <vector>

#include "SimCore.h"

namespace {

// The host's clock, set by integral_tick before every pass.
class HostDrivenClock final : public valence::IClock {
public:
    uint32_t nowUs() const override { return uint32_t(now); }
    uint64_t now = 0;
};
HostDrivenClock g_clock;

// The three NVS keys in memory. The host persists them as one blob:
// repeated [u8 SimBlob][u32 LE length][bytes].
class MemoryStore final : public valence::ISimStore {
public:
    std::span<const std::byte> load(valence::SimBlob b, std::span<std::byte> scratch) override {
        const auto& v = _blobs[idx(b)];
        if (v.empty() || v.size() > scratch.size()) return {};
        std::memcpy(scratch.data(), v.data(), v.size());
        return scratch.first(v.size());
    }
    bool save(valence::SimBlob b, std::span<const std::byte> blob) override {
        if (blob.empty()) return false;
        _blobs[idx(b)].assign(blob.begin(), blob.end());
        dirty = true;
        return true;
    }
    void adopt(const uint8_t* p, size_t n) {
        size_t i = 0;
        while (i + 5 <= n) {
            const uint8_t tag = p[i];
            const uint32_t len = uint32_t(p[i + 1]) | uint32_t(p[i + 2]) << 8 | uint32_t(p[i + 3]) << 16 |
                                 uint32_t(p[i + 4]) << 24;
            i += 5;
            if (len > n - i) return;   // truncated: keep what parsed whole
            if (tag >= 1 && tag <= 3) {
                const auto* b = reinterpret_cast<const std::byte*>(p + i);
                _blobs[tag - 1].assign(b, b + len);
            }
            i += len;
        }
    }
    void encode(std::vector<uint8_t>& out) const {
        out.clear();
        for (size_t k = 0; k < 3; ++k) {
            const auto& v = _blobs[k];
            if (v.empty()) continue;
            const uint32_t len = uint32_t(v.size());
            out.push_back(uint8_t(k + 1));
            for (int s = 0; s < 32; s += 8) out.push_back(uint8_t(len >> s));
            const auto* b = reinterpret_cast<const uint8_t*>(v.data());
            out.insert(out.end(), b, b + v.size());
        }
    }
    bool dirty = false;

private:
    static size_t idx(valence::SimBlob b) { return size_t(b) - 1; }
    std::vector<std::byte> _blobs[3];
};

class MemTransport final : public valence::ITransport {
public:
    static constexpr size_t kRxDepth = 32;
    static constexpr size_t kMuteBytes = 64 * 1024;

    bool open() override { return true; }
    void close() override { hubClosed = true; }
    bool write(std::span<const std::byte> frame) override {
        if (hubClosed || muted) return false;
        if (txBytes + frame.size() > kMuteBytes) {
            muted = true;
            return false;
        }
        tx.emplace_back(reinterpret_cast<const uint8_t*>(frame.data()),
                        reinterpret_cast<const uint8_t*>(frame.data()) + frame.size());
        txBytes += frame.size();
        return true;
    }
    std::optional<valence::FrameBuffer> read() override {
        if (rx.empty()) return std::nullopt;
        valence::FrameBuffer out = rx.front();
        rx.pop_front();
        return out;
    }
    valence::TransportProperties properties() const override {
        valence::TransportProperties p;
        p.mtu = uint16_t(valence::kFrameBufferCapacity);
        p.ordered = true;
        p.reliable = true;
        p.congestion = valence::CongestionSignal::QueueWatermark;
        return p;
    }

    int id = 0;
    bool used = false;
    bool attached = false;
    bool hubClosed = false;     // the hub ended it; the host sees -1 once TX drains
    bool clientClosed = false;  // integral_disconnect; detached at a hub tick
    bool closeHeld = false;
    bool muted = false;
    std::deque<valence::FrameBuffer> rx;
    std::deque<std::vector<uint8_t>> tx;
    size_t txBytes = 0;
    std::vector<uint8_t> polled;
};

valence::SimCore* g_core = nullptr;
MemoryStore g_store;
MemTransport g_slots[valence::kHubMaxSessions + 1];
uint64_t g_lastPassUs = 0;
bool g_firstTick = true;
std::string g_httpBody;
std::vector<uint8_t> g_stateOut;

MemTransport* slotOf(int id) {
    for (auto& s : g_slots)
        if (s.used && s.id == id) return &s;
    return nullptr;
}

void release(MemTransport& s) {
    if (s.attached) g_core->hub().detachTransport(s);
    s.~MemTransport();
    new (&s) MemTransport();
}

// The hub tick's pump: client closes detach here, after one hold for frames
// already received (a GOODBYE, typically), as the native port does.
void pump(uint32_t) {
    for (auto& s : g_slots) {
        if (!s.used || !s.clientClosed) continue;
        if (!s.rx.empty() && !s.closeHeld) {
            s.closeHeld = true;
            continue;
        }
        release(s);
    }
}

// The JSON options: flat keys, booleans and numbers only. An absent key keeps
// its default.
const char* findKey(const char* json, const char* key) {
    const std::string needle = std::string("\"") + key + "\"";
    const char* p = std::strstr(json, needle.c_str());
    if (!p) return nullptr;
    p += needle.size();
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == ':') ++p;
    return p;
}
bool optBool(const char* json, const char* key, bool def) {
    const char* p = findKey(json, key);
    if (!p) return def;
    return std::strncmp(p, "true", 4) == 0;
}
std::optional<double> optNum(const char* json, const char* key) {
    const char* p = findKey(json, key);
    if (!p) return std::nullopt;
    char* end = nullptr;
    const double v = std::strtod(p, &end);
    if (end == p) return std::nullopt;
    return v;
}

}  // namespace

namespace valence {
uint64_t deviceNowUs() { return g_clock.now; }
}  // namespace valence

namespace geiger {
uint32_t hostNowMs() { return uint32_t(g_clock.now / 1000); }
}  // namespace geiger

extern "C" {

// json_opts keys: homed, pairing_window, uncommissioned, motor_switch (bool);
// msw_fault_s, home_sense_at_mm, rail_end_at_mm (number). state: the blob
// integral_state_get last returned, or null. 1 booted, 0 refused.
EMSCRIPTEN_KEEPALIVE int integral_create(const char* json_opts, const uint8_t* state, size_t state_len) {
    if (g_core != nullptr) return 0;
    const char* j = json_opts ? json_opts : "";
    valence::SimConfig cfg;
    cfg.homed = optBool(j, "homed", false);
    cfg.pairingWindow = optBool(j, "pairing_window", false);
    cfg.uncommissioned = optBool(j, "uncommissioned", false);
    cfg.motorSwitch = optBool(j, "motor_switch", false);
    if (auto v = optNum(j, "msw_fault_s")) cfg.mswFaultS = int(*v);
    if (auto v = optNum(j, "home_sense_at_mm")) cfg.homeSenseAtMm = float(*v);
    if (auto v = optNum(j, "rail_end_at_mm")) cfg.railEndAtMm = float(*v);
    cfg.storeName = "host blob";
    if (state && state_len) g_store.adopt(state, state_len);
    g_core = new valence::SimCore();
    g_core->log.setEcho(true);
    if (!g_core->begin(cfg, g_clock, g_store)) return 0;
    return 1;
}

// Runs every 1 ms pass from the previous call up to now_us (the host's
// monotonic clock, any epoch). Past 250 passes behind, the clock jumps and the
// arbiter's stall cap holds, as on a stalled board. Returns bit0 when the
// state blob changed since the last tick: persist integral_state_get.
EMSCRIPTEN_KEEPALIVE int integral_tick(uint64_t now_us) {
    if (g_core == nullptr) return 0;
    if (g_firstTick) {
        g_firstTick = false;
        g_lastPassUs = now_us;
    }
    if (now_us > g_lastPassUs + 250000u) g_lastPassUs = now_us - 250000u;
    while (g_lastPassUs + 1000u <= now_us) {
        g_lastPassUs += 1000u;
        g_clock.now = g_lastPassUs;
        g_core->pass(g_lastPassUs, pump);
    }
    const bool dirty = g_store.dirty;
    g_store.dirty = false;
    return dirty ? 1 : 0;
}

// A WebSocket opened. 1 attached, 0 refused (every session slot busy, or the
// id is already open).
EMSCRIPTEN_KEEPALIVE int integral_connect(int client_id) {
    if (g_core == nullptr || slotOf(client_id)) return 0;
    for (auto& s : g_slots) {
        if (s.used) continue;
        s.used = true;
        s.id = client_id;
        if (!g_core->hub().attachTransport(s)) {
            release(s);
            return 0;
        }
        s.attached = true;
        return 1;
    }
    return 0;
}

// The client closed. Frames already sent are still read at the next hub tick.
EMSCRIPTEN_KEEPALIVE void integral_disconnect(int client_id) {
    if (MemTransport* s = slotOf(client_id)) s->clientClosed = true;
}

// One WebSocket message from the client. 1 queued, 0 dropped (unknown or
// closed client, oversize, or the RX ring full).
EMSCRIPTEN_KEEPALIVE int integral_send(int client_id, const uint8_t* data, size_t len) {
    MemTransport* s = slotOf(client_id);
    if (!s || s->clientClosed || s->hubClosed || len == 0 || len > valence::kFrameBufferCapacity) return 0;
    if (s->rx.size() >= MemTransport::kRxDepth) return 0;
    s->muted = false;
    valence::FrameBuffer fb;
    std::memcpy(fb.writable().data(), data, len);
    fb.setSize(len);
    s->rx.push_back(fb);
    return 1;
}

// The next message to the client: 1 with *out/*len set, 0 none pending, -1
// the hub closed this client (it is released; fire the socket's close).
EMSCRIPTEN_KEEPALIVE int integral_poll(int client_id, uint8_t** out, size_t* len) {
    MemTransport* s = slotOf(client_id);
    if (!s) return -1;
    if (s->tx.empty()) {
        s->muted = false;   // drained: the hub may write again
        if (s->hubClosed) {
            release(*s);
            return -1;
        }
        return 0;
    }
    s->polled = std::move(s->tx.front());
    s->tx.pop_front();
    s->txBytes -= s->polled.size();
    *out = s->polled.data();
    *len = s->polled.size();
    return 1;
}

// The hub's HTTP surface: GET /uitoken (RFC-029 section 4), else 404. Returns
// the status; *out/*out_len is the body.
EMSCRIPTEN_KEEPALIVE int integral_http(const char* method, const char* path, const uint8_t* body, size_t body_len,
                                       const uint8_t** out, size_t* out_len) {
    (void)body;
    (void)body_len;
    if (g_core == nullptr) return 503;
    const int code = g_core->minter().serve(method ? method : "", path ? path : "", g_httpBody);
    *out = reinterpret_cast<const uint8_t*>(g_httpBody.data());
    *out_len = g_httpBody.size();
    return code;
}

// The persisted state as one blob, for the host to keep and hand back to
// integral_create.
EMSCRIPTEN_KEEPALIVE void integral_state_get(const uint8_t** out, size_t* len) {
    g_store.encode(g_stateOut);
    *out = g_stateOut.data();
    *len = g_stateOut.size();
}

// Closes every client. The machine stays in memory (SimCore.h); drop the
// module to free it.
EMSCRIPTEN_KEEPALIVE void integral_destroy() {
    if (g_core == nullptr) return;
    for (auto& s : g_slots)
        if (s.used) release(s);
}

}  // extern "C"
