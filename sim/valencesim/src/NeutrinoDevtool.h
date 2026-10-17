#pragma once

// NeutrinoDevtool -- the dev tool channel's crossings and wire: the input
// queue (dev tool threads to the hub thread), the snapshot mailbox (hub
// thread to the broadcaster), the snapshot, and its text-in / JSON-out codec
// Constraints:
// - TWO CROSSINGS AND NOTHING ELSE SHARED. DevInputQueue and SnapshotMailbox
//   are the only objects two threads touch; each holds its lock for a copy or
//   a swap, never while hub logic or socket I/O runs.
// - THE HUB THREAD NEVER WAITS. Its side of both crossings is a try-lock: a
//   lock the other side holds right now means "next pass", never a block.
//   Its wake of the reader is a notify after the unlock, which waits on no one.
// - Lossless at the mailbox: a snapshot the reader has not taken yet hands
//   its samples and log lines to the next one (absorb), capped at
//   kMaxSamples / kMaxLogLines.
// - No IXWebSocket, no SimCore: native suite test_neutrino_devtool compiles
//   this header alone.
// See: sim/valencesim/README.md (the dev tool), NeutrinoObserver.h

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "NeutrinoLink.h"
#include "hub/ValenceDiscovery.h"
#include "motion/ValenceMotion.h"

namespace valence::devtool {

// ---- the Wi-Fi drop on the discovery port ----------------------------------------

// A board off the air never receives what is sent during the outage; the host's
// socket queues it. While a drop runs, the hub thread keeps polling the port and
// this hook consumes every datagram unanswered, so no probe is answered and no
// RFC-053 e-stop latches after the restore (bd val-6sa).
inline bool discardDatagram(std::span<const std::byte>, uint32_t, uint32_t) { return true; }

// The hook the discovery port runs this pass: `live` (the one the composition
// root wired) unless the Wi-Fi drop is running.
inline DiscoveryResponder::DatagramHook discoveryHook(bool wifiDown, DiscoveryResponder::DatagramHook live) {
    return wifiDown ? &discardDatagram : live;
}

// ---- inputs ------------------------------------------------------------------

// One physical act or network setting from the dev tool.
struct Input {
    enum class Kind : uint8_t { pair, home, estop, drive_alarm, switch_fault, faults, wifi_drop, kick };
    Kind kind = Kind::pair;
    bool down = false;          // pair, home, estop: the contact's level
    int16_t slot = -1;          // faults, kick: the port slot; -1 is every client (faults only)
    LinkFaults faults{};
    uint32_t ms = 0;            // wifi_drop: how long; 0 ends one
};

// One command line, as the dev tool sends it:
//   pair down|up    home down|up    estop down|up    drive-alarm    switch-fault
//   faults all|SLOT LATENCY_MS JITTER_MS LOSS_PCT    wifi-drop SECONDS    kick SLOT
inline bool parseInput(std::string_view line, Input& out) {
    std::array<char, 128> buf{};
    if (line.size() >= buf.size()) return false;
    std::memcpy(buf.data(), line.data(), line.size());
    std::array<char, 24> verb{}, arg{};
    const int n = std::sscanf(buf.data(), "%23s %23s", verb.data(), arg.data());
    if (n < 1) return false;
    const std::string_view v(verb.data()), a(n > 1 ? arg.data() : "");
    out = Input{};
    if (v == "pair" || v == "home" || v == "estop") {
        if (a != "down" && a != "up") return false;
        out.kind = v == "pair" ? Input::Kind::pair : v == "home" ? Input::Kind::home : Input::Kind::estop;
        out.down = a == "down";
        return true;
    }
    if (v == "drive-alarm") { out.kind = Input::Kind::drive_alarm; return n == 1; }
    if (v == "switch-fault") { out.kind = Input::Kind::switch_fault; return n == 1; }
    if (v == "faults") {
        unsigned lat = 0, jit = 0;
        float loss = 0.0f;
        if (std::sscanf(buf.data(), "%*s %*s %u %u %f", &lat, &jit, &loss) != 3) return false;
        if (!std::isfinite(loss) || loss < 0.0f || loss > 100.0f || lat > 60000 || jit > 60000) return false;
        out.kind = Input::Kind::faults;
        out.faults = {lat, jit, loss};
        if (a == "all") return true;
        char* end = nullptr;
        const long s = std::strtol(arg.data(), &end, 10);
        if (end == arg.data() || *end != '\0' || s < 0 || s > 64) return false;
        out.slot = int16_t(s);
        return true;
    }
    if (v == "wifi-drop") {
        float s = 0.0f;
        if (n < 2 || std::sscanf(arg.data(), "%f", &s) != 1 || !std::isfinite(s) || s < 0.0f || s > 3600.0f) return false;
        out.kind = Input::Kind::wifi_drop;
        out.ms = uint32_t(s * 1000.0f);
        return true;
    }
    if (v == "kick") {
        char* end = nullptr;
        const long s = n > 1 ? std::strtol(arg.data(), &end, 10) : -1;
        if (n < 2 || end == arg.data() || *end != '\0' || s < 0 || s > 64) return false;
        out.kind = Input::Kind::kick;
        out.slot = int16_t(s);
        return true;
    }
    return false;
}

// Many dev tool connection threads push; the hub thread drains each pass.
template <class T, size_t N>
class InputQueue {
public:
    // Any thread. False when full: the input is dropped and counted.
    bool push(const T& v) {
        std::lock_guard<std::mutex> lk(_m);
        if (_count == N) {
            ++_dropped;
            return false;
        }
        _q[(_head + _count) % N] = v;
        ++_count;
        return true;
    }
    // Hub thread. Everything queued, in order, into `out`; 0 when empty or
    // when a pusher holds the lock this instant (the next pass drains it).
    size_t drain(std::array<T, N>& out) {
        std::unique_lock<std::mutex> lk(_m, std::try_to_lock);
        if (!lk.owns_lock()) return 0;
        const size_t n = _count;
        for (size_t i = 0; i < n; ++i) out[i] = _q[(_head + i) % N];
        _head = (_head + n) % N;
        _count = 0;
        return n;
    }
    uint32_t dropped() const {
        std::lock_guard<std::mutex> lk(_m);
        return _dropped;
    }

private:
    mutable std::mutex _m;
    std::array<T, N> _q{};
    size_t _head = 0;
    size_t _count = 0;
    uint32_t _dropped = 0;
};

inline constexpr size_t kInputDepth = 64;
using DevInputQueue = InputQueue<Input, kInputDepth>;

// ---- the snapshot ------------------------------------------------------------

inline constexpr size_t kStripLen = 128;     // MotionArbiter.h kStripLen (static_assert in NeutrinoObserver.cpp)
inline constexpr size_t kMaxSamples = 10000; // 10 s of 1 ms samples
inline constexpr size_t kMaxLogLines = 500;  // SessionLog's ring

struct Client {
    uint8_t slot = 0;
    std::string peer;
    std::string name;
    std::string kind;
    bool hasSession = false;
    uint8_t tier = 0;         // AccessLevel
    uint8_t state = 0;        // HubSessionState
    bool ready = false;
    bool muted = false;
    uint32_t rxDrops = 0;     // the transport's RX ring overflowed
    uint32_t streamAccepted = 0;
    uint32_t streamDropped = 0;
    LinkStats link{};
    LinkFaults faults{};      // this client's own knobs, before the global ones
};

struct Sample {
    uint64_t us = 0;
    float pos = 0.0f;
    float vel = 0.0f;
};

struct LogLine {
    char level = 'I';
    std::string text;
};

struct Snapshot {
    uint64_t seq = 0;
    uint64_t nowUs = 0;
    MotionCensus m{};
    const char* source = "idle";   // a string literal
    uint32_t intentSession = 0;
    uint64_t stripT0Us = 0;
    uint16_t stripN = 0;
    std::array<float, kStripLen> strip{};
    bool estopLatched = false;
    bool pauseLatched = false;
    uint8_t safetyWord = 0;
    bool presenceOpen = false;
    bool pairingOpen = false;
    uint32_t configureAsks = 0;
    uint8_t pairLook = 0;          // ValenceDevice PairLook
    bool pairDown = false;
    bool homeDown = false;
    bool estopDown = false;
    uint32_t wifiDropLeftMs = 0;
    LinkFaults net{};
    uint32_t inputsDropped = 0;
    std::vector<Client> clients;
    std::vector<Sample> samples;
    std::vector<LogLine> log;

    // `older` was never taken: its samples and lines go first in this one.
    void absorb(Snapshot& older) {
        older.samples.insert(older.samples.end(), samples.begin(), samples.end());
        samples.swap(older.samples);
        if (samples.size() > kMaxSamples) samples.erase(samples.begin(), samples.end() - kMaxSamples);
        older.log.insert(older.log.end(), std::make_move_iterator(log.begin()), std::make_move_iterator(log.end()));
        log.swap(older.log);
        if (log.size() > kMaxLogLines) log.erase(log.begin(), log.end() - kMaxLogLines);
    }
};

// Depth one, newest wins, by swap: the hub hands its buffer in and gets the
// old one back to refill, so steady state allocates nothing.
template <class T>
class Mailbox {
public:
    // Hub thread. False when the reader holds the lock this instant: `v` is
    // untouched and keeps accumulating for the next try.
    bool publish(T& v) {
        std::unique_lock<std::mutex> lk(_m, std::try_to_lock);
        if (!lk.owns_lock()) return false;
        if (_fresh) v.absorb(_slot);
        std::swap(_slot, v);
        _fresh = true;
        lk.unlock();
        _ready.notify_one();
        return true;
    }
    // Reader thread. False when nothing new was published since the last take.
    bool take(T& out) {
        std::lock_guard<std::mutex> lk(_m);
        return takeLocked(out);
    }
    // Reader thread. take(), waiting up to `wait` for a publish. Do not pace
    // the reader with a sleep instead: in a windowless Windows process a sleep
    // rounds up to the 15.6 ms timer tick and beats against the hub's cadence.
    bool take(T& out, std::chrono::milliseconds wait) {
        std::unique_lock<std::mutex> lk(_m);
        _ready.wait_for(lk, wait, [this] { return _fresh; });
        return takeLocked(out);
    }

private:
    bool takeLocked(T& out) {
        if (!_fresh) return false;
        std::swap(out, _slot);
        _fresh = false;
        return true;
    }

    std::mutex _m;
    std::condition_variable _ready;
    T _slot{};
    bool _fresh = false;
};

using SnapshotMailbox = Mailbox<Snapshot>;

// ---- JSON out ----------------------------------------------------------------

inline void appendf(std::string& out, const char* fmt, ...) {
    std::array<char, 256> buf{};
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(buf.data(), buf.size(), fmt, ap);
    va_end(ap);
    if (n > 0) out.append(buf.data(), size_t(std::min<int>(n, int(buf.size()) - 1)));
}

inline void appendNum(std::string& out, float v, int decimals = 3) {
    appendf(out, "%.*f", decimals, std::isfinite(v) ? double(v) : 0.0);
}

inline void appendStr(std::string& out, std::string_view s) {
    out += '"';
    for (const char c : s) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (uint8_t(c) < 0x20) {
            appendf(out, "\\u%04x", unsigned(uint8_t(c)));
        } else {
            out += c;
        }
    }
    out += '"';
}

inline const char* tierName(uint8_t t) {
    return t == 2 ? "configure" : t == 1 ? "control" : "watch";
}
inline const char* sessionStateName(uint8_t s) {
    constexpr std::array<const char*, 6> k{"free", "validating", "granted", "live", "stale", "closed"};
    return s < k.size() ? k[s] : "unknown";
}
inline const char* pairLookName(uint8_t l) {
    constexpr std::array<const char*, 5> k{"quiet", "window", "request", "refused", "demoted"};
    return l < k.size() ? k[l] : "quiet";
}

inline void appendFaults(std::string& out, const LinkFaults& f) {
    appendf(out, "{\"lat\":%u,\"jit\":%u,\"loss\":", unsigned(f.latency_ms), unsigned(f.jitter_ms));
    appendNum(out, f.loss_pct, 1);
    out += '}';
}

// The snapshot as one JSON object, "t":"snap". Replaces `out`.
inline void encodeSnapshot(const Snapshot& s, std::string& out) {
    const MotionCensus& m = s.m;
    out.clear();
    appendf(out, "{\"t\":\"snap\",\"seq\":%llu,\"us\":%llu,\"m\":{\"pos\":",
            static_cast<unsigned long long>(s.seq), static_cast<unsigned long long>(s.nowUs));
    appendNum(out, m.position_mm);
    out += ",\"vel\":", appendNum(out, m.velocity_mm_s, 2);
    out += ",\"plan\":", appendNum(out, m.plan_mm);
    out += ",\"target\":", appendNum(out, m.target_mm);
    out += ",\"demand\":", appendNum(out, m.demand_mm);
    out += ",\"wmin\":", appendNum(out, m.win_min, 2);
    out += ",\"wmax\":", appendNum(out, m.win_max, 2);
    out += ",\"rail\":", appendNum(out, m.rail_mm, 2);
    out += ",\"vmax\":", appendNum(out, m.input_vmax_mm_s, 1);
    out += ",\"amax\":", appendNum(out, m.input_amax_mm_s2, 1);
    out += ",\"jmax\":", appendNum(out, m.input_jmax_mm_s3, 0);
    out += ",\"homeRail\":", appendNum(out, m.home_rail_mm, 2);
    appendf(out, ",\"homed\":%s,\"homing\":%s,\"estop\":%s,\"paused\":%s,\"busy\":%s,\"stream\":%s,"
                 "\"motor\":%s,\"override\":%s,\"returning\":%s,\"osc\":%s,\"source\":",
            m.homed ? "true" : "false", m.homing ? "true" : "false", m.estop ? "true" : "false",
            m.paused ? "true" : "false", m.busy ? "true" : "false", m.stream ? "true" : "false",
            m.motor_on ? "true" : "false", m.override_mode ? "true" : "false", m.returning ? "true" : "false",
            m.osc_active ? "true" : "false");
    appendStr(out, s.source);
    appendf(out, ",\"owner\":%u,\"plans\":%u,\"anomalies\":%u,\"rejected\":%u,\"strokes\":%u,\"homes\":%u,\"homeFails\":%u}",
            unsigned(s.intentSession), unsigned(m.plans), unsigned(m.anomalies), unsigned(m.rejected),
            unsigned(m.strokes), unsigned(m.homes), unsigned(m.home_fails));

    appendf(out, ",\"strip\":{\"t0\":%llu,\"p\":[", static_cast<unsigned long long>(s.stripT0Us));
    for (size_t i = 0; i < s.stripN && i < kStripLen; ++i) {
        if (i) out += ',';
        appendNum(out, s.strip[i], 2);
    }
    appendf(out, "]},\"safety\":{\"estop\":%s,\"pause\":%s,\"word\":%u}", s.estopLatched ? "true" : "false",
            s.pauseLatched ? "true" : "false", unsigned(s.safetyWord));
    appendf(out, ",\"pair\":{\"presence\":%s,\"pairing\":%s,\"asks\":%u,\"look\":\"%s\"}",
            s.presenceOpen ? "true" : "false", s.pairingOpen ? "true" : "false", unsigned(s.configureAsks),
            pairLookName(s.pairLook));
    appendf(out, ",\"phys\":{\"pair\":%s,\"home\":%s,\"estop\":%s}", s.pairDown ? "true" : "false",
            s.homeDown ? "true" : "false", s.estopDown ? "true" : "false");
    appendf(out, ",\"wifi\":{\"dropLeft\":%u},\"net\":", unsigned(s.wifiDropLeftMs));
    appendFaults(out, s.net);
    appendf(out, ",\"inputsDropped\":%u,\"clients\":[", unsigned(s.inputsDropped));
    for (size_t i = 0; i < s.clients.size(); ++i) {
        const Client& c = s.clients[i];
        if (i) out += ',';
        appendf(out, "{\"slot\":%u,\"peer\":", unsigned(c.slot));
        appendStr(out, c.peer);
        out += ",\"name\":", appendStr(out, c.name);
        out += ",\"kind\":", appendStr(out, c.kind);
        appendf(out, ",\"session\":%s,\"sid\":%u,\"tier\":\"%s\",\"state\":\"%s\",\"ready\":%s,\"muted\":%s,"
                     "\"rtt\":%d,\"queued\":%u,\"drops\":%u,\"retx\":%u,\"refused\":%u,\"failed\":%u,"
                     "\"streamIn\":%u,\"streamDropped\":%u,\"f\":",
                c.hasSession ? "true" : "false", unsigned(c.link.session_id), tierName(c.tier),
                sessionStateName(c.state), c.ready ? "true" : "false", c.muted ? "true" : "false",
                int(c.link.rtt_ms), unsigned(c.link.queued), unsigned(c.rxDrops), unsigned(c.link.retransmits),
                unsigned(c.link.refused), unsigned(c.link.write_fails), unsigned(c.streamAccepted),
                unsigned(c.streamDropped));
        appendFaults(out, c.faults);
        out += '}';
    }
    out += "],\"s\":[";
    for (size_t i = 0; i < s.samples.size(); ++i) {
        const Sample& p = s.samples[i];
        if (i) out += ',';
        appendf(out, "%llu,", static_cast<unsigned long long>(p.us));
        appendNum(out, p.pos);
        out += ',';
        appendNum(out, p.vel, 2);
    }
    out += "],\"log\":[";
    for (size_t i = 0; i < s.log.size(); ++i) {
        if (i) out += ',';
        appendf(out, "[\"%c\",", s.log[i].level);
        appendStr(out, s.log[i].text);
        out += ']';
    }
    out += "]}";
}

}  // namespace valence::devtool
