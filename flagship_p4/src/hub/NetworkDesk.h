#pragma once

// NetworkDesk -- the saved-network list on the hub (RFC-116, amending SPEC
// §13.9): the one gate every 0x000F op passes whatever binding carried it,
// the list edits, the live-network refusal, persistence, and the two STATE
// channels networks (0x0017, watch) and network-names (0x0018, configure);
// the scan op and the networks it heard on visible-networks (0x0019) and
// visible-networks-more (0x001A), both configure (RFC-118)
// Constraints:
// - HUB TASK ONLY. The station joins on its own task; the two meet through
//   INetStation, never through shared memory.
// - THE BINDING RULES LIVE HERE AND ONLY HERE: gate() admits wifi_join only
//   on a credential binding (BLE GATT, serial, in-process) with a pairing
//   window open or the hub factory-fresh, and the list ops on any binding at
//   configure; mayEditLive() lets an edit remove or overwrite the entry the
//   station is joined through only under wifi_join's own terms.
// - SSIDs leave only through network-names and the two visible-networks
//   channels (configure) and the ECHO to the configure session that sent them. Passphrases leave through nothing: ECHO
//   answers `true` for that key (SPEC §8.8 Secrets), and no log line here
//   names either.
// - SCANS (RFC-118), two policies by INetStation::linked(), never while
//   motion runs: an ask during motion is refused INTERLOCK, one accepted
//   earlier waits for motion to stop, and a scan running when motion starts
//   is stopped on that tick and asked again once motion stops;
//   joined (associated or associating): on demand only, at most one start per
//   scan_min_interval_ms, asks inside the interval coalescing into one, and
//   the station keeps the link within scan_offchannel_max_ms of its home
//   channel at a time;
//   not joined (config mode, the offline BLE fallback between attempts): no
//   link to protect, so a scan asked runs at once, and while a configure
//   session is present the desk rescans every scan_offline_period_ms.
// - A CONFIGURE SESSION IS PRESENT when one is ready at configure on any
//   binding, serial and in-process included; a parked (STALE) one is not. An
//   ask pending when the last one leaves is dropped.
// - Hardware-free: valencesim runs it with its own station.
// - TODO(RFC-116, RFC-118): valence::net::draft numbers become registry
//   constants when the RFCs land.
// See: Valence SPEC.md §8.8, §12.2, §13.9; RFC-QUEUE.md RFC-116, RFC-118;
// valence/net/saved_networks.hpp

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>
#include <utility>

#include "geiger/geiger.h"
#include "valence/core/result.hpp"
#include "valence/hub/hub.hpp"
#include "valence/net/saved_networks.hpp"
#include "valence/util/byte_io.hpp"
#include "valence/wire/messages/intent.hpp"

namespace valence {

// Which kind of binding carried an op. `credential` is BLE GATT (§13.4),
// serial (§13.5) or in-process (§13.6); every other binding rides the network
// the list configures.
enum class Binding : uint8_t { network, credential };

// The platform half: the station that joins and the durable store.
class INetStation {
public:
    virtual ~INetStation() = default;
    // Hands the station the current list. false = it has not taken the last
    // one yet; offer again next tick.
    virtual bool offerList(const net::SavedNetworks& list) = 0;
    // SavedNetworks::id of the entry the station is joined through; 0 = none.
    virtual uint8_t liveId() const = 0;
    // That entry's RSSI: the link's for the live entry, else its strongest
    // BSSID in the station's last scan; net::kRssiUnseen when not heard.
    virtual int8_t rssiOf(uint8_t id) const = 0;
    // Makes the list durable. false = not now (an OTA transfer holds NVS
    // writes); asked again next tick.
    virtual bool persist(std::span<const uint8_t> blob) = 0;
    // Associated (IP or not) or a connect under way: the joined policy.
    virtual bool linked() const = 0;
    // Starts one scan of every channel and returns at once; false = none can
    // start now (a join runs, the radio is off), asked again next tick.
    virtual bool beginScan() = 0;
    // A scan begun is still under way, started or queued behind a join.
    virtual bool scanRunning() const = 0;
    // Stops the scan under way before its next channel; it publishes nothing.
    virtual void stopScan() = 0;
    // A scan finished since the last call: its networks go to `out` (which
    // advances out.scan()) and true. Every finished scan counts, the join
    // rule's own included.
    virtual bool takeScan(net::VisibleNetworks& out) = 0;
};

// First boot of a list image: the one network the board already joined
// becomes slot 0, a provisioned pair (an older image's NVS "sta") before the
// compiled-in one. An invalid pair yields an empty list.
inline net::SavedNetworks firstBootList(std::optional<std::pair<std::string_view, std::string_view>> provisioned,
                                        std::string_view compiledSsid, std::string_view compiledPass) {
    net::SavedNetworks l;
    if (provisioned) l.save(std::nullopt, provisioned->first, provisioned->second);
    else l.save(std::nullopt, compiledSsid, compiledPass);
    return l;
}

class NetworkDesk {
public:
    struct Refusal {
        NackCode code = NackCode::INVALID_VALUE;
        const char* detail = "";   // a literal: static storage, never a credential
    };
    using Applied = Result<IntentValueMap, Refusal>;

    static constexpr size_t kNetworksBytes = 9;    // generation u16, capacity, count, live, rssi x4
    static constexpr size_t kNamesBytes = 2 + 1 + net::SavedNetworks::kCapacity * 32;   // generation, open, ssid x4 str32
    static constexpr uint8_t kNoLive = 0xFF;
    static constexpr size_t kVisibleBytes = net::VisibleNetworks::kChannelBytes;

    explicit NetworkDesk(INetStation& station) : _station(station) {}

    // Boot, before the first tick: the list the station already joins from.
    void adopt(const net::SavedNetworks& list) {
        _list = list;
        _offerDue = true;
        _namesDue = true;
        _visibleDue = true;
    }
    // Where tick() publishes. Without it nothing publishes (a native suite).
    void attach(Hub& hub) { _hub = &hub; }

    const net::SavedNetworks& list() const { return _list; }
    uint16_t generation() const { return _generation; }

    static std::optional<uint8_t> opOf(const IntentValueMap& in) {
        const IntentValue* v = field(in, net::kKeyOp);
        if (v == nullptr || v->kind != IntentValue::Kind::U64 || v->u64_val > 0xFF) return std::nullopt;
        return uint8_t(v->u64_val);
    }

    // THE binding gate. `windowOrFresh`: a §12.3 association window is open,
    // or the hub holds no configure token. The session's configure tier is
    // checked before this, by whoever received the frame.
    static std::optional<Refusal> gate(std::optional<uint8_t> op, Binding binding, bool windowOrFresh) {
        if (!op) return Refusal{NackCode::INVALID_VALUE, "op"};
        switch (*op) {
            case provisioning_ops::wifi_join:
                if (binding != Binding::credential)
                    return Refusal{NackCode::ACCESS_DENIED, "wifi_join needs BLE, serial or in-process"};
                if (!windowOrFresh) return Refusal{NackCode::ACCESS_DENIED, "pairing window closed"};
                return std::nullopt;
            case net::draft::op_net_save:
            case net::draft::op_net_remove:
            case net::draft::op_net_move:
            case net::draft::op_net_scan:
                return std::nullopt;
            default:
                return Refusal{NackCode::UNSUPPORTED_OP, "op"};
        }
    }

    // RFC-116 item 5: an edit that removes or overwrites the entry the
    // station is joined through needs wifi_join's own terms, because it can
    // strand the hub at its next reconnect.
    static bool mayEditLive(Binding binding, bool windowOrFresh) {
        return binding == Binding::credential && windowOrFresh;
    }

    // wifi_join, before the join starts: the credentials and the slot. A full
    // list is no refusal: storeJoined() replaces an entry.
    std::optional<Refusal> checkJoinTarget(const IntentValueMap& in) const {
        const IntentValue* ssid = field(in, net::kKeySsid);
        const IntentValue* pass = field(in, net::kKeyPassphrase);
        if (ssid == nullptr || ssid->kind != IntentValue::Kind::Tstr) return Refusal{NackCode::INVALID_VALUE, "ssid"};
        if (pass == nullptr || pass->kind != IntentValue::Kind::Tstr)
            return Refusal{NackCode::INVALID_VALUE, "passphrase"};
        if (const auto bad = net::checkCredentials(ssid->tstr_val, pass->tstr_val))
            return Refusal{NackCode::INVALID_VALUE, bad->data()};
        const auto slot = slotOf(in, net::draft::key_slot);
        if (slot.has_value() && !slot->has_value()) return Refusal{NackCode::INVALID_VALUE, "slot"};
        const std::optional<size_t> s = slot ? *slot : std::nullopt;
        if (s && *s >= _list.count()) return Refusal{NackCode::INVALID_VALUE, "slot"};
        if (const auto same = _list.find(ssid->tstr_val); s && same && *same != *s)
            return Refusal{NackCode::INVALID_VALUE, "duplicate ssid"};
        return std::nullopt;
    }

    // wifi_join joined: the credentials take SavedNetworks::joinTarget()'s
    // entry. A list edited during the join (the slot gone, the SSID saved
    // elsewhere) falls back to the target without a slot, which always
    // stores. Returns the slot.
    size_t storeJoined(std::optional<size_t> slot, std::string_view ssid, std::string_view pass) {
        const std::optional<size_t> live = _list.findId(_station.liveId());
        auto r = _list.save(_list.joinTarget(slot, ssid, live), ssid, pass);
        if (!r.ok) r = _list.save(_list.joinTarget(std::nullopt, ssid, live), ssid, pass);
        changed();
        GLOGI(kTag, "slot %u %s by wifi_join", unsigned(r.saved.slot), r.saved.replaced ? "replaced" : "added");
        return r.saved.slot;
    }

    // net_save, net_remove, net_move, net_scan. The caller passed gate()
    // first, with the same `binding` and `windowOrFresh`.
    Applied apply(const IntentValueMap& in, Binding binding, bool windowOrFresh) {
        const std::optional<uint8_t> op = opOf(in);
        if (const IntentValue* g = field(in, net::draft::key_generation)) {
            if (g->kind != IntentValue::Kind::U64) return refuse(NackCode::INVALID_VALUE, "generation");
            if (g->u64_val != _generation) return refuse(NackCode::CONFLICT, "list changed");
        }
        const std::optional<size_t> live =
            mayEditLive(binding, windowOrFresh) ? std::nullopt : _list.findId(_station.liveId());
        const auto slot = slotOf(in, net::draft::key_slot);
        if (slot.has_value() && !slot->has_value()) return refuse(NackCode::INVALID_VALUE, "slot");
        const std::optional<size_t> s = slot ? *slot : std::nullopt;

        IntentValueMap out;
        out.fields[0] = {net::kKeyOp, IntentValue::ofU64(op.value_or(0))};
        out.count = 1;

        if (op == net::draft::op_net_save) {
            const IntentValue* ssid = field(in, net::kKeySsid);
            const IntentValue* pass = field(in, net::kKeyPassphrase);
            if (ssid == nullptr || ssid->kind != IntentValue::Kind::Tstr) return refuse(NackCode::INVALID_VALUE, "ssid");
            if (pass == nullptr || pass->kind != IntentValue::Kind::Tstr)
                return refuse(NackCode::INVALID_VALUE, "passphrase");
            if (const auto bad = net::checkCredentials(ssid->tstr_val, pass->tstr_val))
                return refuse(NackCode::INVALID_VALUE, bad->data());
            if (s && *s >= _list.count()) return refuse(NackCode::INVALID_VALUE, "slot");
            const std::optional<size_t> target = _list.saveTarget(s, ssid->tstr_val);
            if (target && target == live)
                return refuse(NackCode(net::draft::nack_live_network), "in use: the hub is joined through it");
            const auto r = _list.save(s, ssid->tstr_val, pass->tstr_val);
            if (!r.ok)
                return refuse(NackCode::INVALID_VALUE,
                              r.error == net::SavedNetworks::Error::duplicate ? "duplicate ssid" : "list full");
            changed();
            push(out, net::draft::key_slot, IntentValue::ofU64(r.saved.slot));
            // Not secret (RFC-116 item 7): the sender reads it on 0x0018 anyway.
            push(out, net::kKeySsid, IntentValue::ofTstr(ssid->tstr_val));
            push(out, net::kKeyPassphrase, IntentValue::ofBool(true));
            GLOGI(kTag, "slot %u %s", unsigned(r.saved.slot), r.saved.replaced ? "replaced" : "added");
        } else if (op == net::draft::op_net_remove) {
            if (!s || *s >= _list.count()) return refuse(NackCode::INVALID_VALUE, "slot");
            if (s == live)
                return refuse(NackCode(net::draft::nack_live_network), "in use: the hub is joined through it");
            _list.remove(*s);
            changed();
            push(out, net::draft::key_slot, IntentValue::ofU64(*s));
            GLOGI(kTag, "slot %u removed", unsigned(*s));
        } else if (op == net::draft::op_net_move) {
            const auto to = slotOf(in, net::draft::key_to);
            if (!s || *s >= _list.count()) return refuse(NackCode::INVALID_VALUE, "slot");
            if (!to || !to->has_value() || **to >= _list.count()) return refuse(NackCode::INVALID_VALUE, "to");
            _list.move(*s, **to);
            if (*s != **to) changed();
            push(out, net::draft::key_slot, IntentValue::ofU64(*s));
            push(out, net::draft::key_to, IntentValue::ofU64(**to));
        } else if (op == net::draft::op_net_scan) {
            // A state refusal, as for axis.flipped and homing in motion: the
            // client asks again once motion stops. The results come on
            // 0x0019/0x001A, not here.
            if (_motion) return refuse(NackCode::INTERLOCK, "motion running");
            _scanWanted = true;
        } else {
            return refuse(NackCode::UNSUPPORTED_OP, "op");
        }
        push(out, net::draft::key_generation, IntentValue::ofU64(_generation));
        // The deterministic profile (SPEC 5.3) orders map keys ascending.
        std::sort(out.fields.begin(), out.fields.begin() + out.count,
                  [](const IntentValueField& a, const IntentValueField& b) { return a.key < b.key; });
        return Applied::ok(out);
    }

    // Once per hub tick: persist, hand the station the list, start a scan that
    // is due, publish. `motion`: a plan, a stream source, homing or a return
    // runs now.
    void tick(uint32_t nowMs, bool motion) {
        _motion = motion;
        const bool present = configurePresent();
        if (!present) _scanWanted = false;
        const bool joined = _station.linked();
        const uint32_t since = nowMs - _scanAtMs;
        if (!joined && present && !_scanRunning && (!_scanned || since >= net::draft::scan_offline_period_ms))
            _scanWanted = true;
        const uint32_t gap = joined ? net::draft::scan_min_interval_ms : 0;
        if (_scanWanted && !_scanRunning && !motion && (!_scanned || since >= gap) && _station.beginScan()) {
            _scanWanted = false;
            _scanned = true;
            _scanRunning = true;
            _stopAsked = false;
            _scanAtMs = nowMs;
        }
        const bool heard = _station.takeScan(_visible);
        if (heard) _visibleDue = true;
        if (_scanRunning) {
            if (heard) {
                _scanRunning = false;
            } else if (!_station.scanRunning()) {
                _scanRunning = false;
                _scanWanted = _scanWanted || _stopAsked;   // motion cut it: again once motion stops
            } else if (motion && !_stopAsked) {
                _station.stopScan();
                _stopAsked = true;
            }
        }
        if (_persistDue) {
            std::array<uint8_t, net::SavedNetworks::kBlobBytes> blob{};
            _list.encode(blob);
            const bool done = _station.persist(blob);
            net::wipeBytes(blob.data(), blob.size());
            if (done) _persistDue = false;
        }
        if (_offerDue && _station.offerList(_list)) _offerDue = false;
        if (_hub == nullptr) return;

        std::array<std::byte, kNetworksBytes> s{};
        packNetworks(s);
        if (s != _lastNetworks || !_networksSent) {
            if (_hub->publishState(net::draft::ch_networks, s)) {
                _lastNetworks = s;
                _networksSent = true;
            }
        }
        if (_namesDue) {
            std::array<std::byte, kNamesBytes> n{};
            packNames(n);
            if (_hub->publishState(net::draft::ch_network_names, n)) _namesDue = false;
            net::wipeBytes(n.data(), n.size());
        }
        if (_visibleDue) {
            std::array<std::byte, kVisibleBytes> a{}, b{};
            _visible.pack(0, _list, a);
            _visible.pack(1, _list, b);
            const bool first = _hub->publishState(net::draft::ch_visible_networks, a);
            const bool second = _hub->publishState(net::draft::ch_visible_networks_more, b);
            if (first && second) _visibleDue = false;
        }
    }

    bool scanPending() const { return _scanWanted; }
    const net::VisibleNetworks& visible() const { return _visible; }

    void packNetworks(std::span<std::byte, kNetworksBytes> s) const {
        putU16(s.subspan(0, 2), _generation);
        putU8(s.subspan(2, 1), uint8_t(_list.capacity()));
        putU8(s.subspan(3, 1), uint8_t(_list.count()));
        const auto live = _list.findId(_station.liveId());
        putU8(s.subspan(4, 1), live ? uint8_t(*live) : kNoLive);
        for (size_t i = 0; i < net::SavedNetworks::kCapacity; ++i)
            putU8(s.subspan(5 + i, 1), uint8_t(i < _list.count() ? _station.rssiOf(_list.id(i)) : net::kRssiUnseen));
    }

    // generation first, so a client pairs these names with the networks
    // sample of the same generation.
    void packNames(std::span<std::byte, kNamesBytes> n) const {
        putU16(n.subspan(0, 2), _generation);
        uint8_t open = 0;
        for (size_t i = 0; i < _list.count(); ++i) {
            if (_list.open(i)) open = uint8_t(open | (1u << i));
            const std::string_view ssid = _list.ssid(i);
            std::memcpy(n.data() + 3 + i * 32, ssid.data(), ssid.size());
        }
        n[2] = std::byte(open);
    }

private:
    static constexpr const char* kTag = "nets";

    static const IntentValue* field(const IntentValueMap& in, uint8_t key) {
        for (uint32_t i = 0; i < in.count && i < in.fields.size(); ++i)
            if (in.fields[i].key == key) return &in.fields[i].value;
        return nullptr;
    }
    // nullopt = absent; an inner nullopt = present but not a small uint.
    static std::optional<std::optional<size_t>> slotOf(const IntentValueMap& in, uint8_t key) {
        const IntentValue* v = field(in, key);
        if (v == nullptr) return std::nullopt;
        if (v->kind != IntentValue::Kind::U64 || v->u64_val >= net::SavedNetworks::kCapacity)
            return std::optional<size_t>{};
        return std::optional<size_t>{size_t(v->u64_val)};
    }
    // A ready configure session on any binding (the picker's audience). A
    // parked one has nobody looking; it gets the retained rows on resume.
    bool configurePresent() const {
        if (_hub == nullptr) return false;
        for (size_t i = 0;; ++i) {
            const HubSession* s = _hub->sessionBySlot(i);
            if (s == nullptr) return false;
            if (s->occupied() && s->state != HubSessionState::STALE && s->ready &&
                uint8_t(s->role) >= uint8_t(AccessLevel::configure))
                return true;
        }
    }
    static void push(IntentValueMap& m, uint8_t key, IntentValue v) {
        if (m.count < m.fields.size()) m.fields[m.count++] = {key, v};
    }
    static Applied refuse(NackCode code, const char* detail) { return Applied::err(Refusal{code, detail}); }

    void changed() {
        ++_generation;
        _persistDue = true;
        _offerDue = true;
        _namesDue = true;
        _visibleDue = true;   // the saved bits
    }

    INetStation& _station;
    Hub* _hub = nullptr;
    net::SavedNetworks _list{};
    uint16_t _generation = 0;
    bool _persistDue = false;
    bool _offerDue = false;
    bool _namesDue = false;
    bool _networksSent = false;
    std::array<std::byte, kNetworksBytes> _lastNetworks{};
    net::VisibleNetworks _visible{};
    bool _visibleDue = false;
    bool _scanWanted = false;
    bool _scanned = false;
    bool _scanRunning = false;
    bool _stopAsked = false;
    bool _motion = false;
    uint32_t _scanAtMs = 0;
};

}  // namespace valence
