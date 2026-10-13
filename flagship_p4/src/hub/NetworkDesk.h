#pragma once

// NetworkDesk -- the saved-network list on the hub (RFC-116, amending SPEC
// §13.9): the one gate every 0x000F op passes whatever binding carried it,
// the list edits, the live-network refusal, persistence, and the two STATE
// channels networks (0x0017, watch) and network-names (0x0018, configure)
// Constraints:
// - HUB TASK ONLY. The station joins on its own task; the two meet through
//   INetStation, never through shared memory.
// - THE BINDING RULES LIVE IN gate() AND ONLY THERE: wifi_join only on a
//   credential binding (BLE GATT, serial, in-process) with a pairing window
//   open or the hub factory-fresh; the list ops on any binding at configure.
//   The live-network refusal in apply() is the one other binding-dependent
//   rule: on a network binding the entry the station is joined through may
//   not be removed or overwritten.
// - SSIDs leave only through network-names, whose catalog access is
//   configure. Passphrases leave through nothing: ECHO answers `true` for
//   both keys (SPEC §8.8 Secrets), and no log line here names either.
// - Hardware-free: valencesim runs it with its own station.
// - TODO(RFC-116): valence::net::draft numbers become registry constants
//   when the RFC lands.
// See: Valence SPEC.md §8.8, §12.2, §13.9; RFC-QUEUE.md RFC-116;
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
};

// First boot of a list image: the one network the board already joined
// becomes slot 1, a provisioned pair (an older image's NVS "sta") before the
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
    static constexpr size_t kNamesBytes = 1 + net::SavedNetworks::kCapacity * 32;   // open + ssid x4 str32
    static constexpr uint8_t kNoLive = 0xFF;

    explicit NetworkDesk(INetStation& station) : _station(station) {}

    // Boot, before the first tick: the list the station already joins from.
    void adopt(const net::SavedNetworks& list) {
        _list = list;
        _offerDue = true;
        _namesDue = true;
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
                return std::nullopt;
            default:
                return Refusal{NackCode::UNSUPPORTED_OP, "op"};
        }
    }

    // wifi_join, before the join starts: refused when the joined network
    // would have nowhere to be stored. A `slot` names the entry to replace.
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
        if (!_list.saveTarget(s, ssid->tstr_val) && _list.count() == _list.capacity())
            return Refusal{NackCode::INVALID_VALUE, "list full"};
        return std::nullopt;
    }

    // wifi_join joined: the credentials take their entry. Returns the slot.
    std::optional<size_t> storeJoined(std::optional<size_t> slot, std::string_view ssid, std::string_view pass) {
        const auto r = _list.save(slot, ssid, pass);
        if (!r.ok) return std::nullopt;
        changed();
        return r.saved.slot;
    }

    // net_save, net_remove, net_move. The caller passed gate() first.
    Applied apply(const IntentValueMap& in, Binding binding) {
        const std::optional<uint8_t> op = opOf(in);
        if (const IntentValue* g = field(in, net::draft::key_generation)) {
            if (g->kind != IntentValue::Kind::U64) return refuse(NackCode::INVALID_VALUE, "generation");
            if (g->u64_val != _generation) return refuse(NackCode::CONFLICT, "list changed");
        }
        const std::optional<size_t> live = _list.findId(_station.liveId());
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
            if (binding == Binding::network && target && target == live)
                return refuse(NackCode(net::draft::nack_live_network), "in use: the hub is joined through it");
            const auto r = _list.save(s, ssid->tstr_val, pass->tstr_val);
            if (!r.ok) return refuse(NackCode::INVALID_VALUE, "list full");
            changed();
            push(out, net::draft::key_slot, IntentValue::ofU64(r.saved.slot));
            push(out, net::kKeySsid, IntentValue::ofBool(true));
            push(out, net::kKeyPassphrase, IntentValue::ofBool(true));
            GLOGI(kTag, "slot %u %s", unsigned(r.saved.slot + 1), r.saved.replaced ? "replaced" : "added");
        } else if (op == net::draft::op_net_remove) {
            if (!s || *s >= _list.count()) return refuse(NackCode::INVALID_VALUE, "slot");
            if (binding == Binding::network && s == live)
                return refuse(NackCode(net::draft::nack_live_network), "in use: the hub is joined through it");
            _list.remove(*s);
            changed();
            push(out, net::draft::key_slot, IntentValue::ofU64(*s));
            GLOGI(kTag, "slot %u removed", unsigned(*s + 1));
        } else if (op == net::draft::op_net_move) {
            const auto to = slotOf(in, net::draft::key_to);
            if (!s || *s >= _list.count()) return refuse(NackCode::INVALID_VALUE, "slot");
            if (!to || !to->has_value() || **to >= _list.count()) return refuse(NackCode::INVALID_VALUE, "to");
            _list.move(*s, **to);
            if (*s != **to) changed();
            push(out, net::draft::key_slot, IntentValue::ofU64(*s));
            push(out, net::draft::key_to, IntentValue::ofU64(**to));
        } else {
            return refuse(NackCode::UNSUPPORTED_OP, "op");
        }
        push(out, net::draft::key_generation, IntentValue::ofU64(_generation));
        // The deterministic profile (SPEC 5.3) orders map keys ascending.
        std::sort(out.fields.begin(), out.fields.begin() + out.count,
                  [](const IntentValueField& a, const IntentValueField& b) { return a.key < b.key; });
        return Applied::ok(out);
    }

    // Once per hub tick: persist, hand the station the list, publish.
    void tick() {
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
    }

    void packNetworks(std::span<std::byte, kNetworksBytes> s) const {
        putU16(s.subspan(0, 2), _generation);
        putU8(s.subspan(2, 1), uint8_t(_list.capacity()));
        putU8(s.subspan(3, 1), uint8_t(_list.count()));
        const auto live = _list.findId(_station.liveId());
        putU8(s.subspan(4, 1), live ? uint8_t(*live) : kNoLive);
        for (size_t i = 0; i < net::SavedNetworks::kCapacity; ++i)
            putU8(s.subspan(5 + i, 1), uint8_t(i < _list.count() ? _station.rssiOf(_list.id(i)) : net::kRssiUnseen));
    }

    void packNames(std::span<std::byte, kNamesBytes> n) const {
        uint8_t open = 0;
        for (size_t i = 0; i < _list.count(); ++i) {
            if (_list.open(i)) open = uint8_t(open | (1u << i));
            const std::string_view ssid = _list.ssid(i);
            std::memcpy(n.data() + 1 + i * 32, ssid.data(), ssid.size());
        }
        n[0] = std::byte(open);
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
    static void push(IntentValueMap& m, uint8_t key, IntentValue v) {
        if (m.count < m.fields.size()) m.fields[m.count++] = {key, v};
    }
    static Applied refuse(NackCode code, const char* detail) { return Applied::err(Refusal{code, detail}); }

    void changed() {
        ++_generation;
        _persistDue = true;
        _offerDue = true;
        _namesDue = true;
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
};

}  // namespace valence
