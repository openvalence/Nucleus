#pragma once

// ValenceProvisioning -- the `provisioning` desk (core channel 0x000F, SPEC
// §13.9): wifi_join from a binding that may carry credentials, its gate, one
// join at a time, and the deferred unicast answer
// Constraints:
// - HUB TASK ONLY: offer(), tick() and forget(). The station joins on its own
//   task (ValenceProvisioning.cpp) and is polled here, never waited on.
// - CREDENTIALS NEVER LEAVE: no log line, ECHO value, NACK detail or STATE
//   carries the SSID or the passphrase. The ECHO answers `true` for both keys
//   (SPEC §8.8 Secrets). The desk's one copy is the attempt's, wiped once the
//   station has joined and the store has persisted it, or once it failed.
// - THE BINDING HALF OF THE GATE IS BY CONSTRUCTION: only a binding that may
//   carry credentials (USB serial now, BLE GATT later) calls offer(). A 0x000F
//   INTENT on any other binding reaches the hub, whose delegate refuses it.
// - A FEATURE EXISTS IFF ITS CHANNEL DOES (SPEC §6.3): with no INTENT entry
//   0x000F in the catalog, offer() declines and the hub answers
//   UNKNOWN_CHANNEL.
// - This desk, not the hub, answers what it consumes: the hub's own gates run
//   in the hub's order (session, ready, idempotency, access, CAS) without its
//   rate limiter, then the window and the field checks. The last success ECHO
//   answers a duplicate intent_id; a second join while one runs or persists is
//   BUSY.
// - TODO(val-9u0.22): moves behind the hub delegate once the library answers an
//   intent later than applyIntent() returns and tells it the binding.
// See: Valence SPEC.md §6.4, §8.8, §9.3, §12.3, §13.9; ValenceSerialPort.h

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>

#include "geiger/geiger.h"
#include "valence/hub/hub.hpp"
#include "valence/transport/transport.hpp"
#include "valence/wire/frame_header.hpp"
#include "valence/wire/messages/echo.hpp"
#include "valence/wire/messages/intent.hpp"
#include "valence/wire/messages/nack.hpp"

namespace valence {

// ---- the station seam ----------------------------------------------------------

// Why a join did not complete. The NACK detail names the reason, never a
// credential.
enum class JoinFailure : uint8_t { station_down, auth, not_found, no_address, timeout };

struct JoinOutcome {
    bool joined = false;
    // cbor_keys 47's packing: 192.168.1.229 is 0xC0A801E5.
    uint32_t ipv4 = 0;
    JoinFailure why = JoinFailure::timeout;
};

// The platform half: the radio join and the durable store. The board's is
// ValenceProvisioning.cpp; the native suite fakes it.
class IStation {
public:
    virtual ~IStation() = default;
    // Starts one join and returns at once; false when none can start.
    virtual bool beginJoin(std::string_view ssid, std::string_view passphrase) = 0;
    // The started join's outcome, exactly once, no later than
    // provision_join_timeout_ms plus the time to restore the prior network.
    // nullopt while it runs. A failed join has restored the prior network.
    virtual std::optional<JoinOutcome> pollJoin() = 0;
    // Makes credentials that just joined durable. false = not now, ask again
    // next tick (an OTA transfer holds NVS writes).
    virtual bool persist(std::string_view ssid, std::string_view passphrase) = 0;
};

// ---- the desk ------------------------------------------------------------------

class Provisioning {
public:
    // The channel's schema keys (registry.yaml core_channels 0x000F).
    static constexpr uint8_t kKeyOp = 1;
    static constexpr uint8_t kKeySsid = 2;
    static constexpr uint8_t kKeyPassphrase = 3;
    static constexpr uint8_t kKeyIpv4 = 4;
    static constexpr uint8_t kKeyWsPort = 5;
    // IEEE 802.11: an SSID is 1..32 octets. A WPA2 passphrase is 8..63
    // printable ASCII, or exactly 64 hex digits of raw PSK. Empty is an open
    // network (SPEC §13.9).
    static constexpr size_t kSsidMax = 32;
    static constexpr size_t kPassMin = 8;
    static constexpr size_t kPassMax = 63;
    static constexpr size_t kPskHexLen = 64;

    // `entry` is the catalog's 0x000F entry or nullptr; its presence and
    // access are copied here, the pointer is not kept.
    Provisioning(Hub& hub, const CatalogEntry* entry, IStation& station, uint16_t wsPort)
        : _hub(hub), _station(station), _wsPort(wsPort),
          _declared(entry != nullptr && entry->cls == ChannelClass::INTENT),
          _access(entry != nullptr ? entry->access : AccessLevel::configure) {}

    // A provisioning binding's inbound frame. True = this desk consumed it
    // (answered now, later, or deliberately not at all); false = hand it to
    // the hub unchanged. `sessionId` is the session the binding carries, 0
    // for none yet.
    bool offer(std::span<const std::byte> frame, ITransport& link, uint32_t sessionId, uint32_t nowMs);

    // Once per hub tick: finishes a join, writes its answer, persists.
    void tick();

    // The binding no longer carries the session it did: an answer still owed
    // to it is dropped. The join itself runs on.
    void forget(const ITransport& link) {
        if (_attempt.link == &link) _attempt.link = nullptr;
    }

    bool busy() const { return _running || _persistDue; }

private:
    struct Attempt {
        ITransport* link = nullptr;
        uint32_t sessionId = 0;
        uint16_t intentId = 0;
        uint16_t seq = 0;
        uint32_t startedMs = 0;
        std::array<char, kSsidMax + 1> ssid{};
        std::array<char, kPskHexLen + 1> pass{};
        uint8_t ssidLen = 0;
        uint8_t passLen = 0;
    };

    struct Refusal {
        NackCode code;
        const char* detail;   // a string literal: static storage
    };

    // ECHO payloads are small: {cfg_gen, intent_id, applied{5}}.
    static constexpr size_t kEchoBytes = 64;

    const HubSession* sessionById(uint32_t id) const;
    std::string_view ssid() const { return {_attempt.ssid.data(), _attempt.ssidLen}; }
    std::string_view pass() const { return {_attempt.pass.data(), _attempt.passLen}; }
    void wipe();
    void nack(ITransport& link, NackCode code, uint16_t intentId, uint16_t seq,
              std::string_view detail = {}, uint32_t retryAfterMs = 0);
    static void send(ITransport& link, FrameType type, uint16_t channel, std::span<const std::byte> payload);
    // nullopt = the fields pass.
    static std::optional<Refusal> checkFields(const IntentMsg& in);
    static const IntentValue* field(const IntentMsg& in, uint8_t key);
    static std::string_view failureDetail(JoinFailure why);

    Hub& _hub;
    IStation& _station;
    const uint16_t _wsPort;
    const bool _declared;
    const AccessLevel _access;

    Attempt _attempt{};
    bool _running = false;
    bool _persistDue = false;

    // The last success, for a duplicate intent_id (SPEC §9.3).
    uint32_t _echoSession = 0;
    uint16_t _echoIntent = 0;
    std::array<std::byte, kEchoBytes> _echo{};
    size_t _echoLen = 0;
};

// ---- board functions (ValenceProvisioning.cpp; not built natively) -----------

// The board station. provisioningBegin() starts its task and event hooks,
// after main.cpp's wifi_up(); false leaves every join answered `station down`.
IStation& provisioningStation();
bool provisioningBegin();

// The station's credentials, the one home boot (main.cpp wifi_up) and a
// failed join's restore both read: the provisioned pair in NVS (written only
// after a wifi_join joined), else the compiled-in secrets.h pair. True when
// they came from NVS. Never logs either.
bool stationCredentials(std::span<uint8_t, 32> ssid, std::span<uint8_t, 64> passphrase);

// ---- desk implementation -----------------------------------------------------------

namespace detail {
inline constexpr const char* kProvisioningTag = "prov";
}  // namespace detail

inline const HubSession* Provisioning::sessionById(uint32_t id) const {
    if (id == 0) return nullptr;
    for (size_t i = 0;; ++i) {
        const HubSession* s = _hub.sessionBySlot(i);
        if (s == nullptr) return nullptr;
        if (s->occupied() && s->session_id == id) return s;
    }
}

inline void Provisioning::wipe() {
    // volatile: plain stores to a buffer nobody reads again are dead stores
    // the optimizer may drop.
    volatile char* s = _attempt.ssid.data();
    for (size_t i = 0; i < _attempt.ssid.size(); ++i) s[i] = 0;
    volatile char* p = _attempt.pass.data();
    for (size_t i = 0; i < _attempt.pass.size(); ++i) p[i] = 0;
    _attempt.ssidLen = 0;
    _attempt.passLen = 0;
}

inline void Provisioning::send(ITransport& link, FrameType type, uint16_t channel,
                               std::span<const std::byte> payload) {
    std::array<std::byte, kHeaderBytes + 128> frame{};
    if (payload.size() > frame.size() - kHeaderBytes) return;
    FrameHeader h;
    h.type = uint8_t(type);
    h.channel = channel;
    h.len = uint16_t(payload.size());
    if (encodeFrameHeader(h, frame) != kHeaderBytes) return;
    std::memcpy(frame.data() + kHeaderBytes, payload.data(), payload.size());
    link.write(std::span<const std::byte>(frame.data(), kHeaderBytes + payload.size()));
}

// Shaped like the hub's own NACK (hub_impl.hpp sendNack): header channel 0;
// the refused intent's channel, id and frame seq in the body.
inline void Provisioning::nack(ITransport& link, NackCode code, uint16_t intentId, uint16_t seq,
                               std::string_view detail, uint32_t retryAfterMs) {
    NackMsg n;
    n.code = code;
    n.has_channel_id = true;
    n.channel_id = channels::provisioning;
    n.has_intent_id = true;
    n.intent_id = intentId;
    n.has_intent_seq = true;
    n.intent_seq = seq;
    n.has_detail = !detail.empty();
    n.detail = detail;
    n.has_retry_after_ms = retryAfterMs != 0;
    n.retry_after_ms = retryAfterMs;
    std::array<std::byte, 128> buf{};
    const size_t len = encodeNack(n, buf);
    if (len != 0) send(link, FrameType::NACK, 0, std::span<const std::byte>(buf.data(), len));
}

inline const IntentValue* Provisioning::field(const IntentMsg& in, uint8_t key) {
    for (uint32_t i = 0; i < in.value_count; ++i) {
        if (in.value[i].key == key) return &in.value[i].value;
    }
    return nullptr;
}

inline std::optional<Provisioning::Refusal> Provisioning::checkFields(const IntentMsg& in) {
    const IntentValue* op = field(in, kKeyOp);
    if (op == nullptr || op->kind != IntentValue::Kind::U64) return Refusal{NackCode::INVALID_VALUE, "op"};
    if (op->u64_val != provisioning_ops::wifi_join) return Refusal{NackCode::UNSUPPORTED_OP, "op"};

    const IntentValue* ssid = field(in, kKeySsid);
    if (ssid == nullptr || ssid->kind != IntentValue::Kind::Tstr) return Refusal{NackCode::INVALID_VALUE, "ssid"};
    if (ssid->tstr_val.empty() || ssid->tstr_val.size() > kSsidMax)
        return Refusal{NackCode::INVALID_VALUE, "ssid length"};
    // The station config is NUL-terminated below 32 octets.
    if (ssid->tstr_val.find('\0') != std::string_view::npos) return Refusal{NackCode::INVALID_VALUE, "ssid"};

    const IntentValue* pass = field(in, kKeyPassphrase);
    if (pass == nullptr || pass->kind != IntentValue::Kind::Tstr)
        return Refusal{NackCode::INVALID_VALUE, "passphrase"};
    const std::string_view p = pass->tstr_val;
    if (p.empty()) return std::nullopt;
    if (p.size() == kPskHexLen) {
        for (char c : p) {
            const bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
            if (!hex) return Refusal{NackCode::INVALID_VALUE, "passphrase"};
        }
        return std::nullopt;
    }
    if (p.size() < kPassMin || p.size() > kPassMax) return Refusal{NackCode::INVALID_VALUE, "passphrase length"};
    for (char c : p) {
        if (c < 0x20 || c > 0x7E) return Refusal{NackCode::INVALID_VALUE, "passphrase"};
    }
    return std::nullopt;
}

inline std::string_view Provisioning::failureDetail(JoinFailure why) {
    switch (why) {
        case JoinFailure::station_down: return "station down";
        case JoinFailure::auth:         return "authentication failed";
        case JoinFailure::not_found:    return "network not found";
        case JoinFailure::no_address:   return "no address";
        case JoinFailure::timeout:      break;
    }
    return "timed out";
}

inline bool Provisioning::offer(std::span<const std::byte> frame, ITransport& link, uint32_t sessionId,
                                uint32_t nowMs) {
    if (!_declared) return false;
    const std::optional<FrameHeader> h = decodeFrameHeader(frame);
    if (!h || h->type != uint8_t(FrameType::INTENT) || frame.size() != kHeaderBytes + size_t(h->len)) return false;
    // Routed by the body's channel_id, as the hub routes it; a malformed body
    // stays the hub's to refuse.
    const auto decoded = decodeIntent(frame.subspan(kHeaderBytes));
    if (!decoded || decoded.value().channel_id != channels::provisioning) return false;
    const IntentMsg& in = decoded.value();

    // The hub drops an INTENT that arrives before HELLO, unanswered; so does
    // this desk.
    const HubSession* s = sessionById(sessionId);
    if (s == nullptr) return true;
    if (!s->ready) {
        nack(link, NackCode::NOT_READY, in.intent_id, h->seq);
        return true;
    }
    if (_echoLen != 0 && _echoSession == sessionId && _echoIntent == in.intent_id) {
        send(link, FrameType::ECHO, channels::provisioning, std::span<const std::byte>(_echo.data(), _echoLen));
        return true;
    }
    // A duplicate during the attempt joins it (SPEC §13.9): one answer, later.
    if (_running && _attempt.sessionId == sessionId && _attempt.intentId == in.intent_id) return true;

    if (uint8_t(s->role) < uint8_t(_access)) {
        nack(link, _access == AccessLevel::control ? NackCode::NOT_CONTROLLER : NackCode::ACCESS_DENIED,
             in.intent_id, h->seq);
        return true;
    }
    if (in.has_precondition && in.precondition != _hub.cfgGen()) {
        nack(link, NackCode::CONFLICT, in.intent_id, h->seq);
        return true;
    }
    // SPEC §13.9: an association window open now, or a factory-fresh hub.
    if (!_hub.pairingWindowOpen() && _hub.pairing().hasConfigureToken()) {
        nack(link, NackCode::ACCESS_DENIED, in.intent_id, h->seq, "pairing window closed");
        return true;
    }
    if (const auto refusal = checkFields(in)) {
        nack(link, refusal->code, in.intent_id, h->seq, refusal->detail);
        return true;
    }
    if (busy()) {
        const uint32_t spent = nowMs - _attempt.startedMs;
        const uint32_t left = spent < limits::provision_join_timeout_ms
                                  ? limits::provision_join_timeout_ms - spent : 1000u;
        nack(link, NackCode::BUSY, in.intent_id, h->seq, "join in progress", left);
        return true;
    }

    const std::string_view ssidIn = field(in, kKeySsid)->tstr_val;
    const std::string_view passIn = field(in, kKeyPassphrase)->tstr_val;
    wipe();
    std::memcpy(_attempt.ssid.data(), ssidIn.data(), ssidIn.size());
    std::memcpy(_attempt.pass.data(), passIn.data(), passIn.size());
    _attempt.ssidLen = uint8_t(ssidIn.size());
    _attempt.passLen = uint8_t(passIn.size());
    if (!_station.beginJoin(ssid(), pass())) {
        wipe();
        nack(link, NackCode::NETWORK_JOIN_FAILED, in.intent_id, h->seq, failureDetail(JoinFailure::station_down));
        GLOGW(detail::kProvisioningTag, "wifi_join refused: the station cannot start a join");
        return true;
    }
    _attempt.link = &link;
    _attempt.sessionId = sessionId;
    _attempt.intentId = in.intent_id;
    _attempt.seq = h->seq;
    _attempt.startedMs = nowMs;
    _running = true;
    GLOGI(detail::kProvisioningTag, "wifi_join from session %08lx: joining",
          static_cast<unsigned long>(sessionId));
    return true;
}

inline void Provisioning::tick() {
    if (_persistDue) {
        if (!_station.persist(ssid(), pass())) return;
        _persistDue = false;
        wipe();
    }
    if (!_running) return;
    const std::optional<JoinOutcome> out = _station.pollJoin();
    if (!out) return;
    _running = false;

    // The answer goes only to the link and session that asked; either may be
    // gone by now, and the join's effect stands regardless.
    ITransport* link = (_attempt.link != nullptr && sessionById(_attempt.sessionId) != nullptr)
                           ? _attempt.link : nullptr;
    if (!out->joined) {
        wipe();
        const std::string_view why = failureDetail(out->why);
        if (link != nullptr) nack(*link, NackCode::NETWORK_JOIN_FAILED, _attempt.intentId, _attempt.seq, why);
        GLOGW(detail::kProvisioningTag, "wifi_join failed: %.*s; the prior network stays",
              int(why.size()), why.data());
        return;
    }

    EchoMsg echo;
    echo.intent_id = _attempt.intentId;
    echo.cfg_gen = _hub.cfgGen();
    echo.applied_count = 5;
    echo.applied[0] = {kKeyOp, IntentValue::ofU64(provisioning_ops::wifi_join)};
    echo.applied[1] = {kKeySsid, IntentValue::ofBool(true)};
    echo.applied[2] = {kKeyPassphrase, IntentValue::ofBool(true)};
    echo.applied[3] = {kKeyIpv4, IntentValue::ofU64(out->ipv4)};
    echo.applied[4] = {kKeyWsPort, IntentValue::ofU64(_wsPort)};
    _echoLen = encodeEcho(echo, _echo);
    _echoSession = _attempt.sessionId;
    _echoIntent = _attempt.intentId;
    if (link != nullptr && _echoLen != 0)
        send(*link, FrameType::ECHO, channels::provisioning, std::span<const std::byte>(_echo.data(), _echoLen));
    GLOGI(detail::kProvisioningTag, "wifi_join joined: %u.%u.%u.%u, ws port %u",
          unsigned(out->ipv4 >> 24), unsigned((out->ipv4 >> 16) & 0xFF), unsigned((out->ipv4 >> 8) & 0xFF),
          unsigned(out->ipv4 & 0xFF), unsigned(_wsPort));

    _persistDue = true;
    if (_station.persist(ssid(), pass())) {
        _persistDue = false;
        wipe();
    }
}

}  // namespace valence
