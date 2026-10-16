// NeutrinoWsPort -- implementation. See NeutrinoWsPort.h; nothing is restated here.

#include "NeutrinoWsPort.h"

namespace valence {

bool NeutrinoWsPort::begin(Hub* hub, uint16_t port, bench::SessionLog* log, const std::string& bindHost) {
    _hub = hub;
    _log = log;
    for (auto& s : _slots) s.setNowMsSource(&_nowMs);
    _server = std::make_unique<ix::WebSocketServer>(port, bindHost);
    _server->setOnClientMessageCallback(
        [this](std::shared_ptr<ix::ConnectionState> state, ix::WebSocket& ws, const ix::WebSocketMessagePtr& msg) {
            onMessage(state, ws, msg);
        });
    const auto res = _server->listen();
    if (!res.first) {
        if (_log) _log->logf('E', "ws: listen failed on %s:%u -- %s", bindHost.c_str(), unsigned(port), res.second.c_str());
        return false;
    }
    _server->start();
    if (_log) _log->logf('I', "ws: listening on %s:%u (valence.v1)", bindHost.c_str(), unsigned(port));
    return true;
}

void NeutrinoWsPort::stop() {
    if (_server) _server->stop();
}

// ---- connection threads -------------------------------------------------------

void NeutrinoWsPort::onMessage(std::shared_ptr<ix::ConnectionState> state, ix::WebSocket& ws,
                               const ix::WebSocketMessagePtr& msg) {
    const std::string id = state->getId();
    switch (msg->type) {
        case ix::WebSocketMessageType::Open: {
            std::lock_guard<std::mutex> lk(_mapM);
            uint8_t slot = 0xFF;
            for (uint8_t i = 0; i < kSlots; ++i) {
                if (!_slots[i].inUse()) {
                    slot = i;
                    break;
                }
            }
            if (slot == 0xFF) {
                if (_log) _log->logf('W', "ws: refusing connection %s -- all %u slots busy", id.c_str(), unsigned(kSlots));
                ws.close(1013, "try again later");
                return;
            }
            _slotById[id] = slot;
            _slots[slot].onOpen(&ws, state->getRemoteIp());
            break;
        }
        case ix::WebSocketMessageType::Close: {
            std::lock_guard<std::mutex> lk(_mapM);
            const auto it = _slotById.find(id);
            if (it != _slotById.end()) {
                _slots[it->second].onClose();
                _slotById.erase(it);
            }
            break;
        }
        case ix::WebSocketMessageType::Message: {
            if (!msg->binary) return;   // TEXT is not valence (firmware parity)
            uint8_t slot = 0;
            {
                std::lock_guard<std::mutex> lk(_mapM);
                const auto it = _slotById.find(id);
                if (it == _slotById.end()) return;
                slot = it->second;
            }
            _slots[slot].pushRx(msg->str.data(), msg->str.size());
            break;
        }
        case ix::WebSocketMessageType::Ping:
        case ix::WebSocketMessageType::Pong: {
            std::lock_guard<std::mutex> lk(_mapM);
            const auto it = _slotById.find(id);
            if (it != _slotById.end()) _slots[it->second].noteRx();
            break;
        }
        default:
            break;
    }
}

// ---- hub thread ---------------------------------------------------------------

void NeutrinoWsPort::loop(uint32_t nowMs) {
    _nowMs = nowMs;
    for (uint8_t i = 0; i < kSlots; ++i) {
        auto& s = _slots[i];
        if (s.consumeOpenEvent()) {
            _links[i].bind(&s);
            applyFaults(i);
            if (_hub->attachTransport(_links[i])) {
                _attached[i] = true;
                if (_log) _log->logf('I', "ws: client %s -> slot %u attached", s.peer().c_str(), unsigned(i));
            } else {
                if (_log) _log->logf('W', "ws: hub refused transport (full) -- closing slot %u", unsigned(i));
                s.requestClose();
            }
        }
        if (s.consumeCloseEvent()) {
            if (_attached[i]) {
                _hub->detachTransport(_links[i]);
                _attached[i] = false;
            }
            _links[i].bind(nullptr);
            _own[i] = {};
            if (_log) _log->logf('I', "ws: slot %u detached", unsigned(i));
            s.reap();
        }
        if (!_attached[i]) continue;
        if (s.idleOut(nowMs) && s.requestClose() && _log)
            _log->logf('W', "ws: slot %u silent >%ums -- reaping", unsigned(i),
                       unsigned(bench::ValenceBenchWsTransport::kIdleReapMs));
        if (s.stalledOut(nowMs) && s.requestClose() && _log)
            _log->logf('W', "ws: slot %u stalled >%ums -- disconnecting", unsigned(i),
                       unsigned(bench::ValenceBenchWsTransport::kStallEvictMs));
    }
}

void NeutrinoWsPort::tick(uint64_t nowUs) {
    for (uint8_t i = 0; i < kSlots; ++i) {
        if (_attached[i]) _links[i].tick(nowUs);
    }
}

void NeutrinoWsPort::setGlobalFaults(const LinkFaults& f) {
    _global = f;
    for (uint8_t i = 0; i < kSlots; ++i) applyFaults(i);
}

void NeutrinoWsPort::setClientFaults(uint8_t slot, const LinkFaults& f) {
    if (slot >= kSlots) return;
    _own[slot] = f;
    applyFaults(slot);
}

void NeutrinoWsPort::setHeld(bool held) {
    for (auto& l : _links) l.setHeld(held);
}

void NeutrinoWsPort::kick(uint8_t slot) {
    if (slot < kSlots && _attached[slot]) _slots[slot].requestClose();
}

NeutrinoWsPort::SlotInfo NeutrinoWsPort::info(uint8_t slot) const {
    SlotInfo out;
    if (slot >= kSlots || !_attached[slot]) return out;
    const auto& s = _slots[slot];
    out.inUse = true;
    out.peer = s.peer();
    out.muted = s.muted();
    out.rxDrops = s.rxDrops();
    out.link = _links[slot].stats();
    out.faults = _own[slot];
    return out;
}

}  // namespace valence
