// ValenceWsPort -- implementation. See ValenceWsPort.h for the threading,
// send-hazard, congestion and class rules; nothing is restated here.

#include "ValenceWsPort.h"

#include <cstring>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include <unistd.h>

#include "geiger/geiger.h"

namespace valence {

namespace {
constexpr const char* kTag = "ws";
// There is exactly one port. The httpd callbacks are C function pointers with
// no user context of their own on open_fn/close_fn, so the instance is reached
// through this.
ValenceWsPort* g_port = nullptr;
}  // namespace

// ---- ValenceWsTransport ------------------------------------------------------

void ValenceWsTransport::bind(httpd_handle_t hd, int fd) {
    _hd = hd;
    _closing.store(false, std::memory_order_seq_cst);
    _sending.store(false, std::memory_order_seq_cst);
    _rxHead.store(0, std::memory_order_relaxed);
    _rxTail.store(0, std::memory_order_relaxed);
    _rxDrops.store(0, std::memory_order_relaxed);
    _framesRx.store(0, std::memory_order_relaxed);
    _framesTx = _txDataDrops = _txBlobHolds = _txCtrlFails = 0;
    _ctrlStallSinceMs = 0;
    _blobThisTick = 0;
    _congestionLevel = 0;
    _aboveSinceMs = _belowSinceMs = 0;
    _lastSlowMs = 0;
    _lastRxMs.store(uint32_t(esp_timer_get_time() / 1000), std::memory_order_relaxed);
    _fd.store(fd, std::memory_order_seq_cst);
}

void ValenceWsTransport::noteRx() {
    _lastRxMs.store(uint32_t(esp_timer_get_time() / 1000), std::memory_order_relaxed);
}

void ValenceWsTransport::pushRx(const uint8_t* data, size_t len) {
    if (len == 0 || len > valence::kFrameBufferCapacity) return;  // never a valid frame
    const uint8_t tail = _rxTail.load(std::memory_order_relaxed);
    const uint8_t next = uint8_t((tail + 1) % kRxRingDepth);
    if (next == _rxHead.load(std::memory_order_acquire)) {
        _rxDrops.fetch_add(1, std::memory_order_relaxed);  // full: drop the NEW frame
        return;
    }
    std::memcpy(_rx[tail].writable().data(), data, len);
    _rx[tail].setSize(len);
    // RELEASE: the bytes above must be visible before the index that publishes
    // them. Without this pairing the consumer can parse a half-written buffer.
    _rxTail.store(next, std::memory_order_release);
    _framesRx.fetch_add(1, std::memory_order_relaxed);
}

void ValenceWsTransport::beginClose() {
    _closing.store(true, std::memory_order_seq_cst);
    // Bounded by SO_SNDTIMEO: an in-flight send owns the fd until it returns.
    while (_sending.load(std::memory_order_seq_cst)) vTaskDelay(1);
    _fd.store(-1, std::memory_order_seq_cst);
}

bool ValenceWsTransport::open() { return _fd.load(std::memory_order_seq_cst) >= 0; }

void ValenceWsTransport::close() {
    const int fd = _fd.load(std::memory_order_seq_cst);
    if (fd >= 0 && _hd != nullptr) httpd_sess_trigger_close(_hd, fd);
}

bool ValenceWsTransport::isDroppable(std::span<const std::byte> frame) {
    if (frame.empty()) return false;
    const auto t = uint8_t(frame[0]);  // byte 0 is the frame type (wire/frame_header.hpp)
    return t == uint8_t(valence::FrameType::STATE) || t == uint8_t(valence::FrameType::STREAM);
}

bool ValenceWsTransport::isBlobChunk(std::span<const std::byte> frame) {
    return !frame.empty() && uint8_t(frame[0]) == uint8_t(valence::FrameType::BLOB_CHUNK);
}

bool ValenceWsTransport::write(std::span<const std::byte> frame) {
    if (frame.empty()) return false;

    const bool data = isDroppable(frame);
    const bool blob = isBlobChunk(frame);

    // Class gates BEFORE the syscall, so a congested link is never charged
    // another blocking send for a frame it is allowed to shed.
    if (data && _congestionLevel != 0) {
        ++_txDataDrops;
        return false;
    }
    if (blob) {
        if (_blobThisTick >= valence::limits::blob_chunks_in_flight || _congestionLevel != 0) {
            ++_txBlobHolds;   // held, not lost: the hub retries this index next tick
            return false;
        }
    }

    if (_closing.load(std::memory_order_seq_cst)) return false;
    _sending.store(true, std::memory_order_seq_cst);
    if (_closing.load(std::memory_order_seq_cst)) {
        _sending.store(false, std::memory_order_seq_cst);
        return false;
    }
    const int fd = _fd.load(std::memory_order_seq_cst);
    esp_err_t err = ESP_FAIL;
    int64_t elapsedUs = 0;
    if (fd >= 0 && _hd != nullptr) {
        httpd_ws_frame_t f{};
        f.final = true;
        f.type = HTTPD_WS_TYPE_BINARY;
        f.payload = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(frame.data()));
        f.len = frame.size();
        const int64_t t0 = esp_timer_get_time();
        err = httpd_ws_send_frame_async(_hd, fd, &f);
        elapsedUs = esp_timer_get_time() - t0;
    }
    _sending.store(false, std::memory_order_seq_cst);

    const uint32_t nowMs = uint32_t(esp_timer_get_time() / 1000);
    if (err == ESP_OK) {
        ++_framesTx;
        if (elapsedUs > int64_t(kSlowSendUs)) _lastSlowMs = nowMs;
        if (!data && !blob) _ctrlStallSinceMs = 0;  // cleared ONLY by control going out
        if (blob) ++_blobThisTick;
        return true;
    }

    _lastSlowMs = nowMs;
    if (data) {
        ++_txDataDrops;
    } else if (blob) {
        ++_txBlobHolds;   // never arms the stall timer (T16)
    } else {
        ++_txCtrlFails;
        if (_ctrlStallSinceMs == 0) _ctrlStallSinceMs = uint32_t(esp_timer_get_time() / 1000);
    }
    return false;
}

std::optional<valence::FrameBuffer> ValenceWsTransport::read() {
    const uint8_t head = _rxHead.load(std::memory_order_relaxed);
    // ACQUIRE: pairs with pushRx's release store.
    if (head == _rxTail.load(std::memory_order_acquire)) return std::nullopt;
    valence::FrameBuffer out = _rx[head];
    _rxHead.store(uint8_t((head + 1) % kRxRingDepth), std::memory_order_release);
    return out;
}

valence::TransportProperties ValenceWsTransport::properties() const {
    valence::TransportProperties p;
    p.mtu = uint16_t(valence::kFrameBufferCapacity);
    p.ordered = true;    // TCP
    p.reliable = true;
    p.congestion = valence::CongestionSignal::QueueWatermark;
    return p;
}

bool ValenceWsTransport::stalledOut(uint32_t nowMs) const {
    return _ctrlStallSinceMs != 0 && uint32_t(nowMs - _ctrlStallSinceMs) > kCtrlStallMs;
}

bool ValenceWsTransport::idleOut(uint32_t nowMs) const {
    return uint32_t(nowMs - _lastRxMs.load(std::memory_order_relaxed)) > kIdleReapMs;
}

uint8_t ValenceWsTransport::pollCongestionLevel(uint32_t nowMs) {
    // severe: a never-shed frame is stalled right now. §10.4's own definition.
    if (_ctrlStallSinceMs != 0) {
        _congestionLevel = 2;
        _aboveSinceMs = _belowSinceMs = 0;
        return _congestionLevel;
    }
    // "Above the watermark" is a WINDOW, not one tick: a slow send inside the
    // last kSlowWindowMs. Sampling per tick would demand 200 consecutive slow
    // ticks to reach the §10.3 one-second threshold, which no real backup does.
    const bool above = _lastSlowMs != 0 && uint32_t(nowMs - _lastSlowMs) < kSlowWindowMs;
    if (above) {
        _belowSinceMs = 0;
        if (_aboveSinceMs == 0) _aboveSinceMs = nowMs;
        if (uint32_t(nowMs - _aboveSinceMs) >= kCongestedSustainMs) _congestionLevel = 1;
    } else {
        _aboveSinceMs = 0;
        if (_belowSinceMs == 0) _belowSinceMs = nowMs;
        // Recovery re-probes by construction: at level 1 the data classes shed
        // without touching the socket, so nothing refreshes _lastSlowMs and the
        // level falls back to 0 after the §10.3 window. Control frames keep
        // probing the socket throughout, so a genuinely wedged peer re-arms it.
        if (uint32_t(nowMs - _belowSinceMs) >= kRecoveredSustainMs) _congestionLevel = 0;
    }
    return _congestionLevel;
}

// ---- ValenceWsPort -----------------------------------------------------------

int ValenceWsPort::slotForFd(int fd) const {
    for (uint8_t i = 0; i < kSlots; ++i) {
        if (_inUse[i].load(std::memory_order_acquire) && _slots[i].fd() == fd) return int(i);
    }
    return -1;
}

// Per-socket setup. TCP_NODELAY is not optional on a control link: without it
// Nagle holds a small frame until the peer's delayed-ACK timer fires, measured
// at 40 ms on the C5. SO_SNDTIMEO is what keeps one wedged peer from owning
// the hub tick forever -- see the header.
esp_err_t ValenceWsPort::sockOpen(httpd_handle_t hd, int sockfd) {
    int one = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    struct timeval snd;
    snd.tv_sec = 0;
    snd.tv_usec = ValenceWsTransport::kSendTimeoutMs * 1000;
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd));
    (void)hd;
    return ESP_OK;
}

// THE ONE TEARDOWN FUNNEL (T3). esp_http_server does not notify a WS handler on
// close, so this is the only hook that exists; without it a slot is claimed
// forever and the fifth disconnect wedges the hub. close_fn owns the fd once
// registered, so the close(2) is ours to make.
void ValenceWsPort::sockClose(httpd_handle_t hd, int sockfd) {
    if (g_port != nullptr) {
        for (uint8_t i = 0; i < kSlots; ++i) {
            if (!g_port->_inUse[i].load(std::memory_order_acquire)) continue;
            if (g_port->_slots[i].fd() != sockfd) continue;
            g_port->_slots[i].beginClose();   // waits out any in-flight send
            g_port->_wantDetach[i].store(true, std::memory_order_release);
            break;
        }
    }
    ::close(sockfd);
    (void)hd;
}

esp_err_t ValenceWsPort::wsHandler(httpd_req_t* req) {
    if (g_port == nullptr) return ESP_FAIL;
    const int fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET) {
        // Handshake. esp_http_server has already negotiated and echoed
        // "valence.v1" from the URI registration by the time this runs.
        for (uint8_t i = 0; i < kSlots; ++i) {
            bool expect = false;
            if (g_port->_inUse[i].compare_exchange_strong(expect, true,
                                                          std::memory_order_acq_rel)) {
                g_port->_slots[i].bind(req->handle, fd);
                g_port->_wantAttach[i].store(true, std::memory_order_release);
                GLOGI(kTag, "fd %d -> slot %u", fd, unsigned(i));
                return ESP_OK;
            }
        }
        GLOGW(kTag, "all %u slots busy -- refusing fd %d", unsigned(kSlots), fd);
        httpd_sess_trigger_close(req->handle, fd);
        return ESP_FAIL;
    }

    // A FRAME WE CANNOT PARSE MUST NOT KILL THE SESSION, and a frame whose
    // bytes are not READ desyncs the stream for everything after it. Both
    // lessons are the C5's, paid for with a 30 s reconnect cycle on every .NET
    // client (an unsolicited PONG, RFC 6455 §5.5.3).
    httpd_ws_frame_t f{};
    // ESP_FAIL, not ESP_OK: the return value is what tells esp_http_server
    // whether to KEEP the session. ESP_OK on a dead peer re-invokes the handler
    // immediately, spins the httpd task, and never reaches close_fn -- so the
    // slot is never freed.
    if (httpd_ws_recv_frame(req, &f, 0) != ESP_OK) return ESP_FAIL;

    uint8_t buf[valence::kFrameBufferCapacity];
    if (f.len > sizeof(buf)) return ESP_FAIL;   // cannot consume it; the stream is desynced
    if (f.len > 0) {
        f.payload = buf;
        if (httpd_ws_recv_frame(req, &f, f.len) != ESP_OK) return ESP_FAIL;
    }

    // Slot lookup BEFORE the switch: a control frame is proof of life too, and
    // for an idle client a PING is the only thing that ever arrives.
    const int slot = g_port->slotForFd(fd);
    if (slot >= 0) g_port->_slots[slot].noteRx();

    switch (f.type) {
        case HTTPD_WS_TYPE_PONG:
            return ESP_OK;   // §5.5.3: unidirectional heartbeat, no response expected
        case HTTPD_WS_TYPE_PING: {
            httpd_ws_frame_t pong{};   // §5.5.2: a Pong carries the Ping's data verbatim
            pong.final = true;
            pong.type = HTTPD_WS_TYPE_PONG;
            pong.payload = buf;
            pong.len = f.len;
            httpd_ws_send_frame(req, &pong);
            return ESP_OK;
        }
        case HTTPD_WS_TYPE_CLOSE: {
            httpd_ws_frame_t bye{};   // §5.5.1: echo it, then close the socket
            bye.final = true;
            bye.type = HTTPD_WS_TYPE_CLOSE;
            bye.len = 0;
            httpd_ws_send_frame(req, &bye);
            // ESP_FAIL IS THE TEARDOWN. ESP_OK here leaks the fd, the port slot
            // and the hub session: esp_http_server has already latched
            // sd->ws_close, and from the next select it returns early on this
            // socket without ever deleting the session ("WS was marked close",
            // httpd_parse.c:790). close_fn is then unreachable and the peer --
            // which by RFC 6455 §5.5.1 is waiting for the SERVER to close the
            // TCP connection -- waits forever. val-091.15.
            return ESP_FAIL;
        }
        case HTTPD_WS_TYPE_BINARY:
            break;
        default:
            // TEXT is not this protocol. §13.2: one frame, one BINARY message.
            return ESP_FAIL;
    }

    if (slot < 0 || f.len == 0) return ESP_OK;
    g_port->_slots[slot].pushRx(buf, f.len);
    return ESP_OK;
}

bool ValenceWsPort::begin(valence::Hub* hub, uint16_t port) {
    _hub = hub;
    g_port = this;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = port;
    cfg.ctrl_port = 32769;          // must not collide with the :80 instance
    // kSlots + 2 = 7 client sockets. This instance ALSO costs three fds of its
    // own (listener, ctrl_fd, msg_fd -- httpd_main.c:353/400/407). The whole
    // lwIP socket budget and its arithmetic live in ONE home (C-1):
    // flagship_p4/sdkconfig.defaults, CONFIG_LWIP_MAX_SOCKETS. Raising this
    // number without raising that one is the val-091.15 lockout.
    cfg.max_open_sockets = kSlots + 2;
    cfg.max_uri_handlers = 4;
    cfg.lru_purge_enable = true;
    cfg.open_fn = sockOpen;
    cfg.close_fn = sockClose;
    // The whole stack of this task is the WS receive path plus our SPSC push.
    cfg.stack_size = 6144;

    esp_err_t err = httpd_start(&_srv, &cfg);
    if (err != ESP_OK) {
        GLOGE(kTag, "httpd_start on :%u failed: %d", unsigned(port), int(err));
        return false;
    }

    // BOTH paths. Phosphor opens ws://<host>:82/ (clients/js/session.js:1133)
    // while the SPEC recommends /valence; serving one and not the other is a
    // silent refusal at connect time.
    httpd_uri_t root{"/", HTTP_GET, wsHandler, nullptr, true, true, "valence.v1"};
    httpd_uri_t named{"/valence", HTTP_GET, wsHandler, nullptr, true, true, "valence.v1"};
    if (httpd_register_uri_handler(_srv, &root) != ESP_OK ||
        httpd_register_uri_handler(_srv, &named) != ESP_OK) {
        GLOGE(kTag, "WS URI registration failed");
        return false;
    }
    GLOGI(kTag, "listening on :%u at / and /valence (subprotocol valence.v1), %u slots",
          unsigned(port), unsigned(kSlots));
    return true;
}

void ValenceWsPort::loop(uint32_t nowMs) {
    for (uint8_t i = 0; i < kSlots; ++i) {
        auto& s = _slots[i];

        // ATTACH AND DETACH HAPPEN HERE AND NOWHERE ELSE (T5). The httpd task
        // only records intent; calling into the hub from there would race
        // Hub::update()'s slot walk, which null-checks slot.transport once and
        // dereferences it deeper in the same pass.
        if (_wantAttach[i].exchange(false, std::memory_order_acq_rel)) {
            if (_hub->attachTransport(s)) {
                _attached[i] = true;
            } else {
                GLOGW(kTag, "hub has no session slot for ws slot %u -- closing", unsigned(i));
                s.close();
            }
        }

        if (_wantDetach[i].exchange(false, std::memory_order_acq_rel)) {
            if (_attached[i]) {
                _hub->detachTransport(s);
                _attached[i] = false;
            }
            _inUse[i].store(false, std::memory_order_release);
            GLOGI(kTag, "slot %u released", unsigned(i));
            continue;
        }

        if (!_attached[i]) continue;

        s.newTick();
        _hub->setCongestionLevel(s, s.pollCongestionLevel(nowMs));

        // T19 addendum: a peer that went dark without a FIN never wakes httpd
        // on its socket, so nothing else in this system will ever free it.
        // Closing here runs the ONE funnel -- close_fn detaches and the hub
        // parks the session (RFC-042), which is what a reconnect reattaches to.
        if (s.idleOut(nowMs)) {
            GLOGW(kTag, "slot %u silent >%lums -- reaping", unsigned(i),
                  static_cast<unsigned long>(ValenceWsTransport::kIdleReapMs));
            s.noteRx();   // re-arm: close_fn lands a tick or two later, and a
                          // trigger_close per 5 ms tick would flood the ctrl socket
            s.close();
            continue;
        }

        // Continuous control failure for kCtrlStallMs is the one case where a
        // client really is stranded waiting on a reply that will never come.
        if (s.stalledOut(nowMs)) {
            GLOGW(kTag, "slot %u control stalled >%lums -- disconnecting", unsigned(i),
                  static_cast<unsigned long>(ValenceWsTransport::kCtrlStallMs));
            s.close();
        }
    }
}

size_t ValenceWsPort::openSockets() const {
    if (_srv == nullptr) return 0;
    int fds[kSlots + 2]{};          // exactly cfg.max_open_sockets
    size_t n = sizeof(fds) / sizeof(fds[0]);
    if (httpd_get_client_list(_srv, &n, fds) != ESP_OK) return 0;
    return n;
}

uint32_t ValenceWsPort::framesRx() const {
    uint32_t n = 0;
    for (uint8_t i = 0; i < kSlots; ++i) n += _slots[i].framesRx();
    return n;
}

uint32_t ValenceWsPort::drops() const {
    uint32_t n = 0;
    for (uint8_t i = 0; i < kSlots; ++i) {
        n += _slots[i].rxDrops() + _slots[i].txDataDrops() + _slots[i].txCtrlFails();
    }
    return n;
}

}  // namespace valence
