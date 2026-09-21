#pragma once

// ValenceWsPort / ValenceWsTransport -- the Valence WS binding on
// esp_http_server (SPEC §13.1, §13.2)
// Constraints:
// - ONE VALENCE FRAME IS ONE BINARY WEBSOCKET MESSAGE. A TEXT frame is not
//   this protocol and closes the session.
// - The subprotocol "valence.v1" (registry.yaml:1173) MUST be echoed in the
//   upgrade response or conforming clients refuse the socket (SPEC §13.2).
//   esp_http_server does that from httpd_uri_t::supported_subprotocol, which
//   only exists when CONFIG_HTTPD_WS_SUPPORT=y.
//
// ---- Threading, and it is not negotiable (T5) -------------------------------
//   httpd task : the URI handler, open_fn and close_fn. Produces into the RX
//                ring and RECORDS attach/detach intent. It NEVER calls the hub.
//   hub task   : open/close/write/read, and the actual attach/detach.
// RX is a lock-free SPSC ring: the httpd task alone advances _rxTail with a
// RELEASE store after filling the slot; the hub task alone advances _rxHead
// after an ACQUIRE load of the tail. That pairing is what makes the frame's
// BYTES visible to the reader -- without it the index can land before the data
// and the reader parses a half-written buffer.
//
// ---- The send hazard esp_http_server hands you ------------------------------
// httpd_ws_send_frame_async(hd, fd, ...) takes a RAW fd, and close_fn on the
// httpd task closes that fd. A send in flight would then write into a
// recycled descriptor. The _sending/_closing pair below is a two-flag
// handshake (seq_cst, both directions checked) so close_fn WAITS for an
// in-flight send instead of pulling the fd out from under it; the wait is
// bounded by SO_SNDTIMEO. This transport holds no client object of any kind,
// only the fd, and that is deliberate (T29).
//
// ---- Congestion, from what esp_http_server actually exposes ----------------
// There is NO egress queue and NO queue-depth API here: httpd_ws_send_frame_async
// is a blocking socket write straight into the kernel send buffer. So the
// watermark IS the syscall. Every send is timed; SO_SNDTIMEO bounds it at
// kSendTimeoutMs. A send that returns promptly means the buffer took it; one
// that takes longer than kSlowSendUs means the buffer was full and the kernel
// parked us, which is exactly the "egress queue above the watermark" signal
// CongestionSignal::QueueWatermark names; a send that fails means the peer is
// not draining at all. pollCongestionLevel() bands those three with SPEC
// §10.3's own hysteresis (sustained 1 s up, 5 s down) and feeds
// Hub::setCongestionLevel.
// The cost that buys it: ONE stalled send costs up to kSendTimeoutMs on the
// hub task. It is paid once per stall onset, not per frame -- above level 0
// the data classes shed WITHOUT attempting a send, so a wedged peer cannot
// charge the tick repeatedly.
//
// ---- Three TX classes, same split the S3 runs -------------------------------
//   STATE / STREAM  : data. Shed as soon as congestion is nonzero. Stale
//                     telemetry is worthless and conflation is already the
//                     doctrine.
//   BLOB_CHUNK      : its own class, paced against the registry's OWN
//                     advertised sender budget (limits::blob_chunks_in_flight)
//                     per hub tick. HELD, never dropped -- the hub re-sends the
//                     same index next tick -- and it never arms the stall timer
//                     (T16: a big honest transfer is not a wedged client).
//   everything else : control, EVENT included (a safety edge is the one thing
//                     nobody may shed). Never shed on our own initiative; a
//                     refusal starts the stall timer and only kCtrlStallMs of
//                     CONTINUOUS failure tears the session down. The timer is
//                     cleared ONLY by a control frame actually going out, never
//                     by inbound traffic.
//
// ---- Every way a client leaves runs ONE funnel (T3) -------------------------
// close_fn is the funnel: it waits out an in-flight send, records the detach
// and closes(2) the fd. THREE ways in exist and all three must reach it.
//   TCP FIN/RST, recv error : httpd deletes the session itself -> close_fn.
//   WS CLOSE frame          : does NOT reach it on its own. esp_http_server
//     latches sd->ws_close and then short-circuits that socket forever
//     ("WS was marked close", httpd_parse.c:790) without deleting the session,
//     so the fd, the port slot and the hub session are all held until the peer
//     drops TCP. A peer that waits for the server to close (RFC 6455 §5.5.1
//     makes that the server's duty) never does. The handler therefore echoes
//     the close and returns ESP_FAIL, which is what deletes the session.
//   SILENCE (T19 addendum)  : a locked phone or killed tab sends no FIN, so
//     httpd never wakes on that socket at all. kIdleReapMs closes it. Without
//     this reap the lwIP socket table fills with ghosts and BOTH listeners
//     start refusing with RST while the live clients keep working.
// A PARKED (RFC-042 STALE) session holds NO fd: the fd is closed on the way
// into the park, and only the hub-side slot is retained for reattach.
//
// ---- Slot exhaustion ---------------------------------------------------------
// Refuse AND CLOSE at the handshake. SPEC §6.3's NACK BUSY is the right answer
// on an ATTACHED transport, and the handshake predates HELLO by definition --
// there is no session to NACK into. Returning ESP_FAIL alone leaves the peer
// connected-but-silent, which is the failure users report as a bug, so the
// socket is closed explicitly.
// See: Valence SPEC.md §6.3, §9, §10.3, §13.1, §13.2

#include <atomic>
#include <cstdint>
#include <optional>
#include <span>

#include <esp_http_server.h>

#include "valence/hub/hub.hpp"
#include "valence/transport/transport.hpp"
#include "valence/wire/frame_buffer.hpp"

namespace valence {

class ValenceWsTransport final : public valence::ITransport {
public:
    // 32 inbound frames per slot. The S3 measured 2 drops at depth 8 over an
    // 18-minute 3-client soak; depth is burst tolerance, never a substitute
    // for draining. 5 slots x 32 x 512 B = 82 KB, and the whole port lives in
    // PSRAM with the hub, so it costs the internal heap nothing.
    static constexpr uint8_t kRxRingDepth = 32;

    // How long a CONTROL frame may stay unsendable before the session is torn
    // down. A single refusal is ordinary flow control; two seconds of
    // continuous refusal is a client waiting on a reply that will never come.
    static constexpr uint32_t kCtrlStallMs = 2000;

    // SPEC §10.3's own hysteresis windows, not local tuning.
    static constexpr uint32_t kCongestedSustainMs = 1000;
    static constexpr uint32_t kRecoveredSustainMs = 5000;

    // A send slower than this means the kernel send buffer was full and parked
    // us. Two milliseconds is well above a healthy LAN write and well below
    // the 5 ms tick, so one slow send is visible without a fast one tripping it.
    static constexpr uint32_t kSlowSendUs = 2000;

    // How long one slow send keeps the link "above the watermark". Sampling
    // per 5 ms tick instead would demand 200 consecutive slow ticks to reach
    // §10.3's one-second threshold, which no real backup produces.
    static constexpr uint32_t kSlowWindowMs = 200;

    // IDLE-RX REAP (T19 addendum). A conforming client PINGs every
    // limits::ping_interval_idle_ms, so ten intervals of total inbound silence
    // is ten missed proof-of-life pings. DERIVED from the library's own
    // liveness constant on purpose: a second hand-picked clock answering the
    // same question is the defect class, not the fix. Strictly more lenient
    // than the hub's own 3-interval silence reap, which only marks a session
    // STALE -- this one is the half that releases the SOCKET.
    static constexpr uint32_t kIdleReapMs = 10 * valence::limits::ping_interval_idle_ms;

    // Socket send bound. Mirrors the C5 bridge's value: ~6 frames at the
    // observed relay rate -- long enough that a healthy client on a busy
    // channel is never cut short, short enough that a dead one cannot own the
    // hub tick.
    static constexpr uint32_t kSendTimeoutMs = 50;

    // ---- httpd task ---------------------------------------------------------
    void bind(httpd_handle_t hd, int fd);
    void pushRx(const uint8_t* data, size_t len);
    // Proof of life. ANY inbound frame, control frames included -- a PING is
    // the only thing an idle-but-healthy client sends.
    void noteRx();
    // The ONE teardown funnel (T3). Waits out any in-flight send, then releases
    // the fd to the caller to close.
    void beginClose();

    // ---- hub task -----------------------------------------------------------
    bool open() override;
    void close() override;
    bool write(std::span<const std::byte> frame) override;
    std::optional<valence::FrameBuffer> read() override;
    valence::TransportProperties properties() const override;

    uint8_t pollCongestionLevel(uint32_t nowMs);
    // Called once per hub tick: re-arms the BLOB_CHUNK pacing budget.
    void newTick() { _blobThisTick = 0; }
    // True once the control-stall timer has run past kCtrlStallMs.
    bool stalledOut(uint32_t nowMs) const;
    // True once nothing at all has arrived for kIdleReapMs.
    bool idleOut(uint32_t nowMs) const;

    // ---- diagnostics (either task, relaxed) ---------------------------------
    int fd() const { return _fd.load(std::memory_order_relaxed); }
    uint32_t framesTx() const { return _framesTx; }
    uint32_t framesRx() const { return _framesRx.load(std::memory_order_relaxed); }
    uint32_t rxDrops() const { return _rxDrops.load(std::memory_order_relaxed); }
    uint32_t txDataDrops() const { return _txDataDrops; }
    uint32_t txBlobHolds() const { return _txBlobHolds; }
    uint32_t txCtrlFails() const { return _txCtrlFails; }

private:
    static bool isDroppable(std::span<const std::byte> frame);
    static bool isBlobChunk(std::span<const std::byte> frame);

    httpd_handle_t _hd = nullptr;
    std::atomic<int> _fd{-1};
    std::atomic<bool> _closing{false};
    std::atomic<bool> _sending{false};

    valence::FrameBuffer _rx[kRxRingDepth]{};
    std::atomic<uint8_t> _rxHead{0};
    std::atomic<uint8_t> _rxTail{0};
    std::atomic<uint32_t> _rxDrops{0};
    std::atomic<uint32_t> _framesRx{0};
    std::atomic<uint32_t> _lastRxMs{0};   // httpd task stamps, hub task reads

    // Hub task only -- no atomics wanted.
    uint32_t _framesTx = 0;
    uint32_t _txDataDrops = 0;
    uint32_t _txBlobHolds = 0;
    uint32_t _txCtrlFails = 0;
    uint32_t _ctrlStallSinceMs = 0;
    uint8_t _blobThisTick = 0;
    uint8_t _congestionLevel = 0;
    uint32_t _aboveSinceMs = 0;
    uint32_t _belowSinceMs = 0;
    uint32_t _lastSlowMs = 0;   // ms of the last slow-or-failed send, 0 = never
};

// The server plus its transport slots. Owns TWO httpd instances: the Valence
// socket on 82 and the /uitoken surface on 80 (ValenceUiToken registers into
// the one this class starts), each with its own ctrl_port, because two
// instances sharing one control port silently refuse to start.
class ValenceWsPort {
public:
    // One more slot than the hub has sessions, so a connection that arrives
    // during a teardown has somewhere to land instead of being refused by a
    // slot the hub has not released yet.
    static constexpr uint8_t kSlots = valence::kHubMaxSessions + 1;

    bool begin(valence::Hub* hub, uint16_t port);
    // Hub task: performs the deferred attach/detach and sweeps stalls.
    void loop(uint32_t nowMs);

    httpd_handle_t handle() const { return _srv; }
    // Client sockets this instance currently holds. Cheap: a scan of the
    // socket table, no lock and no allocation (httpd_main.c:173).
    size_t openSockets() const;
    uint32_t framesRx() const;
    uint32_t drops() const;

private:
    static esp_err_t wsHandler(httpd_req_t* req);
    static esp_err_t sockOpen(httpd_handle_t hd, int sockfd);
    static void sockClose(httpd_handle_t hd, int sockfd);

    int slotForFd(int fd) const;

    httpd_handle_t _srv = nullptr;
    valence::Hub* _hub = nullptr;
    ValenceWsTransport _slots[kSlots]{};
    bool _attached[kSlots]{};
    std::atomic<bool> _wantAttach[kSlots]{};
    std::atomic<bool> _wantDetach[kSlots]{};
    std::atomic<bool> _inUse[kSlots]{};
};

}  // namespace valence
