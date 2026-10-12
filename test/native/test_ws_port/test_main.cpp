// test_ws_port -- native doctest suite for the P4 WebSocket transport's send path
// Constraints:
// - ValenceWsPort.cpp is compiled verbatim against the stand-in ESP-IDF
//   headers beside this file. The fake socket models lwIP as measured: a send
//   into a full buffer parks the caller until the peer ACKs or the 1 s netconn
//   poll fires, whatever SO_SNDTIMEO says (api_msg.c, NETCONN_TCP_POLL_INTERVAL).
// - Time is the fake clock: a tick advances it 5 ms, a parked send 1 s.
// See: flagship_p4/src/hub/ValenceWsPort.h, bd val-ni0

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <span>

// Named directly so the library finder adds lib/valence and lib/geiger; it
// does not follow the relative include below.
#include "geiger/geiger.h"
#include "valence/hub/hub.hpp"

#include "esp_http_server.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "../../../flagship_p4/src/hub/ValenceWsPort.cpp"

// ---- the fake socket layer ----------------------------------------------------

namespace {

constexpr int64_t kParkUs = 1000000;   // lwIP's netconn poll: the real bound on a parked send
constexpr int64_t kSendUs = 50;        // a send into a buffer with room

struct FakeSock {
    bool room = true;
    uint32_t delivered = 0;
    uint32_t parked = 0;
    uint32_t closes = 0;
};

int64_t g_nowUs = 1000000;
std::map<int, FakeSock> g_socks;
httpd_handle_t const kHd = reinterpret_cast<httpd_handle_t>(0x1);

}  // namespace

int64_t esp_timer_get_time() { return g_nowUs; }
void vTaskDelay(uint32_t) {}
int test_setsockopt(int, int, int, const void*, unsigned) { return 0; }

int test_select(int nfds, test_fd_set*, test_fd_set* w, test_fd_set*, test_timeval*) {
    int ready = 0;
    for (int fd = 0; fd < nfds; ++fd) {
        if (!FD_ISSET(fd, w)) continue;
        if (g_socks[fd].room) ++ready;
        else w->bits[fd / 64] &= ~(uint64_t(1) << (fd % 64));
    }
    return ready;
}

esp_err_t httpd_ws_send_frame_async(httpd_handle_t, int fd, httpd_ws_frame_t*) {
    FakeSock& s = g_socks[fd];
    if (!s.room) {
        ++s.parked;
        g_nowUs += kParkUs;
        return ESP_FAIL;
    }
    ++s.delivered;
    g_nowUs += kSendUs;
    return ESP_OK;
}

esp_err_t httpd_ws_send_frame(httpd_req_t*, httpd_ws_frame_t*) { return ESP_OK; }
esp_err_t httpd_ws_recv_frame(httpd_req_t*, httpd_ws_frame_t*, size_t) { return ESP_FAIL; }
int httpd_req_to_sockfd(httpd_req_t*) { return -1; }
esp_err_t httpd_sess_trigger_close(httpd_handle_t, int fd) {
    ++g_socks[fd].closes;
    return ESP_OK;
}
esp_err_t httpd_start(httpd_handle_t*, const httpd_config_t*) { return ESP_FAIL; }
esp_err_t httpd_register_uri_handler(httpd_handle_t, const httpd_uri_t*) { return ESP_FAIL; }
esp_err_t httpd_get_client_list(httpd_handle_t, size_t* n, int*) {
    *n = 0;
    return ESP_OK;
}

namespace {

using valence::FrameType;
using valence::ValenceWsTransport;

std::array<std::byte, 24> frameOf(FrameType t) {
    std::array<std::byte, 24> f{};
    f[0] = std::byte(uint8_t(t));
    return f;
}

bool send(ValenceWsTransport& t, FrameType type) {
    auto f = frameOf(type);
    return t.write(std::span<const std::byte>(f));
}

uint32_t nowMs() { return uint32_t(g_nowUs / 1000); }

void reset() {
    g_socks.clear();
    g_nowUs = 1000000;
}

}  // namespace

// ---- isolation ------------------------------------------------------------------

TEST_CASE("WP-01: a peer that stops reading costs the other slot nothing") {
    reset();
    static ValenceWsTransport wedged, healthy;
    wedged.bind(kHd, 1001);
    healthy.bind(kHd, 1002);
    g_socks[1001].room = false;

    // Two seconds of 5 ms hub ticks, each fanning a STATE frame to both slots
    // and, every 100 ms, an EVENT (control) frame.
    const int64_t end = g_nowUs + 2000000;
    uint32_t ticks = 0;
    while (g_nowUs < end) {
        for (ValenceWsTransport* t : {&wedged, &healthy}) {
            t->newTick();
            t->pollCongestionLevel(nowMs());
            send(*t, FrameType::STATE);
            if (ticks % 20 == 0) send(*t, FrameType::EVENT);
        }
        g_nowUs += 5000;
        ++ticks;
    }
    CAPTURE(ticks);
    CHECK(ticks >= 390);                          // the tick kept its 5 ms cadence
    CHECK(g_socks[1002].delivered >= 390);        // the healthy peer kept its rate
    CHECK(g_socks[1001].parked == 0);             // nothing was handed to the full socket
    CHECK(wedged.stalledOut(nowMs() + 100));      // and the wedged one is due for teardown
    CHECK_FALSE(healthy.stalledOut(nowMs()));
}

TEST_CASE("WP-02: a teardown asks httpd to close once, however many ticks it takes") {
    reset();
    static ValenceWsTransport t;
    t.bind(kHd, 1003);
    for (int i = 0; i < 60; ++i) t.close();
    CHECK(g_socks[1003].closes == 1);
    CHECK(t.closeRequested());
    CHECK_FALSE(send(t, FrameType::EVENT));       // nothing more goes to a closing socket

    t.bind(kHd, 1003);                            // the next connection on a reused fd
    CHECK_FALSE(t.closeRequested());
    t.close();
    CHECK(g_socks[1003].closes == 2);
}

// ---- the catalog transfer ---------------------------------------------------------

TEST_CASE("WP-03: a congested link still moves its catalog; only room holds a chunk") {
    reset();
    static ValenceWsTransport t;
    t.bind(kHd, 1004);

    // A full buffer for 1.2 s of STATE frames raises the §10.3 level to 1.
    g_socks[1004].room = false;
    for (int i = 0; i < 240; ++i) {
        t.newTick();
        t.pollCongestionLevel(nowMs());
        send(t, FrameType::STATE);
        g_nowUs += 5000;
    }
    REQUIRE(t.pollCongestionLevel(nowMs()) == 1);

    // No room: the chunk is held, and holding it neither stalls control nor
    // keeps the link marked slow.
    t.newTick();
    CHECK_FALSE(send(t, FrameType::BLOB_CHUNK));
    CHECK_FALSE(t.stalledOut(nowMs() + 5000));

    // Room again: the chunk goes, though the level is still 1. The hub's own
    // in-flight budget is what paces a congested transfer.
    g_socks[1004].room = true;
    t.newTick();
    CHECK(send(t, FrameType::BLOB_CHUNK));
    CHECK(g_socks[1004].delivered == 1);
}

TEST_CASE("WP-04: the buffer a moving transfer fills is not congestion") {
    reset();
    static ValenceWsTransport t;
    t.bind(kHd, 1005);
    for (int i = 0; i < 400; ++i) {               // two seconds of a full-rate download
        t.newTick();
        t.pollCongestionLevel(nowMs());
        g_socks[1005].room = true;
        for (int c = 0; c < 4; ++c) send(t, FrameType::BLOB_CHUNK);
        g_socks[1005].room = false;               // the chunks filled it
        send(t, FrameType::STATE);                // shed this tick, not counted
        g_nowUs += 5000;
    }
    CHECK(t.pollCongestionLevel(nowMs()) == 0);
    CHECK(g_socks[1005].delivered == 1600);
}
