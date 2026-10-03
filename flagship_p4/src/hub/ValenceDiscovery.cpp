// ValenceDiscovery -- the socket host. See ValenceDiscovery.h for the task,
// storm and two-host rules; nothing is restated here.
// Constraints:
// - The only platform split is the socket API: Winsock in the sim (WSAStartup
//   is the sim's ix::initNetSystem()), lwIP through IDF's VFS on the board.

#include "ValenceDiscovery.h"

#include <array>
#include <cerrno>

#include "geiger/geiger.h"
#include "valence/hub/hub.hpp"

// LAST: winsock2.h pulls in windows.h, whose min/max macros break any header
// parsed after it.
#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <fcntl.h>
#include <lwip/sockets.h>
#include <unistd.h>
#endif

namespace valence {

namespace {

constexpr const char* kTag = "discovery";

// Room past the longest datagram this port takes (a 9-byte probe, RFC-053's
// 12-byte ESTOP frame): a longer one reads truncated (lwIP) or fails
// (Winsock), and is neither.
constexpr size_t kRxBytes = 32;

#if defined(_WIN32)
using SockLen = int;
SOCKET native(intptr_t s) { return SOCKET(s); }
intptr_t openUdp() {
    const SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    return s == INVALID_SOCKET ? intptr_t(-1) : intptr_t(s);
}
bool setNonBlocking(intptr_t s) {
    u_long on = 1;
    return ioctlsocket(native(s), FIONBIO, &on) == 0;
}
void closeSocket(intptr_t s) { closesocket(native(s)); }
// Any other error (an ICMP-unreachable echo of an earlier reply surfaces as
// WSAECONNRESET here) costs one bounded iteration, never the loop.
bool queueEmpty() { return WSAGetLastError() == WSAEWOULDBLOCK; }
#else
using SockLen = socklen_t;
int native(intptr_t s) { return int(s); }
intptr_t openUdp() { return intptr_t(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)); }
bool setNonBlocking(intptr_t s) {
    const int fl = fcntl(native(s), F_GETFL, 0);
    return fl >= 0 && fcntl(native(s), F_SETFL, fl | O_NONBLOCK) == 0;
}
void closeSocket(intptr_t s) { close(native(s)); }
bool queueEmpty() { return errno == EWOULDBLOCK || errno == EAGAIN; }
#endif

}  // namespace

bool ValenceDiscoveryPort::begin(uint16_t port, std::string_view hubName, std::string_view fwVersion,
                                 uint16_t wsPort) {
    end();
    _responder.setIdentity(hubName, fwVersion, wsPort);
    const intptr_t s = openUdp();
    if (s == kNoSocket) {
        GLOGW(kTag, "no UDP socket: discovery off, a typed address still works");
        return false;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (!setNonBlocking(s) || bind(native(s), reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        GLOGW(kTag, "UDP :%u unavailable: discovery off, a typed address still works", unsigned(port));
        closeSocket(s);
        return false;
    }
    _sock = s;
    GLOGI(kTag, "answering DISCOVER_PROBE on UDP :%u", unsigned(port));
    return true;
}

void ValenceDiscoveryPort::poll(const Hub& hub, uint32_t nowMs) {
    if (_sock == kNoSocket) return;
    bool live = false;
    for (size_t i = 0; i < kDiscoveryDrainPerPoll; ++i) {
        std::array<std::byte, kRxBytes> in{};
        sockaddr_in from{};
        SockLen fromLen = sizeof(from);
        const int n = int(recvfrom(native(_sock), reinterpret_cast<char*>(in.data()), int(in.size()), 0,
                                   reinterpret_cast<sockaddr*>(&from), &fromLen));
        if (n < 0) {
            if (queueEmpty()) return;
            continue;
        }
        if (!live) {
            _responder.setLive(hub.hubInstanceId(), hub.catalogEtag(), hub.pairingWindowOpen());
            live = true;
        }
        std::array<std::byte, kDiscoverReplyBytes> out{};
        const uint32_t src = ntohl(from.sin_addr.s_addr);
        const size_t len = _responder.answer(std::span<const std::byte>(in.data(), size_t(n)), src, nowMs, out);
        if (len == 0) continue;
        // A failed send is a lost datagram: the client re-probes.
        sendto(native(_sock), reinterpret_cast<const char*>(out.data()), int(len), 0,
               reinterpret_cast<const sockaddr*>(&from), fromLen);
        GLOGI_EVERY_MS(10000, kTag, "answered %u.%u.%u.%u:%u (%lu replies, %lu throttled, %lu not probes)",
                       unsigned(src >> 24), unsigned((src >> 16) & 0xFF), unsigned((src >> 8) & 0xFF),
                       unsigned(src & 0xFF), unsigned(ntohs(from.sin_port)),
                       static_cast<unsigned long>(_responder.replies()),
                       static_cast<unsigned long>(_responder.throttled()),
                       static_cast<unsigned long>(_responder.malformed()));
    }
}

void ValenceDiscoveryPort::end() {
    if (_sock == kNoSocket) return;
    closeSocket(_sock);
    _sock = kNoSocket;
}

}  // namespace valence
