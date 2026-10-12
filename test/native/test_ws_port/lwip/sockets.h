// sockets.h -- host stand-in for lwIP's socket API in test_ws_port
// Constraints:
// - Every name is a macro onto a test_ symbol, as lwIP's own header maps
//   them onto lwip_ ones, so nothing collides with a host socket header
//   pulled in earlier.
#pragma once

#include <cstdint>

struct test_timeval {
    long tv_sec;
    long tv_usec;
};
struct test_fd_set {
    uint64_t bits[32];
};

#define timeval test_timeval
#define fd_set test_fd_set
#undef FD_ZERO
#undef FD_SET
#undef FD_ISSET
#define FD_ZERO(s) (*(s) = test_fd_set{})
#define FD_SET(fd, s) ((s)->bits[(fd) / 64] |= (uint64_t(1) << ((fd) % 64)))
#define FD_ISSET(fd, s) (((s)->bits[(fd) / 64] >> ((fd) % 64)) & 1)

#define IPPROTO_TCP 6
#define TCP_NODELAY 1
#define SOL_SOCKET 0xfff
#define SO_SNDTIMEO 0x1005

int test_setsockopt(int fd, int level, int name, const void* value, unsigned len);
int test_select(int nfds, test_fd_set* r, test_fd_set* w, test_fd_set* e, test_timeval* timeout);
#define setsockopt test_setsockopt
#define select test_select
