// The POSIX sockets the board reaches through lwIP's BSD names, for the
// twin's Linux and macOS builds. Constraint: host-only include path; never on
// a firmware build, where IDF's real lwip/sockets.h answers.
#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
