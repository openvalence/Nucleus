// test_tcp_tally -- native doctest suite for the classifier behind hub-status
// tcp_sent and tcp_resent
// Constraints:
// - Hardware-free: segments are literals in the shape ValenceTcpTally.cpp
//   reads from lwIP's pcb and header. That lwIP never rewinds snd_nxt is
//   lwIP's (tcp_out.c), not asserted here.
// See: flagship_p4/src/system/TcpTally.h, Valence RFC-109 (draft)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "../../../flagship_p4/src/system/TcpTally.h"

using namespace valence::tcptally;

namespace {

constexpr uint16_t kPort = 82;

Segment seg(uint32_t seqno, uint32_t sndNxt, uint32_t len, uint16_t port = kPort) {
    Segment s;
    s.localPort = port;
    s.seqno = seqno;
    s.sndNxt = sndNxt;
    s.seqLen = len;
    return s;
}

}  // namespace

TEST_CASE("TT-01: a segment at snd_nxt is a first send, one below it a resend") {
    CHECK(classify(seg(1000, 1000, 100), kPort) == Kind::sent);
    CHECK(classify(seg(999, 1000, 100), kPort) == Kind::resent);
    CHECK(classify(seg(500, 1000, 100), kPort) == Kind::resent);
}

TEST_CASE("TT-02: sequence space wraps: a resend just below a wrapped snd_nxt still counts as one") {
    CHECK(classify(seg(0xFFFFFF00u, 0x00000010u, 100), kPort) == Kind::resent);
    CHECK(classify(seg(0x00000010u, 0x00000010u, 100), kPort) == Kind::sent);
    CHECK(classify(seg(0x00000010u, 0xFFFFFF00u, 100), kPort) == Kind::sent);
}

TEST_CASE("TT-03: pure ACKs, RSTs and keepalives occupy no sequence space and never count") {
    CHECK(classify(seg(1000, 1000, 0), kPort) == Kind::none);
    CHECK(classify(seg(999, 1000, 0), kPort) == Kind::none);   // a keepalive sits at snd_nxt - 1
}

TEST_CASE("TT-04: SYN and FIN count as one each; the caller folds them into seqLen") {
    CHECK(classify(seg(5000, 5000, 1), kPort) == Kind::sent);
    CHECK(classify(seg(4999, 5000, 1), kPort) == Kind::resent);
}

TEST_CASE("TT-06: a zero-window probe is never counted, though it starts below snd_nxt") {
    Segment probe = seg(999, 1000, 1);
    probe.persisting = true;
    CHECK(classify(probe, kPort) == Kind::none);
}

TEST_CASE("TT-05: another port, a listen pcb or no watched port count nothing") {
    CHECK(classify(seg(1000, 1000, 100, 80), kPort) == Kind::none);
    Segment l = seg(1000, 1000, 100);
    l.listening = true;
    CHECK(classify(l, kPort) == Kind::none);
    CHECK(classify(seg(1000, 1000, 100), 0) == Kind::none);
}

TEST_CASE("TW-01: the window reads resent over sent, hundredths of a percent, rounded") {
    ResentWindow w;
    CHECK(w.push(0, 100, 0) == kNoReading);          // one snapshot is not a rate
    CHECK(w.push(1000, 1100, 10) == 100);            // 10 of 1000
    CHECK(w.push(2000, 1103, 11) == 110);            // 11 of 1003 = 1.0967 %
}

TEST_CASE("TW-02: nothing sent in the window is no reading, never 0") {
    ResentWindow w;
    w.push(0, 500, 5);
    CHECK(w.push(1000, 500, 5) == kNoReading);
}

TEST_CASE("TW-03: ten seconds and the old resends slide out") {
    ResentWindow w;
    w.push(0, 0, 0);
    w.push(1000, 100, 50);
    for (uint32_t t = 2; t <= 11; ++t) w.push(t * 1000, 100 + (t - 1) * 100, 50);
    CHECK(w.push(12000, 1200, 50) == 0);
}

TEST_CASE("TW-04: a publish inside a second replaces the newest, so bursts never shrink the window") {
    ResentWindow w;
    w.push(0, 0, 0);
    for (int i = 0; i < 20; ++i) w.push(500, 100, 1);
    CHECK(w.push(900, 200, 2) == 100);               // still measured from t=0
}

TEST_CASE("TW-05: totals wrap modulo 2^32") {
    ResentWindow w;
    w.push(0, 0xFFFFFF00u, 0xFFFFFFF0u);
    CHECK(w.push(1000, 0x00000064u, 0x00000002u) == 506);    // 18 of 356 resent
}

TEST_CASE("TW-06: never above 100 %") {
    ResentWindow w;
    w.push(0, 0, 0);
    CHECK(w.push(1000, 10, 50) == 10000);
}
