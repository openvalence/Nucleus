// test_crash_ring -- native doctest suite for the breadcrumbs a boot leaves
// for the next one
// Constraints:
// - Hardware-free: the two copies are plain values here. Whether LP RAM or HP
//   SRAM holds across a reset is the silicon's, read off /diag on the bench.
// See: flagship_p4/src/system/CrashRing.h

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstdio>
#include <cstring>

#include "../../../flagship_p4/src/system/CrashRing.h"

using namespace valence::crashring;

namespace {

Ring armed(uint32_t boot = 7) {
    Ring r{};
    arm(r, boot);
    return r;
}

}  // namespace

TEST_CASE("CR-01: a reminder repeated every minute does not push the fault out of the ring") {
    Ring r = armed();
    add(r, "W wifi DROP #1: beacon timeout (200), rssi -61 dBm");
    add(r, "W hosted sdio_drv: SDIO slave unresponsive");
    for (int i = 0; i < 50; ++i) {
        add(r, "E selfcheck motor power held OFF: 2 failed, 5 skipped");
        add(r, "W health RSSI read over esp_hosted failed: ESP_FAIL");
    }
    char out[kCrumbs][kCrumbBytes];
    REQUIRE(copyOut(r, out) == 4);
    CHECK(std::strcmp(out[0], "W wifi DROP #1: beacon timeout (200), rssi -61 dBm") == 0);
    CHECK(std::strcmp(out[1], "W hosted sdio_drv: SDIO slave unresponsive") == 0);
    CHECK(std::strcmp(out[3], "W health RSSI read over esp_hosted failed: ESP_FAIL") == 0);
}

TEST_CASE("CR-02: a full ring reads oldest first and keeps the last kCrumbs distinct lines") {
    Ring r = armed();
    char line[kCrumbBytes];
    for (int i = 0; i < 11; ++i) {
        std::snprintf(line, sizeof(line), "W motion STALL %d", i);
        add(r, line);
    }
    char out[kCrumbs][kCrumbBytes];
    REQUIRE(copyOut(r, out) == kCrumbs);
    CHECK(std::strcmp(out[0], "W motion STALL 3") == 0);
    CHECK(std::strcmp(out[kCrumbs - 1], "W motion STALL 10") == 0);
}

TEST_CASE("CR-03: a line longer than a crumb truncates and still dedupes") {
    Ring r = armed();
    const char* longLine = "E hosted a line well past the sixty bytes a crumb holds, so it truncates here";
    add(r, longLine);
    add(r, longLine);
    REQUIRE(r.count == 1);
    CHECK(std::strlen(r.crumb[0]) == kCrumbBytes - 1);
}

TEST_CASE("CR-04: the LP copy wins, the HP copy stands in, garbage is no copy") {
    Ring lp = armed(16);
    Ring hp = armed(16);
    add(hp, "W ota rebooting into the new slot");
    CHECK(pick(lp, hp) == &lp);

    std::memset(&lp, 0x5A, sizeof(lp));   // the LP RAM a system reset leaves
    CHECK(pick(lp, hp) == &hp);

    Ring older{};
    older.magic = 0x4E554331u;             // the previous layout's magic
    older.magic2 = ~0x4E554331u;
    CHECK(!valid(older));
    std::memset(&hp, 0xA5, sizeof(hp));
    CHECK(pick(lp, hp) == nullptr);
}

TEST_CASE("CR-05: last words truncate, survive the ring's traffic and clear on arm") {
    Ring r = armed();
    setLast(r, "TWDT starved: ValenceHub (CPU 1) | running Planner / IDLE0 and a tail long enough to be cut off here");
    add(r, "W motion STALL");
    CHECK(valid(r));
    CHECK(std::strlen(r.last) == kLastBytes - 1);
    CHECK(std::strncmp(r.last, "TWDT starved: ValenceHub (CPU 1)", 32) == 0);
    arm(r, 8);
    CHECK(r.last[0] == '\0');
    CHECK(r.count == 0);
    CHECK(r.bootSeq == 8);
}

TEST_CASE("CR-06: a copy is a whole number of cache lines") {
    CHECK(alignof(Ring) == 64);
    CHECK(sizeof(Ring) % 64 == 0);
}
