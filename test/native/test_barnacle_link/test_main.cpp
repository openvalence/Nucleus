// test_barnacle_link -- native doctest suite for the barnacle helper link:
// CRC, COBS, the three frames' round trips, loss and resync, the sequence
// tracker, the helper's trip detector and the P4's link policy
// Constraints:
// - Hardware-free: barnacle_link.hpp and BarnacleLink.h's policy. The UARTs,
//   the TRIP line and the ADC are bench work (bd val-omh3).
// - The link header is included by its library name, so the LDF puts
//   lib/barnacle_link/include on the path BarnacleLink.h needs.
// See: lib/barnacle_link/include/barnacle_link/barnacle_link.hpp,
// flagship_p4/src/system/BarnacleLink.h

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "barnacle_link/barnacle_link.hpp"
#include "../../../flagship_p4/src/system/BarnacleLink.h"

namespace bl = barnacle;

namespace {

using Wire = std::vector<uint8_t>;

template <typename T>
Wire wireOf(const T& body, uint8_t seq) {
    std::array<uint8_t, bl::kMaxWire> out{};
    const size_t n = bl::encode(body, seq, out);
    REQUIRE(n > 0);
    return Wire(out.begin(), out.begin() + long(n));
}

// Feeds every byte; returns every verified frame in order.
std::vector<bl::Frame> feedAll(bl::Decoder& d, const Wire& w) {
    std::vector<bl::Frame> got;
    for (const uint8_t b : w)
        if (const std::optional<bl::Frame> f = d.feed(b)) got.push_back(*f);
    return got;
}

bl::Status sampleStatus() {
    bl::Status s;
    s.flags = bl::kFlagButton | bl::kFlagTrip;
    s.current_raw = 0x0A00;   // carries a zero byte
    s.current_ma = -1234;
    s.motor_mv = 36100;
    return s;
}

bl::Slow sampleSlow() {
    bl::Slow s;
    s.uptime_ms = 0x01020304;
    s.boots = 3;
    s.reset_reason = 4;
    s.mode = bl::Mode::homing;
    s.home_trip_ma = 1000;
    s.run_trip_ma = 8000;
    s.ana1_mv = 0;
    s.ana2_mv = bl::kNoMillivolts;
    s.trips = 7;
    s.bad_frames = 2;
    s.late_samples = 0x0100;   // carries a zero byte
    s.shim_version = 1;
    return s;
}

bl::Command sampleCommand() {
    bl::Command c;
    c.mode = bl::Mode::running;
    c.r = 0;
    c.g = 16;
    c.b = 255;
    c.home_trip_ma = 1000;
    c.run_trip_ma = 8000;
    return c;
}

}  // namespace

// ---- CRC and COBS --------------------------------------------------------------

TEST_CASE("CRC-16/CCITT-FALSE check value") {
    constexpr std::array<uint8_t, 9> check{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    static_assert(bl::crc16(check) == 0x29B1);
    CHECK(bl::crc16(check) == 0x29B1);
}

TEST_CASE("COBS round trips zeros, runs and the 254-byte boundary, and never emits a zero") {
    for (const size_t len : {size_t(0), size_t(1), size_t(5), size_t(253), size_t(254), size_t(255), size_t(600)}) {
        for (const int pattern : {0, 1, 2}) {
            std::vector<uint8_t> in(len);
            for (size_t i = 0; i < len; ++i)
                in[i] = pattern == 0 ? 0 : (pattern == 1 ? uint8_t(1 + i % 255) : uint8_t(i % 7 == 0 ? 0 : i));
            std::vector<uint8_t> enc(len + len / 254 + 1);
            const size_t n = bl::cobsEncode(in, enc);
            REQUIRE(n > 0);
            for (size_t i = 0; i < n; ++i) CHECK(enc[i] != 0);
            std::vector<uint8_t> dec(len + 1);
            const std::optional<size_t> got = bl::cobsDecode(std::span<const uint8_t>(enc.data(), n), dec);
            REQUIRE(got.has_value());
            CHECK(*got == len);
            CHECK(std::equal(in.begin(), in.end(), dec.begin()));
        }
    }
}

TEST_CASE("COBS refuses a short output buffer and a code past the end") {
    std::array<uint8_t, 4> in{1, 2, 3, 4};
    std::array<uint8_t, 4> small{};
    CHECK(bl::cobsEncode(in, small) == 0);
    const std::array<uint8_t, 2> bad{5, 1};
    std::array<uint8_t, 8> out{};
    CHECK_FALSE(bl::cobsDecode(bad, out).has_value());
}

// ---- frames --------------------------------------------------------------------

TEST_CASE("status, slow and command round trip with their seq") {
    bl::Decoder d;
    Wire w = wireOf(sampleStatus(), 9);
    const Wire s = wireOf(sampleSlow(), 10);
    const Wire c = wireOf(sampleCommand(), 11);
    w.insert(w.end(), s.begin(), s.end());
    w.insert(w.end(), c.begin(), c.end());
    const std::vector<bl::Frame> got = feedAll(d, w);
    REQUIRE(got.size() == 3);
    CHECK(got[0].seq == 9);
    CHECK(bl::asStatus(got[0]) == std::optional<bl::Status>(sampleStatus()));
    CHECK(got[1].seq == 10);
    CHECK(bl::asSlow(got[1]) == std::optional<bl::Slow>(sampleSlow()));
    CHECK(got[2].seq == 11);
    CHECK(bl::asCommand(got[2]) == std::optional<bl::Command>(sampleCommand()));
    CHECK(d.stats().frames == 3);
    CHECK_FALSE(bl::asCommand(got[0]).has_value());   // a status is not a command
}

TEST_CASE("the status frame is 14 bytes on the wire: the bandwidth budget") {
    CHECK(wireOf(sampleStatus(), 0).size() == 14);
    // 500 Hz of status plus 10 Hz of slow, 10 bits a byte, inside 80 % of the baud.
    const uint32_t bits = 500u * 14u * 10u + 10u * uint32_t(wireOf(sampleSlow(), 0).size()) * 10u;
    CHECK(bits < bl::kBaud * 8 / 10);
}

TEST_CASE("a corrupted byte costs its own frame and nothing after it") {
    const Wire a = wireOf(sampleStatus(), 1);
    const Wire b = wireOf(sampleStatus(), 2);
    for (size_t at = 0; at + 1 < a.size(); ++at) {
        Wire w = a;
        w[at] = uint8_t(w[at] ^ 0x5A);
        if (w[at] == 0) w[at] = 0x33;   // a zero would split it, tested below
        w.insert(w.end(), b.begin(), b.end());
        bl::Decoder d;
        const std::vector<bl::Frame> got = feedAll(d, w);
        REQUIRE(got.size() == 1);
        CHECK(got[0].seq == 2);
        CHECK(d.stats().crc + d.stats().malformed == 1);
    }
}

TEST_CASE("a dropped byte or a joined-mid-frame start resyncs on the next delimiter") {
    const Wire a = wireOf(sampleStatus(), 1);
    const Wire b = wireOf(sampleSlow(), 2);
    for (size_t drop = 0; drop + 1 < a.size(); ++drop) {
        Wire w = a;
        w.erase(w.begin() + long(drop));
        w.insert(w.end(), b.begin(), b.end());
        bl::Decoder d;
        const std::vector<bl::Frame> got = feedAll(d, w);
        REQUIRE(got.size() == 1);
        CHECK(got[0].type == bl::Type::slow);
    }
    // Joining at any byte of a frame: the tail is thrown away, the next frame holds.
    for (size_t start = 1; start < a.size(); ++start) {
        Wire w(a.begin() + long(start), a.end());
        w.insert(w.end(), b.begin(), b.end());
        bl::Decoder d;
        const std::vector<bl::Frame> got = feedAll(d, w);
        REQUIRE(got.size() == 1);
        CHECK(got[0].seq == 2);
    }
}

TEST_CASE("a stray zero splits a frame into two rejects, then the stream holds") {
    Wire w = wireOf(sampleStatus(), 1);
    w.insert(w.begin() + 4, uint8_t(0));
    const Wire b = wireOf(sampleStatus(), 2);
    w.insert(w.end(), b.begin(), b.end());
    bl::Decoder d;
    const std::vector<bl::Frame> got = feedAll(d, w);
    REQUIRE(got.size() == 1);
    CHECK(got[0].seq == 2);
}

TEST_CASE("an overlong block is dropped whole and counted, the next frame holds") {
    const Wire b = wireOf(sampleCommand(), 5);
    Wire w(bl::kMaxWire + 41 + b.size(), uint8_t(0x11));
    w[bl::kMaxWire + 40] = 0;
    std::copy(b.begin(), b.end(), w.begin() + long(bl::kMaxWire + 41));
    bl::Decoder d;
    const std::vector<bl::Frame> got = feedAll(d, w);
    REQUIRE(got.size() == 1);
    CHECK(got[0].seq == 5);
    CHECK(d.stats().overrun == 1);
}

TEST_CASE("another version is dropped and counted, not parsed") {
    std::array<uint8_t, bl::kMaxPayload> p{};
    p[0] = bl::kVersion + 1;
    p[1] = uint8_t(bl::Type::status);
    p[2] = 0;
    const size_t covered = bl::kHeaderBytes + bl::Status::kBytes;
    bl::putU16(p, covered, bl::crc16(std::span<const uint8_t>(p.data(), covered)));
    std::array<uint8_t, bl::kMaxWire> w{};
    const size_t n = bl::cobsEncode(std::span<const uint8_t>(p.data(), covered + bl::kCrcBytes), w);
    bl::Decoder d;
    for (size_t i = 0; i < n; ++i) CHECK_FALSE(d.feed(w[i]).has_value());
    CHECK_FALSE(d.feed(0).has_value());
    CHECK(d.stats().version == 1);
    CHECK(d.stats().frames == 0);
}

TEST_CASE("a layout of the wrong length or an unknown mode never parses") {
    bl::Frame f;
    f.type = bl::Type::command;
    f.len = bl::Command::kBytes - 1;
    CHECK_FALSE(bl::asCommand(f).has_value());
    f.len = bl::Command::kBytes;
    f.body[0] = 9;   // no such mode
    CHECK_FALSE(bl::asCommand(f).has_value());
    f.body[0] = uint8_t(bl::Mode::homing);
    CHECK(bl::asCommand(f).has_value());
}

TEST_CASE("back-to-back delimiters are silence, not errors") {
    bl::Decoder d;
    for (int i = 0; i < 5; ++i) CHECK_FALSE(d.feed(0).has_value());
    CHECK(d.stats().malformed == 0);
    CHECK(d.stats().crc == 0);
}

// ---- sequence ------------------------------------------------------------------

TEST_CASE("sequence gaps count lost frames across the wrap, an outage counts none") {
    bl::SeqTracker t;
    CHECK(t.note(250) == 0);   // the first frame says nothing
    CHECK(t.note(251) == 0);
    CHECK(t.note(254) == 2);
    CHECK(t.note(1) == 2);     // 255 and 0 lost across the wrap
    CHECK(t.note(1) == 0);     // a repeat is not a loss
    CHECK(t.note(200) == 0);   // 199 ahead: read as an outage
    CHECK(t.lost() == 4);
    t.reset();
    CHECK(t.note(7) == 0);
}

// ---- the trip detector ---------------------------------------------------------

TEST_CASE("trip: asserts after kTripOnSamples at or over, never on fewer") {
    bl::TripDetector t;
    for (uint32_t i = 0; i + 1 < bl::kTripOnSamples; ++i) CHECK_FALSE(t.sample(1000, 1000));
    CHECK_FALSE(t.sample(999, 1000));   // one under restarts the count
    for (uint32_t i = 0; i + 1 < bl::kTripOnSamples; ++i) CHECK_FALSE(t.sample(1500, 1000));
    CHECK(t.sample(1500, 1000));
    CHECK(t.assertions() == 1);
}

TEST_CASE("trip: holds through the band, releases after kTripOffSamples under the release level") {
    bl::TripDetector t;
    for (uint32_t i = 0; i < bl::kTripOnSamples; ++i) t.sample(2000, 1000);
    REQUIRE(t.asserted());
    const int32_t release = 1000 * int32_t(bl::kTripReleasePercent) / 100;
    for (int i = 0; i < 20; ++i) CHECK(t.sample(release, 1000));   // in the band: held
    for (uint32_t i = 0; i + 1 < bl::kTripOffSamples; ++i) CHECK(t.sample(release - 1, 1000));
    CHECK_FALSE(t.sample(release - 1, 1000));
    CHECK(t.assertions() == 1);
}

TEST_CASE("trip: regeneration never trips, a zero threshold disables and clears") {
    bl::TripDetector t;
    for (int i = 0; i < 50; ++i) CHECK_FALSE(t.sample(-20000, 1000));
    for (uint32_t i = 0; i < bl::kTripOnSamples; ++i) t.sample(5000, 1000);
    REQUIRE(t.asserted());
    CHECK_FALSE(t.sample(5000, 0));
    CHECK_FALSE(t.asserted());
}

TEST_CASE("trip: the mode picks the threshold; the detect window is the stall-detect budget") {
    CHECK(bl::activeThreshold(bl::Mode::homing, 1000, 8000) == 1000);
    CHECK(bl::activeThreshold(bl::Mode::running, 1000, 8000) == 8000);
    CHECK(bl::activeThreshold(bl::Mode::idle, 1000, 8000) == 8000);
    CHECK(bl::kTripDetectUs == bl::kTripSampleUs * bl::kTripOnSamples);
}

// ---- the P4's link policy (BarnacleLink.h) ---------------------------------------

TEST_CASE("mode: a home cycle is homing, an open gate without ESTOP is running, else idle") {
    using valence::modeFor;
    CHECK(modeFor(true, false, false) == bl::Mode::homing);
    CHECK(modeFor(true, true, true) == bl::Mode::homing);
    CHECK(modeFor(false, true, false) == bl::Mode::running);
    CHECK(modeFor(false, true, true) == bl::Mode::idle);
    CHECK(modeFor(false, false, false) == bl::Mode::idle);
}

TEST_CASE("running trip: once per trip, only while running, and a trip held into running raises") {
    valence::RunTripWatch w;
    CHECK_FALSE(w.sample(true, bl::Mode::homing));    // the stall is the home sense's
    CHECK(w.sample(true, bl::Mode::running));         // held into running: raised once
    CHECK_FALSE(w.sample(true, bl::Mode::running));
    CHECK_FALSE(w.sample(false, bl::Mode::running));  // released
    CHECK(w.sample(true, bl::Mode::running));         // a new trip
    CHECK_FALSE(w.sample(true, bl::Mode::idle));
}

TEST_CASE("helper restart: the boot count moved or the uptime ran backwards") {
    bl::Slow a;
    a.boots = 1;
    a.uptime_ms = 5000;
    bl::Slow b = a;
    b.uptime_ms = 5100;
    CHECK_FALSE(valence::helperRestarted(a, b));
    b.uptime_ms = 40;
    CHECK(valence::helperRestarted(a, b));
    b = a;
    b.boots = 2;
    CHECK(valence::helperRestarted(a, b));
}

TEST_CASE("the self-check's helper row: never heard, stale and no slow frame fail; fresh passes") {
    std::array<char, 72> out{};
    valence::HelperReading h;
    CHECK_FALSE(valence::describeHelper(h, out));
    h.ever = true;
    h.age_ms = 400;
    CHECK_FALSE(valence::describeHelper(h, out));
    CHECK(std::string(out.data()).find("400 ms") != std::string::npos);
    h.fresh = true;
    CHECK_FALSE(valence::describeHelper(h, out));
    h.have_slow = true;
    h.slow.shim_version = 1;
    h.slow.boots = 2;
    h.status.motor_mv = 36100;
    h.status.current_ma = 250;
    CHECK(valence::describeHelper(h, out));
    CHECK(std::string(out.data()) == "helper v1 boot 2: 36.1 V, 250 mA provisional");
    h.status.motor_mv = bl::kNoMillivolts;
    CHECK(valence::describeHelper(h, out));
    CHECK(std::string(out.data()).find("no motor V") != std::string::npos);
}

TEST_CASE("the P4 sends the link's default thresholds") {
    CHECK(valence::kHomeTripMa == bl::kDefaultHomeTripMa);
    CHECK(valence::kRunTripMa == bl::kDefaultRunTripMa);
    CHECK(valence::kHomeTripMa < valence::kRunTripMa);
}
