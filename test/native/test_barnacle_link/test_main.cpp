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
#include <cmath>
#include <cstdlib>
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
    CHECK(bl::kHomeTripDetectUs == bl::kTripSampleUs * (bl::kHomeAverageSamples + bl::kTripOnSamples - 1));
}

// ---- the current zero (val-hjf7) ---------------------------------------------------

namespace {
// The shim's conversion: pin mV through the 10k over 20k divider.
int32_t sensorOf(int32_t pinMv) { return pinMv * 3 / 2; }

constexpr uint32_t kSamplesPerSecond = 1000000 / bl::kTripSampleUs;
constexpr uint32_t kRunSamples = bl::kZeroAgreeWindows * bl::kZeroSamples;

void feed(bl::CurrentZero& z, int32_t sensorMv, uint32_t n, bl::Mode m = bl::Mode::idle) {
    for (uint32_t i = 0; i < n; ++i) z.sample(sensorMv, m);
}

// The settle delay spent at a rail still at 0 V.
void settle(bl::CurrentZero& z) { feed(z, 0, bl::kZeroSettleSamples); }

// Uniform noise in -amp..amp, deterministic.
struct Noise {
    uint32_t state = 12345;
    int32_t operator()(int32_t amp) {
        state = state * 1664525u + 1013904223u;
        return int32_t((state >> 8) % uint32_t(2 * amp + 1)) - amp;
    }
};

// Idle samples from boot until the zero is taken or `limit` runs out, the
// sensor reading shape(seconds) plus noise. Returns the sample that took it.
template <typename Shape>
uint32_t runUntilZero(bl::CurrentZero& z, Shape shape, int32_t noiseMv, uint32_t limit) {
    Noise noise;
    for (uint32_t i = 0; i < limit; ++i) {
        const double t = double(i) / double(kSamplesPerSecond);
        if (z.sample(int32_t(shape(t)) + noise(noiseMv), bl::Mode::idle)) return i;
    }
    return limit;
}
}  // namespace

TEST_CASE("zero: a 5 V CH701 at rest reads 0 mA and never trips (the field's 8500 provisional mA)") {
    bl::CurrentZero z;
    const int32_t rest = sensorOf(1667);   // 2500 mV at the sensor: half of 5 V
    CHECK_FALSE(z.ma(rest, 100).has_value());
    feed(z, rest, bl::kZeroSettleSamples);
    CHECK_FALSE(z.taken());   // the settle never counts, however steady
    uint32_t early = 0;
    for (uint32_t i = 0; i + 1 < kRunSamples; ++i) early += z.sample(rest, bl::Mode::idle) ? 1 : 0;
    CHECK(early == 0);
    CHECK(z.sample(rest, bl::Mode::idle));
    REQUIRE(z.taken());
    CHECK(z.zeroMv() == rest);
    CHECK(*z.ma(rest, 100) == 0);
    CHECK(*z.ma(rest + 850, 100) == 8500);    // 8.5 A really is 8.5 A
    CHECK(*z.ma(rest - 200, 100) == -2000);   // regeneration reads negative
    bl::TripDetector t;
    for (int i = 0; i < 100; ++i) CHECK_FALSE(t.sample(*z.ma(rest, 100), bl::kDefaultRunTripMa));
}

TEST_CASE("zero: the settle and the agreement run are the documented times") {
    CHECK(bl::kZeroSettleSamples * bl::kTripSampleUs <= bl::kZeroSettleMs * 1000);
    CHECK(bl::kZeroSettleSamples * bl::kTripSampleUs > bl::kZeroSettleMs * 1000 - bl::kTripSampleUs);
    CHECK(kRunSamples * bl::kTripSampleUs >= 1900000);
    CHECK(kRunSamples * bl::kTripSampleUs <= 2100000);
}

TEST_CASE("zero: the field's sagging rail (1925 mV, a 3.86 V rail) is out of band, never a zero") {
    bl::CurrentZero z;
    settle(z);
    feed(z, 1925, 4 * kRunSamples);
    CHECK_FALSE(z.taken());
    CHECK(z.refused() == 4 * bl::kZeroAgreeWindows);
    feed(z, 2510, kRunSamples);
    REQUIRE(z.taken());
    CHECK(z.zeroMv() == 2510);
}

TEST_CASE("zero: +-180 mV of sample noise, alternating or random, still converges") {
    bl::CurrentZero z;
    settle(z);
    for (uint32_t i = 0; i < kRunSamples; ++i) z.sample(2510 + int32_t(i % 2 == 0 ? 180 : -180), bl::Mode::idle);
    REQUIRE(z.taken());
    CHECK(z.zeroMv() == 2510);

    bl::CurrentZero r;
    const uint32_t at = runUntilZero(r, [](double) { return 2500.0; }, 180, bl::kZeroSettleSamples + 3 * kRunSamples);
    REQUIRE(r.taken());
    CHECK(at < bl::kZeroSettleSamples + 2 * kRunSamples);
    CHECK(std::abs(r.zeroMv() - 2500) <= 5);
}

TEST_CASE("zero: a power-up ramp, 0 to 2.5 V over 0.5 s with +-180 mV noise, zeroes within 20 mV") {
    const auto ramp = [](double t0) {
        return [t0](double t) { return t < t0 ? 0.0 : (t < t0 + 0.5 ? 5000.0 * (t - t0) : 2500.0); };
    };
    // From boot: the settle outlasts it.
    bl::CurrentZero z;
    runUntilZero(z, ramp(0.0), 180, 10 * kSamplesPerSecond);
    REQUIRE(z.taken());
    CHECK(std::abs(z.zeroMv() - 2500) <= 20);
    // Starting as the settle ends: the band and the agreement alone reject it.
    bl::CurrentZero late;
    const uint32_t at = runUntilZero(late, ramp(double(bl::kZeroSettleMs) / 1000.0), 180, 10 * kSamplesPerSecond);
    REQUIRE(late.taken());
    CHECK(at >= bl::kZeroSettleSamples + kSamplesPerSecond * 9 / 20 + kRunSamples);   // in band from 2250 mV, 0.45 s in
    CHECK(std::abs(late.zeroMv() - 2500) <= 20);
}

TEST_CASE("zero: a rail recovering slowly into band (2250 to 2510 mV, 1 s time constant) zeroes within 20 mV") {
    const auto recover = [](double t0) {
        return [t0](double t) { return t < t0 ? 1925.0 : 2510.0 - 260.0 * std::exp(-(t - t0)); };
    };
    for (const double t0 : {0.0, 2.0, 5.0}) {
        CAPTURE(t0);
        bl::CurrentZero z;
        runUntilZero(z, recover(t0), 180, 30 * kSamplesPerSecond);
        REQUIRE(z.taken());
        CHECK(std::abs(z.zeroMv() - 2510) <= 20);
    }
}

TEST_CASE("zero: never in motion; a homing or running sample discards the window and the run") {
    bl::CurrentZero z;
    settle(z);
    feed(z, 2500, kRunSamples, bl::Mode::running);
    feed(z, 2500, kRunSamples, bl::Mode::homing);
    CHECK_FALSE(z.taken());
    feed(z, 2500, kRunSamples - 1);
    feed(z, 2500, 1, bl::Mode::running);   // the gate opened one sample short
    feed(z, 2500, 1);
    CHECK_FALSE(z.taken());
    feed(z, 2500, kRunSamples - 2);
    CHECK_FALSE(z.taken());   // the run restarted with the window
    feed(z, 2500, 1);
    CHECK(z.taken());
}

TEST_CASE("zero: a window that moves past the spread is discarded") {
    bl::CurrentZero z;
    settle(z);
    for (uint32_t i = 0; i < bl::kZeroSamples; ++i) z.sample(2300 + int32_t(i), bl::Mode::idle);   // a ramp: current flowing
    CHECK_FALSE(z.taken());
    feed(z, 2500, kRunSamples);
    CHECK(z.taken());
}

TEST_CASE("zero: window means that drift past the agreement band start a new run") {
    bl::CurrentZero z;
    settle(z);
    for (uint32_t w = 0; w < 3 * bl::kZeroAgreeWindows; ++w)
        feed(z, 2400 + int32_t(w) * 3, bl::kZeroSamples);   // 20 mV/s, under the window spread
    CHECK_FALSE(z.taken());
    feed(z, 2600, kRunSamples);
    REQUIRE(z.taken());
    CHECK(z.zeroMv() == 2600);
}

TEST_CASE("zero: outside the sane band is no zero, counted, and the next window tries again") {
    bl::CurrentZero z;
    settle(z);
    feed(z, 0, bl::kZeroSamples);                       // sensor unpowered or shorted
    feed(z, sensorOf(3100), bl::kZeroSamples);          // input open at the ADC's top
    feed(z, bl::kZeroMinMv - 1, bl::kZeroSamples);
    feed(z, bl::kZeroMaxMv + 1, bl::kZeroSamples);
    CHECK_FALSE(z.taken());
    CHECK(z.refused() == 4);
    CHECK_FALSE(z.ma(2500, 100).has_value());
    feed(z, bl::kZeroMaxMv, kRunSamples);
    CHECK(z.taken());
}

TEST_CASE("zero: taken once per boot, later samples never move it") {
    bl::CurrentZero z;
    settle(z);
    feed(z, 2300, kRunSamples);
    REQUIRE(z.taken());
    feed(z, 2600, 4 * kRunSamples);
    CHECK(z.zeroMv() == 2300);
}

TEST_CASE("rest current: half the home trip or more refuses a home (the field's 5765 mA)") {
    CHECK(bl::restCurrentSane(0, 1000));
    CHECK(bl::restCurrentSane(499, 1000));
    CHECK(bl::restCurrentSane(-499, 1000));
    CHECK_FALSE(bl::restCurrentSane(500, 1000));
    CHECK_FALSE(bl::restCurrentSane(-500, 1000));
    CHECK_FALSE(bl::restCurrentSane(5765, 1000));
    CHECK_FALSE(bl::restCurrentSane(bl::kNoCurrent, 1000));
    CHECK(bl::restCurrentSane(5765, 20000));
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
    CHECK_FALSE(valence::describeHelper(h, out));   // no 0 A zero yet: no current, no gate
    CHECK(std::string(out.data()) == "helper v1 boot 2: 36.1 V, no current: no ADC or 0 A zero");
    h.status.current_ma = 250;
    CHECK(valence::describeHelper(h, out));
    CHECK(std::string(out.data()) == "helper v1 boot 2: 36.1 V, 250 mA provisional");
    h.status.motor_mv = bl::kNoMillivolts;
    CHECK(valence::describeHelper(h, out));
    CHECK(std::string(out.data()).find("no motor V") != std::string::npos);
}

// ---- the home trip: the stock OSSM firmware's detector --------------------------

namespace {
// Provisional mA from pin mV, as the shim converts: x3/2 to the sensor, 100 mV/A.
constexpr int32_t maOfPin(int32_t pinMv) { return pinMv * 3 / 2 * 10; }

// The shim's homing chain: every sample into the mean, the mode's input into
// the detector.
struct HomeChain {
    bl::Boxcar<bl::kHomeAverageSamples> mean;
    bl::TripDetector trip;
    bool sample(int32_t ma, uint16_t threshold = bl::kDefaultHomeTripMa) {
        const std::optional<int32_t> m = mean.push(ma);
        return trip.sample(bl::tripInput(bl::Mode::homing, ma, m), threshold);
    }
};

constexpr int32_t kPinNoiseMv = 180;   // the field's per-sample noise at IO36
constexpr uint32_t kTenSeconds = 10 * kSamplesPerSecond;
}  // namespace

TEST_CASE("home trip: the reference's 6 % of 4096 counts is 198 mV at the pin, 2970 provisional mA") {
    CHECK(std::fabs(4096.0 * 0.06 * 3300.0 / 4096.0 - double(bl::kRefHomeTripPinMv)) < 0.5);
    CHECK(maOfPin(bl::kRefHomeTripPinMv) == bl::kDefaultHomeTripMa);
    CHECK(bl::kHomeAverageSamples == 50);   // the reference's getAnalogAveragePercent(pin, 50)
}

TEST_CASE("home trip: +-180 mV of pin noise per sample at rest never trips; the raw 1000 mA trip did at once") {
    Noise noise;
    HomeChain home;
    uint32_t trips = 0;
    for (uint32_t i = 0; i < kTenSeconds; ++i) trips += home.sample(maOfPin(noise(kPinNoiseMv))) ? 1 : 0;
    CHECK(trips == 0);
    CHECK(home.trip.assertions() == 0);
    // The mechanism of the field's false stall: three raw samples over 67 mV.
    bl::TripDetector raw;
    uint32_t first = kTenSeconds;
    for (uint32_t i = 0; i < kTenSeconds && first == kTenSeconds; ++i)
        if (raw.sample(maOfPin(noise(kPinNoiseMv)), 1000)) first = i;
    CHECK(first * bl::kTripSampleUs < 150000);
}

TEST_CASE("home trip: a sustained step at the reference's 6 % trips within kHomeTripDetectUs") {
    SUBCASE("a step exactly at the trip, no noise: the stated latency") {
        HomeChain home;
        for (uint32_t i = 0; i < 2 * bl::kHomeAverageSamples; ++i) REQUIRE_FALSE(home.sample(0));
        uint32_t n = 0;
        while (!home.sample(bl::kDefaultHomeTripMa) && n < 1000) ++n;
        CHECK((n + 1) * bl::kTripSampleUs == bl::kHomeTripDetectUs);
    }
    SUBCASE("a quarter over the trip under +-180 mV of noise: inside the stated latency") {
        Noise noise;
        for (int run = 0; run < 50; ++run) {
            CAPTURE(run);
            HomeChain home;
            for (uint32_t i = 0; i < 2 * bl::kHomeAverageSamples; ++i)
                REQUIRE_FALSE(home.sample(maOfPin(noise(kPinNoiseMv))));
            uint32_t n = 1;
            while (!home.sample(bl::kDefaultHomeTripMa * 5 / 4 + maOfPin(noise(kPinNoiseMv))) && n < 1000) ++n;
            CHECK(n * bl::kTripSampleUs <= bl::kHomeTripDetectUs);
        }
    }
}

// A velocity step at a leg's start is a torque burst: a burst of amplitude A
// lasting L never moves the 7.5 ms mean to the trip while A x L stays under
// the trip x 7.5 ms. The sensed legs also ramp at the reference's 1000 mm/s^2
// (MotionArbiter.h kHomeAccelMmS2), so this is the margin, not the plan.
TEST_CASE("home trip: a burst of 2.5x the trip for 2 ms, every 100 ms under noise, never trips") {
    Noise noise;
    HomeChain home;
    constexpr uint32_t kBurst = 2000 / bl::kTripSampleUs;   // 13 samples
    constexpr uint32_t kEvery = 100000 / bl::kTripSampleUs;
    uint32_t trips = 0;
    for (uint32_t i = 0; i < kTenSeconds; ++i) {
        const int32_t burst = i % kEvery < kBurst ? bl::kDefaultHomeTripMa * 5 / 2 : 0;
        trips += home.sample(burst + maOfPin(noise(kPinNoiseMv))) ? 1 : 0;
    }
    CHECK(trips == 0);
    // The same burst on the raw sample, as running judges it, trips.
    bl::TripDetector raw;
    bool tripped = false;
    for (uint32_t i = 0; i < kBurst; ++i) tripped = raw.sample(bl::kDefaultHomeTripMa * 5 / 2, bl::kDefaultHomeTripMa);
    CHECK(tripped);
}

TEST_CASE("home trip: running judges the raw sample, homing the mean, an unfilled mean never trips") {
    CHECK(bl::tripInput(bl::Mode::running, 5000, 100) == 5000);
    CHECK(bl::tripInput(bl::Mode::idle, 5000, 100) == 5000);
    CHECK(bl::tripInput(bl::Mode::homing, 5000, 100) == 100);
    CHECK(bl::tripInput(bl::Mode::homing, 5000, std::nullopt) == bl::kNoCurrent);
    bl::TripDetector t;
    for (int i = 0; i < 10; ++i) CHECK_FALSE(t.sample(bl::tripInput(bl::Mode::homing, 30000, std::nullopt), 2970));
    bl::Boxcar<4> b;
    CHECK_FALSE(b.push(4).has_value());
    CHECK_FALSE(b.push(8).has_value());
    CHECK_FALSE(b.push(12).has_value());
    CHECK(b.push(16) == 10);
    CHECK(b.push(20) == 14);   // the oldest left
    b.reset();
    CHECK_FALSE(b.push(1).has_value());
}

TEST_CASE("home trip: measured from the rest current read before the home, never 0") {
    CHECK(valence::homeTripFrom(2970, bl::kNoCurrent) == 2970);
    CHECK(valence::homeTripFrom(2970, 163) == 3133);   // the field's rest readings
    CHECK(valence::homeTripFrom(2970, -138) == 2832);
    CHECK(valence::homeTripFrom(100, -200) == 1);
    CHECK(valence::homeTripFrom(65000, 1000) == 65535);
    CHECK(valence::kRestFrames * bl::kStatusPeriodUs == 128000);
}

TEST_CASE("the P4 sends the link's default thresholds") {
    CHECK(valence::kHomeTripMa == bl::kDefaultHomeTripMa);
    CHECK(valence::kRunTripMa == bl::kDefaultRunTripMa);
    CHECK(valence::kHomeTripMa < valence::kRunTripMa);
}
