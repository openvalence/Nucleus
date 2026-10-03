// test_drive_link -- native doctest suite for the drive link's cores: the
// Modbus RTU master and the AIM drive model
// Constraints:
// - Hardware-free: ModbusRtu.h and AimDrive.h against a fake port. The UART,
//   the pins and the task are ValenceDriveLink.cpp's and are bench work
//   (val-091.69).
// - Frame vectors are the CRC catalog's check value, the serial-line spec's
//   worked example, and the YZ-AIM manual's own hex frames (v2.55 pp. 13-17),
//   byte for byte.
// - Time is a manual millisecond clock stepped like the host's 5 ms pass.
// See: flagship_p4/src/system/ModbusRtu.h, flagship_p4/src/system/AimDrive.h,
// bd val-091.29

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <algorithm>
#include <cstring>
#include <deque>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "../../../flagship_p4/src/system/AimDrive.h"
#include "../../../flagship_p4/src/system/ModbusRtu.h"

namespace modbus = valence::modbus;
namespace aim = valence::aim;
using Bytes = std::vector<uint8_t>;

namespace {

constexpr uint32_t kPassMs = 5;   // the host task's period

Bytes hex(std::initializer_list<int> v) {
    Bytes b;
    for (int x : v) b.push_back(uint8_t(x));
    return b;
}

Bytes withCrc(Bytes b) {
    const uint16_t c = modbus::crc16(b);
    b.push_back(uint8_t(c));
    b.push_back(uint8_t(c >> 8));
    return b;
}

Bytes encoded(const modbus::Request& r) {
    std::array<uint8_t, modbus::kMaxFrameBytes> out{};
    const size_t n = modbus::encode(r, out);
    return Bytes(out.begin(), out.begin() + long(n));
}

// The UART as the master sees it. A responder answers each written frame;
// `chunk` caps bytes per read so a reply can arrive across passes.
struct FakePort final : modbus::IPort {
    uint32_t baud = 0;
    int discards = 0;
    size_t chunk = 0;
    std::vector<Bytes> written;
    std::deque<uint8_t> rx;
    std::function<Bytes(const Bytes&)> responder;

    void setBaud(uint32_t b) override { baud = b; }
    void discardInput() override {
        rx.clear();
        ++discards;
    }
    void write(std::span<const uint8_t> f) override {
        written.emplace_back(f.begin(), f.end());
        if (responder)
            for (uint8_t b : responder(written.back())) rx.push_back(b);
    }
    size_t read(std::span<uint8_t> into) override {
        size_t n = std::min(into.size(), rx.size());
        if (chunk != 0) n = std::min(n, chunk);
        for (size_t i = 0; i < n; ++i) {
            into[i] = rx.front();
            rx.pop_front();
        }
        return n;
    }
};

// Runs one transaction to its result, 5 ms a pass. nullopt past maxMs.
std::optional<modbus::Result> transact(modbus::Master& m, const modbus::Request& r, uint32_t& nowMs,
                                       uint32_t maxMs = 1000) {
    const uint32_t until = nowMs + maxMs;
    while (!m.start(r, nowMs)) {
        nowMs += 1;
        if (nowMs > until) return std::nullopt;
    }
    while (nowMs <= until) {
        nowMs += kPassMs;
        if (auto res = m.poll(nowMs)) return res;
    }
    return std::nullopt;
}

}  // namespace

// ---- CRC and frames ---------------------------------------------------------------

TEST_CASE("CRC-16/MODBUS: the CRC catalog's check value and the serial-line spec example") {
    const char* check = "123456789";
    const auto* p = reinterpret_cast<const uint8_t*>(check);
    CHECK(modbus::crc16(std::span<const uint8_t>(p, 9)) == 0x4B37);
    // Modbus over Serial Line V1.02 section 6.2.2: 02 07 sends CRC 41 12.
    const Bytes spec = hex({0x02, 0x07});
    CHECK(modbus::crc16(spec) == 0x1241);
    static_assert(modbus::crc16(std::span<const uint8_t>()) == 0xFFFF, "an empty message is the seed");
}

TEST_CASE("encode FC 0x03: the manual's read frames") {
    CHECK(encoded(modbus::readHolding(1, 0x0000, 1)) == hex({0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A}));
    CHECK(encoded(modbus::readHolding(1, aim::reg::position_low, 2)) ==
          hex({0x01, 0x03, 0x00, 0x16, 0x00, 0x02, 0x25, 0xCF}));
}

TEST_CASE("encode FC 0x06: the manual's write frames") {
    CHECK(encoded(modbus::writeSingle(1, aim::reg::modbus_enable, 1)) ==
          hex({0x01, 0x06, 0x00, 0x00, 0x00, 0x01, 0x48, 0x0A}));
    CHECK(encoded(modbus::writeSingle(1, aim::reg::save_flag, 1)) ==
          hex({0x01, 0x06, 0x00, 0x14, 0x00, 0x01, 0x08, 0x0E}));
    CHECK(encoded(modbus::writeSingle(1, aim::reg::acceleration, 5000)) ==
          hex({0x01, 0x06, 0x00, 0x03, 0x13, 0x88, 0x74, 0x9C}));
}

TEST_CASE("encode FC 0x10: the manual's two-register frames, low word first") {
    const std::array<uint16_t, 2> plus4000{0x0FA0, 0x0000};
    CHECK(encoded(modbus::writeMultiple(1, aim::reg::to_go_low, plus4000)) ==
          hex({0x01, 0x10, 0x00, 0x0C, 0x00, 0x02, 0x04, 0x0F, 0xA0, 0x00, 0x00, 0xF0, 0xCC}));
    const std::array<uint16_t, 2> minus4000{0xF060, 0xFFFF};
    CHECK(encoded(modbus::writeMultiple(1, aim::reg::to_go_low, minus4000)) ==
          hex({0x01, 0x10, 0x00, 0x0C, 0x00, 0x02, 0x04, 0xF0, 0x60, 0xFF, 0xFF, 0xC1, 0x54}));
    const std::array<uint16_t, 2> plus8000{0x1F40, 0x0000};
    CHECK(encoded(modbus::writeMultiple(1, aim::reg::position_low, plus8000)) ==
          hex({0x01, 0x10, 0x00, 0x16, 0x00, 0x02, 0x04, 0x1F, 0x40, 0x00, 0x00, 0x74, 0x89}));
    CHECK(aim::joinWords(0xF060, 0xFFFF) == -4000);
}

TEST_CASE("encode refuses broadcast, reserved addresses and bad counts") {
    CHECK(encoded(modbus::readHolding(0, 0, 1)).empty());
    CHECK(encoded(modbus::readHolding(248, 0, 1)).empty());
    CHECK(encoded(modbus::readHolding(1, 0, 0)).empty());
    CHECK(encoded(modbus::readHolding(1, 0, modbus::kMaxRegs + 1)).empty());
    const std::array<uint16_t, modbus::kMaxRegs + 1> tooMany{};
    CHECK(encoded(modbus::writeMultiple(1, 0, tooMany)).empty());
    CHECK(encoded(modbus::readHolding(1, 0, modbus::kMaxRegs)).size() == 8);
}

TEST_CASE("decode: the manual's replies, an exception, a broken CRC, a stranger") {
    // p. 13: one register reading 1.
    modbus::Result r = modbus::decode(modbus::readHolding(1, 0, 1), hex({0x01, 0x03, 0x02, 0x00, 0x01, 0x79, 0x84}));
    CHECK(r.status == modbus::Status::ok);
    CHECK(r.count == 1);
    CHECK(r.regs[0] == 1);
    // p. 15: the FC 0x10 echo of register and count.
    const std::array<uint16_t, 2> w{0x0FA0, 0};
    r = modbus::decode(modbus::writeMultiple(1, 0x0C, w), hex({0x01, 0x10, 0x00, 0x0C, 0x00, 0x02, 0x81, 0xCB}));
    CHECK(r.status == modbus::Status::ok);
    // p. 14: FC 0x06 echoes the request.
    const modbus::Request one = modbus::writeSingle(1, 0, 1);
    CHECK(modbus::decode(one, encoded(one)).status == modbus::Status::ok);
    CHECK(modbus::decode(one, withCrc(hex({0x01, 0x06, 0x00, 0x00, 0x00, 0x02}))).status ==
          modbus::Status::malformed);
    // Exception 2, illegal data address.
    r = modbus::decode(modbus::readHolding(1, 0x15, 1), hex({0x01, 0x83, 0x02, 0xC0, 0xF1}));
    CHECK(r.status == modbus::Status::exception);
    CHECK(r.exception == 2);
    // One bit off in the CRC.
    CHECK(modbus::decode(modbus::readHolding(1, 0, 1), hex({0x01, 0x03, 0x02, 0x00, 0x01, 0x79, 0x85})).status ==
          modbus::Status::crc);
    // A valid frame from address 2.
    CHECK(modbus::decode(modbus::readHolding(1, 0, 1), withCrc(hex({0x02, 0x03, 0x02, 0x00, 0x01}))).status ==
          modbus::Status::malformed);
    // A byte count that does not match the request.
    CHECK(modbus::decode(modbus::readHolding(1, 0, 1), withCrc(hex({0x01, 0x03, 0x04, 0x00, 0x01}))).status ==
          modbus::Status::malformed);
}

TEST_CASE("line timing: wire time rounds up, the gap is 3.5 characters or 1.75 ms") {
    CHECK(modbus::wireMs(15, 19200) == 9);
    CHECK(modbus::gapMs(9600) == 6);
    CHECK(modbus::gapMs(19200) == 4);
    CHECK(modbus::gapMs(115200) == 3);
}

// ---- the master over a fake UART ------------------------------------------------

TEST_CASE("master: a good frame completes in one attempt") {
    FakePort port;
    port.responder = [](const Bytes&) { return withCrc(hex({0x01, 0x03, 0x02, 0x02, 0x58})); };
    modbus::Master m(port);
    REQUIRE(m.setBaud(19200));
    CHECK(port.baud == 19200);
    uint32_t now = 1000;
    const auto r = transact(m, modbus::readHolding(1, aim::reg::standstill, 1), now);
    REQUIRE(r.has_value());
    CHECK(r->status == modbus::Status::ok);
    CHECK(r->attempts == 1);
    CHECK(r->regs[0] == 600);
    CHECK(port.written.size() == 1);
    CHECK(port.discards == 1);   // stale input dropped before the frame went out
    CHECK_FALSE(m.busy());
}

TEST_CASE("master: CRC errors are retried, then reported as crc") {
    FakePort port;
    port.responder = [](const Bytes&) {
        Bytes b = withCrc(hex({0x01, 0x03, 0x02, 0x00, 0x01}));
        b.back() ^= 0x01;
        return b;
    };
    modbus::Master m(port);
    m.setBaud(19200);
    uint32_t now = 0;
    const auto r = transact(m, modbus::readHolding(1, 0, 1), now);
    REQUIRE(r.has_value());
    CHECK(r->status == modbus::Status::crc);
    CHECK(r->attempts == 1 + modbus::kRetries);
    CHECK(port.written.size() == 1 + modbus::kRetries);
}

TEST_CASE("master: a CRC error then a good frame is ok on the second attempt") {
    FakePort port;
    int n = 0;
    port.responder = [&n](const Bytes&) {
        Bytes b = withCrc(hex({0x01, 0x03, 0x02, 0x00, 0x07}));
        if (n++ == 0) b[3] ^= 0xFF;
        return b;
    };
    modbus::Master m(port);
    m.setBaud(19200);
    uint32_t now = 0;
    const auto r = transact(m, modbus::readHolding(1, 0, 1), now);
    REQUIRE(r.has_value());
    CHECK(r->status == modbus::Status::ok);
    CHECK(r->attempts == 2);
    CHECK(r->regs[0] == 7);
}

TEST_CASE("master: silence times out after every attempt, each past its deadline and the gap") {
    FakePort port;
    modbus::Master m(port);
    m.setBaud(19200);
    uint32_t now = 0;
    const uint32_t t0 = now;
    const auto r = transact(m, modbus::readHolding(1, 0, 1), now);
    REQUIRE(r.has_value());
    CHECK(r->status == modbus::Status::timeout);
    CHECK(r->attempts == 1 + modbus::kRetries);
    const uint32_t deadline = modbus::wireMs(8 + 7, 19200) + modbus::kReplyLatencyMs;
    CHECK(now - t0 >= (1 + modbus::kRetries) * deadline);
    // The next request waits out the gap after the last deadline.
    CHECK_FALSE(m.start(modbus::readHolding(1, 0, 1), now));
    CHECK(m.start(modbus::readHolding(1, 0, 1), now + modbus::gapMs(19200)));
}

TEST_CASE("master: an exception is an answer, never retried") {
    FakePort port;
    port.responder = [](const Bytes&) { return withCrc(hex({0x01, 0x83, 0x02})); };
    modbus::Master m(port);
    m.setBaud(19200);
    uint32_t now = 0;
    const auto r = transact(m, modbus::readHolding(1, 0x15, 1), now);
    REQUIRE(r.has_value());
    CHECK(r->status == modbus::Status::exception);
    CHECK(r->exception == 2);
    CHECK(r->attempts == 1);
    CHECK(port.written.size() == 1);
}

TEST_CASE("master: a reply across several reads, after a turnaround glitch byte") {
    FakePort port;
    port.chunk = 2;
    port.responder = [](const Bytes&) {
        Bytes b = hex({0x00});
        const Bytes reply = withCrc(hex({0x01, 0x03, 0x04, 0x12, 0x34, 0xFF, 0xFE}));
        b.insert(b.end(), reply.begin(), reply.end());
        return b;
    };
    modbus::Master m(port);
    m.setBaud(115200);
    uint32_t now = 0;
    const auto r = transact(m, modbus::readHolding(1, aim::reg::position_low, 2), now);
    REQUIRE(r.has_value());
    CHECK(r->status == modbus::Status::ok);
    CHECK(aim::joinWords(r->regs[0], r->regs[1]) == aim::joinWords(0x1234, 0xFFFE));
    CHECK(m.glitches() == 1);
}

TEST_CASE("master: a reply cut short at the deadline is malformed") {
    FakePort port;
    port.responder = [](const Bytes&) { return hex({0x01, 0x03, 0x02}); };
    modbus::Master m(port);
    m.setBaud(19200);
    uint32_t now = 0;
    const auto r = transact(m, modbus::readHolding(1, 0, 1), now);
    REQUIRE(r.has_value());
    CHECK(r->status == modbus::Status::malformed);
}

TEST_CASE("master: one transaction at a time; abort drops it and keeps the gap") {
    FakePort port;
    modbus::Master m(port);
    m.setBaud(19200);
    CHECK(m.start(modbus::readHolding(1, 0, 1), 0));
    CHECK_FALSE(m.start(modbus::readHolding(1, 1, 1), 0));
    CHECK_FALSE(m.setBaud(9600));
    m.abort(10);
    CHECK_FALSE(m.busy());
    CHECK_FALSE(m.poll(20).has_value());
    CHECK_FALSE(m.start(modbus::readHolding(1, 0, 1), 11));
    CHECK(m.start(modbus::readHolding(1, 0, 1), 10 + modbus::gapMs(19200)));
}

// ---- the register map -----------------------------------------------------------

TEST_CASE("register map: 0x00-0x19 in order, each named once, the read-only ones per the manual") {
    std::set<std::string> names;
    for (size_t i = 0; i < aim::kRegisters.size(); ++i) {
        CHECK(aim::kRegisters[i].addr == i);
        names.insert(aim::kRegisters[i].name);
    }
    CHECK(names.size() == aim::kRegisters.size());
    for (uint16_t ro : {aim::reg::to_go_low, aim::reg::to_go_high, aim::reg::alarm_code, aim::reg::current,
                        aim::reg::speed, aim::reg::voltage, aim::reg::temperature, aim::reg::output_pwm,
                        aim::reg::device_address})
        CHECK(aim::kRegisters[ro].access == aim::Access::read_only);
    CHECK(std::string(aim::registerName(aim::reg::device_address)) == "device-address");
    CHECK(std::string(aim::registerName(0x1A)) == "?");
}

TEST_CASE("values: alarm names, the WR polarity digit, the stall digit") {
    CHECK(std::string(aim::alarmName(aim::kAlarmStall)) == "stall");
    CHECK(std::string(aim::alarmName(aim::kAlarmOverVoltage)) == "over-voltage");
    CHECK(std::string(aim::alarmName(0)) == "none");
    CHECK(std::string(aim::alarmName(0x77)) == "unknown");
    CHECK_FALSE(aim::wrNormallyClosed(3000));
    CHECK(aim::wrNormallyClosed(3001));
    CHECK(aim::stallAlarmDigit(600) == 0);
    CHECK(aim::stallAlarmDigit(603) == 3);
}

// ---- the WR alarm and RDY lines -----------------------------------------------------

namespace {

struct Lines {
    aim::DrivePower power;
    aim::StatusLines lines;
    uint32_t now = 0;
    bool powered = false;
    bool alarmLow = false;
    bool readyLow = false;
    int raised = 0;

    void run(uint32_t ms) {
        for (uint32_t end = now + ms; now < end;) {
            now += kPassMs;
            power.sample(powered, now);
            if (lines.sample(alarmLow, readyLow, power, now)) ++raised;
        }
    }
};

}  // namespace

TEST_CASE("alarm: nothing counts until the drive has settled on motor power") {
    Lines l;
    l.alarmLow = true;
    l.run(500);
    CHECK(l.raised == 0);   // unpowered: the opto means nothing
    CHECK(l.lines.alarm());
    l.powered = true;
    l.run(aim::kSettleMs - 10);
    CHECK(l.raised == 0);
    CHECK_FALSE(l.lines.armed());
    l.run(20);
    CHECK(l.lines.armed());
    CHECK(l.raised == 1);   // already asserted when it settled
    CHECK(l.lines.raisedAtArming());
}

TEST_CASE("alarm: a debounced assertion latches once; clearing re-arms it") {
    Lines l;
    l.powered = true;
    l.run(aim::kSettleMs + 50);
    REQUIRE(l.lines.armed());
    l.alarmLow = true;
    l.run(10);
    l.alarmLow = false;   // a 10 ms glitch is not an alarm
    l.run(100);
    CHECK(l.raised == 0);
    l.alarmLow = true;
    l.run(aim::kLineDebounceMs + 2 * kPassMs);
    CHECK(l.raised == 1);
    CHECK_FALSE(l.lines.raisedAtArming());
    l.run(1000);
    CHECK(l.raised == 1);   // held: still one
    l.alarmLow = false;
    l.run(100);
    l.alarmLow = true;
    l.run(100);
    CHECK(l.raised == 2);
    CHECK(l.lines.raised() == 2);
}

TEST_CASE("alarm: motor power loss disarms at once; the next settle re-arms") {
    Lines l;
    l.powered = true;
    l.run(aim::kSettleMs + 50);
    l.powered = false;
    l.run(5);
    CHECK_FALSE(l.lines.armed());
    l.alarmLow = true;   // a browning-out drive's WR
    l.run(200);
    CHECK(l.raised == 0);
    l.alarmLow = false;
    l.powered = true;
    l.run(aim::kSettleMs + 50);
    CHECK(l.lines.armed());
    CHECK(l.raised == 0);
}

TEST_CASE("RDY: debounced and reported, never raised") {
    Lines l;
    l.powered = true;
    l.readyLow = true;
    l.run(aim::kSettleMs + 50);
    CHECK(l.lines.ready());
    l.readyLow = false;
    l.run(10);
    CHECK(l.lines.ready());
    l.run(aim::kLineDebounceMs);
    CHECK_FALSE(l.lines.ready());
    CHECK(l.raised == 0);
}

// ---- the probe and the poll against a fake drive -----------------------------------

namespace {

// An AIM drive on the far end of the fake UART: a register file, a line speed,
// and the faults a probe has to name.
struct FakeDrive {
    uint32_t baud = 19200;
    std::array<uint16_t, 0x1A> regs{};
    bool present = true;
    bool garble = false;
    std::optional<uint16_t> refuse;   // exception 2 for this register
    std::vector<uint16_t> asked;

    FakeDrive() {
        regs[aim::reg::device_address] = 1;
        regs[aim::reg::modbus_enable] = 0;
        regs[aim::reg::output_enable] = 1;
        regs[aim::reg::special_function] = aim::kQuadratureFollow;
        regs[aim::reg::position_kp] = 3000;
        regs[aim::reg::standstill] = 600;
        regs[aim::reg::alarm_code] = 0;
        regs[aim::reg::temperature] = 31;
        setEncoder(-123456);
    }
    void setEncoder(int32_t counts) {
        regs[aim::reg::position_low] = uint16_t(uint32_t(counts) & 0xFFFFu);
        regs[aim::reg::position_high] = uint16_t(uint32_t(counts) >> 16);
    }

    Bytes answer(const Bytes& q, uint32_t lineBaud) {
        if (!present || lineBaud != baud || q.size() < 8) return {};
        const uint16_t reg = uint16_t(q[2] << 8 | q[3]);
        const uint16_t count = uint16_t(q[4] << 8 | q[5]);
        asked.push_back(reg);
        if (refuse && *refuse == reg) return withCrc(hex({q[0], 0x83, 0x02}));
        Bytes b = hex({q[0], 0x03, 2 * count});
        for (size_t i = 0; i < count; ++i) {
            const uint16_t v = reg + i < regs.size() ? regs[reg + i] : 0;
            b.push_back(uint8_t(v >> 8));
            b.push_back(uint8_t(v));
        }
        b = withCrc(b);
        if (garble) b.back() ^= 0x5A;
        return b;
    }
};

struct LinkRig {
    FakeDrive drive;
    FakePort port;
    aim::Link link{port};
    aim::DrivePower power;
    uint32_t now = 0;
    bool powered = true;

    LinkRig() {
        port.responder = [this](const Bytes& q) { return drive.answer(q, port.baud); };
    }
    void run(uint32_t ms, bool alarmRaised = false) {
        for (uint32_t end = now + ms; now < end;) {
            now += kPassMs;
            power.sample(powered, now);
            link.step(now, power, alarmRaised);
            alarmRaised = false;
        }
    }
    const aim::Snapshot& snap() const { return link.snapshot(); }
};

// Long enough for one probe, a full scan of every baud with every retry
// included, and shorter than kReprobeMs, so a window holds at most one probe.
constexpr uint32_t kProbeBudgetMs = 1500;
static_assert(kProbeBudgetMs < aim::kReprobeMs, "a probe window would hold two probes");

}  // namespace

TEST_CASE("link: dark until motor power, quiet while settling") {
    LinkRig rig;
    rig.powered = false;
    rig.run(500);
    CHECK(rig.snap().state == aim::LinkState::unpowered);
    rig.powered = true;
    rig.run(aim::kSettleMs - 10);
    CHECK(rig.snap().state == aim::LinkState::settling);
    CHECK(rig.port.written.empty());
}

TEST_CASE("link: a healthy drive probes ok at the factory baud, then the poll runs") {
    LinkRig rig;
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    const aim::Snapshot& s = rig.snap();
    REQUIRE(s.state == aim::LinkState::up);
    CHECK(s.probe.outcome == aim::ProbeOutcome::ok);
    CHECK(s.probes == 1);
    CHECK(s.baud == 19200);
    CHECK(s.probe.temperatureC == 31);
    CHECK(aim::stallAlarmDigit(s.probe.standstill) == 0);
    CHECK(s.encoderKnown);
    CHECK(s.encoder == -123456);
    // Every probe register was asked, identity first.
    REQUIRE(rig.drive.asked.size() >= aim::kProbeRegs.size());
    CHECK(rig.drive.asked[0] == aim::reg::device_address);
    // The poll keeps the encoder current.
    rig.drive.setEncoder(4242);
    rig.run(4 * aim::kPollEveryMs);
    CHECK(s.encoder == 4242);
    CHECK(s.polls > 0);
    CHECK(s.pollFailures == 0);
}

TEST_CASE("link: a drive reprogrammed to 115200 is found on the scan") {
    LinkRig rig;
    rig.drive.baud = 115200;
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    CHECK(rig.snap().probe.outcome == aim::ProbeOutcome::ok);
    CHECK(rig.snap().baud == 115200);
}

TEST_CASE("link: silence at every baud is no_answer, then a probe every kReprobeMs") {
    LinkRig rig;
    rig.drive.present = false;
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    CHECK(rig.snap().state == aim::LinkState::down);
    CHECK(rig.snap().probe.outcome == aim::ProbeOutcome::no_answer);
    std::set<uint32_t> bauds;
    rig.port.responder = [&](const Bytes&) {
        bauds.insert(rig.port.baud);
        return Bytes{};
    };
    const uint32_t before = rig.snap().probes;
    rig.run(aim::kReprobeMs + kProbeBudgetMs);
    CHECK(rig.snap().probes > before);
    CHECK(bauds.size() == aim::kBauds.size());
    // The drive appears: the next probe finds it.
    rig.drive.present = true;
    rig.port.responder = [&rig](const Bytes& q) { return rig.drive.answer(q, rig.port.baud); };
    rig.run(aim::kReprobeMs + kProbeBudgetMs);
    CHECK(rig.snap().probe.outcome == aim::ProbeOutcome::ok);
    CHECK(rig.snap().state == aim::LinkState::up);
}

TEST_CASE("link: broken frames and silence elsewhere is crc, naming the baud that framed") {
    LinkRig rig;
    rig.drive.garble = true;
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    CHECK(rig.snap().probe.outcome == aim::ProbeOutcome::crc);
    CHECK(rig.snap().probe.baud == 19200);
    CHECK(rig.snap().state == aim::LinkState::down);
}

TEST_CASE("link: an exception on the identity register is wrong_model") {
    LinkRig rig;
    rig.drive.refuse = aim::reg::device_address;
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    const aim::Probe& p = rig.snap().probe;
    CHECK(p.outcome == aim::ProbeOutcome::wrong_model);
    CHECK(p.reg == aim::reg::device_address);
    CHECK(p.exception == 2);
    CHECK(rig.snap().state == aim::LinkState::down);
}

TEST_CASE("link: another address, or a temperature past the map, is wrong_model by value") {
    LinkRig rig;
    rig.drive.regs[aim::reg::device_address] = 7;
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    CHECK(rig.snap().probe.outcome == aim::ProbeOutcome::wrong_model);
    CHECK(rig.snap().probe.value == 7);

    LinkRig hot;
    hot.drive.regs[aim::reg::temperature] = 4000;
    hot.run(aim::kSettleMs + kProbeBudgetMs);
    CHECK(hot.snap().probe.outcome == aim::ProbeOutcome::wrong_model);
    CHECK(hot.snap().probe.reg == aim::reg::temperature);
}

TEST_CASE("link: each configuration the LP core cannot drive is named, and the poll still runs") {
    struct Case {
        uint16_t reg;
        uint16_t value;
        aim::ProbeOutcome outcome;
    };
    for (const Case c : {Case{aim::reg::modbus_enable, 1, aim::ProbeOutcome::modbus_enabled},
                         Case{aim::reg::output_enable, 0, aim::ProbeOutcome::output_off},
                         Case{aim::reg::special_function, 0, aim::ProbeOutcome::not_quadrature},
                         Case{aim::reg::position_kp, 3001, aim::ProbeOutcome::alarm_inverted}}) {
        LinkRig rig;
        rig.drive.regs[c.reg] = c.value;
        rig.run(aim::kSettleMs + kProbeBudgetMs);
        CHECK(rig.snap().probe.outcome == c.outcome);
        CHECK(rig.snap().state == aim::LinkState::up);
    }
}

TEST_CASE("link: silence mid-probe is lost, naming the register") {
    LinkRig rig;
    rig.port.responder = [&rig](const Bytes& q) {
        const uint16_t reg = uint16_t(q[2] << 8 | q[3]);
        return reg == aim::reg::standstill ? Bytes{} : rig.drive.answer(q, rig.port.baud);
    };
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    CHECK(rig.snap().probe.outcome == aim::ProbeOutcome::lost);
    CHECK(rig.snap().probe.reg == aim::reg::standstill);
}

TEST_CASE("link: power lost mid-probe abandons it; the next power-on probes again") {
    LinkRig rig;
    rig.run(aim::kSettleMs + 20);
    REQUIRE(rig.snap().state == aim::LinkState::probing);
    rig.powered = false;
    rig.run(10);
    CHECK(rig.snap().state == aim::LinkState::unpowered);
    CHECK(rig.snap().probes == 0);
    CHECK(rig.snap().probe.outcome == aim::ProbeOutcome::pending);
    rig.powered = true;
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    CHECK(rig.snap().probe.outcome == aim::ProbeOutcome::ok);
}

TEST_CASE("link: a drive gone silent after the probe is lost after kLostAfterFailures polls") {
    LinkRig rig;
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    REQUIRE(rig.snap().state == aim::LinkState::up);
    rig.drive.present = false;
    rig.run(aim::kLostAfterFailures * (aim::kPollEveryMs + 3 * 60) + 1000);
    CHECK(rig.snap().state == aim::LinkState::down);
    CHECK_FALSE(rig.snap().encoderKnown);
    CHECK(rig.snap().pollFailures >= aim::kLostAfterFailures);
}

TEST_CASE("link: a raised alarm reads the alarm code next") {
    LinkRig rig;
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    REQUIRE(rig.snap().state == aim::LinkState::up);
    rig.run(200);   // the poll is mid-rotation
    rig.drive.regs[aim::reg::alarm_code] = aim::kAlarmStall;
    rig.drive.asked.clear();
    rig.run(kPassMs, true);
    rig.run(60);
    REQUIRE_FALSE(rig.drive.asked.empty());
    CHECK(rig.drive.asked.front() == aim::reg::alarm_code);   // ahead of the rotation
    CHECK(rig.snap().alarmCode == aim::kAlarmStall);
}

TEST_CASE("link: an external read is served while up and tagged; writes and a dark link refuse") {
    LinkRig rig;
    CHECK_FALSE(rig.link.canSubmit());
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    REQUIRE(rig.snap().state == aim::LinkState::up);
    CHECK_FALSE(rig.link.submit(modbus::writeSingle(1, aim::reg::modbus_enable, 1), 9));
    REQUIRE(rig.link.submit(modbus::readHolding(1, aim::reg::gear_denominator, 1), 77));
    CHECK_FALSE(rig.link.canSubmit());
    rig.drive.regs[aim::reg::gear_denominator] = 800;
    std::optional<aim::External> got;
    for (int i = 0; i < 100 && !got; ++i) {
        rig.run(kPassMs);
        got = rig.link.takeResult();
    }
    REQUIRE(got.has_value());
    CHECK(got->tag == 77);
    CHECK(got->result.status == modbus::Status::ok);
    CHECK(got->result.regs[0] == 800);
    CHECK_FALSE(rig.link.takeResult().has_value());

    // Pending when the power goes: answered not_sent, never left hanging.
    REQUIRE(rig.link.submit(modbus::readHolding(1, aim::reg::speed_limit, 1), 78));
    rig.powered = false;
    rig.run(kPassMs);
    got = rig.link.takeResult();
    REQUIRE(got.has_value());
    CHECK(got->tag == 78);
    CHECK((got->result.status == modbus::Status::not_sent || got->result.status == modbus::Status::timeout));
}

TEST_CASE("link: reads only, by construction: no transaction it starts is a write") {
    LinkRig rig;
    rig.run(aim::kSettleMs + kProbeBudgetMs);
    rig.run(2000, true);
    for (const Bytes& f : rig.port.written) CHECK(f[1] == modbus::kFcReadHolding);
}
