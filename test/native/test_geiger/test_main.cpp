// Geiger — hardware-free core suite. Exercises the ring/sink/floor/drop
// contract with an injected port (manual clock, counting lock) — the same
// determinism posture as the valence suites.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <string>
#include <vector>

#include "geiger/geiger_core.hpp"

using namespace geiger;

namespace {

struct FakePort final : IPort {
    uint32_t ms = 0;
    uint8_t core = 0;
    int lockDepth = 0;
    int maxDepth = 0;

    uint32_t nowMs() override { return ms; }
    uint8_t coreId() override { return core; }
    void lock() override {
        ++lockDepth;
        if (lockDepth > maxDepth) maxDepth = lockDepth;
    }
    void unlock() override { --lockDepth; }
};

struct CaptureSink final : ISink {
    std::vector<Record> got;
    void write(const Record& r) override { got.push_back(r); }
};

}  // namespace

TEST_CASE("logf formats, stamps, and drains in order") {
    FakePort port;
    LogCore<16> log(port);
    CaptureSink sink;
    REQUIRE(log.addSink(&sink));

    port.ms = 1234;
    port.core = 1;
    log.logf(Level::Info, "wifi", "connected ch%d rssi %d", 6, -52);
    port.ms = 1300;
    port.core = 0;
    log.logf(Level::Warn, "arbiter", "rejected: %s", "not homed");

    CHECK(log.pending() == 2);
    CHECK(log.drain() == 2);
    CHECK(log.pending() == 0);
    REQUIRE(sink.got.size() == 2);

    CHECK(sink.got[0].ms == 1234);
    CHECK(sink.got[0].core == 1);
    CHECK(sink.got[0].level == Level::Info);
    CHECK(std::string(sink.got[0].tag) == "wifi");
    CHECK(std::string(sink.got[0].msg) == "connected ch6 rssi -52");

    CHECK(sink.got[1].level == Level::Warn);
    CHECK(std::string(sink.got[1].msg) == "rejected: not homed");

    CHECK(port.lockDepth == 0);   // every lock released
    CHECK(port.maxDepth == 1);    // never nested
}

TEST_CASE("floor rejects below, runtime-adjustable") {
    FakePort port;
    LogCore<16> log(port, Level::Warn);
    CaptureSink sink;
    log.addSink(&sink);

    log.logf(Level::Info, "t", "dropped");
    log.logf(Level::Warn, "t", "kept");
    log.setFloor(Level::Trace);
    log.logf(Level::Debug, "t", "kept now");

    log.drain();
    REQUIRE(sink.got.size() == 2);
    CHECK(std::string(sink.got[0].msg) == "kept");
    CHECK(std::string(sink.got[1].msg) == "kept now");
}

TEST_CASE("tag and message truncate at their bounds, always NUL-terminated") {
    FakePort port;
    LogCore<16> log(port);
    CaptureSink sink;
    log.addSink(&sink);

    std::string longTag(40, 'T');
    std::string longMsg(300, 'M');
    log.logf(Level::Info, longTag.c_str(), "%s", longMsg.c_str());
    log.drain();

    REQUIRE(sink.got.size() == 1);
    CHECK(std::string(sink.got[0].tag).size() == Record::kTagBytes - 1);
    CHECK(std::string(sink.got[0].msg).size() == Record::kMsgBytes - 1);
}

TEST_CASE("Warn+ overflow drops oldest and accounts the loss") {
    FakePort port;
    LogCore<8> log(port);   // HighReserve = 8/4 = 2, low capacity = 6
    CaptureSink sink;
    log.addSink(&sink);

    // Warn+ is the reserved class: it evicts rather than being refused, so
    // this is the classic drop-oldest FIFO behavior.
    for (int i = 0; i < 11; ++i) log.logf(Level::Warn, "t", "m%d", i);

    // 8 slots, 11 pushes: m0..m2 dropped, m3..m10 survive.
    CHECK(log.pending() == 8);
    CHECK(log.totalLost() == 3);
    CHECK(log.droppedAtLevel(Level::Warn) == 3);
    CHECK(log.droppedHigh() == 3);
    CHECK(log.droppedLow() == 0);
    log.drain();
    REQUIRE(sink.got.size() == 8);
    CHECK(std::string(sink.got[0].msg) == "m3");
    CHECK(std::string(sink.got[7].msg) == "m10");
    // The loss count rides on a surviving record so a reader can see the gap.
    CHECK(sink.got.back().lost == 3);

    // After drain the ring is clean and losses don't leak into new records.
    log.logf(Level::Warn, "t", "fresh");
    log.drain();
    CHECK(sink.got.back().lost == 0);
    CHECK(log.totalLost() == 3);  // lifetime counter unaffected by drain
}

// ---------------------------------------------------------------------------
// Severity-aware retention — the reason this ring is not a plain FIFO.
// The failure mode under test: a flood of Debug/Trace evicting the Error that
// actually explains what went wrong, destroying the evidence exactly when it
// matters most.
// ---------------------------------------------------------------------------

TEST_CASE("THE headline case: a Debug flood cannot evict an Error") {
    FakePort port;
    LogCore<16> log(port);   // HighReserve = 4, low capacity = 12
    CaptureSink sink;
    log.addSink(&sink);

    log.logf(Level::Error, "motor", "STALL at %d steps", 4211);

    // Ten thousand lines of "doing the same thing as before".
    for (int i = 0; i < 10000; ++i) log.logf(Level::Debug, "tick", "same as before %d", i);

    // The Error is still there, still first, still readable.
    log.drain();
    REQUIRE(sink.got.size() >= 1);
    CHECK(sink.got[0].level == Level::Error);
    CHECK(std::string(sink.got[0].msg) == "STALL at 4211 steps");

    // Debug filled exactly its share and no more; the rest was shed at the door.
    CHECK(sink.got.size() == 1 + LogCore<16>::kLowCapacity);
    CHECK(log.droppedAtLevel(Level::Debug) == 10000 - LogCore<16>::kLowCapacity);
    CHECK(log.droppedAtLevel(Level::Error) == 0);
    CHECK(log.droppedHigh() == 0);
}

TEST_CASE("the Warn+ reserve stays available under a saturating low flood") {
    FakePort port;
    LogCore<16> log(port);
    CaptureSink sink;
    log.addSink(&sink);

    // Saturate the low class, then keep hammering.
    for (int i = 0; i < 200; ++i) log.logf(Level::Info, "spam", "i%d", i);
    CHECK(log.lowPending() == LogCore<16>::kLowCapacity);
    CHECK(log.droppedAtLevel(Level::Info) == 200 - LogCore<16>::kLowCapacity);

    // Every one of the reserved slots is still free for Warn+.
    for (int i = 0; i < int(LogCore<16>::kHighReserve); ++i)
        log.logf(Level::Fatal, "boom", "f%d", i);
    CHECK(log.droppedAtLevel(Level::Fatal) == 0);
    CHECK(log.pending() == 16);   // ring is now genuinely full

    // A full ring still refuses low records rather than evicting anything...
    log.logf(Level::Info, "spam", "one more");
    CHECK(log.droppedAtLevel(Level::Info) == 200 - LogCore<16>::kLowCapacity + 1);
    CHECK(log.droppedAtLevel(Level::Fatal) == 0);

    // ...but Warn+ makes room, and the slot it takes comes from the low class.
    log.logf(Level::Fatal, "boom", "overflow");
    CHECK(log.droppedAtLevel(Level::Info) == 200 - LogCore<16>::kLowCapacity + 2);
    CHECK(log.droppedAtLevel(Level::Fatal) == 0);
    CHECK(log.lowPending() == LogCore<16>::kLowCapacity - 1);

    log.drain();
    size_t fatals = 0;
    for (const Record& r : sink.got)
        if (r.level == Level::Fatal) ++fatals;
    CHECK(fatals == LogCore<16>::kHighReserve + 1);
    CHECK(std::string(sink.got.back().msg) == "overflow");
}

TEST_CASE("drop counters are accurate per level and format for display") {
    FakePort port;
    LogCore<16> log(port, Level::Trace);
    CaptureSink sink;
    log.addSink(&sink);

    // 12 low slots. Trace and Debug share them, so the split of the drops is
    // decided by arrival order: 6 Trace + 6 Debug land, the rest are shed.
    for (int i = 0; i < 6; ++i) log.logf(Level::Trace, "t", "tr%d", i);
    for (int i = 0; i < 6; ++i) log.logf(Level::Debug, "t", "db%d", i);
    CHECK(log.lowPending() == 12);

    for (int i = 0; i < 7; ++i) log.logf(Level::Trace, "t", "extra");
    for (int i = 0; i < 3; ++i) log.logf(Level::Debug, "t", "extra");
    for (int i = 0; i < 5; ++i) log.logf(Level::Info, "t", "extra");

    CHECK(log.droppedAtLevel(Level::Trace) == 7);
    CHECK(log.droppedAtLevel(Level::Debug) == 3);
    CHECK(log.droppedAtLevel(Level::Info) == 5);
    CHECK(log.droppedAtLevel(Level::Warn) == 0);
    CHECK(log.droppedLow() == 15);
    CHECK(log.droppedHigh() == 0);
    CHECK(log.totalLost() == 15);

    char buf[64];
    CHECK(log.formatDropSummary(buf, sizeof(buf)) > 0);
    CHECK(std::string(buf) == "T:7 D:3 I:5 W:0 E:0 F:0");

    // A buffer that cannot hold the whole summary reports failure rather than
    // handing back a half-truth.
    char tiny[4];
    CHECK(log.formatDropSummary(tiny, sizeof(tiny)) == 0);
    CHECK(std::string(tiny).empty());
}

TEST_CASE("the shed count rides out on the next record that survives") {
    FakePort port;
    LogCore<16> log(port);
    CaptureSink sink;
    log.addSink(&sink);

    for (int i = 0; i < 12; ++i) log.logf(Level::Debug, "t", "d%d", i);   // fills low cap
    for (int i = 0; i < 5; ++i) log.logf(Level::Debug, "t", "shed");      // dropped-newest

    // The Warn that follows the hole admits how big the hole was.
    log.logf(Level::Warn, "t", "after the gap");
    log.drain();
    REQUIRE(sink.got.size() == 13);
    CHECK(sink.got.back().level == Level::Warn);
    CHECK(sink.got.back().lost == 5);
}

TEST_CASE("the ring recovers to a clean state once the flood stops") {
    FakePort port;
    LogCore<16> log(port);
    CaptureSink sink;
    log.addSink(&sink);

    for (int i = 0; i < 5000; ++i) log.logf(Level::Debug, "flood", "d%d", i);
    CHECK(log.lowPending() == LogCore<16>::kLowCapacity);

    // Draining releases the low-class credit — the accounting is not a
    // one-way latch that would wedge the ring shut after one burst.
    CHECK(log.drain() == LogCore<16>::kLowCapacity);
    CHECK(log.pending() == 0);
    CHECK(log.lowPending() == 0);

    // ...and the ring accepts a full low quota again immediately.
    const uint32_t droppedBefore = log.droppedAtLevel(Level::Debug);
    for (size_t i = 0; i < LogCore<16>::kLowCapacity; ++i) log.logf(Level::Debug, "t", "again");
    CHECK(log.pending() == LogCore<16>::kLowCapacity);
    CHECK(log.droppedAtLevel(Level::Debug) == droppedBefore);   // nothing shed
}

TEST_CASE("the reserve boundary sits at Warn, matching every ring in the chain") {
    CHECK(kReserveFloor == Level::Warn);
    CHECK_FALSE(isReserved(Level::Trace));
    CHECK_FALSE(isReserved(Level::Debug));
    CHECK_FALSE(isReserved(Level::Info));
    CHECK(isReserved(Level::Warn));
    CHECK(isReserved(Level::Error));
    CHECK(isReserved(Level::Fatal));

    // Default split: a quarter of the ring is reserved for Warn+.
    CHECK(LogCore<64>::kHighReserve == 16);
    CHECK(LogCore<64>::kLowCapacity == 48);
    // ...and it is tunable per instance without touching the ring size.
    CHECK((LogCore<32, 4, 8>::kLowCapacity) == 24);
}

TEST_CASE("drain honors maxRecords and multiple sinks see every record") {
    FakePort port;
    LogCore<16> log(port);
    CaptureSink a, b;
    log.addSink(&a);
    log.addSink(&b);

    for (int i = 0; i < 6; ++i) log.logf(Level::Info, "t", "m%d", i);
    CHECK(log.drain(4) == 4);
    CHECK(a.got.size() == 4);
    CHECK(log.drain() == 2);
    CHECK(a.got.size() == 6);
    CHECK(b.got.size() == 6);
}

TEST_CASE("null and overflow sink registration refused") {
    FakePort port;
    LogCore<16, 2> log(port);
    CaptureSink s1, s2, s3;
    CHECK_FALSE(log.addSink(nullptr));
    CHECK(log.addSink(&s1));
    CHECK(log.addSink(&s2));
    CHECK_FALSE(log.addSink(&s3));  // MaxSinks = 2
}

TEST_CASE("per-sink floor filters and can be retuned live (the serial handoff)") {
    FakePort port;
    LogCore<16> log(port);
    CaptureSink web, serial;
    log.addSink(&web);                          // full stream
    log.addSink(&serial, Level::Trace);         // full stream at boot...

    log.logf(Level::Info, "t", "boot line");
    log.drain();
    CHECK(web.got.size() == 1);
    CHECK(serial.got.size() == 1);

    // ...then the UI handshakes: serial demotes to Warn+.
    CHECK(log.setSinkFloor(&serial, Level::Warn));
    log.logf(Level::Info, "t", "web only");
    log.logf(Level::Warn, "t", "both");
    log.drain();
    CHECK(web.got.size() == 3);
    REQUIRE(serial.got.size() == 2);
    CHECK(std::string(serial.got[1].msg) == "both");

    CaptureSink unregistered;
    CHECK_FALSE(log.setSinkFloor(&unregistered, Level::Warn));  // unknown sink refused
}

TEST_CASE("immediate-drain boot mode flushes synchronously, then hands off") {
    FakePort port;
    LogCore<16> log(port);
    CaptureSink sink;
    log.addSink(&sink);

    log.setImmediateDrain(true);
    log.logf(Level::Info, "boot", "line 1");
    CHECK(sink.got.size() == 1);       // no drain() call needed — synchronous
    CHECK(log.pending() == 0);

    log.setImmediateDrain(false);      // tasks are about to spawn
    log.logf(Level::Info, "run", "line 2");
    CHECK(sink.got.size() == 1);       // buffered now...
    CHECK(log.pending() == 1);
    log.drain();
    CHECK(sink.got.size() == 2);       // ...until the task drain runs
}
