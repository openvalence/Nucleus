// VLog — hardware-free logging core. No Arduino, no FreeRTOS, no heap in
// steady state: everything platform-specific (time source, critical section,
// core id, sinks) is injected, so this exact code runs on the S3, the C5
// nodes, and inside the native doctest suite.
//
// Shape: producers on ANY task/core format a bounded Record on their own
// stack, then commit it into a fixed-slot ring under a short externally-
// supplied lock (the ~µs copy is the entire critical section). A single
// drain() caller (Core 0 on firmware) pops records and fans them out to
// registered sinks. Overflow sheds records and counts the loss — a logger
// that can block a motion core is worse than no logger.
//
// SEVERITY-AWARE RETENTION (the reason this ring is not a plain FIFO):
// a flood of Debug/Trace must never evict the Error that explains what went
// wrong. So the ring is split by policy, not by storage:
//
//   * LOW  (below kReserveFloor, i.e. Trace/Debug/Info) may occupy at most
//     `Slots - HighReserve` slots and NEVER evicts anything. Past its cap —
//     or with the ring simply full — a low record is dropped-and-counted at
//     the door. Cheap, O(1), no scan.
//   * HIGH (kReserveFloor and above, i.e. Warn/Error/Fatal) always has
//     `HighReserve` slots it cannot be squeezed out of, and on a genuinely
//     full ring falls back to drop-oldest.
//
// The invariant that buys: a HIGH record can only ever be displaced by
// another HIGH record. Low-severity spam is structurally incapable of
// destroying evidence. Every drop is counted per level (droppedAtLevel) so
// the shedding is visible instead of silent.
#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace vlog {

enum class Level : uint8_t { Trace = 0, Debug, Info, Warn, Error, Fatal, Off };

inline const char* levelName(Level l) {
    switch (l) {
        case Level::Trace: return "TRACE";
        case Level::Debug: return "DEBUG";
        case Level::Info:  return "INFO";
        case Level::Warn:  return "WARN";
        case Level::Error: return "ERROR";
        case Level::Fatal: return "FATAL";
        default:           return "?";
    }
}

inline char levelChar(Level l) { return "TDIWEF?"[uint8_t(l) < 6 ? uint8_t(l) : 6]; }

// The severity line that splits "keep at all costs" from "shed under load".
// Warn and above is what an operator needs after the fact; Trace/Debug/Info
// are the running commentary. Shared by every ring in the chain (core ring,
// the Valence bridge ring, the /api/log web ring) — a severity-blind ring
// anywhere would defeat the whole scheme.
inline constexpr Level kReserveFloor = Level::Warn;
inline constexpr bool isReserved(Level l) { return l >= kReserveFloor; }

// Number of distinct real levels (Trace..Fatal) — the width of every
// per-level drop counter array in the chain.
inline constexpr size_t kLevelCount = 6;

// One log record. Fixed-size by design: slots are copied whole under the
// ring lock and never reference caller memory after commit.
struct Record {
    static constexpr size_t kTagBytes = 12;   // includes NUL; longer tags truncate
    static constexpr size_t kMsgBytes = 104;  // includes NUL; longer messages truncate

    uint32_t ms = 0;        // producer-supplied timestamp (platform millis)
    Level level = Level::Info;
    uint8_t core = 0;       // producing core id (0/1; 0xFF = unknown/host)
    uint16_t lost = 0;      // records dropped immediately before this one
    char tag[kTagBytes] = {};
    char msg[kMsgBytes] = {};
};

// Where drained records go. write() is only ever called from drain()'s
// caller (single-threaded fan-out) — sinks need no locking of their own
// against each other. A sink that can block (network, flash) must do its
// own buffering; drain() trusts sinks to return promptly.
class ISink {
public:
    virtual ~ISink() = default;
    virtual void write(const Record& r) = 0;
};

// Injected platform surface: time + core id + the ring's critical section.
// Firmware backs this with millis()/xPortGetCoreID()/portMUX; native tests
// back it with a ManualClock-style fake and a no-op (or std::mutex) lock.
class IPort {
public:
    virtual ~IPort() = default;
    virtual uint32_t nowMs() = 0;
    virtual uint8_t coreId() = 0;
    virtual void lock() = 0;    // must be safe from any task on any core
    virtual void unlock() = 0;
};

// The core: fixed ring of Records + sink registry + severity-aware drop
// accounting.
// Producers: push()/logf() from anywhere (lock held only for the slot copy).
// Consumer: exactly one caller pumps drain() (not enforced — documented).
//
// HighReserve is the number of slots Trace/Debug/Info may never touch. It
// defaults to a quarter of the ring, which on the firmware's 64 slots leaves
// 16 for Warn+ — comfortably more than any single fault cascade this machine
// produces, while still leaving 48 slots of normal breathing room.
template <size_t Slots = 64, size_t MaxSinks = 4, size_t HighReserve = Slots / 4>
class LogCore {
    static_assert(Slots >= 8, "ring too small to absorb a burst");
    static_assert(HighReserve >= 1, "reserve at least one slot for Warn+");
    static_assert(HighReserve < Slots, "reserve must leave room for Debug/Info");

public:
    // Trace/Debug/Info may never hold more than this many slots.
    static constexpr size_t kLowCapacity = Slots - HighReserve;
    static constexpr size_t kHighReserve = HighReserve;

    explicit LogCore(IPort& port, Level floor = Level::Trace)
        : _port(port), _floor(floor) {}

    // Runtime floor — records below it are rejected at push (cheap), on top
    // of whatever compile-time floor the macros already applied.
    void setFloor(Level l) { _floor = l; }
    Level floor() const { return _floor; }

    // Sinks may carry their own floor: a sink at Warn stays registered but
    // only sees Warn+. setSinkFloor() retunes a live sink (e.g. demote the
    // serial sink once the web UI has proven it is receiving logs).
    bool addSink(ISink* s, Level sinkFloor = Level::Trace) {
        if (s == nullptr || _sinkCount >= MaxSinks) return false;
        _sinks[_sinkCount] = s;
        _sinkFloors[_sinkCount] = sinkFloor;
        ++_sinkCount;
        return true;
    }

    bool setSinkFloor(ISink* s, Level sinkFloor) {
        for (size_t i = 0; i < _sinkCount; ++i) {
            if (_sinks[i] == s) {
                _sinkFloors[i] = sinkFloor;
                return true;
            }
        }
        return false;
    }

    // Boot mode: while set, every commit drains synchronously to the sinks.
    // ONLY safe while a single task is running (setup(), before the
    // scheduler spawns other producers) — flip it off before task creation.
    void setImmediateDrain(bool on) { _immediateDrain = on; }

    // Format-and-commit. Bounded: one vsnprintf into a stack Record, one
    // locked copy. Truncation is silent and fine (kMsgBytes is the contract).
    void logf(Level level, const char* tag, const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vlogf(level, tag, fmt, ap);
        va_end(ap);
    }

    void vlogf(Level level, const char* tag, const char* fmt, va_list ap) {
        if (level < _floor || level >= Level::Off) return;
        Record r;
        r.ms = _port.nowMs();
        r.level = level;
        r.core = _port.coreId();
        copyBounded(r.tag, tag, Record::kTagBytes);
        vsnprintf(r.msg, Record::kMsgBytes, fmt, ap);
        push(r);
    }

    // Commit a pre-built record (producers that format their own).
    //
    // Two shedding policies, picked by severity — see the header comment:
    //   LOW  : refuse the NEWEST record (never evicts, capped at kLowCapacity)
    //   HIGH : evict the OLDEST record (which the low cap guarantees can only
    //          ever be another HIGH once lows are at their ceiling)
    // Both count the loss per level. Everything here is O(1) and touches no
    // memory outside the fixed arrays: no scan, no allocation, no blocking.
    void push(const Record& r) {
        if (r.level < _floor || r.level >= Level::Off) return;
        const bool high = isReserved(r.level);
        _port.lock();
        const bool full = (_write - _read) >= Slots;
        if (!high) {
            if (full || _lowPending >= kLowCapacity) {
                dropLocked(r.level);   // drop-newest: low severity never evicts
                _port.unlock();
                return;
            }
        } else if (full) {
            const Record& victim = _ring[_read % Slots];
            if (!isReserved(victim.level) && _lowPending > 0) --_lowPending;
            dropLocked(victim.level);  // drop-oldest
            _read++;
        }
        Record& slot = _ring[_write % Slots];
        slot = r;
        slot.lost = _lostSinceDrain;  // rides on the next record a reader sees
        _write++;
        if (!high) ++_lowPending;
        _port.unlock();
        if (_immediateDrain) drain();
    }

    // Fan out up to `maxRecords` pending records to every sink. Returns how
    // many were written. Single-consumer by contract.
    size_t drain(size_t maxRecords = Slots) {
        size_t n = 0;
        while (n < maxRecords) {
            Record r;
            _port.lock();
            if (_read == _write) {
                _port.unlock();
                break;
            }
            r = _ring[_read % Slots];
            _read++;
            if (!isReserved(r.level) && _lowPending > 0) --_lowPending;
            _lostSinceDrain = 0;
            _port.unlock();
            for (size_t i = 0; i < _sinkCount; ++i) {
                if (r.level >= _sinkFloors[i]) _sinks[i]->write(r);
            }
            ++n;
        }
        return n;
    }

    // Lifetime drop count (records lost to overflow, ever, all levels).
    uint32_t totalLost() const { return _totalLostShadow; }
    size_t pending() const {
        // Racy read is fine: diagnostic only.
        return size_t(_write - _read);
    }
    // How many Trace/Debug/Info records are currently parked in the ring —
    // the number the low cap governs. Diagnostic; racy read is fine.
    size_t lowPending() const { return _lowPending; }

    // ---- Honest visibility --------------------------------------------------
    // Per-level lifetime drop counts. A silent drop is the one thing this
    // logger refuses to do, so every shed record lands in exactly one of
    // these six buckets and stays there.
    uint32_t droppedAtLevel(Level l) const {
        return uint8_t(l) < kLevelCount ? _dropped[uint8_t(l)] : 0;
    }
    uint32_t droppedLow() const {
        uint32_t n = 0;
        for (size_t i = 0; i < kLevelCount; ++i)
            if (!isReserved(Level(i))) n += _dropped[i];
        return n;
    }
    uint32_t droppedHigh() const {
        uint32_t n = 0;
        for (size_t i = 0; i < kLevelCount; ++i)
            if (isReserved(Level(i))) n += _dropped[i];
        return n;
    }

    // "T:0 D:1841 I:3 W:0 E:0 F:0" — the compact form every surface uses
    // (/api/log footer, hub-status, the serial banner). Returns strlen(out),
    // or 0 if the buffer could not hold the whole summary.
    size_t formatDropSummary(char* out, size_t cap) const {
        if (out == nullptr || cap == 0) return 0;
        int n = snprintf(out, cap, "T:%lu D:%lu I:%lu W:%lu E:%lu F:%lu",
                         (unsigned long)_dropped[0], (unsigned long)_dropped[1],
                         (unsigned long)_dropped[2], (unsigned long)_dropped[3],
                         (unsigned long)_dropped[4], (unsigned long)_dropped[5]);
        if (n < 0 || size_t(n) >= cap) { out[0] = '\0'; return 0; }
        return size_t(n);
    }

private:
    // Called with the port lock HELD. Accounts one shed record: per-level
    // bucket, lifetime total, and the "lost" marker that rides out on the
    // next record a reader actually sees (saturating — a wrapped gap count
    // would understate the hole, which this counter must never misreport).
    void dropLocked(Level l) {
        if (uint8_t(l) < kLevelCount) ++_dropped[uint8_t(l)];
        if (_lostSinceDrain != 0xFFFFu) ++_lostSinceDrain;
        ++_totalLostShadow;
    }

    static void copyBounded(char* dst, const char* src, size_t cap) {
        if (src == nullptr) { dst[0] = '\0'; return; }
        size_t i = 0;
        for (; i + 1 < cap && src[i] != '\0'; ++i) dst[i] = src[i];
        dst[i] = '\0';
    }

    IPort& _port;
    Level _floor;
    Record _ring[Slots] = {};
    // Free-running 32-bit indices (house seq-ring idiom): pending = write-read.
    uint32_t _write = 0;
    uint32_t _read = 0;
    uint16_t _lostSinceDrain = 0;
    uint32_t _totalLostShadow = 0;
    // Live count of Trace/Debug/Info records parked in the ring. This single
    // counter is what makes the reserve O(1): the alternative — scanning the
    // ring for the oldest low-severity slot — would run under the spinlock on
    // whichever task happened to log, which is exactly the cost this logger
    // exists to avoid.
    size_t _lowPending = 0;
    uint32_t _dropped[kLevelCount] = {};   // per-level lifetime shed counts
    ISink* _sinks[MaxSinks] = {};
    Level _sinkFloors[MaxSinks] = {};
    size_t _sinkCount = 0;
    bool _immediateDrain = false;
};

}  // namespace vlog
