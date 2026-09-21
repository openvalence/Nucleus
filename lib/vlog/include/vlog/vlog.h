// VLog -- the systemwide logging front door, plus one platform layer per host.
//
//   SLOGI("wifi", "connected to %s ch%d", ssid, ch);
//   SLOGW_EVERY_MS(1000, "arbiter", "intent rejected: %s", why);   // per-site
//
// Constraints:
// - THREE PLATFORM BRANCHES, ONE MACRO SURFACE: Arduino, pure ESP-IDF, and a
//   mute host build. Each branch supplies the same names -- port(), logger(),
//   drainToSinks(), VLOG_EMIT, VLOG_NOW_MS -- so a call site never spells a
//   platform. A branch that binds only some of them leaves every SLOGx on that
//   host compiling to ((void)0), which is silent and looks like working code.
// - SLOG* is callable from any FreeRTOS task on either core: bounded format
//   plus spinlock'd slot copy, never blocks, never allocates. NOT ISR-safe.
// - Exactly ONE task calls drainToSinks(); sinks run on that task only.
// - Every sink here is non-blocking by contract: drop-and-count when the output
//   is full, never wait. A sink that can wait freezes the drain task.
// - Compile floor VLOG_COMPILE_LEVEL (0=Trace..5=Fatal) removes code and
//   strings outright; vlog::logger().setFloor() is the runtime floor on top.
// See: .claude/rules/logging-leds.md (T6, the drain-point rule)
#pragma once

#include "vlog/vlog_core.hpp"

// ---- Arduino platform layer -------------------------------------------------

#if defined(ARDUINO)
#include <Arduino.h>
#include "freertos/FreeRTOS.h"

namespace vlog {

class Esp32Port final : public IPort {
public:
    uint32_t nowMs() override { return millis(); }
    uint8_t coreId() override { return uint8_t(xPortGetCoreID()); }
    void lock() override { portENTER_CRITICAL(&_mux); }
    void unlock() override { portEXIT_CRITICAL(&_mux); }

private:
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
};

using FirmwareLog = LogCore<64, 4>;

// Meyers singletons: construction is race-free and needs no init() ordering
// games — the first SLOG anywhere (even during static init) just works. Both
// objects are ordinary statics in internal RAM, which is required: the port
// holds a portMUX_TYPE and a spinlock is only defined on internal memory.
inline Esp32Port& port() {
    static Esp32Port p;
    return p;
}
inline FirmwareLog& logger() {
    static FirmwareLog l(port(), Level::Trace);
    return l;
}

// NON-BLOCKING BY CONTRACT: on USB-CDC, writes with no host draining the port
// can block ~100 ms PER LINE, and this sink runs on the drain task. So the line
// is written only when the TX buffer can take the whole thing; otherwise it is
// dropped and counted, and the next line that lands admits the gap.
class SerialSink final : public ISink {
public:
    void write(const Record& r) override {
        char line[192];
        int n = snprintf(line, sizeof(line), "[%7lu.%03lu %c%u %-10s] %s",
                         (unsigned long)(r.ms / 1000u), (unsigned long)(r.ms % 1000u),
                         levelChar(r.level), r.core, r.tag, r.msg);
        if (n < 0) return;
        if (n >= int(sizeof(line))) n = int(sizeof(line)) - 1;
        if (r.lost) {
            n += snprintf(line + n, sizeof(line) - size_t(n), "  (+%u lost)", r.lost);
            if (n >= int(sizeof(line))) n = int(sizeof(line)) - 1;
        }
        // The owner MUST have set Serial.setTxTimeoutMs(0): with a zero TX
        // timeout the CDC driver drops instead of blocking, so unconditional
        // writes here are safe. Do NOT gate on availableForWrite() — on HWCDC
        // it reports 0 until the driver's host-detection heuristic is happy
        // (dumb terminals never satisfy it), which swallowed every line in the
        // field.
        if (_dropped) {
            char note[40];
            int m = snprintf(note, sizeof(note), "(serial dropped %lu)",
                             (unsigned long)_dropped);
            if (m > 0 && Serial.write(reinterpret_cast<const uint8_t*>(note), size_t(m)) == size_t(m)) {
                Serial.write("\r\n", 2);
                _dropped = 0;
            }
        }
        // All-or-nothing when the driver reports genuine backpressure: a
        // NONZERO-but-small availableForWrite is trustworthy; zero is ambiguous
        // on HWCDC, so zero falls through to an attempted write. Partial writes
        // still terminate the line so a pressured stream truncates cleanly
        // instead of interleaving into soup.
        int avail = Serial.availableForWrite();
        if (avail > 0 && avail < n + 2) {
            ++_dropped;
            return;
        }
        size_t wrote = Serial.write(reinterpret_cast<const uint8_t*>(line), size_t(n));
        if (wrote == size_t(n)) {
            Serial.write("\r\n", 2);
        } else {
            if (wrote > 0) Serial.write("\r\n", 2);  // close the mangled line
            ++_dropped;  // buffer full / no listener — dropped, never blocked
        }
    }

private:
    uint32_t _dropped = 0;
};

inline SerialSink& serialSink() {
    static SerialSink s;
    return s;
}

// Registers the console sink. The composition root calls it exactly once,
// after Serial.begin().
inline void logBegin() { logger().addSink(&serialSink()); }

// Pumped from exactly one task.
inline void drainToSinks() { logger().drain(); }

}  // namespace vlog

#define VLOG_EMIT(lvl, tag, ...) ::vlog::logger().logf(lvl, tag, __VA_ARGS__)
#define VLOG_NOW_MS() ((uint32_t)millis())

// ---- ESP-IDF platform layer -------------------------------------------------
// The Arduino branch's twin, and it must stay a twin: arduino-esp32 also
// defines ESP_PLATFORM, so this branch is reachable only when ARDUINO is not.

#elif defined(ESP_PLATFORM)
#include <cstdio>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace vlog {

class IdfPort final : public IPort {
public:
    uint32_t nowMs() override { return uint32_t(esp_timer_get_time() / 1000); }
    uint8_t coreId() override { return uint8_t(xPortGetCoreID()); }
    void lock() override { portENTER_CRITICAL(&_mux); }
    void unlock() override { portEXIT_CRITICAL(&_mux); }

private:
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
};

using FirmwareLog = LogCore<64, 4>;

// Meyers singletons, same contract as the Arduino branch: the first SLOGx
// anywhere works with no init ordering. Ordinary statics, so both live in
// internal RAM — required, because the port holds a portMUX_TYPE and a
// spinlock is only defined on internal memory.
inline IdfPort& port() {
    static IdfPort p;
    return p;
}
inline FirmwareLog& logger() {
    static FirmwareLog l(port(), Level::Debug);
    return l;
}

// The console on an IDF target is whatever stdout is wired to (USB-Serial/JTAG
// on the P4 stamp). Drop-and-count, never wait: this runs on the drain task and
// a detached host must never stall it.
class StdoutSink final : public ISink {
public:
    void write(const Record& r) override {
        char line[192];
        int n = snprintf(line, sizeof(line), "[%7lu.%03lu %c%u %-10s] %s",
                         static_cast<unsigned long>(r.ms / 1000u),
                         static_cast<unsigned long>(r.ms % 1000u),
                         levelChar(r.level), unsigned(r.core), r.tag, r.msg);
        if (n < 0) return;
        if (n >= int(sizeof(line))) n = int(sizeof(line)) - 1;
        if (r.lost) {
            n += snprintf(line + n, sizeof(line) - size_t(n), "  (+%u lost)", unsigned(r.lost));
            if (n >= int(sizeof(line))) n = int(sizeof(line)) - 1;
        }
        line[n] = '\n';
        // fwrite, not printf: one call, no reformatting, and the count tells us
        // whether the whole line landed. A short write is a dropped line, never
        // a retry -- a retry is the blocking sink T6 forbids.
        if (fwrite(line, 1, size_t(n) + 1, stdout) != size_t(n) + 1) ++_dropped;
    }

    uint32_t dropped() const { return _dropped; }

private:
    uint32_t _dropped = 0;
};

inline StdoutSink& stdoutSink() {
    static StdoutSink s;
    return s;
}

// Registers the console sink. Idempotent by call-once discipline, not by
// guard: the composition root calls it exactly once.
inline void logBegin() { logger().addSink(&stdoutSink()); }

// Pumped from exactly one task.
inline void drainToSinks() { logger().drain(); }

}  // namespace vlog

#define VLOG_EMIT(lvl, tag, ...) ::vlog::logger().logf(lvl, tag, __VA_ARGS__)
#define VLOG_NOW_MS() (uint32_t)(esp_timer_get_time() / 1000)

// ---- Host builds ------------------------------------------------------------
// No platform, no sink, no clock: the native suites drive LogCore directly and
// nothing under test emits through the macro surface.

#else
#define VLOG_EMIT(lvl, tag, ...) ((void)0)
#endif

// ---- Compile-time floor -----------------------------------------------------
// 0 Trace, 1 Debug, 2 Info, 3 Warn, 4 Error, 5 Fatal. Default keeps Debug+.
#ifndef VLOG_COMPILE_LEVEL
#define VLOG_COMPILE_LEVEL 1
#endif

#if VLOG_COMPILE_LEVEL <= 0
#define SLOGT(tag, ...) VLOG_EMIT(::vlog::Level::Trace, tag, __VA_ARGS__)
#else
#define SLOGT(tag, ...) ((void)0)
#endif
#if VLOG_COMPILE_LEVEL <= 1
#define SLOGD(tag, ...) VLOG_EMIT(::vlog::Level::Debug, tag, __VA_ARGS__)
#else
#define SLOGD(tag, ...) ((void)0)
#endif
#if VLOG_COMPILE_LEVEL <= 2
#define SLOGI(tag, ...) VLOG_EMIT(::vlog::Level::Info, tag, __VA_ARGS__)
#else
#define SLOGI(tag, ...) ((void)0)
#endif
#if VLOG_COMPILE_LEVEL <= 3
#define SLOGW(tag, ...) VLOG_EMIT(::vlog::Level::Warn, tag, __VA_ARGS__)
#else
#define SLOGW(tag, ...) ((void)0)
#endif
#if VLOG_COMPILE_LEVEL <= 4
#define SLOGE(tag, ...) VLOG_EMIT(::vlog::Level::Error, tag, __VA_ARGS__)
#else
#define SLOGE(tag, ...) ((void)0)
#endif
#define SLOGF(tag, ...) VLOG_EMIT(::vlog::Level::Fatal, tag, __VA_ARGS__)

// ---- Per-call-site rate limiting --------------------------------------------
// Each macro expansion owns its own static throttle state (that's the point:
// the rate limit is per SITE, not per tag). Suppressed emissions are counted
// and reported on the next one that passes: "... (suppressed 42)".
// The unsigned now-last compare is wrap-safe for intervals < 2^31 ms.
// Gated on VLOG_NOW_MS rather than on a platform name, so a new platform layer
// gets throttling by supplying a clock and nothing else.
#if defined(VLOG_NOW_MS)
#define VLOG_EVERY_MS(lvl_macro, interval_ms, tag, fmt, ...)                           \
    do {                                                                               \
        static uint32_t _vlog_last = 0;                                                \
        static uint32_t _vlog_skips = 0;                                               \
        static bool _vlog_ever = false;                                                \
        uint32_t _vlog_now = VLOG_NOW_MS();                                            \
        if (!_vlog_ever || (_vlog_now - _vlog_last) >= (uint32_t)(interval_ms)) {       \
            _vlog_ever = true;                                                          \
            _vlog_last = _vlog_now;                                                     \
            if (_vlog_skips) {                                                          \
                lvl_macro(tag, fmt " (suppressed %lu)", ##__VA_ARGS__,                  \
                          (unsigned long)_vlog_skips);                                  \
                _vlog_skips = 0;                                                        \
            } else {                                                                    \
                lvl_macro(tag, fmt, ##__VA_ARGS__);                                      \
            }                                                                           \
        } else {                                                                        \
            ++_vlog_skips;                                                               \
        }                                                                                \
    } while (0)
#else
#define VLOG_EVERY_MS(lvl_macro, interval_ms, tag, fmt, ...) ((void)0)
#endif

#define SLOGD_EVERY_MS(interval_ms, tag, fmt, ...) VLOG_EVERY_MS(SLOGD, interval_ms, tag, fmt, ##__VA_ARGS__)
#define SLOGI_EVERY_MS(interval_ms, tag, fmt, ...) VLOG_EVERY_MS(SLOGI, interval_ms, tag, fmt, ##__VA_ARGS__)
#define SLOGW_EVERY_MS(interval_ms, tag, fmt, ...) VLOG_EVERY_MS(SLOGW, interval_ms, tag, fmt, ##__VA_ARGS__)
#define SLOGE_EVERY_MS(interval_ms, tag, fmt, ...) VLOG_EVERY_MS(SLOGE, interval_ms, tag, fmt, ##__VA_ARGS__)
