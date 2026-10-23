#pragma once

// CrashRing -- the breadcrumbs a boot leaves for the next one: the last few
// Warn+ lines, and one "last words" line written outside the hub task (a task
// watchdog's starved tasks, or the task that called esp_restart)
// Constraints:
// - Hardware-free, header-only, allocation-free. ValenceDiag.cpp keeps two
//   copies, one in LP RAM and one in HP SRAM, and adopts the first valid one
//   at boot (pick()). Suite test_crash_ring drives it on the host.
// - A line already in the ring is not a new crumb, so the ring holds the last
//   kCrumbs DISTINCT lines: a once-a-minute reminder (the self-check's) must
//   not push the lines that name a fault out of it.
// - add() has one writer, the hub task. setLast() runs on any task or in the
//   task watchdog's ISR; it touches only `last`, never the ring indices.
// - kMagic names the layout: a layout change takes a new magic, so an older
//   image's ring reads as absent, never as garbage.
// See: flagship_p4/src/system/ValenceDiag.h

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace valence::crashring {

constexpr size_t kCrumbs = 8;
constexpr size_t kCrumbBytes = 60;   // includes NUL; longer lines truncate
constexpr size_t kLastBytes = 96;    // includes NUL
constexpr uint32_t kMagic = 0x4E554332u;   // "NUC2"

// alignas(64): one L1 cache line, so the HP copy writes back whole lines and
// never shares one with a neighbor.
struct alignas(64) Ring {
    uint32_t magic;
    uint32_t bootSeq;
    uint32_t head;    // the next slot to write
    uint32_t count;
    char crumb[kCrumbs][kCrumbBytes];
    char last[kLastBytes];
    uint32_t magic2;
};

inline bool valid(const Ring& r) {
    return r.magic == kMagic && r.magic2 == ~kMagic && r.head < kCrumbs && r.count <= kCrumbs &&
           r.last[kLastBytes - 1] == '\0';
}

inline void arm(Ring& r, uint32_t bootSeq) {
    r.magic = kMagic;
    r.magic2 = ~kMagic;
    r.bootSeq = bootSeq;
    r.head = 0;
    r.count = 0;
    std::memset(r.last, 0, sizeof r.last);
}

inline void copyBounded(char* dst, const char* src, size_t cap) {
    size_t i = 0;
    for (; i + 1 < cap && src[i] != '\0'; ++i) dst[i] = src[i];
    dst[i] = '\0';
}

inline void add(Ring& r, const char* line) {
    for (uint32_t i = 0; i < r.count; ++i)
        if (std::strncmp(r.crumb[i], line, kCrumbBytes - 1) == 0) return;
    copyBounded(r.crumb[r.head], line, kCrumbBytes);
    r.head = (r.head + 1) % kCrumbs;
    if (r.count < kCrumbs) ++r.count;
}

inline void setLast(Ring& r, const char* text) { copyBounded(r.last, text, kLastBytes); }

// Oldest first into out; returns the count.
inline uint32_t copyOut(const Ring& r, char (&out)[kCrumbs][kCrumbBytes]) {
    for (uint32_t i = 0; i < r.count; ++i) {
        const uint32_t src = (r.count == kCrumbs) ? ((r.head + i) % kCrumbs) : i;
        std::memcpy(out[i], r.crumb[src], kCrumbBytes);
        out[i][kCrumbBytes - 1] = '\0';
    }
    return r.count;
}

// The LP copy when it survived, else the HP copy, else none.
inline const Ring* pick(const Ring& lp, const Ring& hp) {
    if (valid(lp)) return &lp;
    if (valid(hp)) return &hp;
    return nullptr;
}

}  // namespace valence::crashring
