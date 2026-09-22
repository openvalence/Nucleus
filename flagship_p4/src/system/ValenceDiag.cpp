// ValenceDiag -- implementation. See ValenceDiag.h for the dump-not-stream
// ruling, the no-freeze rule and the cursor contract; nothing is restated here.

#include "system/ValenceDiag.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_http_server.h>
#include <esp_system.h>
#include <esp_timer.h>

#include "geiger/geiger.h"
#include "hub/valence_config.h"
#include "system/ValenceHttp.h"
#include "system/ValenceOta.h"

namespace valence {

namespace {

constexpr const char* kTag = "diag";

// ~2.0 MB of the stamp's 32 MB. At the hub's observed chatter this is hours,
// which is the property that matters: an archive that recycled before the
// operator got to it is the failure this replaces.
constexpr size_t kSlots = 16384;

// ---- the RTC_NOINIT crash ring ----------------------------------------------
// LP memory, uninitialized by the startup code on purpose, so it survives a
// panic reset and is the ONLY evidence of a boot that ended badly. Small: the
// last few Warn+ lines are what name a fault, and a deep ring here would eat
// the LP RAM the emitter is reserved out of.
constexpr size_t kCrumbs = 8;
constexpr size_t kCrumbBytes = 60;
constexpr uint32_t kCrashMagic = 0x4E554331u;   // "NUC1"

struct CrashRing {
    uint32_t magic;
    uint32_t bootSeq;
    uint32_t head;
    uint32_t count;
    char crumb[kCrumbs][kCrumbBytes];
    uint32_t magic2;
};

RTC_NOINIT_ATTR CrashRing g_crash;

bool crashRingValid() {
    return g_crash.magic == kCrashMagic && g_crash.magic2 == ~kCrashMagic &&
           g_crash.head < kCrumbs && g_crash.count <= kCrumbs;
}

const char* resetName(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:  return "poweron";
        case ESP_RST_EXT:      return "external";
        case ESP_RST_SW:       return "software";
        case ESP_RST_PANIC:    return "PANIC";
        case ESP_RST_INT_WDT:  return "int-wdt";
        case ESP_RST_TASK_WDT: return "TASK-WDT";
        case ESP_RST_WDT:      return "other-wdt";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO:     return "sdio";
        case ESP_RST_USB:      return "usb";
        case ESP_RST_JTAG:     return "jtag";
        case ESP_RST_EFUSE:    return "efuse";
        case ESP_RST_PWR_GLITCH: return "power-glitch";
        case ESP_RST_CPU_LOCKUP: return "cpu-lockup";
        default:               return "unknown";
    }
}

// ---- the archive -------------------------------------------------------------
// One writer (the hub task, through geiger::drainToSinks) and one reader (the
// :80 task). The writer stores the record, then publishes it by bumping _write
// with a release; the reader acquires _write, copies, and re-checks that its
// slot is still inside the window. That re-check is the whole of the lapping
// discipline -- there is no lock and the writer is never delayed.
struct Archive final : public geiger::ISink {
    void write(const geiger::Record& r) override {
        const uint32_t w = _write.load(std::memory_order_relaxed);
        _ring[w % kSlots] = r;
        _write.store(w + 1, std::memory_order_release);
        if (geiger::isReserved(r.level)) crumb(r);
    }

    static void crumb(const geiger::Record& r) {
        snprintf(g_crash.crumb[g_crash.head], kCrumbBytes, "%c %s %s",
                 geiger::levelChar(r.level), r.tag, r.msg);
        g_crash.head = (g_crash.head + 1) % kCrumbs;
        if (g_crash.count < kCrumbs) ++g_crash.count;
    }

    std::atomic<uint32_t> _write{0};
    uint32_t _bootSeq = 0;
    uint32_t _prevCount = 0;
    char _prev[kCrumbs][kCrumbBytes] = {};
    geiger::Record _ring[kSlots] = {};
};

Archive* g_ar = nullptr;

// ---- the dump ----------------------------------------------------------------

// One TCP chunk's worth. Stack-resident and modest: the :80 task carries 8 KB
// and this handler must allocate NOTHING -- an instrument that needs heap is
// switched off by exactly the pressure it exists to explain.
constexpr size_t kChunkBytes = 1024;

struct Out {
    httpd_req_t* req;
    char buf[kChunkBytes];
    size_t n = 0;
    bool ok = true;

    void flush() {
        if (!ok || n == 0) return;
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) ok = false;
        n = 0;
    }
    void put(const char* s, size_t len) {
        if (!ok) return;
        if (len >= kChunkBytes) len = kChunkBytes - 1;
        if (n + len > kChunkBytes) flush();
        if (!ok) return;
        std::memcpy(buf + n, s, len);
        n += len;
    }
    void emit(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        char line[224];
        va_list ap;
        va_start(ap, fmt);
        const int m = vsnprintf(line, sizeof(line), fmt, ap);
        va_end(ap);
        if (m > 0) put(line, size_t(m) < sizeof(line) ? size_t(m) : sizeof(line) - 1);
    }
};

// Parses "/diag", "/diag/hub", "/diag?from=9", "/diag/hub?from=9". Returns the
// tag (empty = every tag) and writes `from`.
void parseUri(const char* uri, char* tag, size_t tagCap, uint32_t& from) {
    tag[0] = '\0';
    from = 0;
    const char* p = uri + 5;   // past "/diag"
    if (*p == '/') {
        ++p;
        size_t i = 0;
        while (*p && *p != '?' && i + 1 < tagCap) tag[i++] = *p++;
        tag[i] = '\0';
    }
    while (*p && *p != '?') ++p;
    if (*p != '?') return;
    const char* q = std::strstr(p, "from=");
    if (q != nullptr) from = uint32_t(strtoul(q + 5, nullptr, 10));
}

esp_err_t handleGet(httpd_req_t* req) {
    if (g_ar == nullptr) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "no archive: the PSRAM allocation failed at boot\n");
    }
    char tag[geiger::Record::kTagBytes] = {};
    uint32_t from = 0;
    parseUri(req->uri, tag, sizeof(tag), from);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");

    Out out{req};
    const uint32_t w = g_ar->_write.load(std::memory_order_acquire);
    const uint32_t oldest = (w > kSlots) ? (w - kSlots) : 0;
    // Clamped at BOTH ends. Past the head is the useful shape -- a caller
    // asking for a header-only dump passes a cursor beyond the end -- and
    // echoing that cursor back in the footer would hand it a next= that skips
    // every record written after the read.
    uint32_t seq = (from > oldest) ? from : oldest;
    if (seq > w) seq = w;

    char drops[64];
    geiger::logger().formatDropSummary(drops, sizeof(drops));
    // img= is the OTA acceptance signal and it belongs HERE rather than in a
    // route of its own: "the new version answers" only proves the image
    // booted, and one that boots but never buys itself is a single reset from
    // being gone. The deploy tool reads this line.
    out.emit("# %s %s slot=%s img=%s boot=%lu reset=%s up=%lus\n",
             VALENCE_HUB_NAME, FIRMWARE_VERSION, otaRunningSlot(),
             otaPendingVerify() ? "pending" : "valid",
             static_cast<unsigned long>(g_ar->_bootSeq),
             resetName(esp_reset_reason()),
             static_cast<unsigned long>(esp_timer_get_time() / 1000000));
    out.emit("# archive %u B PSRAM, %u slots, seq %lu..%lu, from=%lu tag=%s, geiger dropped %s\n",
             unsigned(sizeof(Archive)), unsigned(kSlots),
             static_cast<unsigned long>(oldest), static_cast<unsigned long>(w),
             static_cast<unsigned long>(seq), tag[0] ? tag : "*", drops);
    if (g_ar->_prevCount > 0) {
        out.emit("# the PREVIOUS boot left %lu breadcrumbs (RTC_NOINIT, survived the reset):\n",
                 static_cast<unsigned long>(g_ar->_prevCount));
        for (uint32_t i = 0; i < g_ar->_prevCount; ++i)
            out.emit("#   %s\n", g_ar->_prev[i]);
    }

    bool lapped = false;
    for (; seq < w && out.ok; ++seq) {
        const geiger::Record r = g_ar->_ring[seq % kSlots];
        // The copy above is only trustworthy while its slot is still inside
        // the window. Re-check AFTER copying: the writer may have wrapped
        // past it mid-copy, and no amount of locking here is allowed.
        if (g_ar->_write.load(std::memory_order_acquire) - seq > kSlots) {
            lapped = true;
            break;
        }
        if (tag[0] != '\0' && std::strcmp(tag, r.tag) != 0) continue;
        // The gap a record admits to is part of the record: a dump that hides
        // it reads as a complete history it is not.
        char lost[28] = "";
        if (r.lost) snprintf(lost, sizeof(lost), "  (+%u lost)", unsigned(r.lost));
        out.emit("%lu [%7lu.%03lu %c%u %-10s] %s%s\n",
                 static_cast<unsigned long>(seq),
                 static_cast<unsigned long>(r.ms / 1000u),
                 static_cast<unsigned long>(r.ms % 1000u),
                 geiger::levelChar(r.level), unsigned(r.core), r.tag, r.msg, lost);
    }
    if (lapped) {
        const uint32_t now = g_ar->_write.load(std::memory_order_acquire);
        seq = (now > kSlots) ? (now - kSlots) : 0;
        out.emit("# TRUNCATED: the writer lapped this reader; records before seq %lu are gone\n",
                 static_cast<unsigned long>(seq));
    }
    out.emit("next=%lu\n", static_cast<unsigned long>(seq));
    out.flush();
    httpd_resp_send_chunk(req, nullptr, 0);
    httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
    return ESP_OK;
}

}  // namespace

bool diagBegin() {
    // PSRAM via placement-new (T2): two megabytes in .bss would be internal
    // RAM the network stack allocates from, and the archive is the one thing
    // that must never compete with the machine it is watching.
    void* mem = heap_caps_malloc(sizeof(Archive), MALLOC_CAP_SPIRAM);
    if (mem == nullptr) {
        GLOGE(kTag, "PSRAM alloc of %u bytes for the archive failed", unsigned(sizeof(Archive)));
        return false;
    }
    g_ar = new (mem) Archive();

    // Adopt the previous boot's breadcrumbs, then re-arm the ring for this one.
    if (crashRingValid()) {
        g_ar->_bootSeq = g_crash.bootSeq + 1;
        g_ar->_prevCount = g_crash.count;
        for (uint32_t i = 0; i < g_crash.count; ++i) {
            // Oldest first: head points at the next write, so the oldest of a
            // full ring sits there and a partial ring starts at 0.
            const uint32_t src = (g_crash.count == kCrumbs)
                                     ? ((g_crash.head + i) % kCrumbs) : i;
            std::memcpy(g_ar->_prev[i], g_crash.crumb[src], kCrumbBytes);
        }
    } else {
        g_ar->_bootSeq = 1;
    }
    g_crash.magic = kCrashMagic;
    g_crash.magic2 = ~kCrashMagic;
    g_crash.bootSeq = g_ar->_bootSeq;
    g_crash.head = 0;
    g_crash.count = 0;

    geiger::logger().addSink(g_ar);
    GLOGI(kTag, "archive %u B in PSRAM, %u slots; boot %lu after %s, %lu prior breadcrumbs",
          unsigned(sizeof(Archive)), unsigned(kSlots),
          static_cast<unsigned long>(g_ar->_bootSeq), resetName(esp_reset_reason()),
          static_cast<unsigned long>(g_ar->_prevCount));
    return true;
}

bool diagAttachRoutes() {
    httpd_handle_t srv = http80();
    if (srv == nullptr) return false;
    // Wildcard, so /diag, /diag/<tag> and both with ?from= reach one handler
    // and a new subsystem gets a route by logging under a new tag.
    httpd_uri_t u{"/diag*", HTTP_GET, handleGet, nullptr};
    if (httpd_register_uri_handler(srv, &u) != ESP_OK) {
        GLOGE(kTag, "route registration failed");
        return false;
    }
    GLOGI(kTag, "GET /diag[/<tag>][?from=<seq>] on :80");
    return true;
}

size_t diagArchiveBytes() { return g_ar ? sizeof(Archive) : 0; }

}  // namespace valence
