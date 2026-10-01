#pragma once

// Supervisor -- the link between the P4 and the board monitor (U12, CH32V003):
// register ids, block layouts, CRCs, and the encoders and decoders both sides
// share
// Constraints:
// - ONE HOME for the message set. The monitor (flagship_ch32v003/, C on
//   RV32EC) and the P4 (C++) both include this file, so it is the C/C++
//   common subset: no namespaces, no constexpr, no references, fixed-width
//   integers, explicit little-endian packing. Never a packed struct on the
//   wire: the two compilers need not agree on layout.
// - Hardware-free and allocation-free; every function is pure.
// - Transport: the monitor is an I2C target at SV_I2C_ADDR on the motor
//   current monitor's private bus (SDA G34, SCL G36, 400 kHz). A read is a
//   one-byte write of the register id, then a read of that block's length.
//   Every block, in either direction, is [id][len][payload][crc8], len counts
//   all of it, and the CRC covers every byte before it. A block whose id,
//   len or CRC is wrong is dropped whole and never partly applied.
// - The monitor can cut motor power (FAULT_N), never enable it. The P4 can
//   tighten a limit, never loosen one past the monitor's compiled default.
// - Units: millivolts at the rail for rails, millivolts at the monitor pin
//   for SHUNT_TEMP and CLAMP_MON, permille for duties, milliseconds for time.
// See: docs/supervisor.md, Hardware flagship/SPEC.md (2026-09-23
// board-monitor row), bd val-091.19, val-091.20

#include <stdint.h>

// ---- addressing ------------------------------------------------------------

// TODO(val-091.19): operator ruling on the address; it must stay outside the
// INA228/INA237 range 0x40-0x4F.
#define SV_I2C_ADDR 0x2C
#define SV_LINK_VERSION 1

#define SV_REG_IDENT     0x00   // read
#define SV_REG_STATUS    0x10   // read
#define SV_REG_LIMITS    0x20   // read, and write to tighten
#define SV_REG_HEARTBEAT 0x30   // write
#define SV_REG_COMMAND   0x31   // write

#define SV_IDENT_LEN     14
#define SV_STATUS_LEN    37
#define SV_LIMITS_LEN    27
#define SV_HEARTBEAT_LEN 4
#define SV_COMMAND_LEN   6
#define SV_BLOCK_MAX     SV_STATUS_LEN

#define SV_IDENT_MAGIC 0x4D4E   // "NM" on the wire

// Image identity: CRC-32 of the monitor's whole code flash, unwritten bytes
// read as 0xFF. The P4 pads its embedded image the same way.
#define SV_FLASH_BYTES 16384

// ---- channels: ADC scan order and STATUS reading order ----------------------

enum {
    SV_CH_VIN_RAW = 0,   // PD2 A3, 160k/10k
    SV_CH_BUS,           // PD3 A4, 160k/10k
    SV_CH_12V,           // PD4 A7, 39k/10k
    SV_CH_5V,            // PD5 A5, 10k/10k
    SV_CH_5V_SYS,        // PA1 A1, 10k/10k
    SV_CH_3V3_ACC,       // PD6 A6, 10k/10k
    SV_CH_SHUNT_TEMP,    // PA2 A0, 1k tap; pin mV normalized to a 3300 mV pull-up
    SV_CH_CLAMP_MON,     // PC4 A2, 1k tap; pin mV
    SV_CH_COUNT
};

// ---- faults (FAULT_N) and warnings ------------------------------------------

#define SV_F_BOOT          (1u << 0)    // first full scan not complete
#define SV_F_VDD           (1u << 1)    // monitor supply out of window: readings untrusted
#define SV_F_12V           (1u << 2)    // +12V out of window (clamp comparator supply)
#define SV_F_5V_SYS        (1u << 3)    // +5V_SYS out of window (quadrature buffer, drive comms)
#define SV_F_INPUT_STAGE   (1u << 4)    // VIN_RAW present, +BUS not following
#define SV_F_BUS_OV        (1u << 5)    // +BUS above the clamp's reach
#define SV_F_SHUNT_HOT     (1u << 6)    // regen load past its trip temperature
#define SV_F_CLAMP_STUCK   (1u << 7)    // clamp conducting with the trim released and no regen
#define SV_F_HEARTBEAT     (1u << 8)    // armed P4 heartbeat went silent
// Every fault bit pulls FAULT_N; a warning never does.
#define SV_FAULT_MASK      0x01FFu

#define SV_W_5V            (1u << 0)
#define SV_W_3V3_ACC       (1u << 1)
#define SV_W_PUMP_FLT      (1u << 2)    // U14 FLT low (auto-retrying eFuse)
#define SV_W_SHUNT_WARM    (1u << 3)    // trim released for temperature
#define SV_W_CLAMP_FEEDING (1u << 4)    // clamp seen conducting with no regen; trim released
#define SV_W_LINK_DROP     (1u << 5)    // a block from the P4 failed its id, len or CRC

#define SV_FLAG_FAULT_N     (1u << 0)   // FAULT_N is being pulled
#define SV_FLAG_HB_ARMED    (1u << 1)
#define SV_FLAG_TRIM_LATCH  (1u << 2)   // trim target latched from a stable supply
#define SV_FLAG_BUDGET      (1u << 3)   // VIN_RAW absent with +BUS present: F1 on its third pad
#define SV_FLAG_TRIM_HELD   (1u << 4)   // trim released and held until TRIM_RELATCH or reset
#define SV_FLAG_CLAMP_TEST  (1u << 5)   // clamp test pulse in progress

enum { SV_TEST_NONE = 0, SV_TEST_PASS = 1, SV_TEST_FAIL = 2 };

// ---- commands ---------------------------------------------------------------

enum {
    SV_CMD_CLEAR_LATCHED = 1,   // clear latched faults whose live condition is gone
    SV_CMD_CLAMP_TEST    = 2,   // arg: pulse ms, clamped to 30-50
    SV_CMD_TRIM_RELEASE  = 3,   // duty 0 and the trim pin released; held
    SV_CMD_TRIM_RELATCH  = 4    // latch the trim again from the current stable supply
};

// ---- block contents ---------------------------------------------------------

typedef struct {
    uint8_t  link_version;
    uint8_t  fw_major, fw_minor, fw_patch;
    uint32_t image_crc32;
    uint8_t  reset_cause;
} SvIdent;

typedef struct {
    uint8_t  seq;
    uint8_t  flags;
    uint16_t faults_live;
    uint16_t faults_latched;
    uint16_t warns;
    uint8_t  first_fault;          // bit index + 1 of the first latched fault; 0 none
    uint8_t  test_result;          // SV_TEST_*
    uint16_t mv[SV_CH_COUNT];
    uint16_t vdd_mv;
    uint16_t trim_target_mv;       // +BUS clamp threshold the trim aims at; 0 released
    uint16_t trim_permille;
    uint16_t clamp_on_permille;    // CLAMP_MON high fraction over the last second
} SvStatus;

typedef struct {
    uint16_t lo_mv[4], hi_mv[4];   // +12V, +5V, +5V_SYS, +3V3_ACC
    uint16_t bus_ov_mv;
    uint16_t shunt_warm_mv;        // NTC pin: LOWER is hotter
    uint16_t shunt_trip_mv;
    uint16_t heartbeat_timeout_ms;
} SvLimits;

enum { SV_OK = 0, SV_ERR_LEN, SV_ERR_ID, SV_ERR_CRC, SV_ERR_MAGIC };

// ---- checksums --------------------------------------------------------------

// CRC-8, polynomial 0x07, init 0 (the SMBus PEC polynomial). Check value of
// "123456789" is 0xF4.
static inline uint8_t sv_crc8(const uint8_t* p, uint32_t n) {
    uint8_t c = 0;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) c = (uint8_t)((c & 0x80) ? (c << 1) ^ 0x07 : (c << 1));
    }
    return c;
}

// CRC-32/IEEE, reflected. Start from 0xFFFFFFFF, feed chunks, invert at the
// end. Check value of "123456789" is 0xCBF43926.
static inline uint32_t sv_crc32_update(uint32_t c, const uint8_t* p, uint32_t n) {
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) c = (c & 1u) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
    }
    return c;
}

// ---- byte packing -----------------------------------------------------------

static inline void sv_put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline uint16_t sv_get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline void sv_put32(uint8_t* p, uint32_t v) { sv_put16(p, (uint16_t)v); sv_put16(p + 2, (uint16_t)(v >> 16)); }
static inline uint32_t sv_get32(const uint8_t* p) { return sv_get16(p) | ((uint32_t)sv_get16(p + 2) << 16); }

static inline void sv_seal(uint8_t* b, uint8_t id, uint8_t len) {
    b[0] = id;
    b[1] = len;
    b[len - 1] = sv_crc8(b, (uint32_t)len - 1);
}

static inline int sv_check(const uint8_t* b, uint32_t n, uint8_t id, uint8_t len) {
    if (n != len || b[1] != len) return SV_ERR_LEN;
    if (b[0] != id) return SV_ERR_ID;
    if (sv_crc8(b, (uint32_t)len - 1) != b[len - 1]) return SV_ERR_CRC;
    return SV_OK;
}

// ---- IDENT ------------------------------------------------------------------

static inline void sv_ident_encode(const SvIdent* s, uint8_t out[SV_IDENT_LEN]) {
    sv_put16(out + 2, SV_IDENT_MAGIC);
    out[4] = s->link_version;
    out[5] = s->fw_major;
    out[6] = s->fw_minor;
    out[7] = s->fw_patch;
    sv_put32(out + 8, s->image_crc32);
    out[12] = s->reset_cause;
    sv_seal(out, SV_REG_IDENT, SV_IDENT_LEN);
}

static inline int sv_ident_decode(const uint8_t* in, uint32_t n, SvIdent* s) {
    int e = sv_check(in, n, SV_REG_IDENT, SV_IDENT_LEN);
    if (e != SV_OK) return e;
    if (sv_get16(in + 2) != SV_IDENT_MAGIC) return SV_ERR_MAGIC;
    s->link_version = in[4];
    s->fw_major = in[5];
    s->fw_minor = in[6];
    s->fw_patch = in[7];
    s->image_crc32 = sv_get32(in + 8);
    s->reset_cause = in[12];
    return SV_OK;
}

// The P4's boot decision (val-091.20): reprogram on no answer, a bad block, a
// different link version, or ANY image difference. Exact match, not "older
// than": an OTA rollback must take the monitor back with the P4 image.
static inline int sv_needs_flash(int decode_result, const SvIdent* s, uint32_t embedded_crc32) {
    return decode_result != SV_OK || s->link_version != SV_LINK_VERSION
        || s->image_crc32 != embedded_crc32;
}

// ---- STATUS -----------------------------------------------------------------

static inline void sv_status_encode(const SvStatus* s, uint8_t out[SV_STATUS_LEN]) {
    out[2] = s->seq;
    out[3] = s->flags;
    sv_put16(out + 4, s->faults_live);
    sv_put16(out + 6, s->faults_latched);
    sv_put16(out + 8, s->warns);
    out[10] = s->first_fault;
    out[11] = s->test_result;
    for (int i = 0; i < SV_CH_COUNT; i++) sv_put16(out + 12 + 2 * i, s->mv[i]);
    sv_put16(out + 28, s->vdd_mv);
    sv_put16(out + 30, s->trim_target_mv);
    sv_put16(out + 32, s->trim_permille);
    sv_put16(out + 34, s->clamp_on_permille);
    sv_seal(out, SV_REG_STATUS, SV_STATUS_LEN);
}

static inline int sv_status_decode(const uint8_t* in, uint32_t n, SvStatus* s) {
    int e = sv_check(in, n, SV_REG_STATUS, SV_STATUS_LEN);
    if (e != SV_OK) return e;
    s->seq = in[2];
    s->flags = in[3];
    s->faults_live = sv_get16(in + 4);
    s->faults_latched = sv_get16(in + 6);
    s->warns = sv_get16(in + 8);
    s->first_fault = in[10];
    s->test_result = in[11];
    for (int i = 0; i < SV_CH_COUNT; i++) s->mv[i] = sv_get16(in + 12 + 2 * i);
    s->vdd_mv = sv_get16(in + 28);
    s->trim_target_mv = sv_get16(in + 30);
    s->trim_permille = sv_get16(in + 32);
    s->clamp_on_permille = sv_get16(in + 34);
    return SV_OK;
}

// ---- LIMITS -----------------------------------------------------------------

static inline void sv_limits_encode(const SvLimits* s, uint8_t out[SV_LIMITS_LEN]) {
    for (int i = 0; i < 4; i++) {
        sv_put16(out + 2 + 4 * i, s->lo_mv[i]);
        sv_put16(out + 4 + 4 * i, s->hi_mv[i]);
    }
    sv_put16(out + 18, s->bus_ov_mv);
    sv_put16(out + 20, s->shunt_warm_mv);
    sv_put16(out + 22, s->shunt_trip_mv);
    sv_put16(out + 24, s->heartbeat_timeout_ms);
    sv_seal(out, SV_REG_LIMITS, SV_LIMITS_LEN);
}

static inline int sv_limits_decode(const uint8_t* in, uint32_t n, SvLimits* s) {
    int e = sv_check(in, n, SV_REG_LIMITS, SV_LIMITS_LEN);
    if (e != SV_OK) return e;
    for (int i = 0; i < 4; i++) {
        s->lo_mv[i] = sv_get16(in + 2 + 4 * i);
        s->hi_mv[i] = sv_get16(in + 4 + 4 * i);
    }
    s->bus_ov_mv = sv_get16(in + 18);
    s->shunt_warm_mv = sv_get16(in + 20);
    s->shunt_trip_mv = sv_get16(in + 22);
    s->heartbeat_timeout_ms = sv_get16(in + 24);
    return SV_OK;
}

// Field by field, keep the tighter of the two: a higher low edge, a lower high
// edge, a lower bus ceiling, a higher NTC pin voltage (cooler), a shorter
// heartbeat. A request can never loosen the base.
static inline SvLimits sv_limits_tighten(const SvLimits* base, const SvLimits* req) {
    SvLimits r = *base;
    for (int i = 0; i < 4; i++) {
        if (req->lo_mv[i] > r.lo_mv[i]) r.lo_mv[i] = req->lo_mv[i];
        if (req->hi_mv[i] < r.hi_mv[i]) r.hi_mv[i] = req->hi_mv[i];
    }
    if (req->bus_ov_mv < r.bus_ov_mv) r.bus_ov_mv = req->bus_ov_mv;
    if (req->shunt_warm_mv > r.shunt_warm_mv) r.shunt_warm_mv = req->shunt_warm_mv;
    if (req->shunt_trip_mv > r.shunt_trip_mv) r.shunt_trip_mv = req->shunt_trip_mv;
    if (req->heartbeat_timeout_ms != 0 && req->heartbeat_timeout_ms < r.heartbeat_timeout_ms)
        r.heartbeat_timeout_ms = req->heartbeat_timeout_ms;
    return r;
}

// ---- HEARTBEAT and COMMAND (P4 to monitor) ----------------------------------

static inline void sv_heartbeat_encode(uint8_t counter, uint8_t out[SV_HEARTBEAT_LEN]) {
    out[2] = counter;
    sv_seal(out, SV_REG_HEARTBEAT, SV_HEARTBEAT_LEN);
}

static inline int sv_heartbeat_decode(const uint8_t* in, uint32_t n, uint8_t* counter) {
    int e = sv_check(in, n, SV_REG_HEARTBEAT, SV_HEARTBEAT_LEN);
    if (e == SV_OK) *counter = in[2];
    return e;
}

static inline void sv_command_encode(uint8_t op, uint16_t arg, uint8_t out[SV_COMMAND_LEN]) {
    out[2] = op;
    sv_put16(out + 3, arg);
    sv_seal(out, SV_REG_COMMAND, SV_COMMAND_LEN);
}

static inline int sv_command_decode(const uint8_t* in, uint32_t n, uint8_t* op, uint16_t* arg) {
    int e = sv_check(in, n, SV_REG_COMMAND, SV_COMMAND_LEN);
    if (e == SV_OK) {
        *op = in[2];
        *arg = sv_get16(in + 3);
    }
    return e;
}
