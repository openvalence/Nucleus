#pragma once

// monitor_core -- the board monitor's decisions: scaling, window compares with
// debounce, fault latching, the P4 heartbeat watchdog, the regen clamp trim,
// the clamp test, the input-stage cut, and the STATUS snapshot
// Constraints:
// - Hardware-free C that is also valid C++: the native suite
//   test/native/test_supervisor compiles it as C++. No globals; all state is
//   in one McState owned by the main loop. The I2C ISR never touches it.
// - mc_tick() runs once per ADC scan, nominally every 1 ms. Every time
//   constant below is a count of those scans.
// - Integer math only. RV32EC has no multiply or divide instruction; the
//   products here go through libgcc and stay inside 32 bits.
// - The trim can only LOWER the clamp threshold below its hardware ceiling,
//   and the threshold it aims at is always the latched supply plus a margin.
//   A mistake here lets the supply feed the regen load, so the feeding check
//   releases the trim on its own, without the P4.
// - The input-stage cut (mc_cut) follows its conditions: it is high while one
//   holds, the board's hold-up carries it ~1.3-10 s past that, and a P4
//   request holds until reset. A cut on bus power alone ends this chip's
//   supply too, so on that board every cut is a power cycle.
// See: docs/supervisor.md, flagship_p4/src/system/Supervisor.h, bd val-091.19

#include <stdint.h>

#include "system/Supervisor.h"

#define MC_FW_MAJOR 0
#define MC_FW_MINOR 1
#define MC_FW_PATCH 0

// ---- scaling ----------------------------------------------------------------

#define MC_ADC_FULL      1023u   // 10-bit, referenced to VDD (+3V3_SYS)
#define MC_VREFINT_MV    1200u   // datasheet typical; McState.vrefint_mv is the calibration knob
#define MC_NTC_PULLUP_MV 3300u   // SHUNT_TEMP is ratiometric to VDD; report it at a nominal 3.3 V

// ---- regen clamp trim (SPEC 2026-09-23 board-monitor row) ---------------------

// Threshold at duty d: VTH0 - GAIN * d * VDD, from R501 348k, R502 12.1k,
// R503 3.01M, R1118 10k + R1119 47k and the TLV431's 1.24 V. SPEC rounds VTH0
// to 44.5 V; the nominal parts give 44.62 V.
#define MC_TRIM_VTH0_MV      44616u
#define MC_TRIM_GAIN_X1000   6105u    // 348k / 57k
#define MC_TRIM_MARGIN_MV    3000u    // LTC4364 fitted
#define MC_TRIM_MARGIN_BUDGET_MV 1200u // F1 on its third pad
#define MC_TEST_BELOW_BUS_MV 2000u
#define MC_TEST_MIN_MS       30u
#define MC_TEST_MAX_MS       50u
#define MC_TEST_GRACE_TICKS  50u     // the trim filter settles back (~8 ms time constant)

// ---- timing, in scans -----------------------------------------------------------

#define MC_BOOT_TICKS        100u
#define MC_RAIL_TICKS        10u
#define MC_BUS_OV_TICKS      5u
#define MC_INPUT_TICKS       200u
#define MC_SHUNT_TICKS       100u
#define MC_FEED_TICKS        2u
#define MC_STUCK_TICKS       100u
#define MC_STABLE_TICKS      200u
#define MC_CLAMP_WINDOW      1000u
#define MC_FLT_T_TICKS       2u       // glitch filter only: U2 already turned its FETs off
#define MC_CUT_VIN_TICKS     10u

// ---- levels ---------------------------------------------------------------------

#define MC_VDD_LO_MV         3000u
#define MC_VDD_HI_MV         3600u
#define MC_SUPPLY_MIN_MV     20000u   // bucks start at 20 V (SPEC 2026-09-22 rails row)
#define MC_VIN_ABSENT_MV     2000u
#define MC_INPUT_DROP_MV     3000u
#define MC_STABLE_BAND_MV    300u
#define MC_CLAMP_ON_MV       1000u    // CLAMP_MON is the Q501 gate x 3.3/13.3: ~3 V on, 0 off
#define MC_FEED_MARGIN_MV    500u
// Input-stage cut levels (SPEC 2026-10-13 MON_CUT row). VIN_RAW: a second
// opinion behind U16's 39.5 V. SHUNT_TEMP: 125 C on TH501 (pin mV, LOWER is
// hotter), above the 110 C motor cut, so heat left over from regen that
// already tripped the motor does not cut the input; only a load still heating
// with no regen reaches it.
#define MC_VIN_CUT_MV        40000u
#define MC_SHUNT_CUT_MV      173u

enum { MC_TRIM_RELEASED = 0, MC_TRIM_DRIVEN = 1 };

typedef struct {
    uint16_t raw[SV_CH_COUNT];
    uint16_t raw_vrefint;
    uint8_t  pump_flt_low;
    uint8_t  msw_flt_t_low;       // U2 FLT_T asserted (PD0 low)
} McInputs;

typedef struct {
    SvLimits base, active;
    uint16_t vrefint_mv;

    uint16_t mv[SV_CH_COUNT];
    uint16_t vdd_mv;
    uint32_t ticks;
    uint8_t  seq;

    uint16_t cnt_vdd, cnt_rail[4], cnt_bus_ov, cnt_input, cnt_hot, cnt_warm;
    uint16_t cnt_feed, cnt_stuck;
    uint16_t cnt_flt_t, cnt_cut_vin, cnt_cut_shunt;
    uint16_t faults_live, faults_latched, warns;
    uint8_t  first_fault;
    uint8_t  link_drops;

    uint8_t  hb_armed, hb_last;
    uint16_t hb_age_ms;

    uint8_t  budget;
    uint8_t  trim_latched, trim_held, trim_backoff;
    uint16_t stable_ref_mv, stable_ticks;
    uint16_t supply_mv;           // latched supply the trim is built on
    uint16_t trim_target_mv;
    uint16_t trim_permille;
    uint8_t  trim_mode;

    uint16_t clamp_on_count, clamp_window_count, clamp_on_permille;

    uint8_t  test_ms_left, test_len, test_seen, test_result, test_grace;
    uint16_t test_target_mv;

    uint8_t  cut_req;             // SV_CMD_INPUT_CUT arrived; never cleared
    uint8_t  cut_live, cut_latched;   // SV_CUT_*
} McState;

void mc_init(McState* s);
void mc_tick(McState* s, const McInputs* in);
void mc_on_heartbeat(McState* s, uint8_t counter);
void mc_on_command(McState* s, uint8_t op, uint16_t arg);
void mc_on_limits(McState* s, const SvLimits* req);
void mc_on_link_drop(McState* s);
void mc_status(const McState* s, SvStatus* out);

// Duty for a clamp threshold of target_mv at this VDD; 0 when the target is
// at or above the hardware ceiling.
uint16_t mc_trim_permille_for(uint16_t target_mv, uint16_t vdd_mv);

static inline int mc_fault_n(const McState* s) {
    return ((s->faults_live | s->faults_latched) & SV_FAULT_MASK) != 0;
}

// MON_CUT: the only thing that may drive PC5 high.
static inline int mc_cut(const McState* s) {
    return s->cut_live != 0;
}
