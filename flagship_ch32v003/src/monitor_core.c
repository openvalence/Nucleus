// monitor_core -- the board monitor's decisions; see monitor_core.h
// Constraints:
// - Pure: no register access and no clock but mc_tick's cadence.
// - Every default limit below is a proposal until the operator rules
//   (docs/supervisor.md, "Not firm").
// See: monitor_core.h, docs/supervisor.md

#include "monitor_core.h"

// ---- tables -------------------------------------------------------------------

// Divider ratio x1000, rail mV per pin mV, from the U12 sheet of the schematic.
static const uint16_t kRatioX1000[SV_CH_COUNT] = {
    17000,   // VIN_RAW  160k / 10k
    17000,   // +BUS     160k / 10k
    4900,    // +12V     39k / 10k
    2000,    // +5V      10k / 10k
    2000,    // +5V_SYS  10k / 10k
    2000,    // +3V3_ACC 10k / 10k
    1000,    // SHUNT_TEMP pin
    1000,    // CLAMP_MON pin
};

static const uint8_t kRailCh[4] = { SV_CH_12V, SV_CH_5V, SV_CH_5V_SYS, SV_CH_3V3_ACC };
static const uint16_t kRailBit[4] = { SV_F_12V, SV_W_5V, SV_F_5V_SYS, SV_W_3V3_ACC };
static const uint8_t kRailIsFault[4] = { 1, 0, 1, 0 };

// TODO(val-091.19): operator ruling on every value in this block.
static SvLimits defaultLimits(void) {
    SvLimits l;
    l.lo_mv[0] = 10800; l.hi_mv[0] = 13200;   // +12V +/-10 %
    l.lo_mv[1] = 4500;  l.hi_mv[1] = 5500;    // +5V
    l.lo_mv[2] = 4500;  l.hi_mv[2] = 5500;    // +5V_SYS
    l.lo_mv[3] = 3000;  l.hi_mv[3] = 3600;    // +3V3_ACC
    l.bus_ov_mv = 47000;                      // above the 44.5 V clamp, below the ~49 V hardware trip
    l.shunt_warm_mv = 418;                    // TH501 10k B3435 under R512 10k: 85 C
    l.shunt_trip_mv = 238;                    // 110 C
    l.heartbeat_timeout_ms = 500;
    return l;
}

// ---- helpers ------------------------------------------------------------------

static uint16_t sat16(uint32_t v) { return (uint16_t)(v > 0xFFFFu ? 0xFFFFu : v); }

static int debounce(uint16_t* c, int cond, uint16_t n) {
    if (!cond) {
        *c = 0;
        return 0;
    }
    if (*c < n) (*c)++;
    return *c >= n;
}

static uint8_t lowestBit(uint16_t v) {
    uint8_t i = 0;
    while (!(v & 1u)) {
        v >>= 1;
        i++;
    }
    return i;
}

uint16_t mc_trim_permille_for(uint16_t target_mv, uint16_t vdd_mv) {
    if (target_mv >= MC_TRIM_VTH0_MV || vdd_mv == 0) return 0;
    // Truncation rounds the duty down, which rounds the threshold UP: the safe side.
    uint32_t vpwm_mv = (uint32_t)(MC_TRIM_VTH0_MV - target_mv) * 1000u / MC_TRIM_GAIN_X1000;
    uint32_t p = vpwm_mv * 1000u / vdd_mv;
    return (uint16_t)(p > 1000u ? 1000u : p);
}

// ---- lifecycle ----------------------------------------------------------------

void mc_init(McState* s) {
    uint8_t* b = (uint8_t*)s;
    for (uint32_t i = 0; i < sizeof(*s); i++) b[i] = 0;
    s->base = defaultLimits();
    s->active = s->base;
    s->vrefint_mv = MC_VREFINT_MV;
    s->trim_mode = MC_TRIM_RELEASED;
}

void mc_tick(McState* s, const McInputs* in) {
    s->ticks++;
    s->seq++;
    const int booting = s->ticks <= MC_BOOT_TICKS;

    // ---- scale ----
    s->vdd_mv = in->raw_vrefint
        ? sat16((uint32_t)s->vrefint_mv * MC_ADC_FULL / in->raw_vrefint) : 0;
    for (int i = 0; i < SV_CH_COUNT; i++) {
        if (i == SV_CH_SHUNT_TEMP) {
            s->mv[i] = sat16((uint32_t)in->raw[i] * MC_NTC_PULLUP_MV / MC_ADC_FULL);
        } else {
            uint32_t pin = (uint32_t)in->raw[i] * s->vdd_mv / MC_ADC_FULL;
            s->mv[i] = sat16(pin * kRatioX1000[i] / 1000u);
        }
    }
    const uint16_t vin = s->mv[SV_CH_VIN_RAW];
    const uint16_t bus = s->mv[SV_CH_BUS];

    // ---- window compares ----
    uint16_t live = booting ? SV_F_BOOT : 0;
    uint16_t warn = 0;
    if (debounce(&s->cnt_vdd, s->vdd_mv < MC_VDD_LO_MV || s->vdd_mv > MC_VDD_HI_MV, MC_RAIL_TICKS))
        live |= SV_F_VDD;
    for (int r = 0; r < 4; r++) {
        const uint16_t v = s->mv[kRailCh[r]];
        if (debounce(&s->cnt_rail[r], v < s->active.lo_mv[r] || v > s->active.hi_mv[r], MC_RAIL_TICKS)) {
            if (kRailIsFault[r]) live |= kRailBit[r];
            else warn |= kRailBit[r];
        }
    }

    if (!s->trim_latched) s->budget = vin < MC_VIN_ABSENT_MV && bus >= MC_SUPPLY_MIN_MV;
    if (debounce(&s->cnt_input,
                 !s->budget && vin >= MC_SUPPLY_MIN_MV && (uint32_t)bus + MC_INPUT_DROP_MV < vin,
                 MC_INPUT_TICKS))
        live |= SV_F_INPUT_STAGE;
    if (debounce(&s->cnt_bus_ov, bus > s->active.bus_ov_mv, MC_BUS_OV_TICKS)) live |= SV_F_BUS_OV;

    // The NTC pin voltage FALLS as the regen load heats.
    const uint16_t ntc = s->mv[SV_CH_SHUNT_TEMP];
    if (debounce(&s->cnt_hot, ntc <= s->active.shunt_trip_mv, MC_SHUNT_TICKS)) live |= SV_F_SHUNT_HOT;
    const int warm = debounce(&s->cnt_warm, ntc <= s->active.shunt_warm_mv, MC_SHUNT_TICKS);
    if (warm) warn |= SV_W_SHUNT_WARM;
    if (in->pump_flt_low) warn |= SV_W_PUMP_FLT;

    // ---- heartbeat ----
    if (s->hb_armed) {
        if (s->hb_age_ms < 0xFFFFu) s->hb_age_ms++;
        if (s->hb_age_ms > s->active.heartbeat_timeout_ms) live |= SV_F_HEARTBEAT;
    }

    // ---- clamp activity ----
    const int clamp_on = s->mv[SV_CH_CLAMP_MON] >= MC_CLAMP_ON_MV;
    s->clamp_on_count += (uint16_t)clamp_on;
    if (++s->clamp_window_count >= MC_CLAMP_WINDOW) {
        s->clamp_on_permille = (uint16_t)((uint32_t)s->clamp_on_count * 1000u / MC_CLAMP_WINDOW);
        s->clamp_on_count = 0;
        s->clamp_window_count = 0;
    }

    // ---- trim latch: from a supply that has held still ----
    const uint16_t supply = s->budget ? bus : vin;
    if (!s->trim_latched && !s->trim_held && !booting) {
        const uint16_t d = supply > s->stable_ref_mv ? supply - s->stable_ref_mv : s->stable_ref_mv - supply;
        if (supply >= MC_SUPPLY_MIN_MV && d <= MC_STABLE_BAND_MV) {
            if (++s->stable_ticks >= MC_STABLE_TICKS) {
                s->supply_mv = supply;
                s->trim_target_mv = sat16((uint32_t)supply
                    + (s->budget ? MC_TRIM_MARGIN_BUDGET_MV : MC_TRIM_MARGIN_MV));
                s->trim_latched = 1;
            }
        } else {
            s->stable_ref_mv = supply;
            s->stable_ticks = 0;
        }
    }

    // ---- the clamp may only conduct on regen ----
    // No regen means +BUS is not above the supply. Conducting then means the
    // supply is feeding the regen load: back the trim off to the ceiling at
    // once, and call it stuck if it keeps conducting there.
    const int no_regen = s->budget
        ? (s->trim_latched && (uint32_t)bus <= (uint32_t)s->supply_mv + MC_FEED_MARGIN_MV)
        : ((uint32_t)bus <= (uint32_t)vin + MC_FEED_MARGIN_MV);
    const int testing = s->test_ms_left > 0 || s->test_grace > 0;
    if (!s->test_ms_left && s->test_grace) s->test_grace--;
    if (debounce(&s->cnt_feed, clamp_on && no_regen && !testing, MC_FEED_TICKS)) s->trim_backoff = 1;
    const int at_ceiling = s->trim_mode == MC_TRIM_RELEASED || s->trim_permille == 0;
    if (debounce(&s->cnt_stuck, clamp_on && no_regen && !testing && at_ceiling, MC_STUCK_TICKS))
        live |= SV_F_CLAMP_STUCK;
    if (s->trim_backoff) warn |= SV_W_CLAMP_FEEDING;

    // ---- clamp test ----
    if (s->test_ms_left) {
        if (clamp_on && s->test_ms_left <= s->test_len / 2) s->test_seen = 1;
        if (--s->test_ms_left == 0) {
            s->test_result = s->test_seen ? SV_TEST_PASS : SV_TEST_FAIL;
            s->test_grace = MC_TEST_GRACE_TICKS;
        }
    }

    // ---- trim output ----
    if (s->test_ms_left) {
        s->trim_mode = MC_TRIM_DRIVEN;
        s->trim_permille = mc_trim_permille_for(s->test_target_mv, s->vdd_mv);
    } else if (s->trim_held || !s->trim_latched) {
        s->trim_mode = MC_TRIM_RELEASED;
        s->trim_permille = 0;
    } else if (warm || s->trim_backoff) {
        s->trim_mode = MC_TRIM_DRIVEN;
        s->trim_permille = 0;
    } else {
        s->trim_mode = MC_TRIM_DRIVEN;
        s->trim_permille = mc_trim_permille_for(s->trim_target_mv, s->vdd_mv);
    }

    // ---- latch ----
    if (!booting) {
        const uint16_t fresh = live & (uint16_t)~s->faults_latched & (uint16_t)(SV_FAULT_MASK & ~SV_F_BOOT);
        if (fresh) {
            if (!s->faults_latched) s->first_fault = (uint8_t)(lowestBit(fresh) + 1);
            s->faults_latched |= fresh;
        }
    }
    if (s->link_drops) warn |= SV_W_LINK_DROP;
    s->faults_live = live;
    s->warns = warn;
}

// ---- link handlers --------------------------------------------------------------

void mc_on_heartbeat(McState* s, uint8_t counter) {
    // A repeated counter is a stuck writer, not a live P4: it does not feed.
    if (!s->hb_armed || counter != s->hb_last) s->hb_age_ms = 0;
    s->hb_armed = 1;
    s->hb_last = counter;
}

void mc_on_command(McState* s, uint8_t op, uint16_t arg) {
    switch (op) {
        case SV_CMD_CLEAR_LATCHED:
            s->faults_latched &= s->faults_live;
            if (!s->faults_latched) s->first_fault = 0;
            s->link_drops = 0;
            break;
        case SV_CMD_CLAMP_TEST: {
            const uint16_t bus = s->mv[SV_CH_BUS];
            if (!s->trim_latched || s->trim_held || s->trim_backoff || mc_fault_n(s)
                || bus < MC_SUPPLY_MIN_MV) {
                s->test_result = SV_TEST_FAIL;
                break;
            }
            const uint16_t ms = arg < MC_TEST_MIN_MS ? MC_TEST_MIN_MS
                              : arg > MC_TEST_MAX_MS ? MC_TEST_MAX_MS : arg;
            s->test_target_mv = (uint16_t)(bus - MC_TEST_BELOW_BUS_MV);
            s->test_len = (uint8_t)ms;
            s->test_ms_left = (uint8_t)ms;
            s->test_seen = 0;
            s->test_result = SV_TEST_NONE;
            break;
        }
        case SV_CMD_TRIM_RELEASE:
            s->trim_held = 1;
            if (s->test_ms_left) {
                s->test_ms_left = 0;
                s->test_result = SV_TEST_FAIL;
                s->test_grace = MC_TEST_GRACE_TICKS;
            }
            break;
        case SV_CMD_TRIM_RELATCH:
            s->trim_held = 0;
            s->trim_latched = 0;
            s->trim_backoff = 0;
            s->stable_ticks = 0;
            s->cnt_feed = 0;
            s->cnt_stuck = 0;
            break;
        default:
            mc_on_link_drop(s);
            break;
    }
}

void mc_on_limits(McState* s, const SvLimits* req) {
    s->active = sv_limits_tighten(&s->base, req);
}

void mc_on_link_drop(McState* s) {
    if (s->link_drops < 0xFFu) s->link_drops++;
}

void mc_status(const McState* s, SvStatus* o) {
    o->seq = s->seq;
    o->flags = (uint8_t)((mc_fault_n(s) ? SV_FLAG_FAULT_N : 0)
                       | (s->hb_armed ? SV_FLAG_HB_ARMED : 0)
                       | (s->trim_latched ? SV_FLAG_TRIM_LATCH : 0)
                       | (s->budget ? SV_FLAG_BUDGET : 0)
                       | (s->trim_held ? SV_FLAG_TRIM_HELD : 0)
                       | (s->test_ms_left ? SV_FLAG_CLAMP_TEST : 0));
    o->faults_live = s->faults_live;
    o->faults_latched = s->faults_latched;
    o->warns = s->warns;
    o->first_fault = s->first_fault;
    o->test_result = s->test_result;
    for (int i = 0; i < SV_CH_COUNT; i++) o->mv[i] = s->mv[i];
    o->vdd_mv = s->vdd_mv;
    o->trim_target_mv = s->trim_permille == 0 ? 0
                      : s->test_ms_left ? s->test_target_mv : s->trim_target_mv;
    o->trim_permille = s->trim_permille;
    o->clamp_on_permille = s->clamp_on_permille;
}
