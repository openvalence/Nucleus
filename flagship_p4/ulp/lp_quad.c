// lp_quad -- LP core quadrature emitter: renders edges from a velocity the HP
// core hands it, and counts them
// Constraints:
// - Runs free on the LP core and NEVER returns. The HP core steers it only by
//   writing g_step_q8 and g_dir; it never commands an edge.
// - One store per edge. Gray coding changes exactly one line per transition,
//   so every edge is a single w1ts or w1tc write, never a read-modify-write.
// - Shared words are 32-bit ON PURPOSE. There is no lock between HP and LP and
//   the LP core reads them mid-stride, so a 64-bit value would tear.
// - g_step_q8 is cycles-per-edge in Q8. The HP core owns that arithmetic; the
//   PER-EDGE path has one shift and no divide. The mid-wait re-steer path
//   divides once, at the steering rate, to re-price the rest of the interval.
// - g_pos IS THE MACHINE'S POSITION, in steps. This core is its ONLY writer;
//   the HP core reads it and keeps its own origin, so the word never needs a
//   write from the other side.
// - LP core clock is NOT assumed. Everything here is in cycles; converting to
//   seconds is the HP core's job and the rate is measured, not declared.
// - The FINE wait loop is the emitter's measured 125 ns floor. Nothing goes in
//   it; the coarse loop above it exists so nothing has to.
// See: bd val-091.4, sd-1bi.3, .claude/rules/motion-control.md

#include <stdint.h>
#include "sdkconfig.h"
#include "ulp_lp_core_gpio.h"
#include "hal/rtc_io_ll.h"
#include "riscv/csr.h"

// LPG15 and LPG12: the mirrored through-holes opposite the PARLIO pair, so one
// probe setup measures both emitters and the comparison stays honest. LPG15 is
// also LPRXD -- driving it costs the LP core its serial INPUT, which bring-up
// does not need. Move the emitter pin before giving up the console.
#define PIN_A LP_IO_NUM_15
#define PIN_B LP_IO_NUM_12

// How close to a deadline the re-steer loop stops looking. Inside this window
// the FINE loop runs alone and keeps its measured shape. 2,000 cycles = 50 us
// at 40 MHz, well under the HP sampler's 1 ms tick.
#define RESTEER_SLACK_CYCLES 2000u

// ---- HP-facing shared state -------------------------------------------------

volatile uint32_t g_step_q8 = 0;   // cycles per edge, Q8. 0 parks the emitter.
volatile uint32_t g_dir     = 1;   // 1 forward, 0 reverse
volatile uint32_t g_edges   = 0;   // free-running edge count, proof of life
volatile uint32_t g_late    = 0;   // deadlines already past on arrival
volatile uint32_t g_cycles  = 0;   // mcycle at the last edge, for clock measure
volatile int32_t  g_pos     = 0;   // POSITION TRUTH: signed edge count, steps
volatile uint32_t g_resteer = 0;   // waits cut short by a new steering word
volatile uint32_t g_catchup = 0;   // re-steers whose new period was shorter than
                                   // the wait already spent: the rate rose while
                                   // an edge was pending, so the edge is due now.
                                   // NOT a missed deadline -- g_late is that, and
                                   // conflating them makes every ramp out of rest
                                   // look like an emitter that cannot keep up.

// ---- edge table -------------------------------------------------------------

// Forward Gray walk over (B,A): 00 -> 01 -> 11 -> 10 -> 00.
// 00->01 raise A, 01->11 raise B, 11->10 drop A, 10->00 drop B.
// Reverse walks the same table backwards, which is why direction is an index
// step and not a second table.
typedef struct { volatile uint32_t *reg; uint32_t mask; } edge_t;

static edge_t g_seq[4];

int main(void)
{
    ulp_lp_core_gpio_init(PIN_A);
    ulp_lp_core_gpio_init(PIN_B);
    ulp_lp_core_gpio_output_enable(PIN_A);
    ulp_lp_core_gpio_output_enable(PIN_B);
    ulp_lp_core_gpio_set_level(PIN_A, 0);
    ulp_lp_core_gpio_set_level(PIN_B, 0);

    volatile uint32_t *set = &LP_GPIO.out_w1ts.val;
    volatile uint32_t *clr = &LP_GPIO.out_w1tc.val;
    g_seq[0].reg = set; g_seq[0].mask = 1u << PIN_A;
    g_seq[1].reg = set; g_seq[1].mask = 1u << PIN_B;
    g_seq[2].reg = clr; g_seq[2].mask = 1u << PIN_A;
    g_seq[3].reg = clr; g_seq[3].mask = 1u << PIN_B;

    uint32_t prev   = RV_READ_CSR(mcycle);   // the PREVIOUS edge's deadline
    uint32_t acc_q8 = 0;   // sub-cycle remainder, carried, never discarded
    uint32_t i      = 0;   // index of the NEXT edge, under the direction in fwd
    uint32_t fwd    = 1;

    for (;;) {
        uint32_t step = g_step_q8;
        if (step == 0) {
            // Parked. Resync so a restart does not fire a burst of stale edges
            // catching up on deadlines that passed while stopped.
            prev   = RV_READ_CSR(mcycle);
            acc_q8 = 0;
            continue;
        }

        // A reversal is decided ONCE per edge, before the wait, so it lands on
        // an edge boundary: the line just driven is already complete and the
        // next edge is the inverse of it. i holds the next index under the OLD
        // direction, so the correction steps the index the OTHER way and every
        // store stays a real transition. Stepping it the same way on a reversal
        // rewrites a line that already sits at that level: no edge on the wire,
        // one phantom count in g_pos.
        uint32_t d = g_dir;
        if (d != fwd) {
            i   = (i + (d ? 3u : 1u)) & 3u;
            fwd = d;
        }

        // The next deadline comes from the PREVIOUS DEADLINE, never from the
        // instant the last edge actually landed. Feeding the emitted time back
        // makes every late edge permanent and the phase walks. The ONE place
        // real time enters is a mid-wait re-steer below, where the interval is
        // re-priced rather than rebased.
        uint32_t carry    = acc_q8 + step;
        // base/per/rem describe the interval being waited out: it started at
        // `base`, a whole edge at the current rate costs `per`, and `rem` of it
        // is still owed. They are what makes a SECOND re-steer inside one wait
        // price the same remainder again instead of re-measuring it from the
        // previous edge.
        uint32_t base     = prev;
        uint32_t per      = carry >> 8;
        uint32_t rem      = per;
        uint32_t deadline = base + rem;

        // COARSE WAIT: re-read the steering words while the deadline is still
        // far off. Out of rest the plan's opening velocity is tiny and its
        // period is tens of milliseconds; without this the emitter ignores
        // every steering update until that one edge fires, and a move opens
        // with a dead zone the size of its own first period.
        uint32_t resteered = 0;
        while ((int32_t)(RV_READ_CSR(mcycle) - (deadline - RESTEER_SLACK_CYCLES)) < 0) {
            uint32_t d2 = g_dir;
            if (d2 != fwd) {
                i   = (i + (d2 ? 3u : 1u)) & 3u;
                fwd = d2;
            }
            uint32_t s2 = g_step_q8;
            if (s2 != step) {
                uint32_t at_r  = RV_READ_CSR(mcycle);
                uint32_t spent = at_r - base;
                g_resteer++;
                resteered = 1;
                step = s2;
                if (step == 0) break;
                carry = acc_q8 + step;
                // RE-PRICE THE REMAINDER, never the whole interval. The part
                // of this edge already traversed was traversed at the old
                // rate; only the fraction still owed, rem-spent out of per,
                // belongs to the new one. Rebasing the whole interval on the
                // previous deadline instead charges all of it to the new
                // period, so a rise out of a near-standstill lands the
                // deadline up to one OLD period in the past -- measured
                // 77 ms -- and the emitter then renders that debt as thousands
                // of edges at core speed, which is travel the plan never
                // commanded. NOT a shaper: no rate is limited here and the
                // commanded velocity is untouched, only the instant THIS edge
                // is due. The 64-bit divide is legal because this path runs at
                // the steering rate (~1 kHz), never per edge; the per-edge
                // path below still has no divide.
                const uint32_t want = carry >> 8;
                rem = (spent >= rem || per == 0)
                        ? 0u
                        : (uint32_t)(((uint64_t)(rem - spent) * want) / per);
                base     = at_r;
                per      = want;
                deadline = base + rem;
            }
        }
        if (step == 0) {
            prev   = RV_READ_CSR(mcycle);
            acc_q8 = 0;
            continue;
        }

        // FINE WAIT: five cycles, 125 ns at 40 MHz, and that is the emitter's
        // measured jitter floor. Nothing goes in here.
        if ((int32_t)(RV_READ_CSR(mcycle) - deadline) >= 0) {
            // Report, never adjudicate: the deadline stands either way. The two
            // counters are kept apart because only one of them is a defect.
            // A rate that ROSE while this edge was pending makes its deadline
            // land in the past by arithmetic, every ramp out of rest, however
            // fast the core is; a deadline computed at the top of the loop and
            // already past is the core failing to keep up.
            if (resteered) g_catchup++;
            else           g_late++;
        } else {
            while ((int32_t)(RV_READ_CSR(mcycle) - deadline) < 0) { }
        }

        *g_seq[i].reg = g_seq[i].mask;
        prev     = deadline;
        acc_q8   = carry & 0xFFu;
        g_cycles = deadline;
        g_pos    = g_pos + (fwd ? 1 : -1);
        g_edges++;
        i = (i + (fwd ? 1u : 3u)) & 3u;
    }
    return 0;
}
