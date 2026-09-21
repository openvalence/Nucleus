"""Capture LP-core quadrature on the Rigol DHO4204 and decode phase and polarity.

The instrument for val-091.4 item 2 and val-091.8. The bench firmware repeats
+20 / -20 / reversal every ~14.6 s.

PROBE MAP: the bench declares LPG15 = A on CH2 and LPG12 = B on CH1, and that
declaration is REVERSED against the hardware -- measured, see PROBE IDENTITY
below. A phase reading cannot notice, because a swapped probe pair and a
swapped emitter table decode identically. Pass ORIGIN and read the identity
line before trusting any polarity claim from this tool.

NEVER switch this instrument to 50 ohm and never touch the :CALibration tree;
both can destroy it (SlopDrive-32/docs/reference/rigol-dho4204-scope-guide.md).

Three measured traps are built in here, each of which produced a convincing
false result before it was caught (bd val-091.4):

1. :TRIGger:SWEep SINGle IS SILENTLY IGNORED on firmware 00.02.14, :SINGle does
   not arm, and :TRIGger:STATus? never reports TD or WAIT on this unit. There is
   no trigger handshake to wait on. The single shot is NORM sweep + :RUN + dwell
   + :STOP, and the record is validated BY ITS CONTENTS, never by the status.
2. A decode threshold taken from the record's own max/min turns an idle record
   into millions of phantom transitions, because both lines park at a held
   level between legs and the threshold lands inside the noise band. THR is
   fixed at the 1.65 V logic midpoint; an idle record decodes to zero.
3. :WAV:MODE NORM then MAX before setting any window, or a stale STAR/STOP pair
   hangs the read. Windows are 200k points, FORMat WORD.

POLARITY (val-091.8) IS READ FROM THE REVERSAL BURST, NOT FROM TIMING. A leg is
identified by its own structure, never by where it sits in the cycle:
the out and back legs are single-direction bursts of ~4174 edges, while the
reversal is the ONLY burst carrying a direction flip, and its FIRST half is
POSITIVE by construction (benchTask submits +20 mm, then 0 mm 600 ms later).
So the gray sign of the reversal's first half IS the sign a positive move
renders with: +1 means the A-leads walk 00 -> 01 -> 11 -> 10, which is what
lp_quad.c's table and ValenceMotion.cpp's dir = (v<0)?0:1 assert.

Anchoring legs by their 2.5 s spacing instead is what produced val-091.8's
false inversion: the park state between legs depends on the LP's ABSOLUTE edge
count, which includes every edge of the constant-rate startup burst, so the
phase of a park carries no leg identity at all.

Usage:  python tools/lp_leg_capture.py [window_s] [attempts]
Output: artifacts/leg_t.npy, the decoded edge times in seconds.
"""

import sys, numpy as np
from lp_scope import *


def bursts(d):
    # Split the edge train at any span longer than GAP_S; inside a burst, split
    # again at every direction flip. A burst with exactly one flip and two
    # near-equal halves is the reversal, and its FIRST half is the positive one.
    t, g, seq = d['t'], d['g'], d['seq']
    if len(g) == 0: return []
    cut = list(np.flatnonzero(np.diff(t) > GAP_S))
    out = []
    start = 0
    for end in cut + [len(g)]:
        if end <= start: start = end + 1; continue
        seg, legs, k = g[start:end], [], 0
        while k < len(seg):
            j = k
            while j < len(seg) and seg[j] == seg[k]: j += 1
            legs.append((int(seg[k]), j - k))
            k = j
        last = min(end, len(seq) - 1)
        held = int(seq[last])            # the level pair the emitter parks at
        gap = (t[last + 1] - t[last]) if last + 1 < len(t) else float('nan')
        out.append((t[start], t[last] - t[start], legs, held, gap))
        start = end + 1
    return out


WINDOW = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
TRIES = int(sys.argv[2]) if len(sys.argv) > 2 else 6
# The free-running 'edges' count at the pre-bench park, off the console. It is
# per-boot: the constant-rate startup burst is not a fixed length.
ORIGIN = int(sys.argv[3]) if len(sys.argv) > 3 else None

for attempt in range(1, TRIES + 1):
    shoot(WINDOW, WINDOW * 2.0 + 1.0)
    d = decode()
    print('attempt %d: %d samples @ %.2f us, window %.2f s -> %d transitions '
          '(gray+1 %d, gray-1 %d), ILLEGAL %d'
          % (attempt, d['n'], d['DT'] * 1e6, d['n'] * d['DT'], d['total'],
             d['fwd'], d['rev'], d['bad']))
    bs = bursts(d)
    reversal = None
    for t0, dur, legs, held, gap in bs:
        print('   burst t=%7.3f s  %7.1f ms  %-28s parks CH1=%d CH2=%d, idle %.0f ms'
              % (t0, dur * 1e3,
                 '  '.join('%+d x %d' % (s_, n_) for s_, n_ in legs),
                 (held >> 1) & 1, held & 1, gap * 1e3))
        if len(legs) == 2 and legs[0][0] == -legs[1][0] \
           and abs(legs[0][1] - legs[1][1]) < 0.1 * legs[0][1]:
            reversal = (legs, held)
    # Trap 1: the record is accepted on its CONTENTS.
    if d['total'] > 3000 and reversal is not None:
        np.save(OUT, d['t'])
        iv = np.diff(d['t']) * 1e6
        print('saved %d edge times to %s' % (len(d['t']), OUT))
        print('edge interval: min %.1f us  max %.1f us  median %.2f us'
              % (iv.min(), iv.max(), float(np.median(iv))))
        legs, held = reversal
        print('REVERSAL burst %+d x %d then %+d x %d, parks CH1=%d CH2=%d'
              % (legs[0][0], legs[0][1], legs[1][0], legs[1][1],
                 (held >> 1) & 1, held & 1))
        print('POLARITY: a POSITIVE move renders gray %+d as decoded -> %s leads '
              'on the declared probe map (want +1, A = LPG15 = CH2)'
              % (legs[0][0], 'A' if legs[0][0] == 1 else 'B'))
        # PROBE IDENTITY, the bit no phase reading can supply. Phase alone is
        # mirror-symmetric: a swapped probe pair and a swapped emitter table
        # produce the SAME decode, which is why val-091.8 could not separate
        # them. The ABSOLUTE edge count does separate them. lp_quad.c walks
        # (LPG15, LPG12) = (0,0) (1,0) (1,1) (0,1) over k = count mod 4 from the
        # LP core's first edge, so the park level pair names the pins outright
        # once k is known. k = (ORIGIN + census steps) mod 4, where ORIGIN is
        # the free-running 'edges' count at the pre-bench park, read off the
        # console. The reversal parks at census steps = -1.
        if ORIGIN is not None:
            k = (ORIGIN - 1) % 4
            hi15 = k in (1, 2)
            want = 'HIGH' if hi15 else 'LOW'
            ch = 1 if (held >> 1) & 1 else 2
            print('PROBE IDENTITY: origin %d, k=%d -> LPG15 parks %s; '
                  'measured CH1=%d CH2=%d, so LPG15 is on CH%d (declared: CH2)'
                  % (ORIGIN, k, want, (held >> 1) & 1, held & 1,
                     ch if hi15 else (2 if ch == 1 else 1)))
        print('ILLEGAL TRANSITIONS: %d' % d['bad'])
        break
else:
    print('no record carried a reversal burst')

send(':RUN'); s.close()
