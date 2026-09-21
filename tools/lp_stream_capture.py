"""Capture streamed motion on the DHO4204 and compare it to the sine the probe sent.
The instrument for val-091.11: valence_probe.py --stream drives 0x2100 with
target = 0.5 + 0.35 sin(2 pi 0.8 t) over the configured window, at 50 Hz.
This decodes the quadrature edges to a signed position, fits a sine at that
frequency, and reports amplitude, phase, residual and edge-interval stats.
Ceilings shape the rendered curve (a 250 mm window puts the peak demand at
440 mm/s, past the input speed ceiling), so the AMPLITUDE reported here is
what the engine rendered, and only a window small enough to keep the demand
under the ceilings should reproduce the probe's 0.35 span.
Usage:  python tools/lp_stream_capture.py [window_s] [freq_hz]
Output: artifacts/stream_t.npy (edge times), artifacts/stream.png
"""
import sys, os
import numpy as np
from lp_scope import *   # noqa: F401,F403 -- s, send, shoot, decode, ART, THR

STEPS_PER_MM = 208.608   # ValenceMotion.cpp kStepsPerMm
WINDOW = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
FREQ = float(sys.argv[2]) if len(sys.argv) > 2 else 0.8
OUT_T = os.path.join(ART, 'stream_t.npy')
OUT_PNG = os.path.join(ART, 'stream.png')

shoot(WINDOW, WINDOW * 2.0 + 1.0)
d = decode()
t, g = d['t'], d['g']
print('%d samples @ %.2f us, window %.2f s -> %d transitions (gray+1 %d, gray-1 %d), ILLEGAL %d'
      % (d['n'], d['DT'] * 1e6, d['n'] * d['DT'], d['total'], d['fwd'], d['rev'], d['bad']))
if len(g) < 100:
    print('no stream in the record'); send(':RUN'); s.close(); sys.exit(1)

pos = np.concatenate([[0.0], np.cumsum(g)]) / STEPS_PER_MM   # mm, relative to the first edge
tt = t[:len(pos)]
# Least-squares sine at the commanded frequency: pos = c + a sin(wt) + b cos(wt)
w = 2 * np.pi * FREQ
M = np.column_stack([np.ones_like(tt), np.sin(w * tt), np.cos(w * tt)])
c, a, b = np.linalg.lstsq(M, pos, rcond=None)[0]
amp = float(np.hypot(a, b)); fit = M @ (c, a, b); rms = float(np.sqrt(np.mean((pos - fit) ** 2)))
iv = np.diff(t) * 1e6
v = np.gradient(pos, tt)
print('fit @ %.2f Hz: amplitude %.2f mm (peak-to-peak %.2f mm), RMS residual %.3f mm, '
      'span rendered %.2f mm' % (FREQ, amp, 2 * amp, rms, pos.max() - pos.min()))
print('peak |v| %.1f mm/s; edge interval min %.1f us  max %.1f us  median %.2f us'
      % (np.abs(v).max(), iv.min(), iv.max(), float(np.median(iv))))
np.save(OUT_T, t)
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax[0].plot(tt, pos, lw=0.8, label='rendered (quad decode)')
ax[0].plot(tt, fit, lw=0.8, ls='--', label='sine fit %.2f Hz, %.1f mm' % (FREQ, amp))
ax[0].set_ylabel('mm'); ax[0].legend()
ax[1].plot(tt, v, lw=0.6); ax[1].set_ylabel('mm/s'); ax[1].set_xlabel('s')
fig.tight_layout(); fig.savefig(OUT_PNG, dpi=120)
print('saved', OUT_T, 'and', OUT_PNG)
send(':RUN'); s.close()
