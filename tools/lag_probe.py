#!/usr/bin/env python3
"""lag_probe -- measure the stream-to-rendered-position lag on a Valence hub.

Streams a sine on motion-input (0x2100) while logging every motion (0x1100)
STATE frame, then fits both series at the drive frequency and reports the phase
lag in milliseconds plus the amplitude ratio.

THREE NUMBERS, AND THEY ARE NOT THE SAME LAG:
  raw -> pos   the ENGINE's own chase lag. Both series are read out of the SAME
               frame with one timestamp, so network and scheduling cancel.
  sent -> pos  adds the client-to-hub network hop and the hub's scheduling of
               the sample into the arbiter.
  sent -> rx   the TELEMETRY path back: how stale a frame already is when it
               lands here. This is the half a display sits on top of.

Constraints:
- READ-ONLY against SlopSync: it imports tools/slopsync_probe.py from the
  sibling checkout for the wire primitives and never edits it.
- Pick a window the demand fits INSIDE. A sine whose peak velocity exceeds the
  input speed ceiling is shaped by the ceiling, and the phase you measure is
  the clamp's, not the engine's.
- Arrival time is a HINT, never a timeline: the network batches STATE frames,
  so `sent -> rx` is reported as a median and a p95, never interpolated
  against (SlopDrive-32 webui.md T18).

See: bd val-091.11
"""

import argparse
import math
import os
import sys
import time

_SLOPSYNC_TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "..", "SlopSync", "tools")
sys.path.insert(0, os.path.abspath(_SLOPSYNC_TOOLS))

import slopsync_probe as sp  # noqa: E402
import websocket  # noqa: E402


def fit_sine(ts, ys, freq_hz):
    """Least-squares fit y = a*cos(wt) + b*sin(wt) + c at a KNOWN frequency.

    Linear in (a, b, c), so this is a 3x3 normal-equation solve and needs no
    numpy. Returns (amplitude, phase_rad, mean). Phase is atan2(-b, a), the
    lag convention: y = A*cos(wt + phase) + c.
    """
    w = 2.0 * math.pi * freq_hz
    n = len(ts)
    if n < 8:
        return None
    t0 = ts[0]
    sc = ss = scc = sss = scs = sy = syc = sys_ = 0.0
    for t, y in zip(ts, ys):
        c = math.cos(w * (t - t0))
        s = math.sin(w * (t - t0))
        sc += c; ss += s
        scc += c * c; sss += s * s; scs += c * s
        sy += y; syc += y * c; sys_ += y * s
    # Normal equations for [a, b, c] against [cos, sin, 1].
    m = [[scc, scs, sc],
         [scs, sss, ss],
         [sc,  ss,  float(n)]]
    v = [syc, sys_, sy]
    # Gaussian elimination, 3x3, partial pivot.
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(m[r][i]))
        if abs(m[p][i]) < 1e-12:
            return None
        m[i], m[p] = m[p], m[i]
        v[i], v[p] = v[p], v[i]
        for r in range(i + 1, 3):
            f = m[r][i] / m[i][i]
            for k in range(i, 3):
                m[r][k] -= f * m[i][k]
            v[r] -= f * v[i]
    x = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        acc = v[i] - sum(m[i][k] * x[k] for k in range(i + 1, 3))
        x[i] = acc / m[i][i]
    a, b, dc = x
    return math.hypot(a, b), math.atan2(-b, a), dc


def lag_ms(phase_ref, phase_sig, freq_hz):
    """Positive = the signal TRAILS the reference, in milliseconds."""
    d = phase_sig - phase_ref
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    # A trailing signal has a MORE NEGATIVE phase under y = A*cos(wt + phase).
    return -d / (2.0 * math.pi * freq_hz) * 1000.0


def pct(xs, q):
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * (len(s) - 1)))]


def run(args):
    token = sp.mint_uitoken(args.ip)
    if not token:
        print("FAIL: no /uitoken -- a watch-tier session cannot publish a stream. "
              "Close any SlopDeck tab (it retries the mint in a loop and 429s us).")
        return 1
    url = "ws://%s:%d/" % (args.ip, args.port)
    ws = websocket.create_connection(url, subprotocols=[sp.WS_SUBPROTOCOL], timeout=5.0)
    inst = os.urandom(8)
    sp.send_frame(ws, sp.FRAME["HELLO"], 0,
                  sp.build_hello("probe", "lag_probe.py", inst,
                                 publishes=[(sp.CH_MOTION_INPUT, 100.0)], token=token))
    welcome = None
    deadline = time.time() + 5.0
    while time.time() < deadline:
        got = sp.recv_frame(ws, deadline)
        if got is None:
            break
        if got[0]["type"] == sp.FRAME["WELCOME"]:
            welcome = sp.cb_decode_full(got[1])
            break
    if welcome is None:
        print("FAIL: no WELCOME")
        return 1
    grants = welcome.get(sp.K["grants"]) or []
    if not grants:
        print("FAIL: motion-input publish wish not granted (watch tier?)")
        return 1
    sp.send_frame(ws, sp.FRAME["CATALOG_READY"], 0, welcome.get(sp.K["catalog_etag"], b""))

    # The window and the fake home go through the SAME session, so the run is
    # one command and the hub state it measures is the state it set.
    iid = 1
    if args.force_home is not None:
        sp.send_frame(ws, sp.FRAME["INTENT"], 0x3101,
                      sp.build_intent(0x3101, iid,
                                      [(1, sp.cb_uint(2)), (2, sp.cb_f32(args.force_home))]))
        iid += 1
    if args.window is not None:
        lo, hi = args.window
        sp.send_frame(ws, sp.FRAME["INTENT"], 0x3000,
                      sp.build_intent(0x3000, iid,
                                      [(1, sp.cb_f32(lo)), (2, sp.cb_f32(hi))]))
        iid += 1
        args.span_mm = hi - lo
    # Let both echoes and the config push to the arbiter land before measuring.
    t_settle = time.time() + 1.0
    while time.time() < t_settle:
        if sp.recv_frame(ws, t_settle) is None:
            break

    sp.send_frame(ws, sp.FRAME["SUBSCRIBE"], 0,
                  sp.build_subscribe([(sp.CH_MOTION, args.state_rate, 2)]))

    off = sp.estimate_hub_offset(ws, 5.0)
    if off is None:
        print("FAIL: no CLOCK reply")
        return 1
    offset_us, rtt_us = off
    print("clock offset %+d us, rtt %d us" % (offset_us, rtt_us))

    sent = []   # (t_local_s, target_norm)
    rx = []     # (t_local_s, pos_mm, tgt_mm, raw_mm, t_hub_send_estimate_s)

    period = 1.0 / args.rate
    t_start = time.time()
    t_end = t_start + args.seconds
    next_tx = t_start
    while time.time() < t_end:
        now = time.time()
        if now >= next_tx:
            next_tx += period
            t = now - t_start
            target = args.center + args.amp * math.sin(2.0 * math.pi * args.freq * t)
            vel = args.amp * 2.0 * math.pi * args.freq * math.cos(2.0 * math.pi * args.freq * t)
            t_base = (sp.client_now_us() + offset_us) & 0xFFFFFFFF
            sp.send_frame(ws, sp.FRAME["STREAM"], sp.CH_MOTION_INPUT,
                          sp.encode_stream_bundle(t_base, [(0, target, vel)]))
            sent.append((now, target))
        # Drain whatever is waiting without blocking past the next send.
        got = sp.recv_frame(ws, min(next_tx, t_end))
        if got is None:
            continue
        hdr, payload = got
        if hdr["type"] == sp.FRAME["STATE"] and hdr["channel"] == sp.CH_MOTION:
            try:
                d = sp.decode_motion_state(payload)
            except ValueError:
                continue
            rx.append((time.time(), d["pos_mm"], d["tgt_mm"], d.get("raw_mm", float("nan"))))
    sp.send_frame(ws, sp.FRAME["GOODBYE"], 0, sp.build_goodbye(0))
    ws.close()

    # ---- fits ---------------------------------------------------------------
    # Drop the first second: the cold-start plan is a POSITIONING move to the
    # stream's opening position, not content, and fitting it in would measure
    # that instead.
    cut = t_start + 1.0
    rx_w = [r for r in rx if r[0] >= cut]
    sent_w = [s for s in sent if s[0] >= cut]
    print("samples sent=%d, 0x1100 frames rx=%d in %.1fs (%.1f Hz effective)"
          % (len(sent), len(rx), args.seconds, len(rx) / max(args.seconds, 1e-9)))
    if len(rx_w) < 16:
        print("FAIL: too few 0x1100 frames to fit")
        return 1

    ts = [r[0] for r in rx_w]
    f_pos = fit_sine(ts, [r[1] for r in rx_w], args.freq)
    f_tgt = fit_sine(ts, [r[2] for r in rx_w], args.freq)
    f_raw = fit_sine(ts, [r[3] for r in rx_w], args.freq)
    f_sent = fit_sine([s[0] for s in sent_w], [s[1] for s in sent_w], args.freq)
    if not (f_pos and f_raw and f_sent):
        print("FAIL: fit did not converge")
        return 1

    span = args.span_mm
    print("")
    print("fit @ %.2f Hz  (amplitude mm / phase rad / mean mm)" % args.freq)
    for name, f in (("sent(norm)", f_sent), ("raw", f_raw), ("tgt", f_tgt), ("pos", f_pos)):
        if f:
            amp = f[0] * (span if name == "sent(norm)" else 1.0)
            dc = f[2] * (span if name == "sent(norm)" else 1.0)
            print("  %-11s A=%7.3f  ph=%+7.4f  dc=%7.3f" % (name, amp, f[1], dc))
    print("")
    print("ENGINE lag  raw -> pos : %+7.1f ms   amplitude ratio %.3f"
          % (lag_ms(f_raw[1], f_pos[1], args.freq), f_pos[0] / max(f_raw[0], 1e-9)))
    print("WIRE+engine sent-> pos : %+7.1f ms   amplitude ratio %.3f"
          % (lag_ms(f_sent[1], f_pos[1], args.freq),
             f_pos[0] / max(f_sent[0] * span, 1e-9)))
    if f_tgt:
        print("PLAN aim    raw -> tgt : %+7.1f ms   amplitude ratio %.3f"
              % (lag_ms(f_raw[1], f_tgt[1], args.freq), f_tgt[0] / max(f_raw[0], 1e-9)))

    # Telemetry staleness: for each received frame, how long since the sent
    # sample whose target is closest in VALUE is meaningless -- instead report
    # the inter-arrival distribution, which is what a display actually sees.
    gaps = [b[0] - a[0] for a, b in zip(rx_w, rx_w[1:])]
    if gaps:
        print("0x1100 arrival gap: median %.1f ms, p95 %.1f ms, max %.1f ms"
              % (pct(gaps, 0.5) * 1000, pct(gaps, 0.95) * 1000, max(gaps) * 1000))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True)
    ap.add_argument("--port", type=int, default=82)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--freq", type=float, default=0.8, help="sine frequency, Hz")
    ap.add_argument("--rate", type=float, default=50.0, help="sample rate, Hz")
    ap.add_argument("--amp", type=float, default=0.35, help="amplitude, normalized")
    ap.add_argument("--center", type=float, default=0.5, help="center, normalized")
    ap.add_argument("--span-mm", type=float, default=50.0,
                    help="the hub's window span in mm, for scaling the sent series")
    ap.add_argument("--state-rate", type=float, default=60.0,
                    help="0x1100 subscription rate wish, Hz")
    ap.add_argument("--window", type=float, nargs=2, metavar=("MIN", "MAX"),
                    help="config-set the stroke window to MIN..MAX mm first, and "
                         "use its span for --span-mm")
    ap.add_argument("--force-home", type=float, metavar="STROKE",
                    help="send home op 2 with this stroke first (RFC-025 bench op)")
    return run(ap.parse_args())


if __name__ == "__main__":
    sys.exit(main())
