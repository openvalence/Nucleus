"""Bench ceiling probe: stream a waveform at the hub with the planner's limits set
from the command line, log the hub's 0x1100 frames (raw, tgt, pos, speed) and the
plan style, and poll the DRIVE over RS485 (alarm, current, speed, volts, encoder)
in a thread at --poll-hz. Reports swing, peak speed, lag, following error, drift
(= lost steps) and whatever the drive complained about. Waveform-agnostic: no sine fit.
python ceiling.py --wave square --freq 1 --amp 0.45 --speed 3000 --accel 100000 --poll-hz 10
"""
import argparse, os, json, math, os, struct, sys, threading, time
import numpy as np, serial
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'Valence', 'tools')))  # the sibling Valence checkout
import valence_probe as sp, websocket

ap = argparse.ArgumentParser()
ap.add_argument('--ip', default='192.168.1.118'); ap.add_argument('--port', type=int, default=82)
ap.add_argument('--wave', default='sine', choices=['sine', 'tri', 'square'])
ap.add_argument('--freq', type=float, default=1.0); ap.add_argument('--amp', type=float, default=0.45); ap.add_argument('--center', type=float, default=0.5)
ap.add_argument('--seconds', type=float, default=8.0); ap.add_argument('--rate', type=float, default=50.0)   # the hub answers RATE_LIMITED above its grant
ap.add_argument('--window', type=float, nargs=2, default=[0.0, 100.0]); ap.add_argument('--home', type=float, default=200.0)
ap.add_argument('--speed', type=float); ap.add_argument('--accel', type=float); ap.add_argument('--jerk', type=float); ap.add_argument('--max-rail', type=float)
ap.add_argument('--poll-hz', type=float, default=10.0); ap.add_argument('--com', default='COM2')
ap.add_argument('--dump'); ap.add_argument('--tag', default='')
a = ap.parse_args()
lo, hi = a.window; span = hi - lo

# ---- drive poller -------------------------------------------------------------
CPM = 834.4
def crc16(b):
    c = 0xFFFF
    for x in b:
        c ^= x
        for _ in range(8): c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c
drive = []   # (t, alarm, cur, spd, volt, temp, r13, enc_counts)
stop = threading.Event()
def poller():
    ser = serial.Serial(a.com, 19200, timeout=0.05)
    req = bytes([1, 3]) + struct.pack('>HH', 0x0E, 10); req += struct.pack('<H', crc16(req))
    period = 1.0 / a.poll_hz
    while not stop.is_set():
        t0 = time.time(); ser.reset_input_buffer(); ser.write(req); buf = ser.read(25)
        if len(buf) == 25 and crc16(buf[:-2]) == struct.unpack('<H', buf[-2:])[0]:
            r = struct.unpack('>10H', buf[3:23]); enc = struct.unpack('<i', struct.pack('<HH', r[8], r[9]))[0]
            drive.append(((t0 + time.time()) / 2, r[0], r[1], r[2], r[3], r[4], r[5], enc))
        dt = period - (time.time() - t0)
        if dt > 0: time.sleep(dt)
    ser.close()
if a.poll_hz > 0:
    th = threading.Thread(target=poller, daemon=True); th.start()

# ---- hub session --------------------------------------------------------------
token = None
for _ in range(40):
    token = sp.mint_uitoken(a.ip)
    if token: break
    time.sleep(0.137)
if not token: raise SystemExit('no /uitoken')
ws = websocket.create_connection('ws://%s:%d/' % (a.ip, a.port), subprotocols=[sp.WS_SUBPROTOCOL], timeout=5.0)
sp.send_frame(ws, sp.FRAME['HELLO'], 0, sp.build_hello('probe', 'ceiling.py', os.urandom(8), publishes=[(sp.CH_MOTION_INPUT, 100.0)], token=token))
welcome = None; deadline = time.time() + 5
while time.time() < deadline:
    got = sp.recv_frame(ws, deadline)
    if got is None: break
    if got[0]['type'] == sp.FRAME['WELCOME']: welcome = sp.cb_decode_full(got[1]); break
if welcome is None or not welcome.get(sp.K['granted_publishes']): raise SystemExit('no WELCOME / publish not granted')
sp.send_frame(ws, sp.FRAME['CATALOG_READY'], 0, welcome.get(sp.K['catalog_etag'], b''))
iid = 1
sp.send_frame(ws, sp.FRAME['INTENT'], 0x3101, sp.build_intent(0x3101, iid, [(1, sp.cb_uint(2)), (2, sp.cb_f32(a.home))])); iid += 1
if a.max_rail is not None:
    sp.send_frame(ws, sp.FRAME['INTENT'], 0x3000, sp.build_intent(0x3000, iid, [(8, sp.cb_f32(a.max_rail))])); iid += 1
    time.sleep(0.2)
fields = [(1, sp.cb_f32(lo)), (2, sp.cb_f32(hi))]
if a.speed is not None: fields.append((5, sp.cb_f32(a.speed)))
if a.accel is not None: fields.append((6, sp.cb_f32(a.accel)))
if a.jerk is not None: fields.append((7, sp.cb_f32(a.jerk)))
sp.send_frame(ws, sp.FRAME['INTENT'], 0x3000, sp.build_intent(0x3000, iid, fields)); iid += 1
t_settle = time.time() + 1.0
nacks = []
while time.time() < t_settle:
    got = sp.recv_frame(ws, t_settle)
    if got is None: break
    if got[0]['type'] == sp.FRAME['NACK']: nacks.append(('setup', sp.cb_decode_full(got[1])))
sp.send_frame(ws, sp.FRAME['SUBSCRIBE'], 0, sp.build_subscribe([(sp.CH_MOTION, 60.0, 2), (sp.CH_PLAN_STRIP, 45.0, 1)]))
off = sp.estimate_hub_offset(ws, 5.0)
if off is None: raise SystemExit('no CLOCK')
offset_us = off[0]

def wave(t):
    ph = (a.freq * t) % 1.0
    if a.wave == 'sine':
        return a.center + a.amp * math.sin(2 * math.pi * a.freq * t), a.amp * 2 * math.pi * a.freq * math.cos(2 * math.pi * a.freq * t)
    if a.wave == 'tri':
        v = 4 * a.amp * a.freq
        return (a.center - a.amp + v * ph / a.freq, v) if ph < 0.5 else (a.center + a.amp - v * (ph - 0.5) / a.freq, -v)
    return (a.center + a.amp if ph < 0.5 else a.center - a.amp), 0.0

sent, rx, plans = [], [], []
t_start = time.time(); t_end = t_start + a.seconds; next_tx = t_start; period = 1.0 / a.rate
while time.time() < t_end:
    now = time.time()
    if now >= next_tx:
        next_tx += period
        tg, v = wave(now - t_start); tg = min(max(tg, 0.0), 1.0)
        sp.send_frame(ws, sp.FRAME['STREAM'], sp.CH_MOTION_INPUT, sp.encode_stream_bundle((sp.client_now_us() + offset_us) & 0xFFFFFFFF, [(0, tg, v)]))
        sent.append((now, tg))
    got = sp.recv_frame(ws, min(next_tx, t_end))
    if got is None: continue
    hdr, payload = got
    if hdr['type'] == sp.FRAME['NACK']: nacks.append(('run', sp.cb_decode_full(payload))); continue
    if hdr['type'] == sp.FRAME['STATE'] and hdr['channel'] == sp.CH_MOTION:
        try: d = sp.decode_motion_state(payload)
        except ValueError: continue
        rx.append((time.time(), d['pos_mm'], d['tgt_mm'], d.get('raw_mm', float('nan')), d['speed_mm_s'], d['flags']))
    elif hdr['type'] == sp.FRAME['STATE'] and hdr['channel'] == sp.CH_PLAN_STRIP:
        try: p = sp.decode_plan_strip(payload); plans.append((time.time(), p.get('style', p.get('style_name', '?'))))
        except Exception: pass
sp.send_frame(ws, sp.FRAME['GOODBYE'], 0, sp.build_goodbye(0)); ws.close()
time.sleep(0.3); stop.set()

# ---- analysis ---------------------------------------------------------------
S = np.array(sent); R = np.array(rx); D = np.array(drive, dtype=float) if drive else None
t0 = t_start
ts, ys = S[:, 0] - t0, S[:, 1] * span + lo
tr = R[:, 0] - t0; pos, tgt, raw, spd = R[:, 1], R[:, 2], R[:, 3], R[:, 4]
w = (tr > 1.0) & (tr < a.seconds - 0.3)
def swing(y): return float(np.percentile(y, 99) - np.percentile(y, 1))
grid = np.arange(1.0, a.seconds - 0.3, 0.01)
def on(t, y): return np.interp(grid, t, y)
def xlag(ref, sig, maxlag=0.6):
    r0, s0 = ref - ref.mean(), sig - sig.mean(); best, bl = -1e30, 0
    for k in range(-int(maxlag / 0.01), int(maxlag / 0.01) + 1):
        c = np.dot(r0[max(0, -k):len(r0) - max(0, k)], s0[max(0, k):len(s0) - max(0, -k)])
        if c > best: best, bl = c, k
    return bl * 10.0
g_sent, g_tgt, g_pos, g_raw = on(ts, ys), on(tr, tgt), on(tr, pos), on(tr, raw)
vel = np.gradient(g_pos, 0.01); vel_s = np.convolve(vel, np.ones(5) / 5, 'same')
ferr = g_tgt - g_pos
drift_hub = float(np.mean(ferr[-100:]) - np.mean(ferr[:100]))
styles = {}
for _, s in plans: styles[str(s)] = styles.get(str(s), 0) + 1
out = dict(tag=a.tag, wave=a.wave, freq=a.freq, amp=a.amp, span=span, speed=a.speed, accel=a.accel, jerk=a.jerk, poll_hz=a.poll_hz,
           sent_pp=swing(ys[(ts > 1)]), tgt_pp=swing(tgt[w]), pos_pp=swing(pos[w]), raw_pp=swing(raw[w]),
           peak_mm_s=float(np.max(np.abs(vel_s))), hub_speed_peak=float(np.max(np.abs(spd[w]))),
           lag_raw_pos_ms=xlag(g_raw, g_pos), lag_tgt_pos_ms=xlag(g_tgt, g_pos), lag_sent_pos_ms=xlag(g_sent, g_pos),
           ferr_p95=float(np.percentile(np.abs(ferr), 95)), ferr_max=float(np.max(np.abs(ferr))), drift_hub_mm=drift_hub,
           frames=len(rx), frame_hz=len(rx) / a.seconds, nacks=len(nacks), styles=styles)
if D is not None and len(D) > 10:
    td = D[:, 0] - t0; wd = (td > 1.0) & (td < a.seconds - 0.3)
    enc_mm = (D[:, 7] - D[0, 7]) / CPM
    g_enc = np.interp(grid, td, enc_mm)
    g_enc = g_enc - np.mean(g_enc[:50]) + np.mean(g_pos[:50])
    out.update(enc_pp=swing(enc_mm[wd]), enc_lag_vs_hubpos_ms=xlag(g_pos, g_enc),
               enc_minus_hubpos_drift_mm=float(np.mean((g_enc - g_pos)[-100:]) - np.mean((g_enc - g_pos)[:100])),
               enc_minus_tgt_drift_mm=float(np.mean((g_enc - g_tgt)[-100:]) - np.mean((g_enc - g_tgt)[:100])),
               alarm_max=int(D[:, 1].max()), cur_max=int(D[:, 2].max()), r10_max=int(D[:, 3].max()), r10_min=int(D[:, 3].min()),
               volt=int(D[-1, 4]), temp=int(D[-1, 5]), r13_max=int(D[:, 6].max()),
               polls=len(D), poll_hz_actual=len(D) / max(D[-1, 0] - D[0, 0], 1e-9))
def fmt(k, f='%.1f'): return (f % out[k]) if k in out else 'n/a'
print('%-10s %s f=%g amp=%g | sent %5.1f tgt %5.1f pos %5.1f enc %5s mm pp | peak %4.0f mm/s (hub %4.0f) | lag raw>pos %+4.0f tgt>pos %+4.0f ms | ferr p95 %4.1f max %4.1f | drift hub %+5.2f enc-tgt %5s enc-pos %5s mm | alarm %s cur %s r10 %s..%s | styles %s | nacks %d | poll %s Hz' % (
    a.tag, a.wave, a.freq, a.amp, out['sent_pp'], out['tgt_pp'], out['pos_pp'], fmt('enc_pp'), out['peak_mm_s'], out['hub_speed_peak'],
    out['lag_raw_pos_ms'], out['lag_tgt_pos_ms'], out['ferr_p95'], out['ferr_max'], drift_hub, fmt('enc_minus_tgt_drift_mm', '%+.2f'), fmt('enc_minus_hubpos_drift_mm', '%+.2f'),
    out.get('alarm_max', '-'), out.get('cur_max', '-'), out.get('r10_min', '-'), out.get('r10_max', '-'), styles, len(nacks), fmt('poll_hz_actual', '%.0f')))
if nacks: print('  first NACK:', nacks[0])
if a.dump:
    json.dump(dict(out=out, t_start=t_start, sent=sent, rx=rx, drive=drive, plans=[(t, str(s)) for t, s in plans], nacks=[str(n) for n in nacks]), open(a.dump, 'w'), indent=0)
with open('ceiling_results.jsonl', 'a') as f: f.write(json.dumps(out) + '\n')
