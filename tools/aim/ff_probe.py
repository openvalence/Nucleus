"""Feed-forward probe: hub jogs (clean trapezoids at jog_speed/jog_accel) while the drive
runs with a given set of tuning registers (0x03 accel/FF, 0x08 speed FF, 0x07 pos KP,
0x05 speed KP, 0x06 KI), set live through the output-off gate and restored after.
Per leg: the drive's following lag against the hub's emitted count (encoder polled at
~90 Hz), max following error, overshoot, settle time after the plan ends, current peak.
python ff_probe.py --speed 1000 --accel 100000 --regs 0x03=60000,0x08=3900 --tag base
"""
import argparse, os, json, os, struct, sys, threading, time
import numpy as np, serial
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'Valence', 'tools')))  # the sibling Valence checkout
import valence_probe as sp, websocket

ap = argparse.ArgumentParser()
ap.add_argument('--ip', default='192.168.1.118'); ap.add_argument('--com', default='COM2')
ap.add_argument('--speed', type=float, default=1000.0); ap.add_argument('--accel', type=float, default=100000.0)
ap.add_argument('--lo', type=float, default=20.0); ap.add_argument('--hi', type=float, default=620.0); ap.add_argument('--legs', type=int, default=4)
ap.add_argument('--regs', default='', help='comma list addr=value, hex addr ok'); ap.add_argument('--tag', default='')
a = ap.parse_args()
CPM = 834.4; MM_PER_REV = 39.27

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8): c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c
lock = threading.Lock(); ser = serial.Serial(a.com, 19200, timeout=0.03)
def xfer(fc, payload, nreply, timeout=0.08):
    with lock:
        f = bytes([1, fc]) + payload; ser.reset_input_buffer(); ser.write(f + struct.pack('<H', crc16(f)))
        t0 = time.time(); buf = b''
        while len(buf) < nreply and time.time() - t0 < timeout: buf += ser.read(nreply - len(buf))
    return buf if len(buf) == nreply and crc16(buf[:-2]) == struct.unpack('<H', buf[-2:])[0] else None
def read(addr, n=1):
    r = xfer(3, struct.pack('>HH', addr, n), 5 + 2 * n)
    return list(struct.unpack('>' + 'H' * n, r[3:3 + 2 * n])) if r else None
def write(addr, v):
    xfer(6, struct.pack('>HH', addr, v), 8); time.sleep(0.02); rb = read(addr); ok = rb is not None and rb[0] == v
    print('  0x%02X = %-6d -> %s' % (addr, v, 'ok' if ok else 'FAILED (readback %r)' % rb)); return ok
def enc():
    r = read(0x16, 2)
    return struct.unpack('<i', struct.pack('<HH', r[0], r[1]))[0] if r else None

# ---- drive registers for this run ---------------------------------------------
want = {}
for kv in filter(None, a.regs.split(',')):
    k, v = kv.split('='); want[int(k, 0)] = int(v)
base = read(0x00, 26); orig = {k: base[k] for k in want}
out_en = base[1]
if want:
    print('setting', {('0x%02X' % k): v for k, v in want.items()}, 'was', {('0x%02X' % k): v for k, v in orig.items()})
    ok = write(0x00, 1) and write(0x01, 0)
    for k, v in want.items(): ok = write(k, v) and ok
    write(0x01, out_en); xfer(6, struct.pack('>HH', 0x00, 506), 8); time.sleep(0.05); write(0x00, 0)
    if not ok: print('a register refused; continuing with what took')
live = read(0x02, 7); print('  live: 0x02=%d 0x03=%d 0x04=%d 0x05=%d 0x06=%d 0x07=%d 0x08=%d' % tuple(live))

# ---- encoder poller (fast, encoder only) + status every 20th --------------------
encs = []; stats = []; stop = threading.Event()
def poller():
    i = 0
    while not stop.is_set():
        t0 = time.time(); e = enc()
        if e is not None: encs.append(((t0 + time.time()) / 2, e))
        i += 1
        if i % 20 == 0:
            s = read(0x0E, 5)
            if s: stats.append((time.time(), s[0], s[1], s[2] - 65536 if s[2] > 32767 else s[2]))
threading.Thread(target=poller, daemon=True).start()

# ---- hub ------------------------------------------------------------------------
token = None
for _ in range(40):
    token = sp.mint_uitoken(a.ip)
    if token: break
    time.sleep(0.137)
ws = websocket.create_connection('ws://%s:82/' % a.ip, subprotocols=[sp.WS_SUBPROTOCOL], timeout=5.0)
sp.send_frame(ws, sp.FRAME['HELLO'], 0, sp.build_hello('probe', 'ff_probe.py', os.urandom(8), token=token))
deadline = time.time() + 5; welcome = None
while time.time() < deadline:
    got = sp.recv_frame(ws, deadline)
    if got is None: break
    if got[0]['type'] == sp.FRAME['WELCOME']: welcome = sp.cb_decode_full(got[1]); break
if welcome is None: raise SystemExit('no WELCOME')
sp.send_frame(ws, sp.FRAME['CATALOG_READY'], 0, welcome.get(sp.K['catalog_etag'], b''))
iid = [1]
def intent(ch, fields):
    sp.send_frame(ws, sp.FRAME['INTENT'], ch, sp.build_intent(ch, iid[0], sorted(fields))); iid[0] += 1
intent(0x3101, [(1, sp.cb_uint(2)), (2, sp.cb_f32(650.0))]); intent(0x3000, [(8, sp.cb_f32(650.0))]); time.sleep(0.2)
intent(0x3000, [(1, sp.cb_f32(0.0)), (2, sp.cb_f32(a.hi + 10)), (3, sp.cb_f32(a.speed)), (4, sp.cb_f32(a.accel)), (5, sp.cb_f32(3200.0)), (6, sp.cb_f32(a.accel))])
rx = []; nacks = []
def drain(until):
    last = None
    while time.time() < until:
        got = sp.recv_frame(ws, until)
        if got is None: break
        hdr, payload = got
        if hdr['type'] == sp.FRAME['NACK']: nacks.append(sp.cb_decode_full(payload))
        elif hdr['type'] == sp.FRAME['STATE'] and hdr['channel'] == sp.CH_MOTION:
            try: d = sp.decode_motion_state(payload); last = d; rx.append((time.time(), d['pos_mm'], d['tgt_mm'], d['speed_mm_s']))
            except ValueError: pass
    return last
drain(time.time() + 1.0)
sp.send_frame(ws, sp.FRAME['SUBSCRIBE'], 0, sp.build_subscribe([(sp.CH_MOTION, 60.0, 2)])); drain(time.time() + 0.5)
def settled(target, timeout=5.0):
    t_end = time.time() + timeout; q = None
    while time.time() < t_end:
        d = drain(time.time() + 0.05)
        if d is None: continue
        if abs(d['pos_mm'] - target) < 1.0 and abs(d['speed_mm_s']) < 1.0:
            q = q or time.time()
            if time.time() - q > 0.25: return True
        else: q = None
    return False
intent(0x3100, [(1, sp.cb_f32(a.lo)), (2, sp.cb_bool(False))]); settled(a.lo, 8.0); time.sleep(0.6)

legs = []
for i in range(a.legs):
    target = a.hi if i % 2 == 0 else a.lo
    time.sleep(0.5); t_go = time.time(); n_e0 = len(encs); n_s0 = len(stats)
    intent(0x3100, [(1, sp.cb_f32(target)), (2, sp.cb_bool(False))])
    ok = settled(target); time.sleep(0.6); t_end = time.time()
    H = np.array([r for r in rx if t_go - 0.2 <= r[0] <= t_end]); E = np.array(encs[max(0, n_e0 - 5):], dtype=float); S = stats[n_s0:]
    E = E[(E[:, 0] >= t_go - 0.2) & (E[:, 0] <= t_end)]
    if len(H) < 5 or len(E) < 5: print('  leg %d: too little data' % i); continue
    # both in mm from the leg's start, same epoch clock
    h0 = H[0, 1]; e0 = E[0, 1]
    g = np.arange(t_go - 0.1, t_end, 0.005)
    gh = np.interp(g, H[:, 0], H[:, 1] - h0); ge = np.interp(g, E[:, 0], (E[:, 1] - e0) / CPM)
    sign = 1.0 if target > a.lo + 1 else -1.0
    ge = ge * (1.0 if np.dot(ge, gh) >= 0 else -1.0)          # encoder sign follows the hub's
    ferr = gh - ge
    # lag by cross-correlation of velocities (position xcorr is dominated by the ramp)
    vh = np.gradient(gh, 0.005); ve = np.gradient(ge, 0.005); best, bl = -1e30, 0
    for k in range(0, 60):
        c = np.dot(vh[:len(vh) - k], ve[k:])
        if c > best: best, bl = c, k
    lag_ms = bl * 5.0
    plan_end = next((H[j, 0] for j in range(1, len(H)) if abs(H[j - 1, 3]) > 1 and abs(H[j, 3]) <= 1), None)
    cmd = abs(target - (a.lo if target == a.hi else a.hi))
    in_pos = [t for t, e in zip(g, ge) if abs(abs(e) - cmd) < 0.5]
    settle_ms = (in_pos[0] - plan_end) * 1000 if plan_end and in_pos else float('nan')
    over = float(max(0.0, np.max(np.abs(ge)) - cmd))
    cur = max((s[2] for s in S), default=0); spd = max((abs(s[3]) for s in S), default=0); alarm = max((s[1] for s in S), default=0)
    leg = dict(leg=i, lag_ms=lag_ms, ferr_max=float(np.max(np.abs(ferr))), ferr_p95=float(np.percentile(np.abs(ferr), 95)), overshoot=over, settle_ms=settle_ms,
               enc_moved=float(abs(ge[-1])), hub_moved=float(abs(gh[-1])), cur_max=cur, spd_max_rpm=spd / 10.0, alarm=alarm, settled=ok, enc_hz=len(E) / (E[-1, 0] - E[0, 0]))
    # motor's own ramp: time from first motion to 90% of the hub's cruise, from the encoder velocity
    ve_s = np.convolve(np.abs(ve), np.ones(5) / 5, 'same'); vh_s = np.convolve(np.abs(vh), np.ones(5) / 5, 'same')
    cruise = float(np.percentile(vh_s, 95)); i_start = next((j for j in range(len(ve_s)) if ve_s[j] > 20), None)
    i90 = next((j for j in range(len(ve_s)) if ve_s[j] >= 0.9 * cruise), None); ih90 = next((j for j in range(len(vh_s)) if vh_s[j] >= 0.9 * cruise), None)
    leg['t90_motor_ms'] = (i90 - i_start) * 5.0 if i90 is not None and i_start is not None else float('nan')
    leg['t90_hub_ms'] = (ih90 - next((j for j in range(len(vh_s)) if vh_s[j] > 20), 0)) * 5.0 if ih90 is not None else float('nan')
    leg['lead_max_mm'] = float(np.max(np.abs(ge) - np.abs(gh)))   # positive when the motor is ahead of the pulses
    leg['trace'] = [(float(t - t_go), float(h), float(e)) for t, h, e in zip(g[::2], gh[::2], ge[::2])]
    print('       motor 0..90%% of cruise in %3.0f ms (hub ramp %3.0f ms) | motor ahead of pulses by up to %5.2f mm' % (leg['t90_motor_ms'], leg['t90_hub_ms'], leg['lead_max_mm']))
    legs.append(leg)
    print('  leg %d -> %4.0f: lag %3.0f ms | ferr max %5.1f p95 %5.1f mm | overshoot %4.2f mm | settle %5.0f ms after plan end | enc %6.1f hub %6.1f mm | cur %4d | %4.0f rpm | alarm %d | enc %3.0f Hz' % (
        i, target, lag_ms, leg['ferr_max'], leg['ferr_p95'], over, settle_ms, leg['enc_moved'], leg['hub_moved'], cur, leg['spd_max_rpm'], alarm, leg['enc_hz']))
    if alarm: print('  ALARM, stopping'); break
sp.send_frame(ws, sp.FRAME['GOODBYE'], 0, sp.build_goodbye(0)); ws.close(); stop.set(); time.sleep(0.2)
if want:
    print('restoring', {('0x%02X' % k): v for k, v in orig.items()})
    write(0x00, 1); write(0x01, 0)
    for k, v in orig.items(): write(k, v)
    write(0x01, out_en); xfer(6, struct.pack('>HH', 0x00, 506), 8); time.sleep(0.05); write(0x00, 0)
print('%-14s regs %-28s v=%.0f a=%.0f | lag %3.0f ms | ferr max %5.1f p95 %5.1f | overshoot %4.2f | settle %5.0f ms | cur %4d | alarm %d' % (
    a.tag, a.regs or 'baseline', a.speed, a.accel, np.mean([l['lag_ms'] for l in legs]), max(l['ferr_max'] for l in legs), np.mean([l['ferr_p95'] for l in legs]),
    max(l['overshoot'] for l in legs), np.nanmean([l['settle_ms'] for l in legs]), max(l['cur_max'] for l in legs), max(l['alarm'] for l in legs)))
with open('ff_results.jsonl', 'a') as f: f.write(json.dumps(dict(tag=a.tag, regs=a.regs, speed=a.speed, accel=a.accel, live=live, legs=legs)) + '\n')
ser.close()
