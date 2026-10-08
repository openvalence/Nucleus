"""Motor ceiling by JOG: point moves (0x3100) at jog_speed/jog_accel set per run, so
the planner's stream games stay out of it. Per leg: commanded distance vs the hub's
quadrature delta vs the DRIVE's own encoder delta (Modbus 0x16/0x17, read at rest
before and after), leg time, drive speed/current peaks, alarm.
python jog_ceiling.py --speed 1000 --accel 100000 --legs 4
"""
import argparse, os, json, os, struct, sys, threading, time
import numpy as np, serial
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'Valence', 'tools')))  # the sibling Valence checkout
import valence_probe as sp, websocket

ap = argparse.ArgumentParser()
ap.add_argument('--ip', default='192.168.1.118'); ap.add_argument('--port', type=int, default=82)
ap.add_argument('--speed', type=float, required=True); ap.add_argument('--accel', type=float, required=True)
ap.add_argument('--lo', type=float, default=20.0); ap.add_argument('--hi', type=float, default=620.0)
ap.add_argument('--fake-home', type=float, help='bench only: fake-home this stroke (0x3101 op 2). Never on a real rail; the machine homes for real first'); ap.add_argument('--legs', type=int, default=4)
ap.add_argument('--poll-hz', type=float, default=20.0); ap.add_argument('--com', default='COM2'); ap.add_argument('--tag', default='')
ap.add_argument('--input-accel', type=float); ap.add_argument('--input-speed', type=float); ap.add_argument('--dump')
a = ap.parse_args()

CPM = 834.4
def crc16(b):
    c = 0xFFFF
    for x in b:
        c ^= x
        for _ in range(8): c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c
lock = threading.Lock(); drive = []; stop = threading.Event()
ser = serial.Serial(a.com, 19200, timeout=0.05)
REQ = bytes([1, 3]) + struct.pack('>HH', 0x0E, 10); REQ += struct.pack('<H', crc16(REQ))
def read_drive():
    with lock:
        ser.reset_input_buffer(); ser.write(REQ); buf = ser.read(25)
    if len(buf) == 25 and crc16(buf[:-2]) == struct.unpack('<H', buf[-2:])[0]:
        r = struct.unpack('>10H', buf[3:23]); enc = struct.unpack('<i', struct.pack('<HH', r[8], r[9]))[0]
        spd = r[2] - 65536 if r[2] > 32767 else r[2]
        return dict(t=time.time(), alarm=r[0], cur=r[1], spd=spd, volt=r[3], temp=r[4], enc=enc)
    return None
def poller():
    period = 1.0 / a.poll_hz
    while not stop.is_set():
        t0 = time.time(); d = read_drive()
        if d: drive.append(d)
        dt = period - (time.time() - t0)
        if dt > 0: time.sleep(dt)
threading.Thread(target=poller, daemon=True).start()

token = None
for _ in range(40):
    token = sp.mint_uitoken(a.ip)
    if token: break
    time.sleep(0.137)
if not token: raise SystemExit('no /uitoken')
ws = websocket.create_connection('ws://%s:%d/' % (a.ip, a.port), subprotocols=[sp.WS_SUBPROTOCOL], timeout=5.0)
sp.send_frame(ws, sp.FRAME['HELLO'], 0, sp.build_hello('probe', 'jog_ceiling.py', os.urandom(8), token=token))
welcome = None; deadline = time.time() + 5
while time.time() < deadline:
    got = sp.recv_frame(ws, deadline)
    if got is None: break
    if got[0]['type'] == sp.FRAME['WELCOME']: welcome = sp.cb_decode_full(got[1]); break
if welcome is None: raise SystemExit('no WELCOME')
sp.send_frame(ws, sp.FRAME['CATALOG_READY'], 0, welcome.get(sp.K['catalog_etag'], b''))
iid = 1
def intent(ch, fields):
    global iid
    sp.send_frame(ws, sp.FRAME['INTENT'], ch, sp.build_intent(ch, iid, fields)); iid += 1
if a.fake_home is not None:
    intent(0x3101, [(1, sp.cb_uint(2)), (2, sp.cb_f32(a.fake_home))])
    intent(0x3000, [(8, sp.cb_f32(a.fake_home))]); time.sleep(0.2)
cfg = [(1, sp.cb_f32(0.0)), (2, sp.cb_f32(a.hi + 10)), (3, sp.cb_f32(a.speed)), (4, sp.cb_f32(a.accel))]
if a.input_accel is not None: cfg.append((6, sp.cb_f32(a.input_accel)))
if a.input_speed is not None: cfg.append((5, sp.cb_f32(a.input_speed)))
intent(0x3000, sorted(cfg))
nacks = []
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
        elif hdr['type'] == sp.FRAME['STATE'] and hdr['channel'] == sp.CH_MOTION_DIAG:
            try: diag.append((time.time(), sp.decode_motion_diag(payload)))
            except ValueError: pass
    return last
rx = []; diag = []
drain(time.time() + 1.0)
sp.send_frame(ws, sp.FRAME['SUBSCRIBE'], 0, sp.build_subscribe([(sp.CH_MOTION, 60.0, 2), (sp.CH_MOTION_DIAG, 2.0, 0)]))
drain(time.time() + 0.5)

def settled(target, timeout=4.0):
    """Wait until the hub reports pos within 1 mm of target and speed 0 for 200 ms."""
    t_end = time.time() + timeout; quiet_since = None
    while time.time() < t_end:
        d = drain(time.time() + 0.05)
        if d is None: continue
        if abs(d['pos_mm'] - target) < 1.5 and abs(d['speed_mm_s']) < 1.0:
            quiet_since = quiet_since or time.time()
            if time.time() - quiet_since > 0.2: return True
        else: quiet_since = None
    return False

# go to the start quietly
intent(0x3100, [(1, sp.cb_f32(a.lo)), (2, sp.cb_bool(False))]); settled(a.lo, 8.0); time.sleep(0.4)
legs = []
for i in range(a.legs):
    target = a.hi if i % 2 == 0 else a.lo
    time.sleep(0.3); d0 = read_drive(); p0 = rx[-1][1] if rx else float('nan'); n0 = len(drive)
    t_go = time.time(); intent(0x3100, [(1, sp.cb_f32(target)), (2, sp.cb_bool(False))])
    ok = settled(target); t_done = time.time(); time.sleep(0.4); d1 = read_drive(); p1 = rx[-1][1]
    seg = drive[n0:]
    R = np.array([r for r in rx if t_go <= r[0] <= t_done + 0.4])
    if len(R) > 3:
        g = np.arange(R[0, 0], R[-1, 0], 0.005); gp = np.interp(g, R[:, 0], R[:, 1]); v = np.gradient(gp, 0.005); vs = np.convolve(v, np.ones(7) / 7, 'same'); vpk = float(np.max(np.abs(vs)))
        moving = np.abs(vs) > 20; t_move = float(moving.sum() * 0.005)
    else: vpk = t_move = float('nan')
    # cruise speed over the middle 400 mm (first crossing of 120 to first crossing of 520), robust to derivative noise
    cruise = float('nan')
    if len(R) > 3:
        lo_x, hi_x = (120.0, 520.0) if target == a.hi else (520.0, 120.0)
        tt = R[:, 0]; pp = R[:, 1]
        c1 = next((t for t, p in zip(tt, pp) if (p >= lo_x if target == a.hi else p <= lo_x)), None)
        c2 = next((t for t, p in zip(tt, pp) if (p >= hi_x if target == a.hi else p <= hi_x)), None)
        if c1 and c2 and c2 > c1: cruise = 400.0 / (c2 - c1)
        # thirds: time spent in each 200 mm third of the leg
        thirds = []
        for lo3, hi3 in ((20, 220), (220, 420), (420, 620)):
            m = (pp >= lo3) & (pp < hi3); thirds.append(float(m.sum() / 60.0))
    else: thirds = []
    cmd = target - (a.lo if target == a.hi else a.hi)
    enc_mm = (d1['enc'] - d0['enc']) / CPM if d0 and d1 else float('nan')
    leg = dict(leg=i, cmd_mm=cmd, hub_mm=p1 - p0, enc_mm=enc_mm, lost_mm=cmd - enc_mm, hub_vs_enc_mm=(p1 - p0) - enc_mm, settled=ok, t_leg_s=t_done - t_go, t_moving_s=t_move,
               v_peak_hub=vpk, v_avg=abs(cmd) / t_move if t_move and t_move > 0 else float('nan'), cruise=cruise, thirds_s=thirds,
               drv_spd_max=max((abs(s['spd']) for s in seg), default=0), drv_cur_max=max((s['cur'] for s in seg), default=0), alarm=max((s['alarm'] for s in seg), default=0), volt=d1['volt'] if d1 else None)
    legs.append(leg)
    print('  leg %d -> %4.0f: cmd %+6.1f enc %+6.1f mm | lost %+5.2f (hub-enc %+5.2f) | %s moving %.3fs | v_avg %4.0f cruise %4.0f mm/s | thirds %s s | drive spd %5d cur %4d alarm %s' % (
        i, target, cmd, enc_mm, leg['lost_mm'], leg['hub_vs_enc_mm'], 'settled' if ok else 'TIMEOUT', t_move, leg['v_avg'], cruise, ['%.2f' % x for x in thirds], leg['drv_spd_max'], leg['drv_cur_max'], leg['alarm']))
    if leg['alarm']: print('  ALARM, stopping'); break
sp.send_frame(ws, sp.FRAME['GOODBYE'], 0, sp.build_goodbye(0)); ws.close(); stop.set(); time.sleep(0.2); ser.close()
L = legs
print('%-8s v=%g a=%g | lost per leg mean %+.2f max %+.2f mm | hub-enc mean %+.2f | v_avg %4.0f v_peak %4.0f | drive spd max %d cur max %d | alarm %s | nacks %d' % (
    a.tag, a.speed, a.accel, np.mean([l['lost_mm'] for l in L]), max((l['lost_mm'] for l in L), key=abs), np.mean([l['hub_vs_enc_mm'] for l in L]),
    np.nanmean([l['v_avg'] for l in L]), np.nanmax([l['v_peak_hub'] for l in L]), max(l['drv_spd_max'] for l in L), max(l['drv_cur_max'] for l in L), max(l['alarm'] for l in L), len(nacks)))
if nacks: print('  first NACK:', nacks[0])
with open('jog_results.jsonl', 'a') as f: f.write(json.dumps(dict(tag=a.tag, speed=a.speed, accel=a.accel, legs=legs, nacks=[str(n) for n in nacks])) + '\n')
if a.dump:
    json.dump(dict(tag=a.tag, rx=rx, drive=drive, legs=legs, diag=diag), open(a.dump, 'w'))
    print('dumped', len(rx), 'hub frames,', len(drive), 'drive polls,', len(diag), 'diag frames ->', a.dump)
