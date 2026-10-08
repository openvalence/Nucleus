"""Unloaded motor limits WITHOUT the hub: the drive's own Modbus moves (FC 0x10 delta to
0x0C/0x0D, counts, 32768/rev) at a given 0x02 speed, encoder polled at ~100 Hz, status
regs every 10th poll. Reports the plateau speed, the accel and decel slopes, current and
alarm. Arms 0x00=1 for the moves (pulses are deaf meanwhile) and leaves by 506 then 0.
python aim_move.py COM2 --rpm 3000 --revs 50
"""
import argparse, struct, sys, time
import numpy as np, serial

ap = argparse.ArgumentParser()
ap.add_argument('port'); ap.add_argument('--rpm', type=int, required=True); ap.add_argument('--revs', type=float, default=50.0)
ap.add_argument('--restore-rpm', type=int, default=1500); ap.add_argument('--tag', default='')
ap.add_argument('--fw', type=int, help='field weakening 0x04 for this run (restored after)'); ap.add_argument('--ramp', type=int, help='accel 0x03 for this run (restored after)')
a = ap.parse_args()
MM_PER_REV = 39.27; CPR = 32768.0

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8): c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c
ser = serial.Serial(a.port, 19200, timeout=0.03)
def xfer(fc, payload, nreply, timeout=0.08):
    f = bytes([1, fc]) + payload; ser.reset_input_buffer(); ser.write(f + struct.pack('<H', crc16(f)))
    t0 = time.time(); buf = b''
    while len(buf) < nreply and time.time() - t0 < timeout: buf += ser.read(nreply - len(buf))
    return buf if len(buf) == nreply and crc16(buf[:-2]) == struct.unpack('<H', buf[-2:])[0] else None
def read(addr, n=1):
    r = xfer(3, struct.pack('>HH', addr, n), 5 + 2 * n)
    return list(struct.unpack('>' + 'H' * n, r[3:3 + 2 * n])) if r else None
def write(addr, v):
    xfer(6, struct.pack('>HH', addr, v), 8); time.sleep(0.02); rb = read(addr); ok = rb is not None and rb[0] == v
    print('  0x%02X = %-5d -> %s' % (addr, v, 'ok' if ok else 'FAILED (readback %r)' % rb)); return ok
def enc():
    r = read(0x16, 2)
    return struct.unpack('<i', struct.pack('<HH', r[0], r[1]))[0] if r else None
def delta(counts):
    v = counts & 0xFFFFFFFF; payload = struct.pack('>HHB', 0x0C, 2, 4) + struct.pack('>HH', v & 0xFFFF, v >> 16)
    return xfer(0x10, payload, 8) is not None

base = read(0x00, 26); print('before: 0x00=%d 0x01=%d 0x02=%d 0x03=%d 0x04=%d alarm=%d temp=%d' % (base[0], base[1], base[2], base[3], base[4], base[0x0E], base[0x12]))
out_en = base[1]
print('arming modbus, setting 0x02 = %d (output off for the write)' % a.rpm)
fw0, ramp0 = base[4], base[3]
ok = write(0x00, 1) and write(0x01, 0) and write(0x02, a.rpm)
if ok and a.fw is not None: ok = write(0x04, a.fw)
if ok and a.ramp is not None: ok = write(0x03, a.ramp)
if not ok:
    print('could not set; leaving'); write(0x04, fw0); write(0x03, ramp0); write(0x01, out_en); xfer(6, struct.pack('>HH', 0x00, 506), 8); time.sleep(0.05); write(0x00, 0); ser.close(); raise SystemExit(1)
write(0x01, out_en); time.sleep(0.3)
rpm_set = read(0x02)[0]; print('  live now: 0x02=%d 0x03=%d 0x04=%d' % tuple(read(0x02, 3)))

def run(counts, label):
    log = []; stat = []
    e0 = enc(); t_go = time.time(); ok = delta(counts)
    if not ok: print('  delta rejected'); return None
    i = 0; last_move = time.time()
    while time.time() - t_go < 12.0:
        t = time.time(); e = enc()
        if e is not None: log.append((t, e))
        i += 1
        if i % 10 == 0:
            s = read(0x0E, 5)
            if s: stat.append((time.time(), s[0], s[1], s[2] - 65536 if s[2] > 32767 else s[2], s[3], s[4]))
        if len(log) > 5 and abs(log[-1][1] - log[-5][1]) > 50: last_move = time.time()
        if time.time() - last_move > 0.5 and time.time() - t_go > 0.6: break
    L = np.array(log, dtype=float); tt = L[:, 0] - t_go; mm = (L[:, 1] - e0) / CPR * MM_PER_REV
    v = np.gradient(mm, tt); vs = np.convolve(v, np.ones(5) / 5, 'same')
    top = np.sort(np.abs(vs))[-max(5, len(vs) // 5):]; plateau = float(np.median(top))
    # accel: best 60 ms slope of |v| rising; decel: falling
    acc = dec = 0.0
    for j in range(len(tt)):
        k = np.searchsorted(tt, tt[j] + 0.06)
        if k < len(tt) and tt[k] - tt[j] > 0.03:
            s = (abs(vs[k]) - abs(vs[j])) / (tt[k] - tt[j]); acc = max(acc, s); dec = min(dec, s)
    t90 = next((float(tt[j]) for j in range(len(tt)) if abs(vs[j]) >= 0.9 * plateau), float('nan'))
    S = np.array(stat, dtype=float) if stat else None
    moved = (log[-1][1] - e0) / CPR
    print('  %-6s %+6.1f revs cmd, %+6.1f moved | plateau %5.0f mm/s = %5.0f rpm (0x02 %d) | t90 %.3f s | accel %6.0f decel %6.0f mm/s2 | %s | alarm %s cur max %s spd reg max %s | %d polls at %.0f Hz' % (
        label, counts / CPR, moved, plateau, plateau / MM_PER_REV * 60, rpm_set, t90, acc, dec, '%.2f s' % tt[-1],
        int(S[:, 1].max()) if S is not None else '-', int(S[:, 2].max()) if S is not None else '-', int(np.abs(S[:, 3]).max()) if S is not None else '-', len(log), len(log) / tt[-1]))
    return dict(label=label, rpm_set=rpm_set, plateau_mm_s=plateau, plateau_rpm=plateau / MM_PER_REV * 60, t90=t90, accel=acc, decel=dec, moved_revs=moved,
                alarm=int(S[:, 1].max()) if S is not None else None, cur_max=int(S[:, 2].max()) if S is not None else None, trace=[(float(x), float(y)) for x, y in zip(tt, mm)])

n = int(a.revs * CPR)
r1 = run(+n, 'fwd'); time.sleep(0.5); r2 = run(-n, 'back'); time.sleep(0.3)
st = read(0x0E, 5); print('after: alarm %d current %d speed %d volts %d temp %d' % tuple(st))
print('restoring 0x02 = %d and leaving the modbus door' % a.restore_rpm)
write(0x01, 0); write(0x02, a.restore_rpm)
if a.fw is not None: write(0x04, fw0)
if a.ramp is not None: write(0x03, ramp0)
write(0x01, out_en); xfer(6, struct.pack('>HH', 0x00, 506), 8); time.sleep(0.05); write(0x00, 0)
fin = read(0x00, 5); print('door: 0x00=%d 0x01=%d 0x02=%d 0x03=%d 0x04=%d' % tuple(fin))
import json
with open('aim_move_results.jsonl', 'a') as f: f.write(json.dumps(dict(tag=a.tag, rpm=a.rpm, revs=a.revs, fwd=r1, back=r2)) + '\n')
ser.close()
