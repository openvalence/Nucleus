"""Program an AIM drive to the machine drive's settings and persist them.
The save that survives a power cycle (proven 2026-08-07 over three power cycles,
held A/B on the field drive 2026-10-30; the site servo tool's sequence, bd
val-7z2b): 0x00=1, 0x01=0 (output off), then three passes of [the values,
0x14=1, 200 ms with no bus traffic], then 0x01=1, and NO 0x00=506 release:
Modbus control stays on, so the drive ignores its pulse input, until the user
power-cycles it. Do not poll 0x14 through the save, save with the output live,
or release with 506 before the power cycle: those runs read back fine and
revert at power-on. 0x19 engages at power-on from the SAVED value.
Never writes 0x0A=0, never touches the baud envelope.
Usage: python aim_program.py COM2 [--verify] [--baud N]
"""
import struct, sys, time
import aim_link

BAUD = aim_link.pop_baud(sys.argv)
PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM2'
VERIFY = '--verify' in sys.argv
SLAVE = 1
TARGET = {0x02: 2000, 0x03: 60000, 0x04: 495, 0x05: 3000, 0x06: 10, 0x07: 3000, 0x08: 3900,
          0x09: 1, 0x0A: 4, 0x0B: 1, 0x18: 600, 0x19: 2}
NAMES = {0x00: 'modbus enable', 0x01: 'output enable', 0x02: 'target speed rpm', 0x03: 'accel', 0x04: 'field weak', 0x05: 'speed KP',
         0x06: 'speed KI ms', 0x07: 'pos KP', 0x08: 'speed FF', 0x09: 'dir polarity', 0x0A: 'gear num', 0x0B: 'gear den',
         0x0E: 'alarm', 0x11: 'volts mV', 0x12: 'temp C', 0x14: 'save flag', 0x15: 'address', 0x18: 'standstill', 0x19: 'function'}

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def xfer(ser, fc, payload, timeout=0.08):
    f = bytes([SLAVE, fc]) + payload
    ser.reset_input_buffer(); ser.write(f + struct.pack('<H', crc16(f))); ser.flush()
    t0 = time.time(); buf = b''
    while time.time() - t0 < timeout:
        c = ser.read(64)
        if c: buf += c; t0 = time.time(); timeout = 0.012
    if len(buf) < 4 or crc16(buf[:-2]) != struct.unpack('<H', buf[-2:])[0]: return None
    if buf[1] & 0x80: return ('exc', buf[2])
    return ('ok', buf[2:-2])

def read(ser, addr, n=1):
    r = xfer(ser, 0x03, struct.pack('>HH', addr, n))
    return list(struct.unpack('>' + 'H' * n, r[1][1:1 + 2 * n])) if r and r[0] == 'ok' else None

def write(ser, addr, val):
    r = xfer(ser, 0x06, struct.pack('>HH', addr, val))
    time.sleep(0.03)
    rb = read(ser, addr)
    ok = r is not None and r[0] == 'ok' and rb is not None and rb[0] == val
    print('  write 0x%02X %-14s = %-6d -> %s' % (addr, NAMES.get(addr, ''), val, 'ok' if ok else 'FAILED (reply %r, readback %r)' % (r, rb)))
    return ok

def send(ser, addr, val):
    """A write whose reply and read-back are not judged (the save's later passes, 0x14, 0x01)."""
    xfer(ser, 0x06, struct.pack('>HH', addr, val))
    time.sleep(0.03)
    print('  write 0x%02X %-14s = %d' % (addr, NAMES.get(addr, ''), val))

def dump(ser):
    v = read(ser, 0x00, 26)
    return dict(enumerate(v)) if v else None

ser = aim_link.open_drive(PORT, BAUD, timeout=0.02)
time.sleep(0.1)
before = dump(ser)
if not before: raise SystemExit('no answer on %s at %d' % (PORT, ser.baudrate))
print('before:', {('0x%02X' % k): v for k, v in before.items()})
diff = {a: (before[a], t) for a, t in TARGET.items() if before[a] != t}
if VERIFY:
    print('MATCH' if not diff else 'DIFFER: ' + str({'0x%02X' % a: d for a, d in diff.items()}))
    print('save flag 0x14 =', before[0x14], '| function 0x19 =', before[0x19], '| alarm 0x0E =', before[0x0E])
    ser.close(); raise SystemExit(0)
if not diff:
    print('already matching; nothing written'); ser.close(); raise SystemExit(0)
print('to change:', {'0x%02X' % a: d for a, d in diff.items()})
out_en = before[0x01]
ok = write(ser, 0x00, 1) and write(ser, 0x01, 0)
for p in range(1, 4):
    for a, (_, t) in diff.items():
        if p == 1: ok = write(ser, a, t) and ok
        else: send(ser, a, t)
    if not ok: break
    send(ser, 0x14, 1)
    time.sleep(0.2)
    print('  save pass %d sent' % p)
if not ok:
    print('a write failed: not saving; restoring output enable and releasing Modbus control')
    send(ser, 0x01, out_en); send(ser, 0x00, 506); time.sleep(0.05); write(ser, 0x00, 0); ser.close(); raise SystemExit(1)
send(ser, 0x01, 1)
after = dump(ser)
if after: print('after:', {('0x%02X' % k): v for k, v in after.items()})
print('saved: Modbus control stays on (0x00=1, the drive ignores its pulse input) until the power cycle')
print('now power-cycle the drive, then run with --verify')
ser.close()
