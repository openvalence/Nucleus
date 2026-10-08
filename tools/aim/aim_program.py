"""Program the bench 57AIM30 to the machine drive's settings and persist them.
The save gate (bench-verified): 0x00=1, 0x01=0 (output off), write, 0x14=1 last
x3 polling 0x14 for 2, 0x01 back to what it read, then leave the Modbus door
with 0x00=506 then 0x00=0. 0x19 engages at power-on from the SAVED value.
Never writes 0x0A=0, never touches the baud envelope.
Usage: python aim_program.py COM2 [--verify]
"""
import struct, sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM2'
VERIFY = '--verify' in sys.argv
SLAVE = 1
TARGET = {0x02: 1500, 0x03: 60000, 0x04: 495, 0x05: 3000, 0x06: 10, 0x07: 3000, 0x08: 3900,
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

def dump(ser):
    v = read(ser, 0x00, 26)
    return dict(enumerate(v)) if v else None

ser = serial.Serial(PORT, 19200, timeout=0.02)
time.sleep(0.1)
before = dump(ser)
if not before: raise SystemExit('no answer on ' + PORT + ' at 19200')
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
for a, t in TARGET.items():
    if before[a] != t: ok = write(ser, a, t) and ok
if not ok:
    print('a write failed: not saving; restoring output enable'); write(ser, 0x01, out_en); write(ser, 0x00, 506); write(ser, 0x00, 0); ser.close(); raise SystemExit(1)
saved = False
for attempt in range(3):
    xfer(ser, 0x06, struct.pack('>HH', 0x14, 1))
    for _ in range(30):
        time.sleep(0.1); f = read(ser, 0x14)
        if f and f[0] == 2: saved = True; break
    print('  save attempt %d: flag %r' % (attempt + 1, f))
    if saved: break
write(ser, 0x01, out_en)
xfer(ser, 0x06, struct.pack('>HH', 0x00, 506)); time.sleep(0.05)
write(ser, 0x00, 0)
after = dump(ser)
print('after:', {('0x%02X' % k): v for k, v in after.items()})
print('SAVED (flag reached 2)' if saved else 'NOT SAVED: flag never reached 2; values are live only')
print('now power-cycle the drive, then run with --verify')
ser.close()
