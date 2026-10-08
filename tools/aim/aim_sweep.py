"""AIM drive register sweep over Modbus RTU: READS ONLY (FC 0x03 and read-class
function codes). Finds every holding-register address the firmware answers,
and which function codes it honors. Never writes.
Usage: python aim_sweep.py COM2 [slave]
"""
import struct, sys, time, json
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM2'
SLAVE = int(sys.argv[2]) if len(sys.argv) > 2 else 1

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def frame(fc, payload):
    f = bytes([SLAVE, fc]) + payload
    return f + struct.pack('<H', crc16(f))

def xfer(ser, fc, payload, timeout=0.06):
    ser.reset_input_buffer()
    ser.write(frame(fc, payload))
    ser.flush()
    t0 = time.time(); buf = b''
    while time.time() - t0 < timeout:
        chunk = ser.read(64)
        if chunk:
            buf += chunk; t0 = time.time(); timeout = 0.012  # inter-frame gap closes the read
    if len(buf) < 4: return None
    if crc16(buf[:-2]) != struct.unpack('<H', buf[-2:])[0]: return ('badcrc', buf)
    if buf[0] != SLAVE: return ('other', buf)
    if buf[1] & 0x80: return ('exc', buf[2])
    return ('ok', buf[2:-2])

def read_regs(ser, addr, n=1):
    r = xfer(ser, 0x03, struct.pack('>HH', addr, n))
    if r and r[0] == 'ok' and len(r[1]) >= 1 + 2 * n:
        return list(struct.unpack('>' + 'H' * n, r[1][1:1 + 2 * n]))
    return r

def autobaud():
    for baud in (19200, 115200, 9600, 38400, 57600):
        ser = serial.Serial(PORT, baud, timeout=0.02)
        time.sleep(0.05)
        r = read_regs(ser, 0x15, 1)
        if isinstance(r, list):
            print('baud', baud, 'device address register 0x15 =', r[0])
            return ser, baud
        ser.close()
    raise SystemExit('no answer at any baud on ' + PORT)

ser, baud = autobaud()
out = {'port': PORT, 'baud': baud, 'slave': SLAVE}

# 1. the documented block, every value
doc = {}
for a in range(0x00, 0x1A):
    r = read_regs(ser, a); doc['0x%02X' % a] = r[0] if isinstance(r, list) else r
print('documented 0x00..0x19:', doc)
out['documented'] = doc

# 2. dense sweep 0x0000..0x03FF, then stride over the rest to find other blocks
impl, exc, silent = {}, {}, []
def probe(a):
    r = read_regs(ser, a)
    if isinstance(r, list): impl['0x%04X' % a] = r[0]
    elif r and r[0] == 'exc': exc['0x%04X' % a] = r[1]
    else: silent.append(a)
t = time.time()
for a in range(0x0000, 0x0400): probe(a)
print('dense 0..0x3FF: %d answer, %d exception, %d silent (%.0fs)' % (len(impl), len(exc), len(silent), time.time() - t))
t = time.time(); hits_before = len(impl)
for a in range(0x0400, 0x10000, 64): probe(a)
print('stride 64 over 0x400..0xFFFF: %d new answers (%.0fs)' % (len(impl) - hits_before, time.time() - t))
# refine around any stride hits
for key in list(impl):
    a = int(key, 16)
    if a >= 0x0400:
        for b in range(max(0, a - 64), a + 64): probe(b)
out['implemented'] = impl; out['exceptions'] = {k: v for k, v in list(exc.items())[:64]}; out['exception_count'] = len(exc); out['silent_count'] = len(silent)
addrs = sorted(int(k, 16) for k in impl)
# summarize contiguous ranges
ranges, start = [], None
for i, a in enumerate(addrs):
    if start is None: start = a
    if i + 1 == len(addrs) or addrs[i + 1] != a + 1:
        ranges.append((start, a)); start = None
print('implemented ranges:', ['0x%04X..0x%04X' % r for r in ranges])

# 3. multi-register read sizes the firmware accepts (count 1, 2, 4, 8, 16, 26, 32)
counts = {}
for n in (1, 2, 4, 8, 16, 26, 32, 64):
    r = read_regs(ser, 0x00, n); counts[n] = 'ok' if isinstance(r, list) else (r[0] if r else 'silent')
print('FC03 counts from 0x00:', counts); out['fc03_counts'] = counts

# 4. read-class function codes only (never a write): 01 02 03 04 07 08(echo) 11 17(read part only is a write: skipped) 2B(device id)
fcs = {}
for fc, payload in ((0x01, struct.pack('>HH', 0, 8)), (0x02, struct.pack('>HH', 0, 8)), (0x03, struct.pack('>HH', 0, 1)), (0x04, struct.pack('>HH', 0, 1)),
                    (0x07, b''), (0x08, struct.pack('>HH', 0, 0x1234)), (0x11, b''), (0x2B, bytes([0x0E, 0x01, 0x00])), (0x2B, bytes([0x0E, 0x03, 0x00]))):
    r = xfer(ser, fc, payload)
    fcs['0x%02X/%s' % (fc, payload.hex())] = (r[0], r[1].hex() if isinstance(r[1], (bytes, bytearray)) else r[1]) if r else 'silent'
print('function codes:', json.dumps(fcs))
out['function_codes'] = fcs
json.dump(out, open('aim_sweep_result.json', 'w'), indent=1)
print('saved aim_sweep_result.json')
ser.close()
