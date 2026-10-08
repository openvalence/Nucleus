"""Set one AIM drive register LIVE (not saved): the same door as aim_program.py
(0x00=1, output off, write, output back, 0x00=506, 0x00=0) minus the save flag,
so the value reverts on the next power cycle. python aim_live.py COM2 0x02 3000
"""
import struct, sys, time
import serial
PORT, ADDR, VAL = sys.argv[1], int(sys.argv[2], 16), int(sys.argv[3])
def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8): c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c
def xfer(ser, fc, payload):
    f = bytes([1, fc]) + payload; ser.reset_input_buffer(); ser.write(f + struct.pack('<H', crc16(f))); ser.flush()
    time.sleep(0.06); buf = ser.read(64)
    return buf if len(buf) >= 4 and crc16(buf[:-2]) == struct.unpack('<H', buf[-2:])[0] else None
def read(ser, a): r = xfer(ser, 3, struct.pack('>HH', a, 1)); return struct.unpack('>H', r[3:5])[0] if r and len(r) >= 5 else None
def write(ser, a, v):
    xfer(ser, 6, struct.pack('>HH', a, v)); time.sleep(0.03); rb = read(ser, a)
    print('  0x%02X = %d -> readback %s' % (a, v, rb)); return rb == v
ser = serial.Serial(PORT, 19200, timeout=0.02); time.sleep(0.1)
out_en = read(ser, 0x01); print('output enable was', out_en, '| 0x%02X was' % ADDR, read(ser, ADDR))
ok = write(ser, 0x00, 1) and write(ser, 0x01, 0) and write(ser, ADDR, VAL)
write(ser, 0x01, out_en); xfer(ser, 6, struct.pack('>HH', 0x00, 506)); time.sleep(0.05); write(ser, 0x00, 0)
print('LIVE OK' if ok else 'FAILED', '| 0x%02X now' % ADDR, read(ser, ADDR), '| alarm', read(ser, 0x0E), '| output', read(ser, 0x01))
ser.close()
