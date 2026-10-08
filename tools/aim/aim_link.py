"""Open the AIM drive's Modbus RTU link (slave 1), at --baud or by a hunt: the
bench 57AIM30 answers at 19200, the machine's 60AIM40F at 115200.
The hunt is one FC 0x03 read of 0x02 per baud. Never probe with a write, and
never touch 0x00, 0x03, 0x04 or 0x14 here: the arm, the baud envelope and the
save flag (docs/drive-bench-2026-10-07.md, Registers).
"""
import struct, time
import serial

HUNT = (19200, 115200)

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8): c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

def pop_baud(argv):
    """Remove '--baud N' from argv in place; return N, or None when absent."""
    if '--baud' not in argv: return None
    i = argv.index('--baud'); n = int(argv[i + 1]); del argv[i:i + 2]
    return n

def _answers(ser, slave):
    f = bytes([slave, 3]) + struct.pack('>HH', 0x02, 1); f += struct.pack('<H', crc16(f))
    ser.reset_input_buffer(); ser.write(f); ser.flush()
    t = ser.timeout; ser.timeout = 0.08; buf = ser.read(7); ser.timeout = t
    return len(buf) >= 5 and buf[0] == slave and crc16(buf[:-2]) == struct.unpack('<H', buf[-2:])[0]

def open_drive(port, baud=None, timeout=0.02, slave=1):
    """serial.Serial at baud, or at the first of HUNT that answers; timeout is the caller's read timeout."""
    ser = serial.Serial(port, baud or HUNT[0], timeout=timeout)
    if baud:
        print('drive link: %s at %d (--baud)' % (port, baud)); return ser
    for b in HUNT:
        ser.baudrate = b; time.sleep(0.05)
        if _answers(ser, slave):
            print('drive link: %s at %d (hunted)' % (port, b)); return ser
    ser.close()
    raise SystemExit('no answer on %s at %s' % (port, ' or '.join(map(str, HUNT))))
