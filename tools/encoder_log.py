"""Log the AIM drive's encoder over USB-RS485 at a fixed cadence, as a CSV.
The auditor for the harness drift test (bd val-091.13 item 7): the P4 renders
quadrature into the buffer, the drive follows it, and this reads what the
drive's encoder actually saw. Lay the CSV against the P4 census line.
Modbus RTU 8N1, slave 1, FC 0x03 at 0x16 count 2 (encoder LO/HI, signed 32,
low word first), the pair read ServoModbus.cpp uses. ONE MASTER ON THE BUS:
unplug the S3's Modbus connector first, or both masters read garbage.
Scale: 32768 counts/motor-rev x 2:1 reduction / 78.5398 mm per drum-rev
= 834.4 counts/mm, so 4.000 counts per P4 quadrature edge (208.608 edges/mm).
A measured ratio off from 4.000 is a gearing mismatch, louder than any drift.
Usage:  python tools/encoder_log.py COMx [--hz 20] [--seconds 0] [--out artifacts/encoder.csv]
"""
import argparse, os, struct, sys, time
import serial

COUNTS_PER_MM = 32768.0 * 2.0 / (3.14159265 * 25.0)   # 834.4; drum 25 mm
SLAVE = 1


def crc16(b):
    c = 0xFFFF
    for x in b:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


assert crc16(bytes([1, 4, 2, 0xFF, 0xFF])) == 0x80B8   # the textbook vector, LSB first on the wire


def read_pair(port):
    req = bytes([SLAVE, 3, 0, 0x16, 0, 2]); c = crc16(req)
    port.reset_input_buffer(); port.write(req + bytes([c & 0xFF, c >> 8]))
    r = port.read(9)                                  # addr fc len lo_hi lo_lo hi_hi hi_lo crc crc
    if len(r) != 9 or r[0] != SLAVE or r[1] != 3 or r[2] != 4: return None
    if crc16(r[:7]) != (r[7] | r[8] << 8): return None
    lo, hi = struct.unpack('>HH', r[3:7])
    return struct.unpack('<i', struct.pack('<HH', lo, hi))[0]


def open_drive(com):
    for baud in (115200, 19200):                      # the order the firmware probes
        p = serial.Serial(com, baud, timeout=0.05)
        if read_pair(p) is not None:
            print('drive at %d baud' % baud); return p
        p.close()
    sys.exit('no drive answered on %s at 115200 or 19200 (S3 still on the bus?)' % com)


ap = argparse.ArgumentParser()
ap.add_argument('com'); ap.add_argument('--hz', type=float, default=20.0)
ap.add_argument('--seconds', type=float, default=0.0, help='0 = until Ctrl-C')
ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'artifacts', 'encoder.csv'))
a = ap.parse_args()
p = open_drive(a.com)
c0 = read_pair(p); t0 = time.perf_counter(); n = miss = 0; lo = hi = 0
with open(a.out, 'w') as f:
    f.write('t_s,counts,rel_counts,rel_mm\n')
    try:
        while not a.seconds or time.perf_counter() - t0 < a.seconds:
            c = read_pair(p); t = time.perf_counter() - t0
            if c is None: miss += 1; continue
            rel = c - c0; lo = min(lo, rel); hi = max(hi, rel); n += 1
            f.write('%.4f,%d,%d,%.4f\n' % (t, c, rel, rel / COUNTS_PER_MM))
            if n % int(a.hz) == 0:
                print('\r%7.1f s  enc %+9d  rel %+8d = %+9.3f mm   miss %d' % (t, c, rel, rel / COUNTS_PER_MM, miss), end='')
            time.sleep(max(0.0, (n + 1) / a.hz - (time.perf_counter() - t0)))
    except KeyboardInterrupt: pass
print('\n%d samples, %d missed; excursion %+d..%+d counts (%+.3f..%+.3f mm); final rel %+d counts = %+.3f mm -> %s'
      % (n, miss, lo, hi, lo / COUNTS_PER_MM, hi / COUNTS_PER_MM, rel, rel / COUNTS_PER_MM, a.out))
