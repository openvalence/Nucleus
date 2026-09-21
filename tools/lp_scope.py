"""Shared DHO4204 capture and quadrature decode for the LP-core instruments.
Traps 1-3 and the probe map are documented in lp_leg_capture.py, the
instrument whose stamps first measured them. Connects at import.
NEVER switch this instrument to 50 ohm and never touch the :CALibration tree.
"""
import socket, numpy as np, time, sys, os

HOST = '192.168.1.12'
PORT = 5555
THR = 1.65                 # trap 2: the logic midpoint, never the record's own
GAP_S = 0.050              # a span this long between edges separates two bursts
ART = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'artifacts')
OUT = os.path.join(ART, 'leg_t.npy')

s = socket.create_connection((HOST, PORT), 5); s.settimeout(30)
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)


def ask(c):
    s.sendall((c + chr(10)).encode()); b = b''
    while not b.endswith(b'\n'):
        x = s.recv(1 << 16)
        if not x: break
        b += x
    return b.decode('latin-1').strip()


def send(c): s.sendall((c + chr(10)).encode())


def blk():
    h = b''
    while len(h) < 2: h += s.recv(2 - len(h))
    n = int(h[1:2]); L = b''
    while len(L) < n: L += s.recv(n - len(L))
    t = int(L); o = bytearray()
    while len(o) < t:
        c = s.recv(min(65536, t - len(o)))
        if not c: break
        o += c
    s.recv(1); return bytes(o)


def pull(ch):
    send(':WAV:SOUR CHAN%d' % ch); send(':WAV:MODE NORM'); send(':WAV:MODE MAX')
    send(':WAV:FORM WORD')
    MD = int(ask(':WAV:POIN?')); pre = ask(':WAV:PRE?').split(',')
    DT = float(pre[4]); XORG = float(pre[5])
    yinc, yorg, yref = float(pre[7]), float(pre[8]), float(pre[9])
    raw = bytearray(); i = 1
    while i <= MD:
        j = min(i + 199999, MD)
        send(':WAV:STAR %d' % i); send(':WAV:STOP %d' % j); send(':WAV:DATA?')
        raw += blk(); i = j + 1
    return (np.frombuffer(bytes(raw), dtype='<u2').astype(np.float64)
            - yref - yorg) * yinc, DT, XORG


def shoot(window_s, dwell_s):
    # Trap 1: NORM sweep, run, dwell past one full acquisition, stop. No status
    # poll anywhere -- this unit has none to give.
    send(':STOP'); send(':ACQ:MDEP 1M')
    send(':WAV:MODE NORM'); send(':WAV:MODE MAX')
    send(':TIM:SCAL %g' % (window_s / 10.0))
    send(':TIM:OFFS %g' % (window_s * 0.45))
    send(':TRIG:MODE EDGE'); send(':TRIG:EDGE:SOUR CHAN2')
    send(':TRIG:EDGE:SLOP POS'); send(':TRIG:EDGE:LEV %g' % THR)
    send(':TRIG:SWE NORM')
    ask(':TRIG:SWE?')          # flush the setup before the run
    send(':RUN'); time.sleep(dwell_s); send(':STOP'); time.sleep(0.3)


# Gray index over the state word (B<<1)|A: 00 -> 01 -> 11 -> 10 is the forward
# walk, i.e. A leading B by 90 degrees. Step +1 is A-leads, -1 is B-leads.
IX = {0: 0, 1: 1, 3: 2, 2: 3}


def decode():
    B, DT, XORG = pull(1)
    A, _, _ = pull(2)
    n = min(len(A), len(B)); A, B = A[:n], B[:n]
    st = ((B > THR).astype(np.int8) << 1) | (A > THR).astype(np.int8)
    c = np.flatnonzero(np.diff(st) != 0)
    t = c * DT
    seq = st[c + 1]
    g = np.array([(IX[seq[k + 1]] - IX[seq[k]]) % 4
                  for k in range(len(seq) - 1)], dtype=np.int8)
    g = np.where(g == 1, 1, np.where(g == 3, -1, 0))   # 0 marks an illegal step
    return dict(n=n, DT=DT, XORG=XORG, t=t, g=g, seq=seq, total=len(c),
                bad=int(np.sum(g == 0)),
                fwd=int(np.sum(g == 1)), rev=int(np.sum(g == -1)))


