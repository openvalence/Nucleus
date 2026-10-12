#!/usr/bin/env python3
"""sim_link_test.py -- a dead or wedged WebSocket peer costs the other sessions
nothing (bd val-ni0). Launches valencesim on a loopback port and runs:

  wedged  a client that stops reading: an observer keeps its STATE rate, a
          fresh client gets WELCOME, and the wedged one is disconnected
  rst     a client that resets without a close frame: a fresh client gets WELCOME
  silent  a client that goes dark with its socket open: its slot is reaped

Exit 0 when every check holds. Usage:
    python tools/sim_link_test.py [--exe sim/valencesim/build/valencesim.exe]
"""
import argparse
import base64
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "Valence", "tools"))
import valence_probe as vp  # noqa: E402

PORT = 18382


def connect(rcvbuf=None):
    s = socket.socket()
    if rcvbuf:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
    s.settimeout(5)
    s.connect(("127.0.0.1", PORT))
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((f"GET / HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
               f"Sec-WebSocket-Protocol: valence.v1\r\n\r\n").encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        c = s.recv(4096)
        if not c:
            raise RuntimeError("closed during upgrade")
        buf += c
    return s, buf.split(b"\r\n\r\n", 1)[1]


def send(s, frame):
    mask = os.urandom(4)
    n = len(frame)
    hdr = bytes([0x82]) + (bytes([0x80 | n]) if n < 126 else bytes([0xFE]) + struct.pack(">H", n))
    s.sendall(hdr + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(frame)))


def frames(s, buf, deadline):
    while time.time() < deadline:
        while len(buf) >= 2:
            n, off = buf[1] & 0x7F, 2
            if n == 126:
                if len(buf) < 4:
                    break
                n, off = struct.unpack(">H", buf[2:4])[0], 4
            if len(buf) < off + n:
                break
            yield buf[0] & 0x0F, buf[off:off + n]
            buf = buf[off + n:]
        try:
            s.settimeout(max(0.05, deadline - time.time()))
            c = s.recv(65536)
        except socket.timeout:
            continue
        except OSError:
            return
        if not c:
            return
        buf += c


def join(name, subs=None, rcvbuf=None):
    """HELLO, wait for WELCOME, send CATALOG_READY. Returns the socket or None."""
    s, rest = connect(rcvbuf)
    send(s, vp.encode_frame(0x00, 0, vp.build_hello_full("probe", name, os.urandom(8), subscriptions=subs)))
    for op, p in frames(s, rest, time.time() + 5):
        if op == 2 and p[:1] == b"\x01":
            w = vp.cb_decode_full(p[8:])
            send(s, vp.encode_frame(0x19, 0, w.get(vp.K["catalog_etag"], b"")))
            return s
    s.close()
    return None


def rst(s):
    if s is None:
        return
    s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
    s.close()


FAST = [(vp.CH_MOTION, 100.0, 1), (vp.CH_PLAN_STRIP, 100.0, 1), (vp.CH_MOTION_DIAG, 10.0, 0)]


def wedged(results):
    obs = join("link-observer", [(vp.CH_MOTION, 50.0, 1)])
    stamps, stop = [], threading.Event()

    def observe():
        last_ping = 0.0
        for op, p in frames(obs, b"", time.time() + 120):
            if stop.is_set():
                return
            now = time.time()
            if op == 2 and p[:1] == b"":
                stamps.append(now)
            if now - last_ping > 0.5:
                send(obs, vp.encode_frame(0x03, 0, b""))
                last_ping = now

    threading.Thread(target=observe, daemon=True).start()
    victim = join("link-wedged", FAST, rcvbuf=1024)
    if victim is None:
        results.append(("wedged: joins", False))
        return
    t0, gone = time.time(), False
    while time.time() - t0 < 45:   # proves life, never reads
        try:
            send(victim, vp.encode_frame(0x03, 0, b""))
        except OSError:
            gone = True
            break
        time.sleep(0.3)
    t1 = time.time()
    fresh = join("link-fresh-1")
    stop.set()
    # STATE frames per whole second while the victim was wedged; a frozen hub reads 0.
    counts = [sum(1 for x in stamps if t0 + i <= x < t0 + i + 1) for i in range(int(t1 - t0))]
    low = [c for c in counts if c < 40]
    results.append(("wedged: the hub disconnects it", gone))
    results.append((f"wedged: the observer keeps 50 Hz (min {min(counts or [0])}/s)", bool(counts) and not low))
    results.append(("wedged: a fresh client gets WELCOME", fresh is not None))
    for s in (obs, fresh):
        if s:
            rst(s)


def reset(results):
    victim = join("link-rst", FAST)
    time.sleep(2)
    rst(victim)
    time.sleep(1)
    fresh = join("link-fresh-2")
    results.append(("rst: a fresh client gets WELCOME", fresh is not None))
    if fresh:
        rst(fresh)


def silent(results, exe_log):
    victim = join("link-silent")   # no subscriptions, no reads, no PINGs, socket held open
    if victim is None:
        results.append(("silent: joins", False))
        return
    t0 = time.time()
    reaped = False
    while time.time() - t0 < 20 and not reaped:
        time.sleep(0.5)
        reaped = "silent >" in open(exe_log, encoding="utf-8", errors="replace").read()
    results.append(("silent: the slot is reaped", reaped))
    rst(victim)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=os.path.join(HERE, "..", "sim", "valencesim", "build", "valencesim.exe"))
    a = ap.parse_args()
    tmp = tempfile.mkdtemp(prefix="sim_link_")
    log = os.path.join(tmp, "sim.log")
    with open(log, "w") as lf:
        sim = subprocess.Popen([a.exe, "--port", str(PORT), "--bind", "127.0.0.1", "--http", str(PORT + 1),
                                "--headless", "--no-discovery", "--no-mdns", "--no-estop-udp",
                                "--state", os.path.join(tmp, "st")], stdout=lf, stderr=subprocess.STDOUT)
    results = []
    try:
        time.sleep(2)
        wedged(results)
        reset(results)
        silent(results, log)
    finally:
        sim.kill()
    for name, ok in results:
        print(("PASS " if ok else "FAIL ") + name)
    sys.exit(0 if all(ok for _, ok in results) else 1)


if __name__ == "__main__":
    main()
