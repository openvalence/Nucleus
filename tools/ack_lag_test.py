#!/usr/bin/env python3
"""ack_lag_test.py -- a client whose ACKs come late keeps its STATE (bd val-kec).

A watch-tier session subscribes like Phosphor (src/model/wishes.js against the
board catalog) and reports the motion channel's rate and worst gap, PINGs
against PONGs, and every gap over 0.4 s. Watch tier only: it commands nothing.

The lag comes from the client's own egress, so run it inside WSL as root:
    tc qdisc replace dev eth0 root netem delay 30ms      # or: delay 100ms 50ms
    python3 tools/ack_lag_test.py 192.168.1.118 30
    tc qdisc del dev eth0 root
Needs websocket-client (valence_soak.py's wire layer, Valence/tools).

Pass: motion near 50 Hz, worst gap under 100 ms, pongs == pings. 0.1.48 at
+30 ms: 13.7 Hz with 5.2 s gaps; at +100 +-50 ms: 0.8 Hz, 10.5 s gaps.

Loss on the hub's own sends (the direction lwIP retransmits) needs ingress
shaping through an ifb device, filtered to the hub's address:
    ip link add ifb0 type ifb; ip link set ifb0 up
    tc qdisc add dev eth0 handle ffff: ingress
    tc filter add dev eth0 parent ffff: protocol ip u32 match ip src 192.168.1.118/32         action mirred egress redirect dev ifb0
    tc qdisc add dev ifb0 root netem loss 1%
0.1.49 at 1% for 90 s: worst motion gap 1087 ms and PONG round trip 992 ms
(one lwIP retransmission timeout); a 0.3 s blackout cost a 1.27 s gap, and
0.6 s or more parked the session on the 2 s control stall.
"""
import json
import os
import sys
import time
import types

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Valence", "tools"))
import valence_soak as vs  # noqa: E402

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.118"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

SUBS = [(0x0003, 0.0, "critical"), (0x0004, 0.0, "critical"), (0x0006, 1.0, "background"),
        (0x1100, 50.0, "elevated"), (0x1110, 30.0, "elevated"), (0x1111, 1.0, "background"),
        (0x1010, 1.0, "background"), (0x1020, 1.0, "background"), (0x000E, 0.0, "critical"),
        (0x0007, 0.0, "normal"), (0x000B, 0.0, "normal"), (0x1000, 0.0, "normal"),
        (0x1030, 0.0, "normal"), (0x1200, 0.0, "normal"), (0x1210, 0.0, "normal"), (0x0008, 0.0, "background")]
ONE_HZ = {0x0006, 0x1111, 0x1010, 0x1020}

ctx = types.SimpleNamespace(args=types.SimpleNamespace(ip=ip, port=82, timeout=6.0, sub_rate=50.0))
c = vs.SoakClient(ctx, "acklag", sub_rate=50.0, subs=SUBS)
pings = [0]
ping_at, rtts = [], []
arrivals = []
send0, recv0 = c.send, c.recv


def send(ft, ch, p, seq=0):
    if ft == vs.sp.FRAME["PING"]:
        pings[0] += 1
        ping_at.append(time.time())
    return send0(ft, ch, p, seq)


def recv(deadline):
    got = recv0(deadline)
    if got and got != "timeout":
        arrivals.append((time.time(), got[0]["type"], got[0]["channel"]))
        if got[0]["type"] == vs.sp.FRAME["PONG"] and ping_at:
            rtts.append((time.time() - ping_at.pop(0)) * 1000)
    return got


c.send, c.recv = send, recv
c.connect(timeout=6.0)
if not c.handshake(timeout=6.0):
    print(json.dumps({"handshake": False, "errors": c.m.get("errors")}))
    sys.exit(1)
pongs0 = c.m["by_frame_type"].get("PONG", 0)
pings[0] = 0
ping_at.clear()
c.start_loop()
t0 = time.time()
arrivals.clear()
time.sleep(secs)
snap = c.snapshot()
c.stop()
try:
    c.close_clean()
except Exception:  # noqa: BLE001
    pass

m = snap["state"].get("0x1100", {})
rtts.sort()
print(json.dumps({
    "secs": secs, "pings": pings[0], "pongs": snap["by_frame_type"].get("PONG", 0) - pongs0,
    "unexpected_disconnects": snap["unexpected_disconnects"],
    "motion_hz": m.get("measured_hz"), "motion_gap_max_ms": m.get("gap_max_ms"),
    "frames_rx_per_s": round(snap["frames_rx"] / secs),
    "pong_rtt_p50_ms": round(rtts[len(rtts) // 2], 1) if rtts else None,
    "pong_rtt_max_ms": round(rtts[-1], 1) if rtts else None,
}))
last = {}
for t, ty, ch in arrivals:
    k = (ty, ch)
    if k in last and t - last[k] > (1.5 if ch in ONE_HZ else 0.4):
        print("gap %5.2f s at +%6.2f s type 0x%02x ch 0x%04x" % (t - last[k], last[k] - t0, ty, ch))
    last[k] = t
