"""Print the hub's kinetic-diag (0x1111) counters once: plans, failures, anomalies by
kind, the plan times (us) and the stream ingress counters. Moves nothing.
python diag_read.py [ip]"""
import os, sys, time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'Valence', 'tools')))  # the sibling Valence checkout
import valence_probe as sp, websocket
ip = sys.argv[1] if len(sys.argv) > 1 else '192.168.1.118'
token = sp.mint_uitoken(ip, retries=4, timeout=2.0)
ws = websocket.create_connection('ws://%s:82/' % ip, subprotocols=[sp.WS_SUBPROTOCOL], timeout=5.0)
sp.send_frame(ws, sp.FRAME['HELLO'], 0, sp.build_hello('probe', 'diag_read.py', os.urandom(8), token=token))
deadline = time.time() + 5
while time.time() < deadline:
    got = sp.recv_frame(ws, deadline)
    if got is None: raise SystemExit('no WELCOME')
    if got[0]['type'] == sp.FRAME['WELCOME']: welcome = sp.cb_decode_full(got[1]); break
sp.send_frame(ws, sp.FRAME['CATALOG_READY'], 0, welcome.get(sp.K['catalog_etag'], b''))
sp.send_frame(ws, sp.FRAME['SUBSCRIBE'], 0, sp.build_subscribe([(sp.CH_MOTION_DIAG, 2.0, 0)]))
deadline = time.time() + 4
while time.time() < deadline:
    got = sp.recv_frame(ws, deadline)
    if got is None: break
    hdr, payload = got
    if hdr['type'] == sp.FRAME['STATE'] and hdr['channel'] == sp.CH_MOTION_DIAG:
        d = sp.decode_motion_diag(payload)
        print({k: d[k] for k in ('plans', 'failures', 'anomalies', 'mode', 'plan_us_last', 'plan_us_max', 'plan_us_avg', 'sync_bundles', 'sync_samples', 'sync_enqueued', 'sync_dropped', 'sync_seg_bundles')})
        print('by_kind', {k: v for k, v in d['by_kind'].items() if v})
        break
sp.send_frame(ws, sp.FRAME['GOODBYE'], 0, sp.build_goodbye(0)); ws.close()
