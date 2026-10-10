"""Write the hub's config-set keys (0x3000). python hub_config.py [win_min win_max jog_v jog_a in_v in_a in_j [max_rail]]
No arguments restores keys 1 to 7: window 0 to 100 and the factory jog and input limits (valence_config.h DEFAULT_*).
Key 8 (max_rail) is written only when an eighth argument is given; after a real home the hub clamps it and the window
to the measured stroke (val-3kd)."""
import os, sys, time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'Valence', 'tools')))  # the sibling Valence checkout
import valence_probe as sp, websocket
args = [float(x) for x in sys.argv[1:]]
if len(args) not in (0, 7, 8): raise SystemExit(__doc__)
vals = args or [0.0, 100.0, 50.0, 200.0, 1200.0, 100000.0, 20000000.0]
ip = '192.168.1.118'
token = None
for _ in range(40):
    token = sp.mint_uitoken(ip)
    if token: break
    time.sleep(0.137)
ws = websocket.create_connection('ws://%s:82/' % ip, subprotocols=[sp.WS_SUBPROTOCOL], timeout=5.0)
sp.send_frame(ws, sp.FRAME['HELLO'], 0, sp.build_hello('probe', 'hub_config.py', os.urandom(8), token=token))
deadline = time.time() + 5; welcome = None
while time.time() < deadline:
    got = sp.recv_frame(ws, deadline)
    if got is None: break
    if got[0]['type'] == sp.FRAME['WELCOME']: welcome = sp.cb_decode_full(got[1]); break
if welcome is None: raise SystemExit('no WELCOME')
sp.send_frame(ws, sp.FRAME['CATALOG_READY'], 0, welcome.get(sp.K['catalog_etag'], b''))
if len(vals) == 8: sp.send_frame(ws, sp.FRAME['INTENT'], 0x3000, sp.build_intent(0x3000, 1, [(8, sp.cb_f32(vals[7]))])); time.sleep(0.2)
sp.send_frame(ws, sp.FRAME['INTENT'], 0x3000, sp.build_intent(0x3000, 2, [(k + 1, sp.cb_f32(v)) for k, v in enumerate(vals[:7])]))
nacks = []; t_end = time.time() + 1.0
while time.time() < t_end:
    got = sp.recv_frame(ws, t_end)
    if got is None: break
    if got[0]['type'] == sp.FRAME['NACK']: nacks.append(sp.cb_decode_full(got[1]))
sp.send_frame(ws, sp.FRAME['GOODBYE'], 0, sp.build_goodbye(0)); ws.close()
print('config-set', dict(zip(['win_min', 'win_max', 'jog_v', 'jog_a', 'in_v', 'in_a', 'in_j', 'max_rail'], vals)), '| NACKs:', nacks or 'none')
