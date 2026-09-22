#!/usr/bin/env python3
"""ota -- deploy a firmware image to the Nucleus P4 over POST /ota, and prove it.

Three steps, and the third is the one that makes this a deploy rather than an
upload: read the version the hub is REPORTING now, push the image, then wait
until a hub answers with a DIFFERENT version. "Upload completed" is not
"deployed" (C-8); the only evidence that counts is the running hub's identity.

Constraints:
- RAW BODY, NEVER MULTIPART. The firmware streams the request straight into
  esp_ota_write. urllib sends exactly the bytes and a Content-Length, which is
  what the handler reads to size the erase.
- THE TOKEN IS NEVER A COMMAND-LINE ARGUMENT by default: it is read out of the
  git-ignored flagship_p4/src/secrets.h, the same file the firmware compiles
  it from, so the two cannot drift. --token overrides for a foreign board.
- The version read is a tokenless-capable WATCH session: WELCOME carries the
  identity map regardless of tier, so a 429 on the mint never blocks a deploy.
- A NEW IMAGE IS ON TRIAL. It reverts unless it brings the hub and WiFi up, so
  a poll that times out means the board is BACK ON THE OLD SLOT, not bricked.

See: .claude/rules/build-test-deploy.md, bd val-091.16
"""

import argparse
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request

_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.abspath(os.path.join(_ROOT, "..", "Valence", "tools")))

import valence_probe as sp  # noqa: E402
import websocket  # noqa: E402

DEFAULT_IMAGE = os.path.join(_ROOT, "flagship_p4", ".pio", "build", "flagship_p4", "firmware.bin")
SECRETS = os.path.join(_ROOT, "flagship_p4", "src", "secrets.h")


def read_token(path):
    try:
        text = open(path, encoding="utf-8", errors="replace").read()
    except OSError as e:
        return None, "cannot read %s: %s" % (path, e)
    m = re.search(r'#\s*define\s+SECRET_OTA_TOKEN\s+"([^"]+)"', text)
    if not m:
        return None, "no SECRET_OTA_TOKEN in %s (see secrets.example.h)" % path
    return m.group(1), None


def hub_version(ip, port, timeout=5.0):
    """The fw_version the running hub reports, or None if nothing answered."""
    token = sp.mint_uitoken(ip, retries=2, timeout=2.0)   # best effort; watch tier is enough
    try:
        ws = websocket.create_connection("ws://%s:%d/" % (ip, port),
                                         subprotocols=[sp.WS_SUBPROTOCOL], timeout=timeout)
    except Exception:
        return None
    try:
        sp.send_frame(ws, sp.FRAME["HELLO"], 0,
                      sp.build_hello("probe", "ota.py", os.urandom(8), token=token))
        deadline = time.time() + timeout
        while time.time() < deadline:
            got = sp.recv_frame(ws, deadline)
            if got is None:
                return None
            if got[0]["type"] == sp.FRAME["WELCOME"]:
                w = sp.cb_decode_full(got[1])
                ident = w.get(sp.K["identity"]) or {}
                v = ident.get(2) if isinstance(ident, dict) else None   # identity_keys 2
                return v.decode() if isinstance(v, bytes) else v
    except Exception:
        return None
    finally:
        try:
            ws.close()
        except Exception:
            pass
    return None


def image_state(ip, timeout=10.0):
    """The img= field of the /diag header: 'valid' once the image bought itself.

    ?from= past the end returns the header and the footer and nothing between,
    so this costs two lines rather than the whole archive.
    """
    try:
        with urllib.request.urlopen("http://%s/diag?from=4294967295" % ip, timeout=timeout) as r:
            head = r.read().decode("utf-8", "replace").splitlines()[0]
    except Exception:
        return None
    m = re.search(r"\bimg=(\w+)", head)
    return m.group(1) if m else None


def post_image(ip, image, token, timeout):
    body = open(image, "rb").read()
    req = urllib.request.Request("http://%s/ota" % ip, data=body, method="POST")
    req.add_header("X-OTA-Token", token)
    req.add_header("Content-Type", "application/octet-stream")
    t0 = time.time()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read().decode("utf-8", "replace"), time.time() - t0, len(body)
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode("utf-8", "replace"), time.time() - t0, len(body)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.1.118")
    ap.add_argument("--port", type=int, default=82, help="the Valence WS port, for the version read")
    ap.add_argument("--image", default=DEFAULT_IMAGE)
    ap.add_argument("--token", default=None, help="overrides the value read from secrets.h")
    ap.add_argument("--secrets", default=SECRETS)
    ap.add_argument("--timeout", type=float, default=300.0, help="seconds for the upload itself")
    ap.add_argument("--wait", type=float, default=150.0, help="seconds to wait for the new version")
    ap.add_argument("--expect", default=None, help="fail unless the board comes back as this version")
    args = ap.parse_args()

    if not os.path.isfile(args.image):
        print("FAIL: no image at %s (build it first)" % args.image)
        return 1
    token = args.token
    if token is None:
        token, err = read_token(args.secrets)
        if token is None:
            print("FAIL: %s" % err)
            return 1

    before = hub_version(args.ip, args.port)
    print("running : %s" % (before or "<no answer>"))
    print("image   : %s (%u B)" % (args.image, os.path.getsize(args.image)))

    status, body, secs, sent = post_image(args.ip, args.image, token, args.timeout)
    print("POST /ota -> %s in %.1f s (%.0f kB/s)" % (status, secs, sent / 1024.0 / max(secs, 1e-3)))
    for line in body.strip().splitlines():
        print("  %s" % line)
    if status != 200:
        return 1

    # The board reboots into a PENDING_VERIFY image, so a new version answering
    # is NOT the end of the test: the image also has to buy itself, and until
    # it does the next reset takes it away. Wait for both. Anything else inside
    # the window means the rollback worked, not that the tool failed.
    t0 = time.time()
    deadline = t0 + args.wait
    seen = None
    while time.time() < deadline:
        time.sleep(3.0)
        now = hub_version(args.ip, args.port)
        if not now or now == before:
            continue
        if seen != now:
            seen = now
            print("booted  : %s (%.0f s), waiting for it to buy itself" % (now, time.time() - t0))
        if image_state(args.ip) == "valid":
            print("deployed: %s, image bought (%.0f s after the upload)" % (now, time.time() - t0))
            if args.expect and now != args.expect:
                print("FAIL: expected %s" % args.expect)
                return 1
            return 0
    back = hub_version(args.ip, args.port)
    print("FAIL: %s after %.0f s, img=%s -- the image never bought itself"
          % (back or "<no answer>", args.wait, image_state(args.ip)))
    return 1


if __name__ == "__main__":
    sys.exit(main())
