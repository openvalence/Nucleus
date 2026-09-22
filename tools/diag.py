#!/usr/bin/env python3
"""diag -- pull the Nucleus P4's diagnostics archive and keep it.

GET /diag is the whole archive, /diag/<tag> filters one GLOG tag, ?from=<seq>
resumes from the cursor the previous dump's footer printed. The dump is saved
under artifacts/ before anything is printed, because the archive dies with its
boot and a terminal scrollback is not storage.

Constraints:
- READ-ONLY. It pulls bytes and writes a file; nothing here commands the hub.
- The FOOTER is the interface: the last line is next=<seq>, and --from takes
  it back. That is what makes incremental tailing free and a paged pull
  possible without holding a stream open.
- A bare filename would land wherever the caller started, so --out defaults
  under artifacts/ (.claude/rules/repo-layout.md).

See: .claude/rules/logging-leds.md, bd val-091.17
"""

import argparse
import os
import sys
import time
import urllib.error
import urllib.request

_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ARTIFACTS = os.path.join(_ROOT, "artifacts")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.1.118")
    ap.add_argument("--tag", default=None, help="one GLOG tag, e.g. hub / ota / motion")
    ap.add_argument("--from", dest="from_seq", type=int, default=None,
                    help="resume cursor: the next= the last dump printed")
    ap.add_argument("--out", default=None, help="default: artifacts/diag-<stamp>.txt")
    ap.add_argument("--tail", type=int, default=25, help="lines to print; 0 for all")
    ap.add_argument("--timeout", type=float, default=120.0)
    args = ap.parse_args()

    url = "http://%s/diag" % args.ip
    if args.tag:
        url += "/" + args.tag
    if args.from_seq is not None:
        url += "?from=%d" % args.from_seq

    try:
        with urllib.request.urlopen(url, timeout=args.timeout) as r:
            body = r.read().decode("utf-8", "replace")
    except (urllib.error.URLError, OSError) as e:
        print("FAIL: %s: %s" % (url, e))
        return 1

    out = args.out or os.path.join(ARTIFACTS, "diag-%s.txt" % time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(body)

    lines = body.splitlines()
    print("%s -> %s (%u lines, %u B)" % (url, out, len(lines), len(body)))
    shown = lines if args.tail <= 0 else lines[-args.tail:]
    for line in shown:
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
