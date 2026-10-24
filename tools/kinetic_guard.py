# PlatformIO pre-script for the firmware envs and env:native: the build FAILS
# unless ../Kinetic is at kinetic.pin and ../Valence at valence.pin, with the
# paths Nucleus compiles from each clean (canon_lint's two pin rules, called
# here, never restated). Nucleus compiles both sibling checkouts in place, so
# any branch checked out there goes into the image unseen.
#
# Opt-outs for development only, one per sibling: NUCLEUS_KINETIC_UNPINNED=1
# and NUCLEUS_VALENCE_UNPINNED=1 build anyway, warn, and append
# -kinetic-unpinned / -valence-unpinned to NUCLEUS_LABEL_SUFFIX, which
# git_label.py appends to the firmware's git label (/diag git=...). Never
# flash such an image as a release.
import os
import subprocess
import sys

Import("env")

root = subprocess.run(["git", "-C", env.subst("$PROJECT_DIR"), "rev-parse", "--show-toplevel"],
                      capture_output=True, text=True).stdout.strip()
sys.path.insert(0, os.path.join(root, "tools"))
from canon_lint import run_kinetic_pin_check, run_pin_check  # noqa: E402

suffix = ""
for sibling, pin, var, check in (("Kinetic", "kinetic.pin", "NUCLEUS_KINETIC_UNPINNED", run_kinetic_pin_check),
                                 ("Valence", "valence.pin", "NUCLEUS_VALENCE_UNPINNED", run_pin_check)):
    problems = check()
    if not problems:
        continue
    lines = "\n".join("  [%s] %s %s" % (name, where, msg) for name, where, _, _, msg in problems)
    if os.environ.get(var) == "1":
        print("\n" + "!" * 78 + "\n%s UNPINNED (%s=1): this build compiles %s code\n"
              "%s does not name. Development only; never ship it.\n" % (sibling.upper(), var, sibling, pin)
              + lines + "\n" + "!" * 78 + "\n")
        suffix += "-%s-unpinned" % sibling.lower()
    else:
        print("\nkinetic_guard: ../%s is not the pinned %s; build refused.\n%s\n"
              "Check out %s in ../%s (git -C ../%s checkout <sha>), or set\n"
              "%s=1 for development (the image is stamped).\n" % (sibling, pin, lines, pin, sibling, sibling, var))
        env.Exit(1)
if suffix:
    env["NUCLEUS_LABEL_SUFFIX"] = suffix
