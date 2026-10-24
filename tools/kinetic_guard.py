# PlatformIO pre-script for the firmware envs and env:native: the build FAILS
# unless ../Kinetic is at kinetic.pin with the paths Nucleus compiles clean
# (canon_lint's kinetic.pin rule, called here, never restated). Nucleus
# compiles the sibling checkout in place, so any branch checked out there
# goes into the image unseen.
#
# Opt-out for planner development only: NUCLEUS_KINETIC_UNPINNED=1 builds
# anyway, warns, and sets NUCLEUS_LABEL_SUFFIX, which git_label.py appends to
# the firmware's git label (/diag git=...-kinetic-unpinned). Never flash such
# an image as a release.
import os
import subprocess
import sys

Import("env")

root = subprocess.run(["git", "-C", env.subst("$PROJECT_DIR"), "rev-parse", "--show-toplevel"],
                      capture_output=True, text=True).stdout.strip()
sys.path.insert(0, os.path.join(root, "tools"))
from canon_lint import run_kinetic_pin_check  # noqa: E402

problems = run_kinetic_pin_check()
if problems:
    lines = "\n".join("  [%s] %s %s" % (name, where, msg) for name, where, _, _, msg in problems)
    if os.environ.get("NUCLEUS_KINETIC_UNPINNED") == "1":
        print("\n" + "!" * 78 + "\nKINETIC UNPINNED (NUCLEUS_KINETIC_UNPINNED=1): this build compiles planner\n"
              "code kinetic.pin does not name. Development only; never ship it.\n" + lines + "\n" + "!" * 78 + "\n")
        env["NUCLEUS_LABEL_SUFFIX"] = "-kinetic-unpinned"
    else:
        print("\nkinetic_guard: ../Kinetic is not the pinned planner; build refused.\n" + lines +
              "\nCheck out kinetic.pin in ../Kinetic (git -C ../Kinetic checkout <sha>), or set\n"
              "NUCLEUS_KINETIC_UNPINNED=1 for planner development (the image is stamped).\n")
        env.Exit(1)
