# PlatformIO pre-script, run on every `pio run`: the git label that lands in
# esp_app_desc.version (the boot log's "App version", /diag's git=).
#
# pio re-runs CMake only when a CMakeLists, sdkconfig or the framework
# changes (espidf.py is_cmake_reconfigure_required), never when HEAD moves,
# so IDF's own configure-time git describe goes stale (bd val-73g). A changed
# label here deletes CMakeCache.txt, which forces that re-run; an unchanged
# one costs two git calls. CMakeLists.txt reads $BUILD_DIR/git_label.txt.
#
# -dirty counts tracked changes only, like git describe --dirty, and never
# .beads/issues.jsonl, which bd rewrites on its own.
import os
import subprocess

Import("env")


def git(*args):
    try:
        return subprocess.run(["git", "--no-optional-locks", "-C", env.subst("$PROJECT_DIR"), *args],
                              capture_output=True, text=True).stdout.strip()
    except OSError:
        return ""


label = git("rev-parse", "--short", "HEAD") or "unknown"
if git("status", "--porcelain", "--untracked-files=no", "--", ":/", ":(top,exclude).beads/issues.jsonl"):
    label += "-dirty"

build_dir = env.subst("$BUILD_DIR")
stamp = os.path.join(build_dir, "git_label.txt")
old = open(stamp).read() if os.path.isfile(stamp) else None
if label != old:
    os.makedirs(build_dir, exist_ok=True)
    with open(stamp, "w") as f:
        f.write(label)
    cache = os.path.join(build_dir, "CMakeCache.txt")
    if os.path.isfile(cache):
        os.remove(cache)
    print("git label %s -> %s: CMake reconfigures" % (old, label))
