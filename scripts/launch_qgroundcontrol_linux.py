#!/usr/bin/env python3
import os, sys, subprocess, argparse, shutil, stat, logging, time

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

parser = argparse.ArgumentParser(description="Launch QGroundControl AppImage")
# --qgc kept for backward-compat, but defaults to true now
parser.add_argument("--qgc", type=str, default="true", help="true/false (kept for backward compatibility)")
args = parser.parse_args()

def as_bool(v): return str(v).lower() in ("1","true","yes","y","on")

# Resolve QGC path
candidates = [
    os.environ.get("QGC_APPIMAGE"),
    os.path.expanduser("~/dev/northstrike-linux-sim/QGroundControl.AppImage"),
    os.path.expanduser("~/dev/QGroundControl.AppImage"),
    os.path.expanduser("~/Downloads/QGroundControl.AppImage"),
    "/usr/local/bin/QGroundControl.AppImage",
    "/usr/bin/QGroundControl.AppImage",
]
QGC_PATH = next((p for p in candidates if p and os.path.exists(p)), None)

if not QGC_PATH:
    logging.error("QGroundControl.AppImage not found. Set $QGC_APPIMAGE or place it under ~/dev/northstrike-linux-sim/")
    sys.exit(1)

# Ensure executable bit
st = os.stat(QGC_PATH)
if not (st.st_mode & stat.S_IXUSR):
    try:
        os.chmod(QGC_PATH, st.st_mode | stat.S_IXUSR)
    except Exception as e:
        logging.warning(f"Could not chmod +x {QGC_PATH}: {e}")

env = dict(os.environ)
# Wayland fix: force XCB unless user already set this
env.setdefault("QT_QPA_PLATFORM", "xcb")

logging.info(f"Launching QGC: {QGC_PATH}")
subprocess.Popen([QGC_PATH], env=env)
