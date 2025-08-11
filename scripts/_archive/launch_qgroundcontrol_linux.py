#!/usr/bin/env python3
import os, subprocess, sys, time

APP = os.environ.get("QGC_APPIMAGE")
if not APP or not os.path.isfile(APP):
    print("[ERROR] QGC_APPIMAGE not set or file missing."); sys.exit(1)

def is_running():
    try:
        out = subprocess.check_output(["pgrep","-f","QGroundControl"]).decode().strip()
        return bool(out)
    except subprocess.CalledProcessError:
        return False

def try_launch(args, env, tag):
    try:
        p = subprocess.Popen(args, env=env)
        time.sleep(3.0)
        if is_running():
            print(f"[INFO] QGC up ({tag}) pid={p.pid}")
            return True
    except Exception as e:
        print(f"[WARN] {tag} failed: {e}")
    return False

# Start fresh (avoid off-screen geometry quirks)
cfg = os.path.expanduser("~/.config/QGroundControl")
if os.path.isdir(cfg):
    try: os.rename(cfg, cfg + ".bak")
    except Exception: pass

base = os.environ.copy()
base.setdefault("QT_QPA_PLATFORM","xcb")  # force X11

# 1) Normal run
if try_launch([APP, "--clear-settings"], base, "normal"):
    sys.exit(0)

# 2) Software GL fallback (works for your system)
soft = base.copy()
soft["QT_OPENGL"] = "software"
soft["QSG_RHI_BACKEND"] = "software"
if try_launch([APP, "--clear-settings"], soft, "software-GL"):
    sys.exit(0)

# 3) AppImage extract-and-run + software GL (FUSE issues)
if try_launch([APP, "--appimage-extract-and-run", "--clear-settings"], soft, "extract-and-run+software-GL"):
    sys.exit(0)

print("[ERROR] QGC failed to start. Check GPU drivers/OpenGL; try foreground run with:")
print('  QT_QPA_PLATFORM=xcb QT_OPENGL=software QSG_RHI_BACKEND=software "$QGC_APPIMAGE" --clear-settings')
sys.exit(2)
