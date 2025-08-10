#!/usr/bin/env python3
"""
run_all_linux.py — Orchestrator for Northstrike Linux SIM

Order:
  1) clean_env_linux.py
  2) launch_gazebo_world_linux.py [--headless]   (NON-BLOCKING)
  3) launch_px4_linux.py -n N [-m MODEL -o OFFSET] [--detach]
  4) (optional) launch_qgroundcontrol_linux.py when --qgc true

Defaults:
  --headless true
  --qgc     false
"""

import argparse
import logging
import os
import subprocess
import sys
import time
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("run_all_linux")

def as_bool(v) -> bool:
    return str(v).strip().lower() in ("1", "true", "yes", "y", "on")

def run_checked(cmd, cwd=None, env=None):
    log.info("EXEC: %s", " ".join(cmd))
    subprocess.check_call(cmd, cwd=cwd, env=env)

def start_proc(cmd, cwd=None, env=None) -> subprocess.Popen:
    log.info("EXEC: %s", " ".join(cmd))
    return subprocess.Popen(cmd, cwd=cwd, env=env)

def rotate_logs(dir_path: Path, keep: int = 10):
    try:
        files = sorted(dir_path.glob("drone_*.txt"),
                       key=lambda p: p.stat().st_mtime,
                       reverse=True)
        for f in files[keep:]:
            f.unlink(missing_ok=True)
    except Exception as e:
        log.warning("Log rotation skipped: %s", e)

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT  = SCRIPT_DIR.parent

CLEAN = SCRIPT_DIR / "clean_env_linux.py"
GAZEBO = SCRIPT_DIR / "launch_gazebo_world_linux.py"
PX4 = SCRIPT_DIR / "launch_px4_linux.py"
QGC = SCRIPT_DIR / "launch_qgroundcontrol_linux.py"

for p in (CLEAN, GAZEBO, PX4):
    if not p.exists():
        log.error("Missing required script: %s", p)
        sys.exit(1)

parser = argparse.ArgumentParser(description="Northstrike Linux SIM Orchestrator")
parser.add_argument("--drones", type=int, default=1, help="Number of PX4 SITL instances")
parser.add_argument("--headless", type=str, default="true", help="true/false (Gazebo headless)")
parser.add_argument("--qgc", type=str, default="false", help="true/false (launch QGC)")
parser.add_argument("--sleep-between", type=float, default=2.0, help="Seconds to sleep between stages")
parser.add_argument("--detach", type=str, default="false", help="true/false - run PX4 in background")
# passthrough to PX4 launcher:
parser.add_argument("--model", type=str, default="x500", help="PX4/GZ model (default: x500)")
parser.add_argument("--offset", type=float, default=2.0, help="Spacing (m) between drones (default: 2.0)")
args = parser.parse_args()

HEADLESS = as_bool(args.headless)
LAUNCH_QGC = as_bool(args.qgc)
DETACH = as_bool(args.detach)

def main():
    log.info("=== Northstrike Orchestrator ===")
    log.info("Repo: %s", REPO_ROOT)
    log.info("Drones: %d | Headless: %s | QGC: %s | Detach: %s | Model: %s | Offset: %.2f",
             args.drones, HEADLESS, LAUNCH_QGC, DETACH, args.model, args.offset)

    try:
        # 1) Clean
        run_checked([sys.executable, str(CLEAN)])
        time.sleep(args.sleep_between)

        # Headless guard: if headless requested, make sure no GUI gz is alive
        if HEADLESS:
            try:
                subprocess.call(["pkill","-f","gz sim .* -g"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.call(["pkill","-f","gzclient"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass

        # 2) Gazebo (NON-BLOCKING)
        gz_cmd = [sys.executable, str(GAZEBO)]
        if HEADLESS:
            gz_cmd += ["--headless"]  # flag-only
        gz_proc = start_proc(gz_cmd)
        # Give Gazebo a moment to bring up the world before injecting models
        time.sleep(max(2.0, args.sleep_between))

        # 3) PX4 SITL (readiness check happens inside launch_px4_linux.py)
        px4_cmd = [sys.executable, str(PX4),
                   "-n", str(args.drones),
                   "-o", str(args.offset),
                   "-m", args.model]
        if DETACH:
            px4_cmd += ["--detach"]

        run_checked(px4_cmd)
        rotate_logs(SCRIPT_DIR / "logs", keep=10)

        # 4) QGC (optional, fire-and-forget)
        if LAUNCH_QGC and QGC.exists():
            log.info("Launching QGroundControl (optional)…")
            subprocess.Popen([sys.executable, str(QGC)], env={**os.environ})
            log.info("QGC started (if AppImage is available).")

        log.info("All launch steps completed.")
        log.info("Tip: Headless training ⇒ keep --qgc false. Debugging ⇒ pass --qgc true and export $QGC_APPIMAGE.")

    except subprocess.CalledProcessError as e:
        log.error("A step failed (exit %s). See logs above.", e.returncode)
        sys.exit(e.returncode)
    except KeyboardInterrupt:
        log.warning("Interrupted by user.")
        sys.exit(130)

if __name__ == "__main__":
    main()
