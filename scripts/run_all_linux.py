#!/usr/bin/env python3
"""
run_all_linux.py — Orchestrator for Northstrike Linux SIM

Order:
  1) clean_env_linux.py
  2) launch_gazebo_world_linux.py [--headless]   (NON-BLOCKING)
  3) launch_px4_linux.py -n N [--detach optional, with readiness check inside]
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
args = parser.parse_args()

HEADLESS = as_bool(args.headless)
LAUNCH_QGC = as_bool(args.qgc)
DETACH = as_bool(args.detach)

def main():
    log.info("=== Northstrike Orchestrator ===")
    log.info("Repo: %s", REPO_ROOT)
    log.info("Drones: %d | Headless: %s | QGC: %s | Detach: %s", args.drones, HEADLESS, LAUNCH_QGC, DETACH)

    try:
        # 1) Clean
        run_checked([sys.executable, str(CLEAN)])
        time.sleep(args.sleep_between)

        # 2) Gazebo (NON-BLOCKING)
        gz_cmd = [sys.executable, str(GAZEBO)]
        if HEADLESS:
            gz_cmd += ["--headless"]  # flag-only
        gz_proc = start_proc(gz_cmd)
        # Give Gazebo a moment to bring up the world before injecting models
        time.sleep(max(2.0, args.sleep_between))

        # 3) PX4 SITL (readiness check is inside launch_px4_linux.py)
        px4_cmd = [sys.executable, str(PX4), "-n", str(args.drones)]
        if DETACH:
            px4_cmd += ["--detach"]
        run_checked(px4_cmd)  # blocks until PX4 is ready (or exits early if --detach after ready)

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
