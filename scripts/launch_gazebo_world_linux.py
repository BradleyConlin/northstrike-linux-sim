#!/usr/bin/env python3
import argparse, logging, os, subprocess, sys, time
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("launch_gazebo_world")

DEFAULT_WORLD = "default.sdf"  # PX4’s default world in Tools/simulation/gz/worlds

def as_bool(v) -> bool:
    return str(v).strip().lower() in ("1","true","yes","y","on")

def find_world_path(world: str) -> str:
    # Accept absolute/relative path, or just a filename in PX4 tools dir
    if os.path.isabs(world) or os.path.exists(world):
        return world
    # PX4 tools world fallback
    px4_root = os.environ.get("PX4_ROOT", os.path.expanduser("~/dev/px4-autopilot-harmonic"))
    cand = Path(px4_root)/"Tools"/"simulation"/"gz"/"worlds"/world
    return str(cand) if cand.exists() else world

def main():
    p = argparse.ArgumentParser(description="Launch Gazebo (Harmonic) world")
    p.add_argument("--world", default=DEFAULT_WORLD, help="SDF world file or name (default: default.sdf)")
    p.add_argument("--headless", action="store_true", help="Run server-only (no GUI client)")
    args = p.parse_args()

    world_path = find_world_path(args.world)
    env = os.environ.copy()

    # Always make sure PX4 resources are on GZ resource path
    px4_root = Path(env.get("PX4_ROOT", os.path.expanduser("~/dev/px4-autopilot-harmonic")))
    gz_res = (px4_root / "Tools" / "simulation" / "gz").as_posix()
    env["GZ_SIM_RESOURCE_PATH"] = f"{env.get('GZ_SIM_RESOURCE_PATH','')}:{gz_res}" if env.get("GZ_SIM_RESOURCE_PATH") else gz_res

    if args.headless:
        # Kill any existing GUI client so it doesn't keep popping up
        subprocess.call(["pkill","-f","gz sim .* -g"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(["pkill","-f","gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Harden headless: server-only, no GUI; also force offscreen just in case
        env["QT_QPA_PLATFORM"] = "offscreen"
        cmd = ["gz","sim","-s","-r", world_path]   # -s server only, -r starts paused=false
        mode = "headless=True"
    else:
        # GUI client (normal)
        cmd = ["gz","sim", world_path]
        mode = "headless=False"

    log.info("Launching Gazebo with world '%s' (%s)…", Path(world_path).name, mode)
    proc = subprocess.Popen(cmd, env=env)
    log.info("Gazebo started (PID %s).  Press Ctrl+C to stop.", proc.pid)

    try:
        proc.wait()
    except KeyboardInterrupt:
        log.info("Interrupted; terminating Gazebo…")
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()

if __name__ == "__main__":
    main()
