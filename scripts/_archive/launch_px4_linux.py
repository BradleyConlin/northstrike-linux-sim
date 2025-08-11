#!/usr/bin/env python3
"""
Launch PX4 SITL instances for Gazebo Harmonic on native Linux (Gazebo Harmonic / Ignition).

What this does
- Spawns N PX4 SITL instances configured for the Gazebo Harmonic sim
- Injects x500 models at given poses (no QGC required)
- Waits until each instance reports "Startup script completed" in its log (90s timeout)
- By default, BLOCKS and waits so you can Ctrl+C to stop cleanly
- Pass --detach to start, verify readiness, and return immediately (for batch/CI)

Prereqs (in your PX4 repo):
  git submodule update --init --recursive
  DONT_RUN=1 make px4_sitl_default
"""

import argparse
import logging
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import List, Tuple

# ----- Defaults -----
DEFAULT_PX4_ROOT = os.path.expanduser("~/dev/px4-autopilot-harmonic")
DEFAULT_MODEL = "x500"          # Gazebo Harmonic model
DEFAULT_AUTOSTART_ID = 4001     # x500 airframe
READY_PATTERNS = [
    "Startup script returned successfully",
    "Startup script completed",
    "logger started (mode=",
    "MAVLink only on localhost",
]
READY_TIMEOUT_SEC = 90

# ----- Builder -----
def build_px4_command(
    px4_root: Path,
    instance: int,
    autostart_id: int,
    model: str,
    pose: str,
    offboard_port: int,
    gcs_port: int,
) -> Tuple[List[str], dict]:
    """Prepare the command and environment for a PX4 instance."""
    px4_binary = px4_root / "build" / "px4_sitl_default" / "bin" / "px4"
    rootfs_dir = px4_root / "build" / "px4_sitl_default"  # IMPORTANT: PX4 rootfs (contains etc/)
    rcS = rootfs_dir / "etc" / "init.d-posix" / "rcS"

    if not px4_binary.exists():
        raise FileNotFoundError(f"PX4 binary not found at {px4_binary}. Build with 'make px4_sitl_default'.")
    if not rcS.exists():
        raise FileNotFoundError(f"Startup script not found at {rcS}. Rebuild px4_sitl_default to generate rootfs.")

    env = os.environ.copy()
    env.update({
        # PX4 config
        "PX4_SYS_AUTOSTART":            str(autostart_id),

        # Gazebo Harmonic spawn controls (do NOT set PX4_GZ_MODEL_NAME → allows spawn)
        "PX4_GZ_MODEL":                 model,      # e.g., "x500"
        "PX4_GZ_MODEL_POSE":            pose,       # "x,y,z,roll,pitch,yaw"
        "PX4_GZ_WORLD":                 "default",

        # Per-instance UDP ports
        "PX4_SITL_UDP_OFFBOARD_PORT":   str(offboard_port),
        "PX4_SITL_UDP_GCS_PORT":        str(gcs_port),
    })

    # Ensure Gazebo can find PX4 worlds/models
    gz_dir = (px4_root / "Tools" / "simulation" / "gz").as_posix()
    env["GZ_SIM_RESOURCE_PATH"] = (
        f"{env.get('GZ_SIM_RESOURCE_PATH','')}:{gz_dir}"
        if env.get("GZ_SIM_RESOURCE_PATH") else gz_dir
    )

    # px4 CLI:
    #   px4 [-i id] [-d] [-s startup] [-w workdir] <rootfs>
    cmd = [
        str(px4_binary),
        "-i", str(instance),
        "-d",
        "-s", "etc/init.d-posix/rcS",   # relative to <rootfs>
        "-w", str(rootfs_dir),          # working directory = build rootfs
        str(rootfs_dir),                # <rootfs> points to build/px4_sitl_default
    ]
    return cmd, env

# ----- Readiness helpers -----
def wait_for_ready(log_path: Path, patterns: list[str], timeout_sec: int) -> bool:
    """Poll a log file until any pattern appears or timeout elapses."""
    deadline = time.time() + timeout_sec
    last_size = -1
    while not log_path.exists() and time.time() < deadline:
        time.sleep(0.2)

    while time.time() < deadline:
        try:
            size = log_path.stat().st_size if log_path.exists() else 0
            if size != last_size:
                last_size = size
                with log_path.open("r", errors="ignore") as f:
                    content = f.read()
                    for p in patterns:
                        if p in content:
                            return True
        except Exception:
            pass
        time.sleep(0.3)
    return False

def ensure_all_ready(log_paths: list[Path], timeout_sec: int) -> bool:
    """Wait until all PX4 instances report ready; returns True if all succeeded."""
    ok = True
    for idx, log_path in enumerate(log_paths):
        logging.info("Waiting for PX4 #%d to become ready (timeout=%ss)…", idx, timeout_sec)
        if wait_for_ready(log_path, READY_PATTERNS, timeout_sec):
            logging.info("PX4 #%d ready (readiness phrase matched)", idx)
        else:
            logging.error("PX4 #%d did NOT become ready within %ss; see %s", idx, timeout_sec, log_path)
            ok = False
    return ok

# ----- Spawner -----
def launch_instances(
    num_drones: int,
    offset: float,
    autostart_id: int,
    model: str,
    px4_root: str,
) -> Tuple[List[subprocess.Popen], List[Path]]:
    """Spawn multiple PX4 SITL instances on native Linux and return processes + their log paths."""
    processes: List[subprocess.Popen] = []
    logs_dir = Path("logs")
    logs_dir.mkdir(parents=True, exist_ok=True)
    px4_root_path = Path(px4_root)

    log_paths: List[Path] = []

    for instance in range(num_drones):
        # Simple line formation with X-offset
        x = instance * offset
        y = 0.0
        z = 1.0
        pose = f"{x},{y},{z},0,0,0"

        offboard_port = 14540 + instance
        gcs_port = 14550 + instance

        cmd, env = build_px4_command(
            px4_root=px4_root_path,
            instance=instance,
            autostart_id=autostart_id,
            model=model,
            pose=pose,
            offboard_port=offboard_port,
            gcs_port=gcs_port,
        )

        log_path = logs_dir / f"drone_{instance}.txt"
        log_paths.append(log_path)

        logging.info(
            "Spawning PX4 #%d at %s (offboard %d, GCS %d)",
            instance, pose, offboard_port, gcs_port,
        )
        logfile = log_path.open("w")
        proc = subprocess.Popen(cmd, env=env, stdout=logfile, stderr=logfile)
        processes.append(proc)
        time.sleep(1.0)  # small stagger to avoid race on startup

    return processes, log_paths

def ensure_all_ready(log_paths: List[Path], timeout_sec: int) -> bool:
    """Wait until all PX4 instances report ready in their logs; returns True if all succeeded."""
    ok = True
    for idx, log_path in enumerate(log_paths):
        logging.info("Waiting for PX4 #%d to become ready (timeout=%ss)…", idx, timeout_sec)
        if wait_for_ready(log_path, READY_PATTERNS, timeout_sec):
            logging.info("PX4 #%d ready (readiness phrase matched)", idx)
        else:
            logging.error("PX4 #%d did NOT become ready within %ss; see %s", idx, timeout_sec, log_path)
            ok = False
    return ok

# ----- CLI -----
def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Launch PX4 SITL for Gazebo Harmonic on native Linux."
    )

    p.add_argument("-n", "--num-drones", type=int, default=1,
                   help="Number of PX4 instances to launch (default: 1)")

    p.add_argument("-o", "--offset", type=float, default=2.0,
                   help="Distance in metres between successive drones (default: 2.0)")

    p.add_argument("-a", "--autostart-id", type=int, default=DEFAULT_AUTOSTART_ID,
                   help="PX4 airframe autostart ID (default: 4001)")

    p.add_argument("-m", "--model", default=DEFAULT_MODEL,
                   help="Gazebo model name to spawn (default: x500)")

    p.add_argument("-p", "--px4-root", default=DEFAULT_PX4_ROOT,
                   help=f"PX4 root directory (default: {DEFAULT_PX4_ROOT})")

    p.add_argument("--detach", action="store_true",
                   help="Start PX4 processes, verify readiness, then exit immediately")

    p.add_argument("--ready-timeout", type=int, default=READY_TIMEOUT_SEC,
                   help=f"Seconds to wait for PX4 readiness phrases "
                        f"({', '.join(READY_PATTERNS)}) in each log "
                        f"(default: {READY_TIMEOUT_SEC})")

    return p.parse_args(argv)

# ----- Main -----
def main(argv=None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    logging.info(
        "Launching PX4 on Linux: %d drones, model=%s, autostart=%d",
        args.num_drones, args.model, args.autostart_id
    )
    procs: List[subprocess.Popen] = []
    try:
        procs, log_paths = launch_instances(
            args.num_drones, args.offset, args.autostart_id, args.model, args.px4_root
        )

        # Wait for readiness for each instance (timeout per drone)
        if not ensure_all_ready(log_paths, args.ready_timeout):
            # If any failed, stop everything and exit non-zero
            logging.error("One or more PX4 instances failed to become ready. Terminating…")
            for p in procs:
                try: p.terminate()
                except Exception: pass
            for p in procs:
                try: p.wait(timeout=3)
                except Exception: pass
            sys.exit(2)

        logging.info("All %d PX4 instances are ready.", args.num_drones)

        if args.detach:
            pids = [p.pid for p in procs if p and p.poll() is None]
            logging.info("PX4 started (detached). PIDs: %s", pids)
            logging.info("Hint: use scripts/clean_env_linux.py to stop them.")
            return

        logging.info("PX4 instances running. Press Ctrl+C to terminate.")
        for proc in procs:
            proc.wait()

    except KeyboardInterrupt:
        logging.info("Ctrl+C pressed; terminating PX4 instances…")
        try:
            for proc in procs:
                proc.terminate()
            for proc in procs:
                proc.wait(timeout=3)
        except Exception:
            pass
    except Exception as exc:
        logging.exception("Error launching PX4: %s", exc)
        sys.exit(1)

if __name__ == "__main__":
    main()
