#!/usr/bin/env python3
"""
Launch PX4 SITL instances for Gazebo Harmonic on native Linux.

This script spawns one or more PX4 SITL processes configured for the
Gazebo Harmonic (Ignition) simulator. It sets the environment variables
required to spawn the `x500` drone model at specified positions and
assigns unique UDP ports to each instance for offboard and ground control
communication.

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

# ----- Defaults -----
DEFAULT_PX4_ROOT = os.path.expanduser("~/dev/px4-autopilot-harmonic")
DEFAULT_MODEL = "x500"          # Gazebo Harmonic model
DEFAULT_AUTOSTART_ID = 4001     # x500 airframe

# ----- Builder -----
def build_px4_command(
    px4_root: Path,
    instance: int,
    autostart_id: int,
    model: str,
    pose: str,
    offboard_port: int,
    gcs_port: int,
) -> tuple[list[str], dict]:
    """Prepare the command and environment for a PX4 instance."""
    px4_binary = px4_root / "build" / "px4_sitl_default" / "bin" / "px4"
    rootfs_dir = px4_root / "build" / "px4_sitl_default"           # IMPORTANT: PX4 rootfs
    rcS = rootfs_dir / "etc" / "init.d-posix" / "rcS"

    if not px4_binary.exists():
        raise FileNotFoundError(f"PX4 binary not found at {px4_binary}. Build with 'make px4_sitl_default'.")
    if not rcS.exists():
        raise FileNotFoundError(f"Startup script not found at {rcS}. Rebuild px4_sitl_default to generate rootfs.")

    # Environment
    env = os.environ.copy()
    env.update({
        "PX4_SYS_AUTOSTART":            str(autostart_id),

        # Gazebo Harmonic spawn controls
        "PX4_GZ_MODEL":                 model,                      # e.g., "x500"
        "PX4_GZ_MODEL_NAME":            f"{model}_{instance}",      # unique per instance
        "PX4_GZ_MODEL_POSE":            pose,                       # "x,y,z,roll,pitch,yaw"
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

# ----- Spawner -----
def launch_instances(
    num_drones: int,
    offset: float,
    autostart_id: int,
    model: str,
    px4_root: str,
) -> list[subprocess.Popen]:
    """Spawn multiple PX4 SITL instances on native Linux."""
    processes: list[subprocess.Popen] = []
    logs_dir = Path("logs")
    logs_dir.mkdir(parents=True, exist_ok=True)
    px4_root_path = Path(px4_root)

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
        logging.info(
            "Spawning PX4 #%d at %s (offboard %d, GCS %d)",
            instance, pose, offboard_port, gcs_port,
        )
        logfile = open(log_path, "w")
        proc = subprocess.Popen(cmd, env=env, stdout=logfile, stderr=logfile)
        processes.append(proc)
        time.sleep(1.0)  # small stagger to avoid race on startup

    return processes

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
    return p.parse_args(argv)

# ----- Main -----
def main(argv=None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    logging.info(
        "Launching PX4 on Linux: %d drones, model=%s, autostart=%d",
        args.num_drones, args.model, args.autostart_id
    )
    try:
        procs = launch_instances(
            args.num_drones, args.offset, args.autostart_id, args.model, args.px4_root
        )
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
