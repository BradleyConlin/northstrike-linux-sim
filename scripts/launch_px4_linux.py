#!/usr/bin/env python3
"""Launch PX4 SITL instances for Gazebo Harmonic on native Linux.

This script spawns one or more PX4 SITL processes configured for the
Gazebo Harmonic (Ignition) simulator.  It sets the environment variables
required to spawn the ``gz_x500`` drone model at specified positions and
assigns unique UDP ports to each instance for offboard and ground control
communication.

Before using this script, ensure that you have run ``git submodule update
--init --recursive`` in your PX4 repository so that the ``gz_x500`` model
and associated plugins are available.  Also build PX4 with
``DONT_RUN=1 make px4_sitl_default``.
"""

import argparse
import logging
import os
from pathlib import Path
import subprocess
import sys
import time


# Default location of PX4 in the native Linux environment
DEFAULT_PX4_ROOT = os.path.expanduser("~/dev/px4-autopilot-harmonic")
# Default Gazebo model to spawn (gz_x500 for Harmonic)
DEFAULT_MODEL = "iris"
# Default airframe (x500)
DEFAULT_AUTOSTART_ID = 4001


def build_px4_command(px4_root: Path, instance: int, autostart_id: int, model: str,
                      pose: str, offboard_port: int, gcs_port: int) -> tuple[list[str], dict]:
    """Prepare the command and environment for a PX4 instance.

    Returns a tuple of the command list and environment dict.
    """
    px4_binary = px4_root / "build/px4_sitl_default/bin/px4"
    if not px4_binary.exists():
        raise FileNotFoundError(f"PX4 binary not found at {px4_binary}. Build with 'make px4_sitl_default'.")
    env = os.environ.copy()
    env.update({
        "PX4_SYS_AUTOSTART": str(autostart_id),
        "PX4_SIM_MODEL": model,
        "PX4_GZ_MODEL_POSE": pose,
        # Assign per‑instance ports
        "PX4_SITL_UDP_OFFBOARD_PORT": str(offboard_port),
        "PX4_SITL_UDP_GCS_PORT": str(gcs_port),
    })
    cmd = [
        str(px4_binary),
        "-i", str(instance),
        "-d",
        "-s", "etc/init.d-posix/rcS",
        "-y", str(autostart_id),
    ]
    return cmd, env


def launch_instances(num_drones: int, offset: float, autostart_id: int,
                      model: str, px4_root: str) -> list:
    """Spawn multiple PX4 SITL instances on native Linux.

    Returns a list of subprocess.Popen objects.
    """
    processes = []
    logs_dir = Path("logs")
    logs_dir.mkdir(parents=True, exist_ok=True)
    px4_root_path = Path(px4_root)

    for instance in range(num_drones):
        x = instance * offset
        y = 0.0
        z = 1.0
        pose = f"{x},{y},{z},0,0,0"
        offboard_port = 14540 + instance
        gcs_port = 14550 + instance
        cmd, env = build_px4_command(px4_root_path, instance, autostart_id, model, pose,
                                     offboard_port, gcs_port)
        log_path = logs_dir / f"drone_{instance}.txt"
        logging.info("Spawning PX4 #%d at %s (offboard %d, GCS %d)",
                     instance, pose, offboard_port, gcs_port)
        logfile = open(log_path, "w")
        proc = subprocess.Popen(cmd, env=env, stdout=logfile, stderr=logfile)
        processes.append(proc)
        time.sleep(1.0)
    return processes


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch PX4 SITL for Gazebo Harmonic on native Linux.")
    parser.add_argument("-n", "--num-drones", type=int, default=1,
                        help="Number of PX4 instances to launch (default: 1)")
    parser.add_argument("-o", "--offset", type=float, default=2.0,
                        help="Distance in metres between successive drones (default: 2.0)")
    parser.add_argument("-a", "--autostart-id", type=int, default=DEFAULT_AUTOSTART_ID,
                        help="PX4 airframe autostart ID (default: 4001)")
    parser.add_argument("-m", "--model", default=DEFAULT_MODEL,
                        help="Gazebo model name to spawn (default: gz_x500)")
    parser.add_argument("-p", "--px4-root", default=DEFAULT_PX4_ROOT,
                        help=f"PX4 root directory (default: {DEFAULT_PX4_ROOT})")
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    logging.info("Launching PX4 on Linux: %d drones, model=%s, autostart=%d", args.num_drones, args.model, args.autostart_id)
    try:
        processes = launch_instances(args.num_drones, args.offset, args.autostart_id, args.model, args.px4_root)
        logging.info("PX4 instances running.  Press Ctrl+C to terminate.")
        for proc in processes:
            proc.wait()
    except KeyboardInterrupt:
        logging.info("Ctrl+C pressed; terminating PX4 instances…")
        for proc in processes:
            proc.terminate()
        for proc in processes:
            proc.wait()
    except Exception as exc:
        logging.exception("Error launching PX4: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
