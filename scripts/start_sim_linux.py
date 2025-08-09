#!/usr/bin/env python3
"""Orchestrate a native Linux multi‑drone simulation using Gazebo Harmonic and PX4.

This script starts Gazebo with a user‑specified world, then spawns
multiple PX4 SITL instances and optionally launches QGroundControl for
monitoring.  It is tailored for full Linux environments (not WSL2) and
assumes PX4 has been built and its models have been fetched via
``git submodule update --init --recursive``.

Usage example:

    # Start two drones with default world and QGroundControl
    python3 start_sim_linux.py -n 2 -qgc true

    # Start four drones separated by 3 m, headless
    python3 start_sim_linux.py -n 4 -o 3 --headless true
"""

import argparse
import importlib
import logging
import sys
import time
from pathlib import Path


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Start a multi‑drone PX4/Gazebo simulation on native Linux.")
    parser.add_argument("-n", "--num-drones", type=int, default=1,
                        help="Number of drones to launch (default: 1)")
    parser.add_argument("-o", "--offset", type=float, default=2.0,
                        help="Position offset between drones in metres (default: 2.0)")
    default_world = str(Path.home() / "dev/northstrike-linux-sim/worlds/empty_world.sdf") # Added August 8
    parser.add_argument("--world", default=default_world,
                    help=f"Path to the SDF world file (default: {default_world})")
    parser.add_argument("--qgc", choices=["true", "false"], default="false",
                        help="Launch QGroundControl GUI (true/false).  Default: false")
    parser.add_argument("--headless", choices=["true", "false"], default="false",
                        help="Run Gazebo without GUI and suppress QGroundControl (true/false).  Default: false")
    return parser.parse_args(argv)


def to_bool(value: str) -> bool:
    return value.lower() == "true"


def main(argv=None) -> None:
    args = parse_args(argv)
    num_drones = args.num_drones
    offset = args.offset
    world = args.world
    qgc_flag = to_bool(args.qgc)
    headless = to_bool(args.headless)

    # Configure logging
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    logging.info("Starting native Linux simulation: drones=%d, offset=%.2f, headless=%s, qgc=%s",
                 num_drones, offset, headless, qgc_flag)

    # Prepare module import; add scripts directory to sys.path
    scripts_dir = Path(__file__).resolve().parent
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    try:
        launch_gazebo = importlib.import_module("launch_gazebo_world_linux")
        launch_px4 = importlib.import_module("launch_px4_linux")
        launch_qgc = importlib.import_module("launch_qgroundcontrol_linux")
    except Exception as exc:
        logging.exception("Failed to import helper modules: %s", exc)
        sys.exit(1)

    gazebo_proc = None
    px4_procs = []
    qgc_proc = None

    try:
        # Launch Gazebo first
        gazebo_proc = launch_gazebo.launch_gazebo(world, headless)
        logging.info("Gazebo launched (PID %d). Waiting for initialization…", gazebo_proc.pid)
        time.sleep(2.0)
        # Spawn PX4 drones
        px4_procs = launch_px4.launch_instances(num_drones, offset,
                                               launch_px4.DEFAULT_AUTOSTART_ID,
                                               launch_px4.DEFAULT_MODEL,
                                               launch_px4.DEFAULT_PX4_ROOT)
        logging.info("Spawned %d PX4 instances.", len(px4_procs))
        # Start QGroundControl if requested and not headless
        if qgc_flag and not headless:
            app = launch_qgc.find_qgc(None)
            if app is None:
                logging.warning("QGroundControl requested but not found. Skipping.")
            else:
                qgc_proc = launch_qgc.launch_qgc(app)
                logging.info("QGroundControl launched (PID %d).", qgc_proc.pid)
        elif qgc_flag and headless:
            logging.info("Headless mode prevents QGroundControl from launching.")
        # Keep running until Ctrl+C
        logging.info("Simulation running. Press Ctrl+C to terminate.")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logging.info("Ctrl+C detected; cleaning up…")
    except Exception as exc:
        logging.exception("Simulation error: %s", exc)
    finally:
        # Terminate QGroundControl
        if qgc_proc and qgc_proc.poll() is None:
            logging.info("Stopping QGroundControl…")
            qgc_proc.terminate()
        # Terminate PX4 instances
        for proc in px4_procs:
            if proc.poll() is None:
                proc.terminate()
        # Terminate Gazebo
        if gazebo_proc and gazebo_proc.poll() is None:
            logging.info("Stopping Gazebo…")
            gazebo_proc.terminate()
        # Wait for processes to exit
        for proc in px4_procs:
            try:
                proc.wait(timeout=5.0)
            except Exception:
                pass
        if qgc_proc:
            try:
                qgc_proc.wait(timeout=5.0)
            except Exception:
                pass
        if gazebo_proc:
            try:
                gazebo_proc.wait(timeout=5.0)
            except Exception:
                pass
        logging.info("All processes terminated.")


if __name__ == "__main__":
    main()
