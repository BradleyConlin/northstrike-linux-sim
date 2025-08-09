#!/usr/bin/env python3
"""Launch a Gazebo Harmonic world on native Linux for drone simulation.

This script wraps the ``gz sim`` command to start Gazebo (Ignition/Harmonic)
with a specified world file.  It can optionally disable the GUI for
headless operation.  Use this as the first step before starting PX4 SITL
instances.

The script does not spawn drone models directly; PX4 will spawn models
when it starts and sets the appropriate ``PX4_SIM_MODEL`` and
``PX4_GZ_MODEL_POSE`` variables.
"""

import argparse
import logging
import os
import subprocess
import sys


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Start Gazebo Harmonic on native Linux.")
    parser.add_argument("world", nargs="?", default=None,
                        help="Path to the SDF world file to load.  If omitted, the default world is used.")
    parser.add_argument("--headless", "-l", action="store_true",
                        help="Run Gazebo in server-only mode (no GUI)")
    return parser.parse_args(argv)


def launch_gazebo(world_path: str | None, headless: bool) -> subprocess.Popen:
    """Launch Gazebo with the given world path.

    Args:
        world_path: Path to the SDF file.  If None, Gazebo uses its default empty world.
        headless: If True, disables the GUI.

    Returns:
        Popen object for the Gazebo process.
    """
    env = os.environ.copy()
    # HEADLESS=1 hints to Gazebo to skip launching the GUI
    if headless:
        env["HEADLESS"] = "1"
    cmd = ["gz", "sim"]
    if world_path:
        cmd.append(world_path)
    cmd.append("-r")
    if headless:
        cmd.append("-s")
    logging.info("Launching Gazebo with world '%s' (headless=%s)…", world_path or "default", headless)
    return subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main(argv=None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    proc = None
    try:
        proc = launch_gazebo(args.world, args.headless)
        logging.info("Gazebo started (PID %d).  Press Ctrl+C to stop.", proc.pid)
        proc.wait()
    except KeyboardInterrupt:
        logging.info("Interrupted; terminating Gazebo…")
        if proc:
            proc.terminate()
            proc.wait()
    except Exception as exc:
        logging.exception("Error launching Gazebo: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
