#!/usr/bin/env python3
"""Launch QGroundControl for native Linux simulations.

This script finds and launches the QGroundControl AppImage on a native Linux
system.  It respects a headless flag and optional path override.  Use this
only for debugging; QGroundControl is not required for headless training.
"""

import argparse
import logging
import os
import subprocess
import sys
from pathlib import Path

# Default search locations for the AppImage
DEFAULT_QGC_PATHS = [
    os.path.expanduser("~/QGroundControl.AppImage"),
    os.path.expanduser("~/Downloads/QGroundControl.AppImage"),
]


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch QGroundControl on native Linux.")
    parser.add_argument("-q", "--qgc", action="store_true",
                        help="Enable launching of QGroundControl.  If not provided, nothing happens.")
    parser.add_argument("-H", "--headless", action="store_true",
                        help="Suppress QGroundControl when running headless.")
    parser.add_argument("-p", "--qgc-path", default=None,
                        help="Path to the QGroundControl AppImage (overrides default search).")
    return parser.parse_args(argv)


def find_qgc(path_hint: str | None) -> Path | None:
    if path_hint:
        p = Path(path_hint)
        if p.is_file():
            return p
    for candidate in DEFAULT_QGC_PATHS:
        p = Path(candidate)
        if p.is_file():
            return p
    return None


def launch_qgc(app: Path) -> subprocess.Popen:
    os.chmod(app, 0o755)
    logging.info("Launching QGroundControl from %s", app)
    return subprocess.Popen([str(app)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main(argv=None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    if args.headless:
        logging.info("Headless mode; QGroundControl will not be launched.")
        return
    if not args.qgc:
        logging.info("--qgc not set; QGroundControl skipped.")
        return
    app = find_qgc(args.qgc_path)
    if app is None:
        logging.error("QGroundControl AppImage not found.  Specify --qgc-path or place it in ~/Downloads.")
        sys.exit(1)
    proc = None
    try:
        proc = launch_qgc(app)
        logging.info("QGroundControl running (PID %d).  Press Ctrl+C to exit.", proc.pid)
        proc.wait()
    except KeyboardInterrupt:
        logging.info("Ctrl+C pressed; terminating QGroundControl…")
        if proc:
            proc.terminate()
            proc.wait()
    except Exception as exc:
        logging.exception("Failed to run QGroundControl: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
