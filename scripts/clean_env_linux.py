#!/usr/bin/env python3

import os
import signal
import subprocess
import time
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

def cleanup():
    logging.info("Cleaning up running PX4, Gazebo, and QGroundControl processes…")

    # Kill PX4 instances
    subprocess.run(["pkill", "-f", "px4"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Kill Gazebo (gz sim)
    subprocess.run(["pkill", "-f", "gz"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Kill QGroundControl
    subprocess.run(["pkill", "-f", "QGroundControl.AppImage"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Short pause to ensure shutdown
    time.sleep(0.5)

    logging.info("Swarm simulation environment cleaned.")

if __name__ == "__main__":
    cleanup()
