#!/usr/bin/env bash
set -euo pipefail
echo "[stop_all] stopping PX4, Gazebo, QGC, XRCE Agent"
pkill -f "(px4|gz|gzclient|gzserver|QGroundControl|MicroXRCEAgent)" || true
sleep 1
echo "[stop_all] done"
