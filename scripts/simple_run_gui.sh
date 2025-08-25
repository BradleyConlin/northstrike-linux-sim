#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

./stop_all.sh || true
python3 clean_env_linux.py || true
unset QT_QPA_PLATFORM

# Start your normal run (blocks), in background
QGC=true HEADLESS=false DRONES=1 MODEL=x500 DETACH=false ./run_all.sh &

# Wait for Gazebo world, then start ONE GUI if none exists
for i in $(seq 1 "${GZ_GUI_WAIT_MAX:-30}"); do
  gz topic -l 2>/dev/null | grep -qE '^/world/.*/state$' && break
  sleep 1
done
pgrep -f "gz sim -g" >/dev/null || gz sim -g > gz_gui.log 2>&1 &
wait
