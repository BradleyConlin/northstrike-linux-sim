#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

./ns-stop.sh || true
unset QT_QPA_PLATFORM

DRONES="${DRONES:-1}"
MODEL="${MODEL:-x500}"
WORLD="${WORLD:-default}"
HEADLESS="${HEADLESS:-false}"
QGC_WANT="${QGC:-true}"

# Start PX4 + Gazebo SERVER via run_all.sh with QGC disabled to prevent duplicates
QGC=false HEADLESS="${HEADLESS}" DRONES="${DRONES}" MODEL="${MODEL}" DETACH=true ./run_all.sh &

# Attach Gazebo GUI if not headless
if [ "${HEADLESS}" != "true" ]; then
  ./ns-gui.sh
fi

# Start QGC exactly once if requested
if [ "${QGC_WANT}" = "true" ] && ! pgrep -f QGroundControl.AppImage >/dev/null; then
  QT_QPA_PLATFORM=xcb nohup ~/Applications/QGroundControl.AppImage >/dev/null 2>&1 &
fi

exit 0
