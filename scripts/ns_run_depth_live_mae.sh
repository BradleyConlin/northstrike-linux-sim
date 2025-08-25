#!/usr/bin/env bash
set -eE -o pipefail
set +u

REPO=${REPO:-$HOME/dev/northstrike-linux-sim}
BAG="${BAG:-${1:-}}"

source /opt/ros/humble/setup.bash
source "$REPO/.venv_rosnode/bin/activate"

echo "[live-mae] BAG: ${BAG:-<none>}"

python "$REPO/scripts/ns_live_depth_mae.py" --ros-args -p max_depth_m:=20.0 &
NODE_PID=$!

if [[ -n "${BAG}" ]]; then
  ros2 bag play "$BAG"
  kill $NODE_PID || true
  wait $NODE_PID || true
else
  wait $NODE_PID
fi
