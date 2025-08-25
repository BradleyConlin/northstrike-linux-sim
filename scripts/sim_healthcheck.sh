#!/usr/bin/env bash
set -euo pipefail

# ROS env
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "ROS 2 Humble not found at /opt/ros/humble" >&2
  exit 1
fi
# Project env (ROS_DOMAIN_ID etc.)
if [ -f .ros_env ]; then
  set -a; source .ros_env; set +a
fi

echo "== northstrike sim healthcheck =="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>}"

echo "[1/4] Checking MAVROS node..."
if ! ros2 node list | grep -E -q '^/(mavros|mavros_node)$'; then
  echo "MAVROS node not found. Is it running?"
  echo "Try:  ros2 run mavros mavros_node --ros-args -p fcu_url:=udpin://:14540@" >&2
  exit 1
fi
echo " OK"

echo "[2/4] Waiting for /mavros/state..."
STATEFILE="$(mktemp)"
if ! timeout 10s bash -lc "ros2 topic echo --once /mavros/state >'$STATEFILE'"; then
  echo "Timed out waiting for /mavros/state. Is PX4 SITL running and Gazebo unpaused?" >&2
  rm -f "$STATEFILE"
  exit 1
fi
cat "$STATEFILE" | sed -n 's/^mode: /mode: /p;s/^connected: /connected: /p;s/^armed: /armed: /p' || true
if ! grep -q 'connected: true' "$STATEFILE"; then
  echo "MAVROS up, but not connected to PX4 yet (no HEARTBEAT). Give it a few seconds." >&2
  rm -f "$STATEFILE"
  exit 2
fi
rm -f "$STATEFILE"
echo " OK"

echo "[3/4] Sampling /mavros/local_position/pose rate for 5s (EKF ready?)"
if ! timeout 5s ros2 topic hz /mavros/local_position/pose; then
  echo "No /mavros/local_position/pose yet — EKF may still be initializing." >&2
  exit 3
fi

echo "[4/4] Topics snapshot:"
ros2 topic list | grep '^/mavros/' | head -n 20

echo "Health OK ✅"
