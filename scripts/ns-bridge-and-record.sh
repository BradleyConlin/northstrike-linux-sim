#!/usr/bin/env bash
set -Eeuo pipefail

REPO=${REPO:-$HOME/dev/northstrike-linux-sim}
WORLD=${WORLD:-$REPO/simulation/worlds/landing_pad.world.sdf}
DURATION=${DURATION:-30}   # seconds to record
OUT_DIR=${OUT_DIR:-"$REPO/runs/sim_$(date +%F_%H%M)"}

# --- Source environments (nounset-safe) ---
set +u
source /opt/ros/humble/setup.bash
set -u
source "$REPO/scripts/ns-sim-env.sh"

# --- Start Gazebo world (background) ---
gz sim -r "$WORLD" & GZ_PID=$!
sleep 2

# --- Discover camera topic + Gazebo type ---
TOPIC=$(gz topic -l | awk '/\/sensor\/rgb_cam\/image$/ {print $1; exit}')
if [[ -z "${TOPIC:-}" ]]; then
  echo "[ns-bridge-and-record] ERROR: camera topic not found. Is ns_rgb_cam included and Sensors plugin loaded?"
  kill $GZ_PID 2>/dev/null || true
  exit 1
fi
GZTYPE=$(gz topic -i -t "$TOPIC" | awk -F': ' '/Type:/ {print $2}')
: "${GZTYPE:=ignition.msgs.Image}"

echo "[ns-bridge-and-record] Camera topic: $TOPIC  (GZ type: $GZTYPE)"

# --- Start bridges (background) ---
ros2 run ros_gz_bridge parameter_bridge \
  "/world/landing_test/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock" \
  "$TOPIC@sensor_msgs/msg/Image@$GZTYPE" & BRIDGE_PID=$!

# --- Start recorder (explicit topics) ---
NS_TOPICS="/world/landing_test/clock $TOPIC" "$REPO/scripts/ns-record.sh" "$OUT_DIR" "profile:auto" & REC_PID=$!

echo "[ns-bridge-and-record] Recording for ${DURATION}s… (bag: $OUT_DIR)"
sleep "$DURATION" || true

# --- Stop everything cleanly ---
echo "[ns-bridge-and-record] Stopping recorder…"
kill -INT "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true

echo "[ns-bridge-and-record] Stopping bridges and Gazebo…"
kill "$BRIDGE_PID" 2>/dev/null || true
kill "$GZ_PID" 2>/dev/null || true

echo "[ns-bridge-and-record] Done. Bag at: $OUT_DIR"
ros2 bag info "$OUT_DIR"
