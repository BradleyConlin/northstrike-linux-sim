#!/usr/bin/env bash
set -Eeuo pipefail

REPO=${REPO:-$HOME/dev/northstrike-linux-sim}
WORLD=${WORLD:-$REPO/simulation/worlds/landing_pad.world.sdf}
DURATION=${DURATION:-30}         # seconds to record
CACHE_MB=${CACHE_MB:-64}         # keep memory low
STOP_TIMEOUT=${STOP_TIMEOUT:-40} # seconds to wait for clean shutdown

# --- env (handle ROS's use of unset vars) ---
set +u
source /opt/ros/humble/setup.bash
set -u
source "$REPO/scripts/ns-sim-env.sh"

# --- start Gazebo headless if not already running ---
if ! pgrep -f "gz sim .*landing_pad.world.sdf" >/dev/null; then
  gz sim -s -r "$WORLD" >/dev/null 2>&1 &
  sleep 3
fi

# --- fixed topic names from your SDF ---
CLK="/world/landing_test/clock"
RGB="/ns_rgb/image"
DEPTH="/ns_depth/depth_image"
LIDAR="/ns_lidar"
IMU="/ns/imu"
MAG="/ns/mag"
GPS="/ns/navsat"

# confirm Gazebo is publishing them (warn only)
for t in "$CLK" "$RGB" "$DEPTH" "$LIDAR" "$IMU" "$MAG" "$GPS"; do
  gz topic -l | grep -qx "$t" || echo "[ns-bridge] WARN: missing GZ topic $t"
done

# --- detect GZ types with Humble fallbacks ---
gz_type(){ gz topic -i -t "$1" | awk -F': ' '/Type:/ {print $2}'; }
T_CLK="ignition.msgs.Clock"
T_RGB="$(gz_type "$RGB")";     [ -z "${T_RGB:-}" ]   && T_RGB="ignition.msgs.Image"
T_DEPTH="$(gz_type "$DEPTH")"; [ -z "${T_DEPTH:-}" ] && T_DEPTH="ignition.msgs.Image"
T_LIDAR="$(gz_type "$LIDAR")"; [ -z "${T_LIDAR:-}" ] && T_LIDAR="ignition.msgs.LaserScan"
T_IMU="$(gz_type "$IMU")";     [ -z "${T_IMU:-}" ]   && T_IMU="ignition.msgs.IMU"
T_MAG="$(gz_type "$MAG")";     [ -z "${T_MAG:-}" ]   && T_MAG="ignition.msgs.Magnetometer"
T_GPS="$(gz_type "$GPS")";     [ -z "${T_GPS:-}" ]   && T_GPS="ignition.msgs.NavSat"

echo "[ns-bridge] Bridging:"
printf "  %s -> Image         (%s)\n"        "$RGB"   "$T_RGB"
printf "  %s -> Depth Image   (%s)\n"        "$DEPTH" "$T_DEPTH"
printf "  %s -> LaserScan     (%s)\n"        "$LIDAR" "$T_LIDAR"
printf "  %s -> Imu           (%s)\n"        "$IMU"   "$T_IMU"
printf "  %s -> MagneticField (%s)\n"        "$MAG"   "$T_MAG"
printf "  %s -> NavSatFix     (%s)\n"        "$GPS"   "$T_GPS"
printf "  %s -> Clock         (%s)\n"        "$CLK"   "$T_CLK"

# --- start parameter bridge in background ---
ros2 run ros_gz_bridge parameter_bridge \
  "$CLK@rosgraph_msgs/msg/Clock@$T_CLK" \
  "$RGB@sensor_msgs/msg/Image@$T_RGB" \
  "$DEPTH@sensor_msgs/msg/Image@$T_DEPTH" \
  "$LIDAR@sensor_msgs/msg/LaserScan@$T_LIDAR" \
  "$IMU@sensor_msgs/msg/Imu@$T_IMU" \
  "$MAG@sensor_msgs/msg/MagneticField@$T_MAG" \
  "$GPS@sensor_msgs/msg/NavSatFix@$T_GPS" \
  >/dev/null 2>&1 &
BRIDGE_PID=$!

# --- start ros2 bag record in its own process group ---
OUT="runs/sim_$(date +%F_%H%M)"
mkdir -p "$REPO/${OUT}_meta"
printf "%s\n" "$CLK" "$RGB" "$DEPTH" "$LIDAR" "$IMU" "$MAG" "$GPS" > "$REPO/${OUT}_meta/topics.txt"
printf "start_utc=%s\n" "$(date -u +%FT%TZ)" > "$REPO/${OUT}_meta/run.info"

echo "[ns-bridge] Recording ${DURATION}s -> $OUT"
set -m  # enable job control so we can signal process groups
bash -c "exec ros2 bag record -s sqlite3 --max-cache-size $((CACHE_MB*1024*1024)) -o '$REPO/$OUT' \
  '$CLK' '$RGB' '$DEPTH' '$LIDAR' '$IMU' '$MAG' '$GPS'" &
REC_PGID=$!   # this is the job's group id anchor

# give recorder a moment to create the folder
for i in {1..20}; do
  [ -d "$REPO/$OUT" ] && break
  sleep 0.2
done

# --- wait DURATION, then send SIGINT to the whole recorder group ---
sleep "$DURATION" || true
echo "[ns-bridge] Stopping recorder…"
kill -INT -- -"$REC_PGID" 2>/dev/null || true

# wait up to STOP_TIMEOUT for clean shutdown, then escalate
END_T=$((SECONDS + STOP_TIMEOUT))
while kill -0 "$REC_PGID" 2>/dev/null; do
  [ $SECONDS -ge $END_T ] && { echo "[ns-bridge] forcing stop…"; kill -TERM -- -"$REC_PGID" 2>/dev/null || true; sleep 2; kill -KILL -- -"$REC_PGID" 2>/dev/null || true; break; }
  sleep 1
done

# --- stop bridge and print bag info ---
kill "$BRIDGE_PID" 2>/dev/null || true
echo "[ns-bridge] Done. Bag info:"
if [ -d "$REPO/$OUT" ]; then
  ros2 bag info "$REPO/$OUT"
else
  echo "[ns-bridge] ERROR: bag directory not found: $REPO/$OUT"
fi
