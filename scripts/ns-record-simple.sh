#!/usr/bin/env bash
set -euo pipefail

DURATION=${DURATION:-20}          # seconds
CACHE_MB=${CACHE_MB:-128}         # bigger cache = fewer drops
OUT_DIR=${OUT_DIR:-$HOME/dev/northstrike-linux-sim/runs}

# ROS env
set +u; source /opt/ros/humble/setup.bash; set -u

OUT="$OUT_DIR/sim_$(date +%F_%H%M%S)"
echo "[ns-record] -> $OUT  duration=${DURATION}s  cache=${CACHE_MB}MB  (no /world clock)"

# Foreground, show rosbag logs, clean stop; compressed files
timeout --foreground --signal=INT --kill-after=5s "${DURATION}s" \
  ros2 bag record \
    --max-cache-size $((CACHE_MB*1024*1024)) \
    --compression-mode file --compression-format zstd \
    -o "$OUT" \
    /ns_rgb/image /ns_depth/depth_image /ns_lidar /ns/imu /ns/mag /ns/navsat

# Summary
if [ -d "$OUT" ]; then
  ros2 bag info "$OUT"
else
  echo "[ns-record] ERROR: bag directory not created: $OUT"
fi
