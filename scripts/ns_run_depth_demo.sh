#!/usr/bin/env bash
set -eE -o pipefail
set +u  # avoid ROS nounset issues

REPO=${REPO:-$HOME/dev/northstrike-linux-sim}
BAG="${BAG:-$REPO/runs/sim_2025-08-22_122259}"
ONNX="${ONNX:-$REPO/datasets/sim_2025-08-22_122259/_tinyunet_depth.onnx}"
RVIZ_CFG="${RVIZ_CFG:-$REPO/rviz/ns_depth_demo.rviz}"

source /opt/ros/humble/setup.bash
source "$REPO/.venv_rosnode/bin/activate"

[ -f "$ONNX" ] || ONNX="$REPO/models/exported/_tinyunet_depth.onnx"
echo "[demo] ONNX: $ONNX"
echo "[demo] BAG:  $BAG"

NODE_PATH="$REPO/scripts/ns_depth_onnx_node.py"
[ -f "$NODE_PATH" ] || NODE_PATH="$REPO/ns_depth_onnx_node.py"

python "$NODE_PATH" \
  --model "$ONNX" \
  --in /ns_rgb/image \
  --out /ns_depth/pred \
  --size 320 240 \
  --max_depth_m 20.0 &

NODE_PID=$!
(if [ -f "$RVIZ_CFG" ]; then rviz2 -d "$RVIZ_CFG"; else rviz2; fi) >/dev/null 2>&1 &
RVIZ_PID=$! || true

ros2 bag play "$BAG" -l
kill $NODE_PID $RVIZ_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true
wait $RVIZ_PID 2>/dev/null || true
