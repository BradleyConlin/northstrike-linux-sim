#!/usr/bin/env bash
set -eo pipefail
REPO=~/dev/northstrike-linux-sim
OUT=~/dev/northstrike-linux-sim/datasets/sim_2025-08-22_122259

source /opt/ros/humble/setup.bash
source "$REPO/.venv_rosnode/bin/activate"

cat >/tmp/ns_qos.yaml <<'YAML'
/ns_rgb/image: {reliability: reliable, history: keep_last, depth: 10}
YAML

ros2 bag play --loop -r 1.0 --read-ahead-queue-size 2000 \
  --qos-profile-overrides-path /tmp/ns_qos.yaml \
  "$REPO/runs/sim_2025-08-22_122259" &
BAG_PID=$!

python "$REPO/scripts/ns_depth_onnx_node.py" \
  --model "$OUT/_tinyunet_depth.onnx" --size 320 240 --max_depth_m 20.0 &
NODE_PID=$!

rviz2 -d "$REPO/rviz/ns_depth_demo.rviz" &
RVIZ_PID=$!

trap "kill $RVIZ_PID $NODE_PID $BAG_PID 2>/dev/null || true" INT TERM
wait
