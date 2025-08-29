#!/usr/bin/env bash
# Full demo: bag -> depth ONNX -> LZ ONNX -> depth gate (+ optional viewers)
# Usage (examples):
#   BAG=... ONNX_DEPTH=... ONNX_LZ=... VIEW=1 bash scripts/ns_run_full_demo.sh
#   VIEW=0 bash scripts/ns_run_full_demo.sh     # run headless (no viewers)

set -eo pipefail

# --- Config (env overrides ok) ------------------------------------------------
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${BAG:-$ROOT/runs/sim_2025-08-22_122259}"
ONNX_DEPTH="${ONNX_DEPTH:-$ROOT/models/exported/_tinyunet_depth_default.onnx}"
# fallback if default symlink isn’t present
[[ -f "$ONNX_DEPTH" ]] || ONNX_DEPTH="$ROOT/models/exported/_tinyunet_depth_e24.onnx"
ONNX_LZ="${ONNX_LZ:-$ROOT/models/exported/_lz_unet_e20.onnx}"
READAHEAD="${READAHEAD:-20000}"
RATE="${RATE:-0.8}"
MAX_DEPTH_M="${MAX_DEPTH_M:-20.0}"
VIEW="${VIEW:-1}"   # 1 = open viewers, 0 = headless

echo "[demo] BAG=$BAG"
echo "[demo] DEPTH=$ONNX_DEPTH"
echo "[demo] LZ=$ONNX_LZ"

# --- ROS environment ----------------------------------------------------------
# (avoid -u here; some ROS setup uses unset vars)
set +u
source /opt/ros/humble/setup.bash
set +u   # keep nounset off intentionally for ROS

# Optional venv for python ROS nodes
if [[ -d "$ROOT/.venv_rosnode" ]]; then
  # shellcheck disable=SC1091
  . "$ROOT/.venv_rosnode/bin/activate"
fi

# --- Cleanup handling ---------------------------------------------------------
PIDS=()
cleanup() {
  echo "[demo] stopping..."
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
  # gentle grace; then force if needed
  sleep 1
  for p in "${PIDS[@]}"; do kill -9  "$p" 2>/dev/null || true; done
}
trap cleanup INT TERM EXIT

# --- Launch ------------------------------------------------------------------
# 1) rosbag player (RGB only; big buffer; gentle rate; loop)
ros2 bag play "$BAG" \
  --topics /ns_rgb/image \
  --read-ahead-queue-size "$READAHEAD" \
  --rate "$RATE" \
  --loop &
PIDS+=($!)

# 2) LZ ONNX node (publishes /ns_lz/mask and /ns_lz/overlay)
python "$ROOT/scripts/ns_lz_onnx_node.py" \
  --model "$ONNX_LZ" &
PIDS+=($!)

# 3) Depth ONNX node (publishes /ns_depth/pred and /ns_depth/pred_color)
python "$ROOT/scripts/ns_depth_onnx_node.py" \
  --model "$ONNX_DEPTH" &
PIDS+=($!)

# 4) Gate node: masks depth with LZ (publishes /ns_depth/pred_gated and *_color)
python "$ROOT/scripts/ns_depth_gate_lz.py" \
  --max_depth_m "$MAX_DEPTH_M" &
PIDS+=($!)

echo "# add Image View panels for:"
echo "#   - /ns_depth/pred_gated_color"
echo "#   - /ns_lz/overlay"
echo "[demo] up. view with:  rqt_image_view /ns_depth/pred_gated_color   or   /ns_lz/overlay"

# --- Optional auto-viewers (OpenCV image_view) --------------------------------
if [[ "$VIEW" == "1" ]]; then
  # tiny delay so topics exist
  sleep 1
  ros2 run image_view image_view --ros-args \
    -p autosize:=true \
    -r image:=/ns_depth/pred_gated_color &
  PIDS+=($!)
  ros2 run image_view image_view --ros-args \
    -p autosize:=true \
    -r image:=/ns_lz/overlay &
  PIDS+=($!)
  echo "[demo] viewers up: /ns_depth/pred_gated_color and /ns_lz/overlay"
fi

# keep script alive until any child exits; traps handle cleanup
wait -n
