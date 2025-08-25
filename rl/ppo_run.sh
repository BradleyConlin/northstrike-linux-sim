#!/usr/bin/env bash
set -euo pipefail
NUM_ENVS=4
PORTS=""
DEVICE="cuda"
STEPS=12000000
CKPT_EVERY=200000
LOGDIR="runs/ppo_hover_vec4"
CKPTDIR="checkpoints/ppo_hover_vec4"
SEED=42
PX4_REPO="${PX4_REPO:-$HOME/dev/px4-autopilot-harmonic}"
WORLD_SDF="$PX4_REPO/Tools/simulation/gz/worlds/default.sdf"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --num-envs) NUM_ENVS="$2"; shift 2;;
    --ports) PORTS="$2"; shift 2;;
    --device) DEVICE="$2"; shift 2;;
    --steps) STEPS="$2"; shift 2;;
    --ckpt-every) CKPT_EVERY="$2"; shift 2;;
    --logdir) LOGDIR="$2"; shift 2;;
    --checkpoint-dir) CKPTDIR="$2"; shift 2;;
    --seed) SEED="$2"; shift 2;;
    *) echo "Unknown arg: $1" >&2; exit 2;;
  esac
done

IFS=, read -r -a PORT_ARR <<< "${PORTS:-}"
if [[ ${#PORT_ARR[@]} -eq 0 ]]; then
  for ((i=0;i<NUM_ENVS;i++)); do PORT_ARR+=($((14540 + i*10))); done
fi
if [[ ${#PORT_ARR[@]} -ne $NUM_ENVS ]]; then
  echo "Need exactly $NUM_ENVS ports, got ${#PORT_ARR[@]}: ${PORT_ARR[*]}" >&2; exit 3
fi

mkdir -p "$LOGDIR" "$CKPTDIR" logs instances

cleanup(){ pkill -f "$PX4_REPO/build/px4_sitl_default/bin/px4" 2>/dev/null || true; pkill -f "gz sim" 2>/dev/null || true; }
trap cleanup EXIT

# Launch Gazebo headless once
if ! pgrep -fa "gz sim" >/dev/null; then
  gz sim -s -r "$WORLD_SDF" >/dev/null 2>&1 &
  sleep 2
fi

# Launch N PX4 instances (unique workdirs, instance idx offsets ports)
for ((i=0;i<NUM_ENVS;i++)); do
  WORKDIR="$(pwd)/instances/px4_$i"
  mkdir -p "$WORKDIR"
  SITL_INSTANCE=$i PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_NAME="x500_$i" GZ_IP=127.0.0.1 \
  "$PX4_REPO/build/px4_sitl_default/bin/px4" -w "$WORKDIR" -s etc/init.d-posix/rcS \
    > "logs/px4_$i.log" 2>&1 &
  sleep 0.5
done

# Start PPO (SB3) with matching ports
python -m rl.train_ppo \
  --num-envs "$NUM_ENVS" \
  --ports "$(IFS=, ; echo "${PORT_ARR[*]}")" \
  --device "$DEVICE" \
  --steps "$STEPS" \
  --ckpt-every "$CKPT_EVERY" \
  --logdir "$LOGDIR" \
  --checkpoint-dir "$CKPTDIR" \
  --seed "$SEED"
