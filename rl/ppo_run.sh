#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

GUI=${GUI:-false}          # set GUI=true to watch
STEPS=${STEPS:-20000}

if [[ "${GUI}" == "true" ]]; then
  echo "[ppo_run] Starting sim with GUI…"
  HEADLESS=false QGC=true ./scripts/ns-run.sh &
else
  echo "[ppo_run] Starting sim headless…"
  HEADLESS=true QGC=false ./scripts/ns-run.sh &
fi

SIM_PID=$!
echo "[ppo_run] Sim PID: $SIM_PID"
# Give PX4 time to boot; trainer will still wait for readiness on its own
sleep 8

# venv
source .venv/bin/activate
python -m pip show stable-baselines3 >/dev/null || { echo "stable-baselines3 not in venv"; exit 1; }

PYTHONPATH="$PWD" python -m rl.train_ppo --steps "${STEPS}"

echo "[ppo_run] Training finished. You can stop the sim with:"
echo "  ./scripts/ns-stop.sh"
