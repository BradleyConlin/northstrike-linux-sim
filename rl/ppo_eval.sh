#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

GUI=${GUI:-true}
EPISODES=${EPISODES:-3}

if [[ "${GUI}" == "true" ]]; then
  HEADLESS=false QGC=true ./scripts/ns-run.sh &
else
  HEADLESS=true QGC=false ./scripts/ns-run.sh &
fi

sleep 8
source .venv/bin/activate
PYTHONPATH="$PWD" python -m rl.eval_ppo --episodes "${EPISODES}"

echo "[eval] Done. Stop the sim with: ./scripts/ns-stop.sh"
