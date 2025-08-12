#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# ---- env gating ----
if [ "${HEADLESS:-false}" = "true" ]; then
  export PX4_GZ_HEADLESS=1
  export QGC=false   # never start QGC in headless
else
  export PX4_GZ_HEADLESS=0
fi

# ---- clean slate (belt & suspenders) ----
./stop_all.sh || true
python3 clean_env_linux.py || true
pkill -f "gz sim -g" || true
pkill -f QGroundControl.AppImage || pkill -x QGroundControl || true
pkill -f "/px4_sitl_default/bin/px4" || true
sleep 0.5

# ---- launch px4+gz via run_all.sh (pass env explicitly) ----
QGC="${QGC:-false}" \
HEADLESS="${HEADLESS:-false}" \
DRONES="${DRONES:-1}" \
MODEL="${MODEL:-x500}" \
DETACH=true \
./run_all.sh &

# done (run_all.sh manages starting the GZ GUI if HEADLESS!=true)
exit 0
