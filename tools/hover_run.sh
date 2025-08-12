# ~/dev/northstrike-linux-sim/tools/hover_run.sh
#!/usr/bin/env bash
set -euo pipefail

REPO="$HOME/dev/northstrike-linux-sim"
SCRIPTS="$REPO/scripts"
VENV="$REPO/.venv"

# 1) Launch sim WITH GUI (no aliases; call script directly)
export HEADLESS=false
export QGC=true
"$SCRIPTS/ns-run.sh" &

# 2) Wait until PX4 announces it's ready to takeoff (tail PX4 console)
echo "[hover_run] Waiting for PX4 'Ready for takeoff!'…"
# Fall back to fixed sleep if the string isn't visible in this shell
timeout 45 bash -lc '
  until grep -q "Ready for takeoff!" <(tail -n +1 -F '"$SCRIPTS"'/../px4-autopilot-harmonic/build/px4_sitl_default/px4-*.log 2>/dev/null || true); do
    sleep 1
  done
' || sleep 10

# 3) Activate venv and run hover
source "$VENV/bin/activate"
python "$REPO/tools/hover_check.py"
echo "[hover_run] Hover sequence complete."
