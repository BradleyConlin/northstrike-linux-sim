#!/usr/bin/env bash


set -euo pipefail
QGC=${QGC:-true}
HEADLESS=${HEADLESS:-false}

# ---- args ----
WORLD="default"
QGC_BIN=""
for a in "$@"; do
  case "$a" in
    --world=*) WORLD="${a#--world=}" ;;
    --qgc=*)   QGC_BIN="${a#--qgc=}" ;;
  esac
done

PX4_DIR="$HOME/dev/px4-autopilot-harmonic"
WORLD_FILE="$PX4_DIR/Tools/simulation/gz/worlds/${WORLD}.sdf"

echo "[run_all] world: ${WORLD}"

# ---- QGC ----
start_qgc() {
  if [[ -n "$QGC_BIN" && -x "$QGC_BIN" ]]; then
    echo "[run_all] starting QGroundControl -> $QGC_BIN"
    nohup "$QGC_BIN" >/dev/null 2>&1 & disown
    return
  fi
  for c in \
    "$HOME/Downloads/QGroundControl.AppImage" \
    "$HOME/Applications/QGroundControl.AppImage" \
    "$(command -v qgroundcontrol || true)"; do
    if [[ -n "$c" && -x "$c" ]]; then
      echo "[run_all] starting QGroundControl -> ${c}"
      nohup "${c}" >/dev/null 2>&1 & disown
      return
    fi
  done
  if command -v flatpak >/dev/null 2>&1 && flatpak info org.mavlink.qgroundcontrol >/dev/null 2>&1; then
    echo "[run_all] starting QGroundControl (flatpak)"
    nohup flatpak run org.mavlink.qgroundcontrol >/dev/null 2>&1 & disown
    return
  fi
  echo "[run_all] WARNING: QGroundControl not found; continuing without it."
}

if [ "${QGC}" = "true" ] && [ "${HEADLESS}" != "true" ]; then
  start_qgc
fi

# ---- PX4 + Gazebo ----
if [[ ! -f "${WORLD_FILE}" ]]; then
  echo "[run_all] ERROR: world file not found: ${WORLD_FILE}"
  exit 1
fi

cd "${PX4_DIR}"
echo "[run_all] launching PX4 SITL + Gazebo..."

# --- Gazebo GUI watcher (opens once when the world appears) ---
if [ "${HEADLESS:-false}" != "true" ]; then
  (
    unset QT_QPA_PLATFORM
    MAX=${GZ_GUI_WAIT_MAX:-30}
    for i in $(seq 1 "$MAX"); do
      if gz topic -l 2>/dev/null | grep -qE "^/world/.*/state$"; then
        pgrep -f "gz sim -g" >/dev/null || nohup gz sim -g >/dev/null 2>&1 &
        break
      fi
      sleep 1
    done
  ) &
fi

PX4_GZ_WORLD="${WORLD}" make px4_sitl gz_x500

# === Auto-open Gazebo GUI when not headless ===
if [ "${HEADLESS:-false}" != "true" ]; then
