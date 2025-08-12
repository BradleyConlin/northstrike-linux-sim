#!/usr/bin/env bash
set -euo pipefail

for i in $(seq 1 "${GZ_GUI_WAIT_MAX:-30}"); do
  if gz topic -l 2>/dev/null | grep -qE '^/world/.*/state$'; then
    break
  fi
  sleep 1
done

pgrep -f "gz sim -g" >/dev/null || nohup gz sim -g >/dev/null 2>&1 &
