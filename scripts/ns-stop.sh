#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# normal project cleanup
./stop_all.sh || true
python3 clean_env_linux.py || true

# kill anything stubborn
pkill -9 -f "px4_sitl_default/bin/px4"      || true
pkill -9 -f "px4 server"                     || true
pkill -9 -f "gz sim"                         || true
pkill -9 -f "gzserver|gz-transport|gz_bridge"|| true
pkill -9 -f QGroundControl.AppImage          || true
# free UDP ports commonly used by PX4/QGC/MAVSDK
fuser -k 14540/udp 14550/udp 14580/udp 13030/udp 13280/udp 14280/udp 14030/udp 18570/udp 2>/dev/null || true
# remove PX4 lockfiles (name patterns vary by build)
rm -f /tmp/px4-*.lock /tmp/px4_lock-* /tmp/PX4_lock* 2>/dev/null || true
# clear any tmp in the build that might hold a lock
if [ -d ~/dev/px4-autopilot-harmonic/build/px4_sitl_default/tmp ]; then
  rm -rf ~/dev/px4-autopilot-harmonic/build/px4_sitl_default/tmp/* || true
fi
sleep 0.5
