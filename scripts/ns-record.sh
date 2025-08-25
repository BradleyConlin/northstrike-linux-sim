#!/usr/bin/env bash
set -euo pipefail

OUTBASE="${1:-runs/bag_$(date +%F_%H%M%S)}"
PROFILE="${2:-}"   # optional path to domain-rand profile YAML
mkdir -p "$(dirname "$OUTBASE")"

# Discover topics unless NS_TOPICS provided (space-separated)
if [[ -n "${NS_TOPICS:-}" ]]; then
  TOPICS="$NS_TOPICS"
else
  TOPICS=$(ros2 topic list | egrep -i '/(image|camera|depth|points|imu|gnss|gps|odometry|odom|pose|tf|clock)(/|$)' | tr '\n' ' ')
fi

# Snapshot topics + metadata
META_DIR="${OUTBASE}_meta"
mkdir -p "$META_DIR"
echo "$TOPICS" | tr ' ' '\n' | sed '/^$/d' > "$META_DIR/topics.txt"

# Optional: copy the randomization profile we used
if [[ -n "$PROFILE" && -f "$PROFILE" ]]; then
  cp -f "$PROFILE" "$META_DIR/$(basename "$PROFILE")"
fi

# Stamp git rev if available
{
  echo "start_utc=$(date -u +%FT%TZ)"
  git rev-parse --short HEAD 2>/dev/null | sed 's/^/git_rev=/' || true
} > "$META_DIR/run.info"

echo "[ns-record] recording → $OUTBASE"
echo "[ns-record] topics: $(wc -l < "$META_DIR/topics.txt") saved to $META_DIR/topics.txt"

# Record
ros2 bag record -o "$OUTBASE" $TOPICS
