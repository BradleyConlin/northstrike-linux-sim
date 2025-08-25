#!/usr/bin/env bash
set -euo pipefail
REPO=${REPO:-$HOME/dev/northstrike-linux-sim}
set +u; source /opt/ros/humble/setup.bash; set -u

BAG=$(ls -dt "$REPO"/runs/sim_* "$REPO"/runs/quick_* "$REPO"/runs/sim_manual_* 2>/dev/null | head -n1)
[ -z "$BAG" ] && { echo "[ns-export] No bag found under $REPO/runs"; exit 1; }

OUT="$REPO/datasets/$(basename "$BAG")"
EXP="$REPO/scripts/ns_export_dataset.py"
[ ! -f "$EXP" ] && { echo "[ns-export] MISSING: $EXP"; exit 2; }

echo "[ns-export] bag=$BAG"
echo "[ns-export] out=$OUT"
mkdir -p "$OUT"

# ---- make a temp work copy and decompress if needed ----
WORKDIR=$(mktemp -d)
rsync -a "$BAG"/ "$WORKDIR"/

if ls "$WORKDIR"/*.db3.zstd >/dev/null 2>&1; then
  echo "[ns-export] Decompressing copy…"
  ros2 bag decompress -s sqlite3 "$WORKDIR" || {
    echo "[ns-export] ros2 bag decompress failed; trying manual unzstd"
    for f in "$WORKDIR"/*.db3.zstd; do unzstd -f "$f"; done
    # best-effort: flip compression flags in metadata so player won't expect compression
    sed -i 's/^compression_mode: .*/compression_mode: ""/' "$WORKDIR/metadata.yaml" || true
    sed -i 's/^compression_format: .*/compression_format: ""/' "$WORKDIR/metadata.yaml" || true
  }
fi

# ---- run exporter node in background, then play the bag ----
python3 "$EXP" --out "$OUT" &
EXP_PID=$!

# explicit storage id
ros2 bag play -s sqlite3 "$WORKDIR" -r 1.0 || true

kill -INT "$EXP_PID" 2>/dev/null || true
wait "$EXP_PID" 2>/dev/null || true
rm -rf "$WORKDIR"

echo "[ns-export] counts:"
[ -d "$OUT/images/rgb" ]   && echo "rgb:   $(find "$OUT/images/rgb"   -maxdepth 1 -type f | wc -l)"
[ -d "$OUT/images/depth" ] && echo "depth: $(find "$OUT/images/depth" -maxdepth 1 -type f | wc -l)"
for f in imu.csv mag.csv gps.csv lidar.csv; do
  [ -f "$OUT/$f" ] && echo "$f: $(wc -l < "$OUT/$f") rows"
done
