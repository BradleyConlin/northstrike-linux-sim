#!/usr/bin/env python3
import argparse, csv, hashlib, json, os, random, sys
from pathlib import Path
import cv2
import numpy as np

def sha1_file(p: Path, block=1<<20):
    h = hashlib.sha1()
    with p.open('rb') as f:
        while True:
            b = f.read(block)
            if not b: break
            h.update(b)
    return h.hexdigest()

def read_index(csv_path: Path):
    with csv_path.open('r', newline='') as f:
        r = csv.DictReader(f)
        headers = [h.strip().lower() for h in r.fieldnames]
        # tolerate rgb/depth or rgb_path/depth_path
        rgb_key = 'rgb' if 'rgb' in headers else 'rgb_path'
        depth_key = 'depth' if 'depth' in headers else 'depth_path'
        if rgb_key not in headers or depth_key not in headers:
            raise RuntimeError(f"index.csv must contain 'rgb'/'depth' (or 'rgb_path'/'depth_path'). Found: {headers}")
        rows = []
        for row in r:
            rgb = row.get('rgb', row.get('rgb_path', '')).strip()
            dep = row.get('depth', row.get('depth_path', '')).strip()
            rows.append((rgb, dep))
        return rows

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset_root', required=True, help="Path to dataset dir containing index.csv and splits/")
    ap.add_argument('--out', required=True, help="Where to write manifest.json")
    ap.add_argument('--sample', type=int, default=100, help="Sample N pairs to measure depth stats")
    ap.add_argument('--max_depth_m', type=float, default=20.0, help="Max valid depth (meters)")
    args = ap.parse_args()

    root = Path(args.dataset_root).resolve()
    idx = root / "index.csv"
    if not idx.exists():
        print(f"ERROR: {idx} not found", file=sys.stderr)
        sys.exit(2)

    pairs = read_index(idx)
    total_pairs = len(pairs)
    missing_rgb = []
    missing_depth = []

    for rgb_rel, dep_rel in pairs:
        if not (root / rgb_rel).exists():
            missing_rgb.append(rgb_rel)
        if not (root / dep_rel).exists():
            missing_depth.append(dep_rel)

    # check splits
    splits_dir = root / "splits"
    splits = {}
    for name in ("train", "val", "test"):
        p = splits_dir / f"{name}.csv"
        if p.exists():
            with p.open('r', newline='') as f:
                cnt = sum(1 for _ in f) - 1  # minus header
            splits[name] = cnt

    # quick depth stats on a sample
    sample_n = min(args.sample, total_pairs)
    sample_pairs = random.sample(pairs, sample_n) if sample_n > 0 else []
    zeros = 0
    total_pixels = 0
    over_max = 0
    mm_max = int(args.max_depth_m * 1000.0)

    for rgb_rel, dep_rel in sample_pairs:
        dep_path = root / dep_rel
        img = cv2.imread(str(dep_path), cv2.IMREAD_UNCHANGED)
        if img is None:
            continue
        if img.dtype != np.uint16:
            # try convert if 8-bit depth sneaks in
            img = img.astype(np.uint16)
        total_pixels += img.size
        zeros += int((img == 0).sum())
        over_max += int((img > mm_max).sum())

    zeros_pct = (zeros / total_pixels * 100.0) if total_pixels else None
    over_max_pct = (over_max / total_pixels * 100.0) if total_pixels else None

    manifest = {
        "dataset_root": str(root),
        "index_csv": "index.csv",
        "index_sha1": sha1_file(idx),
        "total_pairs": total_pairs,
        "splits_counts": splits,
        "missing_rgb_count": len(missing_rgb),
        "missing_depth_count": len(missing_depth),
        "missing_rgb_examples": missing_rgb[:5],
        "missing_depth_examples": missing_depth[:5],
        "depth_sample_pairs": sample_n,
        "zeros_percent_estimate": zeros_pct,
        "over_max_percent_estimate": over_max_pct,
        "max_depth_m": args.max_depth_m,
    }

    outp = Path(args.out)
    outp.parent.mkdir(parents=True, exist_ok=True)
    with outp.open('w') as f:
        json.dump(manifest, f, indent=2)
    print(f"Wrote {outp}")

if __name__ == "__main__":
    main()
