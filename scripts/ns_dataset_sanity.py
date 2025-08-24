#!/usr/bin/env python3
import csv, json, random, math
from pathlib import Path
from dataclasses import dataclass, asdict
import numpy as np
import cv2

@dataclass
class ImageStats:
    valid_px: int
    total_px: int
    zero_px: int
    min_mm: int
    max_mm: int
    median_mm: float

@dataclass
class Summary:
    samples: int
    depth_valid_px: int
    depth_total_px: int
    depth_zero_px: int
    valid_ratio: float
    zero_ratio: float
    min_mm: int
    max_mm: int
    median_mm_over_imgs: float

def load_pairs(out_dir: Path):
    idx = out_dir / "index.csv"
    rows = list(csv.DictReader(open(idx)))
    return rows

def read_rgb(p: Path):
    img = cv2.imread(str(p), cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(p)
    return img

def read_depth(p: Path):
    dep = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
    if dep is None:
        raise FileNotFoundError(p)
    if dep.dtype != np.uint16:
        dep = dep.astype(np.uint16)
    return dep

def depth_stats(dep_u16: np.ndarray) -> ImageStats:
    total = int(dep_u16.size)
    zero = int((dep_u16 == 0).sum())
    valid = total - zero
    if valid == 0:
        return ImageStats(valid, total, zero, 0, 0, 0.0)
    valid_vals = dep_u16[dep_u16 > 0]
    return ImageStats(
        valid_px=valid,
        total_px=total,
        zero_px=zero,
        min_mm=int(valid_vals.min()),
        max_mm=int(valid_vals.max()),
        median_mm=float(np.median(valid_vals)),
    )

def make_overlay(rgb, dep_u16):
    # Normalize depth for display, colorize, resize to RGB
    d8 = (np.clip(dep_u16 / max(1, dep_u16.max()), 0, 1) * 255).astype(np.uint8)
    color = cv2.applyColorMap(d8, cv2.COLORMAP_TURBO)
    color = cv2.resize(color, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
    return cv2.addWeighted(rgb, 0.7, color, 0.3, 0.0)

def grid(images, cols=6, tile_hw=(240, 320)):
    th, tw = tile_hw
    tiles = []
    row = []
    for i, img in enumerate(images):
        tile = cv2.resize(img, (tw, th), interpolation=cv2.INTER_AREA)
        row.append(tile)
        if (i + 1) % cols == 0:
            tiles.append(cv2.hconcat(row)); row = []
    if row:  # last partial row
        # pad with black tiles to align width
        pad = cols - len(row)
        row += [np.zeros_like(row[0])] * pad
        tiles.append(cv2.hconcat(row))
    return tiles[0] if len(tiles) == 1 else cv2.vconcat(tiles)

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="Dataset directory (with index.csv)")
    ap.add_argument("--samples", type=int, default=500, help="Number of pairs to sample")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--tiles", type=int, default=24, help="tiles in preview grid")
    args = ap.parse_args()

    out = Path(args.out).expanduser()
    rows = load_pairs(out)
    random.seed(args.seed)
    pairs = random.sample(rows, min(args.samples, len(rows)))

    overlays = []
    all_medians = []
    agg_valid = agg_total = agg_zero = 0
    g_min = math.inf
    g_max = 0

    for r in pairs:
        rgb = read_rgb(out / r["rgb_path"])
        dep = read_depth(out / r["depth_path"])
        st = depth_stats(dep)
        agg_valid += st.valid_px
        agg_total += st.total_px
        agg_zero += st.zero_px
        g_min = min(g_min, st.min_mm) if st.valid_px else g_min
        g_max = max(g_max, st.max_mm) if st.valid_px else g_max
        if st.valid_px:
            all_medians.append(st.median_mm)
        if len(overlays) < args.tiles:
            overlays.append(make_overlay(rgb, dep))

    summary = Summary(
        samples=len(pairs),
        depth_valid_px=agg_valid,
        depth_total_px=agg_total,
        depth_zero_px=agg_zero,
        valid_ratio=float(agg_valid / max(1, agg_total)),
        zero_ratio=float(agg_zero / max(1, agg_total)),
        min_mm=(0 if g_min is math.inf else int(g_min)),
        max_mm=int(g_max),
        median_mm_over_imgs=(float(np.median(all_medians)) if all_medians else 0.0),
    )
    (out / "sanity_report.json").write_text(json.dumps(asdict(summary), indent=2))

    if overlays:
        canvas = grid(overlays, cols=args.cols, tile_hw=(240,320))
        cv2.imwrite(str(out / "sanity_preview.jpg"), canvas)

    print(json.dumps(asdict(summary), indent=2))
    print("Wrote:", out / "sanity_report.json")
    if overlays:
        print("Preview:", out / "sanity_preview.jpg")

if __name__ == "__main__":
    main()
