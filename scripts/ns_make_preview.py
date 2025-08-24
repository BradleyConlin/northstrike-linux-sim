#!/usr/bin/env python3
import argparse, os, glob
from pathlib import Path
import cv2
import numpy as np

def imread_rgb(path: str, tile_w: int, tile_h: int) -> np.ndarray:
    img = cv2.imread(path, cv2.IMREAD_COLOR)  # BGR uint8 (3ch)
    if img is None:
        return np.full((tile_h, tile_w, 3), 0, np.uint8)
    img = cv2.resize(img, (tile_w, tile_h), interpolation=cv2.INTER_AREA)
    return img

def imread_depth_colored(path: str, tile_w: int, tile_h: int) -> np.ndarray:
    # Load unchanged; could be 16-bit depth or already 8/3ch
    raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if raw is None:
        return np.full((tile_h, tile_w, 3), 0, np.uint8)

    if raw.ndim == 2 and raw.dtype == np.uint16:
        # Normalize 16-bit to 8-bit and color-map
        # Robust to empty frames: clamp min/max
        dmin, dmax = int(np.percentile(raw, 1)), int(np.percentile(raw, 99))
        if dmax <= dmin:
            dmin, dmax = 0, 65535
        depth8 = cv2.convertScaleAbs(np.clip(raw, dmin, dmax), alpha=255.0 / max(1, dmax - dmin))
        color = cv2.applyColorMap(depth8, cv2.COLORMAP_TURBO)
    else:
        # If it's already 8-bit 3ch or 1ch
        if raw.ndim == 2:
            color = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
        else:
            color = raw.astype(np.uint8, copy=False)

    color = cv2.resize(color, (tile_w, tile_h), interpolation=cv2.INTER_NEAREST)
    return color

def make_grid(imgs: list[np.ndarray], cols: int) -> np.ndarray:
    if not imgs:
        return np.zeros((1, 1, 3), np.uint8)
    h, w = imgs[0].shape[:2]
    # Safeguard: ensure uniform size/type
    safe = []
    for im in imgs:
        if im is None:
            im = np.full((h, w, 3), 0, np.uint8)
        if im.dtype != np.uint8:
            im = im.astype(np.uint8)
        if im.ndim == 2:
            im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
        if im.shape[:2] != (h, w):
            im = cv2.resize(im, (w, h), interpolation=cv2.INTER_AREA)
        safe.append(im)

    rows = []
    for i in range(0, len(safe), cols):
        row = safe[i:i+cols]
        # pad short row to full width
        while len(row) < cols:
            row.append(np.full((h, w, 3), 0, np.uint8))
        rows.append(cv2.hconcat(row))  # same height/type → OK
    grid = cv2.vconcat(rows)          # same width/type across rows → OK
    return grid

def pad_to_width(img: np.ndarray, target_w: int) -> np.ndarray:
    h, w = img.shape[:2]
    if w == target_w:
        return img
    pad = target_w - w
    left = pad // 2
    right = pad - left
    return cv2.copyMakeBorder(img, 0, 0, left, right, borderType=cv2.BORDER_CONSTANT, value=(0,0,0))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="dataset directory containing rgb/ and depth/")
    ap.add_argument("--n", type=int, default=24, help="total tiles per modality")
    ap.add_argument("--cols", type=int, default=6, help="columns per grid")
    ap.add_argument("--tile", type=int, default=256, help="tile width/height in pixels")
    args = ap.parse_args()

    out = Path(args.out)
    rgb_dir = out / "rgb"
    depth_dir = out / "depth"
    rgb_paths = sorted(glob.glob(str(rgb_dir / "*.png")))[:args.n]
    depth_paths = sorted(glob.glob(str(depth_dir / "*.png")))[:args.n]

    # Build per-modality grids (each image normalized to tile x tile and 3-channel uint8)
    rgb_imgs = [imread_rgb(p, args.tile, args.tile) for p in rgb_paths]
    depth_imgs = [imread_depth_colored(p, args.tile, args.tile) for p in depth_paths]

    rgb_grid = make_grid(rgb_imgs, args.cols) if rgb_imgs else None
    depth_grid = make_grid(depth_imgs, args.cols) if depth_imgs else None

    tiles = [g for g in [rgb_grid, depth_grid] if g is not None]

    if not tiles:
        raise SystemExit("No images found in rgb/ or depth/")

    if len(tiles) == 1:
        canvas = tiles[0]
    else:
        # Ensure same width and type before vconcat (OpenCV requirement)
        max_w = max(t.shape[1] for t in tiles)
        tiles = [pad_to_width(t.astype(np.uint8), max_w) for t in tiles]
        canvas = cv2.vconcat(tiles)

    # optional label strip at top
    label_h = 40
    strip = np.full((label_h, canvas.shape[1], 3), 30, np.uint8)
    cv2.putText(strip, f"RGB (top) • Depth (bottom)  |  tiles={args.n} cols={args.cols} tile={args.tile}",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (240,240,240), 2, cv2.LINE_AA)
    out_img = cv2.vconcat([strip, canvas])

    preview = str(out / "preview.jpg")
    cv2.imwrite(preview, out_img)
    print("Wrote", preview)

if __name__ == "__main__":
    main()
