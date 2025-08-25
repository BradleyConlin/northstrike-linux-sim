#!/usr/bin/env python3
import argparse, csv, os, random, shutil, time
from pathlib import Path
import cv2
import numpy as np

# Paired offline aug for RGB (uint8) + depth (uint16 mm or 32F meters -> converted)
# - Horizontal flip (p=0.5)
# - Random resized crop [0.8..1.0] scale, keep aspect
# - Optional gaussian blur on RGB
# - Light color jitter (brightness/contrast)
# Re-run safe: skips writing files that already exist.

def ensure_dir(p: Path): p.mkdir(parents=True, exist_ok=True)

def load_index(idx: Path):
    with idx.open('r', newline='') as f:
        r = csv.DictReader(f)
        rows = []
        rgb_key = 'rgb' if 'rgb' in r.fieldnames else 'rgb_path'
        dep_key = 'depth' if 'depth' in r.fieldnames else 'depth_path'
        for row in r:
            rows.append((row[rgb_key].strip(), row[dep_key].strip()))
        return rows, rgb_key, dep_key

def write_index(idx: Path, rows, rgb_key='rgb', dep_key='depth'):
    with idx.open('w', newline='') as f:
        w = csv.writer(f); w.writerow([rgb_key, dep_key])
        for a,b in rows: w.writerow([a,b])

def rand_crop(h, w, scale_min=0.8):
    s = random.uniform(scale_min, 1.0)
    nh, nw = int(h*s), int(w*s)
    y = 0 if nh==h else random.randint(0, h-nh)
    x = 0 if nw==w else random.randint(0, w-nw)
    return x, y, nw, nh

def clamp01(x): return np.clip(x, 0, 1)
def color_jitter_rgb(rgb):
    b = random.uniform(0.85, 1.15)  # brightness
    c = random.uniform(0.85, 1.15)  # contrast
    return clamp01((rgb * c) * b)

def augment_pair(rgb_img, depth_img):
    if random.random() < 0.5:
        rgb_img = cv2.flip(rgb_img, 1)
        depth_img = cv2.flip(depth_img, 1)
    h, w = rgb_img.shape[:2]
    x, y, nw, nh = rand_crop(h, w, 0.8)
    rgb_crop = rgb_img[y:y+nh, x:x+nw]
    dep_crop = depth_img[y:y+nh, x:x+nw]
    rgb_rs = cv2.resize(rgb_crop, (w, h), interpolation=cv2.INTER_AREA)
    dep_rs = cv2.resize(dep_crop, (w, h), interpolation=cv2.INTER_NEAREST)
    if random.random() < 0.4:
        rgb_rs = cv2.GaussianBlur(rgb_rs, (3,3), 0)
    rgb_f = rgb_rs.astype(np.float32) / 255.0
    rgb_f = color_jitter_rgb(rgb_f)
    rgb_aug = (rgb_f * 255.0).astype(np.uint8)
    return rgb_aug, dep_rs

def read_split_paths(split_csv: Path):
    if not split_csv.exists(): return set()
    with split_csv.open('r', newline='') as f:
        r = csv.DictReader(f)
        key = 'rgb' if 'rgb' in r.fieldnames else 'rgb_path'
        return set(row[key].strip() for row in r)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True, help="Input dataset dir (has index.csv, splits/)")
    ap.add_argument("--out", required=True, help="Output dataset dir")
    ap.add_argument("--copies", type=int, default=1, help="Augmented copies per *train* pair")
    ap.add_argument("--max_depth_m", type=float, default=20.0)
    args = ap.parse_args()

    t0 = time.time()
    src = Path(args.inp).resolve()
    dst = Path(args.out).resolve()
    ensure_dir(dst / "rgb"); ensure_dir(dst / "depth"); ensure_dir(dst / "splits")

    idx_in = src / "index.csv"
    rows, rgb_key, dep_key = load_index(idx_in)

    # Build split sets
    train_set = read_split_paths(src / "splits" / "train.csv")
    val_set   = read_split_paths(src / "splits" / "val.csv")
    test_set  = read_split_paths(src / "splits" / "test.csv")

    print(f"[augment] src={src}")
    print(f"[augment] dst={dst}")
    print(f"[augment] pairs={len(rows)}  train={len(train_set)}  val={len(val_set)}  test={len(test_set)}")
    print(f"[augment] copies per train pair={args.copies}")

    # Mirror originals to flattened names for deterministic remap
    new_rows = []
    for i, (rgb_rel, dep_rel) in enumerate(rows):
        rp = src / rgb_rel; dp = src / dep_rel
        rgb_name = f"{i:08d}.jpg"; dep_name = f"{i:08d}.png"
        rgb_out = dst / "rgb" / rgb_name
        dep_out = dst / "depth" / dep_name
        if not rgb_out.exists(): shutil.copy2(rp, rgb_out)
        if not dep_out.exists(): shutil.copy2(dp, dep_out)
        new_rows.append((f"rgb/{rgb_name}", f"depth/{dep_name}"))
        if (i+1) % 5000 == 0:
            print(f"[copy] originals {i+1}/{len(rows)}")

    # Augment *train* only
    mm_max = int(args.max_depth_m * 1000.0)
    idx_map = {rows[i][0]: i for i in range(len(rows))}
    aug_rows = []
    done = 0; todo = len(train_set) * args.copies; last = time.time()

    for rgb_rel, _ in rows:
        if rgb_rel not in train_set: continue
        i = idx_map[rgb_rel]
        rgb_src = cv2.imread(str(src / rows[i][0]), cv2.IMREAD_COLOR)
        dep_src = cv2.imread(str(src / rows[i][1]), cv2.IMREAD_UNCHANGED)
        if rgb_src is None or dep_src is None: continue
        dep_src = dep_src.astype(np.float32)
        # accept mm (uint16) or meters (32F) and clamp to [0..mm_max]
        if dep_src.dtype == np.float32:
            dep_src = np.clip(dep_src * 1000.0, 0, mm_max).astype(np.uint16)
        else:
            dep_src = np.clip(dep_src, 0, mm_max).astype(np.uint16)

        for c in range(args.copies):
            name_base = f"{i:08d}_a{c}"
            rgb_out = dst / "rgb" / f"{name_base}.jpg"
            dep_out = dst / "depth" / f"{name_base}.png"

            # Fast re-run: if both files already exist, just record them
            if rgb_out.exists() and dep_out.exists():
                aug_rows.append((f"rgb/{rgb_out.name}", f"depth/{dep_out.name}"))
            else:
                rgb_aug, dep_aug = augment_pair(rgb_src, dep_src)
                cv2.imwrite(str(rgb_out), rgb_aug, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                cv2.imwrite(str(dep_out), dep_aug)
                aug_rows.append((f"rgb/{rgb_out.name}", f"depth/{dep_out.name}"))

            done += 1
            if todo and done % 1000 == 0:
                dt = time.time() - last; last = time.time()
                print(f"[aug] {done}/{todo} (+1000 since last ~{dt:.1f}s)")

    # Write index and splits
    write_index(dst / "index.csv", new_rows + aug_rows, rgb_key='rgb', dep_key='depth')

    def write_split(name, items):
        p = dst / "splits" / f"{name}.csv"
        with p.open('w', newline='') as f:
            w = csv.writer(f); w.writerow(['rgb'])
            for r in items: w.writerow([r])

    # Map from original RGB path -> new flattened RGB path
    orig_rgb_name = {rows[i][0]: f"rgb/{i:08d}.jpg" for i in range(len(rows))}
    # BUG FIX: use r[0] (RGB path) as the key, not the whole row tuple
    train_remap = [orig_rgb_name[r[0]] for r in rows if r[0] in train_set]
    val_remap   = [orig_rgb_name[r[0]] for r in rows if r[0] in val_set]
    test_remap  = [orig_rgb_name[r[0]] for r in rows if r[0] in test_set]
    # Add augmented names to train only
    train_aug = train_remap + [r for (r,_) in aug_rows]

    write_split("train", train_aug)
    if val_set:  write_split("val",   val_remap)
    if test_set: write_split("test",  test_remap)

    print(f"[done] originals={len(new_rows)} aug_added={len(aug_rows)} total={len(new_rows)+len(aug_rows)}")
    print(f"[done] out={dst}  index.csv and splits written")
    print(f"[time] {(time.time()-t0):.1f}s")

if __name__ == "__main__":
    main()
