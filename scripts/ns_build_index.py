#!/usr/bin/env python3
import argparse, csv, math
from pathlib import Path

def parse_time_from_name(p: Path):
    try:
        return float(p.stem)
    except Exception:
        return None

def list_images(dirpath: Path):
    files = sorted([p for p in dirpath.glob("*.png") if p.is_file()],
                   key=lambda p: p.stat().st_size)  # cheap pre-sort noise filter
    # final sort by parsed timestamp
    files = sorted([p for p in files if parse_time_from_name(p) is not None],
                   key=lambda p: parse_time_from_name(p))
    times = [parse_time_from_name(p) for p in files]
    return times, files

def pair_nearest(rgb_times, rgb_files, depth_times, depth_files, max_dt):
    pairs = []
    j = 0
    n_depth = len(depth_times)
    for i, t in enumerate(rgb_times):
        # advance depth pointer up to just <= t
        while j + 1 < n_depth and depth_times[j+1] <= t:
            j += 1
        # consider j and j+1
        cand = []
        if 0 <= j < n_depth:
            cand.append((abs(depth_times[j]-t), j))
        if 0 <= j+1 < n_depth:
            cand.append((abs(depth_times[j+1]-t), j+1))
        if not cand:
            continue
        _, jbest = min(cand, key=lambda x: x[0])
        dt = abs(depth_times[jbest] - t)
        if dt <= max_dt:
            pairs.append((t, rgb_files[i], depth_times[jbest], depth_files[jbest], dt))
    return pairs

def write_csv(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_rgb", "rgb_path", "t_depth", "depth_path", "abs_dt"])
        for t_rgb, rgb, t_d, dep, dt in rows:
            # store relative paths to dataset root
            w.writerow([f"{t_rgb:.9f}",
                        str(rgb.as_posix()),
                        f"{t_d:.9f}",
                        str(dep.as_posix()),
                        f"{dt:.6f}"])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="Dataset root (contains rgb/ and depth/)")
    ap.add_argument("--max_dt", type=float, default=0.05, help="Max |dt| to pair (seconds)")
    ap.add_argument("--train", type=float, default=0.8, help="Train fraction")
    ap.add_argument("--val",   type=float, default=0.1, help="Val fraction (rest is test)")
    args = ap.parse_args()

    root = Path(args.out).expanduser().resolve()
    rgb_dir = root / "rgb"
    dep_dir = root / "depth"
    assert rgb_dir.is_dir() and dep_dir.is_dir(), "rgb/ and depth/ must exist"

    print(f"[index] scanning {rgb_dir} and {dep_dir} ...")
    rgb_t, rgb_f = list_images(rgb_dir)
    dep_t, dep_f = list_images(dep_dir)

    print(f"[index] rgb: {len(rgb_t)}  depth: {len(dep_t)}  (pairing with max_dt={args.max_dt}s)")
    pairs = pair_nearest(rgb_t, rgb_f, dep_t, dep_f, args.max_dt)

    # sort by RGB time and write full index
    pairs.sort(key=lambda r: r[0])
    write_csv(root / "index.csv", [
        (t, p.relative_to(root), td, q.relative_to(root), dt) for (t,p,td,q,dt) in pairs
    ])
    print(f"[index] wrote {root/'index.csv'}  pairs={len(pairs)}  hit_rate={len(pairs)/max(1,len(rgb_t)):.2%}")

    # splits by chronology to avoid leakage across time (common best practice)
    n = len(pairs)
    n_train = int(n * args.train)
    n_val   = int(n * args.val)
    n_test  = n - n_train - n_val
    splits = {
        "train.csv": pairs[:n_train],
        "val.csv":   pairs[n_train:n_train+n_val],
        "test.csv":  pairs[n_train+n_val:],
    }
    (root/"splits").mkdir(exist_ok=True)
    for name, rows in splits.items():
        write_csv(root / "splits" / name, [
            (t, (root/p).relative_to(root), td, (root/q).relative_to(root), dt)
            for (t,p,td,q,dt) in rows
        ])
        print(f"[index] wrote splits/{name}  rows={len(rows)}")

if __name__ == "__main__":
    main()
