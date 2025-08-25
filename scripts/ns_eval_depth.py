#!/usr/bin/env python3
import argparse, json, math, random
from pathlib import Path
import numpy as np
import cv2
import torch
import torch.nn.functional as F

# local imports (your training + dataset)
from ns_train_depth_baseline import TinyUNet
try:
    from ns_rgbd_dataset import RgbdDataset
except ImportError:
    from ns_rgbd_dataset import RGBDPairDataset as RgbdDataset

def colorize01(x01):
    x = (np.clip(x01, 0, 1) * 255).astype(np.uint8)
    return cv2.applyColorMap(x, cv2.COLORMAP_TURBO)

def make_grid(rows, cols):
    def pad_to_same(ws):
        H = max(r.shape[0] for r in ws)
        W = max(r.shape[1] for r in ws)
        out = []
        for im in ws:
            h,w = im.shape[:2]
            top = (H-h)//2; bottom = H-h-top
            left = (W-w)//2; right = W-w-left
            out.append(cv2.copyMakeBorder(im, top,bottom,left,right, cv2.BORDER_CONSTANT, value=(0,0,0)))
        return out
    rows = [pad_to_same(r) for r in rows]
    row_imgs = [cv2.hconcat(r) for r in rows]
    return cv2.vconcat(row_imgs)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--size", type=int, nargs=2, default=(320,240), help="W H")
    ap.add_argument("--split", default="val", choices=["train","val","test"])
    ap.add_argument("--limit", type=int, default=5000)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--tiles", type=int, default=24)
    ap.add_argument("--cols", type=int, default=6)
    args = ap.parse_args()

    OUT = Path(args.out)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # model
    model = TinyUNet().to(device)
    ck = torch.load(args.ckpt, map_location="cpu")
    model.load_state_dict(ck["model"]); model.eval()

    # data (returns rgb in [0,1], depth_norm in [0,1])
    ds = RgbdDataset(OUT, split=args.split, size=tuple(args.size[::-1]), limit=args.limit)
    dl = torch.utils.data.DataLoader(ds, batch_size=args.batch, shuffle=False, num_workers=2, pin_memory=True)
    max_depth_m = ds.max_depth_m

    # metrics
    tot_mae = 0.0
    tot_mse = 0.0
    tot_n   = 0

    with torch.no_grad():
        for x,y in dl:
            x = x.to(device)         # (B,3,H,W) in [0,1]
            y = y.to(device)         # (B,1,H,W) in [0,1]
            p = model(x)             # (B,1,H,W) in [0,1]
            bsz = x.size(0)
            mae = F.l1_loss(p, y, reduction="mean").item()
            mse = F.mse_loss(p, y, reduction="mean").item()
            tot_mae += mae * bsz
            tot_mse += mse * bsz
            tot_n   += bsz

    mae01 = tot_mae / max(1, tot_n)
    mse01 = tot_mse / max(1, tot_n)
    rmse01 = math.sqrt(mse01)

    # convert to meters (dataset uses depth_norm = depth_m / max_depth_m)
    mae_m   = mae01 * max_depth_m
    rmse_m  = rmse01 * max_depth_m

    report = {
        "split": args.split,
        "count_images": tot_n,
        "max_depth_m": max_depth_m,
        "mae_norm01": round(mae01, 6),
        "rmse_norm01": round(rmse01, 6),
        "mae_m": round(mae_m, 4),
        "rmse_m": round(rmse_m, 4),
    }
    rep_path = OUT / f"_eval_{args.split}.json"
    rep_path.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))

    # -------- small visual grid --------
    take = min(args.tiles, len(ds))
    sampled = random.sample(range(len(ds)), take)
    rows = []
    with torch.no_grad():
        for i,idx in enumerate(sampled):
            rgb, gt = ds[idx]                  # tensors
            rgb_np = (rgb.permute(1,2,0).numpy()*255).astype(np.uint8)  # HxWx3
            with torch.no_grad():
                pred = model(rgb.unsqueeze(0).to(device))[0,0].cpu().numpy()  # HxW (0..1)
            gt_np = gt[0].numpy()

            # colorize
            c_pred = colorize01(pred)
            c_gt   = colorize01(gt_np)

            # stack horizontally: RGB | Pred | GT
            trip = cv2.hconcat([rgb_np, c_pred, c_gt])

            # start new row every 'cols'
            if i % args.cols == 0: rows.append([])
            rows[-1].append(trip)

    # pad last row if needed
    while len(rows[-1]) < args.cols:
        rows[-1].append(np.zeros_like(rows[-1][0]))
    grid = make_grid(rows, args.cols)
    grid_path = OUT / f"_eval_{args.split}_grid.jpg"
    cv2.imwrite(str(grid_path), grid)
    print("Wrote:", grid_path, "and", rep_path)

if __name__ == "__main__":
    main()
