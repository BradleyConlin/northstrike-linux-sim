#!/usr/bin/env python3
import argparse, os
from pathlib import Path
import numpy as np
import cv2
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from ns_rgbd_dataset import RGBDPairDataset

__all__ = ["TinyUNet"]


# --------- a super small encoder-decoder ---------
class SmallDepthNet(nn.Module):
    def __init__(self):
        super().__init__()
        c = [3, 32, 64, 128]
        self.enc1 = nn.Sequential(nn.Conv2d(c[0], c[1], 3, 2, 1), nn.ReLU(True))
        self.enc2 = nn.Sequential(nn.Conv2d(c[1], c[2], 3, 2, 1), nn.ReLU(True))
        self.enc3 = nn.Sequential(nn.Conv2d(c[2], c[3], 3, 2, 1), nn.ReLU(True))
        self.dec3 = nn.Sequential(nn.ConvTranspose2d(c[3], c[2], 4, 2, 1), nn.ReLU(True))
        self.dec2 = nn.Sequential(nn.ConvTranspose2d(c[2], c[1], 4, 2, 1), nn.ReLU(True))
        self.dec1 = nn.Sequential(nn.ConvTranspose2d(c[1], 32, 4, 2, 1), nn.ReLU(True))
        self.out = nn.Sequential(nn.Conv2d(32, 1, 3, 1, 1), nn.Sigmoid())  # predict normalized depth [0,1]

    def forward(self, x):
        x1 = self.enc1(x)
        x2 = self.enc2(x1)
        x3 = self.enc3(x2)
        y = self.dec3(x3)
        y = self.dec2(y)
        y = self.dec1(y)
        return self.out(y)

TinyUNet = SmallDepthNet

def colorize_depth_norm(dep01):
    d8 = (np.clip(dep01, 0, 1) * 255).astype(np.uint8)
    return cv2.applyColorMap(d8, cv2.COLORMAP_TURBO)

def save_preview(rgb_bchw, pred_bchw, out_jpg, max_depth_m=20.0):
    rgb = (rgb_bchw[0].permute(1,2,0).cpu().numpy() * 255).astype(np.uint8)  # HxWx3 RGB
    pred = pred_bchw[0,0].detach().cpu().numpy()  # HxW [0,1]
    color = colorize_depth_norm(pred)
    overlay = cv2.addWeighted(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), 0.7, color, 0.3, 0)
    cv2.imwrite(str(out_jpg), overlay)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="dataset root (where splits/*.csv live)")
    ap.add_argument("--size", nargs=2, type=int, default=(320,240), help="train size W H")
    ap.add_argument("--limit", type=int, default=10000, help="max pairs to use (train)")
    ap.add_argument("--epochs", type=int, default=1)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--max_depth_m", type=float, default=20.0)
    args = ap.parse_args()

    W,H = args.size
    root = Path(args.out)

    train_ds = RGBDPairDataset(root, "train", (W,H), args.max_depth_m, limit=args.limit, augment=True)
    val_ds   = RGBDPairDataset(root, "val",   (W,H), args.max_depth_m, limit=2000, augment=False)

    # DataLoader batches (standard PyTorch utility)
    train_ld = DataLoader(train_ds, batch_size=args.batch, shuffle=True, num_workers=4, pin_memory=True)
    val_ld   = DataLoader(val_ds,   batch_size=args.batch, shuffle=False, num_workers=2, pin_memory=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = SmallDepthNet().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.L1Loss()  # robust + simple

    steps = 0
    for ep in range(1, args.epochs+1):
        model.train()
        for rgb, dep in train_ld:
            rgb = rgb.to(device, non_blocking=True)
            dep = dep.to(device, non_blocking=True)
            pred = model(rgb)
            loss = loss_fn(pred, dep)
            opt.zero_grad(); loss.backward(); opt.step()
            steps += 1
            if steps % 50 == 0:
                print(f"[ep{ep}] step {steps}  loss {loss.item():.004f}", flush=True)

        # quick val + preview frame
        model.eval()
        with torch.no_grad():
            for rgb, dep in val_ld:
                pred = model(rgb.to(device))
                out_jpg = root / "_pred_preview.jpg"
                save_preview(rgb, pred, out_jpg, args.max_depth_m)
                print("Wrote preview:", out_jpg)
                break

        # save checkpoint
        ckpt = root / f"_ckpt_ep{ep}.pt"
        torch.save({"model": model.state_dict(), "ep": ep, "W": W, "H": H}, ckpt)
        print("Saved:", ckpt)

if __name__ == "__main__":
    main()
