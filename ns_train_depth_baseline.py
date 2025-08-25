#!/usr/bin/env python3
import argparse, csv, os, math
from pathlib import Path
import cv2, numpy as np, torch, torch.nn as nn, torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader

# ---------- Data ----------
def read_index(root: Path):
    idx = {}
    with (root/'index.csv').open('r', newline='') as f:
        r = csv.DictReader(f)
        rgb_key = 'rgb' if 'rgb' in r.fieldnames else 'rgb_path'
        dep_key = 'depth' if 'depth' in r.fieldnames else 'depth_path'
        for row in r:
            idx[row[rgb_key].strip()] = row[dep_key].strip()
    return idx

def read_split(root: Path, name: str):
    p = root/'splits'/f'{name}.csv'
    if not p.exists(): return []
    out = []
    with p.open('r', newline='') as f:
        r = csv.DictReader(f)
        key = 'rgb' if 'rgb' in r.fieldnames else 'rgb_path'
        for row in r:
            out.append(row[key].strip())
    return out

class DepthPairs(Dataset):
    def __init__(self, root, split, size=(320,240), max_depth_m=20.0):
        self.root = Path(root); self.size = tuple(size); self.max_m = float(max_depth_m)
        self.map = read_index(self.root)
        self.rgb_list = read_split(self.root, split)
        if not self.rgb_list:  # fallback: all pairs
            self.rgb_list = list(self.map.keys())
        self.mm_max = int(self.max_m*1000.0)

    def __len__(self): return len(self.rgb_list)

    def __getitem__(self, i):
        rgb_rel = self.rgb_list[i]
        dep_rel = self.map[rgb_rel]
        rgb = cv2.cvtColor(cv2.imread(str(self.root/rgb_rel), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        dep = cv2.imread(str(self.root/dep_rel), cv2.IMREAD_UNCHANGED)
        H, W = self.size[1], self.size[0]
        rgb = cv2.resize(rgb, (W,H), interpolation=cv2.INTER_AREA)
        if dep is None:
            dep = np.zeros((H,W), np.uint16)
        else:
            dep = cv2.resize(dep, (W,H), interpolation=cv2.INTER_NEAREST)
        # depth to meters in [0,max_m]
        if dep.dtype==np.float32: dep_m = np.clip(dep, 0, self.max_m)
        else: dep_m = np.clip(dep.astype(np.float32)/1000.0, 0, self.max_m)
        # model uses normalized 0..1
        dep_n = (dep_m / self.max_m).astype(np.float32)
        rgb_n = (rgb.astype(np.float32)/255.0).transpose(2,0,1)  # CHW
        return torch.from_numpy(rgb_n), torch.from_numpy(dep_n[None,...])  # (3,H,W), (1,H,W)

# ---------- Model ----------
class ConvBNReLU(nn.Module):
    def __init__(self, c_in, c_out, k=3, s=1):
        super().__init__()
        p = k//2
        self.net = nn.Sequential(
            nn.Conv2d(c_in, c_out, k, s, p, bias=False),
            nn.BatchNorm2d(c_out),
            nn.ReLU(inplace=True)
        )
    def forward(self,x): return self.net(x)

class TinyUNet(nn.Module):
    def __init__(self, in_ch=3, base=32):
        super().__init__()
        self.enc1 = nn.Sequential(ConvBNReLU(in_ch, base), ConvBNReLU(base, base))
        self.enc2 = nn.Sequential(nn.MaxPool2d(2), ConvBNReLU(base, base*2), ConvBNReLU(base*2, base*2))
        self.enc3 = nn.Sequential(nn.MaxPool2d(2), ConvBNReLU(base*2, base*4), ConvBNReLU(base*4, base*4))
        self.bot  = nn.Sequential(nn.MaxPool2d(2), ConvBNReLU(base*4, base*8), ConvBNReLU(base*8, base*8))
        self.up3  = nn.ConvTranspose2d(base*8, base*4, 2, 2)
        self.dec3 = nn.Sequential(ConvBNReLU(base*8, base*4), ConvBNReLU(base*4, base*4))
        self.up2  = nn.ConvTranspose2d(base*4, base*2, 2, 2)
        self.dec2 = nn.Sequential(ConvBNReLU(base*4, base*2), ConvBNReLU(base*2, base*2))
        self.up1  = nn.ConvTranspose2d(base*2, base, 2, 2)
        self.dec1 = nn.Sequential(ConvBNReLU(base*2, base), ConvBNReLU(base, base))
        self.out  = nn.Conv2d(base, 1, 1)

    def forward(self,x):
        e1 = self.enc1(x)
        e2 = self.enc2(e1)
        e3 = self.enc3(e2)
        b  = self.bot(e3)
        d3 = self.dec3(torch.cat([self.up3(b), e3], 1))
        d2 = self.dec2(torch.cat([self.up2(d3), e2], 1))
        d1 = self.dec1(torch.cat([self.up1(d2), e1], 1))
        y  = self.out(d1)
        return torch.sigmoid(y)  # normalized 0..1

# ---------- Train ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--epochs', type=int, default=8)
    ap.add_argument('--size', nargs=2, type=int, default=[320,240])
    ap.add_argument('--batch', type=int, default=32)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--max_depth_m', type=float, default=20.0)
    ap.add_argument('--out', default='runs/depth_tinyunet')
    args = ap.parse_args()

    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'device={device}')

    train_ds = DepthPairs(args.data, 'train', tuple(args.size), args.max_depth_m)
    val_ds   = DepthPairs(args.data, 'val',   tuple(args.size), args.max_depth_m)
    train_dl = DataLoader(train_ds, batch_size=args.batch, shuffle=True, num_workers=4, pin_memory=True)
    val_dl   = DataLoader(val_ds,   batch_size=args.batch, shuffle=False, num_workers=4, pin_memory=True)

    model = TinyUNet().to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    best_val = 1e9

    for ep in range(1, args.epochs+1):
        model.train(); tr_loss=0.0; n=0
        for rgb,depth in train_dl:
            rgb,depth = rgb.to(device), depth.to(device)
            pred = model(rgb)
            loss = F.mse_loss(pred, depth)
            opt.zero_grad(); loss.backward(); opt.step()
            tr_loss += loss.item()*rgb.size(0); n+=rgb.size(0)
        tr_loss/=max(1,n)

        # simple val MSE
        model.eval(); va_loss=0.0; m=0
        with torch.no_grad():
            for rgb,depth in val_dl:
                rgb,depth = rgb.to(device), depth.to(device)
                pred = model(rgb)
                loss = F.mse_loss(pred, depth)
                va_loss += loss.item()*rgb.size(0); m+=rgb.size(0)
        va_loss/=max(1,m); sch.step()

        ckpt = out/f'_ckpt_ep{ep}.pt'
        torch.save({'model':model.state_dict(), 'size':tuple(args.size), 'max_depth_m':args.max_depth_m}, ckpt)
        if va_loss<best_val:
            best_val=va_loss; torch.save({'model':model.state_dict(), 'size':tuple(args.size), 'max_depth_m':args.max_depth_m}, out/'_ckpt_best.pt')
        print(f'Epoch {ep:02d}/{args.epochs}  train_mse={tr_loss:.6f}  val_mse={va_loss:.6f}  lr={sch.get_last_lr()[0]:.6e}')
    print('done')

if __name__=='__main__': main()
