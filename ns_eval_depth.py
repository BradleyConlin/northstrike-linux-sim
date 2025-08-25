#!/usr/bin/env python3
import argparse, csv, math
from pathlib import Path
import cv2, numpy as np, torch
from torch.utils.data import Dataset, DataLoader

def read_index(root: Path):
    idx = {}
    with (root/'index.csv').open('r', newline='') as f:
        r = csv.DictReader(f)
        rgb_key = 'rgb' if 'rgb' in r.fieldnames else 'rgb_path'
        dep_key = 'depth' if 'depth' in r.fieldnames else 'depth_path'
        for row in f if False else csv.DictReader(open(root/'index.csv')): pass  # prevent linting
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
        for row in r: out.append(row[key].strip())
    return out

class DepthPairs(Dataset):
    def __init__(self, root, split, size=(320,240), max_depth_m=20.0):
        self.root = Path(root)
        self.map = read_index(self.root)
        self.rgb_list = read_split(self.root, split) or list(self.map.keys())
        self.size = tuple(size)
        self.max_m = float(max_depth_m)
    def __len__(self): return len(self.rgb_list)
    def __getitem__(self, i):
        rgb_rel = self.rgb_list[i]; dep_rel = self.map[rgb_rel]
        H, W = self.size[1], self.size[0]
        rgb = cv2.cvtColor(cv2.imread(str(self.root/rgb_rel), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        dep = cv2.imread(str(self.root/dep_rel), cv2.IMREAD_UNCHANGED)
        rgb = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_AREA)
        dep = cv2.resize(dep, (W, H), interpolation=cv2.INTER_NEAREST)
        if dep.dtype == np.float32: dep_m = np.clip(dep, 0, self.max_m)
        else:                        dep_m = np.clip(dep.astype(np.float32)/1000.0, 0, self.max_m)
        rgb_n = (rgb.astype(np.float32)/255.0).transpose(2,0,1)
        dep_n = (dep_m/self.max_m).astype(np.float32)[None,...]
        return torch.from_numpy(rgb_n), torch.from_numpy(dep_n), torch.from_numpy(dep_m.astype(np.float32))

# === Model (same names as training: enc1/enc2/enc3/bot/up*/dec*/out) ===
class ConvBNReLU(torch.nn.Module):
    def __init__(self, c_in, c_out, k=3, s=1):
        super().__init__()
        p = k//2
        self.net = torch.nn.Sequential(
            torch.nn.Conv2d(c_in, c_out, k, s, p, bias=False),
            torch.nn.BatchNorm2d(c_out),
            torch.nn.ReLU(inplace=True)
        )
    def forward(self,x): return self.net(x)

class TinyUNet(torch.nn.Module):
    def __init__(self, in_ch=3, base=32):
        super().__init__()
        self.enc1 = torch.nn.Sequential(ConvBNReLU(in_ch, base), ConvBNReLU(base, base))
        self.enc2 = torch.nn.Sequential(torch.nn.MaxPool2d(2), ConvBNReLU(base, base*2), ConvBNReLU(base*2, base*2))
        self.enc3 = torch.nn.Sequential(torch.nn.MaxPool2d(2), ConvBNReLU(base*2, base*4), ConvBNReLU(base*4, base*4))
        self.bot  = torch.nn.Sequential(torch.nn.MaxPool2d(2), ConvBNReLU(base*4, base*8), ConvBNReLU(base*8, base*8))
        self.up3  = torch.nn.ConvTranspose2d(base*8, base*4, 2, 2)
        self.dec3 = torch.nn.Sequential(ConvBNReLU(base*8, base*4), ConvBNReLU(base*4, base*4))
        self.up2  = torch.nn.ConvTranspose2d(base*4, base*2, 2, 2)
        self.dec2 = torch.nn.Sequential(ConvBNReLU(base*4, base*2), ConvBNReLU(base*2, base*2))
        self.up1  = torch.nn.ConvTranspose2d(base*2, base, 2, 2)
        self.dec1 = torch.nn.Sequential(ConvBNReLU(base*2, base), ConvBNReLU(base, base))
        self.out  = torch.nn.Conv2d(base, 1, 1)
    def forward(self,x):
        e1 = self.enc1(x)
        e2 = self.enc2(e1)
        e3 = self.enc3(e2)
        b  = self.bot(e3)
        d3 = self.dec3(torch.cat([self.up3(b), e3], 1))
        d2 = self.dec2(torch.cat([self.up2(d3), e2], 1))
        d1 = self.dec1(torch.cat([self.up1(d2), e1], 1))
        y  = self.out(d1)
        return torch.sigmoid(y)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--ckpt', required=True)
    ap.add_argument('--size', nargs=2, type=int, default=[320,240])
    ap.add_argument('--max_depth_m', type=float, default=20.0)
    ap.add_argument('--split', default='val')
    ap.add_argument('--out', default='runs/depth_eval')
    args = ap.parse_args()

    dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    ds  = DepthPairs(args.data, args.split, tuple(args.size), args.max_depth_m)
    dl  = DataLoader(ds, batch_size=16, shuffle=False, num_workers=4, pin_memory=True)

    model = TinyUNet().to(dev)
    ck = torch.load(args.ckpt, map_location='cpu')
    model.load_state_dict(ck['model'], strict=True)
    model.eval()
    max_m = float(ck.get('max_depth_m', args.max_depth_m))

    total_abs=0.0; total_sq=0.0; total_cnt=0
    with torch.no_grad():
        for rgb, dep_n, dep_m in dl:
            rgb = rgb.to(dev)
            pred_n = model(rgb).cpu().squeeze(1).numpy()  # [B,H,W] 0..1
            gt_m   = dep_m.numpy()
            pr_m   = np.clip(pred_n*max_m, 0, max_m)
            mask   = (gt_m>0) & (gt_m<=max_m)
            cnt    = int(mask.sum())
            if cnt==0: continue
            diff = pr_m[mask]-gt_m[mask]
            total_abs += float(np.abs(diff).sum())
            total_sq  += float((diff*diff).sum())
            total_cnt += cnt

    mae = total_abs/max(1,total_cnt)
    rmse = math.sqrt(total_sq/max(1,total_cnt))
    Path(args.out).mkdir(parents=True, exist_ok=True)
    print(f"Eval split={args.split}  pixels={total_cnt}  MAE={mae:.3f} m  RMSE={rmse:.3f} m")
    with open(Path(args.out)/'metrics.txt','w') as f:
        f.write(f"MAE_m {mae:.6f}\nRMSE_m {rmse:.6f}\n")

if __name__=='__main__':
    main()
