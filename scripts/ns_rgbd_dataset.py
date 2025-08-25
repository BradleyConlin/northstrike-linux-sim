#!/usr/bin/env python3
import csv
from pathlib import Path
import numpy as np
import cv2
import torch
from torch.utils.data import Dataset

class RGBDPairDataset(Dataset):
    """
    Loads paired RGB/Depth using your manifests/splits/*.csv.
    - Resizes RGB with AREA
    - Resizes depth with NEAREST (keeps values intact)
    - Normalizes RGB to [0,1], depth to [0,1] by (meters / max_depth_m)
    """
    def __init__(self, root, split="train", size=(320, 240), max_depth_m=20.0, limit=None, augment=True):
        self.root = Path(root)
        self.size = tuple(size)  # (W, H)
        self.max_depth_m = float(max_depth_m)
        self.augment = augment and split == "train"

        csv_path = self.root / "splits" / f"{split}.csv"
        rows = list(csv.DictReader(open(csv_path)))
        self.rows = rows[:int(limit)] if limit else rows
        self.rng = np.random.RandomState(1234)

    def __len__(self):
        return len(self.rows)

    def _read_rgb(self, rel):
        W, H = self.size
        img = cv2.imread(str(self.root / rel))
        if img is None:
            img = np.zeros((H, W, 3), np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
        img = (img.astype(np.float32) / 255.0)
        return torch.from_numpy(img).permute(2, 0, 1)  # CxHxW

    def _read_depth(self, rel):
        W, H = self.size
        dep = cv2.imread(str(self.root / rel), cv2.IMREAD_UNCHANGED)
        if dep is None:
            dep = np.zeros((H, W), np.uint16)
        if dep.dtype != np.uint16:
            dep = dep.astype(np.uint16)
        dep = cv2.resize(dep, (W, H), interpolation=cv2.INTER_NEAREST)
        dep_m = dep.astype(np.float32) / 1000.0  # mm -> m
        dep_norm = np.clip(dep_m / self.max_depth_m, 0.0, 1.0)
        return torch.from_numpy(dep_norm)[None, ...]  # 1xHxW

    def __getitem__(self, idx):
        r = self.rows[idx]
        rgb = self._read_rgb(r["rgb_path"])
        depth = self._read_depth(r["depth_path"])

        if self.augment and self.rng.rand() < 0.5:
            rgb = torch.flip(rgb, dims=[2])
            depth = torch.flip(depth, dims=[2])
        return rgb, depth

RgbdDataset = RGBDPairDataset

