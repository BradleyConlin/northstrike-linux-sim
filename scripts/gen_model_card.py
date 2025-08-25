#!/usr/bin/env python3
import argparse, subprocess, time
from pathlib import Path

def read_metrics(p):
    m = {}
    if Path(p).exists():
        for line in Path(p).read_text().splitlines():
            parts = line.strip().split()
            if len(parts)==2: m[parts[0]] = parts[1]
    return m

def sh(cmd):
    try: return subprocess.check_output(cmd, shell=True, text=True).strip()
    except: return "unknown"

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--ckpt', required=True)
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--metrics', default='runs/depth_eval/metrics.txt')
    ap.add_argument('--out', default='runs/depth_eval/model_card.md')
    args = ap.parse_args()

    git = sh('git rev-parse --short HEAD')
    when = time.strftime('%Y-%m-%d %H:%M:%S')
    m = read_metrics(args.metrics)
    mae = m.get('MAE_m', 'n/a'); rmse = m.get('RMSE_m', 'n/a')

    md = f"""# Northstrike Depth — TinyUNet

**When:** {when}  
**Commit:** {git}

## Data
- Dataset: `{args.data}`

## Training
- Checkpoint: `{args.ckpt}`

## Metrics (val)
- MAE: {mae} m
- RMSE: {rmse} m

## Deployment
- ONNX: `{args.onnx}` (also symlinked as dataset default)

"""
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(md)
    print(f"Wrote {args.out}")

if __name__ == '__main__':
    main()
