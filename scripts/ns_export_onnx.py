#!/usr/bin/env python3
import argparse, torch
from pathlib import Path
from ns_train_depth_baseline import TinyUNet

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--size", type=int, nargs=2, default=(320,240), help="W H")
    args = ap.parse_args()

    W,H = args.size
    model = TinyUNet()
    ck = torch.load(args.ckpt, map_location="cpu")
    model.load_state_dict(ck["model"]); model.eval()

    x = torch.randn(1,3,H,W)  # NCHW
    outp = Path(args.out)
    outp.parent.mkdir(parents=True, exist_ok=True)

    torch.onnx.export(
        model, x, str(outp),
        input_names=["rgb"], output_names=["depth_norm01"],
        dynamic_axes={"rgb": {0: "N", 2: "H", 3: "W"},
                      "depth_norm01": {0: "N", 2: "H", 3: "W"}},
        opset_version=17
    )
    print("Exported ONNX ->", outp, "| input N,3,H,W in [0,1], output 1xHxW in [0,1]")

if __name__ == "__main__":
    main()
