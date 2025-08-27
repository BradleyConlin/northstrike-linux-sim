#!/usr/bin/env python3
import argparse, hashlib, os, time

def sha256(path, chunk=1024*1024):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for b in iter(lambda: f.read(chunk), b''):
            h.update(b)
    return h.hexdigest()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt', required=True)
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--card', required=True)
    ap.add_argument('--out_txt', default=None)
    args = ap.parse_args()

    ck_digest = sha256(args.ckpt)
    onnx_digest = sha256(args.onnx)
    out_txt = args.out_txt or os.path.join(os.path.dirname(args.card), 'artifacts.sha256')

    # Write a plain .sha256 file
    with open(out_txt, 'w') as f:
        f.write(f"{ck_digest}  {args.ckpt}\n")
        f.write(f"{onnx_digest}  {args.onnx}\n")

    # Append a dated block to the model card
    with open(args.card, 'a') as f:
        f.write("\n### Artifacts (SHA256) — " + time.strftime('%Y-%m-%d %H:%M:%S') + "\n\n```\n")
        f.write(f"{ck_digest}  {args.ckpt}\n")
        f.write(f"{onnx_digest}  {args.onnx}\n")
        f.write("```\n")

    print(f"[write] {out_txt}")
    print(f"[update] {args.card}")

if __name__ == '__main__':
    main()
