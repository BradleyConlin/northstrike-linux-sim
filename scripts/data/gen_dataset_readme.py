#!/usr/bin/env python3
import argparse, os, json, time, subprocess
def read_json(p): 
    try: return json.load(open(p))
    except: return {}
def human_bytes(n):
    for u in ['B','KB','MB','GB','TB']:
        if n < 1024 or u=='TB': return f"{n:.1f} {u}"
        n/=1024
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--root', required=True)
    ap.add_argument('--out', default=None)
    a=ap.parse_args()
    root=os.path.abspath(a.root)
    out=a.out or os.path.join(root,'DATASET_README.md')

    integ=read_json(os.path.join(root,'integrity_summary.json'))
    sums =read_json(os.path.join(root,'checksums','SUMMARY.json'))
    idx  =os.path.join(root,'index.csv')
    splits=os.path.join(root,'splits')
    counts=integ.get('splits', {})
    bytes_total=sums.get('bytes_total', 0)
    sz=human_bytes(bytes_total) if bytes_total else "n/a"

    # derive version label
    ver=os.path.basename(root)

    lines=[]
    lines.append(f"# Dataset: {ver}")
    lines.append(f"_generated: {time.strftime('%Y-%m-%d %H:%M:%S')}_\n")
    lines.append("## Contents")
    lines.append(f"- root: `{root}`")
    lines.append(f"- index: `{idx}`")
    lines.append(f"- splits: `{splits}`")
    lines.append(f"- files: rgb+depth = {integ.get('index_rows','n/a')}")
    lines.append(f"- split counts: {counts}")
    lines.append(f"- total size (from checksums): {sz}\n")
    lines.append("## Integrity")
    lines.append("- Verified with `scripts/data/verify_depth_manifest.py` → `integrity_summary.json`.")
    if bytes_total:
        lines.append("- Full-file SHA256 at `checksums/checksums.csv` (resume-safe).")
        lines.append("  - Quick verify sample: `python scripts/data/verify_hashes.py --root ROOT --csv ROOT/checksums/checksums.csv --sample 200`")
        lines.append("  - Full verify: `python scripts/data/verify_hashes.py --root ROOT --csv ROOT/checksums/checksums.csv --sample 0`")
    else:
        lines.append("- Checksums not found yet. Generate with `scripts/data/hash_tree.py --root ROOT --out ROOT/checksums --workers 12 --resume 1`.")
    lines.append("\n## Repro commands")
    lines.append("```bash")
    lines.append(f"# Smoke & integrity")
    lines.append(f"python scripts/data/verify_depth_manifest.py --dataset_root '{root}'")
    if bytes_total:
        lines.append(f"python scripts/data/verify_hashes.py --root '{root}' --csv '{root}/checksums/checksums.csv' --sample 200")
    lines.append("```")
    open(out,'w',encoding='utf-8').write("\n".join(lines)+"\n")
    print(f"[write] {out}")
if __name__=='__main__': main()
