#!/usr/bin/env python3
import argparse, hashlib, os, sys, time, json
from concurrent.futures import ThreadPoolExecutor, as_completed

def sha256_file(path, chunk=1024*1024):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        while True:
            b=f.read(chunk)
            if not b: break
            h.update(b)
    return h.hexdigest()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--root', required=True, help='Directory to hash (e.g., dataset root)')
    ap.add_argument('--out', required=True, help='Output directory for checksums')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--include', nargs='*', default=['rgb','depth','index.csv','splits'], help='subpaths to include')
    args=ap.parse_args()

    root=os.path.abspath(args.root)
    outdir=os.path.abspath(args.out)
    os.makedirs(outdir, exist_ok=True)
    csv_path=os.path.join(outdir,'checksums.csv')
    sum_path=os.path.join(outdir,'SUMMARY.json')

    # Collect paths
    files=[]
    for inc in args.include:
        p=os.path.join(root, inc)
        if os.path.isdir(p):
            for dirpath, _, filenames in os.walk(p):
                for fn in filenames:
                    files.append(os.path.join(dirpath, fn))
        elif os.path.isfile(p):
            files.append(p)

    total=len(files)
    if total==0:
        print(f"[FAIL] nothing to hash under {args.include}", file=sys.stderr); sys.exit(1)

    t0=time.time()
    done=0
    sizes=[]
    print(f"[hash] root={root} files={total} workers={args.workers}")
    with open(csv_path,'w',encoding='utf-8') as csvf:
        csvf.write("relpath,size,sha256\n")
        with ThreadPoolExecutor(max_workers=args.workers) as ex:
            futs={ex.submit(sha256_file, f): f for f in files}
            for fut in as_completed(futs):
                f=futs[fut]
                try:
                    digest=fut.result()
                except Exception as e:
                    print(f"[ERR] {f}: {e}", file=sys.stderr); continue
                rel=os.path.relpath(f, root)
                sz=os.path.getsize(f)
                sizes.append(sz)
                csvf.write(f"{rel},{sz},{digest}\n")
                done+=1
                if done%2000==0 or done==total:
                    dt=time.time()-t0
                    print(f"[hash] {done}/{total} (+{2000 if done%2000==0 else 0} in {dt:.1f}s)")

    summary={
        "root": root,
        "files_hashed": total,
        "bytes_total": int(sum(sizes)),
        "outputs": {"checksums_csv": csv_path}
    }
    with open(sum_path,'w') as f: json.dump(summary,f,indent=2)
    print(f"[write] {csv_path}\n[write] {sum_path}\n[DONE] hashing complete")

if __name__=="__main__":
    main()
