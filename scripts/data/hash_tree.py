#!/usr/bin/env python3
import argparse, hashlib, os, sys, time, json, csv
from concurrent.futures import ThreadPoolExecutor, as_completed

def sha256_file(path, chunk=1024*1024):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        while True:
            b=f.read(chunk)
            if not b: break
            h.update(b)
    return h.hexdigest()

def collect_files(root, includes):
    files=[]
    for inc in includes:
        p=os.path.join(root, inc)
        if os.path.isdir(p):
            for dirpath, _, filenames in os.walk(p):
                for fn in filenames:
                    files.append(os.path.join(dirpath, fn))
        elif os.path.isfile(p):
            files.append(p)
    return files

def load_existing(csv_path):
    if not os.path.exists(csv_path): return {}
    m={}
    with open(csv_path, newline='', encoding='utf-8') as f:
        r=csv.reader(f)
        header=next(r, None)
        for row in r:
            if len(row) < 3: continue
            rel, size, digest = row[0], int(row[1]), row[2]
            m[rel] = (size, digest)
    return m

def write_csv(path, rows):
    with open(path,'w',encoding='utf-8',newline='') as f:
        w=csv.writer(f)
        w.writerow(["relpath","size","sha256"])
        for rel, (size, digest) in rows:
            w.writerow([rel,size,digest])

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--root', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--include', nargs='*', default=['rgb','depth','index.csv','splits'])
    ap.add_argument('--resume', type=int, default=0)
    args=ap.parse_args()

    root=os.path.abspath(args.root)
    outdir=os.path.abspath(args.out)
    os.makedirs(outdir, exist_ok=True)
    csv_path=os.path.join(outdir,'checksums.csv')
    tmp_path=csv_path + ".tmp"
    sum_path=os.path.join(outdir,'SUMMARY.json')

    all_files = collect_files(root, args.include)
    if not all_files:
        print(f"[FAIL] nothing to hash under {args.include}", file=sys.stderr); sys.exit(1)

    # Build worklist (resume support)
    existing = load_existing(csv_path) if args.resume else {}
    todo=[]
    kept=[]
    for f in all_files:
        rel=os.path.relpath(f, root)
        size=os.path.getsize(f)
        if rel in existing and existing[rel][0] == size:
            kept.append((rel, (size, existing[rel][1])))
        else:
            todo.append((rel, f, size))

    print(f"[hash] root={root} files_total={len(all_files)} keep={len(kept)} todo={len(todo)} workers={args.workers}")
    t0=time.time(); done=0
    results=[]

    # Hash TODO in parallel
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        futs={ex.submit(sha256_file, f): (rel, size) for (rel, f, size) in todo}
        for fut in as_completed(futs):
            rel,size = futs[fut]
            try:
                digest=fut.result()
            except Exception as e:
                print(f"[ERR] {rel}: {e}", file=sys.stderr); continue
            results.append((rel,(size,digest)))
            done+=1
            if done%2000==0 or done==len(todo):
                dt=time.time()-t0
                print(f"[hash] {done}/{len(todo)} (+{2000 if done%2000==0 else 0} in {dt:.1f}s)")

    # Merge kept + new, sort stable
    merged = kept + results
    merged.sort(key=lambda x: x[0])

    # Write atomically
    write_csv(tmp_path, merged)
    os.replace(tmp_path, csv_path)

    summary={
        "root": root,
        "files_hashed_total": len(all_files),
        "files_kept_from_resume": len(kept),
        "files_newly_hashed": len(results),
        "outputs": {"checksums_csv": csv_path}
    }
    with open(sum_path,'w') as f: json.dump(summary,f,indent=2)
    print(f"[write] {csv_path}\n[write] {sum_path}\n[DONE] hashing complete/resumed")

if __name__=="__main__":
    main()
