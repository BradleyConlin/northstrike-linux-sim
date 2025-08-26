#!/usr/bin/env python3
import argparse, csv, json, os, random, sys
def die(msg, code=1): print(f"[FAIL] {msg}", file=sys.stderr); sys.exit(code)

def read_index(csv_path):
    rows=[]
    with open(csv_path, newline='') as f:
        r=csv.reader(f)
        header=next(r, None)
        if not header or header[:2] != ['rgb','depth']:
            die(f"index header must be: rgb,depth (got {header})")
        for i,row in enumerate(r,1):
            if len(row) < 2: 
                print(f"[warn] bad index row {i}: {row}", file=sys.stderr); 
                continue
            rows.append((row[0].strip(), row[1].strip()))
    return rows

def _stem(p): return os.path.splitext(os.path.basename(p))[0]

def load_split_ids(path):
    ids=set()
    with open(path, newline='') as f:
        peek=f.read(512); f.seek(0)
        if ',' in peek:
            r=csv.reader(f)
            header=next(r, None)
            h0=(header[0] if header else '').strip().lower().lstrip('\ufeff')
            # treat as header if it looks like a column name
            if h0 not in ('rgb','image','id','rgb_path'):
                # header was actually data
                if header and header[0].strip():
                    ids.add(_stem(header[0].strip()))
            for row in r:
                if not row: continue
                ids.add(_stem(row[0].strip()))
        else:
            for line in (ln.strip() for ln in f if ln.strip()):
                ids.add(_stem(line))
    # guard against stray tokens that are column names
    ids.discard('rgb'); ids.discard('depth'); ids.discard('image'); ids.discard('id')
    return ids

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--dataset_root', required=True)
    ap.add_argument('--sample', type=int, default=200)
    a=ap.parse_args()

    root=a.dataset_root
    idx=os.path.join(root,'index.csv')
    rgb_dir=os.path.join(root,'rgb'); dep_dir=os.path.join(root,'depth'); spl_dir=os.path.join(root,'splits')
    for p in [root, idx, rgb_dir, dep_dir, spl_dir]:
        if not os.path.exists(p): die(f"missing: {p}")

    rows=read_index(idx); n=len(rows)
    print(f"[ok] index rows: {n}")

    sample=min(a.sample,n)
    for r,d in random.sample(rows, sample):
        if not (os.path.exists(os.path.join(root,r)) and os.path.exists(os.path.join(root,d))):
            die("sampled files missing (see index.csv)")
    print(f"[ok] sampled file existence: {sample}/{sample}")

    tr=load_split_ids(os.path.join(spl_dir,'train.csv'))
    va=load_split_ids(os.path.join(spl_dir,'val.csv'))
    te=load_split_ids(os.path.join(spl_dir,'test.csv'))
    print(f"[ok] splits: {{'train': {len(tr)}, 'val': {len(va)}, 'test': {len(te)}}}")

    all_ids=set(_stem(r) for r,_ in rows)
    problems={}
    for name,ids in (('train',tr),('val',va),('test',te)):
        missing=[i for i in ids if i not in all_ids]
        if missing: problems[name]=missing[:10]
    if problems:
        print("[FAIL] split IDs not found in index for:", ", ".join(problems.keys()))
        for k,v in problems.items(): print(f"  {k} (first 10): {v}")
        sys.exit(4)
    print("[ok] split IDs match index")

    # quick counts + summary file
    try:
        rgb_files=sum(1 for _ in os.scandir(rgb_dir))
        dep_files=sum(1 for _ in os.scandir(dep_dir))
        print(f"[ok] dir counts: rgb={rgb_files}, depth={dep_files}")
    except Exception as e:
        print(f"[warn] count dirs: {e}")
    summary={'root':root,'index_rows':n,'splits':{'train':len(tr),'val':len(va),'test':len(te)}}
    with open(os.path.join(root,'integrity_summary.json'),'w') as f: json.dump(summary,f,indent=2)
    print(f"[write] {os.path.join(root,'integrity_summary.json')}")
    print("[DONE] verify_depth_manifest OK")

if __name__=='__main__': import random, os; main()
