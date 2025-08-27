#!/usr/bin/env python3
import argparse, csv, hashlib, os, random, sys, time
def sha256(p, chunk=1024*1024):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(chunk), b''): h.update(b)
    return h.hexdigest()
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--root', required=True)
    ap.add_argument('--csv', required=True)
    ap.add_argument('--sample', type=int, default=200, help='0=all')
    a=ap.parse_args()
    # load expected
    rows=[]
    with open(a.csv, newline='', encoding='utf-8') as f:
        r=csv.reader(f); next(r, None)
        for rel,size,dig in r: rows.append((rel,int(size),dig))
    if a.sample and a.sample < len(rows): rows=random.sample(rows, a.sample)
    t0=time.time(); bad=0; miss=0; done=0
    for rel,size,dig in rows:
        p=os.path.join(a.root, rel)
        if not os.path.exists(p): print("[MISS]", rel); miss+=1; continue
        if os.path.getsize(p)!=size: print("[SIZE]", rel); bad+=1; continue
        if sha256(p)!=dig: print("[HASH]", rel); bad+=1
        done+=1
        if done%200==0: print(f"[verify] {done}/{len(rows)} in {time.time()-t0:.1f}s")
    print(f"[DONE] checked={len(rows)} miss={miss} bad={bad}")
    sys.exit(1 if (miss or bad) else 0)
if __name__=='__main__': main()
