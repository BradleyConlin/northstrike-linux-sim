#!/usr/bin/env python3
import argparse, os, csv, random, time, pathlib
import numpy as np
import cv2
import onnxruntime as ort

def read_index_map(index_csv):
    m={}
    with open(index_csv, newline='') as f:
        r=csv.reader(f)
        for row in r:
            if not row: continue
            if row[0].strip().lower()=='rgb':  # header
                continue
            if len(row)>=2: m[row[0]] = row[1]
    return m

def read_split_rgb_list(split_csv):
    lst=[]
    with open(split_csv, newline='') as f:
        r=csv.reader(f)
        for row in f:
            row=row.strip()
            if not row: continue
            if row.lower()=='rgb':  # header
                continue
            lst.append(row)
    return lst

def make_rows(data_root):
    idx_csv=os.path.join(data_root,'index.csv')
    test_csv=os.path.join(data_root,'splits','test.csv')
    val_csv =os.path.join(data_root,'splits','val.csv')
    idx_map=read_index_map(idx_csv)
    rows=[]
    if os.path.exists(test_csv):
        lst=read_split_rgb_list(test_csv)
    elif os.path.exists(val_csv):
        lst=read_split_rgb_list(val_csv)
    else:
        return [(k, v) for k,v in idx_map.items()]
    if not lst:
        return [(k, v) for k,v in idx_map.items()]
    for rgb in lst:
        if rgb in idx_map:
            rows.append((rgb, idx_map[rgb]))
    return rows

def load_rgb(path, size_wh):
    w,h=size_wh
    img=cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None: raise RuntimeError(f"fail read {path}")
    img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img=cv2.resize(img, (w,h), interpolation=cv2.INTER_AREA)
    return img

def load_depth_mm(path, size_wh):
    w,h=size_wh
    d=cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if d is None: raise RuntimeError(f"fail read {path}")
    if len(d.shape)==3: d=cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
    d=cv2.resize(d, (w,h), interpolation=cv2.INTER_NEAREST)
    if d.dtype!=np.uint16: d=d.astype(np.uint16)
    return d

def colorize_depth_mm(depth_mm, max_mm):
    d=np.clip(depth_mm.astype(np.float32), 0, max_mm) / max_mm
    d=(d*255.0).astype(np.uint8)
    return cv2.applyColorMap(d, cv2.COLORMAP_TURBO)

def save_panel(rgb, pred_mm, gt_mm, max_mm, out_png):
    rgb_disp=cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    pred_col=colorize_depth_mm(pred_mm, max_mm)
    gt_col  =colorize_depth_mm(gt_mm,   max_mm)
    panel=cv2.hconcat([rgb_disp, pred_col, gt_col])
    for i,txt in enumerate(["RGB","Pred","GT"]):
        x=i*rgb.shape[1]+8; y=20
        cv2.putText(panel, txt, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2, cv2.LINE_AA)
        cv2.putText(panel, txt, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
    cv2.imwrite(out_png, panel)

def run_samples_onnx(data_root, onnx_path, out_dir, max_depth_m=20.0, num=4, size_wh=(320,240)):
    rows = make_rows(data_root)
    if len(rows)==0:
        raise RuntimeError("no rows found after joining split list with index.csv")

    # ONNX session (CPU provider is fine)
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    inp_name = sess.get_inputs()[0].name  # usually "rgb"
    mm=int(max_depth_m*1000)
    os.makedirs(out_dir, exist_ok=True)

    sample = random.sample(rows, min(num, len(rows)))
    for i,(rgb_rel, depth_rel) in enumerate(sample, start=1):
        rpath=os.path.join(data_root, rgb_rel)
        dpath=os.path.join(data_root, depth_rel)

        rgb=load_rgb(rpath, size_wh)
        gt =load_depth_mm(dpath, size_wh)

        x=rgb.astype(np.float32)/255.0
        x=np.transpose(x, (2,0,1))[None, ...]  # NCHW
        pred = sess.run(None, {inp_name: x})[0][0,0]  # 0..1
        pred_mm=(pred*mm).astype(np.uint16)

        save_panel(rgb, pred_mm, gt, mm, os.path.join(out_dir, f"sample_{i}.png"))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--ckpt', required=False)        # kept for CLI compatibility; unused
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--metrics', default='runs/depth_eval/metrics.txt')
    ap.add_argument('--out', required=True)
    ap.add_argument('--max_depth_m', type=float, default=20.0)
    ap.add_argument('--num_samples', type=int, default=4)
    ap.add_argument('--width', type=int, default=320)
    ap.add_argument('--height', type=int, default=240)
    a=ap.parse_args()

    samples_dir=os.path.join(os.path.dirname(a.out),'samples')
    run_samples_onnx(a.data, a.onnx, samples_dir, max_depth_m=a.max_depth_m,
                     num=a.num_samples, size_wh=(a.width,a.height))

    lines=[]
    lines.append(f"# Depth Run Report\n_generated: {time.strftime('%Y-%m-%d %H:%M:%S')}_\n")
    lines.append(f"**Data:** `{a.data}`  \n**ONNX:** `{a.onnx}`  \n**Max depth:** {a.max_depth_m} m\n")
    if os.path.exists(a.metrics):
        try:
            txt=open(a.metrics,'r').read().strip()
            if txt: lines.append("## Metrics\n```\n"+txt+"\n```\n")
        except: pass
    lines.append("## Samples (RGB | Pred | GT)\n")
    for i in range(a.num_samples):
        p=os.path.join('samples', f"sample_{i+1}.png")
        if os.path.exists(os.path.join(os.path.dirname(a.out), p)):
            lines.append(f"![sample {i+1}]({p})")
    pathlib.Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    open(a.out,'w',encoding='utf-8').write("\n".join(lines)+"\n")
    print(f"[write] {a.out}\n[write] {samples_dir}/*.png")

if __name__=='__main__':
    main()
