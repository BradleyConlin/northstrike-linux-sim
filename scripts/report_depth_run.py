#!/usr/bin/env python3
import argparse, json, os, re, time
def read_txt(p): 
    try: return open(p,'r',encoding='utf-8').read().strip()
    except: return ""
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--ckpt', required=True)
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--metrics', required=True)   # runs/depth_eval/metrics.txt
    ap.add_argument('--model_card', default='runs/depth_eval/model_card.md')
    ap.add_argument('--out', default='runs/depth_eval/report.md')
    a=ap.parse_args()

    lines=[]
    lines.append("# Depth run report")
    lines.append(f"_generated: {time.strftime('%Y-%m-%d %H:%M:%S')}_\n")

    # Metrics
    m=read_txt(a.metrics)
    mae=rmse=None
    mm=re.search(r"MAE=([0-9.]+) m.*RMSE=([0-9.]+) m", m)
    if mm: mae,rmse=mm.group(1),mm.group(2)
    lines.append("## Metrics")
    lines.append("```\n"+m+"\n```")
    if mae and rmse: lines.append(f"- Parsed: **MAE={mae} m**, **RMSE={rmse} m**")

    # Artifacts
    lines.append("\n## Artifacts")
    for label,path in (("Dataset",a.data),("Checkpoint",a.ckpt),("ONNX",a.onnx),("Metrics",a.metrics)):
        lines.append(f"- {label}: `{os.path.abspath(path)}`")

    # Integrity summaries if present
    summ=os.path.join(a.data,'integrity_summary.json')
    if os.path.exists(summ):
        js=json.load(open(summ))
        lines.append("\n## Dataset integrity")
        lines.append("```json\n"+json.dumps(js,indent=2)+"\n```")

    # Model card (inline)
    mc=read_txt(a.model_card)
    if mc:
        lines.append("\n## Model card")
        lines.append(mc)

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    open(a.out,'w',encoding='utf-8').write("\n".join(lines)+"\n")
    print(f"[write] {a.out}")
if __name__=="__main__": main()
