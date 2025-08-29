# Northstrike Depth — TinyUNet

**When:** 2025-08-25 16:45:36  
**Commit:** 7f570d8

## Data
- Dataset: `/home/brad/dev/northstrike-linux-sim/datasets/sim_2025-08-22_122259_aug`

## Training
- Checkpoint: `/home/brad/dev/northstrike-linux-sim/runs/depth_tinyunet/_ckpt_ep12.pt`

## Metrics (val)
- MAE: 0.005145 m
- RMSE: 0.014710 m

## Deployment
- ONNX: `/home/brad/dev/northstrike-linux-sim/models/exported/_tinyunet_depth_e12.onnx` (also symlinked as dataset default)


### Data Integrity
- Verified with `scripts/data/verify_depth_manifest.py` → `integrity_summary.json`.
- Dataset checksums stored at `datasets/sim_2025-08-22_122259_aug/checksums/`.
- Reproduce eval: `make depth_verify && python ns_eval_depth.py --data DATASET --ckpt CKPT --max_depth_m 20.0 --out runs/depth_eval`

### Artifacts (SHA256)

```
678ba34526567dee7691b8d97563ce538ea7445dfff51057be88298d44a5f22d  runs/depth_tinyunet/_ckpt_ep12.pt
0dc92499d3c0b3110d0065f4e6a79add82c7d02d7c873e1cffaf3df11e85b62e  models/exported/_tinyunet_depth_e12.onnx

```

### Artifacts (SHA256) — 2025-08-26 19:07:18

```
678ba34526567dee7691b8d97563ce538ea7445dfff51057be88298d44a5f22d  runs/depth_tinyunet/_ckpt_ep12.pt
0dc92499d3c0b3110d0065f4e6a79add82c7d02d7c873e1cffaf3df11e85b62e  models/exported/_tinyunet_depth_e12.onnx
```

### Artifacts (SHA256) — 2025-08-26 19:07:29

```
678ba34526567dee7691b8d97563ce538ea7445dfff51057be88298d44a5f22d  runs/depth_tinyunet/_ckpt_ep12.pt
0dc92499d3c0b3110d0065f4e6a79add82c7d02d7c873e1cffaf3df11e85b62e  models/exported/_tinyunet_depth_e12.onnx
```

### Artifacts (SHA256) — 2025-08-27 18:14:48

```
ea47f19a4623f3afc8bfde5e1a7ed6bca6fc43dc801d98534d4a085db274b15c  runs/depth_tinyunet/_ckpt_ep24.pt
28647541639f4ef4f61b7db591be968fd90206000f25a26472cc51091d9f324c  models/exported/_tinyunet_depth_e24.onnx
```
