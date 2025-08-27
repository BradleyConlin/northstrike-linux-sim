# Dataset: sim_2025-08-22_122259_aug
_generated: 2025-08-26 19:07:28_

## Contents
- root: `/home/brad/dev/northstrike-linux-sim/datasets/sim_2025-08-22_122259_aug`
- index: `/home/brad/dev/northstrike-linux-sim/datasets/sim_2025-08-22_122259_aug/index.csv`
- splits: `/home/brad/dev/northstrike-linux-sim/datasets/sim_2025-08-22_122259_aug/splits`
- files: rgb+depth = 278929
- split counts: {'train': 247936, 'val': 15496, 'test': 15497}
- total size (from checksums): n/a

## Integrity
- Verified with `scripts/data/verify_depth_manifest.py` → `integrity_summary.json`.
- Checksums not found yet. Generate with `scripts/data/hash_tree.py --root ROOT --out ROOT/checksums --workers 12 --resume 1`.

## Repro commands
```bash
# Smoke & integrity
python scripts/data/verify_depth_manifest.py --dataset_root '/home/brad/dev/northstrike-linux-sim/datasets/sim_2025-08-22_122259_aug'
```
