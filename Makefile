SHELL := /bin/bash

# Defaults (override on CLI): make target DATASET=... EPOCHS=...
DATASET ?= datasets/sim_2025-08-22_122259
MAX_DEPTH ?= 20.0
EPOCHS ?= 12

CKPT := runs/depth_tinyunet/_ckpt_ep$(EPOCHS).pt
ONNX ?= models/exported/_tinyunet_depth_e$(EPOCHS).onnx

.PHONY: depth_smoke depth_offline_aug depth_train_eval depth_export_onnx \
        depth_demo depth_live_mae depth_verify depth_report depth_demo_real

# 1) Quick dataset smoke test (writes manifest.json)
depth_smoke:
	. .venv/bin/activate && python scripts/data/smoke_depth_dataset.py --dataset_root '$(DATASET)' --max_depth_m $(MAX_DEPTH) --out '$(DATASET)/manifest.json'

# 2) Offline augmentation (writes $(DATASET)_aug)
depth_offline_aug:
	. .venv/bin/activate && python scripts/data/augment_depth_dataset.py --in '$(DATASET)' --out '$(DATASET)_aug' --copies 1 --max_depth_m $(MAX_DEPTH)

# 3) Train → Eval → Export ONNX in one go
depth_train_eval:
	. .venv/bin/activate && python ns_train_depth_baseline.py --data '$(DATASET)' --epochs $(EPOCHS) --size 320 240 --out runs/depth_tinyunet | tee "runs/depth_tinyunet/train_e$(EPOCHS).log"
	. .venv/bin/activate && python ns_eval_depth.py --data '$(DATASET)' --ckpt '$(CKPT)' --max_depth_m $(MAX_DEPTH) --out runs/depth_eval | tee "runs/depth_eval/eval_e$(EPOCHS)_val.log"
	. .venv/bin/activate && python ns_eval_depth.py --data '$(DATASET)' --ckpt '$(CKPT)' --split test --max_depth_m $(MAX_DEPTH) --out runs/depth_eval | tee "runs/depth_eval/eval_e$(EPOCHS)_test.log"
	. .venv/bin/activate && python ns_export_onnx.py --ckpt '$(CKPT)' --out '$(ONNX)'

# 4) Export only (when retracing a checkpoint)
depth_export_onnx:
	. .venv/bin/activate && python ns_export_onnx.py --ckpt '$(CKPT)' --out '$(ONNX)'

# 5) Sim demo (bag + ONNX + RViz)
depth_demo:
	bash scripts/ns_run_depth_demo.sh

# 6) Live MAE helper
depth_live_mae:
	bash scripts/ns_run_depth_live_mae.sh

# 7) Verify dataset manifest/splits quickly
depth_verify:
	. .venv/bin/activate && python scripts/data/verify_depth_manifest.py --dataset_root '$(DATASET)'

# 8) Aggregate eval report from logs
depth_report:
	. .venv/bin/activate && python scripts/gen_depth_report.py --eval_dir runs/depth_eval --data '$(DATASET)' --ckpt '$(CKPT)' --onnx '$(ONNX)' --out runs/depth_eval/report.md

# 9) Real-bag demo wrapper (pass BAG=/path/to/real_bag_dir)
depth_demo_real:
	BAG='$(BAG)' ONNX='$(ONNX)' bash scripts/ns_run_depth_demo_real.sh
