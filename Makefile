SHELL := /bin/bash

# Defaults (override on CLI): make target DATASET=... EPOCHS=...
DATASET ?= datasets/sim_2025-08-22_122259
MAX_DEPTH ?= 20.0
EPOCHS ?= 12

CKPT := runs/depth_tinyunet/_ckpt_ep$(EPOCHS).pt
ONNX ?= models/exported/_tinyunet_depth_e$(EPOCHS).onnx

.PHONY: depth_smoke depth_offline_aug depth_train_eval depth_export_onnx \
        depth_demo depth_live_mae depth_verify depth_hash depth_verify_hashes \
        depth_dataset_readme depth_report depth_stamp

# 1) Quick dataset smoke test (writes manifest.json)
depth_smoke:
	. .venv/bin/activate && python scripts/data/smoke_depth_dataset.py --dataset_root '$(DATASET)' --max_depth_m $(MAX_DEPTH) --out '$(DATASET)/manifest.json'

# 2) Offline augmentation (writes $(DATASET)_aug)
depth_offline_aug:
	. .venv/bin/activate && python scripts/data/augment_depth_dataset.py --in '$(DATASET)' --out '$(DATASET)_aug' --copies 1 --max_depth_m $(MAX_DEPTH)

# 3) Train → Eval (val+test) → Export → Report → Stamp  ✅ one command
depth_train_eval:
	. .venv/bin/activate && python ns_train_depth_baseline.py --data '$(DATASET)' --epochs $(EPOCHS) --size 320 240 --out runs/depth_tinyunet | tee "runs/depth_tinyunet/train_e$(EPOCHS).log"
	. .venv/bin/activate && python ns_eval_depth.py --data '$(DATASET)' --ckpt '$(CKPT)' --max_depth_m $(MAX_DEPTH) --out runs/depth_eval | tee "runs/depth_eval/eval_e$(EPOCHS)_val.log"
	. .venv/bin/activate && python ns_eval_depth.py --data '$(DATASET)' --ckpt '$(CKPT)' --split test --max_depth_m $(MAX_DEPTH) --out runs/depth_eval | tee "runs/depth_eval/eval_e$(EPOCHS)_test.log"
	. .venv/bin/activate && python ns_export_onnx.py --ckpt '$(CKPT)' --out '$(ONNX)'
	$(MAKE) depth_report DATASET='$(DATASET)' EPOCHS='$(EPOCHS)'
	$(MAKE) depth_stamp  EPOCHS='$(EPOCHS)'

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

# 8) Hash the dataset (resume-safe)
depth_hash:
	. .venv/bin/activate && python scripts/data/hash_tree.py \
	  --root '$(DATASET)' \
	  --out  '$(DATASET)/checksums' \
	  --workers $${WORKERS:-12} \
	  --resume $${RESUME:-1}

# 9) Verify hashes (SAMPLE=0 for full check)
depth_verify_hashes:
	. .venv/bin/activate && python scripts/data/verify_hashes.py \
	  --root '$(DATASET)' \
	  --csv  '$(DATASET)/checksums/checksums.csv' \
	  --sample $${SAMPLE:-200}

# 10) Generate dataset README (Item 41)
depth_dataset_readme:
	. .venv/bin/activate && python scripts/data/gen_dataset_readme.py --root '$(DATASET)'

# 11) Generate the depth run report (uses scripts/report_depth_run.py)
depth_report:
	. .venv/bin/activate && python scripts/report_depth_run.py \
	  --data   '$(DATASET)' \
	  --ckpt   'runs/depth_tinyunet/_ckpt_ep$(EPOCHS).pt' \
	  --onnx   'models/exported/_tinyunet_depth_e$(EPOCHS).onnx' \
	  --metrics 'runs/depth_eval/metrics.txt' \
	  --out    'runs/depth_eval/report.md'

# 12) Stamp artifacts into model card (Item 41)
depth_stamp:
	. .venv/bin/activate && python scripts/tools/stamp_artifacts.py \
	  --ckpt 'runs/depth_tinyunet/_ckpt_ep$(EPOCHS).pt' \
	  --onnx 'models/exported/_tinyunet_depth_e$(EPOCHS).onnx' \
	  --card 'runs/depth_eval/model_card.md'

.PHONY: depth_report
depth_report:
	. .venv/bin/activate && python scripts/report_depth_run.py \
	  --data '$(DATASET)' \
	  --onnx '$(ONNX)' \
	  --metrics runs/depth_eval/metrics.txt \
	  --out runs/depth_eval/report.md
.PHONY: depth_train_eval_all
depth_train_eval_all:
. .venv/bin/activate && python ns_train_depth_baseline.py --data '$(DATASET)' --epochs $(EPOCHS) --size 320 240 --out runs/depth_tinyunet
. .venv/bin/activate && python ns_eval_depth.py --data '$(DATASET)' --ckpt '$(CKPT)' --max_depth_m $(MAX_DEPTH) --out runs/depth_eval
. .venv/bin/activate && python ns_export_onnx.py --ckpt '$(CKPT)' --out '$(ONNX)'
. .venv/bin/activate && python scripts/tools/stamp_artifacts.py --ckpt '$(CKPT)' --onnx '$(ONNX)' --card 'runs/depth_eval/model_card.md'
. .venv/bin/activate && python scripts/report_depth_run.py --data '$(DATASET)' --onnx '$(ONNX)' --metrics runs/depth_eval/metrics.txt --out runs/depth_eval/report.md

.PHONY: depth_train_eval_all
depth_train_eval_all:
	. .venv/bin/activate && python ns_train_depth_baseline.py --data '$(DATASET)' --epochs $(EPOCHS) --size 320 240 --out runs/depth_tinyunet
	. .venv/bin/activate && python ns_eval_depth.py          --data '$(DATASET)' --ckpt '$(CKPT)' --max_depth_m $(MAX_DEPTH) --out runs/depth_eval
	. .venv/bin/activate && python ns_export_onnx.py         --ckpt '$(CKPT)' --out '(ONNX)'
	. .venv/bin/activate && python scripts/tools/stamp_artifacts.py --ckpt '$(CKPT)' --onnx '$(ONNX)' --card 'runs/depth_eval/model_card.md'
	. .venv/bin/activate && python scripts/report_depth_run.py --data '$(DATASET)' --onnx '$(ONNX)' --metrics runs/depth_eval/metrics.txt --out runs/depth_eval/report.md
