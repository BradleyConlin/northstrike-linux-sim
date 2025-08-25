.PHONY: help setup sim sim-headless stop hover train eval tb clean

# Defaults (override: make train GUI=false STEPS=5000)
GUI      ?= true
STEPS    ?= 20000
EPISODES ?= 3

help:
	@echo "Targets: setup, sim, sim-headless, stop, hover, train, eval, tb, clean"

setup:
	python3 -m venv .venv
	. .venv/bin/activate && pip install -U pip && pip install -r requirements.txt

sim:
	HEADLESS=false QGC=true ./scripts/ns-run.sh

sim-headless:
	HEADLESS=true QGC=false ./scripts/ns-run.sh

stop:
	./scripts/ns-stop.sh

hover:
	./tools/hover_run.sh

train:
	GUI=$(GUI) ./rl/ppo_run.sh STEPS=$(STEPS)

eval:
	GUI=$(GUI) EPISODES=$(EPISODES) ./rl/ppo_eval.sh

tb:
	. .venv/bin/activate && python -m tensorboard.main --logdir rl/out/tb

clean:
	rm -rf rl/out log *.ulg __pycache__ */__pycache__
SHELL := /bin/bash

# Defaults (override on the command line if needed)
DATASET ?= datasets/sim_2025-08-22_122259
MAX_DEPTH ?= 20.0
EPOCHS ?= 8
CKPT ?= runs/depth_tinyunet/_ckpt_ep$(EPOCHS).pt
ONNX ?= models/exported/_tinyunet_depth.onnx

.PHONY: depth_smoke depth_offline_aug depth_train_eval depth_export_onnx depth_demo depth_live_mae

depth_smoke:
	. .venv/bin/activate && python scripts/data/smoke_depth_dataset.py --dataset_root '$(DATASET)' --max_depth_m $(MAX_DEPTH) --out '$(DATASET)/manifest.json'

depth_offline_aug:
	. .venv/bin/activate && python scripts/data/augment_depth_dataset.py --in '$(DATASET)' --out '$(DATASET)_aug' --copies 1 --max_depth_m $(MAX_DEPTH)

depth_train_eval:
	. .venv/bin/activate && python ns_train_depth_baseline.py --data '$(DATASET)' --epochs $(EPOCHS) --size 320 240 --out runs/depth_tinyunet
	. .venv/bin/activate && python ns_eval_depth.py --data '$(DATASET)' --ckpt '$(CKPT)' --max_depth_m $(MAX_DEPTH) --out runs/depth_eval

depth_export_onnx:
	. .venv/bin/activate && python ns_export_onnx.py --ckpt '$(CKPT)' --out '$(ONNX)'

depth_demo:
	bash scripts/ns_run_depth_demo.sh

depth_live_mae:
	bash scripts/ns_run_depth_live_mae.sh

# ---- one-command depth pipeline ----
W ?= 320
H ?= 240
EPOCHS ?= 12
DATASET ?= datasets/sim_2025-08-22_122259_aug
CKPT ?= runs/depth_tinyunet/_ckpt_ep$(EPOCHS).pt
ONNX ?= models/exported/_tinyunet_depth_e$(EPOCHS).onnx

.PHONY: depth_train_eval depth_demo_new
depth_train_eval:
	. .venv/bin/activate && python -u ns_train_depth_baseline.py --data '$(DATASET)' --epochs $(EPOCHS) --size $(W) $(H) --out runs/depth_tinyunet | tee runs/depth_tinyunet/train_e$(EPOCHS).log
	. .venv/bin/activate && python -u ns_eval_depth.py --data '$(DATASET)' --ckpt '$(CKPT)' --max_depth_m 20.0 --out runs/depth_eval | tee runs/depth_eval/eval_e$(EPOCHS).log
	. .venv/bin/activate && python -u ns_export_onnx.py --ckpt '$(CKPT)' --out '$(ONNX)'
	ln -sf '$(ONNX)' '$(DATASET:/_aug=)/_tinyunet_depth.onnx'

depth_demo_new:
	ONNX='$(ONNX)' bash scripts/ns_run_depth_demo.sh
