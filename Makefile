.PHONY: help setup sim sim-headless stop hover train eval tb clean

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
GUI?=true
GUI=$(GUI) ./rl/ppo_run.sh STEPS?=20000
GUI=$(GUI) ./rl/ppo_run.sh STEPS=$(STEPS)

eval:
GUI?=true
EPISODES?=3
GUI=$(GUI) EPISODES=$(EPISODES) ./rl/ppo_eval.sh

tb:
. .venv/bin/activate && python -m tensorboard.main --logdir rl/out/tb

clean:
rm -rf rl/out log *.ulg
