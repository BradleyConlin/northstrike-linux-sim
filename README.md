# Northstrike Linux Sim — PX4 + Gazebo + RL (PPO)

## Quickstart
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
Run sim (GUI)
bash
Copy
Edit
HEADLESS=false QGC=true ./scripts/ns-run.sh
# stop:
./scripts/ns-stop.sh
Hover smoke test
bash
Copy
Edit
./tools/hover_run.sh
PPO training (alt-hold)
bash
Copy
Edit
GUI=true ./rl/ppo_run.sh STEPS=20000
# TensorBoard:
tensorboard --logdir rl/out/tb
PPO eval
bash
Copy
Edit
GUI=true ./rl/ppo_eval.sh EPISODES=3
Repo layout
scripts/ — sim launchers (ns-run.sh, ns-stop.sh)

tools/ — quick manual tests (hover_check.py, hover_run.sh)

rl/ — gym env + PPO (envs/px4_alt_env.py, train_ppo.py, eval_ppo.py)

rl/out/ — logs & models (gitignored)

Notes
MAVSDK URL uses udpin://0.0.0.0:14540 to avoid the “No network interface supplied” warning.

Protobuf version warnings are harmless for our flow.
