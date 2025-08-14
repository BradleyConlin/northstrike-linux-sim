# Changelog

## 2025-08-12
- Stable launcher: headless gating, single-instance QGC/GZ, world-ready GUI watcher.
- Tools: `hover_check.py` + `hover_run.sh` verified (arm, takeoff, hold, land).
- RL: `Px4AltHoldEnv` (MAVSDK) + PPO wiring; first 20k-step run.
  - Reward improved ~ -700 → -140; logs in `rl/out/tb/`, model `rl/out/ppo_px4_alt_hover.zip`.
- Scripts: `ppo_run.sh` and `ppo_eval.sh`; evaluation successful in GUI.
- Docs/DevEx: README, training plan, Makefile targets for setup/sim/train/eval/TB.
