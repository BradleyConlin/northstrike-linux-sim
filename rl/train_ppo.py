# rl/train_ppo.py
import os
import argparse
from typing import List

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
from stable_baselines3.common.utils import set_random_seed

from rl.envs.px4_alt_env import Px4AltHoldEnv


def parse_list(n: int, csv: str, base: int, step: int) -> List[int]:
    if csv:
        vals = [int(x.strip()) for x in csv.split(",") if x.strip()]
    else:
        vals = [base + i * step for i in range(n)]
    if len(vals) != n:
        raise ValueError(f"Need exactly {n} values, got {len(vals)}: {vals}")
    return vals


def make_env_fn(px4_port: int, grpc_port: int, seed: int):
    def _thunk():
        env = Px4AltHoldEnv(
            target_alt_m=2.0,
            max_steps=400,
            step_dt=0.10,
            max_vz_mps=1.0,
            mav_addr=f"udp://:{px4_port}",
            grpc_port=grpc_port,
        )
        try:
            env.seed(seed)
        except Exception:
            pass
        return Monitor(env)
    return _thunk


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=int, default=12_000_000)
    p.add_argument("--num-envs", type=int, default=4)

    p.add_argument("--ports", type=str, default="", help="CSV PX4 MAVLink ports (e.g., 14540,14541,...)")
    p.add_argument("--base-port", type=int, default=14540)
    p.add_argument("--port-step", type=int, default=1)

    p.add_argument("--grpc-ports", type=str, default="", help="CSV MAVSDK gRPC ports (e.g., 50060,50061,...)")
    p.add_argument("--grpc-base", type=int, default=50060)
    p.add_argument("--grpc-step", type=int, default=1)

    p.add_argument("--device", type=str, default="cuda", choices=["cuda", "cpu", "auto"])
    p.add_argument("--seed", type=int, default=42)

    p.add_argument("--logdir", type=str, default="runs/ppo_hover_vec4")
    p.add_argument("--tb-name", type=str, default="PPO_hover")
    p.add_argument("--checkpoint-dir", type=str, default="checkpoints/ppo_hover_vec4")
    p.add_argument("--ckpt-every", type=int, default=200_000)

    args = p.parse_args()

    device = "cuda" if (args.device == "auto" and torch.cuda.is_available()) else args.device
    if device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("device=cuda requested but CUDA is not available")

    os.makedirs(args.logdir, exist_ok=True)
    os.makedirs(args.checkpoint_dir, exist_ok=True)

    px4_ports = parse_list(args.num_envs, args.ports, args.base_port, args.port_step)
    grpc_ports = parse_list(args.num_envs, args.grpc_ports, args.grpc_base, args.grpc_step)

    set_random_seed(args.seed)

    env_fns = [make_env_fn(px4_ports[i], grpc_ports[i], args.seed + i) for i in range(args.num_envs)]
    vec_env = DummyVecEnv(env_fns)

    policy_kwargs = dict(net_arch=[256, 256])
    model = PPO(
        "MlpPolicy",
        vec_env,
        device=device,
        n_steps=1024,          # quicker first TB write; adjust later
        batch_size=2048,
        n_epochs=5,
        learning_rate=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs=policy_kwargs,
        tensorboard_log=args.logdir,
        verbose=1,
        seed=args.seed,
    )

    ckpt_cb = CheckpointCallback(
        save_freq=args.ckpt_every,
        save_path=args.checkpoint_dir,
        name_prefix="ppo_px4_alt",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )
    model.learn(total_timesteps=args.steps, tb_log_name=args.tb_name, reset_num_timesteps=False,
                callback=CallbackList([ckpt_cb]))
    model.save(os.path.join(args.checkpoint_dir, "model_final.zip"))
    vec_env.close()


if __name__ == "__main__":
    main()
