import os
import argparse
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from rl.envs.px4_alt_env import Px4AltHoldEnv

def make_env():
    env = Px4AltHoldEnv(
        target_alt_m=2.0,
        max_steps=400,
        step_dt=0.10,
        max_vz_mps=1.0,
        mav_addr="udpin://0.0.0.0:14540",
    )
    return Monitor(env)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=20000, help="total timesteps for PPO")
    parser.add_argument("--out", type=str, default="rl/out", help="output dir")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    env = DummyVecEnv([make_env])  # 1 env for now (SITL is single vehicle)
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        n_steps=512,
        batch_size=128,
        learning_rate=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        ent_coef=0.0,
        clip_range=0.2,
        tensorboard_log=os.path.join(args.out, "tb"),
    )

    model.learn(total_timesteps=args.steps)
    model_path = os.path.join(args.out, "ppo_px4_alt_hover.zip")
    model.save(model_path)
    print(f"[train] Saved to {model_path}")

    env.close()

if __name__ == "__main__":
    main()
