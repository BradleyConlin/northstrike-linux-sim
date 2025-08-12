import argparse
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from rl.envs.px4_alt_env import Px4AltHoldEnv

def make_env():
    return Monitor(
        Px4AltHoldEnv(
            target_alt_m=2.0,
            max_steps=400,
            step_dt=0.10,
            max_vz_mps=1.0,
            mav_addr="udpin://0.0.0.0:14540",
        )
    )

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", default="rl/out/ppo_px4_alt_hover.zip")
    p.add_argument("--episodes", type=int, default=3)
    args = p.parse_args()

    env = make_env()
    model = PPO.load(args.model)

    for ep in range(args.episodes):
        obs, info = env.reset()
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

    env.close()

if __name__ == "__main__":
    main()
