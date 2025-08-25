#!/usr/bin/env python3
import os, time, random, yaml, pathlib, socket, subprocess

random.seed()

def u(a,b,prec=2): return round(random.uniform(a,b), prec)

# Base profile
profile = {
    "run_id": int(time.time()),
    "world": os.environ.get("NS_WORLD", "open_field.world.sdf"),
    "lighting": {
        "ambient": [u(0.2,0.6), u(0.2,0.6), u(0.2,0.6), 1.0],
        "background": [u(0.6,0.9), u(0.7,1.0), 1.0, 1.0],
    },
    "wind": {
        "speed_mps": u(0.0, 8.0),
        "direction_deg": u(0, 360, 1),
        "turbulence": u(0.0, 0.8),
    },
    "sensors": {
        "camera_noise_sigma": u(0.0, 0.02, 4),
        "imu_bias_g": u(0.0, 0.02, 4),
        "gps_noise_m": u(0.0, 1.5, 3),
    },
    "textures": {
        "ground": random.choice(["grass_light","grass_dry","asphalt","dirt"]),
    },
    "meta": {
        "host": socket.gethostname(),
        "user": os.environ.get("USER",""),
        "git_rev": "",
    }
}

# Optional: stamp current git rev if repo exists
try:
    top = subprocess.check_output(["git","rev-parse","--show-toplevel"], cwd=str(pathlib.Path(__file__).resolve().parents[2])).decode().strip()
    rev = subprocess.check_output(["git","rev-parse","--short","HEAD"], cwd=top).decode().strip()
    profile["meta"]["git_rev"] = rev
except Exception:
    pass

out_dir = pathlib.Path(__file__).parent / "profiles"
out_dir.mkdir(parents=True, exist_ok=True)
out_path = out_dir / f"profile_{profile['run_id']}.yaml"
with open(out_path, "w") as f:
    yaml.safe_dump(profile, f, sort_keys=False)
print(out_path)
