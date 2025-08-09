#!/usr/bin/env python3
import os, shlex, subprocess, time, sys, signal, pathlib

HOME = pathlib.Path.home()
GZ_PATHS = [
    str(HOME / ".gz" / "models"),
    str(HOME / ".gz" / "models" / "PX4-gazebo-models" / "models"),
]
X500_SDF = str(HOME / ".gz" / "models" / "x500" / "model.sdf")
PX4_ROOT = str(HOME / "dev" / "px4-autopilot-harmonic")
PX4_BIN  = f"{PX4_ROOT}/build/px4_sitl_default/bin/px4"

def add_env_paths(env):
    existing = env.get("GZ_SIM_RESOURCE_PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(GZ_PATHS + ([existing] if existing else []))
    return env

def run(cmd, env=None, quiet=False):
    print(f"$ {cmd}")
    stdout = subprocess.DEVNULL if quiet else None
    stderr = subprocess.DEVNULL if quiet else None
    return subprocess.Popen(shlex.split(cmd), env=env, stdout=stdout, stderr=stderr)

def gz_add_resource_runtime(path):
    cmd = (
        'gz service -s /gazebo/resource_paths/add '
        '--reqtype gz.msgs.StringMsg_V --reptype gz.msgs.Boolean -r '
        f'\'data: "{path}"\''
    )
    subprocess.run(cmd, shell=True, check=False)

def gz_spawn_x500(world="default", pose=(0, 0, 1, 0, 0, 0)):
    x, y, z, rr, pp, yy = pose
    req = (
        f'sdf_filename: "{X500_SDF}"\n'
        f'name: "x500"\n'
        f'pose {{ position {{ x: {x} y: {y} z: {z} }} }}'
    )

    cmd = (
        f'gz service -s /world/{world}/create '
        f'--reptype gz.msgs.Boolean '
        f'--reqtype gz.msgs.EntityFactory '
        f'--timeout 5000 -r '
        f"'{req}'"
    )

    return subprocess.run(cmd, shell=True).returncode == 0

def main():
    # sanity
    if not os.path.isfile(PX4_BIN):
        print("PX4 binary not found. Build first:\n  DONT_RUN=1 make px4_sitl_default", file=sys.stderr)
        sys.exit(1)
    if not os.path.isfile(X500_SDF):
        print(f"x500 SDF not found at {X500_SDF}. Make sure PX4-gazebo-models is cloned.", file=sys.stderr)
        sys.exit(1)

    env = add_env_paths(os.environ.copy())

    # 1) Start Gazebo
    gz = run("gz sim -v 4 " + os.path.join(PX4_ROOT, "Tools/simulation/gz/worlds/default.sdf"), env=env, quiet=True)
    time.sleep(3.0)

    # 2) Ensure paths visible at runtime & spawn x500
    for p in GZ_PATHS:
        gz_add_resource_runtime(p)
    ok = gz_spawn_x500(world="default", pose=(0,0,1,0,0,0))
    if not ok:
        print("Spawn failed (continuing; PX4 may still try to spawn).")

    # 3) Launch PX4
    px4_env = env.copy()
    px4_env.update({
        "PX4_SYS_AUTOSTART": "4001",
        "PX4_SIM_MODEL": "gz_x500",
        "PX4_GZ_WORLD": "default",
        "PX4_GZ_MODEL_NAME": "x500",
        "PX4_GZ_MODEL_POSE": "0,0,1,0,0,0",
    })

    px4 = subprocess.Popen(
        [PX4_BIN, "-i", "0", "-d", "-s", "etc/init.d-posix/rcS"],
        env=px4_env,
        cwd=PX4_ROOT,   # <-- critical so etc/init.d-posix/rcS is found
    )

    print("Running. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for p in (px4, gz):
            if p and p.poll() is None:
                p.terminate()
        for p in (px4, gz):
            try: p.wait(timeout=5)
            except Exception: pass

if __name__ == "__main__":
    main()

