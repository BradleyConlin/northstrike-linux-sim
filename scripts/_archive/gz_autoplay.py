#!/usr/bin/env python3
import subprocess, time, os, sys

WORLD = os.environ.get("GZ_WORLD", "default")
svc = f"/world/{WORLD}/control"
topic = f"/world/{WORLD}/control/state"

def call(cmd):
    return subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0

def unpause_via_service():
    # Gazebo 8: reqtype WorldControl, reptype Boolean
    return call(["gz","service","-s",svc,
                 "--reqtype","gz.msgs.WorldControl",
                 "--reptype","gz.msgs.Boolean",
                 "-p","pause:false",
                 "--timeout","500"])

def unpause_via_topic():
    # Publish WorldControlState with pause:false
    return call(["gz","topic","-t",topic,
                 "-m","gz.msgs.WorldControlState",
                 "-p","world_control: { pause: false }"])

deadline = time.time() + 90
while time.time() < deadline:
    if unpause_via_service() or unpause_via_topic():
        print(f"[INFO] Gazebo world '{WORLD}' unpaused.")
        sys.exit(0)
    time.sleep(0.5)

print(f"[WARN] GZ autoplay timed out (world='{WORLD}').", file=sys.stderr)
sys.exit(1)
