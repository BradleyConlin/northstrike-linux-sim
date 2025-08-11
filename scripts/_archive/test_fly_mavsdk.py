#!/usr/bin/env python3
import asyncio, os, sys, time
from mavsdk import System
import os
import os

# Select which drone & port to control:
# - DRONE: 1 -> 14540, 2 -> 14541, ...
# - PORT: override to use 14550/14551 (QGC ports), etc.
DRONE = int(os.environ.get("DRONE", "1"))
PORT  = int(os.environ.get("PORT", str(14539 + DRONE)))

async def wait_connected(drone, timeout=20):
    print(f"[INFO] Connecting to udp://:{PORT} ...", flush=True)
    addr = os.getenv("MAVSDK_URL","udpin://0.0.0.0:14540"); print(f"[DEBUG] Using MAVSDK URL: {addr}", flush=True); await drone.connect(system_address=addr)
    t0 = time.time()
    async for s in drone.core.connection_state():
        if s.is_connected:
            print("[INFO] Connected.", flush=True)
            return True
        if time.time() - t0 > timeout:
            print("[ERROR] Connection timeout.", flush=True)
            return False

async def relax_prearm(drone):
    # Lighten pre-arm checks for SITL so we can run quickly
    async def seti(name, val):
        try:
            await drone.param.set_param_int(name, val)
            print(f"[PARAM] {name} = {val}", flush=True)
        except Exception as e:
            print(f"[PARAM] {name} set skipped: {e}", flush=True)
    await seti("NAV_DLL_ACT", 0)     # No datalink-loss action
    await seti("COM_DL_LOSS_T", 0)   # No datalink timeout
    await seti("COM_RC_IN_MODE", 1)  # Ignore RC requirement
    await seti("COM_ARM_WO_GPS", 1)  # Allow arm without GPS

async def wait_position(drone, timeout=25):
    print("[INFO] Waiting for position (global OR local)…", flush=True)
    t0 = time.time()
    async for h in drone.telemetry.health():
        if h.is_global_position_ok or h.is_local_position_ok:
            print(f"[INFO] Position OK (global={h.is_global_position_ok}, local={h.is_local_position_ok})", flush=True)
            return True
        if time.time() - t0 > timeout:
            print("[WARN] Position not ready after timeout; proceeding anyway.", flush=True)
            return False

async def main():
    drone = System(mavsdk_server_address="127.0.0.1")
    if not await wait_connected(drone): sys.exit(2)
    await relax_prearm(drone)
    await wait_position(drone)

    try:
        print("[ACTION] Arm", flush=True)
        await drone.action.arm()
        print("[ACTION] Takeoff (~5 m)", flush=True)
        await drone.action.set_takeoff_altitude(5.0)
        await drone.action.takeoff()
        await asyncio.sleep(6)
        print("[ACTION] Land", flush=True)
        await drone.action.land()
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                break
        print("[DONE] Landed.", flush=True)
    except Exception as e:
        print(f"[ERROR] {e}", flush=True); sys.exit(2)

if __name__ == "__main__":
    asyncio.run(main())
