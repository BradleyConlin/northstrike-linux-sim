#!/usr/bin/env python3
import asyncio, sys
from mavsdk import System

CONNECT_URL = "udpin://0.0.0.0:14540"  # PX4 SITL sends MAVLink to 14540 by default

async def wait_connected(drone, timeout=30):
    print("Connecting to PX4 on", CONNECT_URL)
    try:
        await asyncio.wait_for(_wait_connected(drone), timeout)
    except asyncio.TimeoutError:
        raise TimeoutError("No MAVLink connection on :14540 (check SITL is running)")

async def _wait_connected(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            return

async def wait_position_ok(drone, timeout=30):
    print("Waiting for home + global position…")
    try:
        await asyncio.wait_for(_wait_position_ok(drone), timeout)
    except asyncio.TimeoutError:
        raise TimeoutError("EKF not ready (no position). Make sure mag is enabled and EKF is green.")

async def _wait_position_ok(drone):
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            print("Position OK")
            return

async def wait_altitude(drone, min_rel_alt=2.0, timeout=45):
    try:
        await asyncio.wait_for(_wait_altitude(drone, min_rel_alt), timeout)
    except asyncio.TimeoutError:
        raise TimeoutError("Never reached target altitude (check arming/takeoff).")

async def _wait_altitude(drone, min_rel_alt):
    async for p in drone.telemetry.position():
        if p.relative_altitude_m >= min_rel_alt:
            return

async def wait_landed(drone, timeout=60):
    try:
        await asyncio.wait_for(_wait_landed(drone), timeout)
    except asyncio.TimeoutError:
        raise TimeoutError("Landing did not complete in time.")

async def _wait_landed(drone):
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            return

async def main():
    drone = System()
    await drone.connect(system_address=CONNECT_URL)

    await wait_connected(drone)
    await wait_position_ok(drone)

    print("Setting takeoff altitude to 3 m")
    await drone.action.set_takeoff_altitude(3.0)

    print("Arming…")
    await drone.action.arm()
    await asyncio.sleep(1)

    print("Takeoff…")
    await drone.action.takeoff()
    await wait_altitude(drone, 2.0)

    print("Hovering 5 s…")
    await asyncio.sleep(5)

    print("Landing…")
    await drone.action.land()
    await wait_landed(drone)

    print("SMOKE TEST PASS ✅  (takeoff/hover/land)")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"SMOKE TEST FAIL ❌  {e}", file=sys.stderr)
        sys.exit(1)
