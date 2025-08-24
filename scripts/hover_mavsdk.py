#!/usr/bin/env python3
import asyncio
from mavsdk import System

# SITL exposes an onboard link on udp:14540 (PX4 prints this in your console)
CONN = "udp://127.0.0.1:14540"
ALT  = 10.0

async def main():
    drone = System()
    await drone.connect(CONN)

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Connected via {CONN}")
            break

    print("Waiting for global position estimate...")
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            print("EKF ready")
            break

    print("Arming...")
    await drone.action.arm()
    print(f"Taking off to {ALT} m...")
    await drone.action.set_takeoff_altitude(ALT)
    await drone.action.takeoff()

    # Hold position ~10 seconds
    for _ in range(10):
        pos = await drone.telemetry.position().__anext__()
        print(f"Pos: {pos.relative_altitude_m:.1f} m")
        await asyncio.sleep(1)

    print("Landing...")
    await drone.action.land()

    # Wait until disarmed
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed & disarmed")
            break

if __name__ == "__main__":
    asyncio.run(main())
