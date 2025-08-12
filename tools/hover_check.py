# ~/dev/northstrike-linux-sim/tools/hover_check.py
import asyncio
from mavsdk import System

# PX4 SITL sends "Onboard" to 14540 by default; this is correct for MAVSDK.
# You'll see a deprecation warning in PX4 console about 'udp://'; it's harmless.
UDP_URL = "udp://:14540"

async def wait_connected(drone, timeout_s=20):
    async def _wait():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return
    await asyncio.wait_for(_wait(), timeout=timeout_s)

async def wait_healthy(drone, timeout_s=30):
    async def _wait():
        # Wait for EKF & position to be usable (SITL: a few seconds)
        async for h in drone.telemetry.health():
            if (h.is_gyrometer_calibration_ok and
                h.is_accelerometer_calibration_ok and
                h.is_magnetometer_calibration_ok and
                h.is_local_position_ok and
                h.is_home_position_ok and
                h.is_global_position_ok and
                h.is_armable):
                return
    await asyncio.wait_for(_wait(), timeout=timeout_s)

async def main():
    drone = System()
    print(f"Connecting to {UDP_URL} …")
    await drone.connect(system_address=UDP_URL)

    print("Waiting for vehicle connection…")
    await wait_connected(drone)
    print("Connected.")

    print("Waiting for sensors/EKF to be healthy…")
    await wait_healthy(drone)
    print("Healthy.")

    print("Arming…")
    await drone.action.arm()

    print("Setting takeoff altitude to 2 m…")
    await drone.action.set_takeoff_altitude(2.0)

    print("Taking off…")
    await drone.action.takeoff()

    # Wait until we're in air (safety)
    async for in_air in drone.telemetry.in_air():
        if in_air:
            break

    print("Hovering for 8 seconds…")
    await asyncio.sleep(8)

    print("Landing…")
    await drone.action.land()

    # Wait until landed and disarmed
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    try:
        await drone.action.disarm()
    except Exception:
        pass
    print("Done.")

if __name__ == "__main__":
    asyncio.run(main())
