import os
import sys
import asyncio
from mavsdk import System
from mavsdk.action import ActionError

DEFAULT_URL = "udpin://0.0.0.0:14540"

async def wait_connected(drone, timeout_s=20):
    print("[INFO] Waiting for MAVSDK connection…", flush=True)
    async def _wait():
        async for s in drone.core.connection_state():
            if s.is_connected:
                print("[INFO] MAVSDK connected.", flush=True)
                return
    try:
        await asyncio.wait_for(_wait(), timeout=timeout_s)
    except asyncio.TimeoutError:
        print("[ERROR] Timed out waiting for MAVSDK connection.", file=sys.stderr)
        sys.exit(2)

async def wait_health(drone, timeout_s=60):
    """
    Wait until basic health is OK:
      - sensors calibrated
      - local OR global position available
    """
    print("[INFO] Waiting for vehicle health…", flush=True)
    async def _wait():
        async for h in drone.telemetry.health():
            if (h.is_gyrometer_calibration_ok and
                h.is_accelerometer_calibration_ok and
                (h.is_local_position_ok or h.is_global_position_ok)):
                print("[INFO] Vehicle health OK.", flush=True)
                return
    try:
        await asyncio.wait_for(_wait(), timeout=timeout_s)
    except asyncio.TimeoutError:
        print("[WARN] Health not OK after timeout; proceeding anyway.", flush=True)

async def main():
    url = os.getenv("MAVSDK_URL", DEFAULT_URL)
    print(f"[DEBUG] Using MAVSDK URL: {url}", flush=True)

    drone = System()
    await drone.connect(system_address=url)

    await wait_connected(drone, 20)
    await wait_health(drone, 60)

    try:
        print("[ACTION] Setting takeoff altitude to 10 m", flush=True)
        await drone.action.set_takeoff_altitude(10.0)

        print("[ACTION] Arm", flush=True)
        await drone.action.arm()
        print("[STATE] Armed", flush=True)

        print("[ACTION] Takeoff", flush=True)
        await drone.action.takeoff()

        await asyncio.sleep(8)  # hover a bit

        print("[ACTION] Land", flush=True)
        await drone.action.land()

        # wait until landed
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                print("[STATE] Landed", flush=True)
                break

        # tidy disarm (ignore if already disarmed)
        try:
            await drone.action.disarm()
            print("[STATE] Disarmed", flush=True)
        except ActionError:
            pass

        print("[DONE] Smoke test complete.", flush=True)
    except ActionError as e:
        print(f"[ERROR] Action failed: {e}", file=sys.stderr)
        sys.exit(3)

if __name__ == "__main__":
    asyncio.run(main())
