import time
import asyncio
from threading import Thread
from typing import Dict, Any, Optional

import gymnasium as gym
import numpy as np
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError


class _AioBridge:
    """Run a persistent asyncio loop in a thread and submit coroutines to it."""
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.t = Thread(target=self.loop.run_forever, daemon=True)
        self.t.start()

    def run(self, coro):
        fut = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return fut.result()


class Px4AltHoldEnv(gym.Env):
    """
    Minimal PX4 altitude-hold environment to validate PPO + MAVSDK + PX4.

    Action: 1D vertical velocity (NED) in m/s, normalized [-1,1] (scaled to [-1,1] m/s)
    Observation: [altitude_error_m, estimated_vz_mps]
    Reward: -|alt_error| - 0.02*|vz_cmd|
    """
    metadata = {}

    def __init__(
        self,
        target_alt_m: float = 2.0,
        max_steps: int = 400,
        step_dt: float = 0.10,
        max_vz_mps: float = 1.0,
        mav_addr: str = "udpin://0.0.0.0:14540",
    ):
        super().__init__()
        self.target_alt = float(target_alt_m)
        self.max_steps = int(max_steps)
        self.step_dt = float(step_dt)
        self.max_vz = float(max_vz_mps)
        self.mav_addr = mav_addr

        self.action_space = gym.spaces.Box(
            low=np.array([-1.0], dtype=np.float32),
            high=np.array([1.0], dtype=np.float32),
            dtype=np.float32,
        )
        hi = np.array([np.inf, np.inf], dtype=np.float32)
        self.observation_space = gym.spaces.Box(-hi, hi, dtype=np.float32)

        self._bridge = _AioBridge()
        self._drone: Optional[System] = None
        self._step_idx = 0
        self._last_alt: Optional[float] = None

        self._connect_and_prep()

    # ---------- MAVSDK (async behind a sync facade) ----------

    async def _async_connect(self):
        d = System()
        await d.connect(system_address=self.mav_addr)

        async for state in d.core.connection_state():
            if state.is_connected:
                break

        # wait until basic position is OK so takeoff works reliably
        async for health in d.telemetry.health():
            if (
                health.is_gyrometer_calibration_ok
                and health.is_accelerometer_calibration_ok
                and (health.is_local_position_ok or health.is_global_position_ok)
            ):
                break
        return d

    async def _async_arm_takeoff(self, d: System, alt: float):
        await d.action.set_takeoff_altitude(alt)
        await d.action.arm()
        await d.action.takeoff()
        # wait near target altitude (timeout safeguard)
        t0 = time.time()
        while time.time() - t0 < 20.0:
            async for p in d.telemetry.position():
                ra = p.relative_altitude_m or 0.0
                if abs(ra - alt) < 0.3:
                    return
                break
            await asyncio.sleep(0.1)

    async def _async_land_disarm(self, d: System):
        try:
            await d.offboard.stop()
        except Exception:
            pass
        try:
            await d.action.land()
        except Exception:
            pass
        async for in_air in d.telemetry.in_air():
            if not in_air:
                break
        try:
            await d.action.disarm()
        except Exception:
            pass

    async def _async_start_offboard(self, d: System):
        await d.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await d.offboard.start()
        except OffboardError:
            await asyncio.sleep(0.2)
            await d.offboard.start()

    def _connect_and_prep(self):
        self._drone = self._bridge.run(self._async_connect())
        self._bridge.run(self._async_arm_takeoff(self._drone, self.target_alt))
        self._bridge.run(self._async_start_offboard(self._drone))

    def _get_rel_alt(self) -> float:
        async def _read(d: System):
            async for p in d.telemetry.position():
                return p.relative_altitude_m or 0.0
        return float(self._bridge.run(_read(self._drone)))

    # ---------- Gym API ----------

    def reset(self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None):
        super().reset(seed=seed)
        self._step_idx = 0
        try:
            self._bridge.run(self._drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0)))
        except Exception:
            pass
        time.sleep(self.step_dt)
        alt = self._get_rel_alt()
        self._last_alt = alt
        obs = np.array([alt - self.target_alt, 0.0], dtype=np.float32)
        return obs, {}

    def step(self, action: np.ndarray):
        self._step_idx += 1

        a = float(np.clip(action[0], -1.0, 1.0))
        vz_cmd = a * self.max_vz  # NED: positive is down
        self._bridge.run(self._drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, vz_cmd, 0.0)))
        time.sleep(self.step_dt)

        alt = self._get_rel_alt()
        vz_est = 0.0 if self._last_alt is None else (alt - self._last_alt) / self.step_dt
        self._last_alt = alt

        alt_err = alt - self.target_alt
        reward = -abs(alt_err) - 0.02 * abs(vz_cmd)

        terminated = abs(alt_err) > 3.0
        truncated = self._step_idx >= self.max_steps

        obs = np.array([alt_err, vz_est], dtype=np.float32)
        return obs, float(reward), bool(terminated), bool(truncated), {}

    def close(self):
        if self._drone is not None:
            try:
                self._bridge.run(self._async_land_disarm(self._drone))
            except Exception:
                pass
