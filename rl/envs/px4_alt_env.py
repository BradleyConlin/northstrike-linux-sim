# rl/envs/px4_alt_env.py
import os
import sys
import time
import math
import asyncio
import signal
import shutil
import socket
import stat
import subprocess
from pathlib import Path
from typing import Optional, Tuple

# --- Gym / Gymnasium compatibility ---
try:
    import gymnasium as gym
    from gymnasium import spaces
    _GYMNASIUM = True
except Exception:
    import gym
    from gym import spaces
    _GYMNASIUM = False

import numpy as np

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw


def _tcp_listen_on(port: int) -> bool:
    """Return True if something is already listening on localhost:port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.1)
        try:
            s.connect(("127.0.0.1", port))
            return True
        except OSError:
            return False


def _find_mavsdk_server() -> Optional[str]:
    """Locate mavsdk_server via PATH, venv/bin, or site-packages bundle."""
    p = shutil.which("mavsdk_server")
    if p and os.path.isfile(p):
        return p

    cand = Path(sys.prefix) / "bin" / "mavsdk_server"
    if cand.exists():
        return str(cand)

    try:
        import mavsdk as mavsdk_pkg  # type: ignore
        site_bin = Path(mavsdk_pkg.__file__).parent / "bin" / "mavsdk_server"
        if site_bin.exists():
            return str(site_bin)
    except Exception:
        pass

    return None


def _ensure_executable(path: str):
    st = os.stat(path)
    if not (st.st_mode & stat.S_IXUSR):
        os.chmod(path, st.st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


class Px4AltHoldEnv(gym.Env):
    """
    Simple altitude-hold environment:
      Action: vz in [-max_vz, +max_vz]
      Obs: [alt_rel_m, vz_mps, (target - alt)]
      Reward: -|alt_err| - 0.01*|vz|
    """

    metadata = {"render_modes": []} if _GYMNASIUM else {"render.modes": []}

    def __init__(
        self,
        target_alt_m: float = 2.0,
        max_steps: int = 400,
        step_dt: float = 0.10,
        max_vz_mps: float = 1.0,
        mav_addr: str = "udp://0.0.0.0:14540",
        grpc_port: Optional[int] = None,
    ):
        super().__init__()
        self.target_alt_m = float(target_alt_m)
        self.max_steps = int(max_steps)
        self.step_dt = float(step_dt)
        self.max_vz = float(max_vz_mps)
        self.mav_addr = str(mav_addr)
        self.grpc_port = int(grpc_port) if grpc_port is not None else 50051

        self.action_space = spaces.Box(
            low=np.array([-self.max_vz], dtype=np.float32),
            high=np.array([+self.max_vz], dtype=np.float32),
            dtype=np.float32,
        )
        high = np.array([100.0, 20.0, 100.0], dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._drone: Optional[System] = None
        self._mavsdk_proc: Optional[subprocess.Popen] = None
        self._steps = 0
        self._t_last = time.time()
        self._seed = 0

    # ---------- Gym/Gymnasium ----------
    def seed(self, seed: Optional[int] = None):
        if seed is None:
            seed = int(time.time() * 1e3) & 0xFFFF
        self._seed = int(seed)
        np.random.seed(self._seed)
        return [self._seed]

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if seed is not None:
            self.seed(seed)
        self._safe_shutdown()
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._steps = 0
        self._t_last = time.time()
        self._drone = None
        self._mavsdk_proc = None

        self._loop.run_until_complete(self._async_connect())
        self._loop.run_until_complete(self._async_arm_and_start_offboard())
        obs = self._loop.run_until_complete(self._async_get_obs())
        return (obs, {}) if _GYMNASIUM else obs

    def step(self, action: np.ndarray):
        a_vz = float(np.clip(action[0], -self.max_vz, self.max_vz))
        self._loop.run_until_complete(self._async_set_vz(a_vz))

        now = time.time()
        sleep_s = max(0.0, self.step_dt - (now - self._t_last))
        if sleep_s > 0:
            time.sleep(sleep_s)
        self._t_last = time.time()

        alt, vz, err = self._loop.run_until_complete(self._async_state())
        obs = np.array([alt, vz, err], dtype=np.float32)
        reward = -abs(err) - 0.01 * abs(vz)

        self._steps += 1
        terminated = abs(err) > 10.0 or math.isnan(alt) or math.isnan(vz)
        truncated = self._steps >= self.max_steps
        info = {}
        if _GYMNASIUM:
            return obs, float(reward), bool(terminated), bool(truncated), info
        else:
            return obs, float(reward), bool(terminated or truncated), info

    def render(self):  # no-op
        return

    def close(self):
        self._safe_shutdown()

    # ---------- MAVSDK ----------
    async def _async_connect(self):
        # If someone already runs a server on grpc_port, reuse it.
        if not _tcp_listen_on(self.grpc_port) and os.getenv("MAVSDK_NO_SPAWN", "0") != "1":
            server_bin = _find_mavsdk_server()
            if server_bin is None:
                raise RuntimeError("mavsdk_server not found (PATH/venv/site-packages).")
            _ensure_executable(server_bin)
            cmd = [server_bin, "-p", str(self.grpc_port), self.mav_addr]
            print(f"[env] Launching: {' '.join(cmd)}", flush=True)
            self._mavsdk_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            # let it bind
            await asyncio.sleep(0.6)
            if not _tcp_listen_on(self.grpc_port):
                raise RuntimeError(f"Failed to start mavsdk_server on port {self.grpc_port}")

        # Connect client to gRPC
        self._drone = System(mavsdk_server_address="127.0.0.1", port=self.grpc_port)
        # Explicit connect to PX4 endpoint
        await self._drone.connect(system_address=self.mav_addr)

        # Wait for MAVSDK<->PX4 link (max ~10s)
        t0 = time.time()
        async for state in self._drone.core.connection_state():
            if state.is_connected:
                break
            if time.time() - t0 > 10.0:
                raise TimeoutError("Timeout waiting for PX4 connection")

        # Health (best effort)
        t0 = time.time()
        async for health in self._drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break
            if time.time() - t0 > 20.0:
                break

    async def _async_arm_and_start_offboard(self):
        assert self._drone is not None
        try:
            await self._drone.action.arm()
        except Exception:
            pass
        await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await self._drone.offboard.start()
        except OffboardError:
            await asyncio.sleep(0.2)
            await self._drone.offboard.start()

    async def _async_set_vz(self, vz_mps: float):
        assert self._drone is not None
        cmd = VelocityNedYaw(0.0, 0.0, -vz_mps, 0.0)
        await self._drone.offboard.set_velocity_ned(cmd)

    async def _async_state(self) -> Tuple[float, float, float]:
        assert self._drone is not None
        pos = await self._drone.telemetry.position().__anext__()
        vel = await self._drone.telemetry.velocity_ned().__anext__()
        alt = float(pos.relative_altitude_m)
        vz = float(-vel.down_m_s)
        err = float(self.target_alt_m - alt)
        return alt, vz, err

    async def _async_get_obs(self) -> np.ndarray:
        alt, vz, err = await self._async_state()
        return np.array([alt, vz, err], dtype=np.float32)

    def _safe_shutdown(self):
        try:
            if self._drone is not None:
                try:
                    self._loop.run_until_complete(self._drone.offboard.stop())
                except Exception:
                    pass
                try:
                    self._loop.run_until_complete(self._drone.action.disarm())
                except Exception:
                    pass
        except Exception:
            pass

        if self._mavsdk_proc is not None:
            try:
                os.killpg(os.getpgid(self._mavsdk_proc.pid), signal.SIGTERM)
            except Exception:
                pass
            self._mavsdk_proc = None

        try:
            if self._loop.is_running():
                self._loop.stop()
        except Exception:
            pass
        try:
            self._loop.close()
        except Exception:
            pass
