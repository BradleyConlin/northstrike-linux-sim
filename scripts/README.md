# Scripts

## Orchestrator
- **run_all_linux.py** — single command to prep and launch the sim.
  - Order: clean → Gazebo → PX4 → (optional) QGC
  - Defaults: `--headless true`, `--qgc false`
  - Examples:
    - Headless, detached:  
      `python3 scripts/run_all_linux.py --drones 2 --headless true --qgc false --detach true`
    - With GUI + QGC:  
      `QGC_APPIMAGE=~/dev/QGroundControl.AppImage python3 scripts/run_all_linux.py --drones 2 --headless false --qgc true`

## Launchers
- **clean_env_linux.py** — kills PX4/GZ/QGC, clears env.
- **launch_gazebo_world_linux.py** — launches Gazebo Harmonic using PX4’s default world.  
  - `--headless` flag (no value) runs without GUI.
- **launch_px4_linux.py** — spawns N PX4 SITL instances for Gazebo Harmonic.
  - Default model: `x500`
  - Blocks by default; add `--detach` to return immediately.
  - Logs: `logs/drone_*.txt`
- **launch_qgroundcontrol_linux.py** — opens QGC AppImage.
  - Set `QGC_APPIMAGE` to point at your AppImage if it’s not under the repo.

## Tips
- If models don’t appear in Gazebo:
  1) Ensure Gazebo is running first.
  2) Ensure PX4 built: `DONT_RUN=1 make px4_sitl_default`
  3) `launch_px4_linux.py` must **not** set `PX4_GZ_MODEL_NAME` (attach-only).
- Stop everything fast: `python3 scripts/clean_env_linux.py`

### Notes
- If a GUI window appears while `--headless` was requested, run `clean_env_linux.py` first; a previous GUI instance was still running.
