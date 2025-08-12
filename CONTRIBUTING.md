# Contributing / Dev Setup

## Prereqs (system)
- Ubuntu 22.04 (ROS 2 Humble)
- ROS 2 Humble desktop installed via apt
- MAVROS for ROS 2 (`ros-humble-mavros` and `ros-humble-mavros-extras`)
- Gazebo Harmonic (`gz` tooling)
- PX4 Autopilot (harmonic) checked out at: `~/dev/px4-autopilot-harmonic`

> Tip: keep PX4 in `~/dev/px4-autopilot-harmonic` to match paths hardcoded in the README/examples.

## Repo layout
- `scripts/ros2_offboard_hover.py` — OFFBOARD hover example (ROS 2 + MAVROS)
- `worlds/empty_inline_world.sdf` — simple world
- `.ros_env` — local ROS env (edit as needed; committed intentionally)
- `README.md` — quickstart

## One-time Python (optional)
The core flow uses ROS 2 packages (from apt), not pip. Only optional scripts need pip.

```bash
./scripts/bootstrap_venv.sh
# then, whenever needed:
source .venv/bin/activate
pip install -r requirements.txt



```
