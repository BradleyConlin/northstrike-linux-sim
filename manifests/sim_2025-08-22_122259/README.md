# Dataset: sim_2025-08-22_122259

- Source: rosbag2 (sqlite3), exported offline
- RGB: 1280x720, Depth: 640x480 (uint16 -> mm)
- Index & splits committed (data files ignored)

## How to reproduce
```bash
source /opt/ros/humble/setup.bash
python3 scripts/ns_export_offline.py --bag runs/sim_2025-08-22_122259 --out datasets/sim_2025-08-22_122259
python3 scripts/ns_build_index.py --out datasets/sim_2025-08-22_122259 --max_dt 0.05
