#!/usr/bin/env bash
set -euo pipefail

# Northstrike Linux Sim bootstrap: README, env helper, helper scripts, .gitignore, and git commit/push.
# Safe to re-run; it won't overwrite ros2_offboard_hover.py if you already have one.

REPO="${HOME}/dev/northstrike-linux-sim"
echo "[*] Ensuring repo directory exists: ${REPO}"
mkdir -p "${REPO}"
cd "${REPO}"

echo "[*] Making folders (scripts/, runs/)"
mkdir -p scripts runs

# --- README.md ---
echo "[*] Writing README.md"
cat > README.md <<'MD'
# Northstrike Linux Sim — PX4 + Gazebo Harmonic + MAVROS (ROS 2 Humble)

Working flow to run PX4 SITL with Gazebo Harmonic and control via MAVROS (ROS 2).
Includes a simple OFFBOARD hover script.

## Prereqs
- Ubuntu 22.04
- ROS 2 Humble (`/opt/ros/humble`)
- Gazebo Harmonic (`gz sim` runs)
- PX4 repo at `~/dev/px4-autopilot-harmonic` (built once: `make px4_sitl_default`)
- MAVROS for ROS 2 Humble

## Layout
```
northstrike-linux-sim/
├─ scripts/
│  ├─ ros2_offboard_hover.py     # hover (3 m AGL, ~10 s)
│  └─ prime_offboard.py          # primes OFFBOARD
├─ runs/                         # rosbag output (gitignored)
├─ .ros_env                      # env helper
└─ README.md
```

## Quickstart (3 terminals)

### A) PX4 + Gazebo
```bash
cd ~/dev/px4-autopilot-harmonic
unset PX4_GZ_WORLD GZ_WORLD
make px4_sitl gz_x500
```

### B) MAVROS
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@
```

### C) OFFBOARD hover
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 scripts/ros2_offboard_hover.py
```
Expected: OFFBOARD -> arm -> climb ~3 m -> hold ~10 s -> AUTO.LAND.

## Verify
```bash
ros2 topic echo --once /mavros/state
ros2 topic hz /mavros/local_position/pose
```

## Record
```bash
mkdir -p runs
ros2 bag record -o runs/hover_$(date +%F_%H-%M) \
  /mavros/local_position/pose /mavros/state /mavros/imu/data /mavros/velocity/local
```

## Troubleshooting
**Black Gazebo / “.sdf.sdf”**: `unset PX4_GZ_WORLD GZ_WORLD` (use basename only if setting).
**ROS 2 graph empty**: ensure same env everywhere:
```bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop; ros2 daemon start
```
**OFFBOARD rejected / arm refused**: prime setpoints >2 Hz, then:
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```
For SITL:
```
param set COM_ARM_WO_GPS 1
param set NAV_DLL_ACT 0
param set COM_OBL_ACT 0
param save
```
MD

# --- .ros_env ---
echo "[*] Writing .ros_env"
cat > .ros_env <<'ENV'
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV

# --- scripts/prime_offboard.py ---
echo "[*] Writing scripts/prime_offboard.py"
cat > scripts/prime_offboard.py <<'PY'
#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    n = Node('prime_offboard')
    pub = n.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
    sp = PoseStamped(); sp.pose.position.z = 3.0
    t0 = time.time()
    while time.time() - t0 < 3.0:
        sp.header.stamp = n.get_clock().now().to_msg()
        pub.publish(sp)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(1/20)
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
PY
chmod +x scripts/prime_offboard.py

# --- scripts/ros2_offboard_hover.py (only create if missing) ---
if [ ! -f scripts/ros2_offboard_hover.py ]; then
  echo "[*] Creating scripts/ros2_offboard_hover.py"
  cat > scripts/ros2_offboard_hover.py <<'PY'
#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

UAS_NS = '/mavros'
HOVER_Z = 3.0
DURATION = 10.0
PUB_HZ = 20.0

class OffboardHover(Node):
    def __init__(self):
        super().__init__('offboard_hover')
        self.state = State()
        self.create_subscription(State, f'{UAS_NS}/state', self._state_cb, 10)
        self.sp_pub = self.create_publisher(PoseStamped, f'{UAS_NS}/setpoint_position/local', 10)
        self.arm_cli = self.create_client(CommandBool, f'{UAS_NS}/cmd/arming')
        self.mode_cli = self.create_client(SetMode,     f'{UAS_NS}/set_mode')

    def _state_cb(self, msg): self.state = msg

    def _call_sync(self, client, req, timeout=5.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def run(self):
        while not self.arm_cli.wait_for_service(timeout_sec=1.0): pass
        while not self.mode_cli.wait_for_service(timeout_sec=1.0): pass

        sp = PoseStamped()
        sp.pose.position.x = 0.0
        sp.pose.position.y = 0.0
        sp.pose.position.z = HOVER_Z

        period = 1.0 / PUB_HZ

        # Prime OFFBOARD
        for _ in range(int(2.0 * PUB_HZ)):
            sp.header.stamp = self.get_clock().now().to_msg()
            self.sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        # Set OFFBOARD
        mreq = SetMode.Request(); mreq.custom_mode = 'OFFBOARD'
        mres = self._call_sync(self.mode_cli, mreq)
        self.get_logger().info(f'set_mode OFFBOARD -> {getattr(mres, "mode_sent", False)}')

        # Arm
        areq = CommandBool.Request(); areq.value = True
        ares = self._call_sync(self.arm_cli, areq)
        self.get_logger().info(f'arm -> {getattr(ares, "success", False)}')

        # Hold
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < DURATION:
            sp.header.stamp = self.get_clock().now().to_msg()
            self.sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        # Land
        land = SetMode.Request(); land.custom_mode = 'AUTO.LAND'
        self._call_sync(self.mode_cli, land)
        self.get_logger().info('Requested AUTO.LAND')
        time.sleep(2.0)

def main():
    rclpy.init()
    n = OffboardHover()
    try:
        n.run()
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
PY
  chmod +x scripts/ros2_offboard_hover.py
else
  echo "[*] scripts/ros2_offboard_hover.py already exists; leaving it as-is"
fi

# --- .gitignore ---
echo "[*] Writing .gitignore"
cat > .gitignore <<'GI'
__pycache__/
*.pyc
runs/
*.db3
.venv/
.vscode/
.idea/
.DS_Store
*.AppImage
GI

# --- Git init/commit/push ---
if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "[*] Git repo detected"
else
  echo "[*] Initializing new git repo"
  git init
fi

echo "[*] Adding files"
git add README.md .ros_env .gitignore scripts/prime_offboard.py
git add scripts/ros2_offboard_hover.py 2>/dev/null || true

echo "[*] Commit"
git commit -m "Docs + helpers: PX4+Gazebo+MAVROS flow; OFFBOARD hover; prime script; env helper" || true

echo "[*] Attempting push (will print hint if no remote)"
if git remote -v | grep -q .; then
  git push || true
else
  echo "No git remote configured."
  echo "To push: git remote add origin <URL>; git branch -M main; git push -u origin main"
fi

echo "[✓] Bootstrap complete."
