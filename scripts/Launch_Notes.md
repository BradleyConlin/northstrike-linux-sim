Northstrike Linux Sim — PX4 + Gazebo Harmonic + QGroundControl (Working Setup)
This is the exact, known-good setup we just verified on Bradley’s machine.

PX4 source dir: ~/dev/px4-autopilot-harmonic

QGroundControl AppImage: ~/Applications/QGroundControl-x86_64.AppImage

Repo: ~/dev/northstrike-linux-sim

Gazebo Sim: Harmonic (gz 8.x)

Micro XRCE-DDS Agent (ROS 2 bridge): snap package, UDP port 8888

What you get
Gazebo Harmonic launches with the x500 model.

PX4 SITL connects, publishes sensors, and EKF is valid at rest (mag enabled).

QGroundControl connects over UDP 14550.

Arming and takeoff work from the PX4 console or QGC.

First-time prep
QGroundControl (AppImage)

Put the AppImage at: ~/Applications/QGroundControl-x86_64.AppImage

Make it executable (once): chmod +x ~/Applications/QGroundControl-x86_64.AppImage

Micro XRCE-DDS Agent (ROS 2)

Install via snap: sudo snap install micro-xrce-dds-agent

Run order (manual, exact)
Terminal A — start the XRCE Agent (leave running)

Command: micro-xrce-dds-agent udp4 -p 8888

Terminal B — launch PX4 + Gazebo (from ~/dev/px4-autopilot-harmonic)

Ensure Gazebo can find PX4 resources by exporting:

GZ_SIM_RESOURCE_PATH=$HOME/dev/px4-autopilot-harmonic/Tools/simulation/gz/worlds:$HOME/dev/px4-autopilot-harmonic/Tools/simulation/gz/models:$HOME/.gz/models:$HOME/.gazebo/models

Command: PX4_GZ_WORLD=default make px4_sitl gz_x500

You should see: “Gazebo world is ready”, “Spawning Gazebo model”, then a pxh> prompt.

Terminal C — start QGroundControl

Command: ~/Applications/QGroundControl-x86_64.AppImage (double-click or run it)

In QGC (first time only): Settings → Comm Links → Add

Type: UDP

Port: 14550

Automatically Connect on Start: On

Server Addresses: leave empty

Save → Connect

Verification (optional)

Check QGC is listening: ss -u -na | grep :14550 (should list one or two sockets)

In the PX4 console (pxh>): listener sensor_gyro 1 and listener sensor_accel 1 should show samples.

PX4 parameters we use in SITL (apply once per clean build)
Open QGC → Analyze Tools → MAVLink Console (that is the pxh> shell). Set:

Safety / power (bench and sim)

CBRK_IO_SAFETY = 22027

CBRK_SUPPLY_CHK = 894281

No RC / no link failsafes (headless sim)

COM_RC_IN_MODE = 4

NAV_RCL_ACT = 0

NAV_DLL_ACT = 0

COM_RCL_EXCEPT = 6 (ignore RC loss in Hold + Offboard)

EKF / mag (valid yaw/position at rest)

SYS_HAS_MAG = 1

EKF2_MAG_TYPE = 0

Apply and make effective

Save parameters: param save

Restart EKF so mag fusion mode takes effect: ekf2 stop then ekf2 start

Quick checks (all at pxh>)

Estimator status: ekf2 status → expect: attitude: 1, local position: 1, global position: 1

Local position: listener vehicle_local_position 1 → expect: xy_valid: True, z_valid: True

Note: after make clean, SITL resets params. Re-apply the list above (or export/import via QGC Parameters → Tools).

Fly (simple)
From the MAVLink Console (pxh>):

Health: commander check → expect “Preflight check: OK”

Arm: commander arm

Takeoff: commander takeoff

In QGC you should see the vehicle lift and show “Flying • Hold/Position”.

Troubleshooting (fast)
Preflight check failed

Click the red “Not Ready” banner in QGC → Arming Check Report for the exact gate.

Common quick fixes in sim:

Safety switch: set CBRK_IO_SAFETY = 22027

Bench power: set CBRK_SUPPLY_CHK = 894281

RC not present: set COM_RC_IN_MODE = 4

Requires GCS/RC active: set NAV_DLL_ACT = 0 and NAV_RCL_ACT = 0

EKF not valid: ensure SYS_HAS_MAG = 1 and EKF2_MAG_TYPE = 0, then ekf2 stop / ekf2 start

Gazebo sensors “never published”

Ensure you launched using make px4_sitl gz_x500 (this spawns Gazebo and the bridge)

Check sim port is open: ss -u -na | grep :14560 (PX4 listens there for sim data)

QGC not connected

Confirm the UDP Comm Link exists and is Connected (port 14550)

ss -u -na | grep :14550 should show QGC listening

XRCE-DDS “create entities failed” spam

This is the ROS 2 bridge being chatty. It doesn’t block arming. Keep the Agent running for ROS 2 work; if you want a quiet console during basic flights, you can run uxrce_dds_client stop at pxh> and start it again later.

