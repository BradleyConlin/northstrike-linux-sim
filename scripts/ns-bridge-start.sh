#!/usr/bin/env bash
set -e
set +u; source /opt/ros/humble/setup.bash; set -u

# Topics fixed by your SDF + Humble type mappings
ros2 run ros_gz_bridge parameter_bridge \
  /world/landing_test/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock \
  /ns_rgb/image@sensor_msgs/msg/Image@ignition.msgs.Image \
  /ns_depth/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image \
  /ns_lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan \
  /ns/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU \
  /ns/mag@sensor_msgs/msg/MagneticField@ignition.msgs.Magnetometer \
  /ns/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat
