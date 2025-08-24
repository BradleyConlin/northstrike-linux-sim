#!/usr/bin/env python3
import os
from pathlib import Path
import csv
from typing import Sequence

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, Imu, MagneticField, NavSatFix, LaserScan
from cv_bridge import CvBridge


# ---------- QoS (force RELIABLE on all subscriptions) ----------
QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def stamp_to_sec(stamp) -> float:
    return (float(stamp.sec) + float(stamp.nanosec) * 1e-9) if stamp else 0.0


class ExportNode(Node):
    def __init__(self, out_dir: Path):
        super().__init__("ns_export_node")
        self.out = out_dir
        (self.out / "rgb").mkdir(parents=True, exist_ok=True)
        (self.out / "depth").mkdir(parents=True, exist_ok=True)

        # CSV files
        self._imu_f   = open(self.out / "imu.csv",   "w", newline="")
        self._mag_f   = open(self.out / "mag.csv",   "w", newline="")
        self._gps_f   = open(self.out / "gps.csv",   "w", newline="")
        self._lidar_f = open(self.out / "lidar.csv", "w", newline="")

        self.imu_w   = csv.writer(self._imu_f)
        self.mag_w   = csv.writer(self._mag_f)
        self.gps_w   = csv.writer(self._gps_f)
        self.lidar_w = csv.writer(self._lidar_f)

        self.imu_w.writerow(
            ["t_sec", "ori_x", "ori_y", "ori_z", "ori_w",
             "ang_x", "ang_y", "ang_z", "lin_x", "lin_y", "lin_z"]
        )
        self.mag_w.writerow(["t_sec", "mag_x", "mag_y", "mag_z"])
        self.gps_w.writerow(["t_sec", "lat", "lon", "alt", "cov_xx"])
        self.lidar_w.writerow(["t_sec", "angle_min", "angle_increment", "range_count"])

        self.bridge = CvBridge()
        self.rgb_count = 0
        self.depth_count = 0

        # Subscriptions (all RELIABLE)
        self.create_subscription(Image,       "/ns_rgb/image",          self.cb_rgb,    QOS_RELIABLE)
        self.create_subscription(Image,       "/ns_depth/depth_image",  self.cb_depth,  QOS_RELIABLE)
        self.create_subscription(Imu,         "/ns/imu",                self.cb_imu,    QOS_RELIABLE)
        self.create_subscription(MagneticField,"/ns/mag",               self.cb_mag,    QOS_RELIABLE)
        self.create_subscription(NavSatFix,   "/ns/navsat",             self.cb_gps,    QOS_RELIABLE)
        self.create_subscription(LaserScan,   "/ns_lidar",              self.cb_lidar,  QOS_RELIABLE)

        self.get_logger().info(f"Exporting to: {self.out}")

    # ---------- Callbacks ----------
    def cb_rgb(self, msg: Image):
        t = stamp_to_sec(msg.header.stamp)
        cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        fn = self.out / "rgb" / f"{t:.9f}.png"
        cv2.imwrite(str(fn), cv)
        self.rgb_count += 1

    def cb_depth(self, msg: Image):
        t = stamp_to_sec(msg.header.stamp)
        cv = self.bridge.imgmsg_to_cv2(msg)  # keep native type/encoding
        # Save as 16-bit PNG in millimeters when float depth is provided
        if cv.dtype in (np.float32, np.float64):
            depth_mm = np.clip(cv * 1000.0, 0, 65535).astype(np.uint16)
        elif cv.dtype == np.uint16:
            depth_mm = cv
        else:
            depth_mm = cv.astype(np.uint16)
        fn = self.out / "depth" / f"{t:.9f}.png"
        cv2.imwrite(str(fn), depth_mm)
        self.depth_count += 1

    def cb_imu(self, msg: Imu):
        t = stamp_to_sec(msg.header.stamp)
        self.imu_w.writerow([
            t,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
        ])

    def cb_mag(self, msg: MagneticField):
        t = stamp_to_sec(msg.header.stamp)
        self.mag_w.writerow([t, msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])

    def cb_gps(self, msg: NavSatFix):
        t = stamp_to_sec(msg.header.stamp)
        # Avoid truth-testing arrays; convert safely to a list first
        raw_cov = getattr(msg, "position_covariance", None)
        cov_seq = list(raw_cov) if raw_cov is not None else []
        cov_xx = float(cov_seq[0]) if len(cov_seq) > 0 else 0.0
        self.gps_w.writerow([t, msg.latitude, msg.longitude, msg.altitude, cov_xx])

    def cb_lidar(self, msg: LaserScan):
        t = stamp_to_sec(msg.header.stamp)
        self.lidar_w.writerow([t, msg.angle_min, msg.angle_increment, len(msg.ranges)])

    # ---------- teardown ----------
    def close(self):
        for f in (self._imu_f, self._mag_f, self._gps_f, self._lidar_f):
            try:
                f.flush()
                f.close()
            except Exception:
                pass


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="Output dataset directory")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = ExportNode(out)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"RGB saved: {node.rgb_count}  DEPTH saved: {node.depth_count}")
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
