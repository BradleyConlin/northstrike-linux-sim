#!/usr/bin/env python3
import csv
from pathlib import Path
from typing import Dict

import numpy as np
import cv2
from cv_bridge import CvBridge

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RGB_TOPIC   = "/ns_rgb/image"
DEPTH_TOPIC = "/ns_depth/depth_image"
IMU_TOPIC   = "/ns/imu"
MAG_TOPIC   = "/ns/mag"
GPS_TOPIC   = "/ns/navsat"
LIDAR_TOPIC = "/ns_lidar"

def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9 if stamp else 0.0

def open_reader(bag_dir: str) -> tuple[rosbag2_py.SequentialReader, Dict[str, type]]:
    storage = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    type_map = {name: get_message(typ) for name, typ in topic_types.items()}
    return reader, type_map

def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--bag", required=True, help="Path to the rosbag2 directory (contains *_0.db3)")
    p.add_argument("--out", required=True, help="Output dataset directory")
    args = p.parse_args()

    out = Path(args.out)
    (out / "rgb").mkdir(parents=True, exist_ok=True)
    (out / "depth").mkdir(parents=True, exist_ok=True)

    # CSVs
    imu_f   = open(out / "imu.csv",   "w", newline="")
    mag_f   = open(out / "mag.csv",   "w", newline="")
    gps_f   = open(out / "gps.csv",   "w", newline="")
    lidar_f = open(out / "lidar.csv", "w", newline="")
    imu_w, mag_w, gps_w, lidar_w = map(csv.writer, (imu_f, mag_f, gps_f, lidar_f))
    imu_w.writerow(["t_sec","ori_x","ori_y","ori_z","ori_w","ang_x","ang_y","ang_z","lin_x","lin_y","lin_z"])
    mag_w.writerow(["t_sec","mag_x","mag_y","mag_z"])
    gps_w.writerow(["t_sec","lat","lon","alt","cov_xx"])
    lidar_w.writerow(["t_sec","angle_min","angle_increment","range_count"])

    bridge = CvBridge()
    rgb_count = depth_count = 0

    reader, type_map = open_reader(args.bag)

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in type_map:
            continue
        msg = deserialize_message(raw, type_map[topic])

        if topic == RGB_TOPIC:
            t = stamp_to_sec(msg.header.stamp)
            cv = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok = cv2.imwrite(str(out / "rgb" / f"{t:.9f}.png"), cv)
            if ok: rgb_count += 1

        elif topic == DEPTH_TOPIC:
            t = stamp_to_sec(msg.header.stamp)
            cv = bridge.imgmsg_to_cv2(msg)  # keep native type
            if cv.dtype in (np.float32, np.float64):
                depth_mm = np.clip(cv * 1000.0, 0, 65535).astype(np.uint16)
            elif cv.dtype == np.uint16:
                depth_mm = cv
            else:
                depth_mm = cv.astype(np.uint16)
            ok = cv2.imwrite(str(out / "depth" / f"{t:.9f}.png"), depth_mm)
            if ok: depth_count += 1

        elif topic == IMU_TOPIC:
            t = stamp_to_sec(msg.header.stamp)
            imu_w.writerow([t,
                            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        elif topic == MAG_TOPIC:
            t = stamp_to_sec(msg.header.stamp)
            mag_w.writerow([t, msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])

        elif topic == GPS_TOPIC:
            t = stamp_to_sec(msg.header.stamp)
            cov = list(getattr(msg, "position_covariance", []))
            cov_xx = cov[0] if len(cov) > 0 else 0.0
            gps_w.writerow([t, msg.latitude, msg.longitude, msg.altitude, cov_xx])

        elif topic == LIDAR_TOPIC:
            t = stamp_to_sec(msg.header.stamp)
            lidar_w.writerow([t, msg.angle_min, msg.angle_increment, len(msg.ranges)])

    print(f"RGB saved: {rgb_count}  DEPTH saved: {depth_count}")

    for f in (imu_f, mag_f, gps_f, lidar_f):
        try: f.close()
        except Exception: pass

if __name__ == "__main__":
    main()
