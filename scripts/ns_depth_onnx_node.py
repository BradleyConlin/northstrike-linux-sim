#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
import cv2
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthONNXNode(Node):
    def __init__(self, model_path, in_topic, out_topic, size, max_depth_m):
        super().__init__("ns_depth_onnx")
        self.bridge = CvBridge()
        self.size = tuple(size)           # (W, H)
        self.max_depth_m = float(max_depth_m)

        self.session = ort.InferenceSession(str(model_path))
        self.inp_name = self.session.get_inputs()[0].name

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Image, in_topic, self.cb, qos_profile=qos)
        self.pub_depth = self.create_publisher(Image, out_topic, qos)
        self.pub_color = self.create_publisher(Image, out_topic + "_color", qos)

        self.get_logger().info(
            f"Loaded {model_path}; subscribe {in_topic} -> publish {out_topic} (+_color) size={self.size}"
        )

    def cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, self.size, interpolation=cv2.INTER_AREA)
        x = (rgb.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]  # 1x3xHxW

        y, = self.session.run(None, {self.inp_name: x})
        pred01 = np.clip(y[0, 0], 0.0, 1.0)

        # Publish depth in millimeters as 16UC1
        pred_mm = np.clip(pred01 * self.max_depth_m * 1000.0, 0, 65535).astype(np.uint16)
        depth_msg = self.bridge.cv2_to_imgmsg(pred_mm, encoding="16UC1")
        depth_msg.header = msg.header
        self.pub_depth.publish(depth_msg)

        # Publish a colorized preview for quick checks
        color8 = (pred01 * 255).astype(np.uint8)
        color_bgr = cv2.applyColorMap(color8, cv2.COLORMAP_TURBO)
        color_msg = self.bridge.cv2_to_imgmsg(color_bgr, encoding="bgr8")
        color_msg.header = msg.header
        self.pub_color.publish(color_msg)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="Path to ONNX model")
    ap.add_argument("--in", dest="in_topic", default="/ns_rgb/image")
    ap.add_argument("--out", dest="out_topic", default="/ns_depth/pred")
    ap.add_argument("--size", type=int, nargs=2, default=(320, 240), help="W H expected by model")
    ap.add_argument("--max_depth_m", type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = DepthONNXNode(
        Path(args.model),
        in_topic=args.in_topic,
        out_topic=args.out_topic,
        size=tuple(args.size),
        max_depth_m=args.max_depth_m,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
