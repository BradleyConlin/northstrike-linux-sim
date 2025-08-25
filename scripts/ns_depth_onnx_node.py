#!/usr/bin/env python3
import argparse, os
from pathlib import Path
import numpy as np
import cv2
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthONNXNode(Node):
    def __init__(self, model_path: Path, in_topic: str, out_topic: str,
                 size=(320, 240), max_depth_m=20.0):
        super().__init__('ns_depth_onnx')
        self.size = tuple(size)            # (W, H)
        self.max_depth_m = float(max_depth_m)
        self.bridge = CvBridge()
        self._shutting_down = False
        rclpy.get_default_context().on_shutdown(self._mark_shutdown)

        # ONNX session
        so = ort.SessionOptions()
        so.log_severity_level = 3
        self.sess = ort.InferenceSession(str(model_path), so, providers=['CPUExecutionProvider'])
        self.in_name = self.sess.get_inputs()[0].name

        # I/O
        self.pub_depth = self.create_publisher(Image, out_topic, 10)
        self.pub_color = self.create_publisher(Image, out_topic + "_color", 10)
        self.sub = self.create_subscription(Image, in_topic, self.cb, 10)

        self.get_logger().info(
            f"Loaded {model_path}; subscribe {in_topic} -> publish {out_topic} (+_color) size={self.size}"
        )

    # ---- shutdown & publish helpers ----
    def _mark_shutdown(self):
        self._shutting_down = True

    def _safe_publish(self, pub, msg):
        if self._shutting_down or not rclpy.ok():
            return
        try:
            pub.publish(msg)
        except Exception:
            # Context may already be tearing down; ignore.
            pass

    # ---- callback ----
    def cb(self, msg: Image):
        # BGR8 -> RGB, resize to network size
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, self.size, interpolation=cv2.INTER_AREA)

        x = (rgb.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]  # 1x3xHxW
        y, = self.sess.run(None, {self.in_name: x})
        pred01 = np.clip(y[0, 0], 0.0, 1.0)

        # Depth 0..1 -> millimeters in 16UC1
        depth_mm = (pred01 * (self.max_depth_m * 1000.0)).astype(np.uint16)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
        depth_msg.header = msg.header

        # Nice color preview
        color8 = (pred01 * 255.0).astype(np.uint8)
        color_bgr = cv2.applyColorMap(color8, cv2.COLORMAP_TURBO)
        color_msg = self.bridge.cv2_to_imgmsg(color_bgr, encoding='bgr8')
        color_msg.header = msg.header

        self._safe_publish(self.pub_depth, depth_msg)
        self._safe_publish(self.pub_color, color_msg)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="Path to ONNX model")
    ap.add_argument("--in", dest="in_topic", default="/ns_rgb/image")
    ap.add_argument("--out", dest="out_topic", default="/ns_depth/pred")
    ap.add_argument("--size", type=int, nargs=2, default=(320, 240), help="W H")
    ap.add_argument("--max_depth_m", type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = DepthONNXNode(
        model_path=Path(args.model),
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
