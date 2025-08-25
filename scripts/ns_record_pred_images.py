#!/usr/bin/env python3
import argparse, os, rclpy, numpy as np, cv2
from pathlib import Path
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class PredRecorder(Node):
    def __init__(self, out_dir, pred_topic, color_topic, stride, limit, max_depth_m):
        super().__init__('ns_pred_recorder')
        self.out = Path(out_dir); (self.out/'pred').mkdir(parents=True, exist_ok=True); (self.out/'pred_color').mkdir(exist_ok=True)
        self.bridge = CvBridge()
        self.stride = max(1, stride)
        self.limit = limit
        self.max_depth_m = float(max_depth_m)
        self.count = 0
        self.every = 0
        self.sub_pred  = self.create_subscription(Image, pred_topic,  self.cb_pred,  10)
        self.sub_color = self.create_subscription(Image, color_topic, self.cb_color, 10)
        self.get_logger().info(f"Recording to {self.out} (pred in mm, color JPG). stride={self.stride} limit={self.limit or '∞'}")

    def _stamp_name(self, hdr):
        return f"{hdr.stamp.sec}.{hdr.stamp.nanosec:09d}"

    def cb_pred(self, msg: Image):
        if self.limit and self.count >= self.limit: return
        self.every += 1
        if self.every % self.stride: return
        name = self._stamp_name(msg.header)
        # 32FC1 meters -> uint16 millimeters PNG
        depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        depth_mm = np.clip(depth_m * 1000.0, 0, self.max_depth_m*1000.0).astype(np.uint16)
        cv2.imwrite(str(self.out/'pred'/f"{name}.png"), depth_mm)
        self.count += 1

    def cb_color(self, msg: Image):
        if self.limit and self.count >= self.limit: return
        name = self._stamp_name(msg.header)
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(str(self.out/'pred_color'/f"{name}.jpg"), img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='output folder')
    ap.add_argument('--pred_topic',  default='/ns_depth/pred')
    ap.add_argument('--color_topic', default='/ns_depth/pred_color')
    ap.add_argument('--stride', type=int, default=1, help='save every Nth frame')
    ap.add_argument('--limit',  type=int, default=0, help='max frames to save (0=all)')
    ap.add_argument('--max_depth_m', type=float, default=20.0)
    args = ap.parse_args()
    rclpy.init()
    node = PredRecorder(args.out, args.pred_topic, args.color_topic, args.stride, args.limit, args.max_depth_m)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except: pass
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()
