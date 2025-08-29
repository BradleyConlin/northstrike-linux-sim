#!/usr/bin/env python3
import time
import argparse
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def now_sec(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9

class DepthGateLZ(Node):
    def __init__(self, args):
        super().__init__('ns_depth_gate_lz')
        self.bridge = CvBridge()
        self.depth_topic = args.depth_in
        self.mask_topic  = args.mask_in
        self.out_depth   = args.depth_out
        self.out_color   = args.depth_out_color
        self.max_depth_m = args.max_depth_m
        self.max_skew_s  = args.max_skew_s

        self.last_mask = None
        self.last_mask_stamp = 0.0
        self.mask_shape = None

        self.create_subscription(Image, self.mask_topic,  self.cb_mask,  10)
        self.create_subscription(Image, self.depth_topic, self.cb_depth, 10)
        self.pub_depth = self.create_publisher(Image, self.out_depth, 10)
        self.pub_color = self.create_publisher(Image, self.out_color, 10)

        self.get_logger().info(
            f"Gate ready: depth_in={self.depth_topic} mask_in={self.mask_topic} "
            f"→ {self.out_depth}, {self.out_color}; max_depth={self.max_depth_m} m, skew<={self.max_skew_s}s"
        )

    def cb_mask(self, msg: Image):
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            self.last_mask = mask
            self.last_mask_stamp = now_sec(msg.header)
            self.mask_shape = mask.shape[:2]
        except Exception as e:
            self.get_logger().warn(f"mask convert failed: {e}")

    def cb_depth(self, msg: Image):
        # depth: 16UC1 in millimeters
        try:
            d16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().warn(f"depth convert failed: {e}")
            return

        H, W = d16.shape[:2]
        gated = d16

        ok = False
        if self.last_mask is not None and (time.time() - self.last_mask_stamp) <= self.max_skew_s:
            mask = self.last_mask
            if mask.shape[:2] != (H, W):
                mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)
            # gate: keep depth where mask>0
            gated = (d16 * (mask > 0)).astype(np.uint16)
            ok = True

        # publish gated depth
        dmsg = self.bridge.cv2_to_imgmsg(gated, encoding='16UC1')
        dmsg.header = msg.header
        self.pub_depth.publish(dmsg)

        # colorize for visualization
        max_mm = int(self.max_depth_m * 1000.0)
        mm = np.clip(gated, 0, max_mm)
        # map 0..max_mm → 0..255 for colormap
        depth8 = (mm.astype(np.float32) * (255.0 / max_mm)).astype(np.uint8)
        color = cv2.applyColorMap(depth8, cv2.COLORMAP_TURBO)

        # if mask present, dim outside-LZ a bit for clarity (optional)
        if ok:
            mask3 = (mask > 0)[:, :, None].astype(np.uint8)
            color = (color * (0.35 + 0.65 * mask3)).astype(np.uint8)

        cmsg = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        cmsg.header = msg.header
        self.pub_color.publish(cmsg)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--depth_in', default='/ns_depth/pred')
    ap.add_argument('--mask_in',  default='/ns_lz/mask')
    ap.add_argument('--depth_out', default='/ns_depth/pred_gated')
    ap.add_argument('--depth_out_color', default='/ns_depth/pred_gated_color')
    ap.add_argument('--max_depth_m', type=float, default=20.0)
    ap.add_argument('--max_skew_s',  type=float, default=0.20)  # up to 200ms skew
    args = ap.parse_args()

    rclpy.init()
    node = DepthGateLZ(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

