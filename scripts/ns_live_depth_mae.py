#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dataclasses import dataclass

@dataclass
class Buff:
    img: np.ndarray = None
    t: float = 0.0     # header stamp in seconds
    enc: str = ""

class LiveMAENode(Node):
    def __init__(self):
        super().__init__('ns_live_depth_mae')
        self.bridge = CvBridge()
        self.max_mm = int(float(self.declare_parameter('max_depth_m', 20.0).value) * 1000.0)
        self.max_skew = float(self.declare_parameter('max_skew_s', 0.10).value)
        self.pred = Buff(); self.gt = Buff()
        self.sub_pred = self.create_subscription(Image, '/ns_depth/pred', self.cb_pred, 10)
        self.sub_gt   = self.create_subscription(Image, '/ns_depth/depth_image', self.cb_gt, 10)
        self.pub      = self.create_publisher(Float32, '/ns_depth/live_mae_m', 10)
        self.timer    = self.create_timer(0.10, self.on_tick)  # 10 Hz compute
        self.logged_pred_once = False
        self.logged_gt_once = False
        self.get_logger().info(f"Live MAE up. max_depth={self.max_mm} mm, skew<={self.max_skew}s")

    def _stamp_to_sec(self, msg: Image) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def img_to_mm(self, msg: Image, is_pred: bool):
        enc = (msg.encoding or "").lower()
        arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if '16u' in enc:
            if arr.dtype != np.uint16: arr = arr.astype(np.uint16)
            mm = arr
        elif '32f' in enc:
            mm = np.clip(arr * 1000.0, 0, self.max_mm).astype(np.uint16)  # meters->mm
        else:
            if arr.dtype != np.uint16: arr = arr.astype(np.uint16)
            mm = arr
        if is_pred and not self.logged_pred_once:
            self.get_logger().info(f"pred enc={enc or '<?>'} dtype={arr.dtype} min={int(mm.min())} max={int(mm.max())}")
            self.logged_pred_once = True
        if (not is_pred) and not self.logged_gt_once:
            self.get_logger().info(f"gt   enc={enc or '<?>'} dtype={arr.dtype} min={int(mm.min())} max={int(mm.max())}")
            self.logged_gt_once = True
        return mm, enc

    def cb_pred(self, msg: Image):
        mm, enc = self.img_to_mm(msg, True)
        self.pred.img = mm; self.pred.t = self._stamp_to_sec(msg); self.pred.enc = enc

    def cb_gt(self, msg: Image):
        mm, enc = self.img_to_mm(msg, False)
        self.gt.img = mm; self.gt.t = self._stamp_to_sec(msg); self.gt.enc = enc

    def on_tick(self):
        if self.pred.img is None or self.gt.img is None: return
        if abs(self.pred.t - self.gt.t) > self.max_skew: return
        a, b = self.pred.img, self.gt.img
        if a.shape != b.shape:
            import cv2
            b = cv2.resize(b, (a.shape[1], a.shape[0]), interpolation=cv2.INTER_NEAREST)
        valid = (a > 0) & (b > 0) & (a <= self.max_mm) & (b <= self.max_mm)
        if not np.any(valid): return
        mae_m = float(np.mean(np.abs(a[valid].astype(np.int32) - b[valid].astype(np.int32))) / 1000.0)
        self.pub.publish(Float32(data=mae_m))

def main():
    rclpy.init()
    node = LiveMAENode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
