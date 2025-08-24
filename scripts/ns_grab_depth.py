#!/usr/bin/env python3
import sys, time, numpy as np, cv2, rclpy
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

QOS_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

class GrabDepth(Node):
    def __init__(self, topic, outpath):
        super().__init__('ns_grab_depth')
        self.br = CvBridge()
        self.out = Path(outpath)
        self.sub = self.create_subscription(Image, topic, self.cb, QOS_RELIABLE)
        self.got = False

    def cb(self, msg):
        if self.got: 
            return
        cv = self.br.imgmsg_to_cv2(msg)
        if cv.dtype in (np.float32, np.float64):
            cv = np.clip(cv*1000.0, 0, 65535).astype(np.uint16)
        elif cv.dtype != np.uint16:
            cv = cv.astype(np.uint16)
        self.out.parent.mkdir(parents=True, exist_ok=True)
        ok = cv2.imwrite(str(self.out), cv)
        self.get_logger().info(f'Wrote {self.out} ok={ok}')
        self.got = True
        rclpy.shutdown()

def main():
    topic = sys.argv[1] if len(sys.argv)>1 else '/ns_depth/depth_image'
    out   = sys.argv[2] if len(sys.argv)>2 else '/tmp/ns_depth_one.png'
    rclpy.init()
    node = GrabDepth(topic, out)
    t0 = time.time()
    while rclpy.ok() and time.time()-t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)

if __name__ == '__main__':
    main()
