#!/usr/bin/env python3
import sys, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

QOS_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

class Grab(Node):
    def __init__(self, topic, outpath):
        super().__init__('ns_grab_one')
        self.br = CvBridge()
        self.out = Path(outpath)
        self.sub = self.create_subscription(Image, topic, self.cb, QOS_RELIABLE)
        self.got = False
        self.get_logger().info(f'Waiting for one message on {topic} ...')

    def cb(self, msg):
        if self.got: 
            return
        try:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.out.parent.mkdir(parents=True, exist_ok=True)
            ok = cv2.imwrite(str(self.out), img)
            self.get_logger().info(f'Wrote {self.out} ok={ok}')
            self.got = True
        except Exception as e:
            self.get_logger().error(f'cv_bridge/cv2 failed: {e}')
        rclpy.shutdown()

def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/ns_rgb/image'
    out   = sys.argv[2] if len(sys.argv) > 2 else '/tmp/ns_rgb_one.png'
    rclpy.init()
    node = Grab(topic, out)
    t0 = time.time()
    while rclpy.ok() and time.time()-t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)

if __name__ == '__main__':
    main()
