#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

UAS_NS = '/mavros'   # <-- using /mavros namespace
HOVER_Z = 3.0
DURATION = 10.0
PUB_HZ = 20.0

class OffboardHover(Node):
    def __init__(self):
        super().__init__('offboard_hover')
        self.state = State()
        self.state_sub = self.create_subscription(
            State, f'{UAS_NS}/state', self._state_cb, 10
        )
        self.sp_pub = self.create_publisher(
            PoseStamped, f'{UAS_NS}/setpoint_position/local', 10
        )
        self.arm_cli = self.create_client(CommandBool, f'{UAS_NS}/cmd/arming')
        self.mode_cli = self.create_client(SetMode,     f'{UAS_NS}/set_mode')

    def _state_cb(self, msg): self.state = msg

    def _call_sync(self, client, req, timeout=5.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def run(self):
        while not self.arm_cli.wait_for_service(timeout_sec=1.0): pass
        while not self.mode_cli.wait_for_service(timeout_sec=1.0): pass

        sp = PoseStamped()
        sp.pose.position.x = 0.0
        sp.pose.position.y = 0.0
        sp.pose.position.z = HOVER_Z

        period = 1.0 / PUB_HZ

        # Prime OFFBOARD: publish setpoints for ~2s (>2Hz)
        for _ in range(int(2.0 * PUB_HZ)):
            sp.header.stamp = self.get_clock().now().to_msg()
            self.sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        # Set OFFBOARD
        mreq = SetMode.Request(); mreq.custom_mode = 'OFFBOARD'
        mres = self._call_sync(self.mode_cli, mreq)
        self.get_logger().info(f'set_mode OFFBOARD -> {getattr(mres, "mode_sent", False)}')

        # Arm
        areq = CommandBool.Request(); areq.value = True
        ares = self._call_sync(self.arm_cli, areq)
        self.get_logger().info(f'arm -> {getattr(ares, "success", False)}')

        # Hold position
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < DURATION:
            sp.header.stamp = self.get_clock().now().to_msg()
            self.sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        # Land
        land = SetMode.Request(); land.custom_mode = 'AUTO.LAND'
        self._call_sync(self.mode_cli, land)
        self.get_logger().info('Requested AUTO.LAND')
        time.sleep(2.0)

def main():
    rclpy.init()
    node = OffboardHover()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
