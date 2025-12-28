# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .parse import parse_vw_line


class StdinToTwist(Node):

    def __init__(self):
        super().__init__('stdin_to_twist')
        self.declare_parameter('topic', '/cmd_vel')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.pub = self.create_publisher(Twist, topic, 10)

    def wait_for_subscriber(self, timeout_sec: float = 1.0) -> None:
        """Wait a short time for a subscriber to be discovered."""
        start = time.time()
        while self.pub.get_subscription_count() == 0 and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self) -> int:
        """Read STDIN lines and publish Twist messages until EOF."""
        self.wait_for_subscriber(timeout_sec=1.0)

        for line in sys.stdin:
            try:
                vw = parse_vw_line(line)
            except Exception as e:
                print(f'parse_error: {e}', file=sys.stderr, flush=True)
                continue

            msg = Twist()
            msg.linear.x = vw.vx
            msg.angular.z = vw.wz
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)

        # Give DDS a short time to flush outgoing data for short-lived runs.
        time.sleep(0.2)
        return 0


def main() -> None:
    rclpy.init()
    node = StdinToTwist()
    try:
        try:
            code = node.run()
        except KeyboardInterrupt:
            code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
