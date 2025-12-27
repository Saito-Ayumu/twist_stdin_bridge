# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

import sys

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

    def run(self) -> int:
        """Read STDIN lines and publish Twist messages until EOF."""
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

        return 0


def main() -> None:
    rclpy.init()
    node = StdinToTwist()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)
