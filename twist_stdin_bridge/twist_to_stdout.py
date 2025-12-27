# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

import sys

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .parse import format_vw_csv, Vw


class TwistToStdout(Node):
    def __init__(self):
        super().__init__('twist_to_stdout')
        self.declare_parameter('topic', '/cmd_vel')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.create_subscription(Twist, topic, self.cb, 10)

    def cb(self, msg: Twist) -> None:
        """Write Twist values to STDOUT as CSV (numbers only)."""
        vw = Vw(float(msg.linear.x), float(msg.angular.z))
        sys.stdout.write(format_vw_csv(vw))
        sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = TwistToStdout()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
