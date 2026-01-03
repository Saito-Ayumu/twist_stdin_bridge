#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

import sys

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .parse import format_vw_csv, Vw


def twist_to_vw(msg: Twist) -> Vw:
    """Convert Twist to (vx, wz)."""
    return Vw(float(msg.linear.x), float(msg.angular.z))


def twist_to_csv_line(msg: Twist) -> str:
    """Format Twist as one CSV line (numbers only)."""
    return format_vw_csv(twist_to_vw(msg)).rstrip('\n')


class TwistToStdout(Node):
    """Subscribe Twist and print it to STDOUT as CSV."""

    def __init__(self) -> None:
        super().__init__('twist_to_stdout')
        self.declare_parameter('topic', '/cmd_vel')
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # Keep reference to avoid garbage collection.
        self._sub = self.create_subscription(Twist, topic, self.cb, 10)

    def cb(self, msg: Twist) -> None:
        """Write Twist values to STDOUT as CSV (numbers only)."""
        sys.stdout.write(twist_to_csv_line(msg) + '\n')
        sys.stdout.flush()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwistToStdout()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # ExternalShutdownException を import せずに抑える場合
        if e.__class__.__name__ == "ExternalShutdownException":
            pass
        else:
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

