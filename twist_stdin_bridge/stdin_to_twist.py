#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

import sys
import time
from typing import Iterable, Iterator, Optional, TextIO

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .parse import parse_vw_line


def line_to_twist(line: str, err_stream: TextIO) -> Optional[Twist]:
    """Convert one line to Twist. Return None on parse error."""
    try:
        vw = parse_vw_line(line)
    except ValueError as e:
        print(f'parse_error: {e}', file=err_stream, flush=True)
        return None

    msg = Twist()
    msg.linear.x = vw.vx
    msg.angular.z = vw.wz
    return msg


def lines_to_twists(lines: Iterable[str], err_stream: TextIO) -> Iterator[Twist]:
    """Convert lines to Twist messages, skipping invalid lines."""
    for line in lines:
        msg = line_to_twist(line, err_stream)
        if msg is not None:
            yield msg


class StdinToTwist(Node):
    """Read STDIN and publish Twist messages."""

    def __init__(self) -> None:
        super().__init__('stdin_to_twist')
        self.declare_parameter('topic', '/cmd_vel')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self._pub = self.create_publisher(Twist, topic, 10)

    def wait_for_subscriber(self, timeout_sec: float = 1.0) -> None:
        """Wait a short time for a subscriber to be discovered."""
        start = time.time()
        while self._pub.get_subscription_count() == 0 and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

def run(self) -> int:
    """Read STDIN lines and publish Twist messages until EOF."""
    self.wait_for_subscriber(timeout_sec=2.0)

    for msg in lines_to_twists(sys.stdin, sys.stderr):
        self._pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.01)

    # allow last message to be delivered
    for _ in range(10):
        rclpy.spin_once(self, timeout_sec=0.02)
    return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StdinToTwist()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
    raise SystemExit(code)
