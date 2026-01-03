# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

import io

from geometry_msgs.msg import Twist

from twist_stdin_bridge.stdin_to_twist import lines_to_twists
from twist_stdin_bridge.twist_to_stdout import twist_to_csv_line


def test_stdin_to_twist_io() -> None:
    """Check stdin parsing -> Twist generation and stderr output."""
    err = io.StringIO()
    lines = ['bad\n', '0.1 0.2\n', '0.0,-0.5\n']

    msgs = list(lines_to_twists(lines, err))

    # good lines -> 2 Twist messages
    assert len(msgs) == 2
    assert abs(msgs[0].linear.x - 0.1) < 1e-9
    assert abs(msgs[0].angular.z - 0.2) < 1e-9
    assert abs(msgs[1].linear.x - 0.0) < 1e-9
    assert abs(msgs[1].angular.z - (-0.5)) < 1e-9

    # bad line -> error to stderr
    assert 'parse_error:' in err.getvalue()


def test_twist_to_stdout_io() -> None:
    """Check Twist -> CSV formatting (stdout line)."""
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.5

    assert twist_to_csv_line(msg) == '0.100000,-0.500000'
