import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/saito/ros2_ws/src/twist_stdin_bridge/install/twist_stdin_bridge'
