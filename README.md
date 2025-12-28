# twist_stdin_bridge

Bridge UNIX pipelines and ROS 2 `geometry_msgs/msg/Twist`.

This package provides small CLI-style ROS 2 nodes:
- publish `/cmd_vel` from STDIN
- print `/cmd_vel` to STDOUT (machine-readable CSV)

## Nodes

### stdin_to_twist
Publish Twist messages from STDIN.

- Publishes: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Input (STDIN): one message per line: `vx wz` or `vx,wz`
- Output:
  - none to STDOUT
  - parse errors are printed to STDERR (STDOUT is kept clean)

### twist_to_stdout
Print Twist messages to STDOUT.

- Subscribes: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Output (STDOUT): CSV `vx,wz` + newline (numbers only)

## Usage

Terminal A:
```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run twist_stdin_bridge twist_to_stdout

