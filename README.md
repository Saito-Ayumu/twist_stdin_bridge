# twist_stdin_bridge

A small ROS 2 utility that bridges UNIX pipelines and `geometry_msgs/msg/Twist`.
It allows you to publish `/cmd_vel` from STDIN and print Twist messages to STDOUT.

## Nodes

### stdin_to_twist
Publishes Twist from STDIN.

- Publishes: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Input (STDIN): one line per message: `vx wz` or `vx,wz`
- Output: none  
  - Parse errors are printed to STDERR (STDOUT is kept machine-readable).

### twist_to_stdout
Prints Twist messages to STDOUT.

- Subscribes: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Output (STDOUT): CSV `vx,wz\n` (numbers only)

## Usage

Publish Twist from STDIN:
```bash
$ printf "0.1 0.2\n0.0 -0.5\n" | ros2 run twist_stdin_bridge stdin_to_twist

## License
BSD-3-Clause

