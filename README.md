# twist_stdin_bridge

`twist_stdin_bridge` は、UNIXの標準入出力（パイプ）と ROS 2 の `geometry_msgs/msg/Twist` を橋渡しし、ROS 2 ノードを新しく書かなくても `/cmd_vel` を入出力できるようにします。

デバッグやスクリプト化に便利です：
- STDINから `/cmd_vel` をpublish（手入力や生成した数値列をそのまま送信）
- `/cmd_vel` を機械可読なCSVとしてSTDOUTに出力し、UNIXツールでログ保存・解析が可能

## Nodes

### stdin_to_twist
Publish Twist messages from STDIN.

- Publishes: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Input (STDIN): one message per line: `vx wz` or `vx,wz`
- Output:
  - nothing to STDOUT
  - parse errors are printed to STDERR (STDOUT is kept clean)

### twist_to_stdout
Print Twist messages to STDOUT.

- Subscribes: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Output (STDOUT): CSV `vx,wz` + newline (numbers only)

## Usage

Terminal A (print `/cmd_vel`):
```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run twist_stdin_bridge twist_to_stdout

