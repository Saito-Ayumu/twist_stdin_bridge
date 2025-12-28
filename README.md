# twist_stdin_bridge

`twist_stdin_bridge` は、UNIX の標準入出力（パイプ）と ROS 2 の `geometry_msgs/msg/Twist` を橋渡しし、ROS 2 ノードを新しく書かなくても `/cmd_vel` を入出力できるようにします。

デバッグやスクリプト化に便利です：
- STDIN から `/cmd_vel` を publish（手入力や生成した数値列をそのまま送信）
- `/cmd_vel` を機械可読な CSV として STDOUT に出力し、UNIX ツールでログ保存・解析が可能

## ノード

### stdin_to_twist
STDIN から Twist メッセージを読み取り、`/cmd_vel` に publish します。

- Publish 先: `/cmd_vel`（`geometry_msgs/msg/Twist`）
- 入力（STDIN）: 1 行につき 1 メッセージ。形式は `vx wz` または `vx,wz`
- 出力:
  - STDOUT には何も出しません
  - パースエラーは STDERR に出力します（STDOUT は機械処理用にクリーンに保ちます）

### twist_to_stdout
`/cmd_vel` を購読し、STDOUT に CSV として出力します。

- Subscribe 先: `/cmd_vel`（`geometry_msgs/msg/Twist`）
- 出力（STDOUT）: `vx,wz` + 改行（数値のみの CSV）

## 使い方

ターミナルA（`/cmd_vel` を表示）:
```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run twist_stdin_bridge twist_to_stdout

