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

このパッケージは 2 つのノードを使います。  
ターミナルAで `/cmd_vel` を表示しながら、別のターミナルBから `/cmd_vel` に送信します。

### ターミナルA（表示側）
```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run twist_stdin_bridge twist_to_stdout
```

### ターミナルB（送信側）
別のターミナルを開いて、以下を実行して数値を入力します（各行で Enter）:
```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run twist_stdin_bridge stdin_to_twist
0.1 0.2
0.0 -0.5
```

### ターミナルAの出力例
```text
0.100000,0.200000
0.000000,-0.500000
```

### パースエラー例（ターミナルBの STDERR に出ます）
```bash
$ printf "bad\n0.1 0.2\n" | ros2 run twist_stdin_bridge stdin_to_twist 1>/dev/null
```

### STDERR の出力例
parse_error: need two values: vx wz

