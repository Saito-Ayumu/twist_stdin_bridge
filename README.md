- このソフトウェアパッケージは，3条項BSDライセンスの下，再頒
布および使用が許可されます．
- 本パッケージは，第三者の著作物コードを含みません．
- © 2025 Ayumu Saito

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

## 以降の例ではワークスペースを `~/ros2_ws` としています。自分の環境のワークスペース名に読み替えてください。

## インストール / ビルド

例としてワークスペースを `~/ros2_ws` とします（名前は任意）。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Saito-Ayumu/twist_stdin_bridge.git
cd ~/ros2_ws
```

# 依存関係
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# ビルド
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash


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
```text
parse_error: need two values: vx wz
```

