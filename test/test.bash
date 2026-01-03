#!/bin/bash
set -eu

# 第10回の例に合わせて、引数があればそれをホーム扱いにする :contentReference[oaicite:2]{index=2}
dir="$HOME"
[ "${1:-}" != "" ] && dir="$1"

# ワークスペースは /root/ros2_ws（講義コンテナ）想定。自分の環境でも引数なしで動く
cd "$dir/ros2_ws"

# ROSのセットアップ（コンテナはhumble想定）
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$dir/ros2_ws/install/setup.bash"

tmp="$(mktemp -d)"
out1="$tmp/out1.txt"
err1="$tmp/err1.txt"
out2="$tmp/out2.txt"
err2="$tmp/err2.txt"

# ---- 正常系：手入力相当を送って、表示側にCSVが出るか ----
timeout 3 ros2 run twist_stdin_bridge twist_to_stdout > "$out1" &
sleep 0.5

printf "0.1 0.2\n0.0 -0.5\n" | ros2 run twist_stdin_bridge stdin_to_twist 1>/dev/null 2>"$err1"

sleep 0.5

grep -x '0.100000,0.200000' "$out1"
grep -x '0.000000,-0.500000' "$out1"
test ! -s "$err1"

# ---- エラー系：bad を混ぜたら STDERR に parse_error が出るか ----
timeout 3 ros2 run twist_stdin_bridge twist_to_stdout > "$out2" &
sleep 0.5

printf "bad\n0.1 0.2\n" | ros2 run twist_stdin_bridge stdin_to_twist 1>/dev/null 2>"$err2"

sleep 0.5

grep -x '0.100000,0.200000' "$out2"
grep -q 'parse_error:' "$err2"

echo "OK"

