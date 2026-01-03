#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

set -eu

# 講義の例に合わせて「引数があれば HOME 扱い」(GitHub Actions等で使いやすい)
# 例: bash test/test.bash ~
dir="$HOME"
[ "${1:-}" != "" ] && dir="$1"

cd "$dir/ros2_ws"

# ROS環境（ローカルでもActionsでも動くようにする）
# ROS_DISTRO が無ければ humble 扱い（Actionsはhumble）
if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
else
  # 念のため
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

# shellcheck disable=SC1090
source "$dir/ros2_ws/install/setup.bash"

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

outA="$tmp/outA.txt"
errB="$tmp/errB.txt"
outA2="$tmp/outA2.txt"
errB2="$tmp/errB2.txt"

# -----------------------------
# 正常系: B -> /cmd_vel publish, A が CSV を表示
# -----------------------------
timeout 3 ros2 run twist_stdin_bridge twist_to_stdout > "$outA" &
sleep 0.5

printf "0.1 0.2\n0.0 -0.5\n" | ros2 run twist_stdin_bridge stdin_to_twist 1>/dev/null 2>"$errB"

sleep 0.5

# A側に2行出てることを確認（厳密一致）
grep -x '0.100000,0.200000' "$outA"
grep -x '0.000000,-0.500000' "$outA"

# 正常入力では STDERR が空
test ! -s "$errB"

# -----------------------------
# 異常系: bad を混ぜると B の STDERR に parse_error が出る
# -----------------------------
timeout 3 ros2 run twist_stdin_bridge twist_to_stdout > "$outA2" &
sleep 0.5

printf "bad\n0.1 0.2\n" | ros2 run twist_stdin_bridge stdin_to_twist 1>/dev/null 2>"$errB2"

sleep 0.5

# good 行は通って出る
grep -x '0.100000,0.200000' "$outA2"

# bad は parse_error が出る（内容は多少変わっても良いので前方一致で見る）
grep -q '^parse_error:' "$errB2"

echo "OK"

