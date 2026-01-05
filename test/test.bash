#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

set -eu

dir="$HOME"
[ "${1:-}" != "" ] && dir="$1"

cd "$dir/ros2_ws"

# ROS setup
if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
else
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

# shellcheck disable=SC1090
source "$dir/ros2_ws/install/setup.bash"

TOPIC="/cmd_vel"

tmp="$(mktemp -d)"
echo_file="$tmp/echo.txt"
stdout_file="$tmp/stdout.txt"
err_file="$tmp/err.txt"
trap '
  kill "${ECHO_PID:-}" "${STDOUT_PID:-}" 2>/dev/null || true
  rm -rf "$tmp"
' EXIT

# ---------------------------------------
# 1) stdin_to_twist: stdin -> topic を確認
# ---------------------------------------
# topic をファイルへ（一定時間で止める）
timeout 3 ros2 topic echo "$TOPIC" > "$echo_file" &
ECHO_PID=$!
sleep 0.5

# stdin_to_twist に入力（bad混ぜ）
# good: vx=0.1 wz=0.2 が publish される想定
printf "bad\n0.1 0.2\n" | timeout 3 ros2 run twist_stdin_bridge stdin_to_twist \
  --ros-args -p topic:="$TOPIC" 1>/dev/null 2>"$err_file" || true

wait "$ECHO_PID" 2>/dev/null || true

# bad は parse_error が出る
grep -q '^parse_error:' "$err_file"

# topic で Twist を受け取れている（表記ゆれに強め）
# ros2 topic echo は YAML っぽく出るので "x: 0.1" / "z: 0.2" を見る
grep -Eq '^[[:space:]]*x:[[:space:]]*0\.1(0*)?$' "$echo_file"
grep -Eq '^[[:space:]]*z:[[:space:]]*0\.2(0*)?$' "$echo_file"

# ---------------------------------------
# 2) twist_to_stdout: topic -> stdout を確認
# ---------------------------------------
timeout 3 ros2 run twist_stdin_bridge twist_to_stdout \
  --ros-args -p topic:="$TOPIC" > "$stdout_file" &
STDOUT_PID=$!
sleep 0.5

# topic に1回だけ publish
timeout 3 ros2 topic pub -1 "$TOPIC" geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.2}}" 1>/dev/null

wait "$STDOUT_PID" 2>/dev/null || true

# stdout が CSV で出ている（format の桁が変わってもOKにしたいなら前方一致が安全）
# 例: 0.100000,0.200000 など
grep -Eq '^0\.1(0*)?,0\.2(0*)?' "$stdout_file"

echo "OK"

