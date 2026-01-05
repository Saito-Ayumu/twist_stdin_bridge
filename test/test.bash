#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

set -e  # 失敗したら落ちる（テストとして大事）

dir="$HOME"
[ "${1:-}" != "" ] && dir="$1"

cd "$dir/ros2_ws"

# --- ROS setup (ここだけ nounset を無効化) ---
set +u
if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
else
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
# shellcheck disable=SC1090
source "$dir/ros2_ws/install/setup.bash"
set -u
# --------------------------------------------

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
timeout 5 ros2 topic echo "$TOPIC" > "$echo_file" &
ECHO_PID=$!
sleep 1.0  # 発見待ちを少し長めに

printf "bad\n0.1 0.2\n" | timeout 5 ros2 run twist_stdin_bridge stdin_to_twist \
  --ros-args -p topic:="$TOPIC" 1>/dev/null 2>"$err_file" || true

wait "$ECHO_PID" 2>/dev/null || true

grep -q '^parse_error:' "$err_file"

# 表記ゆれに強く（0.100000001 でもOK）
grep -Eq '^[[:space:]]*x:[[:space:]]*0\.1[0-9]*([eE][-+]?[0-9]+)?$' "$echo_file"
grep -Eq '^[[:space:]]*z:[[:space:]]*0\.2[0-9]*([eE][-+]?[0-9]+)?$' "$echo_file"

# ---------------------------------------
# 2) twist_to_stdout: topic -> stdout を確認
# ---------------------------------------
timeout 5 ros2 run twist_stdin_bridge twist_to_stdout \
  --ros-args -p topic:="$TOPIC" > "$stdout_file" &
STDOUT_PID=$!
sleep 1.0

timeout 5 ros2 topic pub -1 "$TOPIC" geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.2}}" 1>/dev/null

wait "$STDOUT_PID" 2>/dev/null || true

# CSVも桁ゆれOK
grep -Eq '^0\.1[0-9]*,0\.2[0-9]*' "$stdout_file"

echo "OK"

