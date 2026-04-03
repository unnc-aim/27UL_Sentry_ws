#!/usr/bin/env bash
# 简化版正赛测试 (手动启动, 相对坐标 1m 往返, 无裁判/自瞄/射击)
#
# Usage:
#   bash ~/sentry_ws/scripts/run_test_competition.sh

WS=/home/soyo/sentry_ws

set +u
source "$WS/install/setup.bash"
set -u

echo "=== 简化正赛测试 ==="
echo "  导航: SLAM, 相对坐标 1m 往返"
echo "  裁判: 无 | 自瞄: 关 | 射击: 关"
echo "====================="

ros2 launch "$WS/scripts/test_competition_launch.py" log_level:=info || true

echo "=== 测试结束, running make restart ==="
cd "$WS"
make restart
