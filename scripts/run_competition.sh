#!/usr/bin/env bash
# RMUL 2025 正赛启动脚本 (Nav2 + 正赛控制器)
#
# 采用最稳妥的 "脚本 + launch 文件" 方式:
#   1. source 工作空间
#   2. ros2 launch → Nav2 (slam=True, 在线建图) + competition_controller.py
#
# Usage:
#   bash ~/sentry_ws/scripts/run_competition.sh                          # 默认地图
#   bash ~/sentry_ws/scripts/run_competition.sh reserve/field_training_latest  # 指定地图

WS=/home/soyo/sentry_ws
WORLD="${1:-reserve/field_training_latest}"

set +u
source "$WS/install/setup.bash"
set -u

echo "=== RMUL 2025 正赛 ==="
echo "Map world: $WORLD"
echo "Launch:    $WS/scripts/competition_launch.py"
echo "========================"

ros2 launch "$WS/scripts/competition_launch.py" \
    log_level:=info world:="$WORLD" || true

echo "=== 正赛脚本退出 ==="
