#!/usr/bin/env bash
# Stationary function test (no movement required).
# Usage:
#   bash ~/sentry_ws/scripts/run_simple_nav_test1.sh              # 不等裁判
#   bash ~/sentry_ws/scripts/run_simple_nav_test1.sh --referee     # 等待裁判系统比赛开始

WS=/home/soyo/sentry_ws
WAIT_REF=false

for arg in "$@"; do
    case "$arg" in
        --referee) WAIT_REF=true ;;
    esac
done

set +u
source "$WS/install/setup.bash"
set -u

echo "=== Stationary Function Test ==="
echo "wait_for_referee: $WAIT_REF"

ros2 launch "$WS/scripts/test_simple_nav_launch1.py" \
    log_level:=info wait_for_referee:="$WAIT_REF" || true

echo "=== Test finished, running make restart ==="
cd "$WS"
make restart
