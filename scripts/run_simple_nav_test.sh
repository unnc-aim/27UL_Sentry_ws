#!/usr/bin/env bash
# Usage: bash ~/sentry_ws/scripts/run_simple_nav_test.sh

WS=/home/soyo/sentry_ws

set +u
source "$WS/install/setup.bash"
set -u

ros2 launch "$WS/scripts/test_simple_nav_launch.py" log_level:=info || true

echo "=== Test finished, running make restart ==="
cd "$WS"
make restart
