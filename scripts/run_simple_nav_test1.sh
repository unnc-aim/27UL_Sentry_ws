#!/usr/bin/env bash
# Stationary function test (no movement required).
# Usage: bash ~/sentry_ws/scripts/run_simple_nav_test1.sh

WS=/home/soyo/sentry_ws

set +u
source "$WS/install/setup.bash"
set -u

ros2 launch "$WS/scripts/test_simple_nav_launch1.py" log_level:=info || true

echo "=== Test finished, running make restart ==="
cd "$WS"
make restart
