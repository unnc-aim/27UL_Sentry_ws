#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
exec python3 scripts/real_acceptance/map_test.py "$@"
