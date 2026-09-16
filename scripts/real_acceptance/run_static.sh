#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source install/setup.bash
if pgrep -f '[/]livox_ros_driver2_node|[/]pointlio_mapping' >/dev/null; then
    echo '已有雷达/定位进程，拒绝重复启动。请先确认现有进程用途。' >&2
    exit 2
fi
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/sentry-static-acceptance/log
mkdir -p "$ROS_LOG_DIR"
printf '%s\n' '{"status":"starting","physical_acceptance_passed":false,"motion_authorized":false}' > /tmp/sentry-static-acceptance/report.json
# Refuse an occupied test domain. This check never alters the live domain.
python3 - <<'PY'
import time
import rclpy
rclpy.init()
n = rclpy.create_node('static_domain_preflight', enable_rosout=False, start_parameter_services=False)
end = time.monotonic() + 3
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=.1)
others = [name for name in n.get_node_names() if name != n.get_name()]
n.destroy_node()
rclpy.shutdown()
if others:
    raise SystemExit('Domain 88 already occupied: ' + ', '.join(others))
PY
python3 - <<'PYRUN'
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

def interrupted(signum, frame):
    raise KeyboardInterrupt
signal.signal(signal.SIGTERM, interrupted)

# A new process group makes cleanup apply only to this diagnostic launch.
with open('/tmp/sentry-static-acceptance/launch.log', 'w') as log:
    child = subprocess.Popen(['ros2', 'launch', './scripts/real_acceptance/static_launch.py'],
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    code = 2
    try:
        time.sleep(12)
        if child.poll() is not None:
            raise RuntimeError('Static launch failed; inspect /tmp/sentry-static-acceptance/launch.log')
        code = subprocess.run(['python3', 'scripts/real_acceptance/check.py', '--isolated',
                               '--seconds', '12', '--output',
                               '/tmp/sentry-static-acceptance/report.json'], timeout=35).returncode
        subprocess.run(['python3', 'scripts/real_acceptance/scene_probe.py'], timeout=20, check=True)
        subprocess.run(['python3', 'scripts/real_acceptance/plan_probe.py'], timeout=25, check=True)
    except KeyboardInterrupt:
        code = 130
    finally:
        for sig, seconds in [(signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)]:
            try:
                os.killpg(child.pid, sig)
            except ProcessLookupError:
                break
            try:
                child.wait(timeout=seconds)
            except subprocess.TimeoutExpired:
                continue
            # Ensure even a child orphaned by an exited launch is stopped.
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                break
        child.wait(timeout=5)
    sys.exit(code)
PYRUN
