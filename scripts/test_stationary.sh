#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash test_stationary.sh --no-referee
  bash test_stationary.sh --with-referee
EOF
}

MODE=""
case "${1:-}" in
  --no-referee)
    MODE="no-referee"
    ;;
  --with-referee)
    MODE="with-referee"
    ;;
  *)
    usage
    exit 1
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
START_TS="$(date '+%Y-%m-%d %H:%M:%S')"
RUN_TS="$(date '+%Y%m%d_%H%M%S')"
LOG_DIR="${WORKSPACE_DIR}/log/stationary_test/${RUN_TS}_${MODE}"
ROSBAG_DIR="${LOG_DIR}/rosbag"
BT_LOG="${LOG_DIR}/bt.log"
HZ_LOG="${LOG_DIR}/topic_hz.log"

BT_SRC_XML="${WORKSPACE_DIR}/src/pb2025_sentry_behavior/behavior_trees/stationary_test.xml"
BT_INSTALL_XML="${WORKSPACE_DIR}/install/pb2025_sentry_behavior/share/pb2025_sentry_behavior/behavior_trees/stationary_test.xml"
PARAMS_SRC="${WORKSPACE_DIR}/src/pb2025_sentry_behavior/params/sentry_behavior_stationary_test.yaml"
PARAMS_INSTALL="${WORKSPACE_DIR}/install/pb2025_sentry_behavior/share/pb2025_sentry_behavior/params/sentry_behavior_stationary_test.yaml"

SERVICES_TO_STOP=(
  "ros2-pb2025-sentry-behavior-field-training.service"
  "ros2-pb2025-sentry-behavior.service"
  "ros2-pb2025-sentry-nav-field-training.service"
)

CORE_SERVICES=(
  "ros2-soem-bringup.service"
  "ros2-universal-controller.service"
)

REF_SERVICE="ros2-dji-referee-protocol.service"

ROSBAG_PID=""
BT_PID=""
MONITOR_PID=""
MANUAL_PUB_PID=""

mkdir -p "${LOG_DIR}" "${ROSBAG_DIR}"

log() {
  printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"
}

log "缓存 sudo 凭据（仅需输入一次密码）..."
sudo -v

run_systemctl() {
  sudo systemctl "$@"
}

collect_journal() {
  local service="$1"
  local output="$2"
  if ! journalctl -u "${service}" --since "${START_TS}" --no-pager >"${output}" 2>&1; then
    true
  fi
}

cleanup() {
  set +e
  log "开始清理后台进程并收集日志..."

  if [[ -n "${MONITOR_PID}" ]] && kill -0 "${MONITOR_PID}" 2>/dev/null; then
    kill "${MONITOR_PID}" 2>/dev/null
    wait "${MONITOR_PID}" 2>/dev/null
  fi

  if [[ -n "${ROSBAG_PID}" ]] && kill -0 "${ROSBAG_PID}" 2>/dev/null; then
    kill -INT "${ROSBAG_PID}" 2>/dev/null
    wait "${ROSBAG_PID}" 2>/dev/null
  fi

  if [[ -n "${MANUAL_PUB_PID}" ]] && kill -0 "${MANUAL_PUB_PID}" 2>/dev/null; then
    kill "${MANUAL_PUB_PID}" 2>/dev/null
    wait "${MANUAL_PUB_PID}" 2>/dev/null
  fi

  if [[ -n "${BT_PID}" ]] && kill -0 "${BT_PID}" 2>/dev/null; then
    kill -INT "${BT_PID}" 2>/dev/null
    wait "${BT_PID}" 2>/dev/null
  fi

  collect_journal "ros2-soem-bringup.service" "${LOG_DIR}/soem.log"
  collect_journal "ros2-universal-controller.service" "${LOG_DIR}/controller.log"
  if [[ "${MODE}" == "with-referee" ]]; then
    collect_journal "${REF_SERVICE}" "${LOG_DIR}/referee.log"
  fi

  log "测试结束，日志目录：${LOG_DIR}"
}

trap cleanup EXIT INT TERM

log "Phase 0: 构建并部署测试文件"
cd "${WORKSPACE_DIR}"
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
colcon build --packages-select universal_controller
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

install -m 0644 "${BT_SRC_XML}" "${BT_INSTALL_XML}"
install -m 0644 "${PARAMS_SRC}" "${PARAMS_INSTALL}"

log "Phase 1: 服务管理"
for svc in "${SERVICES_TO_STOP[@]}"; do
  run_systemctl stop "${svc}" || true
done

run_systemctl restart "ros2-soem-bringup.service"
sleep 3
run_systemctl restart "ros2-universal-controller.service"
sleep 3

if [[ "${MODE}" == "with-referee" ]]; then
  run_systemctl restart "${REF_SERVICE}"
else
  run_systemctl stop "${REF_SERVICE}" || true
fi

log "Phase 2: 环境快照"
{
  echo "==== $(date '+%F %T') systemctl status ===="
  systemctl status ros2-soem-bringup.service --no-pager || true
  systemctl status ros2-universal-controller.service --no-pager || true
  systemctl status "${REF_SERVICE}" --no-pager || true
} >"${LOG_DIR}/service_status.txt" 2>&1

ros2 topic list >"${LOG_DIR}/topic_list.txt" 2>&1 || true
ros2 node list >"${LOG_DIR}/node_list.txt" 2>&1 || true

log "Phase 3: 启动 BT 与 rosbag"
ros2 bag record \
  -o "${ROSBAG_DIR}/stationary_test" \
  /gimbal_scan_cmd \
  /auto_aim_switch \
  /cmd_vel \
  /cmd_spin \
  /referee/common/game_status \
  /referee/parsed/common/constraints \
  /manual_start \
  /rosout \
  >"${LOG_DIR}/rosbag.log" 2>&1 &
ROSBAG_PID="$!"

ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_field_training_launch.py \
  params_file:="${PARAMS_INSTALL}" \
  log_level:=debug \
  > >(tee -a "${BT_LOG}") 2>&1 &
BT_PID="$!"

if [[ "${MODE}" == "no-referee" ]]; then
  sleep 5
  ros2 topic pub -r 2 --times 10 /manual_start std_msgs/msg/Int32 "{data: 1}" >>"${LOG_DIR}/manual_start_pub.log" 2>&1 &
  MANUAL_PUB_PID=$!
fi

log "Phase 4: 周期监控 (每 10 秒采样话题频率)"
(
  while true; do
    {
      echo "==== $(date '+%F %T') ===="
      for topic in /gimbal_scan_cmd /auto_aim_switch /cmd_vel /cmd_spin /referee/common/game_status; do
        echo "[${topic}]"
        timeout 2s ros2 topic hz "${topic}" 2>&1 | awk 'END{print}'
      done
      echo
    } >>"${HZ_LOG}"
    sleep 10
  done
) &
MONITOR_PID="$!"

log "Phase 5: 测试运行中。按 Ctrl+C 结束并自动收集日志。"
wait "${BT_PID}"
