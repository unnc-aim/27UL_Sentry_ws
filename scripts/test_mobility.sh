#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash test_mobility.sh --no-referee          # manual_start 触发
  bash test_mobility.sh --with-referee        # 裁判系统开始比赛后触发

全面移动测试：Nav2 折返导航 + 云台扫描 + 小陀螺 + 自瞄 + 地图诊断
EOF
}

MODE=""
case "${1:-}" in
  --no-referee) MODE="no-referee" ;;
  --with-referee) MODE="with-referee" ;;
  *) usage; exit 1 ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
START_TS="$(date '+%Y-%m-%d %H:%M:%S')"
RUN_TS="$(date '+%Y%m%d_%H%M%S')"
LOG_DIR="${WORKSPACE_DIR}/log/mobility_test/${RUN_TS}_${MODE}"
ROSBAG_DIR="${LOG_DIR}/rosbag"
BT_LOG="${LOG_DIR}/bt.log"
HZ_LOG="${LOG_DIR}/topic_hz.log"
DIAG_LOG="${LOG_DIR}/diagnostics.log"
MAP_DIAG_DIR="${LOG_DIR}/map_snapshots"

BT_SRC_XML="${WORKSPACE_DIR}/src/pb2025_sentry_behavior/behavior_trees/mobility_test.xml"
BT_INSTALL_XML="${WORKSPACE_DIR}/install/pb2025_sentry_behavior/share/pb2025_sentry_behavior/behavior_trees/mobility_test.xml"
PARAMS_SRC="${WORKSPACE_DIR}/src/pb2025_sentry_behavior/params/sentry_behavior_mobility_test.yaml"
PARAMS_INSTALL="${WORKSPACE_DIR}/install/pb2025_sentry_behavior/share/pb2025_sentry_behavior/params/sentry_behavior_mobility_test.yaml"

SERVICES_TO_STOP=(
  "ros2-pb2025-sentry-behavior-field-training.service"
  "ros2-pb2025-sentry-behavior.service"
)

CORE_SERVICES=(
  "ros2-soem-bringup.service"
  "ros2-universal-controller.service"
)

NAV_SERVICE="ros2-pb2025-sentry-nav-field-training.service"
REF_SERVICE="ros2-dji-referee-protocol.service"

ROSBAG_PID=""
BT_PID=""
MONITOR_PID=""
DIAG_PID=""
MANUAL_PUB_PID=""

mkdir -p "${LOG_DIR}" "${ROSBAG_DIR}" "${MAP_DIAG_DIR}"

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
  journalctl -u "${service}" --since "${START_TS}" --no-pager >"${output}" 2>&1 || true
}

cleanup() {
  set +e
  log "开始清理后台进程并收集日志..."

  for pid_var in MONITOR_PID DIAG_PID MANUAL_PUB_PID; do
    local pid="${!pid_var}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null
      wait "${pid}" 2>/dev/null
    fi
  done

  if [[ -n "${ROSBAG_PID}" ]] && kill -0 "${ROSBAG_PID}" 2>/dev/null; then
    kill -INT "${ROSBAG_PID}" 2>/dev/null
    wait "${ROSBAG_PID}" 2>/dev/null
  fi

  if [[ -n "${BT_PID}" ]] && kill -0 "${BT_PID}" 2>/dev/null; then
    kill -INT "${BT_PID}" 2>/dev/null
    wait "${BT_PID}" 2>/dev/null
  fi

  log "收集服务日志..."
  collect_journal "ros2-soem-bringup.service" "${LOG_DIR}/soem.log"
  collect_journal "ros2-universal-controller.service" "${LOG_DIR}/controller.log"
  collect_journal "${NAV_SERVICE}" "${LOG_DIR}/nav.log"
  if [[ "${MODE}" == "with-referee" ]]; then
    collect_journal "${REF_SERVICE}" "${LOG_DIR}/referee.log"
  fi

  log "最终环境快照..."
  {
    echo "==== $(date '+%F %T') final snapshot ===="
    echo "--- ros2 node list ---"
    ros2 node list 2>&1 || true
    echo "--- ros2 topic list ---"
    ros2 topic list 2>&1 || true
    echo "--- ros2 action list ---"
    ros2 action list 2>&1 || true
    echo "--- TF frames ---"
    ros2 run tf2_ros tf2_monitor --once 2>&1 | head -80 || true
  } >>"${LOG_DIR}/final_snapshot.txt" 2>&1

  log "测试结束，日志目录：${LOG_DIR}"
}

trap cleanup EXIT INT TERM

# ==================================================
# Phase 0: 构建并部署
# ==================================================
log "Phase 0: 构建并部署测试文件"
cd "${WORKSPACE_DIR}"
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
colcon build --packages-select universal_controller pb2025_sentry_behavior
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

install -m 0644 "${BT_SRC_XML}" "${BT_INSTALL_XML}"
install -m 0644 "${PARAMS_SRC}" "${PARAMS_INSTALL}"

# ==================================================
# Phase 1: 服务管理
# ==================================================
log "Phase 1: 服务管理"

for svc in "${SERVICES_TO_STOP[@]}"; do
  run_systemctl stop "${svc}" || true
done

run_systemctl restart "ros2-soem-bringup.service"
sleep 3
run_systemctl restart "ros2-universal-controller.service"
sleep 3

log "启动 Nav2 + SLAM..."
run_systemctl restart "${NAV_SERVICE}"
sleep 8

if [[ "${MODE}" == "with-referee" ]]; then
  run_systemctl restart "${REF_SERVICE}"
  sleep 2
else
  run_systemctl stop "${REF_SERVICE}" || true
fi

# ==================================================
# Phase 2: 环境快照
# ==================================================
log "Phase 2: 环境快照"
{
  echo "==== $(date '+%F %T') systemctl status ===="
  for svc in ros2-soem-bringup.service ros2-universal-controller.service "${NAV_SERVICE}" "${REF_SERVICE}"; do
    systemctl status "${svc}" --no-pager 2>&1 || true
    echo
  done
} >"${LOG_DIR}/service_status.txt" 2>&1

{
  echo "--- ros2 node list ---"
  ros2 node list 2>&1 || true
  echo "--- ros2 topic list ---"
  ros2 topic list 2>&1 || true
  echo "--- ros2 action list ---"
  ros2 action list 2>&1 || true
} >"${LOG_DIR}/initial_snapshot.txt" 2>&1

# ==================================================
# Phase 3: 启动 rosbag（全面录制）
# ==================================================
log "Phase 3: 启动 rosbag 录制"
ros2 bag record \
  -o "${ROSBAG_DIR}/mobility_test" \
  /gimbal_scan_cmd \
  /auto_aim_switch \
  /cmd_vel \
  /cmd_vel_nav2_result \
  /cmd_spin \
  /odometry \
  /tf \
  /tf_static \
  /obstacle_scan \
  /map \
  /map_updates \
  /global_costmap/costmap \
  /local_costmap/costmap \
  /referee/common/game_status \
  /referee/parsed/common/constraints \
  /manual_start \
  /rosout \
  >"${LOG_DIR}/rosbag.log" 2>&1 &
ROSBAG_PID="$!"

# ==================================================
# Phase 4: 启动 BT
# ==================================================
log "Phase 4: 启动行为树"
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_field_training_launch.py \
  params_file:="${PARAMS_INSTALL}" \
  log_level:=debug \
  > >(tee -a "${BT_LOG}") 2>&1 &
BT_PID="$!"

if [[ "${MODE}" == "no-referee" ]]; then
  sleep 5
  log "发布 manual_start..."
  ros2 topic pub -r 2 --times 10 /manual_start std_msgs/msg/Int32 "{data: 1}" \
    >>"${LOG_DIR}/manual_start_pub.log" 2>&1 &
  MANUAL_PUB_PID=$!
fi

# ==================================================
# Phase 5: 周期性话题频率监控
# ==================================================
log "Phase 5: 启动话题频率监控"
(
  TOPICS_TO_MONITOR=(
    /gimbal_scan_cmd
    /auto_aim_switch
    /cmd_vel
    /cmd_vel_nav2_result
    /cmd_spin
    /odometry
    /map
    /global_costmap/costmap
    /local_costmap/costmap
    /obstacle_scan
  )
  if [[ "${MODE}" == "with-referee" ]]; then
    TOPICS_TO_MONITOR+=(/referee/common/game_status)
  fi

  while true; do
    {
      echo "==== $(date '+%F %T') ===="
      for topic in "${TOPICS_TO_MONITOR[@]}"; do
        echo "[${topic}]"
        timeout 2s ros2 topic hz "${topic}" 2>&1 | awk 'END{print}'
      done
      echo
    } >>"${HZ_LOG}"
    sleep 15
  done
) &
MONITOR_PID="$!"

# ==================================================
# Phase 6: 周期性地图与导航诊断
# ==================================================
log "Phase 6: 启动地图/导航诊断"
(
  seq_num=0
  while true; do
    ts="$(date '+%H%M%S')"
    {
      echo "======== diag #${seq_num} @ $(date '+%F %T') ========"

      echo "--- TF map->odom ---"
      timeout 2s ros2 run tf2_ros tf2_echo map odom 2>&1 | head -5 || echo "(unavailable)"

      echo "--- TF odom->base_footprint ---"
      timeout 2s ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -5 || echo "(unavailable)"

      echo "--- Nav2 action status ---"
      timeout 2s ros2 action info /navigate_to_pose 2>&1 || echo "(action not available)"

      echo "--- global_costmap info ---"
      timeout 2s ros2 topic info /global_costmap/costmap --verbose 2>&1 | head -20 || echo "(unavailable)"

      echo "--- local_costmap info ---"
      timeout 2s ros2 topic info /local_costmap/costmap --verbose 2>&1 | head -20 || echo "(unavailable)"

      echo "--- SLAM /map info ---"
      timeout 2s ros2 topic info /map --verbose 2>&1 | head -20 || echo "(unavailable)"

      echo "--- /map_updates info ---"
      timeout 2s ros2 topic info /map_updates --verbose 2>&1 | head -10 || echo "(unavailable)"

      echo "--- obstacle_scan info ---"
      timeout 2s ros2 topic info /obstacle_scan --verbose 2>&1 | head -10 || echo "(unavailable)"

      echo "--- rosout errors (last 30s) ---"
      timeout 3s ros2 topic echo /rosout --once --no-arr 2>&1 | head -20 || true

      echo
    } >>"${DIAG_LOG}"

    timeout 3s ros2 topic echo /global_costmap/costmap --once \
      >"${MAP_DIAG_DIR}/global_costmap_${ts}.yaml" 2>&1 || true

    timeout 3s ros2 topic echo /map_metadata --once \
      >"${MAP_DIAG_DIR}/map_metadata_${ts}.yaml" 2>&1 || true

    seq_num=$((seq_num + 1))
    sleep 20
  done
) &
DIAG_PID="$!"

# ==================================================
# Phase 7: 等待测试运行
# ==================================================
log "Phase 7: 测试运行中。按 Ctrl+C 结束并自动收集日志。"
log "  模式: ${MODE}"
log "  日志: ${LOG_DIR}"
log "  行为树: mobility_test (Nav2 折返 + 云台扫描 + 小陀螺)"
wait "${BT_PID}"
