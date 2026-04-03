#!/usr/bin/env bash

set -euo pipefail

workspace_root="/home/soyo/sentry_ws"
default_world_name="reserve/field_training_latest"
session_stamp="$(date +%Y%m%d_%H%M%S)"

world_name="${1:-$default_world_name}"
namespace="${2:-}"
initial_pose_x="${INITIAL_POSE_X:-0.7}"
initial_pose_y="${INITIAL_POSE_Y:-0.5}"
initial_pose_yaw="${INITIAL_POSE_YAW:-0.0}"

setup_script="${workspace_root}/install/setup.bash"
map_base_path="${workspace_root}/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/${world_name}"
pcd_output_path="${workspace_root}/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/${world_name}.pcd"
point_lio_pcd_path="${workspace_root}/src/pb2025_sentry_nav/point_lio/PCD/scans.pcd"
nav_runtime_log_dir="${workspace_root}/log/nav_runtime/${session_stamp}"

declare -a namespace_args=()
if [[ -n "${namespace}" ]]; then
  namespace_args=(--ros-args -r "__ns:=/${namespace}")
fi

declare -a launch_args=()
if [[ -n "${namespace}" ]]; then
  launch_args+=("namespace:=${namespace}")
fi
launch_args+=(
  "initial_pose_x:=${initial_pose_x}"
  "initial_pose_y:=${initial_pose_y}"
  "initial_pose_yaw:=${initial_pose_yaw}"
  "log_dir:=${nav_runtime_log_dir}"
)

set +u
source "${setup_script}"
set -u

mkdir -p "$(dirname "${map_base_path}")"
mkdir -p "$(dirname "${pcd_output_path}")"
mkdir -p "${nav_runtime_log_dir}"

nav_pid=0

save_grid_map() {
  ros2 run nav2_map_server map_saver_cli -f "${map_base_path}" "${namespace_args[@]}"
}

copy_point_lio_pcd() {
  if [[ -f "${point_lio_pcd_path}" ]]; then
    cp -f "${point_lio_pcd_path}" "${pcd_output_path}"
  fi
}

copy_named_nav_logs() {
  local latest_container_log=""

  latest_container_log="$(ls -1t "${nav_runtime_log_dir}"/component_container_isolated*.log 2>/dev/null | head -n 1 || true)"
  if [[ -n "${latest_container_log}" ]]; then
    cp -f "${latest_container_log}" "${nav_runtime_log_dir}/nav2_container.log"
  fi
}

stop_navigation() {
  if [[ "${nav_pid}" -ne 0 ]] && kill -0 "${nav_pid}" 2>/dev/null; then
    kill -INT "${nav_pid}"
    wait "${nav_pid}"
  fi
}

handle_shutdown() {
  set +e
  save_grid_map
  stop_navigation
  copy_point_lio_pcd
  copy_named_nav_logs
  echo "Navigation runtime logs saved to: ${nav_runtime_log_dir}"
  exit 0
}

trap handle_shutdown INT TERM

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  slam:=True \
  use_robot_state_pub:=True \
  "${launch_args[@]}" &
nav_pid=$!

wait "${nav_pid}"
exit_code=$?

set +e
copy_point_lio_pcd
copy_named_nav_logs
echo "Navigation runtime logs saved to: ${nav_runtime_log_dir}"
exit "${exit_code}"