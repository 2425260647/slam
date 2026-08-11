#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/slam_ws"
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${WORKSPACE}/bags/Wheel-float01/Wheel-float01.bag"
RUN_DIR="${RUN_DIR_OVERRIDE:-${SCRIPT_ROOT}/wheel_float01_10s}"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"
CONFIG="${CONFIG:-cartographer_m3dgr_mid360_oad_diagnostic.lua}"
PLAY_DURATION="${PLAY_DURATION:-10}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"
mkdir -p "${RUN_DIR}"

stop_pid() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  fi
}

wait_for_ros() {
  for _ in $(seq 1 120); do
    if rosparam list >/dev/null 2>&1; then return 0; fi
    sleep 0.25
  done
  return 1
}

wait_for_service() {
  local service="$1"
  for _ in $(seq 1 160); do
    if rosservice list 2>/dev/null | rg -q "^${service}$"; then return 0; fi
    sleep 0.25
  done
  return 1
}

test -r "${BAG}"
cartographer_print_configuration \
  -configuration_directories "${CONFIG_DIR}" \
  -configuration_basename "${CONFIG}" \
  >"${RUN_DIR}/resolved_configuration.lua"
{
  printf 'experiment=oad_csm_feasibility_20260804\n'
  printf 'dataset=Wheel-float01\nconfig=%s\nbag=%s\n' "${CONFIG}" "${BAG}"
  printf 'bag_sha256=%s\n' "$(sha256sum "${BAG}" | awk '{print $1}')"
  printf 'play_duration_sec=%s\nplayback_rate=1.0\n' "${PLAY_DURATION}"
  printf 'input_topics=/livox/mid360/lidar,/odom,/vrpn_client_node/UGV/pose\n'
  printf 'imu_used=false\navia_used=false\ntrajectory_control=false\n'
  printf 'diagnostic_only=true\ndiagnostic_linear_window_m=1.0\n'
  printf 'diagnostic_angular_window_source=resolved_configuration.lua\n'
  printf 'diagnostic_sampling_interval=5\n'
  printf 'start_wall_utc=%s\n' "$(date -u +%FT%TZ)"
  printf 'git_head=%s\n' "$(git -C "${WORKSPACE}" rev-parse HEAD)"
} >"${RUN_DIR}/metadata.txt"

export ROS_HOME="${RUN_DIR}/ros_home"
mkdir -p "${ROS_HOME}"
roscore >"${RUN_DIR}/roscore.log" 2>&1 &
roscore_pid=$!
launch_pid=""
capture_pids=()
cleanup() {
  local pid
  for pid in "${capture_pids[@]:-}"; do stop_pid "${pid}"; done
  stop_pid "${launch_pid}"
  stop_pid "${roscore_pid}"
}
trap cleanup EXIT
wait_for_ros
rosparam set /use_sim_time true
roslaunch my_navigation m3dgr_mid360_mapping.launch \
  cartographer_config_dir:="${CONFIG_DIR}" \
  cartographer_config:="${CONFIG}" \
  rviz:=false start_local_grid:=false \
  >"${RUN_DIR}/roslaunch.log" 2>&1 &
launch_pid=$!
wait_for_service /finish_trajectory

capture() {
  local topic="$1" output="$2"
  rostopic echo -p "${topic}" >"${RUN_DIR}/${output}.csv" \
    2>"${RUN_DIR}/${output}.err" &
  capture_pids+=("$!")
}
capture /tracked_pose tracked_pose
capture /vrpn_client_node/UGV/pose mocap_pose
capture /correlative_candidate_pose correlative_candidate_pose
capture /correlative_candidate_offset correlative_candidate_offset
capture /correlative_fixed_map_score correlative_fixed_map_score
capture /correlative_expanded_map_score correlative_expanded_map_score
capture /correlative_second_peak_score correlative_second_peak_score
capture /correlative_peak_margin correlative_peak_margin
capture /correlative_second_peak_valid correlative_second_peak_valid
capture /correlative_candidate_on_boundary correlative_candidate_on_boundary
capture /correlative_candidate_search_time_ms correlative_candidate_search_time_ms
capture /correlative_candidate_count correlative_candidate_count
capture /correlative_candidate_submap_num_range_data correlative_candidate_submap_num_range_data

rosbag play --clock --quiet -u "${PLAY_DURATION}" "${BAG}" \
  --topics /livox/mid360/lidar /odom /vrpn_client_node/UGV/pose \
  >"${RUN_DIR}/rosbag_play.log" 2>&1
sleep 5
rosservice call /finish_trajectory "trajectory_id: 0" \
  >"${RUN_DIR}/finish_trajectory.log" 2>&1 || true
sleep 2
for pid in "${capture_pids[@]:-}"; do stop_pid "${pid}"; done
capture_pids=()
rosnode list >"${RUN_DIR}/nodes.txt" 2>&1 || true
rostopic list >"${RUN_DIR}/topics.txt" 2>&1 || true
rg -n "FATAL|Check failed|Segmentation fault|ERROR" "${RUN_DIR}/roslaunch.log" \
  >"${RUN_DIR}/fatal_scan.txt" || true
printf 'end_wall_utc=%s\n' "$(date -u +%FT%TZ)" >>"${RUN_DIR}/metadata.txt"
test -s "${RUN_DIR}/tracked_pose.csv"
test -s "${RUN_DIR}/mocap_pose.csv"
test -s "${RUN_DIR}/correlative_candidate_pose.csv"
touch "${RUN_DIR}/SUCCESS"
printf 'diagnostic_run=%s\n' "${RUN_DIR}"
