#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/slam_ws"
RUN_ID="${RUN_ID:-corridor02_gate_10s}"
RUN_DIR="${WORKSPACE}/paper_exp/submap_gate_preflight_20260804/${RUN_ID}"
BAG="${WORKSPACE}/bags/Corridor02/Corridor02.bag"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"
CONFIG="${CONFIG:-cartographer_m3dgr_mid360_submap_gate.lua}"
ROS_PORT="${ROS_PORT:-11330}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"

mkdir -p "${RUN_DIR}"
test -r "${BAG}"
cartographer_print_configuration \
  -configuration_directories "${CONFIG_DIR}" \
  -configuration_basename "${CONFIG}" \
  > "${RUN_DIR}/resolved_configuration.lua"
cp "${CONFIG_DIR}/${CONFIG}" "${RUN_DIR}/config_snapshot.lua"
{
  printf 'experiment=submap_insertion_gate_preflight\n'
  printf 'dataset=M3DGR_Corridor02\nconfig=%s\nbag=%s\n' "${CONFIG}" "${BAG}"
  printf 'bag_sha256=%s\n' "$(sha256sum "${BAG}" | awk '{print $1}')"
  printf 'play_duration_sec=10\nplayback_rate=1.0\n'
  printf 'input_topics=/livox/mid360/lidar,/odom\n'
  printf 'imu_used=false\navia_used=false\n'
  printf 'gate_min_information=10.0\n'
  printf 'gate_bad_scan_count=2\ngate_good_scan_count=2\n'
  printf 'gate_max_consecutive_blocked_scans=20\n'
  printf 'git_head=%s\n' "$(git -C "${WORKSPACE}" rev-parse HEAD)"
} > "${RUN_DIR}/metadata.txt"

stop_pid() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  fi
}
wait_for_ros() {
  for _ in $(seq 1 160); do
    rosparam list >/dev/null 2>&1 && return 0
    sleep 0.25
  done
  return 1
}
wait_for_service() {
  local service="$1"
  for _ in $(seq 1 200); do
    rosservice list 2>/dev/null | rg -q "^${service}$" && return 0
    sleep 0.25
  done
  return 1
}

export ROS_HOME="${RUN_DIR}/ros_home"
mkdir -p "${ROS_HOME}"
roscore -p "${ROS_PORT}" > "${RUN_DIR}/roscore.log" 2>&1 &
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
  > "${RUN_DIR}/roslaunch.log" 2>&1 &
launch_pid=$!
wait_for_service /finish_trajectory

capture() {
  local topic="$1" output="$2"
  rostopic echo -p "${topic}" > "${RUN_DIR}/${output}.csv" \
    2> "${RUN_DIR}/${output}.err" &
  capture_pids+=("$!")
}
capture /scan scan
capture /odom odom
capture /tracked_pose tracked_pose
capture /map_metadata map_metadata
capture /lidar_observability_min_information lidar_observability_min_information
capture /lidar_observability_condition_number lidar_observability_condition_number
capture /submap_insertion_gate_blocked submap_insertion_gate_blocked
capture /submap_insertion_gate_forced_insertion submap_insertion_gate_forced_insertion
capture /submap_insertion_gate_bad_scan_count submap_insertion_gate_bad_scan_count
capture /submap_insertion_gate_good_scan_count submap_insertion_gate_good_scan_count

rosbag play --clock --quiet -u 10 "${BAG}" \
  --topics /livox/mid360/lidar /odom \
  > "${RUN_DIR}/rosbag_play.log" 2>&1
sleep 4
rostopic echo -n 1 /map > "${RUN_DIR}/map_one.txt" 2> "${RUN_DIR}/map_one.err" || true
rosservice call /finish_trajectory "trajectory_id: 0" \
  > "${RUN_DIR}/finish_trajectory.log" 2>&1 || true
sleep 2
for pid in "${capture_pids[@]:-}"; do stop_pid "${pid}"; done
capture_pids=()
rosnode list > "${RUN_DIR}/nodes.txt" 2>&1 || true
rostopic list > "${RUN_DIR}/topics.txt" 2>&1 || true
rosnode info /cartographer_node > "${RUN_DIR}/cartographer_node_info.txt" 2>&1 || true
rg -n "FATAL|Check failed|Segmentation fault|ERROR" "${RUN_DIR}/roslaunch.log" \
  > "${RUN_DIR}/fatal_scan.txt" || true
{
  csv_samples() { awk 'NR > 1 { n++ } END { print n + 0 }' "$1"; }
  printf 'scan_samples=%s\n' "$(csv_samples "${RUN_DIR}/scan.csv")"
  printf 'odom_samples=%s\n' "$(csv_samples "${RUN_DIR}/odom.csv")"
  printf 'tracked_pose_samples=%s\n' "$(csv_samples "${RUN_DIR}/tracked_pose.csv")"
  printf 'gate_blocked_samples=%s\n' "$(csv_samples "${RUN_DIR}/submap_insertion_gate_blocked.csv")"
  printf 'gate_forced_samples=%s\n' "$(csv_samples "${RUN_DIR}/submap_insertion_gate_forced_insertion.csv")"
  printf 'map_metadata_samples=%s\n' "$(csv_samples "${RUN_DIR}/map_metadata.csv")"
  printf 'map_width=%s\n' "$(rg -m1 '^  width:' "${RUN_DIR}/map_one.txt" | awk '{print $2}')"
  printf 'map_height=%s\n' "$(rg -m1 '^  height:' "${RUN_DIR}/map_one.txt" | awk '{print $2}')"
} > "${RUN_DIR}/summary.txt"
test -s "${RUN_DIR}/scan.csv"
test -s "${RUN_DIR}/odom.csv"
test -s "${RUN_DIR}/tracked_pose.csv"
test ! -s "${RUN_DIR}/fatal_scan.txt"
touch "${RUN_DIR}/SUCCESS"
printf 'preflight_run=%s\n' "${RUN_DIR}"
