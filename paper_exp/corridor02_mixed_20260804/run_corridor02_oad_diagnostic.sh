#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/slam_ws"
ROOT="${WORKSPACE}/paper_exp/corridor02_mixed_20260804"
BAG="${WORKSPACE}/bags/Corridor02/Corridor02.bag"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"
MODE="${MODE:-diagnostic}"
if [[ -n "${CONFIG:-}" ]]; then
  CONFIG="${CONFIG}"
elif [[ "${MODE}" == "control" ]]; then
  CONFIG="cartographer_m3dgr_mid360_oad.lua"
elif [[ "${MODE}" == "baseline" ]]; then
  CONFIG="cartographer_m3dgr_mid360_clean_baseline.lua"
else
  CONFIG="cartographer_m3dgr_mid360_oad_diagnostic_full_yaw.lua"
fi
PLAY_DURATION="${PLAY_DURATION:-}"
RUN_ID="${RUN_ID:-corridor02_oad_full_yaw}"
RUN_DIR="${RUN_DIR_OVERRIDE:-${ROOT}/runs/${RUN_ID}}"
ROS_PORT="${ROS_PORT:-11320}"
FORCE="${FORCE:-0}"
PLAYBACK_RATE="${PLAYBACK_RATE:-1.0}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"

if [[ -f "${RUN_DIR}/SUCCESS" && "${FORCE}" != "1" ]]; then
  echo "Reusing completed diagnostic: ${RUN_DIR}"
  exit 0
fi
if [[ -d "${RUN_DIR}" && "${FORCE}" != "1" ]]; then
  echo "Incomplete run exists; inspect it or set FORCE=1: ${RUN_DIR}" >&2
  exit 1
fi
mkdir -p "${RUN_DIR}"

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

test -r "${BAG}"
cartographer_print_configuration \
  -configuration_directories "${CONFIG_DIR}" \
  -configuration_basename "${CONFIG}" \
  > "${RUN_DIR}/resolved_configuration.lua"
cp "${CONFIG_DIR}/${CONFIG}" "${RUN_DIR}/config_snapshot.lua"
{
  if [[ "${MODE}" == "control" ]]; then
    printf 'experiment=corridor02_oad_control\n'
    printf 'trajectory_control=true\ndiagnostic_only=false\n'
  elif [[ "${MODE}" == "baseline" ]]; then
    printf 'experiment=corridor02_clean_baseline\n'
    printf 'trajectory_control=false\ndiagnostic_only=false\n'
  else
    printf 'experiment=corridor02_oad_read_only_diagnostic\n'
    printf 'trajectory_control=false\ndiagnostic_only=true\n'
  fi
  printf 'dataset=M3DGR_Corridor02\nconfig=%s\nbag=%s\n' "${CONFIG}" "${BAG}"
  printf 'bag_sha256=%s\n' "$(sha256sum "${BAG}" | awk '{print $1}')"
  printf 'git_head=%s\n' "$(git -C "${WORKSPACE}" rev-parse HEAD)"
  printf 'mode=%s\nplayback_rate=%s\n' "${MODE}" "${PLAYBACK_RATE}"
  printf 'imu_used=false\navia_used=false\nvisual_used=false\n'
  printf 'input_topics=/livox/mid360/lidar,/odom\n'
  printf 'pre_registered_offset_threshold_m=0.15\n'
  printf 'pre_registered_map_score_gain=0.02\n'
  printf 'pre_registered_peak_margin=0.01\n'
  printf 'pre_registered_min_consecutive_samples=2\n'
  printf 'pre_registered_min_events=3\n'
  printf 'start_wall_utc=%s\n' "$(date -u +%FT%TZ)"
} > "${RUN_DIR}/metadata.txt"

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
capture /tracked_pose tracked_pose
capture /odom odom
capture /degeneracy_metric degeneracy_metric
capture /lidar_observability_condition_number lidar_observability_condition_number
capture /lidar_observability_min_information lidar_observability_min_information
capture /lidar_observability_yaw_information lidar_observability_yaw_information
capture /correlative_candidate_pose correlative_candidate_pose
capture /correlative_candidate_offset correlative_candidate_offset
capture /correlative_fixed_map_score correlative_fixed_map_score
capture /correlative_expanded_map_score correlative_expanded_map_score
capture /correlative_second_peak_score correlative_second_peak_score
capture /correlative_peak_margin correlative_peak_margin
capture /correlative_second_peak_valid correlative_second_peak_valid
capture /correlative_candidate_on_boundary correlative_candidate_on_boundary
capture /correlative_oad_triggered correlative_oad_triggered
capture /correlative_oad_accepted correlative_oad_accepted
capture /correlative_oad_timed_out correlative_oad_timed_out
capture /correlative_oad_peak_distinct correlative_oad_peak_distinct
capture /correlative_oad_confirmation_count correlative_oad_confirmation_count
capture /correlative_oad_search_levels correlative_oad_search_levels
capture /correlative_candidate_search_time_ms correlative_candidate_search_time_ms
capture /correlative_candidate_count correlative_candidate_count
capture /correlative_candidate_submap_num_range_data correlative_candidate_submap_num_range_data

play_args=(--clock --quiet --rate="${PLAYBACK_RATE}" "${BAG}" --topics /livox/mid360/lidar /odom)
if [[ -n "${PLAY_DURATION}" ]]; then
  play_args=(-u "${PLAY_DURATION}" "${play_args[@]}")
fi
play_start="$(date +%s.%N)"
rosbag play "${play_args[@]}" > "${RUN_DIR}/rosbag_play.log" 2>&1
play_end="$(date +%s.%N)"
sleep 5
rosservice call /finish_trajectory "trajectory_id: 0" \
  > "${RUN_DIR}/finish_trajectory.log" 2>&1 || true
sleep 2
for pid in "${capture_pids[@]:-}"; do stop_pid "${pid}"; done
capture_pids=()
rosnode list > "${RUN_DIR}/nodes.txt" 2>&1 || true
rostopic list > "${RUN_DIR}/topics.txt" 2>&1 || true
rosnode info /cartographer_node > "${RUN_DIR}/cartographer_node_info.txt" 2>&1 || true
rg -n "FATAL|Check failed|Segmentation fault" "${RUN_DIR}/roslaunch.log" \
  > "${RUN_DIR}/fatal_scan.txt" || true
awk -v start="${play_start}" -v end="${play_end}" \
  'BEGIN {printf "play_wall_duration_sec=%.6f\n", end-start}' \
  >> "${RUN_DIR}/metadata.txt"
printf 'end_wall_utc=%s\n' "$(date -u +%FT%TZ)" >> "${RUN_DIR}/metadata.txt"
test -s "${RUN_DIR}/tracked_pose.csv"
if [[ "${MODE}" == "diagnostic" || "${MODE}" == "control" ]]; then
  test -s "${RUN_DIR}/correlative_candidate_pose.csv"
fi
test ! -s "${RUN_DIR}/fatal_scan.txt"
touch "${RUN_DIR}/SUCCESS"
echo "diagnostic_run=${RUN_DIR}"
