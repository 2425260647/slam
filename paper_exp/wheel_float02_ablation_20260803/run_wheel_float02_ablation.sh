#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/slam_ws"
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${WORKSPACE}/bags/Wheel-float02/Wheel-float02.bag"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"
RUNS_DIR="${RUNS_DIR_OVERRIDE:-${SCRIPT_ROOT}/runs}"
MAP_SAVER="${WORKSPACE}/paper_exp/innovation1_directional_adaptive_fusion/save_occupancy_grid.py"
EVALUATOR="${SCRIPT_ROOT}/evaluate_m3dgr_trajectory.py"
PLAYBACK_RATE="${PLAYBACK_RATE:-1.0}"
PLAY_DURATION="${PLAY_DURATION:-}"
FORCE="${FORCE:-0}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"

mkdir -p "${RUNS_DIR}"

wait_for_ros() {
  for _ in $(seq 1 120); do
    if rosparam list >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.25
  done
  echo "ROS master did not become ready." >&2
  return 1
}

wait_for_service() {
  local service="$1"
  for _ in $(seq 1 160); do
    if rosservice list 2>/dev/null | rg -q "^${service}$"; then
      return 0
    fi
    sleep 0.25
  done
  echo "Service did not become ready: ${service}" >&2
  return 1
}

stop_pid() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  fi
}

start_capture() {
  local topic="$1"
  local output="$2"
  rostopic echo -p "${topic}" >"${output}" 2>"${output%.csv}.err" &
  CAPTURE_PIDS+=("$!")
}

run_case() (
  local case_id="$1"
  local config="$2"
  local run_dir="${RUNS_DIR}/${case_id}"
  if [[ -f "${run_dir}/SUCCESS" && "${FORCE}" != "1" ]]; then
    echo "[Wheel-float02] Reusing completed ${case_id}"
    return 0
  fi
  if [[ -d "${run_dir}" && "${FORCE}" != "1" ]]; then
    echo "Incomplete run exists; inspect or set FORCE=1: ${run_dir}" >&2
    return 1
  fi
  mkdir -p "${run_dir}"
  export ROS_HOME="${run_dir}/ros_home"
  mkdir -p "${ROS_HOME}"

  local roscore_pid="" launch_pid="" tf_pid=""
  CAPTURE_PIDS=()
  cleanup_case() {
    local capture_pid
    for capture_pid in "${CAPTURE_PIDS[@]:-}"; do
      stop_pid "${capture_pid}"
    done
    stop_pid "${tf_pid}"
    stop_pid "${launch_pid}"
    stop_pid "${roscore_pid}"
  }
  trap cleanup_case EXIT

  {
    printf 'experiment=wheel_float02_ablation_20260803\n'
    printf 'case=%s\nconfig=%s\n' "${case_id}" "${config}"
    printf 'bag=%s\n' "${BAG}"
    printf 'bag_sha256=%s\n' "$(sha256sum "${BAG}" | awk '{print $1}')"
    printf 'playback_rate=%s\n' "${PLAYBACK_RATE}"
    printf 'input_topics=/livox/mid360/lidar,/odom,/vrpn_client_node/UGV/pose\n'
    printf 'imu_used=false\navia_used=false\n'
    printf 'official_static_tf=base_footprint->livox_frame\n'
    printf 'projection_height_m=0.20,0.60\n'
    printf 'start_wall_utc=%s\n' "$(date -u +%FT%TZ)"
    printf 'git_head=%s\n' "$(git -C "${WORKSPACE}" rev-parse HEAD)"
  } >"${run_dir}/metadata.txt"
  cp "${CONFIG_DIR}/${config}" "${run_dir}/config_snapshot.lua"
  cartographer_print_configuration \
    -configuration_directories "${CONFIG_DIR}" \
    -configuration_basename "${config}" \
    >"${run_dir}/resolved_configuration.lua"

  roscore >"${run_dir}/roscore.log" 2>&1 &
  roscore_pid=$!
  wait_for_ros
  rosparam set /use_sim_time true

  roslaunch my_navigation m3dgr_mid360_mapping.launch \
    cartographer_config_dir:="${CONFIG_DIR}" \
    cartographer_config:="${config}" \
    rviz:=false \
    start_local_grid:=false \
    >"${run_dir}/roslaunch.log" 2>&1 &
  launch_pid=$!
  wait_for_service "/finish_trajectory"
  wait_for_service "/write_state"

  start_capture /tracked_pose "${run_dir}/tracked_pose.csv"
  start_capture /vrpn_client_node/UGV/pose "${run_dir}/mocap_pose.csv"
  start_capture /degeneracy_metric "${run_dir}/degeneracy_metric.csv"
  start_capture /lidar_observability_condition_number "${run_dir}/lidar_observability_condition_number.csv"
  start_capture /lidar_observability_min_information "${run_dir}/lidar_observability_min_information.csv"
  start_capture /lidar_observability_yaw_information "${run_dir}/lidar_observability_yaw_information.csv"
  start_capture /lidar_observability_weak_direction "${run_dir}/lidar_observability_weak_direction.csv"
  start_capture /slip_metric "${run_dir}/consistency_metric.csv"
  start_capture /slip_state "${run_dir}/consistency_state.csv"
  start_capture /odom_weight_scale "${run_dir}/odom_weight_scale.csv"
  start_capture /odom_min_weight_scale "${run_dir}/odom_min_weight_scale.csv"
  start_capture /slip_lidar_reliability "${run_dir}/lidar_reliability.csv"
  start_capture /consistency_anomaly_trigger_count "${run_dir}/anomaly_trigger_count.csv"
  start_capture /consistency_anomaly_time "${run_dir}/anomaly_time.csv"
  start_capture /correlative_oad_triggered "${run_dir}/correlative_oad_triggered.csv"
  start_capture /correlative_oad_accepted "${run_dir}/correlative_oad_accepted.csv"
  start_capture /correlative_oad_timed_out "${run_dir}/correlative_oad_timed_out.csv"
  start_capture /correlative_oad_peak_distinct "${run_dir}/correlative_oad_peak_distinct.csv"
  start_capture /correlative_oad_confirmation_count "${run_dir}/correlative_oad_confirmation_count.csv"
  start_capture /correlative_candidate_search_time_ms "${run_dir}/correlative_candidate_search_time_ms.csv"
  start_capture /correlative_candidate_count "${run_dir}/correlative_candidate_count.csv"

  local play_start play_end
  play_start="$(date +%s.%N)"
  play_args=(--clock --quiet --rate="${PLAYBACK_RATE}" "${BAG}"
    --topics /livox/mid360/lidar /odom /vrpn_client_node/UGV/pose)
  if [[ -n "${PLAY_DURATION}" ]]; then
    play_args=(-u "${PLAY_DURATION}" "${play_args[@]}")
  fi
  rosbag play "${play_args[@]}" >"${run_dir}/rosbag_play.log" 2>&1
  play_end="$(date +%s.%N)"
  printf 'play_start_wall_sec=%s\nplay_end_wall_sec=%s\n' \
    "${play_start}" "${play_end}" >>"${run_dir}/metadata.txt"
  awk -v start="${play_start}" -v end="${play_end}" \
    'BEGIN {printf "play_wall_duration_sec=%.6f\n", end-start}' \
    >>"${run_dir}/metadata.txt"

  sleep 6
  rosservice call /finish_trajectory "trajectory_id: 0" \
    >"${run_dir}/finish_trajectory.log" 2>&1 || true
  sleep 10
  rosservice call /write_state \
    "{filename: '${run_dir}/state.pbstream', include_unfinished_submaps: true}" \
    >"${run_dir}/write_state.log" 2>&1 || true
  if [[ -z "${PLAY_DURATION}" ]]; then
    python3 "${MAP_SAVER}" --topic /map \
      --output-prefix "${run_dir}/map" --timeout 45 \
      >"${run_dir}/map_saver.log" 2>&1
  else
    python3 "${MAP_SAVER}" --topic /map \
      --output-prefix "${run_dir}/map" --timeout 15 \
      >"${run_dir}/map_saver.log" 2>&1 || true
  fi

  for capture_pid in "${CAPTURE_PIDS[@]:-}"; do
    stop_pid "${capture_pid}"
  done
  rosnode list >"${run_dir}/nodes.txt" 2>&1 || true
  rostopic list >"${run_dir}/topics.txt" 2>&1 || true
  if rg -n "FATAL|Check failed|Segmentation fault" "${run_dir}/roslaunch.log" \
      >"${run_dir}/fatal_scan.txt"; then
    echo "Fatal marker found in ${run_dir}/roslaunch.log" >&2
    return 1
  fi
  test -s "${run_dir}/tracked_pose.csv"
  test -s "${run_dir}/mocap_pose.csv"
  if [[ -z "${PLAY_DURATION}" ]]; then
    test -s "${run_dir}/map.pgm"
    test -s "${run_dir}/map.yaml"
    python3 "${EVALUATOR}" \
      --estimate "${run_dir}/tracked_pose.csv" \
      --reference "${run_dir}/mocap_pose.csv" \
      --output "${run_dir}/online_metrics.json" \
      >"${run_dir}/online_metrics.stdout.json"
  fi
  printf 'end_wall_utc=%s\n' "$(date -u +%FT%TZ)" >>"${run_dir}/metadata.txt"
  touch "${run_dir}/SUCCESS"

  echo "[Wheel-float02] Finished ${case_id}"
)

preflight() {
  test -r "${BAG}"
  for config in \
    cartographer_m3dgr_mid360_clean_baseline.lua \
    cartographer_m3dgr_mid360_innovation1.lua \
    cartographer_m3dgr_mid360_innovation2.lua \
    cartographer_m3dgr_mid360_full_proposed.lua \
    cartographer_m3dgr_mid360_oad.lua; do
    cartographer_print_configuration \
      -configuration_directories "${CONFIG_DIR}" \
      -configuration_basename "${config}" \
      >"${SCRIPT_ROOT}/.${config}.preflight.txt"
  done
  python3 -m py_compile "${EVALUATOR}"
  echo "[Wheel-float02] Preflight passed."
}

preflight

# A comma-separated CASES value supports focused, reproducible comparisons
# without changing the default four-way ablation experiment.
if [[ -n "${CASES:-}" ]]; then
  IFS=',' read -r -a requested_cases <<< "${CASES}"
  for requested_case in "${requested_cases[@]}"; do
    case "${requested_case}" in
      c0_clean_baseline)
        run_case c0_clean_baseline cartographer_m3dgr_mid360_clean_baseline.lua ;;
      c1_innovation1)
        run_case c1_innovation1 cartographer_m3dgr_mid360_innovation1.lua ;;
      c2_innovation2)
        run_case c2_innovation2 cartographer_m3dgr_mid360_innovation2.lua ;;
      c3_full_proposed)
        run_case c3_full_proposed cartographer_m3dgr_mid360_full_proposed.lua ;;
      oad_v2)
        run_case oad_v2 cartographer_m3dgr_mid360_oad.lua ;;
      *)
        echo "Unknown CASES entry: ${requested_case}" >&2
        exit 2 ;;
    esac
  done
else
  run_case c0_clean_baseline cartographer_m3dgr_mid360_clean_baseline.lua
  run_case c1_innovation1 cartographer_m3dgr_mid360_innovation1.lua
  run_case c2_innovation2 cartographer_m3dgr_mid360_innovation2.lua
  run_case c3_full_proposed cartographer_m3dgr_mid360_full_proposed.lua
fi

echo "[Wheel-float02] Requested cases finished under ${RUNS_DIR}"
