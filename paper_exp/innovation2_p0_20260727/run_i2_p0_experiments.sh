#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/slam_ws"
ROOT="${WORKSPACE}/paper_exp/innovation2_p0_20260727"
RUNS="${ROOT}/runs"
BAGS="${ROOT}/generated_bags"
LABELS="${ROOT}/labels"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"
GENERATOR="${ROOT}/generate_i2_pressure_bag.py"
MAP_SAVER="${WORKSPACE}/paper_exp/innovation1_directional_adaptive_fusion/save_occupancy_grid.py"
RATE="2.0"
PRESSURE_VERSION="2"
ALGORITHM_VERSION="innovation1_coordinate_fix_20260728"
SOURCE_MANIFEST="${ROOT}/algorithm_source_manifest.sha256"
SOURCE_FILES=(
  "${WORKSPACE}/src/cartographer/cartographer/mapping/directional_degeneracy_metric.h"
  "${WORKSPACE}/src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc"
  "${WORKSPACE}/src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
  "${WORKSPACE}/src/cartographer/cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
  "${WORKSPACE}/src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.cc"
  "${WORKSPACE}/src/cartographer/cartographer/mapping/internal/optimization/slip_detector.h"
  "${WORKSPACE}/src/cartographer_ros/cartographer_ros/cartographer_ros/node.cc"
  "${WORKSPACE}/src/cartographer_ros/cartographer_ros/cartographer_ros/node.h"
  "${WORKSPACE}/src/my_navigation/src/mapping_health_monitor.cpp"
)

export ROS_HOME="${WORKSPACE}/.ros"
source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"

mkdir -p "${RUNS}" "${BAGS}" "${LABELS}"

normal_bag() {
  printf '%s/bags/corridor_repeat_%02d.bag' "${WORKSPACE}" "$1"
}

pressure_bag() {
  printf '%s/corridor_repeat_%02d_odom_pressure_v%s.bag' \
    "${BAGS}" "$1" "${PRESSURE_VERSION}"
}

gate_bag() {
  printf '%s/corridor_repeat_%02d_lidar_gate_v1.bag' "${BAGS}" "$1"
}

generate_one() {
  local mode="$1" index="$2" output labels experiment_version
  if [[ "${mode}" == "odom_pressure" ]]; then
    output="$(pressure_bag "${index}")"
    experiment_version="${PRESSURE_VERSION}"
  else
    output="$(gate_bag "${index}")"
    experiment_version="1"
  fi
  labels="${LABELS}/corridor_repeat_$(printf '%02d' "${index}")_${mode}_v${experiment_version}.json"
  if [[ -s "${output}" && -s "${labels}" ]]; then
    return
  fi
  python3 "${GENERATOR}" \
    --input "$(normal_bag "${index}")" \
    --output "${output}" \
    --labels "${labels}" \
    --mode "${mode}" \
    --variant "${index}" \
    --experiment-version "${experiment_version}" \
    >"${labels%.json}.generation.log"
}

preflight() {
  python3 -m py_compile "${GENERATOR}" "${MAP_SAVER}"
  for config in \
    innovation2_baseline_static_low.lua \
    innovation2_baseline_static_high.lua \
    innovation2_proposed_slip_adaptive.lua \
    innovation2_proposed_no_reliability_gate.lua; do
    cartographer_print_configuration \
      -configuration_directories "${CONFIG_DIR}" \
      -configuration_basename "${config}" \
      >"${ROOT}/.${config}.preflight.txt"
  done
  for index in 1 2 3; do test -r "$(normal_bag "${index}")"; done
  sha256sum "${SOURCE_FILES[@]}" >"${SOURCE_MANIFEST}"
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

stop_pid() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  fi
}

start_capture() {
  local topic="$1" output="$2"
  rostopic echo -p "${topic}" >"${output}" 2>"${output%.csv}.err" &
  CAPTURE_PIDS+=("$!")
}

run_case() {
  local suite="$1" method="$2" index="$3" config="$4" bag="$5"
  local run_dir="${RUNS}/${suite}/${method}/repeat_$(printf '%02d' "${index}")"
  if [[ -f "${run_dir}/SUCCESS" && "${FORCE:-0}" != "1" ]]; then
    echo "[I2 P0] Skip ${suite}/${method}/repeat_${index}"
    return
  fi
  if [[ -d "${run_dir}" ]]; then
    if [[ "${FORCE:-0}" != "1" ]]; then
      echo "Incomplete run exists: ${run_dir}" >&2
      return 1
    fi
    mv "${run_dir}" \
      "${run_dir}.legacy_pre_coordinate_fix.$(date +%Y%m%d_%H%M%S)"
  fi
  mkdir -p "${run_dir}"

  local roscore_pid="" launch_pid="" tf_pid=""
  CAPTURE_PIDS=()
  cleanup_case() {
    local pid
    for pid in "${CAPTURE_PIDS[@]:-}"; do stop_pid "${pid}"; done
    stop_pid "${tf_pid}"
    stop_pid "${launch_pid}"
    stop_pid "${roscore_pid}"
  }
  trap cleanup_case RETURN

  {
    printf 'suite=%s\nmethod=%s\nrepeat=%s\n' "${suite}" "${method}" "${index}"
    printf 'algorithm_version=%s\n' "${ALGORITHM_VERSION}"
    printf 'config=%s\nbag=%s\n' "${config}" "${bag}"
    printf 'bag_sha256=%s\n' "$(sha256sum "${bag}" | awk '{print $1}')"
    printf 'config_sha256=%s\n' "$(sha256sum "${CONFIG_DIR}/${config}" | awk '{print $1}')"
    printf 'git_head=%s\n' "$(git -C "${WORKSPACE}" rev-parse HEAD)"
    printf 'source_manifest_sha256=%s\n' \
      "$(sha256sum "${SOURCE_MANIFEST}" | awk '{print $1}')"
    printf 'playback_rate=%s\nscan_quality_gate=true\n' "${RATE}"
    printf 'static_tf_z=0.33521\nstatic_tf_yaw=-1.5708\n'
    printf 'min_height=-0.515\nmax_height=1.435\n'
  } >"${run_dir}/metadata.txt"
  cp "${SOURCE_MANIFEST}" "${run_dir}/algorithm_source_manifest.sha256"
  cp "${CONFIG_DIR}/${config}" "${run_dir}/config_snapshot.lua"
  cartographer_print_configuration \
    -configuration_directories "${CONFIG_DIR}" \
    -configuration_basename "${config}" \
    >"${run_dir}/resolved_configuration.lua"

  roscore >"${run_dir}/roscore.log" 2>&1 & roscore_pid=$!
  wait_for_ros
  rosparam set /use_sim_time true
  rosrun tf2_ros static_transform_publisher \
    0 0 0.33521 -1.5708 0 0 base_link laser_link \
    >"${run_dir}/static_tf.log" 2>&1 & tf_pid=$!
  roslaunch my_navigation real_scout_mapping_local_grid.launch \
    cartographer_config_dir:="${CONFIG_DIR}" \
    cartographer_config:="${config}" \
    cloud_topic:=/velodyne_points \
    odom_topic:=/odom \
    target_frame:=laser_link \
    min_height:=-0.515 \
    max_height:=1.435 \
    scan_quality_gate_enabled:=true \
    rviz:=false enable_monitor:=false \
    >"${run_dir}/roslaunch.log" 2>&1 & launch_pid=$!
  wait_for_service /finish_trajectory
  wait_for_service /write_state
  sleep 2

  start_capture /tracked_pose "${run_dir}/tracked_pose.csv"
  start_capture /degeneracy_metric "${run_dir}/degeneracy_metric.csv"
  start_capture /degeneracy_direction "${run_dir}/degeneracy_direction.csv"
  start_capture /slip_metric "${run_dir}/consistency_metric.csv"
  start_capture /slip_state "${run_dir}/consistency_state.csv"
  start_capture /odom_weight_scale "${run_dir}/odom_weight_scale.csv"
  start_capture /odom_min_weight_scale "${run_dir}/odom_min_weight_scale.csv"
  start_capture /slip_lidar_reliability "${run_dir}/lidar_reliability.csv"
  start_capture /consistency_anomaly_trigger_count "${run_dir}/trigger_count.csv"
  start_capture /consistency_anomaly_time "${run_dir}/anomaly_time.csv"

  local start end
  start="$(date +%s.%N)"
  rosbag play --clock --quiet --rate="${RATE}" "${bag}" \
    --topics /velodyne_points /odom \
    >"${run_dir}/rosbag_play.log" 2>&1
  end="$(date +%s.%N)"
  awk -v start="${start}" -v end="${end}" \
    'BEGIN {printf "play_wall_duration_sec=%.6f\n", end-start}' \
    >>"${run_dir}/metadata.txt"

  sleep 5
  rosservice call /finish_trajectory "trajectory_id: 0" \
    >"${run_dir}/finish_trajectory.log" 2>&1
  sleep 12
  rosservice call /write_state \
    "{filename: '${run_dir}/state.pbstream', include_unfinished_submaps: true}" \
    >"${run_dir}/write_state.log" 2>&1
  python3 "${MAP_SAVER}" --topic /map \
    --output-prefix "${run_dir}/map" --timeout 45 \
    >"${run_dir}/map_saver.log" 2>&1

  if rg -n "FATAL|Check failed|Segmentation fault" "${run_dir}/roslaunch.log" \
      >"${run_dir}/fatal_scan.txt"; then
    return 1
  fi
  test -s "${run_dir}/map.pgm"
  test -s "${run_dir}/map.yaml"
  test -s "${run_dir}/state.pbstream"
  touch "${run_dir}/SUCCESS"
  cleanup_case
  trap - RETURN
  sleep 3
}

generate_all() {
  for index in 1 2 3; do
    generate_one odom_pressure "${index}"
    generate_one lidar_gate "${index}"
  done
}

run_map_index() {
  local index="$1" bag
  generate_one odom_pressure "${index}"
  bag="$(pressure_bag "${index}")"
  run_case map_protection_v2 static_low "${index}" \
    innovation2_baseline_static_low.lua "${bag}"
  run_case map_protection_v2 static_high "${index}" \
    innovation2_baseline_static_high.lua "${bag}"
  run_case map_protection_v2 proposed_gate "${index}" \
    innovation2_proposed_slip_adaptive.lua "${bag}"
}

run_gate_index() {
  local index="$1" bag
  generate_one lidar_gate "${index}"
  bag="$(gate_bag "${index}")"
  run_case gate_ablation gate_off "${index}" \
    innovation2_proposed_no_reliability_gate.lua "${bag}"
  run_case gate_ablation gate_on "${index}" \
    innovation2_proposed_slip_adaptive.lua "${bag}"
}

case "${1:-}" in
  --preflight) preflight ;;
  --generate-all) preflight; generate_all ;;
  --precheck-map) preflight; run_map_index 3 ;;
  --precheck-gate) preflight; run_gate_index 3 ;;
  --formal-map) preflight; for index in 1 2 3; do run_map_index "${index}"; done ;;
  --formal-gate) preflight; for index in 1 2 3; do run_gate_index "${index}"; done ;;
  --all) preflight; generate_all; for index in 1 2 3; do run_map_index "${index}"; run_gate_index "${index}"; done ;;
  *) echo "Usage: $0 {--preflight|--generate-all|--precheck-map|--precheck-gate|--formal-map|--formal-gate|--all}" >&2; exit 2 ;;
esac
