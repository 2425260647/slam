#!/usr/bin/env bash
set -euo pipefail

# Formal, serial experiment runner for the three repeated corridor recordings.
# Every case uses the same projection, scan-quality gate, occupancy parameters,
# playback rate, static transform, and input topics.  Only the named ablation
# variable differs between methods.

WORKSPACE="/home/slam/slam_ws"
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP_ROOT="${EXP_ROOT_OVERRIDE:-${SCRIPT_ROOT}}"
RUNS_DIR="${EXP_ROOT}/runs"
GENERATED_BAGS_DIR="${EXP_ROOT}/generated_bags"
LABELS_DIR="${EXP_ROOT}/anomaly_labels"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"
MAP_SAVER="${WORKSPACE}/paper_exp/innovation1_directional_adaptive_fusion/save_occupancy_grid.py"
PLAYBACK_RATE="2.0"
ALGORITHM_VERSION="innovation_combined_v2_20260730"
SOURCE_MANIFEST="${EXP_ROOT}/algorithm_source_manifest.sha256"
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

mkdir -p "${RUNS_DIR}" "${GENERATED_BAGS_DIR}" "${LABELS_DIR}"

wait_for_ros() {
  for _ in $(seq 1 120); do
    if rosparam list >/dev/null 2>&1; then return 0; fi
    sleep 0.25
  done
  echo "ROS master did not become ready." >&2
  return 1
}

wait_for_service() {
  local service="$1"
  for _ in $(seq 1 160); do
    if rosservice list 2>/dev/null | rg -q "^${service}$"; then return 0; fi
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

normal_bag() {
  printf '%s/bags/corridor_repeat_%02d.bag' "${WORKSPACE}" "$1"
}

anomaly_bag() {
  printf '%s/corridor_repeat_%02d_anomaly.bag' "${GENERATED_BAGS_DIR}" "$1"
}

generate_anomaly_bags() {
  for index in 1 2 3; do
    local output
    output="$(anomaly_bag "${index}")"
    local labels="${LABELS_DIR}/corridor_repeat_$(printf '%02d' "${index}")_events.json"
    if [[ -f "${output}" && -f "${labels}" ]]; then
      echo "[Formal experiment] Reusing generated anomaly bag ${output}"
      continue
    fi
    python3 "${SCRIPT_ROOT}/generate_consistency_anomaly_bag.py" \
      --input "$(normal_bag "${index}")" \
      --output "${output}" \
      --labels "${labels}" \
      --variant "${index}" \
      >"${LABELS_DIR}/corridor_repeat_$(printf '%02d' "${index}")_generation.log"
  done
}

preflight() {
  python3 -m py_compile \
    "${SCRIPT_ROOT}/generate_consistency_anomaly_bag.py" \
    "${MAP_SAVER}"
  for config in \
    innovation1_baseline_a_static_low.lua \
    innovation1_baseline_b_static_high.lua \
    innovation1_proposed_dynamic_anisotropic.lua \
    innovation2_baseline_static_low.lua \
    innovation2_baseline_static_high.lua \
    innovation2_proposed_slip_adaptive.lua; do
    cartographer_print_configuration \
      -configuration_directories "${CONFIG_DIR}" \
      -configuration_basename "${config}" \
      >"${EXP_ROOT}/.${config}.preflight.txt"
  done
  for index in 1 2 3; do
    test -r "$(normal_bag "${index}")"
  done
  sha256sum "${SOURCE_FILES[@]}" >"${SOURCE_MANIFEST}"
  {
    printf 'generated_at_utc=%s\n' "$(date -u +%FT%TZ)"
    printf 'algorithm_version=%s\n' "${ALGORITHM_VERSION}"
    printf 'git_head=%s\n' "$(git -C "${WORKSPACE}" rev-parse HEAD)"
    printf 'source_manifest_sha256=%s\n' \
      "$(sha256sum "${SOURCE_MANIFEST}" | awk '{print $1}')"
    printf 'playback_rate=%s\n' "${PLAYBACK_RATE}"
    printf 'static_tf=0 0 0.33521 -1.5708 0 0 base_link laser_link\n'
    printf 'projection_height=-0.515,1.435\n'
    printf 'scan_quality_gate=true\n'
    sha256sum "$(normal_bag 1)" "$(normal_bag 2)" "$(normal_bag 3)"
  } >"${EXP_ROOT}/formal_input_manifest.txt"
  echo "[Formal experiment] Preflight passed."
}

start_csv_capture() {
  local topic="$1"
  local output="$2"
  rostopic echo -p "${topic}" >"${output}" 2>"${output%.csv}.err" &
  CAPTURE_PIDS+=("$!")
}

run_case() {
  local suite="$1"
  local method="$2"
  local repeat_index="$3"
  local config="$4"
  local bag="$5"
  local run_dir="${RUNS_DIR}/${suite}/${method}/repeat_$(printf '%02d' "${repeat_index}")"

  if [[ -f "${run_dir}/SUCCESS" && "${FORCE:-0}" != "1" ]]; then
    echo "[Formal experiment] Skip completed ${suite}/${method}/repeat_${repeat_index}."
    return 0
  fi
  if [[ -d "${run_dir}" && "${FORCE:-0}" != "1" ]]; then
    echo "Incomplete run directory exists; set FORCE=1 after inspecting it: ${run_dir}" >&2
    return 1
  fi
  if [[ "${FORCE:-0}" == "1" && -d "${run_dir}" ]]; then
    local archived="${run_dir}.legacy_pre_coordinate_fix.$(date +%Y%m%d_%H%M%S)"
    mv "${run_dir}" "${archived}"
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

  echo "[Formal experiment] Start ${suite}/${method}/repeat_${repeat_index}"
  {
    printf 'suite=%s\nmethod=%s\nrepeat=%s\n' "${suite}" "${method}" "${repeat_index}"
    printf 'algorithm_version=%s\n' "${ALGORITHM_VERSION}"
    printf 'git_head=%s\n' "$(git -C "${WORKSPACE}" rev-parse HEAD)"
    printf 'source_manifest_sha256=%s\n' \
      "$(sha256sum "${SOURCE_MANIFEST}" | awk '{print $1}')"
    printf 'config=%s\nbag=%s\n' "${config}" "${bag}"
    printf 'bag_sha256=%s\n' "$(sha256sum "${bag}" | awk '{print $1}')"
    printf 'playback_rate=%s\n' "${PLAYBACK_RATE}"
    printf 'scan_quality_gate=true\n'
    printf 'min_height=-0.515\nmax_height=1.435\n'
    printf 'static_tf_z=0.33521\nstatic_tf_yaw=-1.5708\n'
    printf 'start_wall_utc=%s\n' "$(date -u +%FT%TZ)"
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
    rviz:=false \
    enable_monitor:=false \
    >"${run_dir}/roslaunch.log" 2>&1 & launch_pid=$!

  wait_for_service "/finish_trajectory"
  wait_for_service "/write_state"
  sleep 2

  start_csv_capture /tracked_pose "${run_dir}/tracked_pose.csv"
  start_csv_capture /degeneracy_metric "${run_dir}/degeneracy_metric.csv"
  start_csv_capture /degeneracy_direction "${run_dir}/degeneracy_direction.csv"
  start_csv_capture /slip_metric "${run_dir}/consistency_metric.csv"
  start_csv_capture /slip_state "${run_dir}/consistency_state.csv"
  start_csv_capture /odom_weight_scale "${run_dir}/odom_weight_scale.csv"
  start_csv_capture /odom_min_weight_scale "${run_dir}/odom_min_weight_scale.csv"
  start_csv_capture /slip_lidar_reliability "${run_dir}/lidar_reference_reliability.csv"
  start_csv_capture /consistency_anomaly_trigger_count "${run_dir}/anomaly_trigger_count.csv"
  start_csv_capture /consistency_anomaly_time "${run_dir}/anomaly_time.csv"

  local play_start play_end
  play_start="$(date +%s.%N)"
  rosbag play --clock --quiet --rate="${PLAYBACK_RATE}" "${bag}" \
    --topics /velodyne_points /odom \
    >"${run_dir}/rosbag_play.log" 2>&1
  play_end="$(date +%s.%N)"
  printf 'play_start_wall_sec=%s\nplay_end_wall_sec=%s\n' \
    "${play_start}" "${play_end}" >>"${run_dir}/metadata.txt"
  awk -v start="${play_start}" -v end="${play_end}" \
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

  rosnode list >"${run_dir}/nodes.txt" 2>&1 || true
  rostopic list >"${run_dir}/topics.txt" 2>&1 || true
  if rg -n "FATAL|Check failed|Segmentation fault" "${run_dir}/roslaunch.log" \
      >"${run_dir}/fatal_scan.txt"; then
    echo "Fatal marker found in ${run_dir}/roslaunch.log" >&2
    return 1
  fi
  test -s "${run_dir}/map.pgm"
  test -s "${run_dir}/map.yaml"
  test -s "${run_dir}/state.pbstream"
  printf 'end_wall_utc=%s\n' "$(date -u +%FT%TZ)" >>"${run_dir}/metadata.txt"
  touch "${run_dir}/SUCCESS"

  cleanup_case
  trap - RETURN
  sleep 3
  echo "[Formal experiment] Finished ${suite}/${method}/repeat_${repeat_index}"
}

run_i1() {
  # Rotated deterministic order reduces method-specific thermal/order bias.
  run_case i1 baseline_a 1 innovation1_baseline_a_static_low.lua "$(normal_bag 1)"
  run_case i1 baseline_b 1 innovation1_baseline_b_static_high.lua "$(normal_bag 1)"
  run_case i1 proposed 1 innovation1_proposed_dynamic_anisotropic.lua "$(normal_bag 1)"
  run_case i1 baseline_b 2 innovation1_baseline_b_static_high.lua "$(normal_bag 2)"
  run_case i1 proposed 2 innovation1_proposed_dynamic_anisotropic.lua "$(normal_bag 2)"
  run_case i1 baseline_a 2 innovation1_baseline_a_static_low.lua "$(normal_bag 2)"
  run_case i1 proposed 3 innovation1_proposed_dynamic_anisotropic.lua "$(normal_bag 3)"
  run_case i1 baseline_a 3 innovation1_baseline_a_static_low.lua "$(normal_bag 3)"
  run_case i1 baseline_b 3 innovation1_baseline_b_static_high.lua "$(normal_bag 3)"
}

run_i2() {
  generate_anomaly_bags
  run_case i2 baseline_a 1 innovation2_baseline_static_low.lua "$(anomaly_bag 1)"
  run_case i2 baseline_b 1 innovation2_baseline_static_high.lua "$(anomaly_bag 1)"
  run_case i2 proposed 1 innovation2_proposed_slip_adaptive.lua "$(anomaly_bag 1)"
  run_case i2 baseline_b 2 innovation2_baseline_static_high.lua "$(anomaly_bag 2)"
  run_case i2 proposed 2 innovation2_proposed_slip_adaptive.lua "$(anomaly_bag 2)"
  run_case i2 baseline_a 2 innovation2_baseline_static_low.lua "$(anomaly_bag 2)"
  run_case i2 proposed 3 innovation2_proposed_slip_adaptive.lua "$(anomaly_bag 3)"
  run_case i2 baseline_a 3 innovation2_baseline_static_low.lua "$(anomaly_bag 3)"
  run_case i2 baseline_b 3 innovation2_baseline_static_high.lua "$(anomaly_bag 3)"
}

run_normal_full() {
  for index in 1 2 3; do
    run_case normal_full proposed "${index}" \
      innovation2_proposed_slip_adaptive.lua "$(normal_bag "${index}")"
  done
}

run_i1_coordinate_fix_validation() {
  # Keep the post-fix validation isolated from the frozen formal matrix.
  run_case i1_coordinate_fix_validation proposed_map_direction 3 \
    innovation1_proposed_dynamic_anisotropic.lua "$(normal_bag 3)"
}

usage() {
  echo "Usage: $0 {--preflight|--generate|--precheck-i2|--i1|--i2|--normal|--i1-coordinate-fix-validation|--all}" >&2
}

case "${1:-}" in
  --preflight) preflight ;;
  --generate) generate_anomaly_bags ;;
  --precheck-i2)
    preflight
    generate_anomaly_bags
    run_case precheck_i2 proposed 1 innovation2_proposed_slip_adaptive.lua \
      "$(anomaly_bag 1)"
    ;;
  --i1) preflight; run_i1 ;;
  --i2) preflight; run_i2 ;;
  --normal) preflight; run_normal_full ;;
  --i1-coordinate-fix-validation)
    preflight
    run_i1_coordinate_fix_validation
    ;;
  --all) preflight; generate_anomaly_bags; run_i1; run_i2; run_normal_full ;;
  *) usage; exit 2 ;;
esac
