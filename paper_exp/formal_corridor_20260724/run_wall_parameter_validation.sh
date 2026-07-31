#!/usr/bin/env bash
set -euo pipefail

# [Experiment] Blind validation of the wall probability selected on
# corridor_repeat_03. Innovations and the scan quality gate stay disabled so
# this run measures transfer of the frozen probability setting only.
WORKSPACE="/home/slam/slam_ws"
EXP_ROOT="${EXP_ROOT:-${WORKSPACE}/paper_exp/formal_corridor_20260724/wall_parameter_validation}"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"
CONFIG_FILE="${CONFIG_DIR}/.wall_validation_frozen.lua"
HIT="${HIT:-0.58}"
MISS="${MISS:-0.495}"

export ROS_HOME="${WORKSPACE}/.ros"
source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"
mkdir -p "${EXP_ROOT}"

printf '%s\n' \
  'include "clean_baseline_no_innovation.lua"' \
  "TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = ${HIT}" \
  "TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = ${MISS}" \
  'return options' \
  >"${CONFIG_FILE}"

wait_for_ros() {
  for _ in $(seq 1 80); do
    if rosparam list >/dev/null 2>&1; then return 0; fi
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

run_case() {
  local case_id="$1"
  local bag="$2"
  local run_dir="${EXP_ROOT}/${case_id}"
  local roscore_pid="" launch_pid="" tf_pid=""

  mkdir -p "${run_dir}"
  echo "[Wall validation] ${case_id} hit=${HIT} miss=${MISS}"

  cleanup_case() {
    stop_pid "${tf_pid}"
    stop_pid "${launch_pid}"
    stop_pid "${roscore_pid}"
  }
  trap cleanup_case RETURN

  roscore >"${run_dir}/roscore.log" 2>&1 & roscore_pid=$!
  wait_for_ros
  rosparam set /use_sim_time true

  roslaunch my_navigation real_scout_mapping_local_grid.launch \
    cartographer_config_dir:="${CONFIG_DIR}" \
    cartographer_config:="$(basename "${CONFIG_FILE}")" \
    cloud_topic:=/velodyne_points \
    odom_topic:=/odom \
    target_frame:=laser_link \
    min_height:=-0.515 \
    max_height:=1.435 \
    scan_quality_gate_enabled:=false \
    rviz:=false \
    enable_monitor:=false \
    >"${run_dir}/roslaunch.log" 2>&1 & launch_pid=$!

  sleep 6
  for _ in $(seq 1 80); do
    if rosservice list 2>/dev/null | rg -q '^/finish_trajectory$'; then
      break
    fi
    sleep 0.25
  done
  rosservice list 2>/dev/null | rg -q '^/finish_trajectory$'
  rosrun tf2_ros static_transform_publisher \
    0 0 0.33521 -1.5708 0 0 base_link laser_link \
    >"${run_dir}/static_tf.log" 2>&1 & tf_pid=$!
  sleep 3

  rosbag play --clock --quiet --rate=2.0 "${bag}" \
    --topics /velodyne_points /odom \
    >"${run_dir}/rosbag_play.log" 2>&1

  sleep 5
  rosservice call /finish_trajectory 0 \
    >"${run_dir}/finish_trajectory.log" 2>&1
  sleep 10
  rosservice call /write_state \
    "{filename: '${run_dir}/state.pbstream', include_unfinished_submaps: true}" \
    >"${run_dir}/write_state.log" 2>&1
  python3 "${WORKSPACE}/paper_exp/innovation1_directional_adaptive_fusion/save_occupancy_grid.py" \
    --topic /map --output-prefix "${run_dir}/map" --timeout 30 \
    >"${run_dir}/map_saver.log" 2>&1

  cleanup_case
  trap - RETURN
  sleep 3
  echo "[Wall validation] Finished ${case_id}"
}

trap 'rm -f "${CONFIG_FILE}"' EXIT
run_case "repeat_01" "${WORKSPACE}/bags/corridor_repeat_01.bag"
run_case "repeat_02" "${WORKSPACE}/bags/corridor_repeat_02.bag"

python3 "${WORKSPACE}/paper_exp/formal_corridor_20260724/evaluate_wall_parameter_maps.py" \
  --root "${EXP_ROOT}" \
  --output "${EXP_ROOT}/repeat_01_02_wall_parameter_metrics.json" \
  >"${EXP_ROOT}/repeat_01_02_wall_parameter_metrics.stdout"

echo "[Wall validation] Results: ${EXP_ROOT}"
