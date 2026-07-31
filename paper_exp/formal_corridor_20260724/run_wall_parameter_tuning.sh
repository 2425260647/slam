#!/usr/bin/env bash
set -euo pipefail

# [Experiment] Calibrate only hit/miss probability on repeat_03. Innovations
# remain disabled so this experiment isolates probability-grid wall erasure.
WORKSPACE="/home/slam/slam_ws"
EXP_ROOT="${WORKSPACE}/paper_exp/formal_corridor_20260724/wall_parameter_tuning"
RUN_ROOT="${EXP_ROOT}/runs"
BAG="${WORKSPACE}/bags/corridor_repeat_03.bag"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"
BASE_CONFIG="clean_baseline_no_innovation.lua"
RATE="${RATE:-2.0}"

export ROS_HOME="${WORKSPACE}/.ros"
source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"
mkdir -p "${RUN_ROOT}"

declare -a CASES=(
  "baseline_055_049:0.55:0.490"
  "hit_056_049:0.56:0.490"
  "hit_058_049:0.58:0.490"
  "hit_060_049:0.60:0.490"
  "hit_058_495:0.58:0.495"
)

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
  local spec="$1"
  local case_id hit miss
  IFS=: read -r case_id hit miss <<<"${spec}"
  local run_dir="${RUN_ROOT}/${case_id}"
  local config_file="${CONFIG_DIR}/.wall_tuning_${case_id}.lua"
  local roscore_pid="" launch_pid="" tf_pid=""
  mkdir -p "${run_dir}"

  printf '%s\n' \
    "include \"${BASE_CONFIG}\"" \
    "TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = ${hit}" \
    "TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = ${miss}" \
    "return options" >"${config_file}"

  cleanup_case() {
    stop_pid "${tf_pid}"
    stop_pid "${launch_pid}"
    stop_pid "${roscore_pid}"
    rm -f "${config_file}"
  }
  trap cleanup_case RETURN

  echo "[Wall tuning] ${case_id} hit=${hit} miss=${miss}"
  roscore >"${run_dir}/roscore.log" 2>&1 & roscore_pid=$!
  wait_for_ros
  rosparam set /use_sim_time true

  roslaunch my_navigation real_scout_mapping_local_grid.launch \
    cartographer_config_dir:="${CONFIG_DIR}" \
    cartographer_config:="$(basename "${config_file}")" \
    cloud_topic:=/velodyne_points odom_topic:=/odom target_frame:=laser_link \
    min_height:=-0.515 max_height:=1.435 \
    scan_quality_gate_enabled:=false rviz:=false enable_monitor:=false \
    >"${run_dir}/roslaunch.log" 2>&1 & launch_pid=$!
  sleep 6
  rosrun tf2_ros static_transform_publisher \
    0 0 0.33521 -1.5708 0 0 base_link laser_link \
    >"${run_dir}/static_tf.log" 2>&1 & tf_pid=$!
  sleep 3

  rosbag play --clock --quiet --rate="${RATE}" "${BAG}" \
    --topics /velodyne_points /odom \
    >"${run_dir}/rosbag_play.log" 2>&1
  sleep 5
  rosservice call /finish_trajectory 0 >"${run_dir}/finish_trajectory.log" 2>&1
  sleep 10
  rosservice call /write_state \
    "{filename: '${run_dir}/state.pbstream', include_unfinished_submaps: true}" \
    >"${run_dir}/write_state.log" 2>&1
  python3 "${WORKSPACE}/paper_exp/innovation1_directional_adaptive_fusion/save_occupancy_grid.py" \
    --topic /map --output-prefix "${run_dir}/map" --timeout 30 \
    >"${run_dir}/map_saver.log" 2>&1
  cleanup_case
  trap - RETURN
}

for case in "${CASES[@]}"; do run_case "${case}"; done

python3 "${WORKSPACE}/paper_exp/formal_corridor_20260724/evaluate_wall_parameter_maps.py" \
  --root "${RUN_ROOT}" --output "${EXP_ROOT}/repeat_03_wall_parameter_metrics.json" \
  >"${EXP_ROOT}/repeat_03_wall_parameter_metrics.stdout" 

echo "[Wall tuning] Results: ${EXP_ROOT}"
