#!/usr/bin/env bash
set -euo pipefail

cd /home/slam/slam_ws
export ROS_HOME=/home/slam/slam_ws/.ros
source /opt/ros/noetic/setup.bash
source install_isolated/setup.bash

EXP_ROOT="/home/slam/slam_ws/paper_exp/innovation2_slip_adaptive_backend"
SOURCE_BAG="/home/slam/slam_ws/bags/corridor_01.bag"
LAUNCH_PKG="my_navigation"
LAUNCH_FILE="real_scout_mapping_local_grid.launch"
CONFIG_DIR="/home/slam/slam_ws/src/my_navigation/config"
RUNS_DIR="${EXP_ROOT}/reference_candidate_runs"
mkdir -p "${RUNS_DIR}"

write_config() {
  local config_path="$1"
  local reliability_min="$2"
  local high_threshold="$3"
  local low_threshold="$4"
  printf '%s\n' \
    'include "cartographer_scout_2d.lua"' \
    '' \
    '-- [Innovation 2] Generated reference-candidate config.' \
    'POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = true' \
    'POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5' \
    'POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5' \
    "POSE_GRAPH.optimization_problem.slip_high_threshold = ${high_threshold}" \
    "POSE_GRAPH.optimization_problem.slip_low_threshold = ${low_threshold}" \
    'POSE_GRAPH.optimization_problem.slip_lidar_reliability_gate_enabled = true' \
    "POSE_GRAPH.optimization_problem.slip_lidar_reliability_min = ${reliability_min}" \
    '' \
    'return options' \
    >"${config_path}"
}

cleanup_configs() {
  rm -f "${CONFIG_DIR}/scan_r055_h0025.lua"
  rm -f "${CONFIG_DIR}/scan_r050_h0030.lua"
}
trap cleanup_configs EXIT

wait_for_ros() {
  for _ in $(seq 1 80); do
    if rosparam list >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.25
  done
  echo "ROS master did not become ready." >&2
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
  local config="$2"
  local run_dir="${RUNS_DIR}/${case_id}"
  rm -rf "${run_dir}"
  mkdir -p "${run_dir}"

  echo "[Innovation2 Reference Scan] Running ${case_id} with ${config}"

  local roscore_pid=""
  local launch_pid=""
  local tf_pid=""
  local slip_metric_pid=""
  local slip_state_pid=""
  local weight_scale_pid=""
  local lidar_reliability_pid=""

  roscore >"${run_dir}/roscore.log" 2>&1 &
  roscore_pid=$!
  wait_for_ros
  rosparam set /use_sim_time true

  roslaunch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
    cartographer_config_dir:="${CONFIG_DIR}" \
    cartographer_config:="${config}" \
    cloud_topic:=/velodyne_points \
    target_frame:=laser_link \
    min_height:=-0.35 \
    max_height:=1.60 \
    odom_topic:=/odom \
    rviz:=false \
    enable_monitor:=false \
    >"${run_dir}/roslaunch.log" 2>&1 &
  launch_pid=$!

  sleep 6

  rosrun tf2_ros static_transform_publisher \
    0 0 0.17 -1.5708 0 0 base_link laser_link \
    >"${run_dir}/static_tf.log" 2>&1 &
  tf_pid=$!

  sleep 3

  rostopic echo -p /slip_metric >"${run_dir}/slip_metric.csv" 2>"${run_dir}/slip_metric.err" &
  slip_metric_pid=$!
  rostopic echo -p /slip_state >"${run_dir}/slip_state.csv" 2>"${run_dir}/slip_state.err" &
  slip_state_pid=$!
  rostopic echo -p /odom_weight_scale >"${run_dir}/odom_weight_scale.csv" 2>"${run_dir}/odom_weight_scale.err" &
  weight_scale_pid=$!
  rostopic echo -p /slip_lidar_reliability >"${run_dir}/slip_lidar_reliability.csv" 2>"${run_dir}/slip_lidar_reliability.err" &
  lidar_reliability_pid=$!

  rosbag play --clock --quiet "${SOURCE_BAG}" --topics /velodyne_points /odom \
    >"${run_dir}/rosbag_play.log" 2>&1

  sleep 12

  python3 "${EXP_ROOT}/save_occupancy_grid.py" \
    --topic /map \
    --output-prefix "${run_dir}/map" \
    --timeout 30 \
    >"${run_dir}/map_saver.log" 2>&1

  stop_pid "${slip_metric_pid}"
  stop_pid "${slip_state_pid}"
  stop_pid "${weight_scale_pid}"
  stop_pid "${lidar_reliability_pid}"
  stop_pid "${tf_pid}"
  stop_pid "${launch_pid}"
  stop_pid "${roscore_pid}"
  sleep 3
}

write_config "${CONFIG_DIR}/scan_r055_h0025.lua" 0.55 0.025 0.010
write_config "${CONFIG_DIR}/scan_r050_h0030.lua" 0.50 0.030 0.012

run_case "ref_scan_r055_h0025" "scan_r055_h0025.lua"
run_case "ref_scan_r050_h0030" "scan_r050_h0030.lua"

echo "[Innovation2 Reference Scan] Finished. Outputs: ${RUNS_DIR}"
