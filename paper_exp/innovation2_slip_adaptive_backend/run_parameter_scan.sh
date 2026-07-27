#!/usr/bin/env bash
set -euo pipefail

cd /home/slam/slam_ws
export ROS_HOME=/home/slam/slam_ws/.ros
source /opt/ros/noetic/setup.bash
source install_isolated/setup.bash

EXP_ROOT="/home/slam/slam_ws/paper_exp/innovation2_slip_adaptive_backend"
SOURCE_BAG="/home/slam/slam_ws/bags/corridor_01.bag"
SLIP_BAG="${EXP_ROOT}/corridor_01_slip_injected.bag"
LAUNCH_PKG="my_navigation"
LAUNCH_FILE="real_scout_mapping_local_grid.launch"
# [Innovation 2] Cartographer's Lua include resolver searches the configured
# directory; generated scan configs therefore live next to cartographer_scout_2d.lua.
SCAN_CONFIG_DIR="/home/slam/slam_ws/src/my_navigation/config"
SCAN_RUNS_DIR="${EXP_ROOT}/parameter_scan_runs"
mkdir -p "${SCAN_CONFIG_DIR}" "${SCAN_RUNS_DIR}"

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

ensure_slip_bag() {
  if [[ -f "${SLIP_BAG}" ]]; then
    echo "[Innovation2 Scan] Reusing ${SLIP_BAG}"
    return 0
  fi
  python3 "${EXP_ROOT}/generate_slip_injected_bag.py" \
    --input "${SOURCE_BAG}" \
    --output "${SLIP_BAG}"
}

write_config() {
  local config_path="$1"
  local reliability_min="$2"
  local high_threshold="$3"
  local low_threshold="$4"
  {
    printf 'include "cartographer_scout_2d.lua"\n\n'
    printf '%s\n' '-- [Innovation 2 parameter scan] Generated config.'
    printf 'POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = true\n'
    printf 'POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5\n'
    printf 'POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5\n'
    printf 'POSE_GRAPH.optimization_problem.slip_lateral_error_weight = 1.0\n'
    printf 'POSE_GRAPH.optimization_problem.slip_yaw_error_weight = 1.0\n'
    printf 'POSE_GRAPH.optimization_problem.slip_high_threshold = %.6f\n' "${high_threshold}"
    printf 'POSE_GRAPH.optimization_problem.slip_low_threshold = %.6f\n' "${low_threshold}"
    printf 'POSE_GRAPH.optimization_problem.slip_min_weight_scale = 0.1\n'
    printf 'POSE_GRAPH.optimization_problem.slip_max_weight_scale = 1.0\n'
    printf 'POSE_GRAPH.optimization_problem.slip_recovery_alpha = 0.05\n'
    printf 'POSE_GRAPH.optimization_problem.slip_min_motion_distance = 0.02\n'
    printf 'POSE_GRAPH.optimization_problem.slip_min_motion_angle = 0.01\n'
    printf 'POSE_GRAPH.optimization_problem.slip_lidar_reliability_gate_enabled = true\n'
    printf 'POSE_GRAPH.optimization_problem.slip_lidar_reliability_min = %.6f\n' "${reliability_min}"
    printf 'POSE_GRAPH.optimization_problem.slip_degeneracy_metric_max_time_delta_sec = 0.25\n'
    printf 'POSE_GRAPH.optimization_problem.slip_unknown_keep_previous_weight = true\n\n'
    printf 'return options\n'
  } >"${config_path}"
}

run_case() {
  local case_id="$1"
  local config_basename="$2"
  local bag="$3"
  local run_dir="${SCAN_RUNS_DIR}/${case_id}"
  rm -rf "${run_dir}"
  mkdir -p "${run_dir}"

  echo "[Innovation2 Scan] Running ${case_id} with ${config_basename}"

  local roscore_pid=""
  local launch_pid=""
  local tf_pid=""
  local slip_metric_pid=""
  local slip_state_pid=""
  local weight_scale_pid=""
  local lidar_reliability_pid=""
  local degeneracy_metric_pid=""

  roscore >"${run_dir}/roscore.log" 2>&1 &
  roscore_pid=$!
  wait_for_ros
  rosparam set /use_sim_time true

  roslaunch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
    cartographer_config_dir:="${SCAN_CONFIG_DIR}" \
    cartographer_config:="${config_basename}" \
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
  rostopic echo -p /degeneracy_metric >"${run_dir}/degeneracy_metric.csv" 2>"${run_dir}/degeneracy_metric.err" &
  degeneracy_metric_pid=$!

  rosbag play --clock --quiet "${bag}" --topics /velodyne_points /odom \
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
  stop_pid "${degeneracy_metric_pid}"
  stop_pid "${tf_pid}"
  stop_pid "${launch_pid}"
  stop_pid "${roscore_pid}"
  sleep 3
}

ensure_slip_bag

PARAMS=(
  "scan_r045_h0025 0.45 0.025 0.010"
  "scan_r045_h0030 0.45 0.030 0.012"
  "scan_r050_h0025 0.50 0.025 0.010"
  "scan_r050_h0030 0.50 0.030 0.012"
  "scan_r055_h0025 0.55 0.025 0.010"
  "scan_r055_h0030 0.55 0.030 0.012"
)

for entry in "${PARAMS[@]}"; do
  read -r case_id reliability_min high_threshold low_threshold <<<"${entry}"
  config_basename="${case_id}.lua"
  write_config "${SCAN_CONFIG_DIR}/${config_basename}" \
    "${reliability_min}" "${high_threshold}" "${low_threshold}"
  run_case "${case_id}" "${config_basename}" "${SLIP_BAG}"
done

echo "[Innovation2 Scan] Finished. Outputs: ${SCAN_RUNS_DIR}"
