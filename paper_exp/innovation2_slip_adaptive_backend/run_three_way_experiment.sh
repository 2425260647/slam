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
CONFIG_DIR="/home/slam/slam_ws/src/my_navigation/config"
RUNS_DIR="${EXP_ROOT}/runs"
mkdir -p "${RUNS_DIR}"

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
    echo "[Innovation2 Experiment] Reusing ${SLIP_BAG}"
    return 0
  fi
  python3 "${EXP_ROOT}/generate_slip_injected_bag.py" \
    --input "${SOURCE_BAG}" \
    --output "${SLIP_BAG}"
}

run_case() {
  local case_id="$1"
  local config="$2"
  local bag="$3"
  local run_dir="${RUNS_DIR}/${case_id}"
  rm -rf "${run_dir}"
  mkdir -p "${run_dir}"

  echo "[Innovation2 Experiment] Running ${case_id} with ${config}"

  local roscore_pid=""
  local launch_pid=""
  local tf_pid=""
  local slip_metric_pid=""
  local slip_state_pid=""
  local weight_scale_pid=""
  local lidar_reliability_pid=""
  local degeneracy_metric_pid=""
  local scan_hz_pid=""
  local odom_hz_pid=""

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
  rostopic echo -p /degeneracy_metric >"${run_dir}/degeneracy_metric.csv" 2>"${run_dir}/degeneracy_metric.err" &
  degeneracy_metric_pid=$!
  timeout 45s rostopic hz /scan >"${run_dir}/scan_hz.log" 2>&1 &
  scan_hz_pid=$!
  timeout 45s rostopic hz /odom >"${run_dir}/odom_hz.log" 2>&1 &
  odom_hz_pid=$!

  rosbag play --clock --quiet "${bag}" --topics /velodyne_points /odom \
    >"${run_dir}/rosbag_play.log" 2>&1

  sleep 12

  timeout 8s rostopic echo -n 1 /map/info >"${run_dir}/map_info.txt" 2>&1 || true
  rosnode list >"${run_dir}/nodes.txt" 2>&1 || true

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
  stop_pid "${scan_hz_pid}"
  stop_pid "${odom_hz_pid}"
  stop_pid "${tf_pid}"
  stop_pid "${launch_pid}"
  stop_pid "${roscore_pid}"
  sleep 3
}

ensure_slip_bag
run_case "reference_original_proposed" "innovation2_proposed_slip_adaptive.lua" "${SOURCE_BAG}"
run_case "slip_static_low" "innovation2_baseline_static_low.lua" "${SLIP_BAG}"
run_case "slip_static_high" "innovation2_baseline_static_high.lua" "${SLIP_BAG}"
run_case "slip_proposed_adaptive" "innovation2_proposed_slip_adaptive.lua" "${SLIP_BAG}"

echo "[Innovation2 Experiment] All cases finished. Outputs: ${RUNS_DIR}"
