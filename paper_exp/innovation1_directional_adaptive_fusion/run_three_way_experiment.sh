#!/usr/bin/env bash
set -euo pipefail

cd /home/slam/slam_ws
source /opt/ros/noetic/setup.bash
source install_isolated/setup.bash

EXP_ROOT="/home/slam/slam_ws/paper_exp/innovation1_directional_adaptive_fusion"
BAG="/home/slam/slam_ws/bags/corridor_01.bag"
LAUNCH_PKG="my_navigation"
LAUNCH_FILE="real_scout_mapping_local_grid.launch"
CONFIG_DIR="/home/slam/slam_ws/src/my_navigation/config"
RUNS_DIR="${EXP_ROOT}/runs_valid"
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

run_case() {
  local case_id="$1"
  local config="$2"
  local run_dir="${RUNS_DIR}/${case_id}"
  mkdir -p "${run_dir}"

  echo "[Innovation1 Experiment] Running ${case_id} with ${config}"
  if [[ -f "${run_dir}/map.pgm" && -f "${run_dir}/map.yaml" ]]; then
    echo "[Innovation1 Experiment] Skip ${case_id}; map already exists."
    return 0
  fi
  if [[ ! -f "${CONFIG_DIR}/${config}" ]]; then
    echo "Missing config: ${CONFIG_DIR}/${config}" >&2
    return 1
  fi

  local roscore_pid=""
  local launch_pid=""
  local tf_pid=""
  local metric_pid=""
  local direction_pid=""
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

  rostopic echo -p /degeneracy_metric >"${run_dir}/degeneracy_metric.csv" 2>"${run_dir}/degeneracy_metric.err" &
  metric_pid=$!
  rostopic echo -p /degeneracy_direction >"${run_dir}/degeneracy_direction.csv" 2>"${run_dir}/degeneracy_direction.err" &
  direction_pid=$!
  timeout 45s rostopic hz /scan >"${run_dir}/scan_hz.log" 2>&1 &
  scan_hz_pid=$!
  timeout 45s rostopic hz /odom >"${run_dir}/odom_hz.log" 2>&1 &
  odom_hz_pid=$!

  rosbag play --clock --quiet "${BAG}" --topics /velodyne_points /odom \
    >"${run_dir}/rosbag_play.log" 2>&1

  sleep 10

  timeout 8s rostopic echo -n 1 /map/info >"${run_dir}/map_info.txt" 2>&1 || true
  timeout 8s rostopic echo -n 1 /local_occupancy_grid/header >"${run_dir}/local_grid_header.txt" 2>&1 || true
  rosnode list >"${run_dir}/nodes.txt" 2>&1 || true

  python3 "${EXP_ROOT}/save_occupancy_grid.py" \
    --topic /map \
    --output-prefix "${run_dir}/map" \
    --timeout 30 \
    >"${run_dir}/map_saver.log" 2>&1

  stop_pid "${metric_pid}"
  stop_pid "${direction_pid}"
  stop_pid "${scan_hz_pid}"
  stop_pid "${odom_hz_pid}"
  stop_pid "${tf_pid}"
  stop_pid "${launch_pid}"
  stop_pid "${roscore_pid}"
  sleep 3
}

run_case "baseline_a_static_low" "innovation1_baseline_a_static_low.lua"
run_case "baseline_b_static_high" "innovation1_baseline_b_static_high.lua"
run_case "proposed_dynamic_anisotropic" "innovation1_proposed_dynamic_anisotropic.lua"

echo "[Innovation1 Experiment] All cases finished. Outputs: ${RUNS_DIR}"
