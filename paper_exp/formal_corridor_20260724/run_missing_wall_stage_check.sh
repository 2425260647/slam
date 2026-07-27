#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/slam_ws"
OUTPUT_DIR="${WORKSPACE}/paper_exp/formal_corridor_20260724/missing_wall_stage_check/outbound_only"
BAG="${WORKSPACE}/bags/corridor_repeat_01.bag"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"

export ROS_HOME="${WORKSPACE}/.ros"
source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"
mkdir -p "${OUTPUT_DIR}"

roscore_pid=""
launch_pid=""
tf_pid=""

stop_pid() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  stop_pid "${tf_pid}"
  stop_pid "${launch_pid}"
  stop_pid "${roscore_pid}"
}
trap cleanup EXIT

roscore >"${OUTPUT_DIR}/roscore.log" 2>&1 &
roscore_pid=$!
for _ in $(seq 1 80); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done
rosparam set /use_sim_time true

roslaunch my_navigation real_scout_mapping_local_grid.launch \
  cartographer_config_dir:="${CONFIG_DIR}" \
  cartographer_config:=clean_baseline_no_innovation.lua \
  cloud_topic:=/velodyne_points \
  odom_topic:=/odom \
  target_frame:=laser_link \
  min_height:=-0.515 \
  max_height:=1.435 \
  scan_quality_gate_enabled:=false \
  rviz:=false \
  enable_monitor:=false \
  >"${OUTPUT_DIR}/roslaunch.log" 2>&1 &
launch_pid=$!

sleep 6
rosrun tf2_ros static_transform_publisher \
  0 0 0.33521 -1.5708 0 0 base_link laser_link \
  >"${OUTPUT_DIR}/static_tf.log" 2>&1 &
tf_pid=$!
sleep 3

rosbag play --clock --quiet --rate=2.0 --duration=295 \
  "${BAG}" --topics /velodyne_points /odom \
  >"${OUTPUT_DIR}/rosbag_play.log" 2>&1

sleep 5
rosservice call /finish_trajectory 0 \
  >"${OUTPUT_DIR}/finish_trajectory.log" 2>&1
sleep 10
rosservice call /write_state \
  "{filename: '${OUTPUT_DIR}/state.pbstream', include_unfinished_submaps: true}" \
  >"${OUTPUT_DIR}/write_state.log" 2>&1

python3 "${WORKSPACE}/paper_exp/innovation1_directional_adaptive_fusion/save_occupancy_grid.py" \
  --topic /map \
  --output-prefix "${OUTPUT_DIR}/map" \
  --timeout 30 \
  >"${OUTPUT_DIR}/map_saver.log" 2>&1

echo "[Missing Wall Check] Outbound-only result: ${OUTPUT_DIR}"
