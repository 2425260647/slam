#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/slam_ws"
EXP_ROOT="${WORKSPACE}/paper_exp/formal_corridor_20260724/repeat_02_03_clean_baseline"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"

export ROS_HOME="${WORKSPACE}/.ros"
source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"
mkdir -p "${EXP_ROOT}"

wait_for_ros() {
  for _ in $(seq 1 80); do
    if rosparam list >/dev/null 2>&1; then
      return 0
    fi
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
  local roscore_pid=""
  local launch_pid=""
  local tf_pid=""

  mkdir -p "${run_dir}"
  echo "[Repeat Baseline] Starting ${case_id}"

  roscore >"${run_dir}/roscore.log" 2>&1 &
  roscore_pid=$!
  wait_for_ros
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
    >"${run_dir}/roslaunch.log" 2>&1 &
  launch_pid=$!

  sleep 6
  rosrun tf2_ros static_transform_publisher \
    0 0 0.33521 -1.5708 0 0 base_link laser_link \
    >"${run_dir}/static_tf.log" 2>&1 &
  tf_pid=$!
  sleep 3

  rosbag play --clock --quiet --rate=2.0 \
    "${bag}" --topics /velodyne_points /odom \
    >"${run_dir}/rosbag_play.log" 2>&1

  sleep 5
  rosservice call /finish_trajectory 0 \
    >"${run_dir}/finish_trajectory.log" 2>&1
  sleep 10
  rosservice call /write_state \
    "{filename: '${run_dir}/state.pbstream', include_unfinished_submaps: true}" \
    >"${run_dir}/write_state.log" 2>&1

  python3 "${WORKSPACE}/paper_exp/innovation1_directional_adaptive_fusion/save_occupancy_grid.py" \
    --topic /map \
    --output-prefix "${run_dir}/map" \
    --timeout 30 \
    >"${run_dir}/map_saver.log" 2>&1

  stop_pid "${tf_pid}"
  stop_pid "${launch_pid}"
  stop_pid "${roscore_pid}"
  sleep 3
  echo "[Repeat Baseline] Finished ${case_id}"
}

run_case "repeat_02" "${WORKSPACE}/bags/corridor_repeat_02.bag"
run_case "repeat_03" "${WORKSPACE}/bags/corridor_repeat_03.bag"

echo "[Repeat Baseline] Outputs: ${EXP_ROOT}"
