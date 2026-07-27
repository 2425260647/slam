#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/slam_ws"
EXP_ROOT="${WORKSPACE}/paper_exp/formal_corridor_20260724/partial_precheck"
BAG="${WORKSPACE}/bags/corridor_repeat_01.bag"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"

cd "${WORKSPACE}"
export ROS_HOME="${WORKSPACE}/.ros"
source /opt/ros/noetic/setup.bash
source install_isolated/setup.bash

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
  local config="$2"
  local quality_gate="$3"
  local run_dir="${EXP_ROOT}/${case_id}"
  local roscore_pid=""
  local launch_pid=""
  local tf_pid=""
  local tracked_pose_pid=""
  local degeneracy_pid=""
  local slip_metric_pid=""
  local slip_state_pid=""
  local weight_pid=""
  local reliability_pid=""

  mkdir -p "${run_dir}"
  echo "[Partial Precheck] Starting ${case_id}"

  roscore >"${run_dir}/roscore.log" 2>&1 &
  roscore_pid=$!
  wait_for_ros
  rosparam set /use_sim_time true

  roslaunch my_navigation real_scout_mapping_local_grid.launch \
    cartographer_config_dir:="${CONFIG_DIR}" \
    cartographer_config:="${config}" \
    cloud_topic:=/velodyne_points \
    odom_topic:=/odom \
    target_frame:=laser_link \
    min_height:=-0.515 \
    max_height:=1.435 \
    scan_quality_gate_enabled:="${quality_gate}" \
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

  rostopic echo -p /tracked_pose >"${run_dir}/tracked_pose.csv" 2>"${run_dir}/tracked_pose.err" &
  tracked_pose_pid=$!
  rostopic echo -p /degeneracy_metric >"${run_dir}/degeneracy_metric.csv" 2>"${run_dir}/degeneracy_metric.err" &
  degeneracy_pid=$!
  rostopic echo -p /slip_metric >"${run_dir}/consistency_metric.csv" 2>"${run_dir}/consistency_metric.err" &
  slip_metric_pid=$!
  rostopic echo -p /slip_state >"${run_dir}/consistency_state.csv" 2>"${run_dir}/consistency_state.err" &
  slip_state_pid=$!
  rostopic echo -p /odom_weight_scale >"${run_dir}/odom_weight_scale.csv" 2>"${run_dir}/odom_weight_scale.err" &
  weight_pid=$!
  rostopic echo -p /slip_lidar_reliability >"${run_dir}/lidar_reliability.csv" 2>"${run_dir}/lidar_reliability.err" &
  reliability_pid=$!

  rosbag play --clock --quiet "${BAG}" --topics /velodyne_points /odom \
    >"${run_dir}/rosbag_play.log" 2>&1

  sleep 5
  rosservice call /finish_trajectory 0 >"${run_dir}/finish_trajectory.log" 2>&1 || true
  sleep 12
  rosservice call /write_state \
    "{filename: '${run_dir}/state.pbstream', include_unfinished_submaps: true}" \
    >"${run_dir}/write_state.log" 2>&1 || true

  python3 "${WORKSPACE}/paper_exp/innovation1_directional_adaptive_fusion/save_occupancy_grid.py" \
    --topic /map \
    --output-prefix "${run_dir}/map" \
    --timeout 30 \
    >"${run_dir}/map_saver.log" 2>&1
  timeout 8s rostopic echo -n 1 /map/info >"${run_dir}/map_info.txt" 2>&1 || true

  stop_pid "${tracked_pose_pid}"
  stop_pid "${degeneracy_pid}"
  stop_pid "${slip_metric_pid}"
  stop_pid "${slip_state_pid}"
  stop_pid "${weight_pid}"
  stop_pid "${reliability_pid}"
  stop_pid "${tf_pid}"
  stop_pid "${launch_pid}"
  stop_pid "${roscore_pid}"
  sleep 3
  echo "[Partial Precheck] Finished ${case_id}"
}

run_case "clean_baseline" "clean_baseline_no_innovation.lua" "false"
run_case "full_proposed" "cartographer_scout_2d.lua" "true"

echo "[Partial Precheck] Outputs: ${EXP_ROOT}"
