#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/slam_ws"
EXP_ROOT="${EXP_ROOT:-${WORKSPACE}/paper_exp/corridor_repeat03_pointcloud_20260810}"
BAG="${BAG:-${WORKSPACE}/bags/corridor_repeat_03.bag}"
PORT="${PORT:-11556}"
RATE="${RATE:-5.0}"
CONFIG_BASENAME="${CONFIG_BASENAME:-repeat03_pointcloud.lua}"
REPLAY_LAUNCH="${REPLAY_LAUNCH:-${WORKSPACE}/paper_exp/deployment_integration_20260810/repeat03_pointcloud_replay.launch}"
read -r -a PLAY_TOPICS_ARRAY <<< "${PLAY_TOPICS:-/velodyne_points /odom /tf /tf_static}"
DURATION_ARGS=()
if [[ -n "${DURATION:-}" ]]; then
  DURATION_ARGS=(--duration="${DURATION}")
fi

mkdir -p "${EXP_ROOT}"
export ROS_MASTER_URI="http://127.0.0.1:${PORT}"
export ROS_HOME="${WORKSPACE}/.ros_runtime/corridor_repeat03_pointcloud/home"
export ROS_LOG_DIR="${WORKSPACE}/.ros_runtime/corridor_repeat03_pointcloud/log"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"

master_pid=""
launch_pid=""
record_pid=""
cleanup() {
  set +e
  [[ -n "${record_pid}" ]] && kill -INT "${record_pid}" 2>/dev/null
  [[ -n "${launch_pid}" ]] && kill "${launch_pid}" 2>/dev/null
  [[ -n "${master_pid}" ]] && kill "${master_pid}" 2>/dev/null
  [[ -n "${record_pid}" ]] && wait "${record_pid}" 2>/dev/null
  [[ -n "${launch_pid}" ]] && wait "${launch_pid}" 2>/dev/null
  [[ -n "${master_pid}" ]] && wait "${master_pid}" 2>/dev/null
}
trap cleanup EXIT INT TERM

roscore -p "${PORT}" >"${EXP_ROOT}/roscore.log" 2>&1 &
master_pid=$!
for _ in $(seq 1 120); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.25
done
rosparam list >/dev/null

roslaunch "${REPLAY_LAUNCH}" \
  configuration_basename:="${CONFIG_BASENAME}" \
  >"${EXP_ROOT}/roslaunch.log" 2>&1 &
launch_pid=$!
sleep 8
rosnode list >"${EXP_ROOT}/nodes.txt"
rosparam dump "${EXP_ROOT}/rosparam.yaml"

rosbag record --lz4 -O "${EXP_ROOT}/outputs.bag" \
  /map /tf /tf_static /local_occupancy_grid /local_occupancy_grid_map \
  >"${EXP_ROOT}/record.log" 2>&1 &
record_pid=$!
sleep 2

/usr/bin/time -f 'play_wall_duration_sec=%e' -o "${EXP_ROOT}/wallclock.txt" \
  rosbag play --clock --quiet --rate="${RATE}" "${DURATION_ARGS[@]}" "${BAG}" \
  --topics "${PLAY_TOPICS_ARRAY[@]}" \
  >"${EXP_ROOT}/rosbag_play.log" 2>&1

sleep 8
rosservice call /finish_trajectory 0 >"${EXP_ROOT}/finish_trajectory.log" 2>&1 || true
sleep 12
timeout 45s rosrun map_server map_saver -f "${EXP_ROOT}/map" \
  >"${EXP_ROOT}/map_saver.log" 2>&1 || true

if [[ -n "${record_pid}" ]] && kill -0 "${record_pid}" 2>/dev/null; then
  kill -INT "${record_pid}" 2>/dev/null || true
  wait "${record_pid}" 2>/dev/null || true
  record_pid=""
fi
rosbag info "${EXP_ROOT}/outputs.bag" >"${EXP_ROOT}/outputs_info.txt" 2>&1 || true
if [[ -s "${EXP_ROOT}/map.pgm" && -s "${EXP_ROOT}/map.yaml" ]]; then
  touch "${EXP_ROOT}/SUCCESS"
fi
