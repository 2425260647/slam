#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/slam_ws"
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${WORKSPACE}/bags/Wheel-float02/Wheel-float02.bag"
MAP_SAVER="${WORKSPACE}/paper_exp/innovation1_directional_adaptive_fusion/save_occupancy_grid.py"
EVALUATOR="${SCRIPT_ROOT}/evaluate_m3dgr_trajectory.py"
RUN_DIR="${RUN_DIR_OVERRIDE:-${SCRIPT_ROOT}/hector_wheel_float02}"
PLAYBACK_RATE="${PLAYBACK_RATE:-1.0}"
PLAY_DURATION="${PLAY_DURATION:-}"
FORCE="${FORCE:-0}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"

wait_for_ros() {
  for _ in $(seq 1 120); do
    if rosparam list >/dev/null 2>&1; then return 0; fi
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

if [[ -f "${RUN_DIR}/SUCCESS" && "${FORCE}" != "1" ]]; then
  echo "[Hector Wheel-float02] Reusing completed run: ${RUN_DIR}"
  exit 0
fi
if [[ -d "${RUN_DIR}" && "${FORCE}" != "1" ]]; then
  echo "Incomplete run exists; inspect or set FORCE=1: ${RUN_DIR}" >&2
  exit 1
fi

mkdir -p "${RUN_DIR}"
export ROS_HOME="${RUN_DIR}/ros_home"
mkdir -p "${ROS_HOME}"

roscore >"${RUN_DIR}/roscore.log" 2>&1 &
roscore_pid=$!
cleanup() {
  stop_pid "${capture_pose_pid:-}"
  stop_pid "${capture_mocap_pid:-}"
  stop_pid "${hector_pid:-}"
  stop_pid "${adapter_pid:-}"
  stop_pid "${roscore_pid:-}"
}
trap cleanup EXIT
wait_for_ros
rosparam set /use_sim_time true

{
  printf 'experiment=hector_wheel_float02_reference\n'
  printf 'algorithm=hector_mapping\n'
  printf 'bag=%s\n' "${BAG}"
  printf 'bag_sha256=%s\n' "$(sha256sum "${BAG}" | awk '{print $1}')"
  printf 'input_topics=/livox/mid360/lidar,/vrpn_client_node/UGV/pose\n'
  printf 'odom_used=false\nimu_used=false\navia_used=false\n'
  printf 'official_static_tf=base_footprint->livox_frame\n'
  printf 'projection_height_m=0.20,0.60\n'
  printf 'start_wall_utc=%s\n' "$(date -u +%FT%TZ)"
  printf 'git_head=%s\n' "$(git -C "${WORKSPACE}" rev-parse HEAD)"
} >"${RUN_DIR}/metadata.txt"

roslaunch my_navigation m3dgr_mid360_mapping.launch \
  rviz:=false start_cartographer:=false start_local_grid:=false \
  >"${RUN_DIR}/adapter_roslaunch.log" 2>&1 &
adapter_pid=$!
sleep 3

roslaunch hector_mapping mapping_default.launch \
  scan_topic:=/scan \
  base_frame:=base_footprint \
  odom_frame:=nav \
  pub_map_odom_transform:=false \
  >"${RUN_DIR}/hector_roslaunch.log" 2>&1 &
hector_pid=$!
sleep 4

rostopic echo -p /slam_out_pose >"${RUN_DIR}/tracked_pose.csv" \
  2>"${RUN_DIR}/tracked_pose.err" &
capture_pose_pid=$!
rostopic echo -p /vrpn_client_node/UGV/pose >"${RUN_DIR}/mocap_pose.csv" \
  2>"${RUN_DIR}/mocap_pose.err" &
capture_mocap_pid=$!

play_args=(--clock --quiet --rate="${PLAYBACK_RATE}" "${BAG}"
  --topics /livox/mid360/lidar /vrpn_client_node/UGV/pose)
if [[ -n "${PLAY_DURATION}" ]]; then
  play_args=(-u "${PLAY_DURATION}" "${play_args[@]}")
fi
rosbag play "${play_args[@]}" >"${RUN_DIR}/rosbag_play.log" 2>&1
sleep 5

if [[ -z "${PLAY_DURATION}" ]]; then
  python3 "${MAP_SAVER}" --topic /map \
    --output-prefix "${RUN_DIR}/map" --timeout 45 \
    >"${RUN_DIR}/map_saver.log" 2>&1
else
  python3 "${MAP_SAVER}" --topic /map \
    --output-prefix "${RUN_DIR}/map" --timeout 15 \
    >"${RUN_DIR}/map_saver.log" 2>&1 || true
fi

stop_pid "${capture_pose_pid}"
stop_pid "${capture_mocap_pid}"
rosnode list >"${RUN_DIR}/nodes.txt" 2>&1 || true
rostopic list >"${RUN_DIR}/topics.txt" 2>&1 || true
rg -n "FATAL|Check failed|Segmentation fault" \
  "${RUN_DIR}/adapter_roslaunch.log" "${RUN_DIR}/hector_roslaunch.log" \
  >"${RUN_DIR}/fatal_scan.txt" || true

test -s "${RUN_DIR}/tracked_pose.csv"
test -s "${RUN_DIR}/mocap_pose.csv"
if [[ -z "${PLAY_DURATION}" ]]; then
  test -s "${RUN_DIR}/map.pgm"
  test -s "${RUN_DIR}/map.yaml"
  python3 "${EVALUATOR}" \
    --estimate "${RUN_DIR}/tracked_pose.csv" \
    --reference "${RUN_DIR}/mocap_pose.csv" \
    --output "${RUN_DIR}/online_metrics.json" \
    >"${RUN_DIR}/online_metrics.stdout.json"
fi
printf 'end_wall_utc=%s\n' "$(date -u +%FT%TZ)" >>"${RUN_DIR}/metadata.txt"
touch "${RUN_DIR}/SUCCESS"
echo "[Hector Wheel-float02] Finished ${RUN_DIR}"
