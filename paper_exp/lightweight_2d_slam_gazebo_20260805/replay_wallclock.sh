#!/usr/bin/env bash
set -euo pipefail

algorithm="${1:?algorithm: lightweight|cartographer|gmapping}"
port="${2:?ROS master port}"
bag="${3:?input bag}"
out_dir="${4:?output directory}"

mkdir -p "${out_dir}"
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_LOG_DIR="${PWD}/.ros_runtime/wallclock_${algorithm}"
export ROS_HOME="${ROS_LOG_DIR}/home"
mkdir -p "${ROS_LOG_DIR}" "${ROS_HOME}"

source /opt/ros/noetic/setup.bash
source "${PWD}/install_isolated/setup.bash"
if [[ "${algorithm}" == "gmapping" ]]; then
  source "${PWD}/devel_isolated/gmapping/setup.bash"
fi

roscore -p "${port}" >"${out_dir}/roscore.log" 2>&1 &
master_pid=$!
launch_pid=""
bag_pid=""
cleanup() {
  set +e
  if [[ -n "${bag_pid}" ]]; then kill "${bag_pid}" 2>/dev/null; fi
  if [[ -n "${launch_pid}" ]]; then kill "${launch_pid}" 2>/dev/null; fi
  kill "${master_pid}" 2>/dev/null
  wait "${bag_pid}" 2>/dev/null
  wait "${launch_pid}" 2>/dev/null
  wait "${master_pid}" 2>/dev/null
}
trap cleanup EXIT INT TERM

for attempt in $(seq 1 60); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.2
done
rosparam list >/dev/null

case "${algorithm}" in
  lightweight)
    roslaunch lightweight_2d_slam offline_lightweight_slam.launch \
      enable_loop_closure:=true >"${out_dir}/roslaunch.log" 2>&1 &
    ;;
  cartographer)
    roslaunch my_navigation cartographer_scout_2d_offline_clean.launch \
      >"${out_dir}/roslaunch.log" 2>&1 &
    ;;
  gmapping)
    roslaunch lightweight_2d_slam offline_gmapping_scout.launch \
      >"${out_dir}/roslaunch.log" 2>&1 &
    ;;
  *)
    echo "unknown algorithm: ${algorithm}" >&2
    exit 2
    ;;
esac
launch_pid=$!
sleep 8

{
  printf 'algorithm=%s\n' "${algorithm}"
  printf 'bag=%s\n' "${bag}"
  printf 'bag_duration_sec=193.21\n'
  /usr/bin/time -f 'play_wall_duration_sec=%e' \
    rosbag play --clock --quiet "${bag}" \
      --topics /scan /scout_mini_velocity_controller/odom \
      /gazebo/model_states /tf /tf_static
} 2>"${out_dir}/wallclock.txt"

sleep 3
