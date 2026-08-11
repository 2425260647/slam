#!/usr/bin/env bash
set -euo pipefail

workspace="/home/slam/slam_ws"
exp_root="${workspace}/paper_exp/corridor_repeat03_threeway_20260805"
source_bag="${workspace}/bags/corridor_repeat_03.bag"
output_bag="${exp_root}/common_inputs.bag"
port="${1:-11531}"

if [[ -e "${output_bag}" || -e "${output_bag}.active" ]]; then
  echo "Refusing to overwrite existing common input: ${output_bag}" >&2
  exit 2
fi

export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_LOG_DIR="${workspace}/.ros_runtime/corridor_repeat03_projection"
export ROS_HOME="${ROS_LOG_DIR}/home"
mkdir -p "${exp_root}" "${ROS_LOG_DIR}" "${ROS_HOME}"

source /opt/ros/noetic/setup.bash
source "${workspace}/install_isolated/setup.bash"

roscore -p "${port}" >"${exp_root}/projection_roscore.log" 2>&1 &
master_pid=$!
launch_pid=""
record_pid=""
cleanup() {
  set +e
  if [[ -n "${record_pid}" ]]; then kill -INT "${record_pid}" 2>/dev/null; fi
  if [[ -n "${launch_pid}" ]]; then kill "${launch_pid}" 2>/dev/null; fi
  kill "${master_pid}" 2>/dev/null
  wait "${record_pid}" 2>/dev/null
  wait "${launch_pid}" 2>/dev/null
  wait "${master_pid}" 2>/dev/null
}
trap cleanup EXIT INT TERM

for attempt in $(seq 1 80); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.25
done
rosparam list >/dev/null

roslaunch "${exp_root}/project_common_scan.launch" \
  >"${exp_root}/projection_roslaunch.log" 2>&1 &
launch_pid=$!

for attempt in $(seq 1 80); do
  if rosnode list 2>/dev/null | grep -qx '/corridor_repeat03_cloud_to_scan'; then
    break
  fi
  sleep 0.25
done
rosnode list | grep -qx '/corridor_repeat03_cloud_to_scan'

rosbag record --lz4 -O "${output_bag}" /scan /odom \
  >"${exp_root}/projection_record.log" 2>&1 &
record_pid=$!

for attempt in $(seq 1 80); do
  if rosnode list 2>/dev/null | grep -q '^/record'; then break; fi
  sleep 0.25
done
rosnode list | grep -q '^/record'
sleep 1

/usr/bin/time -f 'projection_play_wall_sec=%e' \
  -o "${exp_root}/projection_wallclock.txt" \
  rosbag play --clock --quiet --rate=2.0 "${source_bag}" \
    --topics /velodyne_points /odom

sleep 2
kill -INT "${record_pid}"
wait "${record_pid}"
record_pid=""

rosbag info "${output_bag}" >"${exp_root}/common_inputs_info.txt"
sha256sum "${source_bag}" "${output_bag}" \
  >"${exp_root}/input_hashes.sha256"

scan_count=$(rosbag info --yaml "${output_bag}" | awk \
  '$1 == "-" && $2 == "topic:" && $3 == "/scan" {found=1; next} \
   found && $1 == "messages:" {print $2; exit}')
odom_count=$(rosbag info --yaml "${output_bag}" | awk \
  '$1 == "-" && $2 == "topic:" && $3 == "/odom" {found=1; next} \
   found && $1 == "messages:" {print $2; exit}')
if [[ "${scan_count:-0}" -lt 5700 || "${odom_count:-0}" -lt 28900 ]]; then
  echo "Common input is incomplete: scan=${scan_count:-0}, odom=${odom_count:-0}" >&2
  exit 3
fi

printf 'scan_count=%s\nodom_count=%s\n' "${scan_count}" "${odom_count}" \
  >"${exp_root}/common_input_counts.txt"
echo "Generated ${output_bag}: scan=${scan_count}, odom=${odom_count}"
