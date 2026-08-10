#!/usr/bin/env bash
set -euo pipefail

workspace="/home/slam/slam_ws"
exp_root="${workspace}/paper_exp/corridor02_lightweight_v6_20260806"
source_bag="${workspace}/bags/Corridor02/Corridor02.bag"
mode="${1:-full}"
port="${2:-11641}"

case "${mode}" in
  preflight)
    output_dir="${exp_root}/preflight"
    output_bag="${output_dir}/common_inputs_deskewed_30s.bag"
    play_duration_args=(--duration=30)
    minimum_scan_count=250
    ;;
  full)
    output_dir="${exp_root}"
    output_bag="${output_dir}/common_inputs_deskewed.bag"
    play_duration_args=()
    minimum_scan_count=2900
    ;;
  *)
    echo "Usage: $0 [preflight|full] [ros_master_port]" >&2
    exit 2
    ;;
esac

run_name="deskew_projection_${mode}"
if [[ -e "${output_bag}" || -e "${output_bag}.active" ]]; then
  echo "Refusing to overwrite existing deskewed input: ${output_bag}" >&2
  exit 3
fi
if [[ ! -r "${source_bag}" ]]; then
  echo "Source bag is missing or unreadable: ${source_bag}" >&2
  exit 4
fi

export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_LOG_DIR="${workspace}/.ros_runtime/${run_name}"
export ROS_HOME="${ROS_LOG_DIR}/home"
mkdir -p "${output_dir}" "${ROS_LOG_DIR}" "${ROS_HOME}"

source /opt/ros/noetic/setup.bash
source "${workspace}/install_isolated/setup.bash"

roscore -p "${port}" >"${output_dir}/${run_name}_roscore.log" 2>&1 &
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

roslaunch "${exp_root}/project_deskewed_scan.launch" \
  >"${output_dir}/${run_name}_roslaunch.log" 2>&1 &
launch_pid=$!
for attempt in $(seq 1 80); do
  if rosnode list 2>/dev/null | grep -qx \
      '/m3dgr_mid360_deskewed_scan'; then
    break
  fi
  sleep 0.25
done
rosnode list | grep -qx '/m3dgr_mid360_deskewed_scan'

rosbag record --lz4 -O "${output_bag}" /scan /odom \
  >"${output_dir}/${run_name}_record.log" 2>&1 &
record_pid=$!
for attempt in $(seq 1 80); do
  if rosnode list 2>/dev/null | grep -q '^/record'; then break; fi
  sleep 0.25
done
rosnode list | grep -q '^/record'
sleep 1

/usr/bin/time -f 'projection_play_wall_sec=%e' \
  -o "${output_dir}/${run_name}_wallclock.txt" \
  rosbag play --clock --quiet --rate=1.0 "${play_duration_args[@]}" \
    "${source_bag}" --topics /livox/mid360/lidar /odom

sleep 3
kill -INT "${record_pid}"
wait "${record_pid}"
record_pid=""

info_file="${output_bag%.bag}_info.txt"
count_file="${output_bag%.bag}_counts.txt"
hash_file="${output_bag%.bag}_hashes.sha256"
audit_file="${output_bag%.bag}_audit.json"
rosbag info "${output_bag}" >"${info_file}"
sha256sum "${source_bag}" "${output_bag}" >"${hash_file}"
scan_count=$(rosbag info --yaml "${output_bag}" | awk \
  '$1 == "-" && $2 == "topic:" && $3 == "/scan" {found=1; next} \
   found && $1 == "messages:" {print $2; exit}')
odom_count=$(rosbag info --yaml "${output_bag}" | awk \
  '$1 == "-" && $2 == "topic:" && $3 == "/odom" {found=1; next} \
   found && $1 == "messages:" {print $2; exit}')
if [[ "${scan_count:-0}" -lt "${minimum_scan_count}" ]]; then
  echo "Deskewed input incomplete: scan=${scan_count:-0}, odom=${odom_count:-0}" >&2
  exit 5
fi
printf 'mode=%s\nscan_count=%s\nodom_count=%s\n' \
  "${mode}" "${scan_count}" "${odom_count}" >"${count_file}"
python3 "${exp_root}/audit_deskewed_scan.py" \
  "${output_bag}" --output "${audit_file}"
echo "Generated ${mode} deskewed input: scan=${scan_count}, odom=${odom_count}"
