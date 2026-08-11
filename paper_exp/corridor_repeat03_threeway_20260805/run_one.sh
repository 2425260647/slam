#!/usr/bin/env bash
set -euo pipefail

algorithm="${1:?algorithm: lightweight|cartographer|gmapping}"
mode="${2:?mode: preflight|full}"
port="${3:?ROS master port}"

workspace="/home/slam/slam_ws"
exp_root="${workspace}/paper_exp/corridor_repeat03_threeway_20260805"
input_bag="${exp_root}/common_inputs.bag"
run_dir="${exp_root}/${mode}/${algorithm}"
duration_args=()
if [[ "${mode}" == "preflight" ]]; then
  duration_args=(--duration=30)
elif [[ "${mode}" != "full" ]]; then
  echo "Unknown mode: ${mode}" >&2
  exit 2
fi

if [[ ! -s "${input_bag}" ]]; then
  echo "Missing common input: ${input_bag}" >&2
  exit 3
fi
if [[ -e "${run_dir}/SUCCESS" ]]; then
  echo "Refusing to overwrite completed run: ${run_dir}" >&2
  exit 4
fi

mkdir -p "${run_dir}"
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_LOG_DIR="${workspace}/.ros_runtime/corridor_repeat03_${mode}_${algorithm}"
export ROS_HOME="${ROS_LOG_DIR}/home"
mkdir -p "${ROS_LOG_DIR}" "${ROS_HOME}"

source /opt/ros/noetic/setup.bash
source "${workspace}/install_isolated/setup.bash"
if [[ "${algorithm}" == "gmapping" ]]; then
  source "${workspace}/devel_isolated/gmapping/setup.bash"
fi

roscore -p "${port}" >"${run_dir}/roscore.log" 2>&1 &
master_pid=$!
launch_pid=""
tf_pid=""
pose_pid=""
record_pid=""
cleanup() {
  set +e
  if [[ -n "${record_pid}" ]]; then kill -INT "${record_pid}" 2>/dev/null; fi
  if [[ -n "${pose_pid}" ]]; then kill "${pose_pid}" 2>/dev/null; fi
  if [[ -n "${tf_pid}" ]]; then kill "${tf_pid}" 2>/dev/null; fi
  if [[ -n "${launch_pid}" ]]; then kill "${launch_pid}" 2>/dev/null; fi
  kill "${master_pid}" 2>/dev/null
  wait "${record_pid}" 2>/dev/null
  wait "${pose_pid}" 2>/dev/null
  wait "${tf_pid}" 2>/dev/null
  wait "${launch_pid}" 2>/dev/null
  wait "${master_pid}" 2>/dev/null
}
trap cleanup EXIT INT TERM

for attempt in $(seq 1 80); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.25
done
rosparam list >/dev/null

rosrun tf2_ros static_transform_publisher \
  0 0 0.33521 -1.5708 0 0 base_link laser_link \
  >"${run_dir}/static_tf.log" 2>&1 &
tf_pid=$!

case "${algorithm}" in
  lightweight)
    roslaunch lightweight_2d_slam offline_lightweight_slam.launch \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  cartographer)
    roslaunch my_navigation cartographer_scout_2d_offline_clean.launch \
      configuration_basename:=clean_baseline_no_innovation.lua \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  gmapping)
    roslaunch lightweight_2d_slam offline_gmapping_scout.launch \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  *)
    echo "Unknown algorithm: ${algorithm}" >&2
    exit 2
    ;;
esac
launch_pid=$!

if [[ "${algorithm}" == "cartographer" ]]; then
  rosrun lightweight_2d_slam tf_pose_publisher.py \
    _map_frame:=map _base_frame:=base_link \
    _output_topic:=/tracked_pose _publish_rate:=20.0 \
    >"${run_dir}/pose_publisher.log" 2>&1 &
  pose_pid=$!
fi

sleep 6
rosnode list >"${run_dir}/nodes.txt"
while read -r node; do
  rosnode info "${node}" || true
done <"${run_dir}/nodes.txt" >"${run_dir}/node_info.txt" 2>&1
rosparam dump "${run_dir}/params.yaml"

rosbag record --lz4 -O "${run_dir}/outputs.bag" \
  /tracked_pose /slam_diagnostics \
  >"${run_dir}/record.log" 2>&1 &
record_pid=$!
for attempt in $(seq 1 80); do
  if rosnode list 2>/dev/null | grep -q '^/record'; then break; fi
  sleep 0.25
done
rosnode list | grep -q '^/record'
sleep 1

{
  printf 'algorithm=%s\n' "${algorithm}"
  printf 'mode=%s\n' "${mode}"
  printf 'input_bag=%s\n' "${input_bag}"
  /usr/bin/time -f 'play_wall_duration_sec=%e' \
    rosbag play --clock --quiet --rate=1.0 "${duration_args[@]}" \
      "${input_bag}" /odom:=/scout_mini_velocity_controller/odom \
      --topics /scan /odom
} >"${run_dir}/play.stdout" 2>"${run_dir}/wallclock.txt"

if [[ "${algorithm}" == "cartographer" ]]; then
  rosservice call /finish_trajectory 0 \
    >"${run_dir}/finish_trajectory.log" 2>&1 || true
fi
sleep 5

timeout 30s rosrun map_server map_saver -f "${run_dir}/map" \
  >"${run_dir}/map_saver.stdout" 2>"${run_dir}/map_saver.stderr"

kill -INT "${record_pid}"
wait "${record_pid}"
record_pid=""

if [[ ! -s "${run_dir}/map.pgm" || ! -s "${run_dir}/outputs.bag" ]]; then
  echo "Run did not produce required outputs" >&2
  exit 5
fi
if grep -Eqi 'FATAL|segmentation fault|core dumped|Check failed' \
    "${run_dir}/roslaunch.log"; then
  echo "Fatal pattern found in roslaunch log" >&2
  exit 6
fi

python3 "${exp_root}/summarize_run.py" \
  --algorithm "${algorithm}" --mode "${mode}" \
  --input-bag "${input_bag}" --run-dir "${run_dir}" \
  >"${run_dir}/summary.stdout"
touch "${run_dir}/SUCCESS"
echo "Completed ${mode}/${algorithm}"
