#!/usr/bin/env bash
set -euo pipefail

mode="${1:-preflight}"
port="${2:-11624}"
variant="${3:-lightweight_v7}"
workspace="/home/slam/slam_ws"
source_root="${workspace}/paper_exp/corridor_repeat03_threeway_20260805"
exp_root="${workspace}/paper_exp/corridor_repeat03_submap_topk_20260805"
input_bag="${source_root}/common_inputs.bag"
run_dir="${exp_root}/${mode}/${variant}"

case "${variant}" in
  lightweight_v7)
    launch_file="offline_lightweight_v7.launch"
    ;;
  lightweight_v7b)
    launch_file="offline_lightweight_v7b.launch"
    ;;
  lightweight_v7c)
    launch_file="offline_lightweight_v7c.launch"
    ;;
  *)
    echo "variant must be lightweight_v7, lightweight_v7b or lightweight_v7c" >&2
    exit 2
    ;;
esac

if [[ "${mode}" == "preflight" ]]; then
  duration_args=(--duration=30)
elif [[ "${mode}" == "full" ]]; then
  duration_args=()
else
  echo "mode must be preflight or full" >&2
  exit 2
fi
if [[ ! -s "${input_bag}" ]]; then
  echo "missing input bag: ${input_bag}" >&2
  exit 3
fi
if [[ -e "${run_dir}/SUCCESS" ]]; then
  echo "refusing to overwrite completed run: ${run_dir}" >&2
  exit 4
fi

mkdir -p "${run_dir}"
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_LOG_DIR="${workspace}/.ros_runtime/corridor_repeat03_${variant}_${mode}"
export ROS_HOME="${ROS_LOG_DIR}/home"
mkdir -p "${ROS_LOG_DIR}" "${ROS_HOME}"

source /opt/ros/noetic/setup.bash
source "${workspace}/install_isolated/setup.bash"

roscore -p "${port}" >"${run_dir}/roscore.log" 2>&1 &
master_pid=$!
launch_pid=""
tf_pid=""
record_pid=""
cleanup() {
  set +e
  if [[ -n "${record_pid}" ]]; then kill -INT "${record_pid}" 2>/dev/null; fi
  if [[ -n "${tf_pid}" ]]; then kill "${tf_pid}" 2>/dev/null; fi
  if [[ -n "${launch_pid}" ]]; then kill "${launch_pid}" 2>/dev/null; fi
  kill "${master_pid}" 2>/dev/null
  wait "${record_pid}" 2>/dev/null
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
roslaunch lightweight_2d_slam "${launch_file}" \
  >"${run_dir}/roslaunch.log" 2>&1 &
launch_pid=$!
sleep 6
rosnode list >"${run_dir}/nodes.txt"
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
  printf 'mode=%s\n' "${mode}"
  printf 'input_bag=%s\n' "${input_bag}"
  /usr/bin/time -f 'play_wall_duration_sec=%e' \
    rosbag play --clock --quiet --rate=1.0 "${duration_args[@]}" \
      "${input_bag}" /odom:=/scout_mini_velocity_controller/odom \
      --topics /scan /odom
} >"${run_dir}/play.stdout" 2>"${run_dir}/wallclock.txt"

sleep 5
timeout 30s rosrun map_server map_saver -f "${run_dir}/map" \
  >"${run_dir}/map_saver.stdout" 2>"${run_dir}/map_saver.stderr"
kill -INT "${record_pid}"
wait "${record_pid}"
record_pid=""

python3 "${source_root}/summarize_run.py" \
  --algorithm "${variant}" --mode "${mode}" \
  --input-bag "${input_bag}" --run-dir "${run_dir}" \
  >"${run_dir}/summary.stdout"
if [[ ! -s "${run_dir}/map.pgm" || ! -s "${run_dir}/outputs.bag" ]]; then
  echo "missing required outputs" >&2
  exit 5
fi
if grep -Eqi 'FATAL|segmentation fault|core dumped|Check failed' \
    "${run_dir}/roslaunch.log"; then
  echo "fatal pattern found in roslaunch log" >&2
  exit 6
fi
touch "${run_dir}/SUCCESS"
echo "Completed ${mode}/${variant}"
