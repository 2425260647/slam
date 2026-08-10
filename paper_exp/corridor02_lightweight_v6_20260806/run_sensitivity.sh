#!/usr/bin/env bash
set -euo pipefail

variant="${1:?variant: coarse0|prior0|both0|oad|oad_v7c|odom1|odom5|oad_v7c_odom1|v7c_multihyp}"
mode="${2:-preflight}"
port="${3:?ROS master port}"
workspace="/home/slam/slam_ws"
exp_root="${workspace}/paper_exp/corridor02_lightweight_v6_20260806"
input_bag="${exp_root}/common_inputs.bag"
run_dir="${exp_root}/${mode}/sensitivity_${variant}"
run_dir="${run_dir}${RUN_SUFFIX:-}"
case "${variant}" in
  oad)
    args=()
    launch_file="${exp_root}/offline_lightweight_oad_corridor02.launch" ;;
  oad_v7c)
    args=()
    launch_file="${exp_root}/offline_lightweight_oad_v7c_corridor02.launch" ;;
  odom1)
    args=()
    launch_file="${exp_root}/offline_lightweight_odom1_corridor02.launch" ;;
  odom5)
    args=()
    launch_file="${exp_root}/offline_lightweight_odom5_corridor02.launch" ;;
  oad_v7c_odom1)
    args=()
    launch_file="${exp_root}/offline_lightweight_oad_v7c_odom1_corridor02.launch" ;;
  v7c_multihyp)
    args=()
    launch_file="${exp_root}/offline_lightweight_v7c_multihyp_corridor02.launch" ;;
  coarse0)
    args=(correlative_translation_cost_weight:=0.0 correlative_rotation_cost_weight:=0.0 \
          prior_translation_weight:=0.5 prior_rotation_weight:=3.0) ;;
  prior0)
    args=(correlative_translation_cost_weight:=0.01 correlative_rotation_cost_weight:=0.005 \
          prior_translation_weight:=0.0 prior_rotation_weight:=0.0) ;;
  both0)
    args=(correlative_translation_cost_weight:=0.0 correlative_rotation_cost_weight:=0.0 \
          prior_translation_weight:=0.0 prior_rotation_weight:=0.0) ;;
  *) echo "unknown variant: ${variant}" >&2; exit 2 ;;
esac
if [[ "${variant}" != "oad" && "${variant}" != "oad_v7c" &&
      "${variant}" != "odom1" && "${variant}" != "odom5" &&
      "${variant}" != "oad_v7c_odom1" &&
      "${variant}" != "v7c_multihyp" ]]; then
  launch_file="${exp_root}/offline_lightweight_sensitivity.launch"
fi
if [[ "${mode}" == "preflight" ]]; then duration_args=(--duration=30); \
elif [[ "${mode}" == "full" ]]; then duration_args=(); \
else echo "mode must be preflight or full" >&2; exit 2; fi
if [[ ! -s "${input_bag}" ]]; then echo "missing input bag" >&2; exit 3; fi
if [[ -e "${run_dir}/SUCCESS" ]]; then echo "run already completed" >&2; exit 4; fi
mkdir -p "${run_dir}"
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_LOG_DIR="${workspace}/.ros_runtime/corridor02_${mode}_sensitivity_${variant}"
export ROS_HOME="${ROS_LOG_DIR}/home"
mkdir -p "${ROS_LOG_DIR}" "${ROS_HOME}"
source /opt/ros/noetic/setup.bash
source "${workspace}/install_isolated/setup.bash"
roscore -p "${port}" >"${run_dir}/roscore.log" 2>&1 & master_pid=$!
launch_pid=""; record_pid=""
cleanup() {
  set +e
  [[ -n "${record_pid}" ]] && kill -INT "${record_pid}" 2>/dev/null
  [[ -n "${launch_pid}" ]] && kill "${launch_pid}" 2>/dev/null
  kill "${master_pid}" 2>/dev/null
  wait "${record_pid}" 2>/dev/null; wait "${launch_pid}" 2>/dev/null; wait "${master_pid}" 2>/dev/null
}
trap cleanup EXIT INT TERM
for attempt in $(seq 1 80); do rosparam list >/dev/null 2>&1 && break; sleep 0.25; done
rosparam list >/dev/null
roslaunch "${launch_file}" "${args[@]}" \
  >"${run_dir}/roslaunch.log" 2>&1 & launch_pid=$!
sleep 6
rosnode list >"${run_dir}/nodes.txt"
rosparam dump "${run_dir}/params.yaml"
rosbag record --lz4 -O "${run_dir}/outputs.bag" /tracked_pose /slam_diagnostics \
  >"${run_dir}/record.log" 2>&1 & record_pid=$!
for attempt in $(seq 1 80); do rosnode list 2>/dev/null | grep -q '^/record' && break; sleep 0.25; done
rosnode list | grep -q '^/record'; sleep 1
{
  printf 'variant=%s\nmode=%s\ninput_bag=%s\n' "${variant}" "${mode}" "${input_bag}"
  /usr/bin/time -f 'play_wall_duration_sec=%e' rosbag play --clock --quiet --rate=1.0 \
    "${duration_args[@]}" "${input_bag}" --topics /scan /odom
} >"${run_dir}/play.stdout" 2>"${run_dir}/wallclock.txt"
sleep 5
timeout 30s rosrun map_server map_saver -f "${run_dir}/map" \
  >"${run_dir}/map_saver.stdout" 2>"${run_dir}/map_saver.stderr"
kill -INT "${record_pid}"; wait "${record_pid}"; record_pid=""
python3 "${workspace}/paper_exp/corridor_repeat03_threeway_20260805/summarize_run.py" \
  --algorithm "sensitivity_${variant}" --mode "${mode}" --input-bag "${input_bag}" \
  --run-dir "${run_dir}" >"${run_dir}/summary.stdout"
touch "${run_dir}/SUCCESS"
echo "Completed ${mode}/sensitivity_${variant}"
