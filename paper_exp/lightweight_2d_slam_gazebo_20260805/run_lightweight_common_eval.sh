#!/usr/bin/env bash
set -euo pipefail

port="${1:-11421}"
bag="${2:-paper_exp/lightweight_2d_slam_gazebo_20260805/large_common_loop/common_inputs.bag}"
out_dir="${3:-paper_exp/lightweight_2d_slam_gazebo_20260805/large_common_loop/optimization/current}"
algorithm="${4:-lightweight_optimized}"
use_likelihood_field="${5:-true}"
enable_fine_correlative="${6:-true}"
min_match_score="${7:-0.25}"
fine_linear_step="${8:-0.0125}"
fine_angular_window="${9:-0.0}"
fine_angular_step="${10:-0.0125}"
prior_rotation_weight="${11:-3.0}"
max_refinement_iterations="${12:-8}"
enable_loop_closure="${13:-true}"
active_keyframes="${14:-60}"
point_stride="${15:-3}"
active_grid_insert_free_space="${16:-true}"
fine_candidate_count="${17:-1}"
use_likelihood_refinement="${18:-true}"
correlative_rotation_cost_weight="${19:-0.005}"
odom_constraint_translation_weight="${20:-0.0}"
odom_constraint_rotation_weight="${21:-0.0}"
likelihood_refinement_weight="${22:-0.5}"
loop_min_match_score="${23:-0.75}"

mkdir -p "${out_dir}"
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_LOG_DIR="${PWD}/.ros_runtime/${algorithm}"
export ROS_HOME="${ROS_LOG_DIR}/home"
mkdir -p "${ROS_LOG_DIR}" "${ROS_HOME}"

source /opt/ros/noetic/setup.bash
source "${PWD}/install_isolated/setup.bash"

roscore -p "${port}" >"${out_dir}/roscore.log" 2>&1 &
master_pid=$!
launch_pid=""
evaluator_pid=""
bag_pid=""
cleanup() {
  set +e
  if [[ -n "${bag_pid}" ]]; then kill "${bag_pid}" 2>/dev/null; fi
  if [[ -n "${evaluator_pid}" ]]; then kill "${evaluator_pid}" 2>/dev/null; fi
  if [[ -n "${launch_pid}" ]]; then kill "${launch_pid}" 2>/dev/null; fi
  kill "${master_pid}" 2>/dev/null
  wait "${bag_pid}" 2>/dev/null
  wait "${evaluator_pid}" 2>/dev/null
  wait "${launch_pid}" 2>/dev/null
  wait "${master_pid}" 2>/dev/null
}
trap cleanup EXIT INT TERM

for attempt in $(seq 1 60); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.2
done
rosparam list >/dev/null

roslaunch lightweight_2d_slam offline_lightweight_slam.launch \
  enable_loop_closure:="${enable_loop_closure}" \
  use_likelihood_field:="${use_likelihood_field}" \
  enable_fine_correlative:="${enable_fine_correlative}" \
  fine_candidate_count:="${fine_candidate_count}" \
  use_likelihood_refinement:="${use_likelihood_refinement}" \
  likelihood_refinement_weight:="${likelihood_refinement_weight}" \
  correlative_rotation_cost_weight:="${correlative_rotation_cost_weight}" \
  odom_constraint_translation_weight:="${odom_constraint_translation_weight}" \
  odom_constraint_rotation_weight:="${odom_constraint_rotation_weight}" \
  loop_min_match_score:="${loop_min_match_score}" \
  min_match_score:="${min_match_score}" \
  fine_linear_step:="${fine_linear_step}" \
  fine_angular_window:="${fine_angular_window}" \
  fine_angular_step:="${fine_angular_step}" \
  prior_rotation_weight:="${prior_rotation_weight}" \
  max_refinement_iterations:="${max_refinement_iterations}" \
  active_keyframes:="${active_keyframes}" \
  point_stride:="${point_stride}" \
  active_grid_insert_free_space:="${active_grid_insert_free_space}" \
  >"${out_dir}/roslaunch.log" 2>&1 &
launch_pid=$!
sleep 6

rosrun lightweight_2d_slam evaluate_replay_trajectory.py \
  --algorithm "${algorithm}" \
  --pose-topic /tracked_pose \
  --diagnostics-topic /slam_diagnostics \
  --output-csv "${PWD}/${out_dir}/trajectory.csv" \
  --output-json "${PWD}/${out_dir}/summary.json" \
  >"${out_dir}/evaluator.stdout" 2>"${out_dir}/evaluator.stderr" &
evaluator_pid=$!
sleep 2

/usr/bin/time -f 'play_wall_duration_sec=%e' -o "${out_dir}/wallclock.txt" \
  rosbag play --clock --quiet "${bag}" \
    --topics /scan /scout_mini_velocity_controller/odom \
    /gazebo/model_states /tf /tf_static

wait "${evaluator_pid}"
evaluator_pid=""
sleep 4
rosrun map_server map_saver -f "${PWD}/${out_dir}/map" \
  >"${out_dir}/map_saver.stdout" 2>"${out_dir}/map_saver.stderr"
