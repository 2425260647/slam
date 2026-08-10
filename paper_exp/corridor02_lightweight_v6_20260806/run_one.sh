#!/usr/bin/env bash
set -euo pipefail

algorithm="${1:?algorithm: lightweight_v6|lightweight_v7c|lightweight_multiscale_consensus|lightweight_multiscale_temporal|lightweight_multiscale_temporal_gatefix|lightweight_multiscale_proposaldiag|lightweight_multiscale_sep250|lightweight_sep250_candidate_diag|lightweight_sep250_verified_loop|lightweight_sep250_verified_loop_smooth|lightweight_sep250_verified_loop_smooth_fixed|lightweight_sep250_verified_loop_smooth_fixed_densemap|lightweight_sep250_verified_loop_smooth_fixed_connectedmap|lightweight_sep250_verified_loop_smooth_fixed_connectedmap_rerun|lightweight_sep250_verified_loop_smooth_fixed_sensor_model|lightweight_sep250_verified_loop_smooth_fixed_hit060|lightweight_sep250_verified_loop_smooth_fixed_hit060_dense|lightweight_rigid_submap_diagnostic|lightweight_rigid_submap_v1|lightweight_prediction_prior_rigid_v2|lightweight_submap_texture_phase_a|lightweight_submap_texture_phase_a_v2|lightweight_submap_texture_phase_a_v3|lightweight_submap_texture_phase_a_v4_sync|lightweight_submap_texture_phase_b_deskew|lightweight_phase_b_fullres_texture|backend_scan_huber|backend_switch1|backend_switch1_full|backend_odom|backend_full|lightweight_hybrid|lightweight_quality_robust|cartographer_clean|cartographer_clean_phase_b_deskew}"
mode="${2:?mode: preflight|full}"
port="${3:?ROS master port}"

workspace="/home/slam/slam_ws"
exp_root="${workspace}/paper_exp/corridor02_lightweight_v6_20260806"
input_bag="${exp_root}/common_inputs.bag"
case "${algorithm}" in
  lightweight_submap_texture_phase_b_deskew|lightweight_phase_b_fullres_texture|cartographer_clean_phase_b_deskew)
    input_bag="${exp_root}/common_inputs_deskewed.bag"
    ;;
esac
run_dir="${exp_root}/${mode}/${algorithm}"
gt="${workspace}/bags/Corridor02/GTCorridor02.txt"
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
export ROS_LOG_DIR="${workspace}/.ros_runtime/corridor02_${mode}_${algorithm}"
export ROS_HOME="${ROS_LOG_DIR}/home"
mkdir -p "${ROS_LOG_DIR}" "${ROS_HOME}"

source /opt/ros/noetic/setup.bash
source "${workspace}/install_isolated/setup.bash"

roscore -p "${port}" >"${run_dir}/roscore.log" 2>&1 &
master_pid=$!
launch_pid=""
pose_pid=""
record_pid=""
map_hz_pid=""
path_hz_pid=""
local_grid_hz_pid=""
cleanup() {
  set +e
  if [[ -n "${record_pid}" ]]; then kill -INT "${record_pid}" 2>/dev/null; fi
  if [[ -n "${pose_pid}" ]]; then kill "${pose_pid}" 2>/dev/null; fi
  if [[ -n "${map_hz_pid}" ]]; then kill "${map_hz_pid}" 2>/dev/null; fi
  if [[ -n "${path_hz_pid}" ]]; then kill "${path_hz_pid}" 2>/dev/null; fi
  if [[ -n "${local_grid_hz_pid}" ]]; then kill "${local_grid_hz_pid}" 2>/dev/null; fi
  if [[ -n "${launch_pid}" ]]; then kill "${launch_pid}" 2>/dev/null; fi
  kill "${master_pid}" 2>/dev/null
  wait "${record_pid}" 2>/dev/null
  wait "${pose_pid}" 2>/dev/null
  wait "${map_hz_pid}" 2>/dev/null
  wait "${path_hz_pid}" 2>/dev/null
  wait "${local_grid_hz_pid}" 2>/dev/null
  wait "${launch_pid}" 2>/dev/null
  wait "${master_pid}" 2>/dev/null
}
trap cleanup EXIT INT TERM
for attempt in $(seq 1 80); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.25
done
rosparam list >/dev/null

case "${algorithm}" in
  lightweight_v6)
    roslaunch "${exp_root}/offline_lightweight_v6.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_quality_robust)
    roslaunch "${exp_root}/offline_lightweight_quality_robust_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_v7c)
    roslaunch "${exp_root}/offline_lightweight_v7c_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_multiscale_consensus)
    roslaunch "${exp_root}/offline_lightweight_multiscale_consensus_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_multiscale_temporal)
    roslaunch "${exp_root}/offline_lightweight_multiscale_temporal_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_multiscale_temporal_gatefix)
    roslaunch "${exp_root}/offline_lightweight_multiscale_temporal_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_multiscale_proposaldiag)
    roslaunch "${exp_root}/offline_lightweight_multiscale_temporal_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_multiscale_sep250)
    roslaunch "${exp_root}/offline_lightweight_multiscale_temporal_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_sep250_candidate_diag)
    roslaunch "${exp_root}/offline_lightweight_sep250_candidate_diag_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_sep250_verified_loop)
    roslaunch "${exp_root}/offline_lightweight_sep250_verified_loop_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_sep250_verified_loop_smooth)
    roslaunch "${exp_root}/offline_lightweight_sep250_verified_loop_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_sep250_verified_loop_smooth_fixed)
    roslaunch "${exp_root}/offline_lightweight_sep250_verified_loop_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_sep250_verified_loop_smooth_fixed_densemap)
    roslaunch "${exp_root}/offline_lightweight_sep250_verified_loop_densemap_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_sep250_verified_loop_smooth_fixed_connectedmap)
    roslaunch "${exp_root}/offline_lightweight_sep250_verified_loop_connected_map_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_sep250_verified_loop_smooth_fixed_connectedmap_rerun)
    roslaunch "${exp_root}/offline_lightweight_sep250_verified_loop_connected_map_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_sep250_verified_loop_smooth_fixed_sensor_model)
    roslaunch "${exp_root}/offline_lightweight_sep250_verified_loop_sensor_model_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_sep250_verified_loop_smooth_fixed_hit060)
    roslaunch "${exp_root}/offline_lightweight_sep250_verified_loop_hit060_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_sep250_verified_loop_smooth_fixed_hit060_dense)
    roslaunch "${exp_root}/offline_lightweight_sep250_verified_loop_hit060_dense_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_rigid_submap_diagnostic)
    roslaunch "${exp_root}/offline_lightweight_sep250_verified_loop_hit060_dense_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_rigid_submap_v1)
    roslaunch "${exp_root}/offline_lightweight_rigid_submap_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_prediction_prior_rigid_v2)
    roslaunch "${exp_root}/offline_lightweight_prediction_prior_rigid_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_submap_texture_phase_a)
    roslaunch "${exp_root}/offline_lightweight_submap_texture_phase_a_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_submap_texture_phase_a_v2)
    roslaunch "${exp_root}/offline_lightweight_submap_texture_phase_a_v2_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_submap_texture_phase_a_v3)
    roslaunch "${exp_root}/offline_lightweight_submap_texture_phase_a_v3_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_submap_texture_phase_a_v4_sync)
    roslaunch "${exp_root}/offline_lightweight_submap_texture_phase_a_v4_sync_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_submap_texture_phase_b_deskew)
    roslaunch "${exp_root}/offline_lightweight_submap_texture_phase_b_deskew_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_phase_b_fullres_texture)
    roslaunch "${exp_root}/offline_lightweight_submap_texture_phase_b_deskew_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  backend_scan_huber)
    roslaunch "${exp_root}/offline_lightweight_backend_robust_corridor02.launch" \
      scan_robust:=true scan_huber_scale:=1.0 switch_prior_weight:=25.0 \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  backend_switch1)
    roslaunch "${exp_root}/offline_lightweight_backend_robust_corridor02.launch" \
      scan_robust:=false switch_prior_weight:=1.0 \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  backend_switch1_full)
    roslaunch "${exp_root}/offline_lightweight_backend_robust_corridor02.launch" \
      scan_robust:=false switch_prior_weight:=1.0 anisotropic_loops:=false \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  backend_odom)
    roslaunch "${exp_root}/offline_lightweight_backend_robust_corridor02.launch" \
      scan_robust:=true scan_huber_scale:=1.0 odom_weight:=0.5 \
      odom_robust:=true odom_huber_scale:=1.0 switch_prior_weight:=1.0 \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  backend_full)
    roslaunch "${exp_root}/offline_lightweight_backend_robust_corridor02.launch" \
      scan_robust:=true scan_huber_scale:=2.0 odom_weight:=0.5 \
      odom_robust:=true odom_huber_scale:=1.0 switch_prior_weight:=1.0 \
      optimize_every:=100 \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  lightweight_hybrid)
    roslaunch "${exp_root}/offline_lightweight_hybrid_corridor02.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  cartographer_clean)
    roslaunch "${exp_root}/offline_cartographer_clean.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  cartographer_clean_phase_b_deskew)
    roslaunch "${exp_root}/offline_cartographer_clean_phase_b_deskew.launch" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    ;;
  *)
    echo "Unknown algorithm: ${algorithm}" >&2
    exit 2
    ;;
esac
launch_pid=$!
sleep 6

rosnode list >"${run_dir}/nodes.txt"
while read -r node; do
  rosnode info "${node}" || true
done <"${run_dir}/nodes.txt" >"${run_dir}/node_info.txt" 2>&1
rosparam dump "${run_dir}/params.yaml"
sha256sum "${input_bag}" >"${run_dir}/input.sha256"

rostopic echo -p /tracked_pose >"${run_dir}/tracked_pose.csv" \
  2>"${run_dir}/tracked_pose.err" &
pose_pid=$!
rostopic hz -w 20 /map >"${run_dir}/map_hz.txt" 2>&1 &
map_hz_pid=$!
rostopic hz -w 20 /lightweight_slam/path >"${run_dir}/path_hz.txt" 2>&1 &
path_hz_pid=$!
rostopic hz -w 20 /local_occupancy_grid \
  >"${run_dir}/local_grid_hz.txt" 2>&1 &
local_grid_hz_pid=$!
rosbag record --lz4 -O "${run_dir}/outputs.bag" \
  /tracked_pose /slam_diagnostics /lightweight_slam/path \
  /lightweight_slam/local_path /lightweight_slam/submap_path \
  /lightweight_slam/loop_edges /tf /tf_static \
  >"${run_dir}/record.log" 2>&1 &
record_pid=$!
for attempt in $(seq 1 80); do
  if rosnode list 2>/dev/null | grep -q '^/record'; then break; fi
  sleep 0.25
done
rosnode list | grep -q '^/record'
sleep 1

{
  printf 'algorithm=%s\nmode=%s\ninput_bag=%s\n' \
    "${algorithm}" "${mode}" "${input_bag}"
  /usr/bin/time -f 'play_wall_duration_sec=%e' \
    rosbag play --clock --quiet --rate=1.0 "${duration_args[@]}" \
      "${input_bag}" --topics /scan /odom
} >"${run_dir}/play.stdout" 2>"${run_dir}/wallclock.txt"

if [[ "${algorithm}" == "cartographer_clean" ||
      "${algorithm}" == "cartographer_clean_phase_b_deskew" ]]; then
  rosservice call /finish_trajectory 0 \
    >"${run_dir}/finish_trajectory.log" 2>&1 || true
fi
sleep 5
timeout 30s rosrun map_server map_saver -f "${run_dir}/map" \
  >"${run_dir}/map_saver.stdout" 2>"${run_dir}/map_saver.stderr"

kill -INT "${record_pid}"
wait "${record_pid}"
record_pid=""
kill "${pose_pid}" 2>/dev/null || true
wait "${pose_pid}" 2>/dev/null || true
pose_pid=""
for monitor_pid in "${map_hz_pid}" "${path_hz_pid}" "${local_grid_hz_pid}"; do
  kill "${monitor_pid}" 2>/dev/null || true
  wait "${monitor_pid}" 2>/dev/null || true
done
map_hz_pid=""
path_hz_pid=""
local_grid_hz_pid=""

if [[ ! -s "${run_dir}/map.pgm" || ! -s "${run_dir}/outputs.bag" || \
      ! -s "${run_dir}/tracked_pose.csv" ]]; then
  echo "Run did not produce required outputs" >&2
  exit 5
fi
if grep -Eqi 'FATAL|segmentation fault|core dumped|Check failed' \
    "${run_dir}/roslaunch.log"; then
  echo "Fatal pattern found in roslaunch log" >&2
  exit 6
fi

python3 "${workspace}/paper_exp/corridor_repeat03_threeway_20260805/summarize_run.py" \
  --algorithm "${algorithm}" --mode "${mode}" \
  --input-bag "${input_bag}" --run-dir "${run_dir}" \
  >"${run_dir}/summary.stdout"
if [[ "${mode}" == "full" ]]; then
  python3 "${exp_root}/evaluate_endpoint.py" \
    --trajectory "${run_dir}/tracked_pose.csv" \
    --ground-truth "${gt}" --output "${run_dir}/endpoint_metrics.json" \
    >"${run_dir}/endpoint_metrics.stdout"
fi
touch "${run_dir}/SUCCESS"
echo "Completed ${mode}/${algorithm}"
