#!/usr/bin/env bash
set -euo pipefail

# Same-bag ablation for the heterogeneous self-recorded corridor_new sequence.
# This bag contains /velodyne_points and /odom, but no continuous external truth.
# Therefore this runner records stability/diagnostic/map-structure evidence only.

WORKSPACE="/home/slam/slam_ws"
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${BAG_OVERRIDE:-${WORKSPACE}/bags/corridor_new.bag}"
RUNS_DIR="${RUNS_DIR_OVERRIDE:-${SCRIPT_ROOT}/runs}"
CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"
MAP_SAVER="${WORKSPACE}/paper_exp/innovation1_directional_adaptive_fusion/save_occupancy_grid.py"
PLAYBACK_RATE="${PLAYBACK_RATE:-2.0}"
FORCE="${FORCE:-0}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"

wait_for_ros() {
  for _ in $(seq 1 160); do
    if rosparam list >/dev/null 2>&1; then return 0; fi
    sleep 0.25
  done
  echo "ROS master did not become ready." >&2
  return 1
}

wait_for_service() {
  local service="$1"
  for _ in $(seq 1 200); do
    if rosservice list 2>/dev/null | rg -q "^${service}$"; then return 0; fi
    sleep 0.25
  done
  echo "Service did not become ready: ${service}" >&2
  return 1
}

stop_pid() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  fi
}

run_case() {
  local name="$1"
  local config="$2"
  local run_dir="${RUNS_DIR}/${name}"
  local roscore_pid="" launch_pid="" tf_laser_pid="" tf_footprint_pid=""
  local -a capture_pids=()

  if [[ -f "${run_dir}/SUCCESS" && "${FORCE}" != "1" ]]; then
    echo "[corridor_new] Reusing ${name}"
    return 0
  fi
  if [[ -d "${run_dir}" && "${FORCE}" != "1" ]]; then
    echo "Incomplete run exists; set FORCE=1 after inspection or FORCE=1: ${run_dir}" >&2
    return 1
  fi
  if [[ -d "${run_dir}" && "${FORCE}" == "1" ]]; then
    mv "${run_dir}" "${run_dir}.legacy.$(date +%Y%m%d_%H%M%S)"
  fi
  mkdir -p "${run_dir}"
  export ROS_HOME="${run_dir}/ros_home"
  mkdir -p "${ROS_HOME}"

  cleanup_case() {
    local pid
    for pid in "${capture_pids[@]:-}"; do stop_pid "${pid}"; done
    stop_pid "${tf_laser_pid}"
    stop_pid "${tf_footprint_pid}"
    stop_pid "${launch_pid}"
    stop_pid "${roscore_pid}"
  }
  trap cleanup_case RETURN

  {
    printf 'experiment=corridor_new_same_bag_ablation\n'
    printf 'case=%s\nconfig=%s\n' "${name}" "${config}"
    printf 'bag=%s\nbag_sha256=%s\n' "${BAG}" "$(sha256sum "${BAG}" | awk '{print $1}')"
    printf 'algorithm_version=innovation_combined_v2_20260730\n'
    printf 'git_head=%s\n' "$(git -C "${WORKSPACE}" rev-parse HEAD)"
    printf 'playback_rate=%s\n' "${PLAYBACK_RATE}"
    printf 'input_topics=/velodyne_points,/odom\n'
    printf 'imu_used=false\navia_used=false\n'
    printf 'static_tf_base_link_to_laser_link=0 0 0.17 -1.5708 0 0\n'
    printf 'static_tf_base_link_to_base_footprint=0 0 -0.23479 0 0 0\n'
    printf 'projection_target_frame=laser_link\nprojection_height_m=-0.515,1.435\n'
    printf 'start_wall_utc=%s\n' "$(date -u +%FT%TZ)"
  } >"${run_dir}/metadata.txt"
  cp "${CONFIG_DIR}/${config}" "${run_dir}/config_snapshot.lua"
  cartographer_print_configuration \
    -configuration_directories "${CONFIG_DIR}" \
    -configuration_basename "${config}" \
    >"${run_dir}/resolved_configuration.lua"

  roscore >"${run_dir}/roscore.log" 2>&1 &
  roscore_pid=$!
  wait_for_ros
  rosparam set /use_sim_time true

  rosrun tf static_transform_publisher \
    0 0 0.17 -1.5708 0 0 base_link laser_link 100 \
    >"${run_dir}/static_tf_laser.log" 2>&1 &
  tf_laser_pid=$!
  rosrun tf static_transform_publisher \
    0 0 -0.23479 0 0 0 base_link base_footprint 100 \
    >"${run_dir}/static_tf_footprint.log" 2>&1 &
  tf_footprint_pid=$!

  roslaunch my_navigation real_scout_mapping_local_grid.launch \
    cartographer_config_dir:="${CONFIG_DIR}" \
    cartographer_config:="${config}" \
    cloud_topic:=/velodyne_points \
    odom_topic:=/odom \
    target_frame:=laser_link \
    min_height:=-0.515 \
    max_height:=1.435 \
    scan_quality_gate_enabled:=true \
    rviz:=false \
    enable_monitor:=false \
    >"${run_dir}/roslaunch.log" 2>&1 &
  launch_pid=$!

  wait_for_service "/finish_trajectory"
  wait_for_service "/write_state"
  sleep 2

  for topic in /tracked_pose /degeneracy_metric /degeneracy_direction \
      /slip_metric /slip_state /odom_weight_scale /odom_min_weight_scale \
      /slip_lidar_reliability /consistency_anomaly_trigger_count \
      /consistency_anomaly_time; do
    local stem="${topic#/}"
    rostopic echo -p "${topic}" >"${run_dir}/${stem}.csv" \
      2>"${run_dir}/${stem}.err" &
    capture_pids+=("$!")
  done

  local play_start play_end
  play_start="$(date +%s.%N)"
  rosbag play --clock --quiet --rate="${PLAYBACK_RATE}" "${BAG}" \
    --topics /velodyne_points /odom \
    >"${run_dir}/rosbag_play.log" 2>&1
  play_end="$(date +%s.%N)"
  printf 'play_start_wall_sec=%s\nplay_end_wall_sec=%s\n' \
    "${play_start}" "${play_end}" >>"${run_dir}/metadata.txt"
  awk -v start="${play_start}" -v end="${play_end}" \
    'BEGIN {printf "play_wall_duration_sec=%.6f\n", end-start}' \
    >>"${run_dir}/metadata.txt"

  sleep 5
  rosservice call /finish_trajectory "trajectory_id: 0" \
    >"${run_dir}/finish_trajectory.log" 2>&1
  sleep 12
  rosservice call /write_state \
    "{filename: '${run_dir}/state.pbstream', include_unfinished_submaps: true}" \
    >"${run_dir}/write_state.log" 2>&1
  python3 "${MAP_SAVER}" --topic /map \
    --output-prefix "${run_dir}/map" --timeout 60 \
    >"${run_dir}/map_saver.log" 2>&1

  rosnode list >"${run_dir}/nodes.txt" 2>&1 || true
  rostopic list >"${run_dir}/topics.txt" 2>&1 || true
  rg -n "FATAL|Check failed|Segmentation fault|terminate called" \
    "${run_dir}/roslaunch.log" "${run_dir}/static_tf_laser.log" \
    "${run_dir}/static_tf_footprint.log" \
    >"${run_dir}/fatal_scan.txt" || true

  test -s "${run_dir}/tracked_pose.csv"
  test -s "${run_dir}/map.pgm"
  test -s "${run_dir}/map.yaml"
  test -s "${run_dir}/state.pbstream"
  printf 'end_wall_utc=%s\n' "$(date -u +%FT%TZ)" >>"${run_dir}/metadata.txt"
  touch "${run_dir}/SUCCESS"
  cleanup_case
  trap - RETURN
  sleep 3
  echo "[corridor_new] Finished ${name}"
}

mkdir -p "${RUNS_DIR}"
run_case c0_clean_baseline cartographer_m3dgr_mid360_clean_baseline.lua
run_case c1_innovation1 cartographer_m3dgr_mid360_innovation1.lua
run_case c3_full_proposed cartographer_m3dgr_mid360_full_proposed.lua

python3 - "${RUNS_DIR}" "${SCRIPT_ROOT}/corridor_new_ablation_metrics.json" <<'PY'
import csv
import json
import os
import sys

root, output = sys.argv[1:3]
module_dir = "/home/slam/slam_ws/paper_exp/formal_corridor_20260724"
sys.path.insert(0, module_dir)
from evaluate_wall_parameter_maps import map_metrics

def values(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path, encoding="utf-8") as stream:
        for row in csv.reader(stream):
            if len(row) >= 2:
                try:
                    out.append(float(row[1]))
                except ValueError:
                    pass
    return out

report = {
    "dataset": "self-recorded corridor_new",
    "reference_limit": "no continuous external truth; map/diagnostic/stability evidence only",
    "heterogeneous_path": True,
    "runs": {},
}
for name in ("c0_clean_baseline", "c1_innovation1", "c3_full_proposed"):
    d = os.path.join(root, name)
    metrics = map_metrics(os.path.join(d, "map"))
    def mean(v):
        return sum(v) / len(v) if v else None
    def ratio(v, target):
        return sum(1 for x in v if x > target) / len(v) if v else None
    report["runs"][name] = {
        "success": os.path.isfile(os.path.join(d, "SUCCESS")),
        "fatal_scan_bytes": os.path.getsize(os.path.join(d, "fatal_scan.txt")),
        "tracked_pose_rows": max(0, sum(1 for _ in open(os.path.join(d, "tracked_pose.csv"), encoding="utf-8")) - 1),
        "map": metrics,
        "degeneracy_mean": mean(values(os.path.join(d, "degeneracy_metric.csv"))),
        "degeneracy_max": max(values(os.path.join(d, "degeneracy_metric.csv")), default=None),
        "slip_metric_mean": mean(values(os.path.join(d, "slip_metric.csv"))),
        "slip_metric_max": max(values(os.path.join(d, "slip_metric.csv")), default=None),
        "slip_state_true_ratio": ratio(values(os.path.join(d, "slip_state.csv")), 0.5),
        "odom_weight_min": min(values(os.path.join(d, "odom_weight_scale.csv")), default=None),
        "odom_weight_mean": mean(values(os.path.join(d, "odom_weight_scale.csv"))),
        "anomaly_trigger_max": max(values(os.path.join(d, "consistency_anomaly_trigger_count.csv")), default=None),
    }
with open(output, "w", encoding="utf-8") as stream:
    json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
    stream.write("\n")
print(json.dumps(report, ensure_ascii=False, indent=2))
PY

echo "[corridor_new] Completed all cases under ${RUNS_DIR}"
