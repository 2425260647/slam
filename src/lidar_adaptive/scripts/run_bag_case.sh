#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 5 ]]; then
  echo "usage: $0 BAG C0|C1|C2 OUTPUT_DIR [SECONDS] [PLAY_RATE]" >&2
  exit 2
fi

bag_path="$1"
case_name="$2"
output_dir="$3"
duration="${4:-0}"
play_rate="${5:-1.0}"
package_dir="$(rospack find lidar_adaptive)"
workspace_dir="$(cd "$package_dir/../.." && pwd)"

if [[ ! -f "$bag_path" ]]; then
  echo "bag does not exist: $bag_path" >&2
  exit 2
fi
case "$case_name" in
  C0|C1|C2) ;;
  *) echo "case must be C0, C1, or C2" >&2; exit 2 ;;
esac
if [[ "$duration" != "0" && ! "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "SECONDS must be zero or a positive number" >&2
  exit 2
fi

mkdir -p "$output_dir/config_snapshot" "$output_dir/input_manifest" \
  "$output_dir/logs" "$output_dir/topics" "$output_dir/metrics"
if [[ ! -f "$output_dir/README.md" ]]; then
  {
    printf '# %s %s\n\n' "$(basename "$bag_path")" "$case_name"
    printf -- '- Input bag: `%s`\n' "$bag_path"
    printf -- '- Case: `%s`\n' "$case_name"
    printf -- '- Playback: %.3f x, duration limit %.3f s\n' "$play_rate" "$duration"
    printf -- '- C0: baseline pointcloud_to_laserscan; C1: confidence projection; C2: external scan-thinning.\n'
    printf -- '- This real Scout corridor bag has no continuous external ground truth; ATE/RPE are not reported.\n'
    printf -- '- Raw evidence is under `topics/` and logs; derived metrics are under `metrics/`.\n'
  } > "$output_dir/README.md"
fi
sha256sum "$bag_path" > "$output_dir/input_manifest/sha256.txt"
rosbag info --yaml "$bag_path" > "$output_dir/input_manifest/rosbag_info.yaml"
git -C "$workspace_dir" rev-parse HEAD > "$output_dir/input_manifest/git_commit.txt"
git -C "$workspace_dir" diff --check > "$output_dir/input_manifest/git_diff_check.txt"
{
  printf 'date: %s\n' "$(date --iso-8601=seconds)"
  printf 'ros_distro: %s\n' "$(rosversion -d)"
  printf 'kernel: %s\n' "$(uname -sr)"
  printf 'hostname: %s\n' "$(hostname)"
} > "$output_dir/input_manifest/build_environment.txt"
cp "$package_dir/config/projection.yaml" "$output_dir/config_snapshot/"
cp "$package_dir/config/selector.yaml" "$output_dir/config_snapshot/"
cp "$package_dir/launch/lidar_adaptive_bag.launch" "$output_dir/config_snapshot/"
cp "$(rospack find my_navigation)/config/cartographer_scout_2d.lua" \
  "$output_dir/config_snapshot/"

launch_args=()
record_topics=(
  /clock /velodyne_points /scan /scan_confidence /scan_selected
  /lidar_scan_quality /lidar_valid_beam_ratio /lidar_valid_beams
  /lidar_information_score /keyframe_selected /scan_selection
  /lidar_quality_sync_ok /lidar_projection_processing_ms
  /lidar_selector_processing_ms /odom /scout_mini_velocity_controller/odom
  /tf /tf_static /map /gazebo/model_states
)
case "$case_name" in
  C0)
    launch_args=(use_confidence:=false use_baseline_projection:=true
                 cartographer_scan_topic:=/scan)
    ;;
  C1)
    launch_args=(use_confidence:=true forward_selected_scans:=false
                 require_ring:=true)
    ;;
  C2)
    launch_args=(use_confidence:=true forward_selected_scans:=true
                 require_ring:=true)
    ;;
esac

printf '%s\n' \
  "roslaunch lidar_adaptive lidar_adaptive_bag.launch ${launch_args[*]}" \
  "rosbag play --clock -r ${play_rate} ${duration:+--duration=${duration}} ${bag_path}" \
  > "$output_dir/logs/command.txt"

roslaunch lidar_adaptive lidar_adaptive_bag.launch "${launch_args[@]}" \
  > "$output_dir/logs/roslaunch.log" 2>&1 &
launch_pid=$!
record_pid=""
cleanup() {
  if [[ -n "$record_pid" ]]; then
    kill -INT "$record_pid" 2>/dev/null || true
    wait "$record_pid" 2>/dev/null || true
  fi
  kill -INT "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 5
rosbag record --lz4 -O "$output_dir/topics/run.bag" "${record_topics[@]}" \
  > "$output_dir/logs/rosbag_record.log" 2>&1 &
record_pid=$!
sleep 2
if [[ "$duration" == "0" ]]; then
  rosbag play --clock -r "$play_rate" "$bag_path" \
    > "$output_dir/logs/rosbag_play.log" 2>&1
else
  rosbag play --clock -r "$play_rate" --duration="$duration" "$bag_path" \
    > "$output_dir/logs/rosbag_play.log" 2>&1
fi
sleep 1
kill -INT "$record_pid" 2>/dev/null || true
wait "$record_pid" 2>/dev/null || true
record_pid=""

rosbag info --yaml "$output_dir/topics/run.bag" \
  > "$output_dir/topics/run_info.yaml"
rosrun lidar_adaptive analyze_experiment_bag.py \
  "$output_dir/topics/run.bag" --output "$output_dir/metrics/summary.json"
if grep -q '^    - topic: /gazebo/model_states$' \
    "$output_dir/topics/run_info.yaml"; then
  rosrun lidar_adaptive compute_gazebo_trajectory_metrics.py \
    "$output_dir/topics/run.bag" --output "$output_dir/metrics/trajectory.json"
fi
