#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "usage: $0 C0|C1|C2 OUTPUT_DIR SECONDS [WORLD]" >&2
  exit 2
fi

case_name="$1"
output_dir="$2"
duration="$3"
package_dir="$(rospack find lidar_adaptive)"
workspace_dir="$(cd "$package_dir/../.." && pwd)"
world_path="${4:-$package_dir/worlds/corridor_elevator.world}"

case "$case_name" in C0|C1|C2) ;; *) echo "case must be C0, C1, or C2" >&2; exit 2 ;; esac
[[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "SECONDS must be positive" >&2; exit 2; }
[[ -f "$world_path" ]] || { echo "world does not exist: $world_path" >&2; exit 2; }

mkdir -p "$output_dir/config_snapshot" "$output_dir/input_manifest" \
  "$output_dir/logs" "$output_dir/topics" "$output_dir/metrics"
{
  printf '# Gazebo corridor elevator %s\n\n' "$case_name"
  printf -- '- World: `%s`\n' "$world_path"
  printf -- '- Motion: Scout Mini starts at x=-2 m and receives 0.35 m/s forward velocity, crossing the elevator opening.\n'
  printf -- '- Duration: %.3f s\n' "$duration"
  printf -- '- C0: baseline pointcloud_to_laserscan; C1: confidence projection; C2: external scan-thinning.\n'
  printf -- '- Gazebo `/gazebo/model_states` is continuous truth; ATE/RPE are computed only for this simulated run.\n'
} > "$output_dir/README.md"
sha256sum "$world_path" > "$output_dir/input_manifest/world_sha256.txt"
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
cp "$package_dir/launch/lidar_adaptive_cartographer.launch" "$output_dir/config_snapshot/"
cp "$(rospack find my_navigation)/config/cartographer_scout_2d.lua" \
  "$output_dir/config_snapshot/"

launch_args=(start_sim:=true world_name:="$world_path" gui:=false rviz:=false x:=-2 y:=0 yaw:=0)
record_topics=(
  /clock /velodyne_points_raw /velodyne_points /scan /scan_confidence /scan_selected
  /lidar_scan_quality /lidar_valid_beam_ratio /lidar_valid_beams
  /lidar_information_score /keyframe_selected /scan_selection /lidar_quality_sync_ok
  /lidar_projection_processing_ms /lidar_selector_processing_ms
  /scout_mini_velocity_controller/odom /tf /tf_static /map /gazebo/model_states
)
case "$case_name" in
  C0) launch_args+=(use_confidence:=false use_baseline_projection:=true
                    cartographer_scan_topic:=/scan) ;;
  C1) launch_args+=(use_confidence:=true use_baseline_projection:=false
                    forward_selected_scans:=false cartographer_scan_topic:=/scan_selected) ;;
  C2) launch_args+=(use_confidence:=true use_baseline_projection:=false
                    forward_selected_scans:=true cartographer_scan_topic:=/scan_selected) ;;
esac
printf '%s\n' \
  "roslaunch lidar_adaptive lidar_adaptive_cartographer.launch ${launch_args[*]}" \
  "rostopic pub -r 10 /scout_mini_velocity_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.35}, angular: {z: 0.0}}'" \
  "rosbag record --lz4 -O $output_dir/topics/run.bag ${record_topics[*]}" \
  > "$output_dir/logs/command.txt"

roslaunch lidar_adaptive lidar_adaptive_cartographer.launch "${launch_args[@]}" \
  > "$output_dir/logs/roslaunch.log" 2>&1 &
launch_pid=$!
record_pid=""
cmd_pid=""
cleanup() {
  [[ -n "$cmd_pid" ]] && kill -INT "$cmd_pid" 2>/dev/null || true
  [[ -n "$record_pid" ]] && kill -INT "$record_pid" 2>/dev/null || true
  [[ -n "$record_pid" ]] && wait "$record_pid" 2>/dev/null || true
  kill -INT "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 8
rosbag record --lz4 -O "$output_dir/topics/run.bag" "${record_topics[@]}" \
  > "$output_dir/logs/rosbag_record.log" 2>&1 &
record_pid=$!
sleep 2
rostopic pub -r 10 /scout_mini_velocity_controller/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.35}, angular: {z: 0.0}}" \
  > "$output_dir/logs/cmd_vel.log" 2>&1 &
cmd_pid=$!
sleep "$duration"
kill -INT "$cmd_pid" 2>/dev/null || true
wait "$cmd_pid" 2>/dev/null || true
cmd_pid=""
sleep 2
kill -INT "$record_pid" 2>/dev/null || true
wait "$record_pid" 2>/dev/null || true
record_pid=""

rosbag info --yaml "$output_dir/topics/run.bag" > "$output_dir/topics/run_info.yaml"
rosrun lidar_adaptive analyze_experiment_bag.py \
  "$output_dir/topics/run.bag" --output "$output_dir/metrics/summary.json"
rosrun lidar_adaptive compute_gazebo_trajectory_metrics.py \
  "$output_dir/topics/run.bag" --output "$output_dir/metrics/trajectory.json"
