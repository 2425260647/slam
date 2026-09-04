#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 OUTPUT_BAG [SECONDS]" >&2
  exit 2
fi

output_bag="$1"
duration="${2:-0}"
mkdir -p "$(dirname "${output_bag}")"

topics=(
  /clock /velodyne_points /scan /scan_confidence /scan_selected
  /lidar_scan_quality /lidar_valid_beam_ratio /lidar_valid_beams
  /lidar_information_score /keyframe_selected /scan_selection /lidar_quality_sync_ok
  /lidar_projection_processing_ms /lidar_selector_processing_ms
  /odom /scout_mini_velocity_controller/odom /tf /tf_static /gazebo/model_states
)

echo "[lidar_adaptive] recording ${output_bag}"
if [[ "${duration}" == "0" ]]; then
  exec rosbag record --lz4 -O "${output_bag}" "${topics[@]}"
fi

rosbag record --lz4 -O "${output_bag}" "${topics[@]}" &
record_pid=$!
trap 'kill -INT "${record_pid}" 2>/dev/null || true' EXIT INT TERM
sleep "${duration}"
kill -INT "${record_pid}"
wait "${record_pid}" || true
