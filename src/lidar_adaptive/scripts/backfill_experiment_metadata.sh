#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 EXPERIMENT_DIR [...]" >&2
  exit 2
fi

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace_dir="$(cd "$package_dir/../.." && pwd)"
for output_dir in "$@"; do
  [[ -d "$output_dir" ]] || { echo "missing directory: $output_dir" >&2; exit 2; }
  mkdir -p "$output_dir/input_manifest"
  case_name="unknown"
  if [[ "$(basename "$output_dir")" =~ (c[0-3])$ ]]; then
    case_name="${BASH_REMATCH[1]^^}"
  fi
  bag_path="unknown"
  if [[ -f "$output_dir/input_manifest/rosbag_info.yaml" ]]; then
    bag_path="$(awk '$1 == "path:" {print substr($0, 7); exit}' \
      "$output_dir/input_manifest/rosbag_info.yaml")"
  fi
  if [[ ! -f "$output_dir/README.md" ]]; then
    {
      printf '# %s %s\n\n' "$(basename "$bag_path")" "$case_name"
      printf -- '- Input bag: `%s`\n' "$bag_path"
      printf -- '- Case: `%s`\n' "$case_name"
      printf -- '- Existing 60 s formal replay; playback rate was 1.0 x.\n'
      printf -- '- C0: baseline pointcloud_to_laserscan; C1: confidence projection; C2: external scan-thinning.\n'
      printf -- '- This real Scout corridor bag has no continuous external ground truth; ATE/RPE are not reported.\n'
      printf -- '- Raw evidence is under `topics/` and logs; derived metrics are under `metrics/`.\n'
    } > "$output_dir/README.md"
  fi
  git -C "$workspace_dir" rev-parse HEAD > "$output_dir/input_manifest/git_commit.txt"
  git -C "$workspace_dir" diff --check > "$output_dir/input_manifest/git_diff_check.txt"
  {
    printf 'date: %s\n' "$(date --iso-8601=seconds)"
    printf 'ros_distro: %s\n' "$(rosversion -d 2>/dev/null || printf unknown)"
    printf 'kernel: %s\n' "$(uname -sr)"
    printf 'hostname: %s\n' "$(hostname)"
  } > "$output_dir/input_manifest/build_environment.txt"
done
