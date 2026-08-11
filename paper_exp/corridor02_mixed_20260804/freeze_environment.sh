#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${WORKSPACE}/paper_exp/corridor02_mixed_20260804/snapshots"
mkdir -p "${OUT}"
source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/install_isolated/setup.bash"

{
  printf 'timestamp_utc=%s\n' "$(date -u +%FT%TZ)"
  printf 'git_head=%s\n' "$(git -C "${WORKSPACE}" rev-parse HEAD)"
  printf 'git_status_porcelain_begin\n'
  git -C "${WORKSPACE}" status --short
  printf 'git_status_porcelain_end\n'
  printf 'ros_distro=%s\n' "$(rosversion -d)"
  printf 'cartographer_ros=%s\n' "$(rosversion cartographer_ros)"
  printf 'compiler=%s\n' "$(c++ --version | sed -n '1p')"
  printf 'cmake=%s\n' "$(cmake --version | sed -n '1p')"
  printf 'ninja=%s\n' "$(ninja --version)"
  printf 'disk=%s\n' "$(df -h "${WORKSPACE}" | tail -n 1)"
} > "${OUT}/environment.txt"

sha256sum \
  "${WORKSPACE}/bags/Corridor02/Corridor02.bag" \
  "${WORKSPACE}/bags/Wheel-float01/Wheel-float01.bag" \
  "${WORKSPACE}/bags/Wheel-float02/Wheel-float02.bag" \
  > "${OUT}/bags.sha256"

CONFIG_DIR="${WORKSPACE}/src/my_navigation/config"
for config in \
  cartographer_m3dgr_mid360_clean_baseline.lua \
  cartographer_m3dgr_mid360_innovation1.lua \
  cartographer_m3dgr_mid360_innovation2.lua \
  cartographer_m3dgr_mid360_full_proposed.lua \
  cartographer_m3dgr_mid360_oad_diagnostic.lua \
  cartographer_m3dgr_mid360_oad_diagnostic_full_yaw.lua; do
  cp "${CONFIG_DIR}/${config}" "${OUT}/${config}"
  cartographer_print_configuration \
    -configuration_directories "${CONFIG_DIR}" \
    -configuration_basename "${config}" \
    > "${OUT}/${config%.lua}.resolved.lua"
done

{
  git -C "${WORKSPACE}" diff --binary
  git -C "${WORKSPACE}" ls-files --others --exclude-standard -z | while IFS= read -r -d '' file; do
    case "${file}" in
      src/cartographer/*|src/cartographer_ros/*|src/my_navigation/*|paper_exp/corridor02_mixed_20260804/*)
        printf '\n=====UNTRACKED:%s=====\n' "${file}"
        sed -n '1,1200p' "${WORKSPACE}/${file}"
        ;;
    esac
  done
} > "${OUT}/source_snapshot.txt"

find "${WORKSPACE}/src/cartographer" "${WORKSPACE}/src/cartographer_ros" "${WORKSPACE}/src/my_navigation/config" \
  -type f -print0 | sort -z | xargs -0 sha256sum > "${OUT}/source_tree.sha256"
printf 'snapshot_dir=%s\n' "${OUT}"
