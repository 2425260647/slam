# LiDAR submap admission gate preflight

Date: 2026-08-04

## Scope

This is an implementation and chain-validity preflight, not a paper performance
experiment. The gate uses the LiDAR-only occupied-space Jacobian diagnostic and
is enabled only in `cartographer_m3dgr_mid360_submap_gate.lua`. The clean run
uses `cartographer_m3dgr_mid360_clean_baseline.lua`. Both runs replay the same
first 10 seconds of `bags/Corridor02/Corridor02.bag`, with only MID-360,
projected `/scan`, and `/odom`; no IMU, Avia, or visual topics are replayed.

## Implementation checks

- `catkin_make_isolated --install --use-ninja --pkg cartographer cartographer_ros`
  passed after the gate implementation.
- Lua configuration resolution passed and showed the gate enabled only in the
  gate configuration.
- Existing 2D tests passed: Ceres scan matcher 9/9, real-time correlative scan
  matcher 8/8, optimization problem 2D 3/3.
- `git diff --check` passed.

## Chain results

| Run | `/scan` | `/odom` | `tracked_pose` | map | gate blocked | forced |
|---|---:|---:|---:|---|---:|---:|
| clean baseline | 99 | 198 | 911 | 148 x 764 | 0 | 0 |
| admission gate | 100 | 200 | 903 | 149 x 842 | 36 diagnostic samples | 0 |

The gate run produced a non-empty latched `/map` and no FATAL, check failure,
segmentation fault, or ROS launch error. The blocked topic was asserted during a
short low-information interval and returned to false after valid scans. The
36 blocked messages are timer-level diagnostics, not 36 independent scans.

## Interpretation and limits

The run proves that the new guard can preserve local pose/TF and global map
publication while temporarily withholding `InsertionResult` for weak scans. It
does not prove lower ATE/RPE, better corridor structure, or global consistency.
The provisional `min_information=10.0` threshold was selected only to exercise
the state machine during preflight; it is not frozen for a final experiment and
must be calibrated on a development segment, then evaluated once on an
independent test segment. The different 10-second map extents are therefore not
reported as an improvement.

Run artifacts:

- `corridor02_gate_10s/`
- `corridor02_clean_baseline_10s/`
