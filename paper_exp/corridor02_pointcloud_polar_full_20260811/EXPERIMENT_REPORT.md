# Corridor02 pointcloud polar insertion validation

Date: 2026-08-11

## Scope

- Stable deployment baseline: Cartographer consumes `/scan`.
- Candidate: Cartographer consumes time-bearing `/velodyne_points`; scan
  matching keeps the full cloud, while submap insertion keeps the nearest
  return in each `0.125 deg` azimuth bin.
- The polar filter remains disabled by default. This experiment does not
  overwrite the stable deployment entry.

The two runs do not use an identical sensor-input chain. Their comparison is
therefore a deployment-candidate gate, not an isolated polar-filter ablation.

## Runtime validation

- Full bag duration: `293 s`
- Playback rate: `1.0x`
- `/map`: `315` messages
- `/local_occupancy_grid`: `2934` messages
- `/local_occupancy_grid_map`: `2932` messages
- `/tf`: `25572` messages
- Final map: `2103 x 1373` cells at `0.05 m`
- Fatal/abort/segmentation errors: `0`
- Whole-frame drops in the pointcloud-to-scan path: `0`

The Livox point stream produced `2835` adjacent-message overlap warnings and
cropped `120641` points, approximately `0.21%` of about 58.6 million input
points. No whole pointcloud message was rejected. This remains an input-time
audit item rather than evidence of a CPU real-time failure.

## Structural comparison

These are fixed-parameter occupancy-grid structure proxies. Corridor02 has no
continuous ground-truth trajectory suitable for this run, so the metrics are
not ATE/RPE and do not establish absolute localization accuracy.

| Metric | Stable `/scan` baseline | Pointcloud + polar candidate | Direction |
|---|---:|---:|---|
| Occupied components | 540 | 551 | `+2.0%`, worse |
| Longest Hough wall | 29.723 m | 30.063 m | `+1.1%`, better |
| Median wall p95 residual | 0.0698 m | 0.0737 m | `+5.5%`, worse |
| Worst wall p95 residual | 0.2098 m | 0.2027 m | `-3.4%`, better |
| Median local heading p95 | 1.755 deg | 1.961 deg | `+11.7%`, worse |
| Worst local heading p95 | 3.977 deg | 8.282 deg | `+108.2%`, worse |
| Median parallel-spacing std. | 0.0325 m | 0.0473 m | `+45.7%`, worse |
| Worst parallel-spacing std. | 0.0537 m | 0.1687 m | `+214.0%`, worse |
| Occupied cells | 12124 | 12885 | `+6.3%` |
| Known cells | 265040 | 300168 | `+13.3%` |

The candidate preserves the closed-corridor topology and slightly extends the
longest detected wall. However, visual inspection agrees with the fixed
metrics: several local wall segments fluctuate more strongly, parallel-wall
spacing is less stable, and ray-shaped artifacts and occupied fragments are
more visible.

## Decision

The candidate fails the Corridor02 generalization gate. It must not replace
the stable `/scan` deployment path. The repeat03 improvement is retained as a
dataset-specific positive result, but it is insufficient evidence for a
general algorithm upgrade.

The next scientifically valid isolation test, if this branch is continued, is
`pointcloud polar OFF` versus `pointcloud polar ON` on the same Corridor02
pointcloud input, same executable, same Lua parameters, and same playback
rate. Even a positive isolated ablation would still require a short real-car
test before deployment because neither current bag provides continuous pose
ground truth.

## Reproduction artifacts

- Candidate map: `map.pgm`, `map.yaml`, `map_preview.png`
- Fixed structural metrics: `straightness_comparison.json`
- Runtime logs: `roslaunch.log`, `rosbag_play.log`, `record.log`
- Recorded outputs: `outputs.bag`

