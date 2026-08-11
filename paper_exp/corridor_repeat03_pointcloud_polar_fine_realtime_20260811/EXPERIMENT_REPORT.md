# repeat03 pointcloud polar insertion experiment

Date: 2026-08-11

## Scope

- Cartographer scan matching consumes `/velodyne_points` with per-point time.
- Submap insertion alone keeps the nearest return in each `0.125 deg` azimuth
  bin.
- `/scan` remains available to `local_grid_mapper`.
- The stable `/scan` deployment configuration is not overwritten.

## Runtime validation

- Full bag duration: `578 s`
- Playback rate: `1.0x`
- Playback wall time: `579.80 s`
- `/map`: `601` messages, approximately `1.04 Hz`
- `/local_occupancy_grid`: `5784` messages, approximately `10.0 Hz`
- `/local_occupancy_grid_map`: `5782` messages, approximately `10.0 Hz`
- `/tf`: `89569` messages
- Fatal/abort/segmentation errors: `0`
- The final PGM is byte-identical between the `1.0x` and `5.0x` runs.

The `174` `Dropped earlier points` warnings occur in both playback rates. They
therefore cannot be attributed to insufficient real-time CPU throughput alone;
they are retained as a point-time/collation audit item.

## Structural comparison

These are fixed-parameter occupancy-grid proxies, not surveyed wall truth and
not ATE/RPE.

| Metric | Stable `/scan` baseline | Pointcloud + 0.125 deg polar insertion |
|---|---:|---:|
| Occupied components | 130 | 107 |
| Longest Hough wall | 24.950 m | 27.004 m |
| Median wall p95 residual | 0.113 m | 0.107 m |
| Worst wall p95 residual | 0.160 m | 0.168 m |
| Median local heading p95 | 3.948 deg | 2.775 deg |
| Median parallel-spacing std. | 0.028 m | 0.013 m |

The `0.25 deg` variant reduced median wall residual further but shortened the
longest detected wall to `17.403 m`, showing that its angular bins were too
coarse. The `0.125 deg` variant restored continuity while preserving most of
the local wall-line improvement.

## Conclusion

The `0.125 deg` variant is a validated candidate for pointcloud-input mapping:
it improves several repeat03 wall structure proxies and preserves the required
ROS interfaces. It does not prove better absolute localization because this
bag has no continuous ground truth, and its worst wall residual is slightly
worse than the stable baseline. It must not replace the current `/scan` main
entry until the input-chain change is explicitly approved and independently
validated on Corridor02 or a short real-vehicle run.

## Post-hoc strict-ablation correction

A later `1.0x` pointcloud OFF run used the same executable and input chain as
this ON run. It showed that occupied components were already `107` with polar
filtering disabled, so the earlier `130 -> 107` comparison against the `/scan`
baseline cannot be attributed to polar filtering alone.

The isolated ON-versus-OFF effects are: longest wall `26.052 -> 27.004 m`,
median wall residual `0.1194 -> 0.1074 m`, median heading p95
`3.915 -> 2.775 deg`, and worst heading p95 `8.610 -> 4.826 deg`. Worst wall
residual changes `0.1560 -> 0.1683 m`, while the main-corridor spacing standard
deviation changes about `0.0175 -> 0.0433 m`. The corrected decision is that
polar ON passes repeat03 isolation but remains a cross-run candidate rather
than a deployment-approved default. See
`paper_exp/corridor_repeat03_pointcloud_off_realtime_20260811_rerun/THREE_WAY_ABLATION_REPORT.md`.
