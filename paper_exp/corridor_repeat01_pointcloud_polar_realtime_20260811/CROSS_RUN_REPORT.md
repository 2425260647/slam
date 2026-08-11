# Corridor Repeat01 Pointcloud Polar Cross-Run Regression

## 1. Purpose and experimental boundary

This experiment checks whether the `0.125 deg` polar submap-insertion filter
benefit observed on `corridor_repeat_03.bag` repeats on another run recorded by
the same robot, LiDAR and environment.

`corridor_repeat_01.bag` was used in earlier project experiments, so this is a
same-domain cross-run regression, not a previously unseen final test. The bag
does not contain continuous external ground truth. All map geometry values
below are fixed-parameter engineering proxies, not ATE, RPE or surveyed wall
error.

The comparison is isolated as follows:

- Both runs consume `/velodyne_points` directly and use odometry.
- Both runs use the same Cartographer binary, whose SHA256 is
  `b134df9bbb9ea83b29b4172247ddebfae5d3e1a52ba90354f3f8dfb980e86b8c`.
- Both runs replay the complete bag at `1.0x`.
- The ON configuration includes the OFF configuration and changes only
  `submap_insertion_polar_filter_enabled=true` and
  `submap_insertion_polar_angular_resolution=0.0021816616 rad`.

## 2. Run completeness

| Check | Pointcloud OFF | Polar ON |
|---|---:|---:|
| Bag coverage | 565 s | 565 s |
| Replay wall time | 566.46 s | 566.47 s |
| `/map` messages | 587 | 587 |
| `/local_occupancy_grid` messages | 5655 | 5655 |
| `/local_occupancy_grid_map` messages | 5653 | 5653 |
| `/tf` messages | 87232 | 87382 |
| Time-crop warnings / points | 300 / 740 | 300 / 740 |
| Fatal/check/abort/segfault | 0 | 0 |
| Final map | 1618 x 341 | 1619 x 344 |

The equal bag coverage, runtime, map/local-map counts and time-crop counts make
the two results operationally comparable. The PGM hashes differ as expected:

- OFF: `a8d93592dfc95030254cf96abafd95b9abce41a706721c6c09b33e47eccdf38e`
- ON: `5fc3babae02b645247f67dd43f013e2838454ec1b08c353a58edda091d8348e5`

## 3. Frozen structural metrics

| Metric | Pointcloud OFF | Polar ON | ON trend |
|---|---:|---:|---:|
| Occupied components | 99 | 116 | 17.2% worse |
| Occupied cells | 10876 | 9969 | 8.3% fewer |
| Longest Hough wall | 22.223 m | 26.332 m | 18.5% better |
| Wall P95 residual, median | 0.1111 m | 0.0896 m | 19.3% better |
| Wall P95 residual, worst | 0.1902 m | 0.2326 m | 22.3% worse |
| Local heading P95, median | 3.159 deg | 3.054 deg | 3.3% better |
| Local heading P95, worst | 11.467 deg | 5.000 deg | 56.4% better |
| Parallel-wall spacing std, median | 0.0316 m | 0.0080 m | 74.8% better |
| Parallel-wall spacing std, worst | 0.0721 m | 0.0561 m | 22.3% better |

The main positive result is consistent with repeat03: the polar filter improves
the dominant long-wall straightness and heading stability. It is not a uniform
win. The worst residual regresses and must not be hidden.

## 4. Fragmentation and worst-region audit

The component count increase does not mean that a comparable mass of new
occupied noise was added:

| Component statistic | Pointcloud OFF | Polar ON |
|---|---:|---:|
| Single-pixel components | 21 | 36 |
| Components with at most 3 pixels | 42 | 51 |
| Cells contained in components with at most 3 pixels | 71 | 72 |
| Cells contained in components with at most 10 pixels | 208 | 190 |

Thus, the additional component count is mainly a topology effect from thinner
occupied lines splitting into smaller pieces; the total area of very small
fragments is effectively unchanged. Nevertheless, thin-wall gaps can matter to
a costmap and remain a deployment risk.

The ON worst residual belongs to an automatically detected `5.308 m` transverse
wall near the right-hand branch/junction, not the dominant long corridor wall.
This explains why median long-wall and heading metrics improve while the single
worst residual worsens. It does not invalidate the negative metric.

## 5. Planning-input proxy

Unknown and occupied cells were both treated as blocked. Free space was eroded
with circular kernels corresponding to conservative `0.25 m` and `0.35 m`
radii. This is a map-connectivity proxy, not a move_base or vehicle test.

| Erosion radius | Pointcloud OFF largest free component | Polar ON largest free component |
|---|---:|---:|
| 0.25 m | 54066 / 54066 cells | 53726 / 53733 cells |
| 0.35 m | 44743 / 44799 cells | 44311 / 44393 cells |

Both maps preserve one dominant navigable free-space component after erosion;
the polar map does not close the main corridor. Actual planning still requires
the real footprint, inflation parameters, localization continuity and online
vehicle validation.

## 6. Decision

The polar filter's principal long-wall benefit repeats across repeat03 and
repeat01, so it remains a valid same-robot deployment candidate. It must not be
made the default yet because:

1. the worst local wall residual regresses;
2. thin-wall fragmentation increases;
3. the Corridor02 stress regression was negative;
4. none of the three bags provides continuous ground truth; and
5. repeat01 is not a previously unseen final test.

Freeze this candidate for a newly recorded, unseen bag and then for an online
Scout Mini mapping/localization test. Default deployment remains the stable
`/scan` path until those two gates pass.

## 7. Reproduction artifacts

- OFF run: `paper_exp/corridor_repeat01_pointcloud_off_realtime_20260811/`
- ON run: `paper_exp/corridor_repeat01_pointcloud_polar_realtime_20260811/`
- Fixed structural records: `straightness_comparison.json`
- Per-map raster records: each run's `map_metrics.json`
- Visual previews: each run's `map_preview.png`
