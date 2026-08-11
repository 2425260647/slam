# repeat03 strict pointcloud polar ablation

Date: 2026-08-11

## Question

Does nearest-return polar filtering improve the repeat03 map when the sensor
input, executable, playback rate, and all non-polar parameters are held fixed?

## Compared runs

| Run | Global SLAM input | Polar insertion | Playback |
|---|---|---|---:|
| Stable deployment baseline | `/scan` | Off | `1.0x` |
| Strict control | `/velodyne_points` | Off | `1.0x` |
| Strict candidate | `/velodyne_points` | On, `0.125 deg` | `1.0x` |

The strict OFF and ON runs used the same Cartographer executable
`b134df9bbb9ea83b29b4172247ddebfae5d3e1a52ba90354f3f8dfb980e86b8c`.
Their Lua configurations differ only by enabling the polar filter and setting
its angular resolution. Both completed the full `578 s` bag in `579.80 s`.

## Runtime integrity

| Runtime item | Pointcloud OFF | Pointcloud ON |
|---|---:|---:|
| `/map` messages | 601 | 601 |
| `/local_occupancy_grid` | 5758 | 5784 |
| `/local_occupancy_grid_map` | 5756 | 5782 |
| `/tf` messages | 89623 | 89569 |
| Fatal/check failure | 0 | 0 |
| Earlier-point warning lines | 174 | 174 |
| Earlier points cropped | 727 | 727 |

The identical time-cropping counts show that the map difference is not caused
by different point-time handling or playback load. The initial sandboxed OFF
launch failed before bag playback because ROS master could not open its XML-RPC
socket; its directory is retained as an invalid startup audit and is excluded.

## Fixed structural proxies

These metrics use one frozen extraction configuration. They are occupancy-map
structure proxies, not surveyed wall truth and not ATE/RPE.

| Metric | Stable `/scan` | Pointcloud OFF | Pointcloud ON |
|---|---:|---:|---:|
| Occupied components | 130 | 107 | 107 |
| Occupied cells | 9107 | 11107 | 10358 |
| Longest Hough wall | 24.950 m | 26.052 m | 27.004 m |
| Median wall p95 residual | 0.1134 m | 0.1194 m | 0.1074 m |
| Worst wall p95 residual | 0.1595 m | 0.1560 m | 0.1683 m |
| Median local heading p95 | 3.948 deg | 3.915 deg | 2.775 deg |
| Worst local heading p95 | 6.326 deg | 8.610 deg | 4.826 deg |
| Median parallel-spacing std. | 0.0283 m | 0.0063 m | 0.0127 m |
| Worst parallel-spacing std. | 0.0316 m | 0.0175 m | 0.0778 m |

Against strict pointcloud OFF, polar ON improves longest detected wall by
`3.7%`, median wall residual by `10.0%`, median local heading variation by
`29.1%`, and worst local heading variation by `44.0%`. It worsens worst wall
residual by `7.9%` and the spacing proxy.

The worst ON spacing pair has a mean spacing of about `8.43 m` and is not the
main corridor wall pair. For the detected main corridor pair, spacing standard
deviation changes from about `0.0175 m` OFF to `0.0433 m` ON. This is an
absolute increase of about `0.026 m`, below one `0.05 m` map cell, but it is a
real regression and must remain visible in the decision.

## Visual review

All three maps preserve the long-corridor topology. Pointcloud OFF produces
thicker, more connected occupied traces but retains stronger local wall
waviness. Polar ON reduces much of that waviness and produces longer coherent
wall evidence, while removing about `6.7%` of OFF occupied cells. It does not
visibly close the traversable main corridor, but some branch and junction
evidence is thinner.

## Decision

Polar ON passes the repeat03 strict isolation test and is promoted to a
same-vehicle cross-run candidate. It is not approved as the default real-car
configuration yet because:

1. spacing and worst residual do not improve uniformly;
2. the Corridor02 stress regression was negative;
3. repeat01/repeat02 were used in historical parameter experiments and are
   regression data, not untouched final tests;
4. none of the three bags contains continuous external pose ground truth.

The next valid gate is to run the frozen pointcloud OFF/ON pair on one other
same-car recording without changing any parameter, followed by a newly
recorded untouched bag and an online vehicle test. The stable `/scan` entry
must remain the deployment default until those gates pass.

## Artifacts

- Strict OFF map and logs: this directory
- Strict ON result:
  `paper_exp/corridor_repeat03_pointcloud_polar_fine_realtime_20260811/`
- Stable `/scan` map:
  `paper_exp/corridor_repeat03_results/corridor_repeat_03_final_tuned_map.*`
- Full fixed-metric data: `three_way_straightness.json`
