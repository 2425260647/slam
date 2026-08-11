# Gazebo large common-loop comparison

## Scope

This is a controlled ROS1 Gazebo simulation comparison of the independent
`lightweight_2d_slam` system against Cartographer clean and GMapping. It is an
engineering baseline for the ordinary-scene system; it is not a real-dataset
or degeneracy-robustness result.

## Frozen input

- Bag: `../common_inputs.bag`
- Duration: 193.21 s
- `/scan`: 1930 messages
- `/scout_mini_velocity_controller/odom`: 9659 messages
- `/gazebo/model_states`: 19315 messages
- `/tf`: 9651 messages; `/tf_static`: 1 message
- IMU, Avia and camera topics were not replayed or subscribed.
- Each algorithm used the same bag and the same scan/odometry/TF inputs.
- Replay used `rosbag play --clock` while excluding the bag's stored `/clock`;
  this avoids the previously invalid dual-clock run.
- Gazebo truth was recorded and used only by the independent evaluator, never
  by any SLAM node.

The route returned close to its starting pose in Gazebo (translation closure
error 0.179 m, yaw closure error 0.008 rad). This is a route-quality check,
not an external motion-capture guarantee.

## Pose comparison

The evaluator reports online start-aligned SE(2) errors against the replayed
Gazebo model pose. This is explicitly **not** standard real-world ATE/RPE.

| Algorithm | Translation RMSE (m) | Yaw RMSE (rad) | Final translation (m) | Final yaw (rad) |
| --- | ---: | ---: | ---: | ---: |
| Lightweight | 0.1276 | 0.0240 | 0.0963 | 0.0078 |
| Cartographer clean | 0.0481 | 0.0252 | 0.0252 | 0.0008 |
| GMapping | 0.0463 | 0.0225 | 0.0156 | 0.0041 |

On this ordinary Gazebo route, the lightweight system is not more accurate:
its translation RMSE is about 2.65 times Cartographer's and 2.75 times
GMapping's. Its yaw RMSE is close to Cartographer (slightly lower) but higher
than GMapping. This result must be reported as a limitation, not hidden.

## Runtime and safety

All three replays used the same 193.21 s input and completed without launch-log
errors. The measured wall-clock durations were:

| Algorithm | Wall time (s) | Wall/input ratio |
| --- | ---: | ---: |
| Lightweight | 194.21 | 1.0052 |
| Cartographer clean | 194.33 | 1.0058 |
| GMapping | 194.29 | 1.0056 |

The lightweight diagnostic replay reported zero dropped scans, mean/P95/max
scan processing times of 15.52/22.95/36.50 ms at a 10 Hz scan stream, 635
keyframes, 13 submaps and two accepted loop constraints. The two loop
constraints were independently confirmed and did not produce a measurable
accuracy gain on this route.

## Map audit

All maps are non-empty 0.05 m occupancy grids. The audit is in
`map_audit.json` and was computed from occupied/free/unknown PGM cells.

| Algorithm | Image (px) | Occupied cells | Occupied components | Largest component ratio |
| --- | ---: | ---: | ---: | ---: |
| Lightweight | 461 x 456 | 4926 | 163 | 7.88% |
| Cartographer clean | 438 x 432 | 6387 | 69 | 9.63% |
| GMapping | 1184 x 1184 | 6831 | 90 | 9.08% |

The map canvases and unknown extents differ, especially because GMapping uses a
fixed 60 m canvas. Therefore occupied-cell counts and component counts are
only structural diagnostics; they are not a fair map-quality ranking. Existing
visual inspection found no new catastrophic wall duplication in the repaired
lightweight loop-on output, but the higher component count indicates that its
map structure still needs improvement before it can be described as globally
consistent at mature-baseline quality.

## Validity and next gate

This experiment establishes that the self-built system can publish a non-empty
global grid, tracked pose and TF in real time on an ordinary Gazebo route, and
that its asynchronous loop-closure implementation is safe on this input. It
does **not** establish superiority over Cartographer/GMapping, does not support
a long-corridor or slip claim, and does not provide real-world ATE/RPE evidence.

The next admissible research step is to freeze this ordinary-scene baseline,
then add the degradation-aware front end as a separate controlled variant and
test it on an independently held-out degradation sequence. The baseline must
remain reproducible and its current accuracy deficit must be acknowledged in
the paper.
