# corridor_repeat_01 smoke C0

This is a short real-bag baseline projection smoke test using the first 15 s
of `bags/corridor_repeat_01.bag` with `rosbag play --clock`. The launch uses
`use_baseline_projection=true`, `use_confidence=false`, and maps Cartographer
to `/scan`; the command is recorded in `logs/command.txt`.

This run verifies that the baseline `pointcloud_to_laserscan` path receives the
same point cloud, odometry, and TF as C1/C2. It has no continuous external pose
ground truth, so no ATE/RPE or accuracy improvement is reported.

The first launch attempt exposed a condition bug and is retained in the log
directory as a failed run. The corrected run uses the explicit
`use_baseline_projection=true` switch and writes `derived_15s_fixed.bag`.
