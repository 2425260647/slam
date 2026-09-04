# corridor_repeat_01 smoke C1

This is a short real-bag pipeline smoke test, not an accuracy result. It uses
the first 20 s of `bags/corridor_repeat_01.bag` with `rosbag play --clock`.

Configuration:

- C1 confidence projection with `forward_selected_scans=false`;
- `require_ring=true`, projection range capped at 30 m;
- Cartographer configuration `my_navigation/config/cartographer_scout_2d.lua`;
- `/odom`, `/tf`, `/tf_static` replayed from the bag;
- playback rate and command line are stored in `logs/command.txt`.

Acceptance checks for this run are topic type/frequency, timestamp quality
matching, node survival, and absence of malformed PointCloud2 errors. No ATE,
RPE, or improvement percentage is reported because this real bag has no
continuous external pose ground truth.
