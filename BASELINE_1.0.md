# Scout Mini Total Baseline 1.0

This tag identifies the integrated Scout Mini stack preserved in this
workspace as `baseline-1.0`.

## Included system

- Cartographer 2D mapping and localization with `map -> odom -> base_link`
  TF and `local_grid_mapper` local occupancy output.
- Frontier exploration and `move_base`/TEB path planning through
  `nav_driver`, including safety stop and the single-writer `cmd_vel_arbiter`.
- Optional Ultralytics mouse detector with `/object_direction`, `/object_area`
  and the no-depth visual-servo state machine.  Visual control remains
  disabled by default and must be explicitly enabled for a vehicle test.
- `lidar_adaptive` research projection/scan-selection package and its launch
  and test scaffolding.  It is not inserted into the standard mapping launch
  unless explicitly selected.

## Runtime entry points

Standard mapping/localization:

```bash
roslaunch my_navigation real_scout_mapping_local_grid.launch
```

Integrated autonomous navigation with vision enabled:

```bash
roslaunch my_navigation real_scout_autonomous_navigation.launch \
  enable_object_detection:=true \
  object_detection_python:="$CONDA_PREFIX/bin/python3"
```

The integrated entry point starts one detector and sets both detector startup
and `navigation_driver` visual-servo enablement.  Do not start a second
detector publishing the same topics.

## Version and limits

- Git reference: `baseline-1.0` (annotated tag on the total integrated commit).
- Package versions retain their component compatibility values (`3.4.1` for
  navigation/mapping and the existing detector package version); `1.0` is the
  total-system baseline identifier, not a ROS package API version.
- The camera has no depth in the production configuration.  Visual approach
  therefore uses bearing and image-area ratio; a `0.033` area ratio is the
  configured stop criterion and is not a metric distance estimate.
- Real-vehicle motion, detector accuracy and planner reachability still need
  to be validated on the target machine after rebuilding and sourcing its ROS
  install space.
