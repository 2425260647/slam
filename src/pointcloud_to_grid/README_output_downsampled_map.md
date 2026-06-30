# output_downsampled.pcd to AMCL Map

This package converts the A-LOAM accumulated point cloud generated from
`corridor_99.bag` into a ROS `map_server` compatible 2D occupancy grid map.

## Input Data

Expected project-level files:

```text
F:\小论文\corridor_99.bag
F:\小论文\output_downsampled.pcd
```

The converter reads `output_downsampled.pcd` directly. The bag file is useful for
rebuilding or validating the point cloud, but the final AMCL map is generated
from the PCD.

## Recommended Command

The converter is written for Python 2.7, matching Ubuntu 18.04 and ROS Melodic.

On this Windows machine, the project-local Python 2.7 command is:

```powershell
F:\小论文\tools\Python27\python.exe F:\小论文\scout_0_ws\src\pointcloud_to_grid\scripts\pcd_to_nav_map.py `
  --pcd F:\小论文\output_downsampled.pcd `
  --output F:\小论文\scout_0_ws\src\my_navigation\maps\output_map
```

On the Ubuntu Scout workspace, put `output_downsampled.pcd` at the workspace
root, then run:

```bash
cd ~/test_ws
source devel/setup.bash
roslaunch pointcloud_to_grid generate_output_map.launch \
  pcd_path:=$(pwd)/output_downsampled.pcd \
  output_prefix:=$(rospack find my_navigation)/maps/output_map
```

The command writes:

```text
src/my_navigation/maps/output_map.pgm
src/my_navigation/maps/output_map.yaml
```

Use `output_map.yaml` in `map_server` if you want AMCL to localize on this
generated map.

## ROI Map for the Task Area

For the current corridor experiment, the robot only runs inside the marked task
area. Generate a smaller task-level global map from `output_map_smooth`:

```bash
rosrun pointcloud_to_grid crop_roi_map.py \
  --input-pgm $(rospack find my_navigation)/maps/output_map_smooth.pgm \
  --input-yaml $(rospack find my_navigation)/maps/output_map_smooth.yaml \
  --output-prefix $(rospack find my_navigation)/maps/roi_map \
  --x-min 0 --x-max 250 \
  --y-min 500 --y-max 850 \
  --min-obstacle-area 8 \
  --free-expand-cells 2 \
  --obstacle-keepout-cells 2 \
  --fill-hole-area 80
```

This writes:

```text
src/my_navigation/maps/roi_map.pgm
src/my_navigation/maps/roi_map.yaml
```

Use `roi_map.yaml` for the red-box task area in `map_server`.

## Algorithm

```text
output_downsampled.pcd
  -> decode PCD binary_compressed fields
  -> estimate ground height from the lower z percentile
  -> collect low-height points as free-space evidence
  -> collect stable points above the ground as wall/obstacle evidence
  -> vote points into 2D grid cells
  -> remove isolated occupied dots
  -> lightly inflate obstacle and free cells
  -> save ROS map_server PGM/YAML
```

## Important Parameters

`resolution`: map resolution in meters per cell. Default is `0.05`.

`z_ground_percentile`: percentile used to estimate ground height.

`z_ground_band`: points close to the estimated ground are treated as free-space
evidence.

`z_obstacle_min`: minimum height above the estimated ground for wall/obstacle
evidence.

`z_obstacle_max`: maximum height above the estimated ground. Higher points are
ignored to avoid ceiling or high noise.

`min_points_per_cell`: minimum votes required before a cell is marked occupied
or free.

`obstacle_inflate_cells`: small raster inflation for AMCL-visible structure.
Keep this low because navigation costmaps should handle safety inflation.

## Quick AMCL Check

```bash
rosrun map_server map_server $(rospack find my_navigation)/maps/output_map.yaml
rviz
```

In RViz, add a Map display on `/map`.

Expected colors:

```text
black: occupied wall or obstacle
white: known free evidence
gray: unknown area
```
