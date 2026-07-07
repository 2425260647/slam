# Scout Mini Gazebo simulation

This workspace uses the existing `scout_description` package from `src/scout_ros-master` to avoid duplicate robot model packages.

## Files to copy

Copy the whole directory:

```bash
src/ugv_gazebo_sim-master
```

The target workspace must also keep the existing project packages used by this simulation:

```bash
src/scout_ros-master/scout_description
src/my_navigation
src/pointcloud_to_grid
src/cartographer
src/cartographer_ros
```

## Build

```bash
cd ~/slam_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
```

## Launch only Gazebo

```bash
roslaunch scout_gazebo_sim scout_mini_diff_drive_empty_world.launch
```

Drive test:

```bash
rostopic pub /scout_mini_velocity_controller/cmd_vel geometry_msgs/Twist "linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

## Launch Gazebo + Cartographer 2D

```bash
roslaunch scout_gazebo_sim scout_mini_cartographer_2d.launch
```

Expected topics:

```bash
/velodyne_points_raw
/velodyne_points
/scan
/map
/submap_list
/tf
```

The simulated LS 16-line lidar is mounted at `base_link -> laser_link` with yaw `-1.5708`, matching the real vehicle setting where lidar zero degree points to the robot right side.

`/velodyne_points_raw` is the Gazebo block laser output in the legacy `sensor_msgs/PointCloud` format. The launch starts `pointcloud_to_pointcloud2.py` to republish it as `/velodyne_points` in `sensor_msgs/PointCloud2`, keeping the same input type used by the current Cartographer pipeline.

The default Gazebo world is `clearpath_playpen.world`. Lidar ray visualization is disabled in Gazebo to keep the GUI readable; lidar data is still published normally.
