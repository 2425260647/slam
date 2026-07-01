cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_to_grid)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pointcloud_projection_optimizer.py
  scripts/pcd_to_nav_map.py
  scripts/compute_map_metrics.py
  scripts/enhance_map.py
  scripts/save_stable_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
