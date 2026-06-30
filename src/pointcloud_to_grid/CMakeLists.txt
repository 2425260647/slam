cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_to_grid)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pcd_to_nav_map.py
  scripts/crop_roi_map.py
  scripts/manual_wall_connect_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
