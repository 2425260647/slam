cmake_minimum_required(VERSION 3.0.2)
project(lightweight_2d_slam)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  gazebo_msgs
  geometry_msgs
  nav_msgs
  rospy
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lightweight_2d_slam_core
  CATKIN_DEPENDS diagnostic_msgs gazebo_msgs geometry_msgs nav_msgs rospy
                 roscpp sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(lightweight_2d_slam_core
  src/occupancy_grid.cpp
  src/pose_graph.cpp
  src/scan_matcher.cpp
  src/submap_2d.cpp
  src/slam_system.cpp
)
target_link_libraries(lightweight_2d_slam_core
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(lightweight_2d_slam_node src/lightweight_2d_slam_node.cpp)
target_link_libraries(lightweight_2d_slam_node
  lightweight_2d_slam_core
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/validate_pose_sources.py
  scripts/validate_square_trajectory.py
  scripts/truth_guided_closed_loop.py
  scripts/odom_to_tf.py
  scripts/tf_pose_publisher.py
  scripts/evaluate_replay_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(lightweight_2d_slam_core_test test/core_test.cpp)
  if(TARGET lightweight_2d_slam_core_test)
    target_link_libraries(lightweight_2d_slam_core_test
      lightweight_2d_slam_core
      ${catkin_LIBRARIES}
      ${CERES_LIBRARIES}
    )
  endif()
endif()

install(TARGETS lightweight_2d_slam_core lightweight_2d_slam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
