cmake_minimum_required(VERSION 3.0.2)
project(lidar_adaptive)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

add_message_files(
  FILES
    ScanQuality.msg
    ScanSelection.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime nav_msgs roscpp sensor_msgs std_msgs tf2 tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(confidence_projection_node
  src/confidence_projection_node.cpp
)
add_dependencies(confidence_projection_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(confidence_projection_node ${catkin_LIBRARIES})

add_executable(adaptive_scan_selector_node
  src/adaptive_scan_selector_node.cpp
)
add_dependencies(adaptive_scan_selector_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(adaptive_scan_selector_node ${catkin_LIBRARIES})

install(TARGETS confidence_projection_node adaptive_scan_selector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config doc msg worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES README.md IMPLEMENTATION_PLAN.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/analyze_scan_quality.py
  scripts/analyze_experiment_bag.py
  scripts/compute_gazebo_trajectory_metrics.py
  scripts/aggregate_experiment_metrics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/record_lidar_experiment.sh
  scripts/backfill_experiment_metadata.sh
  scripts/run_gazebo_case.sh
  scripts/run_bag_case.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_launch_structure.py)
endif()
