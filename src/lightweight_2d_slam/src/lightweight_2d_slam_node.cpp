#include <ros/ros.h>

#include "lightweight_2d_slam/slam_system.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lightweight_2d_slam");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");
  lightweight_2d_slam::SlamSystem system(node_handle, private_node_handle);
  ros::spin();
  return 0;
}

