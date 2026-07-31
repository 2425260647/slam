#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace {

class LivoxCustomMsgToPointCloud2 {
 public:
  LivoxCustomMsgToPointCloud2() {
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("input_topic", input_topic_,
                                  "/livox/mid360/lidar");
    private_nh.param<std::string>("output_topic", output_topic_,
                                  "/velodyne_points");
    private_nh.param<std::string>("output_frame", output_frame_, "");
    private_nh.param<int>("queue_size", queue_size_, 5);

    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 5);
    subscriber_ = nh_.subscribe(input_topic_, queue_size_,
                                 &LivoxCustomMsgToPointCloud2::callback, this);

    ROS_INFO_STREAM("Converting " << input_topic_ << " (livox_ros_driver2/CustomMsg) to "
                    << output_topic_ << " (sensor_msgs/PointCloud2), preserving the input frame"
                    << (output_frame_.empty() ? "" : " and using output_frame=" + output_frame_));
  }

 private:
  void callback(const livox_ros_driver2::CustomMsg::ConstPtr& message) {
    const std::size_t point_count = message->points.size();
    if (point_count == 0) {
      ROS_WARN_THROTTLE(5.0, "Received an empty MID-360 CustomMsg");
      return;
    }

    sensor_msgs::PointCloud2 cloud;
    cloud.header = message->header;
    if (!output_frame_.empty()) {
      cloud.header.frame_id = output_frame_;
    }
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        6, "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "ring", 1, sensor_msgs::PointField::UINT16,
        "time", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(point_count);

    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity_it(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> ring_it(cloud, "ring");
    sensor_msgs::PointCloud2Iterator<float> time_it(cloud, "time");

    for (const auto& point : message->points) {
      *x_it = point.x;
      *y_it = point.y;
      *z_it = point.z;
      *intensity_it = static_cast<float>(point.reflectivity);
      *ring_it = static_cast<uint16_t>(point.line);
      *time_it = static_cast<float>(point.offset_time) * 1.0e-9f;
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        cloud.is_dense = false;
      }
      ++x_it;
      ++y_it;
      ++z_it;
      ++intensity_it;
      ++ring_it;
      ++time_it;
    }

    cloud.width = static_cast<uint32_t>(point_count);
    cloud.row_step = cloud.point_step * cloud.width;
    publisher_.publish(cloud);

    ++published_messages_;
    if (published_messages_ == 1 || published_messages_ % 250 == 0) {
      ROS_INFO_STREAM("Published " << published_messages_ << " point clouds ("
                      << cloud.width << " points, stamp=" << cloud.header.stamp
                      << ", frame=" << cloud.header.frame_id << ")");
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  int queue_size_;
  std::size_t published_messages_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_custommsg_to_pointcloud2");
  LivoxCustomMsgToPointCloud2 converter;
  ros::spin();
  return 0;
}
