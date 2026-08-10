#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double NormalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct TimedPose {
  ros::Time stamp;
  Pose2D pose;
};

class LivoxDeskewedLaserScan {
 public:
  LivoxDeskewedLaserScan() : private_node_handle_("~") {
    private_node_handle_.param("input_topic", input_topic_,
                               std::string("/livox/mid360/lidar"));
    private_node_handle_.param("odom_topic", odom_topic_,
                               std::string("/odom"));
    private_node_handle_.param("output_topic", output_topic_,
                               std::string("/scan"));
    private_node_handle_.param("output_frame", output_frame_,
                               std::string("base_footprint"));
    private_node_handle_.param("min_height", min_height_, 0.20);
    private_node_handle_.param("max_height", max_height_, 0.60);
    private_node_handle_.param("angle_min", angle_min_, -kPi);
    private_node_handle_.param("angle_max", angle_max_, kPi);
    private_node_handle_.param("angle_increment", angle_increment_,
                               0.0043633);
    private_node_handle_.param("range_min", range_min_, 0.30);
    private_node_handle_.param("range_max", range_max_, 30.0);
    private_node_handle_.param("filter_isolated_returns",
                               filter_isolated_returns_, true);
    private_node_handle_.param("isolation_window_bins",
                               isolation_window_bins_, 3);
    private_node_handle_.param("isolation_max_distance",
                               isolation_max_distance_, 0.30);
    private_node_handle_.param("minimum_valid_bins", minimum_valid_bins_, 30);

    double translation_x = -0.063742232981;
    double translation_y = 1.003639208011;
    double translation_z = 0.417493046931;
    double quaternion_x = -0.014813385023;
    double quaternion_y = 0.255990978057;
    double quaternion_z = -0.028832269454;
    double quaternion_w = 0.966135540706;
    private_node_handle_.param("base_from_lidar_x", translation_x,
                               translation_x);
    private_node_handle_.param("base_from_lidar_y", translation_y,
                               translation_y);
    private_node_handle_.param("base_from_lidar_z", translation_z,
                               translation_z);
    private_node_handle_.param("base_from_lidar_qx", quaternion_x,
                               quaternion_x);
    private_node_handle_.param("base_from_lidar_qy", quaternion_y,
                               quaternion_y);
    private_node_handle_.param("base_from_lidar_qz", quaternion_z,
                               quaternion_z);
    private_node_handle_.param("base_from_lidar_qw", quaternion_w,
                               quaternion_w);
    tf2::Quaternion rotation(quaternion_x, quaternion_y, quaternion_z,
                             quaternion_w);
    rotation.normalize();
    base_from_lidar_.setOrigin(
        tf2::Vector3(translation_x, translation_y, translation_z));
    base_from_lidar_.setRotation(rotation);

    angle_increment_ = std::max(1e-5, angle_increment_);
    angle_max_ = std::max(angle_min_ + angle_increment_, angle_max_);
    range_min_ = std::max(0.0, range_min_);
    range_max_ = std::max(range_min_ + 0.01, range_max_);
    isolation_window_bins_ = std::max(1, isolation_window_bins_);
    isolation_max_distance_ = std::max(0.01, isolation_max_distance_);
    minimum_valid_bins_ = std::max(1, minimum_valid_bins_);

    publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>(
        output_topic_, 5, false);
    odom_subscriber_ = node_handle_.subscribe(
        odom_topic_, 500, &LivoxDeskewedLaserScan::OdomCallback, this);
    cloud_subscriber_ = node_handle_.subscribe(
        input_topic_, 20, &LivoxDeskewedLaserScan::CloudCallback, this);
    ROS_INFO_STREAM("Deskewing " << input_topic_ << " with " << odom_topic_
                    << " into " << output_topic_ << " in " << output_frame_);
  }

 private:
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const auto& position = message->pose.pose.position;
    odom_buffer_.push_back(
        {message->header.stamp,
         {position.x, position.y,
          tf2::getYaw(message->pose.pose.orientation)}});
    while (odom_buffer_.size() > 5000) odom_buffer_.pop_front();
    while (odom_buffer_.size() > 2 &&
           (odom_buffer_.back().stamp - odom_buffer_.front().stamp).toSec() >
               30.0) {
      odom_buffer_.pop_front();
    }
    ProcessPendingClouds();
  }

  void CloudCallback(const livox_ros_driver2::CustomMsg::ConstPtr& message) {
    const auto insertion = std::upper_bound(
        pending_clouds_.begin(), pending_clouds_.end(), message->header.stamp,
        [](const ros::Time& stamp,
           const livox_ros_driver2::CustomMsg::ConstPtr& queued) {
          return stamp < queued->header.stamp;
        });
    pending_clouds_.insert(insertion, message);
    while (pending_clouds_.size() > 20) {
      pending_clouds_.pop_front();
      ++dropped_clouds_;
      ROS_ERROR_THROTTLE(2.0, "Deskew input queue overflow");
    }
    ProcessPendingClouds();
  }

  bool LookupOdom(const ros::Time& stamp, Pose2D* pose) const {
    if (odom_buffer_.empty()) return false;
    if (stamp <= odom_buffer_.front().stamp) {
      if ((odom_buffer_.front().stamp - stamp).toSec() > 0.10) return false;
      *pose = odom_buffer_.front().pose;
      return true;
    }
    if (stamp > odom_buffer_.back().stamp) return false;
    const auto upper = std::lower_bound(
        odom_buffer_.begin(), odom_buffer_.end(), stamp,
        [](const TimedPose& value, const ros::Time& target) {
          return value.stamp < target;
        });
    if (upper == odom_buffer_.begin() || upper == odom_buffer_.end()) {
      return false;
    }
    const auto lower = std::prev(upper);
    const double duration = (upper->stamp - lower->stamp).toSec();
    if (duration <= 1e-9) {
      *pose = upper->pose;
      return true;
    }
    const double ratio = (stamp - lower->stamp).toSec() / duration;
    pose->x = lower->pose.x + ratio * (upper->pose.x - lower->pose.x);
    pose->y = lower->pose.y + ratio * (upper->pose.y - lower->pose.y);
    pose->yaw = NormalizeAngle(
        lower->pose.yaw + ratio *
            NormalizeAngle(upper->pose.yaw - lower->pose.yaw));
    return true;
  }

  void ProcessPendingClouds() {
    while (!pending_clouds_.empty()) {
      const auto& message = pending_clouds_.front();
      if (message->points.empty()) {
        pending_clouds_.pop_front();
        continue;
      }
      const uint32_t maximum_offset = std::max_element(
          message->points.begin(), message->points.end(),
          [](const livox_ros_driver2::CustomPoint& lhs,
             const livox_ros_driver2::CustomPoint& rhs) {
            return lhs.offset_time < rhs.offset_time;
          })->offset_time;
      const ros::Time reference_stamp =
          message->header.stamp + ros::Duration(maximum_offset * 1.0e-9);
      if (odom_buffer_.empty() ||
          odom_buffer_.back().stamp < reference_stamp) {
        return;
      }
      Pose2D reference_pose;
      if (!LookupOdom(reference_stamp, &reference_pose)) {
        pending_clouds_.pop_front();
        ++dropped_clouds_;
        ROS_WARN_THROTTLE(2.0, "No odometry covering a MID-360 scan");
        continue;
      }
      Project(*message, reference_stamp, reference_pose);
      pending_clouds_.pop_front();
    }
  }

  void Project(const livox_ros_driver2::CustomMsg& message,
               const ros::Time& reference_stamp,
               const Pose2D& reference_pose) {
    const int bin_count = std::max(
        1, static_cast<int>(std::ceil(
               (angle_max_ - angle_min_) / angle_increment_)));
    std::vector<float> ranges(static_cast<std::size_t>(bin_count),
                              std::numeric_limits<float>::infinity());
    const double reference_cos = std::cos(reference_pose.yaw);
    const double reference_sin = std::sin(reference_pose.yaw);

    for (const auto& point : message.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      Pose2D point_pose;
      const ros::Time point_stamp =
          message.header.stamp + ros::Duration(point.offset_time * 1.0e-9);
      if (!LookupOdom(point_stamp, &point_pose)) continue;

      const tf2::Vector3 base_point =
          base_from_lidar_ * tf2::Vector3(point.x, point.y, point.z);
      if (base_point.z() < min_height_ || base_point.z() > max_height_) {
        continue;
      }
      const double point_cos = std::cos(point_pose.yaw);
      const double point_sin = std::sin(point_pose.yaw);
      const double world_x = point_pose.x +
          point_cos * base_point.x() - point_sin * base_point.y();
      const double world_y = point_pose.y +
          point_sin * base_point.x() + point_cos * base_point.y();
      const double delta_x = world_x - reference_pose.x;
      const double delta_y = world_y - reference_pose.y;
      const double x = reference_cos * delta_x + reference_sin * delta_y;
      const double y = -reference_sin * delta_x + reference_cos * delta_y;
      const double range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_) continue;
      const double angle = std::atan2(y, x);
      const int bin = static_cast<int>(
          std::floor((angle - angle_min_) / angle_increment_));
      if (bin < 0 || bin >= bin_count) continue;
      float& stored = ranges[static_cast<std::size_t>(bin)];
      stored = std::min(stored, static_cast<float>(range));
    }

    if (filter_isolated_returns_) FilterIsolatedReturns(&ranges);
    const int valid_bins = static_cast<int>(std::count_if(
        ranges.begin(), ranges.end(),
        [](float range) { return std::isfinite(range); }));
    if (valid_bins < minimum_valid_bins_) {
      ++dropped_clouds_;
      ROS_WARN_THROTTLE(2.0, "Projected MID-360 scan has too few valid bins");
      return;
    }

    sensor_msgs::LaserScan scan;
    scan.header = message.header;
    scan.header.stamp = reference_stamp;
    scan.header.frame_id = output_frame_;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.angle_max = static_cast<float>(
        angle_min_ + (bin_count - 1) * angle_increment_);
    scan.time_increment = 0.0f;
    scan.scan_time = static_cast<float>(
        (reference_stamp - message.header.stamp).toSec());
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges = std::move(ranges);
    publisher_.publish(scan);
    ++published_scans_;
    if (published_scans_ == 1 || published_scans_ % 250 == 0) {
      ROS_INFO_STREAM("Published " << published_scans_
                      << " deskewed scans; valid_bins=" << valid_bins
                      << " duration=" << scan.scan_time
                      << " dropped=" << dropped_clouds_);
    }
  }

  void FilterIsolatedReturns(std::vector<float>* ranges) const {
    const std::vector<float> input = *ranges;
    for (int index = 0; index < static_cast<int>(input.size()); ++index) {
      const double range = input[static_cast<std::size_t>(index)];
      if (!std::isfinite(range)) continue;
      bool supported = false;
      for (int offset = -isolation_window_bins_;
           offset <= isolation_window_bins_; ++offset) {
        if (offset == 0) continue;
        const int neighbor = index + offset;
        if (neighbor < 0 || neighbor >= static_cast<int>(input.size())) {
          continue;
        }
        const double neighbor_range = input[static_cast<std::size_t>(neighbor)];
        if (!std::isfinite(neighbor_range)) continue;
        const double angular_difference = offset * angle_increment_;
        const double squared_distance =
            range * range + neighbor_range * neighbor_range -
            2.0 * range * neighbor_range * std::cos(angular_difference);
        if (squared_distance <=
            isolation_max_distance_ * isolation_max_distance_) {
          supported = true;
          break;
        }
      }
      if (!supported) {
        (*ranges)[static_cast<std::size_t>(index)] =
            std::numeric_limits<float>::infinity();
      }
    }
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher publisher_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber cloud_subscriber_;
  std::string input_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double min_height_ = 0.20;
  double max_height_ = 0.60;
  double angle_min_ = -kPi;
  double angle_max_ = kPi;
  double angle_increment_ = 0.0043633;
  double range_min_ = 0.30;
  double range_max_ = 30.0;
  bool filter_isolated_returns_ = true;
  int isolation_window_bins_ = 3;
  double isolation_max_distance_ = 0.30;
  int minimum_valid_bins_ = 30;
  tf2::Transform base_from_lidar_;
  std::deque<TimedPose> odom_buffer_;
  std::deque<livox_ros_driver2::CustomMsg::ConstPtr> pending_clouds_;
  std::size_t published_scans_ = 0;
  std::size_t dropped_clouds_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_deskewed_laserscan");
  LivoxDeskewedLaserScan node;
  ros::spin();
  return 0;
}
