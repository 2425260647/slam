#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

struct FieldInfo {
  bool present = false;
  uint32_t offset = 0;
  uint8_t datatype = 0;
};

template <typename T>
T ReadValue(const sensor_msgs::PointCloud2& cloud, const std::size_t offset) {
  T value{};
  std::memcpy(&value, cloud.data.data() + offset, sizeof(T));
  return value;
}

FieldInfo FindField(const sensor_msgs::PointCloud2& cloud,
                    const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) {
      return {true, field.offset, field.datatype};
    }
  }
  return {};
}

class VelodyneDeskewedLaserScan {
 public:
  VelodyneDeskewedLaserScan()
      : private_node_handle_("~"), tf_buffer_(), tf_listener_(tf_buffer_) {
    private_node_handle_.param("input_topic", input_topic_,
                               std::string("/velodyne_points"));
    private_node_handle_.param("odom_topic", odom_topic_,
                               std::string("/odom"));
    private_node_handle_.param("output_topic", output_topic_,
                               std::string("/scan"));
    private_node_handle_.param("base_frame", base_frame_,
                               std::string("base_link"));
    private_node_handle_.param("output_frame", output_frame_,
                               std::string("laser_link"));
    private_node_handle_.param("ring_min", ring_min_, 8);
    private_node_handle_.param("ring_max", ring_max_, 9);
    private_node_handle_.param("angle_min", angle_min_, -kPi);
    private_node_handle_.param("angle_max", angle_max_, kPi);
    private_node_handle_.param("angle_increment", angle_increment_,
                               0.0043633231);
    private_node_handle_.param("range_min", range_min_, 0.30);
    private_node_handle_.param("range_max", range_max_, 30.0);
    private_node_handle_.param("filter_isolated_returns",
                               filter_isolated_returns_, true);
    private_node_handle_.param("isolation_window_bins",
                               isolation_window_bins_, 2);
    private_node_handle_.param("isolation_max_distance",
                               isolation_max_distance_, 0.30);
    private_node_handle_.param("minimum_valid_bins", minimum_valid_bins_, 60);
    private_node_handle_.param("odom_tolerance_sec", odom_tolerance_sec_,
                               0.10);

    ring_min_ = std::max(0, ring_min_);
    ring_max_ = std::max(ring_min_, ring_max_);
    angle_increment_ = std::max(1.e-5, angle_increment_);
    angle_max_ = std::max(angle_min_ + angle_increment_, angle_max_);
    range_min_ = std::max(0.0, range_min_);
    range_max_ = std::max(range_min_ + 0.01, range_max_);
    isolation_window_bins_ = std::max(1, isolation_window_bins_);
    isolation_max_distance_ = std::max(0.01, isolation_max_distance_);
    minimum_valid_bins_ = std::max(1, minimum_valid_bins_);

    publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>(
        output_topic_, 5, false);
    odom_subscriber_ = node_handle_.subscribe(
        odom_topic_, 1000, &VelodyneDeskewedLaserScan::OdomCallback, this);
    cloud_subscriber_ = node_handle_.subscribe(
        input_topic_, 10, &VelodyneDeskewedLaserScan::CloudCallback, this);
    ROS_INFO_STREAM("Deskewing " << input_topic_ << " with " << odom_topic_
                    << "; rings " << ring_min_ << ".." << ring_max_
                    << " -> " << output_topic_);
  }

 private:
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const auto& position = message->pose.pose.position;
    odom_buffer_.push_back(
        {message->header.stamp,
         {position.x, position.y,
          std::atan2(2.0 * (message->pose.pose.orientation.w *
                            message->pose.pose.orientation.z),
                     1.0 - 2.0 * (message->pose.pose.orientation.z *
                                  message->pose.pose.orientation.z))}});
    while (odom_buffer_.size() > 6000) odom_buffer_.pop_front();
    while (odom_buffer_.size() > 2 &&
           (odom_buffer_.back().stamp - odom_buffer_.front().stamp).toSec() >
               40.0) {
      odom_buffer_.pop_front();
    }
    ProcessPendingClouds();
  }

  void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    pending_clouds_.push_back(message);
    while (pending_clouds_.size() > 20) {
      pending_clouds_.pop_front();
      ++dropped_clouds_;
      ROS_WARN_THROTTLE(2.0, "Deskew point cloud queue overflow");
    }
    ProcessPendingClouds();
  }

  bool LookupOdom(const ros::Time& stamp, Pose2D* pose) const {
    if (odom_buffer_.empty()) return false;
    if (stamp <= odom_buffer_.front().stamp) {
      if ((odom_buffer_.front().stamp - stamp).toSec() > odom_tolerance_sec_) {
        return false;
      }
      *pose = odom_buffer_.front().pose;
      return true;
    }
    if (stamp >= odom_buffer_.back().stamp) {
      if ((stamp - odom_buffer_.back().stamp).toSec() > odom_tolerance_sec_) {
        return false;
      }
      *pose = odom_buffer_.back().pose;
      return true;
    }
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
    if (duration <= 1.e-9) {
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

  bool PointStamp(const double raw_time, const ros::Time& header,
                  ros::Time* stamp) const {
    if (!std::isfinite(raw_time)) return false;
    if (std::abs(raw_time) > 1.e6) {
      stamp->fromSec(raw_time);
    } else {
      *stamp = header + ros::Duration(raw_time);
    }
    return stamp->isValid();
  }

  void ProcessPendingClouds() {
    while (!pending_clouds_.empty()) {
      const auto& cloud = pending_clouds_.front();
      ros::Time reference_stamp;
      if (!FindReferenceStamp(*cloud, &reference_stamp)) {
        pending_clouds_.pop_front();
        ++dropped_clouds_;
        continue;
      }
      if (odom_buffer_.empty() || odom_buffer_.back().stamp < reference_stamp) {
        return;
      }
      Pose2D reference_pose;
      if (!LookupOdom(reference_stamp, &reference_pose)) {
        pending_clouds_.pop_front();
        ++dropped_clouds_;
        continue;
      }
      Project(*cloud, reference_stamp, reference_pose);
      pending_clouds_.pop_front();
    }
  }

  bool FindReferenceStamp(const sensor_msgs::PointCloud2& cloud,
                          ros::Time* reference_stamp) const {
    const FieldInfo time = FindField(cloud, "time");
    if (!time.present || time.datatype != sensor_msgs::PointField::FLOAT64) {
      ROS_ERROR_THROTTLE(5.0,
                         "PointCloud2 requires a FLOAT64 time field");
      return false;
    }
    const std::size_t point_count =
        static_cast<std::size_t>(cloud.width) * cloud.height;
    bool found = false;
    double maximum = -std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < point_count; ++index) {
      const double raw = ReadValue<double>(
          cloud, index * cloud.point_step + time.offset);
      if (std::isfinite(raw) && (!found || raw > maximum)) {
        maximum = raw;
        found = true;
      }
    }
    if (!found) return false;
    return PointStamp(maximum, cloud.header.stamp, reference_stamp);
  }

  void Project(const sensor_msgs::PointCloud2& cloud,
               const ros::Time& reference_stamp,
               const Pose2D& reference_pose) {
    const FieldInfo x = FindField(cloud, "x");
    const FieldInfo y = FindField(cloud, "y");
    const FieldInfo z = FindField(cloud, "z");
    const FieldInfo ring = FindField(cloud, "ring");
    const FieldInfo time = FindField(cloud, "time");
    if (!x.present || !y.present || !z.present || !ring.present ||
        !time.present || time.datatype != sensor_msgs::PointField::FLOAT64 ||
        ring.datatype != sensor_msgs::PointField::UINT16) {
      ROS_ERROR_THROTTLE(5.0,
                         "PointCloud2 fields x/y/z/ring/time have wrong types");
      return;
    }
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
          base_frame_, cloud.header.frame_id, reference_stamp,
          ros::Duration(0.2));
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(2.0, "Deskew TF lookup failed: %s", error.what());
      ++dropped_clouds_;
      return;
    }
    tf2::Transform base_from_laser;
    base_from_laser.setOrigin(tf2::Vector3(
        transform.transform.translation.x, transform.transform.translation.y,
        transform.transform.translation.z));
    base_from_laser.setRotation(tf2::Quaternion(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w));
    const tf2::Transform laser_from_base = base_from_laser.inverse();

    const int bin_count = std::max(
        1, static_cast<int>(std::ceil(
               (angle_max_ - angle_min_) / angle_increment_)));
    std::vector<float> ranges(static_cast<std::size_t>(bin_count),
                              std::numeric_limits<float>::infinity());
    const double reference_cos = std::cos(reference_pose.yaw);
    const double reference_sin = std::sin(reference_pose.yaw);
    const std::size_t point_count =
        static_cast<std::size_t>(cloud.width) * cloud.height;
    for (std::size_t index = 0; index < point_count; ++index) {
      const std::size_t offset = index * cloud.point_step;
      const uint16_t point_ring = ReadValue<uint16_t>(cloud, offset + ring.offset);
      if (point_ring < ring_min_ || point_ring > ring_max_) continue;
      const float px = ReadValue<float>(cloud, offset + x.offset);
      const float py = ReadValue<float>(cloud, offset + y.offset);
      const float pz = ReadValue<float>(cloud, offset + z.offset);
      const double raw_time = ReadValue<double>(cloud, offset + time.offset);
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
        continue;
      }
      ros::Time point_stamp;
      if (!PointStamp(raw_time, cloud.header.stamp, &point_stamp)) continue;
      Pose2D point_pose;
      if (!LookupOdom(point_stamp, &point_pose)) continue;
      const tf2::Vector3 point_in_base =
          base_from_laser * tf2::Vector3(px, py, pz);
      const double point_cos = std::cos(point_pose.yaw);
      const double point_sin = std::sin(point_pose.yaw);
      const double world_x = point_pose.x + point_cos * point_in_base.x() -
                             point_sin * point_in_base.y();
      const double world_y = point_pose.y + point_sin * point_in_base.x() +
                             point_cos * point_in_base.y();
      const double delta_x = world_x - reference_pose.x;
      const double delta_y = world_y - reference_pose.y;
      const double base_x = reference_cos * delta_x + reference_sin * delta_y;
      const double base_y = -reference_sin * delta_x + reference_cos * delta_y;
      const tf2::Vector3 point_in_laser =
          laser_from_base * tf2::Vector3(base_x, base_y, 0.0);
      const double range = std::hypot(point_in_laser.x(), point_in_laser.y());
      if (range < range_min_ || range > range_max_) continue;
      const double angle = std::atan2(point_in_laser.y(), point_in_laser.x());
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
      ROS_WARN_THROTTLE(2.0, "Deskewed scan has too few valid bins");
      return;
    }
    sensor_msgs::LaserScan scan;
    scan.header.stamp = reference_stamp;
    scan.header.frame_id = output_frame_;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.angle_max = static_cast<float>(
        angle_min_ + (bin_count - 1) * angle_increment_);
    scan.time_increment = 0.0f;
    scan.scan_time = 0.1f;
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges = std::move(ranges);
    publisher_.publish(scan);
    ++published_scans_;
    if (published_scans_ == 1 || published_scans_ % 250 == 0) {
      ROS_INFO_STREAM("Published " << published_scans_
                      << " deskewed scans; valid_bins=" << valid_bins
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
        if (neighbor < 0 || neighbor >= static_cast<int>(input.size())) continue;
        const double neighbor_range = input[static_cast<std::size_t>(neighbor)];
        if (!std::isfinite(neighbor_range)) continue;
        const double angle = offset * angle_increment_;
        const double squared_distance =
            range * range + neighbor_range * neighbor_range -
            2.0 * range * neighbor_range * std::cos(angle);
        if (squared_distance <= isolation_max_distance_ * isolation_max_distance_) {
          supported = true;
          break;
        }
      }
      if (!supported) (*ranges)[static_cast<std::size_t>(index)] =
          std::numeric_limits<float>::infinity();
    }
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher publisher_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber cloud_subscriber_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::deque<TimedPose> odom_buffer_;
  std::deque<sensor_msgs::PointCloud2::ConstPtr> pending_clouds_;
  std::string input_topic_, odom_topic_, output_topic_;
  std::string base_frame_, output_frame_;
  int ring_min_ = 8;
  int ring_max_ = 9;
  double angle_min_ = -kPi;
  double angle_max_ = kPi;
  double angle_increment_ = 0.0043633231;
  double range_min_ = 0.30;
  double range_max_ = 30.0;
  bool filter_isolated_returns_ = true;
  int isolation_window_bins_ = 2;
  double isolation_max_distance_ = 0.30;
  int minimum_valid_bins_ = 60;
  double odom_tolerance_sec_ = 0.10;
  std::size_t published_scans_ = 0;
  std::size_t dropped_clouds_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "velodyne_deskewed_laserscan");
  VelodyneDeskewedLaserScan node;
  ros::spin();
  return 0;
}
