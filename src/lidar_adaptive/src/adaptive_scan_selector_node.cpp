#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

namespace {

double NormalizeAngle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

Pose2D ToPose(const nav_msgs::Odometry& odom) {
  const auto& p = odom.pose.pose.position;
  const auto& q = odom.pose.pose.orientation;
  return {p.x, p.y,
          std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
}

class AdaptiveScanSelectorNode {
 public:
  AdaptiveScanSelectorNode() : private_nh_("~") {
    private_nh_.param("scan_topic", scan_topic_, std::string("/scan_confidence"));
    private_nh_.param("quality_topic", quality_topic_, std::string("/lidar_scan_quality"));
    private_nh_.param("odom_topic", odom_topic_, std::string("/odom"));
    private_nh_.param("output_topic", output_topic_, std::string("/scan_selected"));
    private_nh_.param("score_topic", score_topic_, std::string("/lidar_information_score"));
    private_nh_.param("selected_topic", selected_topic_, std::string("/keyframe_selected"));
    private_nh_.param("forward_selected_scans", forward_selected_scans_, false);
    private_nh_.param("min_quality", min_quality_, 0.20);
    private_nh_.param("selection_score", selection_score_, 0.46);
    private_nh_.param("novelty_range_threshold", novelty_range_threshold_, 0.25);
    private_nh_.param("min_motion_distance", min_motion_distance_, 0.05);
    private_nh_.param("min_motion_angle", min_motion_angle_, 0.015);
    private_nh_.param("max_skip_seconds", max_skip_seconds_, 1.0);

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic_, 5);
    score_pub_ = nh_.advertise<std_msgs::Float32>(score_topic_, 5);
    selected_pub_ = nh_.advertise<std_msgs::Bool>(selected_topic_, 5);
    scan_sub_ = nh_.subscribe(scan_topic_, 5,
                               &AdaptiveScanSelectorNode::ScanCallback, this);
    quality_sub_ = nh_.subscribe(quality_topic_, 5,
                                 &AdaptiveScanSelectorNode::QualityCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 50,
                               &AdaptiveScanSelectorNode::OdomCallback, this);

    ROS_INFO_STREAM("[LIDAR_ADAPTIVE] scan selector " << scan_topic_ << " -> "
                    << output_topic_ << "; forward_selected_scans="
                    << (forward_selected_scans_ ? "true" : "false"));
  }

 private:
  void QualityCallback(const std_msgs::Float32::ConstPtr& message) {
    latest_quality_ = std::max(0.0, std::min(1.0,
                                             static_cast<double>(message->data)));
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    latest_pose_ = ToPose(*message);
    have_pose_ = true;
  }

  double ScanNovelty(const sensor_msgs::LaserScan& scan) const {
    if (!have_previous_scan_ || previous_scan_.ranges.size() != scan.ranges.size()) {
      return 1.0;
    }
    int comparable = 0;
    int changed = 0;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const float current = scan.ranges[i];
      const float previous = previous_scan_.ranges[i];
      if (!std::isfinite(current) || !std::isfinite(previous)) continue;
      ++comparable;
      if (std::abs(static_cast<double>(current) - previous) >=
          novelty_range_threshold_) {
        ++changed;
      }
    }
    return comparable > 0 ? static_cast<double>(changed) / comparable : 1.0;
  }

  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    int valid = 0;
    for (const float range : scan->ranges) {
      if (std::isfinite(range) && range >= scan->range_min &&
          range <= scan->range_max) {
        ++valid;
      }
    }
    const double valid_ratio = scan->ranges.empty()
                                   ? 0.0
                                   : static_cast<double>(valid) /
                                         scan->ranges.size();
    const double novelty = ScanNovelty(*scan);
    double motion_score = 0.0;
    if (have_pose_ && have_last_selected_pose_) {
      const double distance = std::hypot(latest_pose_.x - last_selected_pose_.x,
                                         latest_pose_.y - last_selected_pose_.y);
      const double angle = std::abs(
          NormalizeAngle(latest_pose_.yaw - last_selected_pose_.yaw));
      motion_score = std::min(1.0, distance / std::max(0.01, min_motion_distance_)) * 0.7 +
                     std::min(1.0, angle / std::max(0.001, min_motion_angle_)) * 0.3;
    } else {
      motion_score = 1.0;
    }
    const double angular_coverage = valid_ratio;
    const double information_score =
        0.35 * latest_quality_ + 0.25 * valid_ratio +
        0.20 * angular_coverage + 0.15 * novelty + 0.05 * motion_score;

    bool selected = false;
    if (!have_previous_scan_) {
      selected = true;
    } else if (latest_quality_ >= min_quality_ &&
               information_score >= selection_score_) {
      selected = true;
    }
    if (have_last_selected_stamp_ &&
        (scan->header.stamp - last_selected_stamp_).toSec() >= max_skip_seconds_) {
      selected = true;
    }

    std_msgs::Float32 score_msg;
    score_msg.data = static_cast<float>(information_score);
    score_pub_.publish(score_msg);
    std_msgs::Bool selected_msg;
    selected_msg.data = selected;
    selected_pub_.publish(selected_msg);

    if (!forward_selected_scans_ || selected) {
      scan_pub_.publish(*scan);
    }
    if (selected) {
      last_selected_stamp_ = scan->header.stamp;
      have_last_selected_stamp_ = true;
      if (have_pose_) {
        last_selected_pose_ = latest_pose_;
        have_last_selected_pose_ = true;
      }
    }
    previous_scan_ = *scan;
    have_previous_scan_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber quality_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher scan_pub_;
  ros::Publisher score_pub_;
  ros::Publisher selected_pub_;

  std::string scan_topic_;
  std::string quality_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string score_topic_;
  std::string selected_topic_;
  bool forward_selected_scans_ = false;
  double min_quality_ = 0.20;
  double selection_score_ = 0.46;
  double novelty_range_threshold_ = 0.25;
  double min_motion_distance_ = 0.05;
  double min_motion_angle_ = 0.015;
  double max_skip_seconds_ = 1.0;

  double latest_quality_ = 0.0;
  Pose2D latest_pose_;
  Pose2D last_selected_pose_;
  bool have_pose_ = false;
  bool have_last_selected_pose_ = false;
  bool have_previous_scan_ = false;
  bool have_last_selected_stamp_ = false;
  ros::Time last_selected_stamp_;
  sensor_msgs::LaserScan previous_scan_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "adaptive_scan_selector_node");
  AdaptiveScanSelectorNode node;
  ros::spin();
  return 0;
}
