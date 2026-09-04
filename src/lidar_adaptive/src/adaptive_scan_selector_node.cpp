#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include <lidar_adaptive/ScanQuality.h>
#include <lidar_adaptive/ScanSelection.h>
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

struct QualitySample {
  ros::Time stamp;
  double quality = 0.0;
};

struct PendingScan {
  sensor_msgs::LaserScan scan;
  ros::WallTime received_wall;
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
    private_nh_.param("selection_detail_topic", selection_detail_topic_,
                      std::string("/scan_selection"));
    private_nh_.param("processing_time_topic", processing_time_topic_,
                      std::string("/lidar_selector_processing_ms"));
    private_nh_.param("quality_sync_topic", quality_sync_topic_,
                      std::string("/lidar_quality_sync_ok"));
    private_nh_.param("forward_selected_scans", forward_selected_scans_, false);
    private_nh_.param("min_quality", min_quality_, 0.20);
    private_nh_.param("selection_score", selection_score_, 0.76);
    private_nh_.param("novelty_range_threshold", novelty_range_threshold_, 0.25);
    private_nh_.param("min_motion_distance", min_motion_distance_, 0.05);
    private_nh_.param("min_motion_angle", min_motion_angle_, 0.015);
    private_nh_.param("max_skip_seconds", max_skip_seconds_, 1.0);
    private_nh_.param("quality_sync_tolerance", quality_sync_tolerance_, 0.05);
    private_nh_.param("pending_timeout_sec", pending_timeout_sec_, 0.20);
    private_nh_.param("max_pending_scans", max_pending_scans_, 20);
    quality_sync_tolerance_ = std::max(0.005, quality_sync_tolerance_);
    pending_timeout_sec_ = std::max(0.01, pending_timeout_sec_);
    max_pending_scans_ = std::max(1, max_pending_scans_);

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic_, 5);
    score_pub_ = nh_.advertise<std_msgs::Float32>(score_topic_, 5);
    selected_pub_ = nh_.advertise<std_msgs::Bool>(selected_topic_, 5);
    selection_detail_pub_ = nh_.advertise<lidar_adaptive::ScanSelection>(
        selection_detail_topic_, 5);
    processing_time_pub_ = nh_.advertise<std_msgs::Float32>(
        processing_time_topic_, 5);
    quality_sync_pub_ = nh_.advertise<std_msgs::Bool>(quality_sync_topic_, 5);
    scan_sub_ = nh_.subscribe(scan_topic_, 5,
                               &AdaptiveScanSelectorNode::ScanCallback, this);
    quality_sub_ = nh_.subscribe(quality_topic_, 5,
                                 &AdaptiveScanSelectorNode::QualityCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 50,
                               &AdaptiveScanSelectorNode::OdomCallback, this);
    pending_timer_ = nh_.createWallTimer(
        ros::WallDuration(0.02),
        &AdaptiveScanSelectorNode::PendingTimerCallback, this);

    ROS_INFO_STREAM("[LIDAR_ADAPTIVE] scan selector " << scan_topic_ << " -> "
                    << output_topic_ << "; forward_selected_scans="
                    << (forward_selected_scans_ ? "true" : "false"));
  }

 private:
  void QualityCallback(const lidar_adaptive::ScanQuality::ConstPtr& message) {
    const ros::Time stamp = message->header.stamp;
    const double quality = std::max(0.0, std::min(1.0,
                                                   static_cast<double>(message->quality)));
    latest_quality_ = quality;
    latest_quality_stamp_ = stamp;
    quality_samples_.push_back({stamp, quality});
    while (quality_samples_.size() > 100) quality_samples_.pop_front();
    ProcessPendingScans(false);
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

  double QualityForScan(const sensor_msgs::LaserScan& scan, bool* synchronized) {
    *synchronized = false;
    double selected_quality = 0.0;
    double best_delta = std::numeric_limits<double>::infinity();
    for (const QualitySample& sample : quality_samples_) {
      if (sample.stamp.isZero() || scan.header.stamp.isZero()) continue;
      const double delta = std::abs((sample.stamp - scan.header.stamp).toSec());
      if (delta < best_delta) {
        best_delta = delta;
        selected_quality = sample.quality;
      }
    }
    if (best_delta <= quality_sync_tolerance_) {
      *synchronized = true;
      return selected_quality;
    }
    if (!latest_quality_stamp_.isZero() && !scan.header.stamp.isZero() &&
        std::abs((latest_quality_stamp_ - scan.header.stamp).toSec()) <=
            quality_sync_tolerance_) {
      *synchronized = true;
      return latest_quality_;
    }
    int valid = 0;
    for (const float range : scan.ranges) {
      if (std::isfinite(range) && range >= scan.range_min &&
          range <= scan.range_max) {
        ++valid;
      }
    }
    return scan.ranges.empty() ? 0.0
                               : static_cast<double>(valid) / scan.ranges.size();
  }

  void PendingTimerCallback(const ros::WallTimerEvent&) {
    const ros::WallTime now = ros::WallTime::now();
    while (!pending_scans_.empty() &&
           (now - pending_scans_.front().received_wall).toSec() >=
               pending_timeout_sec_) {
      ProcessPendingScanWithFallback();
    }
  }

  void ProcessPendingScanWithFallback() {
    if (pending_scans_.empty()) return;
    const sensor_msgs::LaserScan scan = pending_scans_.front().scan;
    pending_scans_.pop_front();
    bool quality_synchronized = false;
    const double projection_quality =
        QualityForScan(scan, &quality_synchronized);
    ProcessScan(scan, projection_quality, quality_synchronized);
  }

  void ProcessPendingScans(bool force_fallback) {
    while (!pending_scans_.empty()) {
      bool quality_synchronized = false;
      const double projection_quality =
          QualityForScan(pending_scans_.front().scan, &quality_synchronized);
      if (!quality_synchronized && !force_fallback) break;
      const sensor_msgs::LaserScan scan = pending_scans_.front().scan;
      pending_scans_.pop_front();
      ProcessScan(scan, projection_quality, quality_synchronized);
    }
  }

  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    pending_scans_.push_back({*scan, ros::WallTime::now()});
    ProcessPendingScans(false);
    while (pending_scans_.size() > static_cast<std::size_t>(max_pending_scans_)) {
      ProcessPendingScanWithFallback();
    }
  }

  void ProcessScan(const sensor_msgs::LaserScan& scan,
                   double projection_quality,
                   bool quality_synchronized) {
    const ros::WallTime processing_start = ros::WallTime::now();
    int valid = 0;
    for (const float range : scan.ranges) {
      if (std::isfinite(range) && range >= scan.range_min &&
          range <= scan.range_max) {
        ++valid;
      }
    }
    const double valid_ratio = scan.ranges.empty()
                                   ? 0.0
                                   : static_cast<double>(valid) /
                                         scan.ranges.size();
    const double novelty = ScanNovelty(scan);
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
        0.35 * projection_quality + 0.25 * valid_ratio +
        0.20 * angular_coverage + 0.15 * novelty + 0.05 * motion_score;

    bool selected = false;
    if (!have_previous_scan_) {
      selected = true;
    } else if (projection_quality >= min_quality_ &&
               information_score >= selection_score_) {
      selected = true;
    }
    if (have_last_selected_stamp_ &&
        (scan.header.stamp - last_selected_stamp_).toSec() >= max_skip_seconds_) {
      selected = true;
    }

    std_msgs::Float32 score_msg;
    score_msg.data = static_cast<float>(information_score);
    score_pub_.publish(score_msg);
    std_msgs::Bool selected_msg;
    selected_msg.data = selected;
    selected_pub_.publish(selected_msg);

    lidar_adaptive::ScanSelection selection_msg;
    selection_msg.header = scan.header;
    selection_msg.selected = selected;
    selection_msg.information_score = static_cast<float>(information_score);
    selection_msg.projection_quality = static_cast<float>(projection_quality);
    selection_msg.quality_synchronized = quality_synchronized;
    selection_detail_pub_.publish(selection_msg);

    std_msgs::Bool quality_sync_msg;
    quality_sync_msg.data = quality_synchronized;
    quality_sync_pub_.publish(quality_sync_msg);
    if (!quality_synchronized) {
      ROS_WARN_THROTTLE(5.0,
                        "[LIDAR_ADAPTIVE] no timestamp-matched quality for scan; using scan valid ratio fallback");
    }

    if (!forward_selected_scans_ || selected) {
      scan_pub_.publish(scan);
    }
    std_msgs::Float32 processing_time_msg;
    processing_time_msg.data = static_cast<float>(
        (ros::WallTime::now() - processing_start).toSec() * 1000.0);
    processing_time_pub_.publish(processing_time_msg);
    if (selected) {
      last_selected_stamp_ = scan.header.stamp;
      have_last_selected_stamp_ = true;
      if (have_pose_) {
        last_selected_pose_ = latest_pose_;
        have_last_selected_pose_ = true;
      }
    }
    previous_scan_ = scan;
    have_previous_scan_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber quality_sub_;
  ros::Subscriber odom_sub_;
  ros::WallTimer pending_timer_;
  ros::Publisher scan_pub_;
  ros::Publisher score_pub_;
  ros::Publisher selected_pub_;
  ros::Publisher selection_detail_pub_;
  ros::Publisher quality_sync_pub_;
  ros::Publisher processing_time_pub_;

  std::string scan_topic_;
  std::string quality_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string score_topic_;
  std::string selected_topic_;
  std::string selection_detail_topic_;
  std::string processing_time_topic_;
  std::string quality_sync_topic_;
  bool forward_selected_scans_ = false;
  double min_quality_ = 0.20;
  double selection_score_ = 0.76;
  double novelty_range_threshold_ = 0.25;
  double min_motion_distance_ = 0.05;
  double min_motion_angle_ = 0.015;
  double max_skip_seconds_ = 1.0;
  double quality_sync_tolerance_ = 0.05;
  double pending_timeout_sec_ = 0.20;
  int max_pending_scans_ = 20;

  double latest_quality_ = 0.0;
  ros::Time latest_quality_stamp_;
  std::deque<QualitySample> quality_samples_;
  Pose2D latest_pose_;
  Pose2D last_selected_pose_;
  bool have_pose_ = false;
  bool have_last_selected_pose_ = false;
  bool have_previous_scan_ = false;
  bool have_last_selected_stamp_ = false;
  ros::Time last_selected_stamp_;
  std::deque<PendingScan> pending_scans_;
  sensor_msgs::LaserScan previous_scan_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "adaptive_scan_selector_node");
  AdaptiveScanSelectorNode node;
  ros::spin();
  return 0;
}
