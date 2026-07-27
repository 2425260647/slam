#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace {

// [Innovation 1] Rejects geometrically insufficient projected scans before
// they can drive the local scan matcher into a false corridor orientation.
class ScanQualityGate {
public:
  ScanQualityGate() : private_node_("~") {
    private_node_.param<std::string>("input_topic", input_topic_,
                                     "/scan_projected");
    private_node_.param<std::string>("output_topic", output_topic_, "/scan");
    private_node_.param<bool>("enabled", enabled_, true);
    private_node_.param<int>("min_valid_ranges", min_valid_ranges_, 250);
    private_node_.param<int>("sector_count", sector_count_, 8);
    private_node_.param<int>("min_active_sectors", min_active_sectors_, 2);
    private_node_.param<int>("min_ranges_per_sector", min_ranges_per_sector_,
                             10);

    min_valid_ranges_ = std::max(1, min_valid_ranges_);
    sector_count_ = std::max(1, sector_count_);
    min_active_sectors_ =
        std::max(1, std::min(sector_count_, min_active_sectors_));
    min_ranges_per_sector_ = std::max(1, min_ranges_per_sector_);

    publisher_ = node_.advertise<sensor_msgs::LaserScan>(output_topic_, 10);
    subscriber_ =
        node_.subscribe(input_topic_, 10, &ScanQualityGate::HandleScan, this);

    ROS_INFO("[Innovation1 Support] Scan quality gate input=%s output=%s "
             "enabled=%s min_valid=%d sectors=%d min_active=%d "
             "min_per_sector=%d",
             input_topic_.c_str(), output_topic_.c_str(),
             enabled_ ? "true" : "false", min_valid_ranges_, sector_count_,
             min_active_sectors_, min_ranges_per_sector_);
  }

private:
  // [Innovation 1] A scan is accepted only when enough finite ranges exist
  // and those ranges occupy multiple angular sectors. This prevents a short
  // one-sided return burst from being treated as a full geometric observation.
  void HandleScan(const sensor_msgs::LaserScan::ConstPtr &scan) {
    if (!enabled_) {
      publisher_.publish(scan);
      return;
    }

    std::vector<int> sector_counts(static_cast<std::size_t>(sector_count_), 0);
    int valid_ranges = 0;
    for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
      const float range = scan->ranges[i];
      if (!std::isfinite(range) || range < scan->range_min ||
          range > scan->range_max) {
        continue;
      }
      ++valid_ranges;
      const std::size_t sector = std::min(
          static_cast<std::size_t>(sector_count_ - 1),
          i * static_cast<std::size_t>(sector_count_) / scan->ranges.size());
      ++sector_counts[sector];
    }

    int active_sectors = 0;
    for (const int count : sector_counts) {
      if (count >= min_ranges_per_sector_) {
        ++active_sectors;
      }
    }

    if (valid_ranges < min_valid_ranges_ ||
        active_sectors < min_active_sectors_) {
      ++dropped_scans_;
      ROS_WARN_THROTTLE(
          2.,
          "[Innovation1 Support] Dropping sparse scan: valid=%d/%d "
          "active_sectors=%d/%d dropped=%llu accepted=%llu",
          valid_ranges, min_valid_ranges_, active_sectors, min_active_sectors_,
          static_cast<unsigned long long>(dropped_scans_),
          static_cast<unsigned long long>(accepted_scans_));
      return;
    }

    ++accepted_scans_;
    publisher_.publish(scan);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string input_topic_;
  std::string output_topic_;
  bool enabled_ = true;
  int min_valid_ranges_ = 250;
  int sector_count_ = 8;
  int min_active_sectors_ = 2;
  int min_ranges_per_sector_ = 10;
  std::uint64_t accepted_scans_ = 0;
  std::uint64_t dropped_scans_ = 0;
};

} // namespace

// [Innovation 1] ROS entry point for projected-scan quality gating.
int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_quality_gate");
  ScanQualityGate gate;
  ros::spin();
  return 0;
}
