#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <lidar_adaptive/ScanQuality.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct FieldInfo {
  bool present = false;
  uint32_t offset = 0;
  uint8_t datatype = 0;
};

struct BinEvidence {
  float range = std::numeric_limits<float>::infinity();
  float confidence = 0.f;
  int support_count = 0;
  bool stable_height = false;
  bool has_hit = false;
};

FieldInfo FindField(const sensor_msgs::PointCloud2& cloud,
                    const std::string& name) {
  for (const sensor_msgs::PointField& field : cloud.fields) {
    if (field.name == name) {
      return {true, field.offset, field.datatype};
    }
  }
  return {};
}

template <typename T>
T ReadValue(const sensor_msgs::PointCloud2& cloud, std::size_t offset,
            bool* ok) {
  T value{};
  if (offset + sizeof(T) <= cloud.data.size()) {
    std::memcpy(&value, cloud.data.data() + offset, sizeof(T));
  } else {
    *ok = false;
  }
  return value;
}

double ReadNumeric(const sensor_msgs::PointCloud2& cloud,
                   std::size_t offset, uint8_t datatype, bool* ok) {
  *ok = true;
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
      return ReadValue<int8_t>(cloud, offset, ok);
    case sensor_msgs::PointField::UINT8:
      return ReadValue<uint8_t>(cloud, offset, ok);
    case sensor_msgs::PointField::INT16:
      return ReadValue<int16_t>(cloud, offset, ok);
    case sensor_msgs::PointField::UINT16:
      return ReadValue<uint16_t>(cloud, offset, ok);
    case sensor_msgs::PointField::INT32:
      return ReadValue<int32_t>(cloud, offset, ok);
    case sensor_msgs::PointField::UINT32:
      return ReadValue<uint32_t>(cloud, offset, ok);
    case sensor_msgs::PointField::FLOAT32:
      return ReadValue<float>(cloud, offset, ok);
    case sensor_msgs::PointField::FLOAT64:
      return ReadValue<double>(cloud, offset, ok);
    default:
      *ok = false;
      return 0.0;
  }
}

class ConfidenceProjectionNode {
 public:
  ConfidenceProjectionNode()
      : private_nh_("~") {
    private_nh_.param("input_topic", input_topic_, std::string("/velodyne_points"));
    private_nh_.param("output_topic", output_topic_, std::string("/scan_confidence"));
    private_nh_.param("quality_topic", quality_topic_, std::string("/lidar_scan_quality"));
    private_nh_.param("processing_time_topic", processing_time_topic_,
                      std::string("/lidar_projection_processing_ms"));
    private_nh_.param("valid_ratio_topic", valid_ratio_topic_, std::string("/lidar_valid_beam_ratio"));
    private_nh_.param("valid_bins_topic", valid_bins_topic_, std::string("/lidar_valid_beams"));
    private_nh_.param("ring_min", ring_min_, 0);
    private_nh_.param("ring_max", ring_max_, 255);
    private_nh_.param("require_ring", require_ring_, false);
    private_nh_.param("min_height", min_height_, -0.60);
    private_nh_.param("max_height", max_height_, 1.50);
    private_nh_.param("stable_min_height", stable_min_height_, -0.10);
    private_nh_.param("stable_max_height", stable_max_height_, 0.80);
    private_nh_.param("range_min", range_min_, 0.30);
    private_nh_.param("range_max", range_max_, 50.0);
    private_nh_.param("angle_min", angle_min_, -kPi);
    private_nh_.param("angle_max", angle_max_, kPi);
    private_nh_.param("angle_increment", angle_increment_, 0.0043633231);
    private_nh_.param("neighbor_window_bins", neighbor_window_bins_, 2);
    private_nh_.param("neighbor_distance", neighbor_distance_, 0.30);
    private_nh_.param("ring_consistency_distance", ring_consistency_distance_, 0.15);
    private_nh_.param("min_valid_bins", min_valid_bins_, 60);
    private_nh_.param("filter_isolated", filter_isolated_, true);
    private_nh_.param("publish_empty_on_failure", publish_empty_on_failure_, false);

    ring_min_ = std::max(0, ring_min_);
    ring_max_ = std::max(ring_min_, ring_max_);
    angle_increment_ = std::max(1.e-5, angle_increment_);
    angle_max_ = std::max(angle_min_ + angle_increment_, angle_max_);
    range_max_ = std::max(range_min_ + 0.01, range_max_);
    neighbor_window_bins_ = std::max(1, neighbor_window_bins_);
    neighbor_distance_ = std::max(0.01, neighbor_distance_);
    ring_consistency_distance_ = std::max(0.01, ring_consistency_distance_);
    min_valid_bins_ = std::max(1, min_valid_bins_);

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic_, 5);
    quality_pub_ = nh_.advertise<lidar_adaptive::ScanQuality>(quality_topic_, 5);
    processing_time_pub_ = nh_.advertise<std_msgs::Float32>(
        processing_time_topic_, 5);
    valid_ratio_pub_ = nh_.advertise<std_msgs::Float32>(valid_ratio_topic_, 5);
    valid_bins_pub_ = nh_.advertise<std_msgs::UInt32>(valid_bins_topic_, 5);
    cloud_sub_ = nh_.subscribe(input_topic_, 5,
                               &ConfidenceProjectionNode::CloudCallback, this);

    ROS_INFO_STREAM("[LIDAR_ADAPTIVE] confidence projection " << input_topic_
                    << " -> " << output_topic_ << "; height ["
                    << min_height_ << ", " << max_height_ << "] stable ["
                    << stable_min_height_ << ", " << stable_max_height_
                    << "] rings [" << ring_min_ << ", " << ring_max_
                    << "] require_ring=" << (require_ring_ ? "true" : "false"));
  }

 private:
  void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    const ros::WallTime processing_start = ros::WallTime::now();
    const FieldInfo x = FindField(*cloud, "x");
    const FieldInfo y = FindField(*cloud, "y");
    const FieldInfo z = FindField(*cloud, "z");
    const FieldInfo ring = FindField(*cloud, "ring");
    if (!x.present || !y.present || !z.present) {
      ROS_ERROR_THROTTLE(5.0, "[LIDAR_ADAPTIVE] PointCloud2 needs x/y/z fields");
      return;
    }
    if (require_ring_ && !ring.present) {
      ROS_ERROR_THROTTLE(5.0, "[LIDAR_ADAPTIVE] ring field required but missing");
      return;
    }

    const int bin_count = std::max(
        1, static_cast<int>(std::ceil((angle_max_ - angle_min_) /
                                       angle_increment_)));
    std::vector<BinEvidence> bins(static_cast<std::size_t>(bin_count));
    const std::size_t point_count =
        static_cast<std::size_t>(cloud->width) * cloud->height;
    if (cloud->point_step == 0 || cloud->width == 0 ||
        cloud->row_step < cloud->width * cloud->point_step ||
        cloud->data.size() < static_cast<std::size_t>(cloud->row_step) *
                                 cloud->height) {
      ROS_ERROR_THROTTLE(5.0,
                         "[LIDAR_ADAPTIVE] invalid PointCloud2 layout");
      return;
    }
    for (std::size_t i = 0; i < point_count; ++i) {
      const std::size_t row = i / cloud->width;
      const std::size_t column = i % cloud->width;
      const std::size_t base = row * cloud->row_step +
                               column * cloud->point_step;
      bool ok_x = false, ok_y = false, ok_z = false;
      const double px = ReadNumeric(*cloud, base + x.offset, x.datatype, &ok_x);
      const double py = ReadNumeric(*cloud, base + y.offset, y.datatype, &ok_y);
      const double pz = ReadNumeric(*cloud, base + z.offset, z.datatype, &ok_z);
      if (!ok_x || !ok_y || !ok_z || !std::isfinite(px) ||
          !std::isfinite(py) || !std::isfinite(pz)) {
        continue;
      }

      int point_ring = -1;
      if (ring.present) {
        bool ok_ring = false;
        point_ring = static_cast<int>(std::llround(
            ReadNumeric(*cloud, base + ring.offset, ring.datatype, &ok_ring)));
        if (!ok_ring || point_ring < ring_min_ || point_ring > ring_max_) {
          continue;
        }
      }
      if (pz < min_height_ || pz > max_height_) continue;
      const double range = std::hypot(px, py);
      if (range < range_min_ || range > range_max_) continue;
      const double angle = std::atan2(py, px);
      const int bin = static_cast<int>(
          std::floor((angle - angle_min_) / angle_increment_));
      if (bin < 0 || bin >= bin_count) continue;

      const bool stable_height =
          pz >= stable_min_height_ && pz <= stable_max_height_;
      const float height_score = stable_height ? 1.0f : 0.35f;
      BinEvidence& evidence = bins[static_cast<std::size_t>(bin)];
      // Prefer a stable-height return over a closer low-confidence return;
      // within one class retain the nearest echo.
      if (!evidence.has_hit ||
          (stable_height && !evidence.stable_height) ||
          (stable_height == evidence.stable_height && range < evidence.range)) {
        evidence.range = static_cast<float>(range);
        evidence.confidence = height_score;
        evidence.support_count = 1;
        evidence.stable_height = stable_height;
        evidence.has_hit = true;
      } else if (std::abs(range - evidence.range) <= ring_consistency_distance_) {
        ++evidence.support_count;
        evidence.confidence = std::min(1.0f, evidence.confidence + 0.10f);
      }
    }

    int candidate_bins = 0;
    int valid_bins = 0;
    int supported_bins = 0;
    float confidence_sum = 0.f;
    for (int i = 0; i < bin_count; ++i) {
      BinEvidence& evidence = bins[static_cast<std::size_t>(i)];
      if (!evidence.has_hit) continue;
      ++candidate_bins;
      bool supported = false;
      if (!filter_isolated_) {
        supported = true;
      } else {
        for (int offset = -neighbor_window_bins_;
             offset <= neighbor_window_bins_; ++offset) {
          if (offset == 0) continue;
          const int neighbor = i + offset;
          if (neighbor < 0 || neighbor >= bin_count) continue;
          const BinEvidence& other = bins[static_cast<std::size_t>(neighbor)];
          if (!other.has_hit) continue;
          const double a = offset * angle_increment_;
          const double squared_distance =
              evidence.range * evidence.range + other.range * other.range -
              2.0 * evidence.range * other.range * std::cos(a);
          if (squared_distance <= neighbor_distance_ * neighbor_distance_) {
            supported = true;
            break;
          }
        }
      }
      if (!supported) {
        evidence.has_hit = false;
        continue;
      }
      ++valid_bins;
      ++supported_bins;
      confidence_sum += evidence.confidence;
    }

    const float valid_ratio = static_cast<float>(valid_bins) /
                              static_cast<float>(bin_count);
    const float support_ratio = candidate_bins > 0
                                    ? static_cast<float>(supported_bins) /
                                          static_cast<float>(candidate_bins)
                                    : 0.f;
    const float mean_confidence = valid_bins > 0
                                      ? confidence_sum / valid_bins
                                      : 0.f;
    const float quality = 0.50f * valid_ratio +
                          0.20f * support_ratio + 0.30f * mean_confidence;

    lidar_adaptive::ScanQuality quality_msg;
    quality_msg.header = cloud->header;
    quality_msg.quality = quality;
    quality_msg.valid_ratio = valid_ratio;
    quality_msg.valid_bins = static_cast<uint32_t>(valid_bins);
    quality_msg.candidate_bins = static_cast<uint32_t>(candidate_bins);
    quality_msg.support_ratio = support_ratio;
    quality_msg.mean_height_confidence = mean_confidence;
    quality_pub_.publish(quality_msg);
    std_msgs::Float32 ratio_msg;
    ratio_msg.data = valid_ratio;
    valid_ratio_pub_.publish(ratio_msg);
    std_msgs::UInt32 bins_msg;
    bins_msg.data = static_cast<uint32_t>(valid_bins);
    valid_bins_pub_.publish(bins_msg);

    if (valid_bins < min_valid_bins_) {
      ROS_WARN_THROTTLE(2.0, "[LIDAR_ADAPTIVE] too few valid beams: %d/%d",
                        valid_bins, bin_count);
      if (!publish_empty_on_failure_) return;
    }

    sensor_msgs::LaserScan scan;
    scan.header = cloud->header;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.angle_max = static_cast<float>(angle_min_ +
                                        (bin_count - 1) * angle_increment_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.scan_time = 0.1f;
    scan.time_increment = 0.0f;
    scan.ranges.assign(static_cast<std::size_t>(bin_count),
                       std::numeric_limits<float>::infinity());
    for (int i = 0; i < bin_count; ++i) {
      const BinEvidence& evidence = bins[static_cast<std::size_t>(i)];
      if (evidence.has_hit) {
        scan.ranges[static_cast<std::size_t>(i)] = evidence.range;
      }
    }
    scan_pub_.publish(scan);
    std_msgs::Float32 processing_time_msg;
    processing_time_msg.data = static_cast<float>(
        (ros::WallTime::now() - processing_start).toSec() * 1000.0);
    processing_time_pub_.publish(processing_time_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher scan_pub_;
  ros::Publisher quality_pub_;
  ros::Publisher valid_ratio_pub_;
  ros::Publisher valid_bins_pub_;
  ros::Publisher processing_time_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string quality_topic_;
  std::string valid_ratio_topic_;
  std::string valid_bins_topic_;
  std::string processing_time_topic_;
  int ring_min_ = 0;
  int ring_max_ = 255;
  bool require_ring_ = false;
  double min_height_ = -0.60;
  double max_height_ = 1.50;
  double stable_min_height_ = -0.10;
  double stable_max_height_ = 0.80;
  double range_min_ = 0.30;
  double range_max_ = 50.0;
  double angle_min_ = -kPi;
  double angle_max_ = kPi;
  double angle_increment_ = 0.0043633231;
  int neighbor_window_bins_ = 2;
  double neighbor_distance_ = 0.30;
  double ring_consistency_distance_ = 0.15;
  int min_valid_bins_ = 60;
  bool filter_isolated_ = true;
  bool publish_empty_on_failure_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "confidence_projection_node");
  ConfidenceProjectionNode node;
  ros::spin();
  return 0;
}
