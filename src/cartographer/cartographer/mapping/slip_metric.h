#ifndef CARTOGRAPHER_MAPPING_SLIP_METRIC_H_
#define CARTOGRAPHER_MAPPING_SLIP_METRIC_H_

#include <cstdint>

#include "Eigen/Core"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

// [Innovation 2] Thread-safe snapshot for IMU-free consistency-anomaly
// [Innovation 2] diagnostics. Legacy SlipMetric naming is kept for API and
// [Innovation 2] experiment-script compatibility.
// [Innovation 2] The pose-graph optimization thread writes this after each
// [Innovation 2] evaluated odometry edge; cartographer_ros reads a copy for
// [Innovation 2] visualization topics.
struct SlipMetric {
  bool enabled = false;
  bool slipping = false;
  double slip_score = 0.;
  double lateral_error = 0.;
  double yaw_error = 0.;
  double gated_slip_score = 0.;
  bool lidar_reliability_available = false;
  double lidar_reliability = 0.;
  double weight_scale = 1.;
  double translation_weight = 0.;
  double rotation_weight = 0.;
  common::Time time = common::FromUniversal(0);
  // [Innovation 2] 以下字段汇总本次 Solve() 按时间重放历史边得到的结果，
  // [Innovation 2] 不跨 Solve() 重复累计同一条历史边。
  std::uint32_t anomaly_trigger_count = 0;
  double minimum_weight_scale = 1.;
  int latest_anomaly_trajectory_id = -1;
  int latest_anomaly_node_index = -1;
  common::Time latest_anomaly_time = common::FromUniversal(0);
  double latest_anomaly_raw_score = 0.;
  double latest_anomaly_gated_score = 0.;
};

// [Innovation 2] Returns the latest slip metric copy protected by an internal
// [Innovation 2] mutex in the optimization implementation.
SlipMetric GetLatestSlipMetric();

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SLIP_METRIC_H_
