#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_SLIP_DETECTOR_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_SLIP_DETECTOR_H_

#include <algorithm>
#include <cmath>

#include "cartographer/common/math.h"
#include "cartographer/mapping/proto/pose_graph/optimization_problem_options.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {
namespace optimization {

// [Innovation 2] 单条相邻节点 odometry 边的抗打滑检测结果。
struct SlipDetectorResult {
  bool enabled = false;
  bool slipping = false;
  double lateral_error = 0.;
  double yaw_error = 0.;
  double slip_score = 0.;
  double gated_slip_score = 0.;
  bool lidar_reliability_available = false;
  double lidar_reliability = 0.;
  double weight_scale = 1.;
};

// [Innovation 2] 无 IMU 的 2D 后端 odometry 边抗打滑检测器。
// [Innovation 2] 核心思想：比较相邻关键帧之间的 LiDAR local SLAM 相对位姿
// [Innovation 2] 和轮式里程计相对位姿，只重点看横向残差 lateral residual e_y
// [Innovation 2] 与航向残差 yaw residual e_theta。差速/四轮差速底盘正常运动时
// [Innovation 2] 横向位移应接近 0；若轮子侧滑或打滑，e_y/e_theta 会突变。
// [Innovation 2] 这里故意不使用纵向 x 误差，因为长走廊中激光纵向本来可能退化。
class SlipDetector {
 public:
  explicit SlipDetector(const proto::OptimizationProblemOptions& options)
      : enabled_(options.slip_adaptive_odometry_weight_enabled()),
        lateral_error_weight_(
            std::max(0., options.slip_lateral_error_weight())),
        yaw_error_weight_(std::max(0., options.slip_yaw_error_weight())),
        high_threshold_(std::max(0., options.slip_high_threshold())),
        low_threshold_(std::max(0., std::min(options.slip_low_threshold(),
                                             options.slip_high_threshold()))),
        min_weight_scale_(std::max(0., options.slip_min_weight_scale())),
        max_weight_scale_(
            std::max(min_weight_scale_, options.slip_max_weight_scale())),
        recovery_alpha_(
            std::min(1., std::max(0., options.slip_recovery_alpha()))),
        min_motion_distance_(std::max(0., options.slip_min_motion_distance())),
        min_motion_angle_(std::max(0., options.slip_min_motion_angle())),
        lidar_reliability_gate_enabled_(
            options.slip_lidar_reliability_gate_enabled()),
        lidar_reliability_min_(
            std::min(1., std::max(0., options.slip_lidar_reliability_min()))),
        unknown_keep_previous_weight_(
            options.slip_unknown_keep_previous_weight()),
        slipping_(false),
        current_weight_scale_(max_weight_scale_) {}

  // [Innovation 2] 计算一条 odometry 约束边的动态权重缩放系数。
  // [Innovation 2] scan_relative：激光前端相邻帧相对位姿 T_scan_ij。
  // [Innovation 2] odom_relative：轮式里程计相邻帧相对位姿 T_odom_ij。
  SlipDetectorResult Update(const transform::Rigid2d& scan_relative,
                            const transform::Rigid2d& odom_relative) {
    return Update(scan_relative, odom_relative,
                  true /* lidar_reliability_available */,
                  1. /* lidar_reliability */);
  }

  // [Innovation 2] 第二版入口：增加 LiDAR 可靠性门控。
  // [Innovation 2] lidar_reliability 来自创新点一的时间对齐退化指标缓存。
  // [Innovation 2] 当可靠性未知或低于阈值时，不能用 LiDAR-odom 差异武断判定
  // [Innovation 2] wheel slip；默认保持上一条边的状态和权重。
  SlipDetectorResult Update(const transform::Rigid2d& scan_relative,
                            const transform::Rigid2d& odom_relative,
                            const bool lidar_reliability_available,
                            const double lidar_reliability) {
    SlipDetectorResult result;
    result.enabled = enabled_;
    result.weight_scale = current_weight_scale_;
    result.slipping = slipping_;
    result.lidar_reliability_available =
        !lidar_reliability_gate_enabled_ || lidar_reliability_available;
    result.lidar_reliability =
        lidar_reliability_gate_enabled_
            ? std::min(1., std::max(0., lidar_reliability))
            : 1.;
    if (!enabled_) {
      return result;
    }

    const double motion_distance = std::max(scan_relative.translation().norm(),
                                            odom_relative.translation().norm());
    const double motion_angle =
        std::max(std::abs(scan_relative.rotation().angle()),
                 std::abs(odom_relative.rotation().angle()));
    const bool enough_motion = motion_distance >= min_motion_distance_ ||
                               motion_angle >= min_motion_angle_;

    // [Innovation 2] 残差在激光相对运动坐标系中表达：
    // [Innovation 2] E_ij = inv(T_scan_ij) * T_odom_ij。
    // [Innovation 2] E_ij.translation().y() 是横向不一致量 lateral
    // inconsistency； [Innovation 2] yaw(E_ij) 是航向不一致量 heading
    // inconsistency。 [Innovation 2] 这样避免直接比较世界坐标 y
    // 导致坐标系含义混乱。
    const transform::Rigid2d error = scan_relative.inverse() * odom_relative;
    result.lateral_error = std::abs(error.translation().y());
    result.yaw_error =
        std::abs(common::NormalizeAngleDifference(error.rotation().angle()));
    result.slip_score = lateral_error_weight_ * result.lateral_error +
                        yaw_error_weight_ * result.yaw_error;
    result.gated_slip_score = lidar_reliability_gate_enabled_
                                  ? result.lidar_reliability * result.slip_score
                                  : result.slip_score;

    const bool lidar_reliable_enough =
        !lidar_reliability_gate_enabled_ ||
        (lidar_reliability_available &&
         result.lidar_reliability >= lidar_reliability_min_);
    if (!lidar_reliable_enough && unknown_keep_previous_weight_) {
      // [Innovation 2] Unknown hold：LiDAR
      // 横向/航向不可靠或没有时间对齐指标时， [Innovation 2]
      // 本条边只沿用上一权重，不更新 slipping 状态，避免把 [Innovation 2] LiDAR
      // 自身匹配不稳误判成轮式里程计打滑。
      return result;
    }

    if (enough_motion) {
      // [Innovation 2] 迟滞判定 hysteresis：高阈值触发 slipping，低阈值恢复
      // normal， [Innovation 2]
      // 中间区间保持上一状态，防止权重在临界值附近频繁抖动。
      if (result.gated_slip_score > high_threshold_) {
        slipping_ = true;
      } else if (result.gated_slip_score < low_threshold_) {
        slipping_ = false;
      }
    }

    if (slipping_) {
      // [Innovation 2] 快速降权 fast attack：检测到打滑时立即降低里程计可信度。
      current_weight_scale_ = min_weight_scale_;
    } else {
      // [Innovation 2] 慢速恢复 slow recovery：一阶低通滤波 low-pass filter
      // [Innovation 2] 将权重平滑恢复到最大值，避免后端优化约束突然跳变。
      current_weight_scale_ = recovery_alpha_ * max_weight_scale_ +
                              (1. - recovery_alpha_) * current_weight_scale_;
    }

    result.slipping = slipping_;
    result.weight_scale = current_weight_scale_;
    return result;
  }

 private:
  const bool enabled_;
  const double lateral_error_weight_;
  const double yaw_error_weight_;
  const double high_threshold_;
  const double low_threshold_;
  const double min_weight_scale_;
  const double max_weight_scale_;
  const double recovery_alpha_;
  const double min_motion_distance_;
  const double min_motion_angle_;
  const bool lidar_reliability_gate_enabled_;
  const double lidar_reliability_min_;
  const bool unknown_keep_previous_weight_;

  bool slipping_;
  double current_weight_scale_;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_SLIP_DETECTOR_H_
