#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_SLIP_DETECTOR_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_SLIP_DETECTOR_H_

#include <algorithm>
#include <cmath>

#include "Eigen/Core"
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
  double translation_weight_scale = 1.;
  double rotation_weight_scale = 1.;
  // Legacy aggregate scale retained for diagnostics. It is the more
  // conservative of the two component scales.
  double weight_scale = 1.;
};

inline double ComputeObservableTranslationError(
    const Eigen::Vector2d& translation_error,
    const Eigen::Vector2d& degeneracy_direction,
    const double adaptation_strength) {
  if (!translation_error.allFinite() ||
      !degeneracy_direction.allFinite() ||
      degeneracy_direction.norm() < 1e-6) {
    return translation_error.norm();
  }
  const Eigen::Vector2d normalized_direction =
      degeneracy_direction.normalized();
  const Eigen::Vector2d observable_direction(-normalized_direction.y(),
                                             normalized_direction.x());
  const double strength =
      std::min(1., std::max(0., adaptation_strength));
  return (1. - strength) * translation_error.norm() +
         strength * std::abs(translation_error.dot(observable_direction));
}

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
        min_consecutive_anomalies_(
            std::max(1, options.slip_min_consecutive_anomalies())),
        translation_slipping_(false),
        rotation_slipping_(false),
        current_translation_weight_scale_(max_weight_scale_),
        current_rotation_weight_scale_(max_weight_scale_),
        consecutive_translation_anomaly_count_(0),
        consecutive_rotation_anomaly_count_(0) {}

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
  // [Innovation 2] lidar_reliability comes from the time-aligned scan-match
  // quality cached with Innovation 1's directional metric.
  // [Innovation 2] 当可靠性未知或低于阈值时，不能用 LiDAR-odom 差异武断判定
  // [Innovation 2] wheel slip；策略由 unknown_keep_previous_weight 控制。
  SlipDetectorResult Update(const transform::Rigid2d& scan_relative,
                            const transform::Rigid2d& odom_relative,
                            const bool lidar_reliability_available,
                            const double lidar_reliability) {
    // We are deciding whether the wheel constraint is suspect, so a LiDAR-only
    // jump is not evidence of wheel motion. Require odometry itself to report
    // enough motion before updating the anomaly state.
    const double motion_distance = odom_relative.translation().norm();
    const double motion_angle = std::abs(odom_relative.rotation().angle());
    // [Innovation 2] 残差在激光相对运动坐标系中表达：
    // [Innovation 2] E_ij = inv(T_scan_ij) * T_odom_ij。
    // [Innovation 2] E_ij.translation().y() 是横向不一致量 lateral
    // inconsistency； [Innovation 2] yaw(E_ij) 是航向不一致量 heading
    // inconsistency。 [Innovation 2] 这样避免直接比较世界坐标 y
    // 导致坐标系含义混乱。
    const transform::Rigid2d error = scan_relative.inverse() * odom_relative;
    return UpdateFromResiduals(
        std::abs(error.translation().y()),
        std::abs(common::NormalizeAngleDifference(error.rotation().angle())),
        motion_distance, motion_angle, lidar_reliability_available,
        lidar_reliability);
  }

  // Direction-aware entry point. The caller expresses the trusted translation
  // residual in a common frame before passing it here, so this state machine is
  // independent of the scan/odometry coordinate convention.
  SlipDetectorResult UpdateFromResiduals(
      const double observable_translation_error, const double yaw_error,
      const double motion_distance, const double motion_angle,
      const bool lidar_reliability_available,
      const double lidar_reliability) {
    SlipDetectorResult result;
    result.enabled = enabled_;
    result.translation_weight_scale = current_translation_weight_scale_;
    result.rotation_weight_scale = current_rotation_weight_scale_;
    result.weight_scale = std::min(result.translation_weight_scale,
                                   result.rotation_weight_scale);
    result.slipping = translation_slipping_ || rotation_slipping_;
    result.lidar_reliability_available =
        !lidar_reliability_gate_enabled_ || lidar_reliability_available;
    result.lidar_reliability =
        lidar_reliability_gate_enabled_
            ? std::min(1., std::max(0., lidar_reliability))
            : 1.;
    result.lateral_error = std::max(0., observable_translation_error);
    result.yaw_error = std::max(0., yaw_error);
    result.slip_score = lateral_error_weight_ * result.lateral_error +
                        yaw_error_weight_ * result.yaw_error;
    result.gated_slip_score = lidar_reliability_gate_enabled_
                                  ? result.lidar_reliability * result.slip_score
                                  : result.slip_score;
    if (!enabled_) {
      return result;
    }

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
    if (!lidar_reliable_enough) {
      // Conservative fallback: without trustworthy LiDAR evidence, do not keep
      // a stale low odometry weight. Restore the exact baseline constraint.
      RestoreBaseline();
      SetResultState(&result);
      return result;
    }

    const bool enough_motion = motion_distance >= min_motion_distance_ ||
                               motion_angle >= min_motion_angle_;
    if (!enough_motion) {
      // A stationary wheel edge provides no evidence for weakening odometry.
      // It must also clear stale low weights left by a preceding event.
      RestoreBaseline();
      SetResultState(&result);
      return result;
    }

    const double reliability_scale =
        lidar_reliability_gate_enabled_ ? result.lidar_reliability : 1.;
    const double gated_translation_score =
        reliability_scale * lateral_error_weight_ * result.lateral_error;
    const double gated_rotation_score =
        reliability_scale * yaw_error_weight_ * result.yaw_error;
    UpdateComponent(gated_translation_score,
                    &consecutive_translation_anomaly_count_,
                    &translation_slipping_,
                    &current_translation_weight_scale_);
    UpdateComponent(gated_rotation_score,
                    &consecutive_rotation_anomaly_count_,
                    &rotation_slipping_, &current_rotation_weight_scale_);
    SetResultState(&result);
    return result;
  }

 private:
  void RestoreBaseline() {
    translation_slipping_ = false;
    rotation_slipping_ = false;
    consecutive_translation_anomaly_count_ = 0;
    consecutive_rotation_anomaly_count_ = 0;
    current_translation_weight_scale_ = max_weight_scale_;
    current_rotation_weight_scale_ = max_weight_scale_;
  }

  void UpdateComponent(const double gated_score, int* const consecutive_count,
                       bool* const slipping,
                       double* const current_weight_scale) {
    if (gated_score > high_threshold_) {
      *consecutive_count =
          std::min(min_consecutive_anomalies_, *consecutive_count + 1);
      if (*consecutive_count >= min_consecutive_anomalies_) {
        *slipping = true;
      }
    } else if (gated_score < low_threshold_) {
      *slipping = false;
      *consecutive_count = 0;
    } else if (!*slipping) {
      *consecutive_count = 0;
    }

    if (*slipping) {
      const double target_weight_scale =
          high_threshold_ > 0.
              ? std::min(max_weight_scale_,
                         std::max(min_weight_scale_,
                                  max_weight_scale_ * high_threshold_ /
                                      std::max(gated_score, high_threshold_)))
              : min_weight_scale_;
      *current_weight_scale =
          std::min(*current_weight_scale, target_weight_scale);
    } else {
      *current_weight_scale =
          recovery_alpha_ * max_weight_scale_ +
          (1. - recovery_alpha_) * *current_weight_scale;
    }
  }

  void SetResultState(SlipDetectorResult* const result) const {
    result->translation_weight_scale = current_translation_weight_scale_;
    result->rotation_weight_scale = current_rotation_weight_scale_;
    result->weight_scale = std::min(result->translation_weight_scale,
                                    result->rotation_weight_scale);
    result->slipping = translation_slipping_ || rotation_slipping_;
  }

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
  const int min_consecutive_anomalies_;

  bool translation_slipping_;
  bool rotation_slipping_;
  double current_translation_weight_scale_;
  double current_rotation_weight_scale_;
  int consecutive_translation_anomaly_count_;
  int consecutive_rotation_anomaly_count_;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_SLIP_DETECTOR_H_
