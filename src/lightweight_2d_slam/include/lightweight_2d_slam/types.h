#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace lightweight_2d_slam {

constexpr double kPi = 3.14159265358979323846;

inline double NormalizeAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

using PointCloud2D = std::vector<Point2D>;

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

inline Pose2D Compose(const Pose2D& lhs, const Pose2D& rhs) {
  const double c = std::cos(lhs.yaw);
  const double s = std::sin(lhs.yaw);
  return {lhs.x + c * rhs.x - s * rhs.y,
          lhs.y + s * rhs.x + c * rhs.y,
          NormalizeAngle(lhs.yaw + rhs.yaw)};
}

inline Pose2D Inverse(const Pose2D& pose) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {-c * pose.x - s * pose.y,
           s * pose.x - c * pose.y,
          NormalizeAngle(-pose.yaw)};
}

inline Pose2D Between(const Pose2D& from, const Pose2D& to) {
  return Compose(Inverse(from), to);
}

inline Point2D TransformPoint(const Pose2D& pose, const Point2D& point) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * point.x - s * point.y,
          pose.y + s * point.x + c * point.y};
}

inline double TranslationDistance(const Pose2D& lhs, const Pose2D& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline Pose2D InterpolatePose(const Pose2D& from, const Pose2D& to,
                             double alpha) {
  alpha = std::max(0.0, std::min(1.0, alpha));
  return {from.x + alpha * (to.x - from.x),
          from.y + alpha * (to.y - from.y),
          NormalizeAngle(from.yaw +
                         alpha * NormalizeAngle(to.yaw - from.yaw))};
}

inline Pose2D PropagateOptimizedPose(const Pose2D& previous_local_pose,
                                    const Pose2D& previous_optimized_pose,
                                    const Pose2D& current_local_pose) {
  return Compose(previous_optimized_pose,
                 Between(previous_local_pose, current_local_pose));
}

struct PropagatedPoseBlend {
  Pose2D start;
  Pose2D target;
  Pose2D published;
};

inline PropagatedPoseBlend PropagatePoseBlend(
    const Pose2D& previous_local_pose,
    const Pose2D& current_local_pose,
    const Pose2D& previous_blend_start,
    const Pose2D& previous_blend_target,
    double alpha) {
  PropagatedPoseBlend result;
  result.start = PropagateOptimizedPose(
      previous_local_pose, previous_blend_start, current_local_pose);
  result.target = PropagateOptimizedPose(
      previous_local_pose, previous_blend_target, current_local_pose);
  result.published = InterpolatePose(result.start, result.target, alpha);
  return result;
}

struct Keyframe {
  std::size_t id = 0;
  std::size_t submap_id = 0;
  double stamp = 0.0;
  Pose2D local_pose;
  Pose2D odom_pose;
  PointCloud2D points;
  std::vector<float> descriptor;
  double scan_quality = 1.0;
};

}  // namespace lightweight_2d_slam
