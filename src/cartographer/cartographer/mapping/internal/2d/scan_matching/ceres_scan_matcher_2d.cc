/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "absl/types/optional.h"
#include "cartographer/common/internal/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

// [Innovation 1] Global latest metric storage for cartographer_ros topics.
// [Innovation 1] A plain mutex avoids local SLAM vs ROS timer data races.
std::mutex* LatestMetricMutex() {
  static auto* mutex = new std::mutex;
  return mutex;
}

// [Innovation 1] Global latest metric value.
// [Innovation 1] Function-local static avoids initialization order issues.
::cartographer::mapping::DirectionalDegeneracyMetric* LatestMetric() {
  static auto* metric = new DirectionalDegeneracyMetric;
  return metric;
}

// [Innovation 1] Bounded history for time-aligned degeneracy lookup.
// [Innovation 1] Front-end local SLAM writes this buffer, backend optimization
// [Innovation 1] reads it. A dedicated mutex keeps the lock scope small and
// [Innovation 1] avoids coupling it to Cartographer's larger pose-graph locks.
std::mutex* MetricHistoryMutex() {
  static auto* mutex = new std::mutex;
  return mutex;
}

std::deque<::cartographer::mapping::DirectionalDegeneracyMetric>*
MetricHistory() {
  static auto* history =
      new std::deque<::cartographer::mapping::DirectionalDegeneracyMetric>;
  return history;
}

constexpr size_t kMaxMetricHistorySize = 5000;

// [Innovation 1] Numerically stable sigmoid.
// [Innovation 1] This avoids hard threshold switching near corridor cases.
double Sigmoid(const double x) {
  if (x >= 0.) {
    const double z = std::exp(-x);
    return 1. / (1. + z);
  }
  const double z = std::exp(x);
  return z / (1. + z);
}

// [Innovation 1] Publishes a thread-safe metric snapshot for ROS.
void SetLatestDirectionalDegeneracyMetric(
    const ::cartographer::mapping::DirectionalDegeneracyMetric& metric) {
  std::lock_guard<std::mutex> lock(*LatestMetricMutex());
  *LatestMetric() = metric;
}

}  // namespace

DirectionalDegeneracyMetric GetLatestDirectionalDegeneracyMetric() {
  std::lock_guard<std::mutex> lock(*LatestMetricMutex());
  return *LatestMetric();
}

void RecordDirectionalDegeneracyMetric(DirectionalDegeneracyMetric metric,
                                       const common::Time time) {
  // [Innovation 1] LocalTrajectoryBuilder2D owns the sensor timestamp, so it
  // [Innovation 1] stamps the scan matcher metric here before the backend reads
  // [Innovation 1] it. Disabled metrics are still ignored by callers.
  metric.time = time;
  {
    std::lock_guard<std::mutex> lock(*MetricHistoryMutex());
    auto* const history = MetricHistory();
    if (!history->empty() && history->back().time > metric.time) {
      // [Innovation 1] Keep the buffer monotonic for lower_bound. This rare
      // [Innovation 1] branch handles bag timestamp jumps without exposing
      // [Innovation 1] partially ordered data to the backend thread.
      history->clear();
    }
    history->push_back(metric);
    while (history->size() > kMaxMetricHistorySize) {
      history->pop_front();
    }
  }
  SetLatestDirectionalDegeneracyMetric(metric);
}

absl::optional<DirectionalDegeneracyMetric> QueryDirectionalDegeneracyMetric(
    const common::Time time, const common::Duration max_delta) {
  std::lock_guard<std::mutex> lock(*MetricHistoryMutex());
  const auto* const history = MetricHistory();
  if (history->empty()) {
    return absl::nullopt;
  }
  const auto lower = std::lower_bound(
      history->begin(), history->end(), time,
      [](const DirectionalDegeneracyMetric& metric,
         const common::Time query_time) { return metric.time < query_time; });

  absl::optional<DirectionalDegeneracyMetric> best;
  common::Duration best_delta = max_delta + common::FromSeconds(1.);
  if (lower != history->end()) {
    const common::Duration delta =
        lower->time > time ? lower->time - time : time - lower->time;
    if (delta <= max_delta && delta < best_delta) {
      best_delta = delta;
      best = *lower;
    }
  }
  if (lower != history->begin()) {
    const auto previous = std::prev(lower);
    const common::Duration delta =
        previous->time > time ? previous->time - time : time - previous->time;
    if (delta <= max_delta && delta < best_delta) {
      best_delta = delta;
      best = *previous;
    }
  }
  return best;
}

namespace scan_matching {

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions2D options;
  options.set_occupied_space_weight(
      parameter_dictionary->GetDouble("occupied_space_weight"));
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  // [Innovation 1] Parameters remain optional for upstream compatibility.
  // [Innovation 1] Project Lua files still declare them explicitly.
  options.set_directional_adaptive_fusion_enabled(
      parameter_dictionary->HasKey("directional_adaptive_fusion_enabled")
          ? parameter_dictionary->GetBool("directional_adaptive_fusion_enabled")
          : false);
  options.set_directional_degeneracy_condition_number_threshold(
      parameter_dictionary->HasKey(
          "directional_degeneracy_condition_number_threshold")
          ? parameter_dictionary->GetDouble(
                "directional_degeneracy_condition_number_threshold")
          : 8.);
  options.set_directional_degeneracy_sigmoid_slope(
      parameter_dictionary->HasKey("directional_degeneracy_sigmoid_slope")
          ? parameter_dictionary->GetDouble(
                "directional_degeneracy_sigmoid_slope")
          : 0.5);
  options.set_directional_degeneracy_smoothing_alpha(
      parameter_dictionary->HasKey("directional_degeneracy_smoothing_alpha")
          ? parameter_dictionary->GetDouble(
                "directional_degeneracy_smoothing_alpha")
          : 0.2);
  options.set_directional_degeneracy_min_num_points(
      parameter_dictionary->HasKey("directional_degeneracy_min_num_points")
          ? parameter_dictionary->GetNonNegativeInt(
                "directional_degeneracy_min_num_points")
          : 10);
  options.set_directional_degeneracy_eigenvalue_epsilon(
      parameter_dictionary->HasKey("directional_degeneracy_eigenvalue_epsilon")
          ? parameter_dictionary->GetDouble(
                "directional_degeneracy_eigenvalue_epsilon")
          : 1e-4);
  options.set_directional_adaptive_odom_longitudinal_alpha(
      parameter_dictionary->HasKey(
          "directional_adaptive_odom_longitudinal_alpha")
          ? parameter_dictionary->GetDouble(
                "directional_adaptive_odom_longitudinal_alpha")
          : 1.);
  options.set_directional_adaptive_odom_lateral_alpha(
      parameter_dictionary->HasKey("directional_adaptive_odom_lateral_alpha")
          ? parameter_dictionary->GetDouble(
                "directional_adaptive_odom_lateral_alpha")
          : 0.);
  options.set_directional_adaptive_scan_longitudinal_beta(
      parameter_dictionary->HasKey(
          "directional_adaptive_scan_longitudinal_beta")
          ? parameter_dictionary->GetDouble(
                "directional_adaptive_scan_longitudinal_beta")
          : 0.5);
  options.set_directional_adaptive_scan_lateral_beta(
      parameter_dictionary->HasKey("directional_adaptive_scan_lateral_beta")
          ? parameter_dictionary->GetDouble(
                "directional_adaptive_scan_lateral_beta")
          : 0.);
  options.set_directional_adaptive_min_scan_weight_scale(
      parameter_dictionary->HasKey("directional_adaptive_min_scan_weight_scale")
          ? parameter_dictionary->GetDouble(
                "directional_adaptive_min_scan_weight_scale")
          : 0.2);
  options.set_directional_adaptive_log_scale_change_threshold(
      parameter_dictionary->HasKey(
          "directional_adaptive_log_scale_change_threshold")
          ? parameter_dictionary->GetDouble(
                "directional_adaptive_log_scale_change_threshold")
          : 0.25);
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

CeresScanMatcher2D::CeresScanMatcher2D(
    const proto::CeresScanMatcherOptions2D& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

Eigen::Matrix<double, 2, 2>
CeresScanMatcher2D::ComputeAnisotropicTranslationSqrtInformation(
    const sensor::PointCloud& point_cloud,
    const double real_time_correlative_score) const {
  const double base_translation_weight = options_.translation_weight();
  Eigen::Matrix<double, 2, 2> sqrt_information =
      Eigen::Matrix<double, 2, 2>::Identity() * base_translation_weight;

  DirectionalDegeneracyMetric metric;
  metric.real_time_correlative_score = real_time_correlative_score;
  metric.direction = Eigen::Vector2d::UnitX();

  if (!options_.directional_adaptive_fusion_enabled()) {
    SetLatestDirectionalDegeneracyMetric(metric);
    return sqrt_information;
  }

  metric.enabled = true;
  const int min_num_points =
      std::max(10, options_.directional_degeneracy_min_num_points());
  if (static_cast<int>(point_cloud.size()) < min_num_points) {
    SetLatestDirectionalDegeneracyMetric(metric);
    return sqrt_information;
  }

  // [Innovation 1] Compute 2D scan covariance in the current scan frame.
  // [Innovation 1] This is O(N) and uses only x/y, without IMU logic.
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const sensor::RangefinderPoint& point : point_cloud) {
    mean += point.position.head<2>().cast<double>();
  }
  mean /= static_cast<double>(point_cloud.size());

  Eigen::Matrix<double, 2, 2> covariance = Eigen::Matrix<double, 2, 2>::Zero();
  for (const sensor::RangefinderPoint& point : point_cloud) {
    const Eigen::Vector2d centered =
        point.position.head<2>().cast<double>() - mean;
    covariance += centered * centered.transpose();
  }
  covariance /= static_cast<double>(point_cloud.size());

  const double eigenvalue_epsilon =
      std::max(1e-9, options_.directional_degeneracy_eigenvalue_epsilon());
  covariance += Eigen::Matrix<double, 2, 2>::Identity() * eigenvalue_epsilon;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 2, 2>> solver(covariance);
  if (solver.info() != Eigen::Success) {
    SetLatestDirectionalDegeneracyMetric(metric);
    return sqrt_information;
  }

  const double lambda_min =
      std::max(eigenvalue_epsilon, solver.eigenvalues()(0));
  const double lambda_max =
      std::max(eigenvalue_epsilon, solver.eigenvalues()(1));
  const double condition_number = lambda_max / lambda_min;
  Eigen::Vector2d v_lat = solver.eigenvectors().col(0).normalized();
  Eigen::Vector2d v_long = solver.eigenvectors().col(1).normalized();
  if (!v_long.allFinite() || !v_lat.allFinite()) {
    SetLatestDirectionalDegeneracyMetric(metric);
    return sqrt_information;
  }

  // [Innovation 1] Eigenvector signs are arbitrary.
  // [Innovation 1] Align with the previous direction before smoothing.
  DirectionalDegeneracyMetric previous_metric;
  {
    std::lock_guard<std::mutex> lock(metric_mutex_);
    previous_metric = smoothed_metric_;
  }
  if (previous_metric.enabled && previous_metric.direction.dot(v_long) < 0.) {
    v_long = -v_long;
    v_lat = -v_lat;
  }

  Eigen::Matrix<double, 2, 2> rotation;
  rotation.col(0) = v_long;
  rotation.col(1) = v_lat;
  if (rotation.determinant() < 0.) {
    rotation.col(1) = -rotation.col(1);
  }

  const double sigmoid_input =
      options_.directional_degeneracy_sigmoid_slope() *
      (condition_number -
       options_.directional_degeneracy_condition_number_threshold());
  const double raw_confidence = Sigmoid(sigmoid_input);
  const double smoothing_alpha = std::min(
      1., std::max(0., options_.directional_degeneracy_smoothing_alpha()));
  const double confidence =
      previous_metric.enabled
          ? smoothing_alpha * raw_confidence +
                (1. - smoothing_alpha) * previous_metric.confidence
          : raw_confidence;
  Eigen::Vector2d smoothed_direction =
      previous_metric.enabled
          ? smoothing_alpha * v_long +
                (1. - smoothing_alpha) * previous_metric.direction
          : v_long;
  if (smoothed_direction.norm() < 1e-6 || !smoothed_direction.allFinite()) {
    smoothed_direction = v_long;
  }
  smoothed_direction.normalize();

  rotation.col(0) = smoothed_direction;
  rotation.col(1) =
      Eigen::Vector2d(-smoothed_direction.y(), smoothed_direction.x());

  // [Innovation 1] The occupied-space residual is scalar per laser point.
  // [Innovation 1] Directional fusion is injected through the 2D motion prior.
  // [Innovation 1] scan_*_scale changes relative LiDAR influence without
  // [Innovation 1] changing the grid cost function ABI.
  const double min_scan_scale =
      std::max(1e-3, options_.directional_adaptive_min_scan_weight_scale());
  const double scan_long_scale = std::max(
      min_scan_scale,
      1. - options_.directional_adaptive_scan_longitudinal_beta() * confidence);
  const double scan_lat_scale = std::max(
      min_scan_scale,
      1. - options_.directional_adaptive_scan_lateral_beta() * confidence);
  const double odom_long_scale =
      1. + options_.directional_adaptive_odom_longitudinal_alpha() * confidence;
  const double odom_lat_scale =
      1. + options_.directional_adaptive_odom_lateral_alpha() * confidence;
  const double longitudinal_weight =
      base_translation_weight * odom_long_scale / scan_long_scale;
  const double lateral_weight =
      base_translation_weight * odom_lat_scale / scan_lat_scale;

  sqrt_information =
      rotation *
      (Eigen::Vector2d(longitudinal_weight, lateral_weight).asDiagonal()) *
      rotation.transpose();

  metric.confidence = confidence;
  metric.condition_number = condition_number;
  metric.direction = smoothed_direction;
  metric.longitudinal_scale = longitudinal_weight / base_translation_weight;
  metric.lateral_scale = lateral_weight / base_translation_weight;

  {
    std::lock_guard<std::mutex> lock(metric_mutex_);
    const double log_threshold = std::max(
        0., options_.directional_adaptive_log_scale_change_threshold());
    const bool scale_changed =
        std::abs(metric.longitudinal_scale -
                 smoothed_metric_.longitudinal_scale) > log_threshold ||
        std::abs(metric.lateral_scale - smoothed_metric_.lateral_scale) >
            log_threshold;
    if (confidence > 0.5 && scale_changed) {
      LOG(INFO) << "[Innovation1] Directional Degeneracy detected! "
                << "longitudinal weight scaled by " << metric.longitudinal_scale
                << ", lateral weight scaled by " << metric.lateral_scale << ".";
    }
    smoothed_metric_ = metric;
  }
  SetLatestDirectionalDegeneracyMetric(metric);
  return sqrt_information;
}

void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                               const transform::Rigid2d& initial_pose_estimate,
                               const sensor::PointCloud& point_cloud,
                               const Grid2D& grid,
                               transform::Rigid2d* const pose_estimate,
                               ceres::Solver::Summary* const summary,
                               const double real_time_correlative_score) const {
  // 优化变量只有 3 个：x、y、yaw。Cartographer 2D 已经在前面做了重力对齐，
  // 所以这里不估计 z、roll、pitch。
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  CHECK_GT(options_.occupied_space_weight(), 0.);
  switch (grid.GetGridType()) {
    case GridType::PROBABILITY_GRID:
      // 占用空间残差：把当前点云按候选位姿投到概率栅格上，点越落在高占用概率
      // 区域，代价越小。这个项负责让 /scan 的墙线贴到 submap 的墙线上。
      problem.AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, grid),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
    case GridType::TSDF:
      // TSDF 模式下使用到障碍边界的截断符号距离作为匹配代价，语义类似：点云
      // 越贴近已有表面，匹配越好。
      problem.AddResidualBlock(
          CreateTSDFMatchCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, static_cast<const TSDF2D&>(grid)),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
  }
  CHECK_GT(options_.translation_weight(), 0.);
  // 平移先验：不要让 scan matcher 为了贴合局部地图而无限偏离外推位姿。
  // 这个约束能抑制走廊、重复结构等场景中的错误跳变。
  // [Innovation 1] Use scan-covariance-derived anisotropic sqrt information.
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          ComputeAnisotropicTranslationSqrtInformation(
              point_cloud, real_time_correlative_score),
          target_translation),
      nullptr /* loss function */, ceres_pose_estimate);
  CHECK_GT(options_.rotation_weight(), 0.);
  // 旋转先验：限制 yaw 偏离初值。这里使用当前 ceres_pose_estimate[2] 作为目标
  // 角度，即 initial_pose_estimate 的角度。
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.rotation_weight(), ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  // 把优化后的 3 个标量重新封装为 Rigid2d，返回给 LocalTrajectoryBuilder2D。
  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
