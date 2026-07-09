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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_

#include <memory>
#include <mutex>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/directional_degeneracy_metric.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

// Align scans with an existing map using Ceres.
//
// 中文导读：
// CeresScanMatcher2D 是前端局部定位的“精匹配”环节。它接收：
//   - initial_pose_estimate：外推器/实时相关匹配给出的初值；
//   - point_cloud：当前 /scan 过滤后的点；
//   - grid：当前 submap 的概率栅格或 TSDF。
// 优化目标由三部分组成：点落在已占用区域的代价、偏离预测平移的代价、偏离预测
// 旋转的代价。输出 pose_estimate 会反馈给 LocalTrajectoryBuilder2D，并用于
// 插入 submap 和生成后端 trajectory node。
class CeresScanMatcher2D {
 public:
  explicit CeresScanMatcher2D(const proto::CeresScanMatcherOptions2D& options);
  virtual ~CeresScanMatcher2D();

  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const Eigen::Vector2d& target_translation,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, const Grid2D& grid,
             transform::Rigid2d* pose_estimate,
             ceres::Solver::Summary* summary,
             double real_time_correlative_score = 0.) const;

 private:
  // [Innovation 1] Computes the scan-covariance driven 2x2 sqrt information.
  // [Innovation 1] Disabled or underconstrained scans return isotropic weight.
  Eigen::Matrix<double, 2, 2> ComputeAnisotropicTranslationSqrtInformation(
      const sensor::PointCloud& point_cloud,
      double real_time_correlative_score) const;

  const proto::CeresScanMatcherOptions2D options_;
  ceres::Solver::Options ceres_solver_options_;

  // [Innovation 1] Protects the smoothed metric used by consecutive scans.
  mutable std::mutex metric_mutex_;
  mutable DirectionalDegeneracyMetric smoothed_metric_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
