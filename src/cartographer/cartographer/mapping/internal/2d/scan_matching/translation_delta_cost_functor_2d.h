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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes the cost of translating 'pose' to 'target_translation'.
// Cost increases with the solution's distance from 'target_translation'.
class TranslationDeltaCostFunctor2D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const Eigen::Vector2d& target_translation) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals */,
                                           3 /* pose variables */>(
        new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
  }

  // [Innovation 1] Creates an anisotropic translation prior.
  // [Innovation 1] The residual uses a precomputed double-precision 2x2 sqrt
  // [Innovation 1] information matrix.
  // [Innovation 1] Formula: r = S * (p - p0),
  // [Innovation 1] S = R * diag(w_long, w_lat) * R^T.
  // [Innovation 1] The eigensystem R is computed outside the AutoDiff functor
  // [Innovation 1] from the current 2D scan covariance.
  // [Innovation 1] Keeping S as Eigen::Matrix<double, 2, 2> avoids running
  // [Innovation 1] eigen decomposition or matrix square roots on Ceres Jet
  // [Innovation 1] types.
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const Eigen::Matrix<double, 2, 2>& sqrt_information,
      const Eigen::Vector2d& target_translation) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals */,
                                           3 /* pose variables */>(
        new TranslationDeltaCostFunctor2D(sqrt_information,
                                          target_translation));
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    // [Innovation 1] The matrix stays double; Eigen multiplies its double
    // [Innovation 1] coefficients with the templated Ceres scalar T.
    const Eigen::Matrix<T, 2, 1> error(pose[0] - T(x_), pose[1] - T(y_));
    const Eigen::Matrix<T, 2, 1> weighted_error =
        sqrt_information_ * error;
    residual[0] = weighted_error.x();
    residual[1] = weighted_error.y();
    return true;
  }

 private:
  // Constructs a new TranslationDeltaCostFunctor2D from the given
  // 'target_translation' (x, y).
  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d& target_translation)
      : sqrt_information_(Eigen::Matrix<double, 2, 2>::Identity() *
                          scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()) {}

  // [Innovation 1] Constructs a directional 2D translation prior.
  // [Innovation 1] The matrix is already the residual sqrt information.
  explicit TranslationDeltaCostFunctor2D(
      const Eigen::Matrix<double, 2, 2>& sqrt_information,
      const Eigen::Vector2d& target_translation)
      : sqrt_information_(sqrt_information),
        x_(target_translation.x()),
        y_(target_translation.y()) {}

  TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
  TranslationDeltaCostFunctor2D& operator=(
      const TranslationDeltaCostFunctor2D&) = delete;

  // [Innovation 1] Constant double matrix used by AutoDiff.
  const Eigen::Matrix<double, 2, 2> sqrt_information_;
  const double x_;
  const double y_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
