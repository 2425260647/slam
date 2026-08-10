#include "lightweight_2d_slam/scan_matcher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Eigenvalues>
#include <ceres/ceres.h>

namespace lightweight_2d_slam {
namespace {

struct PointToLineResidual {
  PointToLineResidual(const Point2D& point, const Point2D& centroid,
                      const Point2D& normal, double weight = 1.0)
      : point(point), centroid(centroid), normal(normal), weight(weight) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    const T c = ceres::cos(pose[2]);
    const T s = ceres::sin(pose[2]);
    const T transformed_x = pose[0] + c * T(point.x) - s * T(point.y);
    const T transformed_y = pose[1] + s * T(point.x) + c * T(point.y);
    residual[0] = T(weight) *
                  (T(normal.x) * (transformed_x - T(centroid.x)) +
                   T(normal.y) * (transformed_y - T(centroid.y)));
    return true;
  }

  Point2D point;
  Point2D centroid;
  Point2D normal;
  double weight;
};

class LikelihoodFieldResidual final
    : public ceres::SizedCostFunction<1, 3> {
 public:
  LikelihoodFieldResidual(const ProbabilityGrid* grid, const Point2D& point,
                          int radius_cells, double weight)
      : grid_(grid), point_(point), radius_cells_(radius_cells),
        weight_(weight) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double* const pose = parameters[0];
    const double cosine = std::cos(pose[2]);
    const double sine = std::sin(pose[2]);
    const Point2D world_point{
        pose[0] + cosine * point_.x - sine * point_.y,
        pose[1] + sine * point_.x + cosine * point_.y};
    Point2D gradient;
    const double likelihood = grid_->LikelihoodScoreAndGradient(
        world_point, radius_cells_, &gradient);
    residuals[0] = weight_ * (1.0 - likelihood);

    if (jacobians != nullptr && jacobians[0] != nullptr) {
      double* const jacobian = jacobians[0];
      jacobian[0] = -weight_ * gradient.x;
      jacobian[1] = -weight_ * gradient.y;
      const double dx_dyaw = -sine * point_.x - cosine * point_.y;
      const double dy_dyaw = cosine * point_.x - sine * point_.y;
      jacobian[2] = -weight_ *
                    (gradient.x * dx_dyaw + gradient.y * dy_dyaw);
    }
    return true;
  }

 private:
  const ProbabilityGrid* grid_;
  Point2D point_;
  int radius_cells_;
  double weight_;
};

struct PosePriorResidual {
  PosePriorResidual(const Pose2D& prior, double translation_weight,
                    double rotation_weight)
      : prior(prior), translation_weight(translation_weight),
        rotation_weight(rotation_weight) {}

  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    residuals[0] = T(translation_weight) * (pose[0] - T(prior.x));
    residuals[1] = T(translation_weight) * (pose[1] - T(prior.y));
    residuals[2] = T(rotation_weight) * ceres::sin(pose[2] - T(prior.yaw));
    return true;
  }

  Pose2D prior;
  double translation_weight;
  double rotation_weight;
};

PointCloud2D UniformSample(const PointCloud2D& points, int maximum_size) {
  if (maximum_size <= 0 || static_cast<int>(points.size()) <= maximum_size) {
    return points;
  }
  PointCloud2D sampled;
  sampled.reserve(static_cast<std::size_t>(maximum_size));
  const double stride = static_cast<double>(points.size()) /
                        static_cast<double>(maximum_size);
  for (int i = 0; i < maximum_size; ++i) {
    const auto index = static_cast<std::size_t>(std::floor(i * stride));
    sampled.push_back(points[std::min(index, points.size() - 1)]);
  }
  return sampled;
}

double EvaluateMapScore(const ProbabilityGrid& grid,
                        const PointCloud2D& points,
                        const Pose2D& pose,
                        int maximum_size,
                        int endpoint_neighborhood,
                        bool use_likelihood_field) {
  const PointCloud2D sampled = UniformSample(points, maximum_size);
  if (sampled.empty()) return 0.0;
  double score = 0.0;
  for (const auto& point : sampled) {
    const Point2D transformed = TransformPoint(pose, point);
    score += use_likelihood_field
                 ? grid.LikelihoodScore(transformed, endpoint_neighborhood)
                 : grid.EndpointScore(transformed, endpoint_neighborhood);
  }
  return score / static_cast<double>(sampled.size());
}

}  // namespace

ScanMatchResult ScanMatcher::Match(const ProbabilityGrid& grid,
                                   const PointCloud2D& points,
                                   const Pose2D& initial_pose) const {
  return MatchWithWindows(grid, points, initial_pose,
                          options_.linear_search_window,
                          options_.angular_search_window);
}

ScanMatchResult ScanMatcher::MatchWithWindows(const ProbabilityGrid& grid,
                                              const PointCloud2D& points,
                                              const Pose2D& initial_pose,
                                              double linear_window,
                                              double angular_window) const {
  const int candidate_count = options_.enable_oad_csm
                                  ? std::max(2, options_.oad_candidate_count)
                                  : (options_.enable_fine_correlative
                                         ? options_.fine_candidate_count
                                         : 1);
  ScanMatchResult result = MatchCore(
      grid, points, initial_pose, linear_window, angular_window,
      candidate_count, !options_.enable_oad_csm);
  result.oad_search_window = linear_window;
  if (!options_.enable_oad_csm || grid.Empty() || points.empty()) return result;

  const bool ambiguous = result.score_margin < options_.oad_min_score_margin;
  const bool weak_translation =
      result.translation_observability <
      options_.oad_min_translation_observability;
  // A multi-peak score without a measured weak direction is not actionable:
  // expanding an arbitrary axis only increases the chance of selecting the
  // wrong peak. OAD is reserved for a weak direction plus ambiguity/failure.
  if (!weak_translation || (result.success && !ambiguous)) {
    if (ambiguous) result.oad_triggered = true;
    return result;
  }

  constexpr double kHalfPi = 1.57079632679489661923;
  const double strong_direction =
      std::isfinite(result.translation_strong_direction_yaw)
          ? result.translation_strong_direction_yaw
          : initial_pose.yaw;
  const double weak_direction = NormalizeAngle(strong_direction + kHalfPi);
  const double expansion_direction = weak_direction;
  const double expanded_linear = std::min(
      options_.oad_max_linear_search_window,
      std::max(linear_window,
               linear_window * options_.oad_search_expansion_factor));
  // Translation degeneracy does not justify widening yaw indiscriminately.
  const double expanded_angular = angular_window;
  if (expanded_linear <= linear_window + 1e-9) {
    result.oad_triggered = true;
    return result;
  }

  ScanMatchResult expanded = MatchCoreDirectional(
      grid, points, initial_pose, expansion_direction, expanded_linear,
      linear_window, expanded_angular, candidate_count, true);
  expanded.oad_triggered = true;
  expanded.oad_expanded = true;
  expanded.oad_search_window = expanded_linear;
  // Compare both hypotheses using the same prior normalization.  The
  // expanded search has a larger denominator when its coarse candidates are
  // scored, so comparing their stored prior_penalty values directly would
  // make a farther hypothesis look artificially cheap.
  const auto normalized_prior_penalty = [&](const Pose2D& pose) {
    const double normalized_translation =
        std::hypot(pose.x - initial_pose.x, pose.y - initial_pose.y) /
        std::max(0.01, linear_window);
    const double normalized_rotation =
        std::abs(NormalizeAngle(pose.yaw - initial_pose.yaw)) /
        std::max(0.01, angular_window);
    return options_.correlative_translation_cost_weight *
               normalized_translation * normalized_translation +
           options_.correlative_rotation_cost_weight * normalized_rotation *
               normalized_rotation;
  };
  const double result_combined_score =
      result.map_score - normalized_prior_penalty(result.pose);
  const double expanded_combined_score =
      expanded.map_score - normalized_prior_penalty(expanded.pose);
  const bool expanded_is_better =
      expanded.success &&
      (!result.success ||
       expanded_combined_score >=
           result_combined_score + options_.oad_min_score_gain) &&
      (result.residual_rms == 0.0 ||
       expanded.residual_rms <= result.residual_rms + 0.01);
  if (expanded_is_better) return expanded;
  result.oad_triggered = true;
  return result;
}

ScanMatchResult ScanMatcher::MatchCore(
    const ProbabilityGrid& grid, const PointCloud2D& points,
    const Pose2D& initial_pose, double linear_window, double angular_window,
    int candidate_count, bool refine_all_candidates) const {
  return MatchCoreDirectional(grid, points, initial_pose, 0.0, linear_window,
                              linear_window, angular_window, candidate_count,
                              refine_all_candidates);
}

ScanMatchResult ScanMatcher::MatchCoreDirectional(
    const ProbabilityGrid& grid, const PointCloud2D& points,
    const Pose2D& initial_pose, double axis_yaw, double axis_window,
    double cross_window, double angular_window, int candidate_count,
    bool refine_all_candidates) const {
  ScanMatchResult result;
  result.pose = initial_pose;
  if (grid.Empty() || points.empty()) return result;
  std::vector<CorrelativeCandidate> map_candidates;
  const std::vector<CorrelativeCandidate> coarse_candidates =
      CorrelativeCandidatesDirectional(
          grid, points, initial_pose, axis_yaw, axis_window, cross_window,
          angular_window, options_.linear_step, options_.angular_step,
          candidate_count, options_.enable_oad_csm ? &map_candidates : nullptr);
  if (coarse_candidates.empty()) return result;
  std::size_t selected_index = 0;
  Pose2D refined_seed = coarse_candidates.front().pose;
  double refined_score = coarse_candidates.front().score;
  if (options_.enable_fine_correlative) {
    refined_score = -std::numeric_limits<double>::infinity();
    const std::size_t requested_fine_count = refine_all_candidates
                                                 ? coarse_candidates.size()
                                                 : 1u;
    const std::size_t fine_count =
        std::min(coarse_candidates.size(), requested_fine_count);
    for (std::size_t index = 0; index < fine_count; ++index) {
      double candidate_score = 0.0;
      const auto& candidate = coarse_candidates[index];
      const Pose2D candidate_pose = CorrelativeMatch(
          grid, points, candidate.pose, options_.linear_step,
          options_.fine_angular_window, options_.fine_linear_step,
          options_.fine_angular_step, &candidate_score);
      if (candidate_score > refined_score) {
        refined_score = candidate_score;
        refined_seed = candidate_pose;
        selected_index = index;
      }
    }
  }
  result = Refine(grid, points, refined_seed, initial_pose, refined_score);
  const std::vector<CorrelativeCandidate>& margin_candidates =
      options_.enable_oad_csm ? map_candidates : coarse_candidates;
  result.score_margin = MapPeakMargin(
      margin_candidates, options_.linear_step, options_.angular_step);
  result.map_score = EvaluateMapScore(
      grid, points, result.pose, options_.max_coarse_points,
      options_.endpoint_neighborhood, options_.use_likelihood_field);
  result.prior_penalty = coarse_candidates[selected_index].prior_penalty;
  const double score_quality = std::max(0.0, std::min(1.0, result.map_score));
  const double residual_quality =
      std::isfinite(result.residual_rms)
          ? std::exp(-result.residual_rms /
                     std::max(1e-3, options_.refinement_huber_scale))
          : 0.0;
  const double correspondence_denominator = std::max(
      1.0, 0.60 * static_cast<double>(points.size()));
  const double correspondence_quality = std::max(
      0.0, std::min(1.0, static_cast<double>(result.correspondences) /
                              correspondence_denominator));
  result.match_quality = std::cbrt(
      std::max(0.0, score_quality * residual_quality *
                         correspondence_quality));
  return result;
}

double ScanMatcher::MapPeakMargin(
    const std::vector<CorrelativeCandidate>& candidates, double linear_step,
    double angular_step) const {
  if (candidates.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }
  std::vector<CorrelativeCandidate> ranked = candidates;
  std::sort(ranked.begin(), ranked.end(),
            [](const CorrelativeCandidate& lhs,
               const CorrelativeCandidate& rhs) {
              if (lhs.map_score != rhs.map_score) {
                return lhs.map_score > rhs.map_score;
              }
              return lhs.score > rhs.score;
            });
  const double minimum_translation_separation =
      std::max(0.10, 2.0 * linear_step);
  const double minimum_angular_separation = std::max(0.05, 2.0 * angular_step);
  const CorrelativeCandidate& best = ranked.front();
  for (std::size_t index = 1; index < ranked.size(); ++index) {
    const CorrelativeCandidate& candidate = ranked[index];
    if (std::hypot(candidate.pose.x - best.pose.x,
                   candidate.pose.y - best.pose.y) >=
            minimum_translation_separation ||
        std::abs(NormalizeAngle(candidate.pose.yaw - best.pose.yaw)) >=
            minimum_angular_separation) {
      return best.map_score - candidate.map_score;
    }
  }
  return std::numeric_limits<double>::infinity();
}

std::vector<ScanMatcher::CorrelativeCandidate>
ScanMatcher::CorrelativeCandidates(
    const ProbabilityGrid& grid, const PointCloud2D& points,
    const Pose2D& initial_pose, double linear_window, double angular_window,
    double linear_step, double angular_step, int candidate_count) const {
  return CorrelativeCandidatesDirectional(
      grid, points, initial_pose, 0.0, linear_window, linear_window,
      angular_window, linear_step, angular_step, candidate_count);
}

std::vector<ScanMatcher::CorrelativeCandidate>
ScanMatcher::CorrelativeCandidatesDirectional(
    const ProbabilityGrid& grid, const PointCloud2D& points,
    const Pose2D& initial_pose, double axis_yaw, double axis_window,
    double cross_window, double angular_window, double linear_step,
    double angular_step, int candidate_count,
    std::vector<CorrelativeCandidate>* map_candidates) const {
  const PointCloud2D sampled = UniformSample(points, options_.max_coarse_points);
  std::vector<CorrelativeCandidate> candidates;
  if (sampled.empty()) return candidates;
  const std::size_t keep =
      static_cast<std::size_t>(std::max(1, candidate_count));
  candidates.reserve(keep);
  if (map_candidates != nullptr) map_candidates->reserve(keep);
  const int axis_steps = std::max(
      0, static_cast<int>(std::ceil(axis_window / linear_step)));
  const int cross_steps = std::max(
      0, static_cast<int>(std::ceil(cross_window / linear_step)));
  const int angular_steps = std::max(
      0, static_cast<int>(std::ceil(angular_window / angular_step)));

  for (int yaw_step = -angular_steps; yaw_step <= angular_steps; ++yaw_step) {
    for (int cross_step = -cross_steps; cross_step <= cross_steps;
         ++cross_step) {
      for (int axis_step = -axis_steps; axis_step <= axis_steps;
           ++axis_step) {
        const double axis_offset = axis_step * linear_step;
        const double cross_offset = cross_step * linear_step;
        const double cosine = std::cos(axis_yaw);
        const double sine = std::sin(axis_yaw);
        Pose2D candidate_pose{
            initial_pose.x + axis_offset * cosine - cross_offset * sine,
            initial_pose.y + axis_offset * sine + cross_offset * cosine,
            NormalizeAngle(initial_pose.yaw + yaw_step * angular_step)};
        double map_score = 0.0;
        for (const auto& point : sampled) {
          const Point2D transformed = TransformPoint(candidate_pose, point);
          map_score += options_.use_likelihood_field
                           ? grid.LikelihoodScore(
                                 transformed, options_.endpoint_neighborhood)
                           : grid.EndpointScore(
                                 transformed, options_.endpoint_neighborhood);
        }
        map_score /= static_cast<double>(sampled.size());
        const double normalized_translation =
            std::hypot(axis_offset, cross_offset) /
            std::max(0.01, std::max(axis_window, cross_window));
        const double normalized_rotation =
            std::abs(yaw_step * angular_step) /
            std::max(0.01, angular_window);
        const double prior_penalty =
            options_.correlative_translation_cost_weight *
                normalized_translation * normalized_translation +
            options_.correlative_rotation_cost_weight * normalized_rotation *
                normalized_rotation;
        const CorrelativeCandidate candidate{
            candidate_pose, map_score - prior_penalty, map_score,
            prior_penalty};
        const auto insertion = std::find_if(
            candidates.begin(), candidates.end(),
            [&candidate](const CorrelativeCandidate& existing) {
              return candidate.score > existing.score;
            });
        if (insertion != candidates.end()) {
          candidates.insert(insertion, candidate);
        } else if (candidates.size() < keep) {
          candidates.push_back(candidate);
        }
        if (candidates.size() > keep) candidates.pop_back();
        if (map_candidates != nullptr) {
          const auto map_insertion = std::find_if(
              map_candidates->begin(), map_candidates->end(),
              [&candidate](const CorrelativeCandidate& existing) {
                if (candidate.map_score != existing.map_score) {
                  return candidate.map_score > existing.map_score;
                }
                return candidate.score > existing.score;
              });
          if (map_insertion != map_candidates->end()) {
            map_candidates->insert(map_insertion, candidate);
          } else if (map_candidates->size() < keep) {
            map_candidates->push_back(candidate);
          }
          if (map_candidates->size() > keep) map_candidates->pop_back();
        }
      }
    }
  }
  return candidates;
}

Pose2D ScanMatcher::CorrelativeMatch(const ProbabilityGrid& grid,
                                     const PointCloud2D& points,
                                     const Pose2D& initial_pose,
                                     double linear_window,
                                     double angular_window,
                                     double linear_step,
                                     double angular_step,
                                     double* best_score) const {
  const std::vector<CorrelativeCandidate> candidates =
      CorrelativeCandidates(
          grid, points, initial_pose, linear_window, angular_window,
          linear_step, angular_step, 1);
  if (candidates.empty()) {
    *best_score = -std::numeric_limits<double>::infinity();
    return initial_pose;
  }
  *best_score = candidates.front().score;
  return candidates.front().pose;
}

ScanMatchResult ScanMatcher::Refine(const ProbabilityGrid& grid,
                                    const PointCloud2D& points,
                                    const Pose2D& coarse_pose,
                                    const Pose2D& prediction_pose,
                                    double coarse_score) const {
  double pose[3] = {coarse_pose.x, coarse_pose.y, coarse_pose.yaw};
  int correspondence_count = 0;

  if (options_.use_likelihood_refinement &&
      options_.max_refinement_iterations > 0) {
    ceres::Problem problem;
    const PointCloud2D sampled = UniformSample(
        points, options_.max_likelihood_refinement_points);
    const double normalized_weight =
        options_.likelihood_refinement_weight /
        std::sqrt(static_cast<double>(std::max<std::size_t>(1, sampled.size())));
    for (const auto& point : sampled) {
      problem.AddResidualBlock(
          new LikelihoodFieldResidual(
              &grid, point, options_.endpoint_neighborhood,
              normalized_weight),
          new ceres::HuberLoss(options_.refinement_huber_scale), pose);
    }

    if (options_.use_hybrid_refinement) {
      const double line_weight =
          options_.point_line_refinement_weight /
          std::sqrt(static_cast<double>(std::max<std::size_t>(
              1, sampled.size())));
      for (const auto& point : sampled) {
        const Point2D world_point = TransformPoint(coarse_pose, point);
        Point2D centroid;
        Point2D normal;
        if (!grid.FitLine(world_point, options_.line_search_radius_cells,
                          options_.line_min_neighbors, &centroid, &normal,
                          options_.line_max_eigenvalue_ratio,
                          options_.line_min_eigenvalue)) {
          continue;
        }
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<PointToLineResidual, 1, 3>(
                new PointToLineResidual(point, centroid, normal,
                                        line_weight)),
            new ceres::HuberLoss(options_.refinement_huber_scale), pose);
      }
    }

    const Pose2D prior_pose{
        options_.use_prediction_translation_prior ? prediction_pose.x
                                                  : coarse_pose.x,
        options_.use_prediction_translation_prior ? prediction_pose.y
                                                  : coarse_pose.y,
        coarse_pose.yaw};
    auto* prior = new ceres::AutoDiffCostFunction<PosePriorResidual, 3, 3>(
        new PosePriorResidual(prior_pose, options_.prior_translation_weight,
                              options_.prior_rotation_weight));
    problem.AddResidualBlock(prior, nullptr, pose);

    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.max_num_iterations = options_.max_refinement_iterations;
    solver_options.num_threads = 1;
    solver_options.minimizer_progress_to_stdout = false;
    solver_options.function_tolerance = 1e-5;
    solver_options.parameter_tolerance = 1e-5;
    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);
    if (summary.IsSolutionUsable()) {
      pose[2] = NormalizeAngle(pose[2]);
    } else {
      pose[0] = coarse_pose.x;
      pose[1] = coarse_pose.y;
      pose[2] = coarse_pose.yaw;
    }
  } else {
    for (int outer_iteration = 0;
         outer_iteration < options_.max_refinement_iterations;
         ++outer_iteration) {
      ceres::Problem problem;
      correspondence_count = 0;
      for (const auto& point : points) {
        const Pose2D current_pose{pose[0], pose[1], pose[2]};
        const Point2D world_point = TransformPoint(current_pose, point);
        Point2D centroid;
        Point2D normal;
        if (!grid.FitLine(world_point, options_.line_search_radius_cells,
                          options_.line_min_neighbors, &centroid, &normal,
                          options_.line_max_eigenvalue_ratio,
                          options_.line_min_eigenvalue)) {
          continue;
        }
        auto* cost = new ceres::AutoDiffCostFunction<PointToLineResidual, 1, 3>(
            new PointToLineResidual(point, centroid, normal));
        problem.AddResidualBlock(cost, new ceres::HuberLoss(0.10), pose);
        ++correspondence_count;
      }
      if (correspondence_count < options_.min_correspondences) break;

      const Pose2D prior_pose{
          options_.use_prediction_translation_prior ? prediction_pose.x
                                                    : coarse_pose.x,
          options_.use_prediction_translation_prior ? prediction_pose.y
                                                    : coarse_pose.y,
          coarse_pose.yaw};
      auto* prior = new ceres::AutoDiffCostFunction<PosePriorResidual, 3, 3>(
          new PosePriorResidual(prior_pose,
                                options_.prior_translation_weight,
                                options_.prior_rotation_weight));
      problem.AddResidualBlock(prior, nullptr, pose);

      ceres::Solver::Options solver_options;
      solver_options.linear_solver_type = ceres::DENSE_QR;
      solver_options.max_num_iterations = 5;
      solver_options.num_threads = 1;
      solver_options.minimizer_progress_to_stdout = false;
      solver_options.function_tolerance = 1e-5;
      solver_options.parameter_tolerance = 1e-5;
      ceres::Solver::Summary summary;
      ceres::Solve(solver_options, &problem, &summary);
      pose[2] = NormalizeAngle(pose[2]);
      if (!summary.IsSolutionUsable()) break;
    }
  }

  double squared_error = 0.0;
  int final_correspondences = 0;
  Eigen::Matrix2d translation_information = Eigen::Matrix2d::Zero();
  const Pose2D final_pose{pose[0], pose[1], NormalizeAngle(pose[2])};
  for (const auto& point : points) {
    const Point2D world_point = TransformPoint(final_pose, point);
    Point2D centroid;
    Point2D normal;
    if (!grid.FitLine(world_point, options_.line_search_radius_cells,
                      options_.line_min_neighbors, &centroid, &normal,
                      options_.line_max_eigenvalue_ratio,
                      options_.line_min_eigenvalue)) {
      continue;
    }
    const double residual = normal.x * (world_point.x - centroid.x) +
                            normal.y * (world_point.y - centroid.y);
    squared_error += residual * residual;
    const Eigen::Vector2d translation_jacobian(normal.x, normal.y);
    translation_information +=
        translation_jacobian * translation_jacobian.transpose();
    ++final_correspondences;
  }

  ScanMatchResult result;
  result.pose = final_pose;
  result.coarse_score = coarse_score;
  result.correspondences = final_correspondences;
  if (final_correspondences > 0) {
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(
        translation_information);
    if (eigen_solver.info() == Eigen::Success &&
        eigen_solver.eigenvalues()[1] > 1e-12) {
      result.translation_observability =
          eigen_solver.eigenvalues()[0] / eigen_solver.eigenvalues()[1];
      const Eigen::Vector2d strong_direction =
          eigen_solver.eigenvectors().col(1);
      result.translation_strong_direction_yaw = std::atan2(
          strong_direction.y(), strong_direction.x());
    }
  }
  result.residual_rms = final_correspondences > 0
                            ? std::sqrt(squared_error / final_correspondences)
                            : std::numeric_limits<double>::infinity();
  result.success = final_correspondences >= options_.min_correspondences &&
                   std::isfinite(result.residual_rms);
  return result;
}

}  // namespace lightweight_2d_slam
