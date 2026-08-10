#pragma once

#include <cstddef>
#include <vector>

#include "lightweight_2d_slam/occupancy_grid.h"
#include "lightweight_2d_slam/types.h"

namespace lightweight_2d_slam {

struct ScanMatcherOptions {
  double linear_search_window = 0.15;
  double angular_search_window = 0.20;
  double linear_step = 0.05;
  double angular_step = 0.025;
  bool use_likelihood_field = true;
  bool enable_fine_correlative = true;
  int fine_candidate_count = 1;
  double fine_linear_step = 0.025;
  double fine_angular_window = 0.025;
  double fine_angular_step = 0.0125;
  int endpoint_neighborhood = 1;
  int max_coarse_points = 240;
  double correlative_translation_cost_weight = 0.01;
  double correlative_rotation_cost_weight = 0.005;
  bool enable_oad_csm = false;
  int oad_candidate_count = 5;
  double oad_min_score_margin = 0.015;
  double oad_min_translation_observability = 0.08;
  double oad_search_expansion_factor = 2.0;
  double oad_max_linear_search_window = 0.60;
  double oad_max_angular_search_window = 0.35;
  double oad_min_score_gain = 0.005;
  int line_search_radius_cells = 4;
  int line_min_neighbors = 4;
  int max_refinement_iterations = 8;
  bool use_likelihood_refinement = false;
  // Jointly optimize a continuous likelihood-field residual and geometric
  // point-to-line residuals. The default remains the legacy single objective.
  bool use_hybrid_refinement = false;
  int max_likelihood_refinement_points = 240;
  double likelihood_refinement_weight = 1.0;
  double point_line_refinement_weight = 0.35;
  double refinement_huber_scale = 0.20;
  int min_correspondences = 20;
  double prior_translation_weight = 0.5;
  double prior_rotation_weight = 1.0;
  // Keep continuous translation refinement tied to the motion prediction.
  // The correlative result still seeds the optimizer and supplies the yaw
  // prior, but it must not redefine forward motion in repetitive corridors.
  bool use_prediction_translation_prior = false;
  double line_max_eigenvalue_ratio = 0.35;
  double line_min_eigenvalue = 1e-8;
};

struct ScanMatchResult {
  Pose2D pose;
  double coarse_score = 0.0;
  double residual_rms = 0.0;
  double score_margin = 0.0;
  double map_score = 0.0;
  double prior_penalty = 0.0;
  double translation_observability = 0.0;
  double translation_strong_direction_yaw = 0.0;
  double match_quality = 0.0;
  int correspondences = 0;
  bool success = false;
  bool oad_triggered = false;
  bool oad_expanded = false;
  double oad_search_window = 0.0;
};

class ScanMatcher {
 public:
  explicit ScanMatcher(const ScanMatcherOptions& options) : options_(options) {}

  ScanMatchResult Match(const ProbabilityGrid& grid,
                        const PointCloud2D& points,
                        const Pose2D& initial_pose) const;

  ScanMatchResult MatchWithWindows(const ProbabilityGrid& grid,
                                   const PointCloud2D& points,
                                   const Pose2D& initial_pose,
                                   double linear_window,
                                   double angular_window) const;

 private:
  struct CorrelativeCandidate {
    Pose2D pose;
    double score = 0.0;
    double map_score = 0.0;
    double prior_penalty = 0.0;
  };

  ScanMatchResult MatchCore(const ProbabilityGrid& grid,
                            const PointCloud2D& points,
                            const Pose2D& initial_pose,
                            double linear_window,
                            double angular_window,
                            int candidate_count,
                            bool refine_all_candidates) const;
  ScanMatchResult MatchCoreDirectional(
      const ProbabilityGrid& grid, const PointCloud2D& points,
      const Pose2D& initial_pose, double axis_yaw, double axis_window,
      double cross_window, double angular_window, int candidate_count,
      bool refine_all_candidates) const;
  double MapPeakMargin(const std::vector<CorrelativeCandidate>& candidates,
                       double linear_step, double angular_step) const;

  std::vector<CorrelativeCandidate> CorrelativeCandidates(
      const ProbabilityGrid& grid, const PointCloud2D& points,
      const Pose2D& initial_pose, double linear_window,
      double angular_window, double linear_step, double angular_step,
      int candidate_count) const;
  std::vector<CorrelativeCandidate> CorrelativeCandidatesDirectional(
      const ProbabilityGrid& grid, const PointCloud2D& points,
      const Pose2D& initial_pose, double axis_yaw, double axis_window,
      double cross_window, double angular_window, double linear_step,
      double angular_step, int candidate_count,
      std::vector<CorrelativeCandidate>* map_candidates = nullptr) const;
  Pose2D CorrelativeMatch(const ProbabilityGrid& grid,
                          const PointCloud2D& points,
                          const Pose2D& initial_pose,
                          double linear_window,
                          double angular_window,
                          double linear_step,
                          double angular_step,
                          double* best_score) const;
  ScanMatchResult Refine(const ProbabilityGrid& grid,
                         const PointCloud2D& points,
                         const Pose2D& coarse_pose,
                         const Pose2D& prediction_pose,
                         double coarse_score) const;

  ScanMatcherOptions options_;
};

}  // namespace lightweight_2d_slam
