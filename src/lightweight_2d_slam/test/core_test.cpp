#include <gtest/gtest.h>

#include <cmath>

#include "lightweight_2d_slam/occupancy_grid.h"
#include "lightweight_2d_slam/loop_closure_gate.h"
#include "lightweight_2d_slam/pose_graph.h"
#include "lightweight_2d_slam/scan_matcher.h"
#include "lightweight_2d_slam/submap_2d.h"
#include "lightweight_2d_slam/types.h"

namespace lightweight_2d_slam {
namespace {

int MaximumOccupancyNear(const nav_msgs::OccupancyGrid& map,
                         double x, double y, double radius) {
  int maximum = -1;
  const int center_x = static_cast<int>(std::floor(
      (x - map.info.origin.position.x) / map.info.resolution));
  const int center_y = static_cast<int>(std::floor(
      (y - map.info.origin.position.y) / map.info.resolution));
  const int radius_cells =
      static_cast<int>(std::ceil(radius / map.info.resolution)) + 1;
  for (int cell_y = center_y - radius_cells;
       cell_y <= center_y + radius_cells; ++cell_y) {
    for (int cell_x = center_x - radius_cells;
         cell_x <= center_x + radius_cells; ++cell_x) {
      if (cell_x < 0 || cell_y < 0 ||
          cell_x >= static_cast<int>(map.info.width) ||
          cell_y >= static_cast<int>(map.info.height)) {
        continue;
      }
      maximum = std::max(maximum, static_cast<int>(map.data[
          static_cast<std::size_t>(cell_y * static_cast<int>(map.info.width) +
                                   cell_x)]));
    }
  }
  return maximum;
}

}  // namespace

TEST(TypesTest, ComposeAndInverseRecoverIdentity) {
  const Pose2D first{1.2, -0.4, 0.7};
  const Pose2D second{0.8, 0.3, -0.2};
  const Pose2D composed = Compose(first, second);
  const Pose2D recovered = Compose(Inverse(first), composed);
  EXPECT_NEAR(recovered.x, second.x, 1e-9);
  EXPECT_NEAR(recovered.y, second.y, 1e-9);
  EXPECT_NEAR(recovered.yaw, second.yaw, 1e-9);
}

TEST(TypesTest, AngleWrapsToPi) {
  EXPECT_NEAR(NormalizeAngle(3.0 * kPi), kPi, 1e-9);
  EXPECT_NEAR(NormalizeAngle(-3.0 * kPi), -kPi, 1e-9);
}

TEST(TypesTest, PropagatesOptimizedPoseWithoutDroppingMapCorrection) {
  const Pose2D previous_local{10.0, 2.0, 0.2};
  const Pose2D previous_optimized{1.0, -1.0, -0.1};
  const Pose2D current_local = Compose(previous_local, {0.5, 0.1, 0.03});
  const Pose2D current_optimized = PropagateOptimizedPose(
      previous_local, previous_optimized, current_local);
  const Pose2D optimized_increment =
      Between(previous_optimized, current_optimized);
  EXPECT_NEAR(optimized_increment.x, 0.5, 1e-9);
  EXPECT_NEAR(optimized_increment.y, 0.1, 1e-9);
  EXPECT_NEAR(optimized_increment.yaw, 0.03, 1e-9);
  EXPECT_GT(TranslationDistance(current_optimized, current_local), 4.0);
}

TEST(TypesTest, InterpolatesPoseAcrossAngleWrap) {
  const Pose2D midpoint = InterpolatePose(
      {0.0, 2.0, 170.0 * kPi / 180.0},
      {10.0, 4.0, -170.0 * kPi / 180.0}, 0.5);
  EXPECT_NEAR(midpoint.x, 5.0, 1e-12);
  EXPECT_NEAR(midpoint.y, 3.0, 1e-12);
  EXPECT_NEAR(std::abs(midpoint.yaw), kPi, 1e-12);
}

TEST(TypesTest, PropagatesNewAnchorThroughActiveGlobalCorrectionBlend) {
  const Pose2D previous_local{10.0, 2.0, 0.2};
  const Pose2D current_local = Compose(previous_local, {0.25, 0.04, 0.02});
  const Pose2D previous_blend_start{-7.6, 14.1, -0.30};
  const Pose2D previous_blend_target{-1.2, 0.8, -0.42};
  constexpr double kAlpha = 0.08;

  const Pose2D previous_published = InterpolatePose(
      previous_blend_start, previous_blend_target, kAlpha);
  const Pose2D pose_before_anchor_switch = PropagateOptimizedPose(
      previous_local, previous_published, current_local);
  const PropagatedPoseBlend extension = PropagatePoseBlend(
      previous_local, current_local, previous_blend_start,
      previous_blend_target, kAlpha);

  EXPECT_LT(TranslationDistance(extension.published,
                                pose_before_anchor_switch),
            0.01);
  EXPECT_NEAR(NormalizeAngle(extension.published.yaw -
                             pose_before_anchor_switch.yaw),
              0.0, 1e-12);
  const Pose2D completed = InterpolatePose(
      extension.start, extension.target, 1.0);
  EXPECT_NEAR(completed.x, extension.target.x, 1e-12);
  EXPECT_NEAR(completed.y, extension.target.y, 1e-12);
  EXPECT_NEAR(completed.yaw, extension.target.yaw, 1e-12);
}

TEST(ProbabilityGridTest, InsertScanProducesOccupiedAndFreeCells) {
  ProbabilityGrid grid(0.1, 100, 100, -5.0, -5.0);
  PointCloud2D scan{{2.0, 0.0}};
  grid.InsertScan(Pose2D{}, scan, true);
  EXPECT_GT(grid.EndpointScore(Point2D{2.0, 0.0}, 0), 0.55);
  EXPECT_GT(grid.EndpointScore(Point2D{1.0, 0.0}, 0), 0.0);
  const nav_msgs::OccupancyGrid message =
      grid.ToMessage("map", ros::Time(1.0));
  EXPECT_EQ(message.info.width, 100u);
  EXPECT_EQ(message.info.height, 100u);
  EXPECT_FALSE(message.data.empty());
}

TEST(ProbabilityGridTest, DefaultSensorModelPreservesSingleHitBehavior) {
  ProbabilityGrid grid(0.1, 100, 100, -5.0, -5.0);
  grid.InsertScan(Pose2D{}, {{2.0, 0.0}}, false);
  EXPECT_NEAR(grid.EndpointScore({2.0, 0.0}, 0),
              1.0 / (1.0 + std::exp(-0.85)), 1e-6);
}

TEST(ProbabilityGridTest, CalibratedSensorModelRequiresRepeatedHits) {
  ProbabilityGrid grid(0.1, 100, 100, -5.0, -5.0);
  ASSERT_TRUE(grid.SetUpdateProbabilities(0.60, 0.49));

  grid.InsertScan(Pose2D{}, {{2.0, 0.0}}, false);
  const double single_hit = grid.EndpointScore({2.0, 0.0}, 0);
  EXPECT_NEAR(single_hit, 0.60, 1e-6);
  EXPECT_LT(single_hit, 0.65);

  grid.InsertScan(Pose2D{}, {{2.0, 0.0}}, false);
  EXPECT_GT(grid.EndpointScore({2.0, 0.0}, 0), 0.65);
}

TEST(ProbabilityGridTest, RejectsInvalidSensorModelWithoutChangingDefault) {
  ProbabilityGrid grid(0.1, 100, 100, -5.0, -5.0);
  EXPECT_FALSE(grid.SetUpdateProbabilities(0.50, 0.49));
  EXPECT_FALSE(grid.SetUpdateProbabilities(0.60, 0.50));
  grid.InsertScan(Pose2D{}, {{2.0, 0.0}}, false);
  EXPECT_NEAR(grid.EndpointScore({2.0, 0.0}, 0),
              1.0 / (1.0 + std::exp(-0.85)), 1e-6);
}

TEST(ProbabilityGridTest, ClipsOutOfBoundsRayAsFreeSpace) {
  ProbabilityGrid grid(0.1, 20, 20, -1.0, -1.0);
  grid.InsertScan(Pose2D{}, PointCloud2D{{5.0, 0.0}}, true);

  const double interior_score = grid.EndpointScore(Point2D{0.8, 0.0}, 0);
  EXPECT_GT(interior_score, 0.0);
  EXPECT_LT(interior_score, 0.5);
}

TEST(ProbabilityGridTest, HitDominatesRepeatedMissesWithinOneScan) {
  ProbabilityGrid grid(0.1, 100, 100, -5.0, -5.0);
  PointCloud2D scan{{2.0, 0.0}};
  for (int index = 0; index < 20; ++index) {
    scan.push_back({3.0, 0.0});
  }
  grid.InsertScan(Pose2D{}, scan, true);

  EXPECT_GT(grid.EndpointScore(Point2D{2.0, 0.0}, 0), 0.55);
  EXPECT_NEAR(grid.EndpointScore(Point2D{1.0, 0.0}, 0),
              1.0 / (1.0 + std::exp(0.4)), 1e-6);
}

TEST(ProbabilityGridTest, ConnectsOnlyNearbyOrderedScanEndpoints) {
  ProbabilityGrid connected(0.05, 120, 120, -3.0, -3.0);
  connected.InsertScan(Pose2D{}, {{2.0, 0.0}, {2.0, 0.20}}, false,
                       0.25);
  EXPECT_GT(connected.EndpointScore({2.0, 0.10}, 0), 0.55);

  ProbabilityGrid separated(0.05, 120, 120, -3.0, -3.0);
  separated.InsertScan(Pose2D{}, {{2.0, 0.0}, {2.0, 0.50}}, false,
                       0.25);
  EXPECT_EQ(separated.EndpointScore({2.0, 0.25}, 0), 0.0);
}

TEST(ProbabilityGridTest, FitLineFindsWallNormal) {
  ProbabilityGrid grid(0.05, 200, 200, -5.0, -5.0);
  PointCloud2D scan;
  for (int i = -30; i <= 30; ++i) scan.push_back({2.0, i * 0.05});
  grid.InsertScan(Pose2D{}, scan, false);
  Point2D centroid;
  Point2D normal;
  EXPECT_TRUE(grid.FitLine(Point2D{2.0, 0.0}, 3, 4, &centroid, &normal));
  EXPECT_NEAR(std::abs(normal.x), 1.0, 0.1);
  EXPECT_NEAR(centroid.x, 2.0, 0.1);
}

TEST(ProbabilityGridTest, LikelihoodGradientMatchesFiniteDifference) {
  ProbabilityGrid grid(0.05, 200, 200, -5.0, -5.0);
  PointCloud2D wall;
  for (int index = -30; index <= 30; ++index) {
    wall.push_back({2.0, index * 0.05});
  }
  grid.InsertScan(Pose2D{}, wall, false);

  const Point2D query{1.96, 0.17};
  Point2D gradient;
  grid.LikelihoodScoreAndGradient(query, 2, &gradient);
  constexpr double kEpsilon = 1e-5;
  const double numeric_x =
      (grid.LikelihoodScore({query.x + kEpsilon, query.y}, 2) -
       grid.LikelihoodScore({query.x - kEpsilon, query.y}, 2)) /
      (2.0 * kEpsilon);
  const double numeric_y =
      (grid.LikelihoodScore({query.x, query.y + kEpsilon}, 2) -
       grid.LikelihoodScore({query.x, query.y - kEpsilon}, 2)) /
      (2.0 * kEpsilon);
  EXPECT_NEAR(gradient.x, numeric_x, 1e-3);
  EXPECT_NEAR(gradient.y, numeric_y, 1e-3);
}

TEST(ScanMatcherTest, RecoversPoseAgainstSquareRoom) {
  ProbabilityGrid grid(0.05, 240, 240, -6.0, -6.0);
  PointCloud2D scan;
  for (int index = -40; index <= 40; ++index) {
    const double coordinate = index * 0.05;
    scan.push_back({2.0, coordinate});
    scan.push_back({-2.0, coordinate});
    scan.push_back({coordinate, 2.0});
    scan.push_back({coordinate, -2.0});
  }
  grid.InsertScan(Pose2D{}, scan, false);
  ScanMatcherOptions options;
  options.linear_search_window = 0.25;
  options.angular_search_window = 0.10;
  options.min_correspondences = 30;
  ScanMatcher matcher(options);
  const ScanMatchResult result = matcher.Match(
      grid, scan, Pose2D{0.12, -0.08, 0.04});
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.coarse_score, 0.5);
  EXPECT_LT(std::hypot(result.pose.x, result.pose.y), 0.08);
  EXPECT_LT(std::abs(result.pose.yaw), 0.03);
  EXPECT_GT(result.translation_observability, 0.5);
}

TEST(ScanMatcherTest, DetectsUnobservableCorridorTranslation) {
  ProbabilityGrid grid(0.05, 240, 240, -6.0, -6.0);
  PointCloud2D scan;
  for (int index = -80; index <= 80; ++index) {
    const double x = index * 0.05;
    scan.push_back({x, 2.0});
    scan.push_back({x, -2.0});
  }
  grid.InsertScan(Pose2D{}, scan, false);
  ScanMatcherOptions options;
  options.linear_search_window = 0.10;
  options.angular_search_window = 0.05;
  options.min_correspondences = 30;
  const ScanMatchResult result =
      ScanMatcher(options).Match(grid, scan, Pose2D{});
  ASSERT_TRUE(result.success);
  EXPECT_LT(result.translation_observability, 0.02);
}

TEST(ScanMatcherTest, PredictionPriorPreservesUnobservableTranslation) {
  ProbabilityGrid grid(0.05, 500, 300, -5.0, -7.5);
  PointCloud2D map_scan;
  for (double x = -4.0; x <= 12.0; x += 0.05) {
    map_scan.push_back({x, -2.0});
    map_scan.push_back({x, 2.0});
  }
  grid.InsertScan(Pose2D{}, map_scan, false);

  PointCloud2D query;
  for (double x = -3.0; x <= 3.0; x += 0.05) {
    query.push_back({x, -2.0});
    query.push_back({x, 2.0});
  }
  ScanMatcherOptions legacy_options;
  legacy_options.use_likelihood_refinement = true;
  legacy_options.max_refinement_iterations = 12;
  legacy_options.correlative_translation_cost_weight = 0.0;
  legacy_options.prior_translation_weight = 5.0;
  legacy_options.prior_rotation_weight = 3.0;
  legacy_options.use_prediction_translation_prior = false;
  const Pose2D prediction{2.0, 0.0, 0.0};
  const ScanMatchResult legacy =
      ScanMatcher(legacy_options).Match(grid, query, prediction);

  ScanMatcherOptions prediction_options = legacy_options;
  prediction_options.use_prediction_translation_prior = true;
  const ScanMatchResult prediction_anchored =
      ScanMatcher(prediction_options).Match(grid, query, prediction);

  EXPECT_TRUE(legacy.success);
  EXPECT_TRUE(prediction_anchored.success);
  EXPECT_LT(std::abs(prediction_anchored.pose.x - prediction.x),
            std::abs(legacy.pose.x - prediction.x));
  EXPECT_NEAR(prediction_anchored.pose.x, prediction.x, 0.03);
}

TEST(ScanMatcherTest, LikelihoodRefinementRecoversPoseAgainstSquareRoom) {
  ProbabilityGrid grid(0.05, 240, 240, -6.0, -6.0);
  PointCloud2D scan;
  for (int index = -30; index <= 30; ++index) {
    const double coordinate = index * 0.05;
    scan.push_back({2.0, coordinate});
    scan.push_back({-2.0, coordinate});
    scan.push_back({coordinate, 2.0});
    scan.push_back({coordinate, -2.0});
  }
  grid.InsertScan(Pose2D{}, scan, false);
  ScanMatcherOptions options;
  options.linear_search_window = 0.25;
  options.angular_search_window = 0.10;
  options.min_correspondences = 30;
  options.use_likelihood_refinement = true;
  options.max_likelihood_refinement_points = 160;
  ScanMatcher matcher(options);
  const ScanMatchResult result = matcher.Match(
      grid, scan, Pose2D{0.12, -0.08, 0.04});
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.coarse_score, 0.5);
  EXPECT_LT(std::hypot(result.pose.x, result.pose.y), 0.08);
  EXPECT_LT(std::abs(result.pose.yaw), 0.03);
}

TEST(ScanMatcherTest, FineCorrelativeReducesCoarseGridQuantization) {
  ProbabilityGrid grid(0.05, 160, 160, -4.0, -4.0);
  const PointCloud2D scan{{1.00, 0.00},  {0.35, 1.40}, {-1.10, 0.70},
                          {-0.60, -1.20}, {1.20, -0.80}, {1.70, 0.60}};
  const Pose2D expected_pose{0.025, -0.025, 0.0125};
  grid.InsertScan(expected_pose, scan, false);

  ScanMatcherOptions coarse_options;
  coarse_options.linear_search_window = 0.10;
  coarse_options.angular_search_window = 0.05;
  coarse_options.linear_step = 0.05;
  coarse_options.angular_step = 0.025;
  coarse_options.endpoint_neighborhood = 1;
  coarse_options.max_refinement_iterations = 0;
  coarse_options.use_likelihood_field = true;
  coarse_options.enable_fine_correlative = false;
  const ScanMatchResult coarse_result =
      ScanMatcher(coarse_options).Match(grid, scan, Pose2D{});

  ScanMatcherOptions fine_options = coarse_options;
  fine_options.enable_fine_correlative = true;
  fine_options.fine_candidate_count = 3;
  fine_options.fine_linear_step = 0.0125;
  fine_options.fine_angular_step = 0.00625;
  const ScanMatchResult fine_result =
      ScanMatcher(fine_options).Match(grid, scan, Pose2D{});

  const double coarse_translation_error =
      std::hypot(coarse_result.pose.x - expected_pose.x,
                 coarse_result.pose.y - expected_pose.y);
  const double fine_translation_error =
      std::hypot(fine_result.pose.x - expected_pose.x,
                 fine_result.pose.y - expected_pose.y);
  EXPECT_LT(fine_translation_error, coarse_translation_error)
      << "coarse pose=" << coarse_result.pose.x << ","
      << coarse_result.pose.y << "," << coarse_result.pose.yaw
      << " score=" << coarse_result.coarse_score << " fine pose="
      << fine_result.pose.x << "," << fine_result.pose.y << ","
      << fine_result.pose.yaw << " score=" << fine_result.coarse_score;
  EXPECT_LT(fine_translation_error, 0.02);
  EXPECT_LT(std::abs(NormalizeAngle(fine_result.pose.yaw - expected_pose.yaw)),
            0.015);
}

TEST(PoseGraphTest, OptimizesSequentialConstraints) {
  PoseGraph graph;
  const std::size_t node0 = graph.AddNode(Pose2D{});
  const std::size_t node1 = graph.AddNode(Pose2D{1.2, 0.2, 0.05});
  const std::size_t node2 = graph.AddNode(Pose2D{2.3, -0.1, -0.04});
  graph.AddConstraint({node0, node1, Pose2D{1.0, 0.0, 0.0},
                       10.0, 10.0, ConstraintType::kScan});
  graph.AddConstraint({node1, node2, Pose2D{1.0, 0.0, 0.0},
                       10.0, 10.0, ConstraintType::kScan});
  graph.AddConstraint({node0, node2, Pose2D{2.0, 0.0, 0.0},
                       10.0, 10.0, ConstraintType::kLoopClosure});
  graph.RequestOptimization();
  ASSERT_TRUE(graph.WaitForIdle(2.0));
  const std::vector<Pose2D> poses = graph.GetOptimizedPoses();
  ASSERT_EQ(poses.size(), 3u);
  EXPECT_NEAR(poses[1].x, 1.0, 1e-3);
  EXPECT_NEAR(poses[1].y, 0.0, 1e-3);
  EXPECT_NEAR(poses[2].x, 2.0, 1e-3);
  EXPECT_NEAR(poses[2].y, 0.0, 1e-3);
}

TEST(PoseGraphTest, SwitchableLoopConstraintIsSuppressedWhenInconsistent) {
  PoseGraph graph;
  const std::size_t node0 = graph.AddNode(Pose2D{});
  const std::size_t node1 = graph.AddNode(Pose2D{1.0, 0.0, 0.0});
  const std::size_t node2 = graph.AddNode(Pose2D{2.0, 0.0, 0.0});
  graph.AddConstraint({node0, node1, Pose2D{1.0, 0.0, 0.0},
                       20.0, 20.0, ConstraintType::kScan});
  graph.AddConstraint({node1, node2, Pose2D{1.0, 0.0, 0.0},
                       20.0, 20.0, ConstraintType::kScan});
  PoseGraphConstraint false_loop{node0, node2, Pose2D{8.0, 0.0, 0.0},
                                 20.0, 20.0, ConstraintType::kLoopClosure};
  false_loop.switchable = true;
  false_loop.switch_prior_weight = 1.0;
  graph.AddConstraint(false_loop);
  graph.RequestOptimization();
  ASSERT_TRUE(graph.WaitForIdle(2.0));

  const std::vector<Pose2D> poses = graph.GetOptimizedPoses();
  const PoseGraphStatus status = graph.GetStatus();
  ASSERT_EQ(poses.size(), 3u);
  EXPECT_NEAR(poses[2].x, 2.0, 1e-2);
  EXPECT_LT(status.minimum_loop_switch, 0.5);
  EXPECT_EQ(status.suppressed_loop_constraints, 1u);
}

TEST(PoseGraphTest, SwitchableLoopConstraintRemainsActiveWhenConsistent) {
  PoseGraph graph;
  const std::size_t node0 = graph.AddNode(Pose2D{});
  const std::size_t node1 = graph.AddNode(Pose2D{1.05, 0.02, 0.01});
  const std::size_t node2 = graph.AddNode(Pose2D{2.1, 0.03, 0.01});
  graph.AddConstraint({node0, node1, Pose2D{1.0, 0.0, 0.0},
                       20.0, 20.0, ConstraintType::kScan});
  graph.AddConstraint({node1, node2, Pose2D{1.0, 0.0, 0.0},
                       20.0, 20.0, ConstraintType::kScan});
  PoseGraphConstraint consistent_loop{
      node0, node2, Pose2D{2.0, 0.0, 0.0}, 20.0, 20.0,
      ConstraintType::kLoopClosure};
  consistent_loop.switchable = true;
  consistent_loop.switch_prior_weight = 25.0;
  graph.AddConstraint(consistent_loop);
  graph.RequestOptimization();
  ASSERT_TRUE(graph.WaitForIdle(2.0));

  const std::vector<Pose2D> poses = graph.GetOptimizedPoses();
  const PoseGraphStatus status = graph.GetStatus();
  ASSERT_EQ(poses.size(), 3u);
  EXPECT_NEAR(poses[2].x, 2.0, 1e-3);
  EXPECT_GT(status.minimum_loop_switch, 0.9);
  EXPECT_EQ(status.suppressed_loop_constraints, 0u);
}

TEST(PoseGraphTest, AnisotropicConstraintLeavesWeakDirectionUnforced) {
  PoseGraph graph;
  const std::size_t node0 = graph.AddNode(Pose2D{});
  const std::size_t node1 = graph.AddNode(Pose2D{3.0, 2.0, 0.1});
  PoseGraphConstraint constraint{
      node0, node1, Pose2D{100.0, 0.0, 0.0}, 10.0, 10.0,
      ConstraintType::kLoopClosure};
  constraint.anisotropic_translation = true;
  constraint.translation_direction_yaw = 0.5 * kPi;
  constraint.orthogonal_translation_weight = 0.0;
  graph.AddConstraint(constraint);
  graph.RequestOptimization();
  ASSERT_TRUE(graph.WaitForIdle(2.0));
  const std::vector<Pose2D> poses = graph.GetOptimizedPoses();
  ASSERT_EQ(poses.size(), 2u);
  EXPECT_NEAR(poses[1].x, 3.0, 1e-3);
  EXPECT_NEAR(poses[1].y, 0.0, 1e-3);
  EXPECT_NEAR(poses[1].yaw, 0.0, 1e-3);
}

TEST(PoseGraphTest, NodeToSubmapLoopMovesRigidlyConnectedNodesTogether) {
  PoseGraph graph;
  const std::size_t fixed_submap = graph.AddSubmap(Pose2D{});
  const std::size_t moving_submap =
      graph.AddSubmap(Pose2D{10.0, 0.0, 0.0});
  const std::size_t first = graph.AddNode(Pose2D{10.0, 0.0, 0.0});
  const std::size_t second = graph.AddNode(Pose2D{11.0, 0.0, 0.0});

  graph.AddNodeToSubmapConstraint(
      {moving_submap, first, Pose2D{}, 30.0, 30.0,
       ConstraintType::kScan});
  graph.AddNodeToSubmapConstraint(
      {moving_submap, second, Pose2D{1.0, 0.0, 0.0}, 30.0, 30.0,
       ConstraintType::kScan});
  graph.AddNodeToSubmapConstraint(
      {fixed_submap, first, Pose2D{2.0, 0.0, 0.0}, 30.0, 30.0,
       ConstraintType::kLoopClosure});
  graph.RequestOptimization();
  ASSERT_TRUE(graph.WaitForIdle(2.0));

  const std::vector<Pose2D> nodes = graph.GetOptimizedPoses();
  const std::vector<Pose2D> submaps = graph.GetOptimizedSubmapPoses();
  const PoseGraphStatus status = graph.GetStatus();
  ASSERT_EQ(nodes.size(), 2u);
  ASSERT_EQ(submaps.size(), 2u);
  EXPECT_NEAR(submaps[moving_submap].x, 2.0, 1e-3);
  EXPECT_NEAR(nodes[first].x, 2.0, 1e-3);
  EXPECT_NEAR(nodes[second].x, 3.0, 1e-3);
  EXPECT_NEAR(nodes[second].x - nodes[first].x, 1.0, 1e-3);
  EXPECT_EQ(status.node_submap_constraints, 3u);
  EXPECT_EQ(status.loop_constraints, 1u);
}

TEST(LoopClosureGateTest, RequiresConsistentIndependentConfirmations) {
  LoopClosureGate gate({2, 0.15, 0.12, 30});
  const LoopClosureProposal first{10, 100, 0, 2, {0.20, 0.0, 0.10}};
  EXPECT_FALSE(gate.Consider(first));
  EXPECT_FALSE(gate.Consider(first));
  const LoopClosureProposal inconsistent{
      11, 105, 0, 2, {0.45, 0.0, 0.10}};
  EXPECT_FALSE(gate.Consider(inconsistent));
  const LoopClosureProposal confirmed{
      12, 110, 0, 2, {0.43, 0.01, 0.11}};
  EXPECT_TRUE(gate.Consider(confirmed));
  EXPECT_TRUE(gate.IsPairAccepted(0, 2));
}

TEST(LoopClosureGateTest, RejectsDuplicatePairsAndEnforcesCooldown) {
  LoopClosureGate gate({1, 0.15, 0.12, 30});
  EXPECT_TRUE(gate.Consider({10, 100, 0, 2, {0.10, 0.0, 0.02}}));
  EXPECT_FALSE(gate.Consider({11, 105, 0, 2, {0.10, 0.0, 0.02}}));
  EXPECT_FALSE(gate.Consider({20, 120, 0, 3, {0.08, 0.0, 0.02}}));
  EXPECT_TRUE(gate.Consider({21, 130, 0, 3, {0.08, 0.0, 0.02}}));
}

TEST(LoopClosureGateTest, TracksInterleavedSubmapPairsIndependently) {
  LoopClosureGate gate({2, 0.15, 0.12, 0});
  EXPECT_FALSE(gate.Consider({10, 100, 0, 4, {1.0, 0.2, 0.05}}));
  EXPECT_FALSE(gate.Consider({20, 101, 1, 4, {3.0, -0.5, 0.20}}));
  EXPECT_TRUE(gate.Consider({11, 105, 0, 4, {1.05, 0.22, 0.04}}));
  EXPECT_TRUE(gate.IsPairAccepted(0, 4));
  EXPECT_FALSE(gate.IsPairAccepted(1, 4));
}

TEST(Submap2DTest, FreezesAnImmutableAnchorRelativeGrid) {
  Submap2D submap(3, 0.05, 10.0, false);
  Keyframe first;
  first.id = 10;
  first.submap_id = 3;
  first.local_pose = {8.0, -2.0, 0.0};
  first.points = {{2.0, 0.0}, {2.0, 0.2}};
  first.descriptor = BuildPolarDescriptor(first.points, 10.0);
  ASSERT_TRUE(submap.AddKeyframe(first));

  Keyframe second = first;
  second.id = 11;
  second.local_pose = {9.0, -2.0, 0.0};
  ASSERT_TRUE(submap.AddKeyframe(second));
  ASSERT_TRUE(submap.Freeze());

  EXPECT_TRUE(submap.frozen());
  EXPECT_EQ(submap.first_keyframe_id(), 10u);
  EXPECT_EQ(submap.last_keyframe_id(), 11u);
  EXPECT_EQ(submap.keyframe_count(), 2u);
  EXPECT_EQ(submap.anchor_keyframe_id(), 10u);
  EXPECT_NEAR(submap.anchor_local_pose().x, 8.0, 1e-12);
  EXPECT_GT(submap.grid().EndpointScore({2.0, 0.0}, 1), 0.55);
  EXPECT_GT(submap.grid().EndpointScore({3.0, 0.0}, 1), 0.55);

  Keyframe rejected = second;
  rejected.id = 12;
  rejected.local_pose = {10.0, -2.0, 0.0};
  EXPECT_FALSE(submap.AddKeyframe(rejected));
  EXPECT_EQ(submap.keyframe_count(), 2u);
  EXPECT_FALSE(submap.Freeze());
}

TEST(SubmapTextureTest, RepeatedWallDominatesOneFrameTransientHit) {
  Submap2D submap(0, 0.1, 10.0, false, 0.55, 0.49);
  Keyframe frame;
  frame.id = 0;
  frame.submap_id = 0;
  frame.points = {{2.0, 0.0}, {2.0, 1.0}};
  ASSERT_TRUE(submap.AddKeyframe(frame, true));
  for (std::size_t id = 1; id <= 5; ++id) {
    frame.id = id;
    frame.points = {{2.0, 0.0}};
    ASSERT_TRUE(submap.AddKeyframe(frame, true));
  }

  const auto texture = submap.GetTextureSnapshot();
  nav_msgs::OccupancyGrid map;
  ASSERT_TRUE(ComposeSubmapTextures(
      {texture}, {Pose2D{}}, 0.1, 100000, "map", ros::Time(1.0), &map));
  EXPECT_GT(MaximumOccupancyNear(map, 2.0, 0.0, 0.1), 65);
  EXPECT_LT(MaximumOccupancyNear(map, 2.0, 1.0, 0.1), 65);
}

TEST(SubmapTextureTest, PrimaryOwnershipAvoidsOverlapDoubleCounting) {
  Submap2D overlap_submap(0, 0.1, 10.0, false, 0.55, 0.49);
  Submap2D primary_submap(1, 0.1, 10.0, false, 0.55, 0.49);
  Keyframe frame;
  frame.id = 25;
  frame.submap_id = 1;
  frame.points = {{2.0, 0.0}};

  ASSERT_TRUE(overlap_submap.AddKeyframe(frame, false));
  ASSERT_TRUE(primary_submap.AddKeyframe(frame, true));
  EXPECT_TRUE(overlap_submap.GetTextureSnapshot()->grid.empty());
  EXPECT_FALSE(primary_submap.GetTextureSnapshot()->grid.empty());
  EXPECT_GT(overlap_submap.grid().EndpointScore({2.0, 0.0}, 0), 0.55);
}

TEST(SubmapTextureTest, FullResolutionTextureDoesNotChangeMatchingGrid) {
  Submap2D submap(0, 0.1, 10.0, false, 0.70, 0.45);
  Keyframe frame;
  frame.id = 0;
  frame.submap_id = 0;
  frame.points = {{2.0, 0.0}};
  const PointCloud2D mapping_points{{2.0, 0.0}, {0.0, 2.0}};

  ASSERT_TRUE(submap.AddKeyframe(frame, true, &mapping_points));

  EXPECT_GT(submap.grid().EndpointScore({2.0, 0.0}, 0), 0.55);
  EXPECT_LT(submap.grid().EndpointScore({0.0, 2.0}, 0), 0.55);
  nav_msgs::OccupancyGrid map;
  ASSERT_TRUE(ComposeSubmapTextures(
      {submap.GetTextureSnapshot()}, {Pose2D{}}, 0.1, 100000, "map",
      ros::Time(1.0), &map));
  EXPECT_GT(MaximumOccupancyNear(map, 2.0, 0.0, 0.1), 65);
  EXPECT_GT(MaximumOccupancyNear(map, 0.0, 2.0, 0.1), 65);
}

TEST(SubmapTextureTest, RotatedCompositionPreservesConnectedWall) {
  Submap2D submap(0, 0.1, 10.0, false, 0.70, 0.49, 0.11);
  Keyframe frame;
  frame.id = 0;
  frame.submap_id = 0;
  for (int index = 0; index <= 10; ++index) {
    frame.points.push_back({1.0 + 0.1 * index, 0.0});
  }
  ASSERT_TRUE(submap.AddKeyframe(frame, true));

  nav_msgs::OccupancyGrid map;
  ASSERT_TRUE(ComposeSubmapTextures(
      {submap.GetTextureSnapshot()}, {{0.0, 0.0, 0.5 * kPi}},
      0.1, 100000, "map", ros::Time(1.0), &map));
  int occupied = 0;
  int min_x = static_cast<int>(map.info.width);
  int max_x = -1;
  int min_y = static_cast<int>(map.info.height);
  int max_y = -1;
  for (int y = 0; y < static_cast<int>(map.info.height); ++y) {
    for (int x = 0; x < static_cast<int>(map.info.width); ++x) {
      const int value = map.data[static_cast<std::size_t>(
          y * static_cast<int>(map.info.width) + x)];
      if (value <= 65) continue;
      ++occupied;
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }
  }
  EXPECT_GE(occupied, 10);
  EXPECT_LE(max_x - min_x, 1);
  EXPECT_GE(max_y - min_y, 9);
}

TEST(SubmapTextureTest, FrozenSnapshotIsStableAndReused) {
  Submap2D submap(4, 0.1, 10.0, false, 0.55, 0.49);
  Keyframe frame;
  frame.id = 100;
  frame.submap_id = 4;
  frame.points = {{2.0, 0.0}};
  ASSERT_TRUE(submap.AddKeyframe(frame, true));
  ASSERT_TRUE(submap.Freeze());
  const auto first = submap.GetTextureSnapshot();
  const std::vector<float> frozen_values = first->grid.log_odds;

  frame.id = 101;
  EXPECT_FALSE(submap.AddKeyframe(frame, true));
  const auto second = submap.GetTextureSnapshot();
  EXPECT_EQ(first.get(), second.get());
  EXPECT_EQ(second->grid.log_odds, frozen_values);
}

TEST(Submap2DTest, CircularDescriptorReturnsSignedYawOffset) {
  const std::vector<float> reference{0.1f, 0.2f, 0.3f, 0.4f,
                                     0.5f, 0.6f, 0.7f, 0.8f};
  std::vector<float> query(reference.size());
  const std::size_t expected_shift = 2;
  for (std::size_t index = 0; index < query.size(); ++index) {
    query[index] = reference[(index + expected_shift) % reference.size()];
  }

  const CircularDescriptorMatch match =
      MatchCircularDescriptor(query, reference);
  ASSERT_TRUE(match.valid);
  EXPECT_EQ(match.shift_bins, static_cast<int>(expected_shift));
  EXPECT_NEAR(match.yaw_offset, 0.5 * kPi, 1e-12);
  EXPECT_NEAR(match.similarity, 1.0, 1e-7);
}

TEST(Submap2DTest, MultiScaleDescriptorUsesOneYawShiftAcrossScales) {
  constexpr int kBins = 12;
  constexpr int kScales = 3;
  PointCloud2D reference_points{{1.0, 0.0}, {3.0, 1.0},
                                {-2.0, 4.0}, {-5.0, -1.0}};
  const double yaw = 2.0 * kPi * 3.0 / static_cast<double>(kBins);
  PointCloud2D query_points;
  query_points.reserve(reference_points.size());
  for (const auto& point : reference_points) {
    query_points.push_back(TransformPoint({0.0, 0.0, -yaw}, point));
  }
  const std::vector<float> reference = BuildMultiScalePolarDescriptor(
      reference_points, 9.0, kBins, kScales);
  const std::vector<float> query = BuildMultiScalePolarDescriptor(
      query_points, 9.0, kBins, kScales);
  ASSERT_EQ(reference.size(), static_cast<std::size_t>(kBins * kScales));

  const CircularDescriptorMatch match =
      MatchMultiScaleCircularDescriptor(query, reference, kBins);
  ASSERT_TRUE(match.valid);
  EXPECT_EQ(match.shift_bins, 3);
  EXPECT_NEAR(match.yaw_offset, yaw, 1e-12);
  EXPECT_NEAR(match.similarity, 1.0, 1e-7);
}

TEST(Submap2DTest, MultiScaleDescriptorSeparatesNearAndFarGeometry) {
  PointCloud2D near_points{{1.0, 0.0}, {0.0, 1.0},
                           {-1.0, 0.0}, {0.0, -1.0}};
  PointCloud2D far_points{{5.0, 0.0}, {0.0, 5.0},
                          {-5.0, 0.0}, {0.0, -5.0}};
  const std::vector<float> near_descriptor =
      BuildMultiScalePolarDescriptor(near_points, 9.0, 12, 3);
  const std::vector<float> far_descriptor =
      BuildMultiScalePolarDescriptor(far_points, 9.0, 12, 3);
  const CircularDescriptorMatch same = MatchMultiScaleCircularDescriptor(
      near_descriptor, near_descriptor, 12);
  const CircularDescriptorMatch different = MatchMultiScaleCircularDescriptor(
      near_descriptor, far_descriptor, 12);
  ASSERT_TRUE(same.valid);
  ASSERT_TRUE(different.valid);
  EXPECT_NEAR(same.similarity, 1.0, 1e-7);
  EXPECT_GT(same.similarity, different.similarity + 0.10);
}

TEST(Submap2DTest, Se2ConsensusRequiresIndependentSubmaps) {
  const std::vector<LoopCorrectionHypothesis> repeated_one_submap{
      {0, 2, {4.0, -1.0, 0.10}, 0.90, 0.03},
      {1, 2, {4.1, -1.0, 0.11}, 0.92, 0.03}};
  const LoopCorrectionConsensus rejected = SelectLoopCorrectionConsensus(
      repeated_one_submap, 2, 0.30, 0.05);
  EXPECT_FALSE(rejected.valid);
  EXPECT_EQ(rejected.supporting_submaps, 1);

  std::vector<LoopCorrectionHypothesis> independent = repeated_one_submap;
  independent.push_back({2, 5, {3.95, -0.92, 0.09}, 0.88, 0.02});
  const LoopCorrectionConsensus accepted = SelectLoopCorrectionConsensus(
      independent, 2, 0.30, 0.05);
  ASSERT_TRUE(accepted.valid);
  EXPECT_EQ(accepted.supporting_submaps, 2);
  EXPECT_EQ(accepted.representative_index, 1u);
}

TEST(Submap2DTest, Se2ConsensusRejectsHighScoreOutlier) {
  const std::vector<LoopCorrectionHypothesis> hypotheses{
      {0, 1, {8.0, 5.0, 0.8}, 0.99, 0.01},
      {1, 2, {1.0, -0.1, 0.05}, 0.86, 0.04},
      {2, 4, {1.1, -0.05, 0.04}, 0.84, 0.03}};
  const LoopCorrectionConsensus consensus = SelectLoopCorrectionConsensus(
      hypotheses, 2, 0.30, 0.08);
  ASSERT_TRUE(consensus.valid);
  EXPECT_EQ(consensus.supporting_submaps, 2);
  EXPECT_EQ(consensus.representative_index, 1u);
}

TEST(Submap2DTest, GlobalTopKDoesNotDependOnDriftedPosition) {
  auto make_submap = [](std::size_t submap_id, std::size_t keyframe_id,
                        const Pose2D& pose,
                        const std::vector<float>& descriptor) {
    std::shared_ptr<Submap2D> submap(
        new Submap2D(submap_id, 0.05, 10.0, false));
    Keyframe keyframe;
    keyframe.id = keyframe_id;
    keyframe.submap_id = submap_id;
    keyframe.local_pose = pose;
    keyframe.points = {{2.0, 0.0}, {2.0, 0.2}};
    keyframe.descriptor = descriptor;
    EXPECT_TRUE(submap->AddKeyframe(keyframe));
    EXPECT_TRUE(submap->Freeze());
    return std::shared_ptr<const Submap2D>(submap);
  };

  const std::vector<float> target{0.1f, 0.9f, 0.2f, 0.8f,
                                  0.3f, 0.7f, 0.4f, 0.6f};
  const std::vector<float> distractor(target.size(), 0.5f);
  std::vector<std::shared_ptr<const Submap2D>> submaps;
  submaps.push_back(make_submap(0, 0, {100.0, 50.0, 0.0}, target));
  submaps.push_back(make_submap(1, 10, {0.0, 0.0, 0.0}, distractor));

  const std::vector<SubmapCandidate> candidates = RetrieveTopKSubmaps(
      target, submaps, 4, 2, 2, 0.0);
  ASSERT_EQ(candidates.size(), 2u);
  EXPECT_EQ(candidates.front().submap->id(), 0u);
  EXPECT_EQ(candidates.front().reference_keyframe_id, 0u);
  EXPECT_NEAR(candidates.front().descriptor_similarity, 1.0, 1e-7);
}

TEST(Submap2DTest, CanRetainMultipleDescriptorHypothesesPerSubmap) {
  std::shared_ptr<Submap2D> mutable_submap(
      new Submap2D(0, 0.05, 10.0, false));
  const std::vector<float> query{0.1f, 0.9f, 0.2f, 0.8f};
  for (std::size_t index = 0; index < 3; ++index) {
    Keyframe keyframe;
    keyframe.id = index;
    keyframe.submap_id = 0;
    keyframe.local_pose = {static_cast<double>(index), 0.0, 0.0};
    keyframe.points = {{2.0, 0.0}, {2.0, 0.2}};
    keyframe.descriptor = query;
    ASSERT_TRUE(mutable_submap->AddKeyframe(keyframe));
  }
  ASSERT_TRUE(mutable_submap->Freeze());
  const std::vector<SubmapCandidate> candidates = RetrieveTopKSubmaps(
      query, {std::shared_ptr<const Submap2D>(mutable_submap)}, 2, 1, 3,
      0.5, 3);
  ASSERT_EQ(candidates.size(), 3u);
  EXPECT_EQ(candidates[0].reference_keyframe_id, 0u);
  EXPECT_EQ(candidates[1].reference_keyframe_id, 1u);
  EXPECT_EQ(candidates[2].reference_keyframe_id, 2u);
}

TEST(Submap2DTest, RetrievalEnforcesKeyframeSeparationBeforeTopK) {
  std::shared_ptr<Submap2D> mutable_submap(
      new Submap2D(0, 0.05, 10.0, false));
  const std::vector<float> query{0.1f, 0.9f, 0.2f, 0.8f};
  for (std::size_t id : {10u, 90u}) {
    Keyframe keyframe;
    keyframe.id = id;
    keyframe.submap_id = 0;
    keyframe.local_pose = {static_cast<double>(id), 0.0, 0.0};
    keyframe.points = {{2.0, 0.0}, {2.0, 0.2}};
    keyframe.descriptor = query;
    ASSERT_TRUE(mutable_submap->AddKeyframe(keyframe));
  }
  ASSERT_TRUE(mutable_submap->Freeze());
  const std::vector<SubmapCandidate> candidates = RetrieveTopKSubmaps(
      query, {std::shared_ptr<const Submap2D>(mutable_submap)}, 2, 1, 1,
      0.5, 2, 0, 100, 50);
  ASSERT_EQ(candidates.size(), 1u);
  EXPECT_EQ(candidates.front().reference_keyframe_id, 10u);
}

TEST(Submap2DTest, SubmapRelativeSeedRecoversAfterLargeTrajectoryDrift) {
  std::shared_ptr<Submap2D> mutable_submap(
      new Submap2D(0, 0.05, 16.0, false));
  Keyframe anchor;
  anchor.id = 0;
  anchor.submap_id = 0;
  anchor.local_pose = {100.0, 50.0, 0.1};
  anchor.points = {{1.0, 0.0}};
  anchor.descriptor = std::vector<float>(60, 0.5f);
  ASSERT_TRUE(mutable_submap->AddKeyframe(anchor));

  PointCloud2D query;
  for (int index = -30; index <= 30; ++index) {
    const double coordinate = index * 0.05;
    query.push_back({2.0, coordinate});
    query.push_back({-1.5, coordinate});
    query.push_back({coordinate, 1.25});
    query.push_back({coordinate, -1.75});
  }
  Keyframe reference;
  reference.id = 5;
  reference.submap_id = 0;
  reference.local_pose = Compose(anchor.local_pose, {3.0, 1.0, 0.2});
  reference.points = query;
  reference.descriptor = BuildPolarDescriptor(query, 10.0);
  ASSERT_TRUE(mutable_submap->AddKeyframe(reference));
  ASSERT_TRUE(mutable_submap->Freeze());
  const std::shared_ptr<const Submap2D> submap(mutable_submap);

  const std::vector<SubmapCandidate> candidates = RetrieveTopKSubmaps(
      reference.descriptor, {submap}, 3, 2, 1, 0.5);
  ASSERT_EQ(candidates.size(), 1u);
  EXPECT_EQ(candidates.front().reference_keyframe_id, 5u);

  ScanMatcherOptions options;
  options.linear_search_window = 0.3;
  options.angular_search_window = 0.15;
  options.linear_step = 0.05;
  options.angular_step = 0.025;
  options.use_likelihood_field = true;
  options.enable_fine_correlative = true;
  options.fine_candidate_count = 3;
  options.fine_linear_step = 0.025;
  options.fine_angular_window = 0.025;
  options.fine_angular_step = 0.0125;
  options.min_correspondences = 30;
  Pose2D initial = candidates.front().reference_relative_pose;
  initial.yaw = NormalizeAngle(initial.yaw + candidates.front().yaw_offset);
  const ScanMatchResult result =
      ScanMatcher(options).Match(submap->grid(), query, initial);
  ASSERT_TRUE(result.success);
  EXPECT_LT(TranslationDistance(result.pose, Pose2D{3.0, 1.0, 0.2}), 0.08);
  EXPECT_LT(std::abs(NormalizeAngle(result.pose.yaw - 0.2)), 0.03);

  const Pose2D matched_local =
      Compose(submap->anchor_local_pose(), result.pose);
  const Pose2D drifted_current{115.0, 60.0, matched_local.yaw + 0.2};
  const Pose2D correction = Between(drifted_current, matched_local);
  EXPECT_GT(std::hypot(correction.x, correction.y), 4.0);
}

TEST(Submap2DTest, GeometryRejectsDescriptorTieWithWrongStructure) {
  PointCloud2D correct_scan;
  PointCloud2D wrong_scan;
  for (int index = -35; index <= 35; ++index) {
    const double coordinate = index * 0.04;
    correct_scan.push_back({2.0, coordinate});
    correct_scan.push_back({coordinate, -1.6});
    wrong_scan.push_back({3.0, coordinate});
    wrong_scan.push_back({coordinate, 2.8});
  }
  const std::vector<float> shared_descriptor =
      BuildPolarDescriptor(correct_scan, 10.0);
  auto freeze = [&shared_descriptor](std::size_t id,
                                     const PointCloud2D& points) {
    std::shared_ptr<Submap2D> submap(
        new Submap2D(id, 0.05, 12.0, false));
    Keyframe keyframe;
    keyframe.id = id * 10;
    keyframe.submap_id = id;
    keyframe.points = points;
    keyframe.descriptor = shared_descriptor;
    EXPECT_TRUE(submap->AddKeyframe(keyframe));
    EXPECT_TRUE(submap->Freeze());
    return std::shared_ptr<const Submap2D>(submap);
  };
  const auto correct = freeze(0, correct_scan);
  const auto wrong = freeze(1, wrong_scan);
  const std::vector<SubmapCandidate> candidates = RetrieveTopKSubmaps(
      shared_descriptor, {correct, wrong}, 4, 2, 2, 0.5);
  ASSERT_EQ(candidates.size(), 2u);

  ScanMatcherOptions options;
  options.linear_search_window = 0.4;
  options.angular_search_window = 0.15;
  options.min_correspondences = 30;
  options.max_refinement_iterations = 0;
  const ScanMatcher matcher(options);
  const ScanMatchResult correct_result =
      matcher.Match(correct->grid(), correct_scan, Pose2D{});
  const ScanMatchResult wrong_result =
      matcher.Match(wrong->grid(), correct_scan, Pose2D{});
  ASSERT_TRUE(correct_result.success);
  EXPECT_GT(correct_result.coarse_score, wrong_result.coarse_score + 0.10);
}

}  // namespace lightweight_2d_slam

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
