#include "cartographer/mapping/internal/optimization/slip_detector.h"

#include "Eigen/Core"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

// [Innovation 2] 构造仅包含一致性异常检测所需字段的最小测试参数。
proto::OptimizationProblemOptions MakeOptions() {
  proto::OptimizationProblemOptions options;
  options.set_slip_adaptive_odometry_weight_enabled(true);
  options.set_slip_lateral_error_weight(1.);
  options.set_slip_yaw_error_weight(1.);
  options.set_slip_high_threshold(0.2);
  options.set_slip_low_threshold(0.1);
  options.set_slip_min_weight_scale(0.1);
  options.set_slip_max_weight_scale(1.);
  options.set_slip_recovery_alpha(0.25);
  options.set_slip_min_motion_distance(0.);
  options.set_slip_min_motion_angle(0.);
  options.set_slip_lidar_reliability_gate_enabled(false);
  options.set_slip_lidar_reliability_min(0.5);
  options.set_slip_unknown_keep_previous_weight(true);
  options.set_slip_min_consecutive_anomalies(1);
  return options;
}

// [Innovation 2] 测试位姿均表示相邻节点之间的二维相对运动。
transform::Rigid2d Pose(const double x, const double y, const double yaw) {
  return transform::Rigid2d(Eigen::Vector2d(x, y), yaw);
}

TEST(SlipDetectorTest, DisabledKeepsFullWeight) {
  auto options = MakeOptions();
  options.set_slip_adaptive_odometry_weight_enabled(false);
  SlipDetector detector(options);

  const SlipDetectorResult result =
      detector.Update(Pose(0., 0., 0.), Pose(0., 1., 1.));

  EXPECT_FALSE(result.enabled);
  EXPECT_FALSE(result.slipping);
  EXPECT_DOUBLE_EQ(result.weight_scale, 1.);
}

TEST(SlipDetectorTest, NormalizesYawAcrossPiBoundary) {
  constexpr double kPi = 3.14159265358979323846;
  SlipDetector detector(MakeOptions());

  const SlipDetectorResult result =
      detector.Update(Pose(0., 0., kPi - 0.01), Pose(0., 0., -kPi + 0.01));

  EXPECT_NEAR(result.yaw_error, 0.02, 1e-9);
  EXPECT_FALSE(result.slipping);
}

TEST(SlipDetectorTest, HysteresisHoldsStateInMiddleBand) {
  SlipDetector detector(MakeOptions());

  const SlipDetectorResult triggered =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.3, 0.));
  const SlipDetectorResult middle =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.15, 0.));
  const SlipDetectorResult recovered =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.05, 0.));

  EXPECT_TRUE(triggered.slipping);
  EXPECT_NEAR(triggered.weight_scale, 2. / 3., 1e-12);
  EXPECT_TRUE(middle.slipping);
  EXPECT_NEAR(middle.weight_scale, 2. / 3., 1e-12);
  EXPECT_FALSE(recovered.slipping);
  EXPECT_NEAR(recovered.weight_scale, 0.75, 1e-12);
}

TEST(SlipDetectorTest, UnknownReliabilityKeepsPreviousStateAndWeight) {
  auto options = MakeOptions();
  options.set_slip_lidar_reliability_gate_enabled(true);
  SlipDetector detector(options);

  const SlipDetectorResult unknown_while_normal =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.3, 0.), false, 0.);
  const SlipDetectorResult reliable_anomaly =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.3, 0.), true, 1.);
  const SlipDetectorResult unknown_while_anomalous =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0., 0.), false, 0.);

  EXPECT_FALSE(unknown_while_normal.slipping);
  EXPECT_DOUBLE_EQ(unknown_while_normal.weight_scale, 1.);
  EXPECT_TRUE(reliable_anomaly.slipping);
  EXPECT_LT(reliable_anomaly.weight_scale, 1.);
  EXPECT_TRUE(unknown_while_anomalous.slipping);
  EXPECT_DOUBLE_EQ(unknown_while_anomalous.weight_scale,
                   reliable_anomaly.weight_scale);
}

TEST(SlipDetectorTest, UnknownEvidenceRestoresBaselineWhenConfigured) {
  auto options = MakeOptions();
  options.set_slip_lidar_reliability_gate_enabled(true);
  options.set_slip_unknown_keep_previous_weight(false);
  SlipDetector detector(options);

  const SlipDetectorResult anomaly =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.3, 0.), true, 1.);
  const SlipDetectorResult unknown =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.3, 0.), false, 0.);

  EXPECT_TRUE(anomaly.slipping);
  EXPECT_LT(anomaly.weight_scale, 1.);
  EXPECT_FALSE(unknown.slipping);
  EXPECT_DOUBLE_EQ(unknown.weight_scale, 1.);
}

TEST(SlipDetectorTest, RequiresConsecutiveReliableAnomalies) {
  auto options = MakeOptions();
  options.set_slip_min_consecutive_anomalies(2);
  SlipDetector detector(options);

  const SlipDetectorResult first =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.3, 0.));
  const SlipDetectorResult second =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.3, 0.));

  EXPECT_FALSE(first.slipping);
  EXPECT_DOUBLE_EQ(first.weight_scale, 1.);
  EXPECT_TRUE(second.slipping);
  EXPECT_LT(second.weight_scale, 1.);
}

TEST(SlipDetectorTest, ProjectsResidualOntoObservableDirection) {
  const Eigen::Vector2d error(3., 4.);
  const Eigen::Vector2d degeneracy_direction = Eigen::Vector2d::UnitX();

  EXPECT_DOUBLE_EQ(ComputeObservableTranslationError(
                       error, degeneracy_direction, 0.),
                   5.);
  EXPECT_DOUBLE_EQ(ComputeObservableTranslationError(
                       error, degeneracy_direction, 1.),
                   4.);
  EXPECT_DOUBLE_EQ(ComputeObservableTranslationError(
                       error, degeneracy_direction, 0.5),
                   4.5);
}

TEST(SlipDetectorTest, RecoversWeightMonotonicallyAfterAnomaly) {
  SlipDetector detector(MakeOptions());
  detector.Update(Pose(0., 0., 0.), Pose(0., 0.3, 0.));

  const SlipDetectorResult first_recovery =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.05, 0.));
  const SlipDetectorResult second_recovery =
      detector.Update(Pose(0., 0., 0.), Pose(0., 0.05, 0.));

  EXPECT_FALSE(first_recovery.slipping);
  EXPECT_GT(first_recovery.weight_scale, 0.1);
  EXPECT_LT(first_recovery.weight_scale, second_recovery.weight_scale);
  EXPECT_LT(second_recovery.weight_scale, 1.);
}

TEST(SlipDetectorTest, TranslationAnomalyPreservesRotationWeight) {
  SlipDetector detector(MakeOptions());

  const SlipDetectorResult result = detector.UpdateFromResiduals(
      0.3, 0., 0.3, 0., true, 1.);

  EXPECT_TRUE(result.slipping);
  EXPECT_NEAR(result.translation_weight_scale, 2. / 3., 1e-12);
  EXPECT_DOUBLE_EQ(result.rotation_weight_scale, 1.);
}

TEST(SlipDetectorTest, RotationAnomalyPreservesTranslationWeight) {
  SlipDetector detector(MakeOptions());

  const SlipDetectorResult result = detector.UpdateFromResiduals(
      0., 0.4, 0., 0.4, true, 1.);

  EXPECT_TRUE(result.slipping);
  EXPECT_DOUBLE_EQ(result.translation_weight_scale, 1.);
  EXPECT_NEAR(result.rotation_weight_scale, 0.5, 1e-12);
}

TEST(SlipDetectorTest, StationaryWheelEdgeRestoresBaseline) {
  auto options = MakeOptions();
  options.set_slip_min_motion_distance(0.02);
  options.set_slip_min_motion_angle(0.01);
  SlipDetector detector(options);
  const SlipDetectorResult anomaly = detector.UpdateFromResiduals(
      0.3, 0.4, 0.3, 0.4, true, 1.);
  ASSERT_TRUE(anomaly.slipping);

  const SlipDetectorResult stationary = detector.UpdateFromResiduals(
      0.3, 0.4, 0., 0., true, 1.);

  EXPECT_FALSE(stationary.slipping);
  EXPECT_DOUBLE_EQ(stationary.translation_weight_scale, 1.);
  EXPECT_DOUBLE_EQ(stationary.rotation_weight_scale, 1.);
  EXPECT_DOUBLE_EQ(stationary.weight_scale, 1.);
}

}
}
}
}
