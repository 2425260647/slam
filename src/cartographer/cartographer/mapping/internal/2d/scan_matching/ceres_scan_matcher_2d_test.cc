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

#include <cmath>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

class CeresScanMatcherTest : public ::testing::Test {
 protected:
  CeresScanMatcherTest()
      : probability_grid_(
            MapLimits(1., Eigen::Vector2d(10., 10.), CellLimits(20, 20)),
            &conversion_tables_) {
    probability_grid_.SetProbability(
        probability_grid_.limits().GetCellIndex(Eigen::Vector2f(-3.5f, 2.5f)),
        kMaxProbability);

    point_cloud_.push_back({Eigen::Vector3f{-3.f, 2.f, 0.f}});

    auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
          occupied_space_weight = 1.,
          translation_weight = 0.1,
          rotation_weight = 1.5,
          ceres_solver_options = {
            use_nonmonotonic_steps = true,
            max_num_iterations = 50,
            num_threads = 1,
          },
        })text");
    const proto::CeresScanMatcherOptions2D options =
        CreateCeresScanMatcherOptions2D(parameter_dictionary.get());
    ceres_scan_matcher_ = absl::make_unique<CeresScanMatcher2D>(options);
  }

  void TestFromInitialPose(const transform::Rigid2d& initial_pose) {
    transform::Rigid2d pose;
    const transform::Rigid2d expected_pose =
        transform::Rigid2d::Translation({-0.5, 0.5});
    ceres::Solver::Summary summary;
    ceres_scan_matcher_->Match(initial_pose.translation(), initial_pose,
                               point_cloud_, probability_grid_, &pose,
                               &summary);
    EXPECT_NEAR(0., summary.final_cost, 1e-2) << summary.FullReport();
    EXPECT_THAT(pose, transform::IsNearly(expected_pose, 1e-2))
        << "Actual: " << transform::ToProto(pose).DebugString()
        << "\nExpected: " << transform::ToProto(expected_pose).DebugString();
  }

  ValueConversionTables conversion_tables_;
  ProbabilityGrid probability_grid_;
  sensor::PointCloud point_cloud_;
  std::unique_ptr<CeresScanMatcher2D> ceres_scan_matcher_;
};

TEST_F(CeresScanMatcherTest, testPerfectEstimate) {
  TestFromInitialPose(transform::Rigid2d::Translation({-0.5, 0.5}));
}

TEST_F(CeresScanMatcherTest, testOptimizeAlongX) {
  TestFromInitialPose(transform::Rigid2d::Translation({-0.3, 0.5}));
}

TEST_F(CeresScanMatcherTest, testOptimizeAlongY) {
  TestFromInitialPose(transform::Rigid2d::Translation({-0.45, 0.3}));
}

TEST_F(CeresScanMatcherTest, testOptimizeAlongXY) {
  TestFromInitialPose(transform::Rigid2d::Translation({-0.3, 0.3}));
}

TEST(CeresScanMatcherDirectionalFrameTest,
     RotatesTrackingDirectionIntoLocalSlamFrame) {
  auto parameter_dictionary = common::MakeDictionary(R"text(
      return {
        occupied_space_weight = 1.,
        translation_weight = 0.1,
        rotation_weight = 1.5,
        directional_adaptive_fusion_enabled = true,
        directional_degeneracy_min_num_points = 10,
        ceres_solver_options = {
          use_nonmonotonic_steps = true,
          max_num_iterations = 5,
          num_threads = 1,
        },
      })text");
  const proto::CeresScanMatcherOptions2D options =
      CreateCeresScanMatcherOptions2D(parameter_dictionary.get());
  CeresScanMatcher2D matcher(options);

  ValueConversionTables conversion_tables;
  ProbabilityGrid probability_grid(
      MapLimits(1., Eigen::Vector2d(10., 10.), CellLimits(20, 20)),
      &conversion_tables);
  sensor::PointCloud longitudinal_tracking_cloud;
  for (int i = -10; i <= 10; ++i) {
    longitudinal_tracking_cloud.push_back(
        {Eigen::Vector3f(static_cast<float>(i) * 0.1f, 0.f, 0.f)});
  }

  // [Innovation 1] A tracking-frame X principal axis must become local Y
  // [Innovation 1] when the initial Ceres pose is rotated by +90 degrees.
  const double half_pi = std::acos(-1.) * 0.5;
  const transform::Rigid2d initial_pose(Eigen::Vector2d::Zero(), half_pi);
  transform::Rigid2d pose_estimate;
  ceres::Solver::Summary summary;
  matcher.Match(initial_pose.translation(), initial_pose,
                longitudinal_tracking_cloud, probability_grid, &pose_estimate,
                &summary);

  const DirectionalDegeneracyMetric metric =
      GetLatestDirectionalDegeneracyMetric();
  ASSERT_TRUE(metric.enabled);
  ASSERT_TRUE(metric.valid);
  EXPECT_GT(metric.adaptation_strength, 0.9);
  EXPECT_GT(metric.longitudinal_scale, 1.);
  EXPECT_NEAR(0., std::abs(metric.direction.x()), 1e-6);
  EXPECT_NEAR(1., std::abs(metric.direction.y()), 1e-6);
}

TEST(CeresScanMatcherDirectionalFallbackTest,
     IsotropicGeometryKeepsExactBaselineWeight) {
  auto parameter_dictionary = common::MakeDictionary(R"text(
      return {
        occupied_space_weight = 1.,
        translation_weight = 0.1,
        rotation_weight = 1.5,
        directional_adaptive_fusion_enabled = true,
        directional_degeneracy_min_num_points = 10,
        directional_adaptive_activation_confidence = 0.5,
        ceres_solver_options = {
          use_nonmonotonic_steps = true,
          max_num_iterations = 5,
          num_threads = 1,
        },
      })text");
  const proto::CeresScanMatcherOptions2D options =
      CreateCeresScanMatcherOptions2D(parameter_dictionary.get());
  CeresScanMatcher2D matcher(options);

  ValueConversionTables conversion_tables;
  ProbabilityGrid probability_grid(
      MapLimits(1., Eigen::Vector2d(10., 10.), CellLimits(20, 20)),
      &conversion_tables);
  sensor::PointCloud isotropic_cloud;
  constexpr int kPointCount = 40;
  const double two_pi = 2. * std::acos(-1.);
  for (int i = 0; i < kPointCount; ++i) {
    const double angle = two_pi * static_cast<double>(i) / kPointCount;
    isotropic_cloud.push_back({Eigen::Vector3f(
        static_cast<float>(std::cos(angle)),
        static_cast<float>(std::sin(angle)), 0.f)});
  }

  const transform::Rigid2d initial_pose = transform::Rigid2d::Identity();
  transform::Rigid2d pose_estimate;
  ceres::Solver::Summary summary;
  matcher.Match(initial_pose.translation(), initial_pose, isotropic_cloud,
                probability_grid, &pose_estimate, &summary, 0.8);

  const DirectionalDegeneracyMetric metric =
      GetLatestDirectionalDegeneracyMetric();
  ASSERT_TRUE(metric.enabled);
  ASSERT_TRUE(metric.valid);
  EXPECT_LT(metric.confidence, 0.5);
  EXPECT_DOUBLE_EQ(metric.adaptation_strength, 0.);
  EXPECT_DOUBLE_EQ(metric.longitudinal_scale, 1.);
  EXPECT_DOUBLE_EQ(metric.lateral_scale, 1.);
}

TEST(CeresScanMatcherDirectionalFallbackTest,
     OrthogonalDegeneracyAxisKeepsExactBaselineWeight) {
  auto parameter_dictionary = common::MakeDictionary(R"text(
      return {
        occupied_space_weight = 1.,
        translation_weight = 0.1,
        rotation_weight = 1.5,
        directional_adaptive_fusion_enabled = true,
        directional_degeneracy_min_num_points = 10,
        directional_adaptive_activation_confidence = 0.5,
        directional_adaptive_motion_alignment_min_cosine = 0.7,
        ceres_solver_options = {
          use_nonmonotonic_steps = true,
          max_num_iterations = 5,
          num_threads = 1,
        },
      })text");
  CeresScanMatcher2D matcher(
      CreateCeresScanMatcherOptions2D(parameter_dictionary.get()));

  ValueConversionTables conversion_tables;
  ProbabilityGrid probability_grid(
      MapLimits(1., Eigen::Vector2d(10., 10.), CellLimits(20, 20)),
      &conversion_tables);
  sensor::PointCloud lateral_tracking_cloud;
  for (int i = -10; i <= 10; ++i) {
    lateral_tracking_cloud.push_back(
        {Eigen::Vector3f(0.f, static_cast<float>(i) * 0.1f, 0.f)});
  }

  transform::Rigid2d pose_estimate;
  ceres::Solver::Summary summary;
  matcher.Match(Eigen::Vector2d::Zero(), transform::Rigid2d::Identity(),
                lateral_tracking_cloud, probability_grid, &pose_estimate,
                &summary, 0.8);

  const DirectionalDegeneracyMetric metric =
      GetLatestDirectionalDegeneracyMetric();
  ASSERT_TRUE(metric.valid);
  EXPECT_GT(metric.confidence, 0.9);
  EXPECT_DOUBLE_EQ(metric.adaptation_strength, 0.);
  EXPECT_DOUBLE_EQ(metric.longitudinal_scale, 1.);
  EXPECT_DOUBLE_EQ(metric.lateral_scale, 1.);
}

TEST(CeresScanMatcherDirectionalLimitTest, CapsLongitudinalScale) {
  auto parameter_dictionary = common::MakeDictionary(R"text(
      return {
        occupied_space_weight = 1.,
        translation_weight = 0.1,
        rotation_weight = 1.5,
        directional_adaptive_fusion_enabled = true,
        directional_degeneracy_min_num_points = 10,
        directional_adaptive_activation_confidence = 0.5,
        directional_adaptive_motion_alignment_min_cosine = 0.7,
        directional_adaptive_max_longitudinal_scale = 1.5,
        ceres_solver_options = {
          use_nonmonotonic_steps = true,
          max_num_iterations = 5,
          num_threads = 1,
        },
      })text");
  CeresScanMatcher2D matcher(
      CreateCeresScanMatcherOptions2D(parameter_dictionary.get()));

  ValueConversionTables conversion_tables;
  ProbabilityGrid probability_grid(
      MapLimits(1., Eigen::Vector2d(10., 10.), CellLimits(20, 20)),
      &conversion_tables);
  sensor::PointCloud longitudinal_tracking_cloud;
  for (int i = -10; i <= 10; ++i) {
    longitudinal_tracking_cloud.push_back(
        {Eigen::Vector3f(static_cast<float>(i) * 0.1f, 0.f, 0.f)});
  }

  transform::Rigid2d pose_estimate;
  ceres::Solver::Summary summary;
  matcher.Match(Eigen::Vector2d::Zero(), transform::Rigid2d::Identity(),
                longitudinal_tracking_cloud, probability_grid, &pose_estimate,
                &summary, 0.8);

  const DirectionalDegeneracyMetric metric =
      GetLatestDirectionalDegeneracyMetric();
  ASSERT_TRUE(metric.valid);
  EXPECT_GT(metric.adaptation_strength, 0.9);
  EXPECT_NEAR(metric.longitudinal_scale, 1.5, 1e-12);
  EXPECT_DOUBLE_EQ(metric.lateral_scale, 1.);
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
