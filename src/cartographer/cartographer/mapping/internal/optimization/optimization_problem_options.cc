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

#include "cartographer/mapping/internal/optimization/optimization_problem_options.h"

#include "cartographer/common/internal/ceres_solver_options.h"

namespace cartographer {
namespace mapping {
namespace optimization {

proto::OptimizationProblemOptions CreateOptimizationProblemOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::OptimizationProblemOptions options;
  options.set_huber_scale(parameter_dictionary->GetDouble("huber_scale"));
  options.set_acceleration_weight(
      parameter_dictionary->GetDouble("acceleration_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_odometry_translation_weight(
      parameter_dictionary->GetDouble("odometry_translation_weight"));
  options.set_odometry_rotation_weight(
      parameter_dictionary->GetDouble("odometry_rotation_weight"));
  // [Innovation 2] Optional parameters keep upstream Lua files compatible.
  options.set_slip_adaptive_odometry_weight_enabled(
      parameter_dictionary->HasKey("slip_adaptive_odometry_weight_enabled")
          ? parameter_dictionary->GetBool(
                "slip_adaptive_odometry_weight_enabled")
          : false);
  options.set_slip_lateral_error_weight(
      parameter_dictionary->HasKey("slip_lateral_error_weight")
          ? parameter_dictionary->GetDouble("slip_lateral_error_weight")
          : 1.);
  options.set_slip_yaw_error_weight(
      parameter_dictionary->HasKey("slip_yaw_error_weight")
          ? parameter_dictionary->GetDouble("slip_yaw_error_weight")
          : 1.);
  options.set_slip_high_threshold(
      parameter_dictionary->HasKey("slip_high_threshold")
          ? parameter_dictionary->GetDouble("slip_high_threshold")
          : 0.15);
  options.set_slip_low_threshold(
      parameter_dictionary->HasKey("slip_low_threshold")
          ? parameter_dictionary->GetDouble("slip_low_threshold")
          : 0.05);
  options.set_slip_min_weight_scale(
      parameter_dictionary->HasKey("slip_min_weight_scale")
          ? parameter_dictionary->GetDouble("slip_min_weight_scale")
          : 0.1);
  options.set_slip_max_weight_scale(
      parameter_dictionary->HasKey("slip_max_weight_scale")
          ? parameter_dictionary->GetDouble("slip_max_weight_scale")
          : 1.);
  options.set_slip_recovery_alpha(
      parameter_dictionary->HasKey("slip_recovery_alpha")
          ? parameter_dictionary->GetDouble("slip_recovery_alpha")
          : 0.05);
  options.set_slip_min_motion_distance(
      parameter_dictionary->HasKey("slip_min_motion_distance")
          ? parameter_dictionary->GetDouble("slip_min_motion_distance")
          : 0.02);
  options.set_slip_min_motion_angle(
      parameter_dictionary->HasKey("slip_min_motion_angle")
          ? parameter_dictionary->GetDouble("slip_min_motion_angle")
          : 0.01);
  options.set_slip_lidar_reliability_gate_enabled(
      parameter_dictionary->HasKey("slip_lidar_reliability_gate_enabled")
          ? parameter_dictionary->GetBool("slip_lidar_reliability_gate_enabled")
          : false);
  options.set_slip_lidar_reliability_min(
      parameter_dictionary->HasKey("slip_lidar_reliability_min")
          ? parameter_dictionary->GetDouble("slip_lidar_reliability_min")
          : 0.6);
  options.set_slip_degeneracy_metric_max_time_delta_sec(
      parameter_dictionary->HasKey("slip_degeneracy_metric_max_time_delta_sec")
          ? parameter_dictionary->GetDouble(
                "slip_degeneracy_metric_max_time_delta_sec")
          : 0.25);
  options.set_slip_unknown_keep_previous_weight(
      parameter_dictionary->HasKey("slip_unknown_keep_previous_weight")
          ? parameter_dictionary->GetBool("slip_unknown_keep_previous_weight")
          : true);
  options.set_local_slam_pose_translation_weight(
      parameter_dictionary->GetDouble("local_slam_pose_translation_weight"));
  options.set_local_slam_pose_rotation_weight(
      parameter_dictionary->GetDouble("local_slam_pose_rotation_weight"));
  options.set_fixed_frame_pose_translation_weight(
      parameter_dictionary->GetDouble("fixed_frame_pose_translation_weight"));
  options.set_fixed_frame_pose_rotation_weight(
      parameter_dictionary->GetDouble("fixed_frame_pose_rotation_weight"));
  options.set_fixed_frame_pose_use_tolerant_loss(
      parameter_dictionary->GetBool("fixed_frame_pose_use_tolerant_loss"));
  options.set_fixed_frame_pose_tolerant_loss_param_a(
      parameter_dictionary->GetDouble(
          "fixed_frame_pose_tolerant_loss_param_a"));
  options.set_fixed_frame_pose_tolerant_loss_param_b(
      parameter_dictionary->GetDouble(
          "fixed_frame_pose_tolerant_loss_param_b"));
  options.set_log_solver_summary(
      parameter_dictionary->GetBool("log_solver_summary"));
  options.set_use_online_imu_extrinsics_in_3d(
      parameter_dictionary->GetBool("use_online_imu_extrinsics_in_3d"));
  options.set_fix_z_in_3d(parameter_dictionary->GetBool("fix_z_in_3d"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
