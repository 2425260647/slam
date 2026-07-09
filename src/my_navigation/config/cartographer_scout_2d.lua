include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 30.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 30.

TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 30.

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
-- [Innovation 1: Directional Adaptive Fusion Parameters]
-- Enables 2D scan covariance based corridor degeneracy detection. This module
-- is automatically disabled by cartographer_ros when use_odometry = false.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_degeneracy_condition_number_threshold = 8.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_degeneracy_sigmoid_slope = 0.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_degeneracy_smoothing_alpha = 0.2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_degeneracy_min_num_points = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_degeneracy_eigenvalue_epsilon = 1e-4
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_odom_longitudinal_alpha = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_odom_lateral_alpha = 0.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_scan_longitudinal_beta = 0.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_scan_lateral_beta = 0.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_min_scan_weight_scale = 0.2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_log_scale_change_threshold = 0.25

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.08
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 70
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

return options
