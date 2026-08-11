include "cartographer_scout_2d.lua"

-- Clean Cartographer 2D baseline for the common Gazebo replay. Keep all
-- sensor, map and pose-graph parameters identical to the Scout configuration,
-- but explicitly disable every project-specific behavioral module.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.oad_enabled = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled =
    false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.lidar_only_observability_diagnostic_enabled =
    false
TRAJECTORY_BUILDER_2D.submap_insertion_gate_enabled = false

POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = false
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
