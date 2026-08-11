include "cartographer_m3dgr_mid360.lua"

-- M3DGR clean ablation baseline:
-- use the same base_footprint/livox_frame chain and all common parameters as
-- the proposed variants, while disabling both thesis innovations.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled =
    false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.

POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = false
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
