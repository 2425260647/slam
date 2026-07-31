include "cartographer_scout_2d.lua"

-- [Innovation 1 experiment] Baseline A: static isotropic low/normal front-end
-- translation prior. Innovation 2 is explicitly disabled so this ablation
-- changes only the Ceres scan-matcher translation prior.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled = false
POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = false
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
