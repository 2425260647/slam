include "cartographer_scout_2d.lua"

-- [Innovation 1 experiment] Baseline B: static isotropic high front-end prior.
-- The value is close to the strongest longitudinal prior observed from the
-- proposed adaptive method, but it is applied in every direction and scene.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 35.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled = false
POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = false
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
