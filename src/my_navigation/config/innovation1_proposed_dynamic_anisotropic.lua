include "cartographer_scout_2d.lua"

-- [Innovation 1 experiment] Proposed: base prior plus directional adaptive
-- anisotropic fusion. Longitudinal/lateral weights are computed online from
-- the current 2D scan covariance in CeresScanMatcher2D. Innovation 2 remains
-- disabled so the result isolates the front-end anisotropic prior.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled = true
POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = false
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
