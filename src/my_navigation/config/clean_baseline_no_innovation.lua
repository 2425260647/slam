include "cartographer_scout_2d.lua"

-- Clean baseline for bag quality diagnosis.
-- This keeps the current Cartographer source code, but disables both thesis
-- innovations through Lua so the run behaves like static-weight Cartographer 2D.

-- [Innovation 1] Disabled: no directional degeneracy driven anisotropic scan
-- matching weight. Keep the original scalar scan matcher translation weight.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled =
    false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.

-- [Innovation 2] Disabled: no consistency-anomaly driven backend odometry
-- down-weighting. Keep the standard static pose graph odometry weights.
POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = false
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
