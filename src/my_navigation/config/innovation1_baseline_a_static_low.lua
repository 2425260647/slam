include "cartographer_scout_2d.lua"

-- [Innovation 1 experiment] Baseline A: static isotropic low/normal odometry
-- prior. This keeps the original 2D Ceres translation prior scalar and disables
-- the directional adaptive fusion module.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled = false

return options
