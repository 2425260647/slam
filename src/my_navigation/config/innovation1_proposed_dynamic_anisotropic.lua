include "cartographer_scout_2d.lua"

-- [Innovation 1 experiment] Proposed: base prior plus directional adaptive
-- anisotropic fusion. Longitudinal/lateral weights are computed online from
-- the current 2D scan covariance in CeresScanMatcher2D.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled = true

return options
