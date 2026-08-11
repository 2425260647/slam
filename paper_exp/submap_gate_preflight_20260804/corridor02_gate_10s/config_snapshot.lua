include "cartographer_m3dgr_mid360.lua"

-- Experimental, fail-open submap admission guard. The geometry diagnostic is
-- read-only; this file enables only the admission decision and keeps the old
-- OAD/directional weighting modules disabled for an isolated experiment.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled =
    false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.lidar_only_observability_diagnostic_enabled =
    true
TRAJECTORY_BUILDER_2D.submap_insertion_gate_enabled = true
TRAJECTORY_BUILDER_2D.submap_insertion_gate_min_information = 10.0
TRAJECTORY_BUILDER_2D.submap_insertion_gate_max_condition_number = 0.0
TRAJECTORY_BUILDER_2D.submap_insertion_gate_bad_scan_count = 2
TRAJECTORY_BUILDER_2D.submap_insertion_gate_good_scan_count = 2
TRAJECTORY_BUILDER_2D.submap_insertion_gate_max_consecutive_blocked_scans = 20

return options
