include "cartographer_m3dgr_mid360_clean_baseline.lua"

-- Proposed OAD-CSM configuration. This is intentionally separate from all
-- historical diagnostic and C0/C1/C2/C3 configurations.
local csm = TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher
csm.oad_enabled = true
csm.oad_weak_map_score_threshold = 0.54
csm.oad_wide_peak_margin = 0.02
csm.oad_linear_search_window = 0.45
csm.oad_angular_search_window = math.rad(25.)
csm.oad_coarse_to_fine_levels = 2
csm.oad_min_peak_margin = 0.015
csm.oad_min_score_gain = 0.02
csm.oad_confirmation_count = 2
csm.oad_max_candidates = 1800
csm.oad_max_search_time_ms = 20.

return options
