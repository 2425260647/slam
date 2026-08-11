include "cartographer_m3dgr_mid360_oad_diagnostic.lua"

-- Superset check: retain the production +/-20 degree angular window while
-- extending translation to 1 m. This is used only to audit the narrower
-- diagnostic configuration and is not a production controller.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.candidate_diagnostic_angular_search_window = math.rad(20.)

return options
