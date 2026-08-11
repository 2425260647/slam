include "repeat03_pointcloud.lua"

-- Fine angular de-duplication variant. This retains up to 2880 azimuth bins
-- per revolution to recover wall continuity while still collapsing duplicate
-- multi-ring returns before 2D submap insertion.
TRAJECTORY_BUILDER_2D.submap_insertion_polar_filter_enabled = true
TRAJECTORY_BUILDER_2D.submap_insertion_polar_angular_resolution = 0.0021816616

return options
