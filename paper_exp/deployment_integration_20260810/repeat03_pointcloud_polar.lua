include "repeat03_pointcloud.lua"

-- Keep the dense point cloud for scan matching, but de-duplicate returns only
-- when inserting them into the 2D submaps. 0.25 degrees matches the scan
-- projection used by the existing repeat03 replay chain.
TRAJECTORY_BUILDER_2D.submap_insertion_polar_filter_enabled = true
TRAJECTORY_BUILDER_2D.submap_insertion_polar_angular_resolution = 0.0043633231

return options
