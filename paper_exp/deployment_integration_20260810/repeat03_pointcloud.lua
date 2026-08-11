include "repeat03_clean.lua"

-- Consume the original PointCloud2 directly so Cartographer can preserve the
-- per-point `time` field during scan undistortion. /scan remains available to
-- local_grid_mapper, but is no longer the global SLAM input.
options.num_laser_scans = 0
options.num_point_clouds = 1
TRAJECTORY_BUILDER_2D.submap_insertion_voxel_filter_size = 0.025

return options
