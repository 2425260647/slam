include "corridor02_clean.lua"

-- Cartographer consumes the MID-360 PointCloud2 directly so per-point Livox
-- offset_time remains available for motion compensation. The existing
-- deskewed /scan remains the local-grid input.
options.num_laser_scans = 0
options.num_point_clouds = 1

-- Match the vertical band used by the validated Corridor02 /scan baseline.
TRAJECTORY_BUILDER_2D.min_z = 0.20
TRAJECTORY_BUILDER_2D.max_z = 0.60
TRAJECTORY_BUILDER_2D.submap_insertion_voxel_filter_size = 0.025
TRAJECTORY_BUILDER_2D.submap_insertion_polar_filter_enabled = true
TRAJECTORY_BUILDER_2D.submap_insertion_polar_angular_resolution = 0.0021816616

return options
