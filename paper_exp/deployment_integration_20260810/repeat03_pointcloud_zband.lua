include "repeat03_pointcloud.lua"

-- Keep the horizontal LiDAR returns in the tracking frame.  The recorded
-- Velodyne has several vertical rings; inserting all rings into a 2D grid
-- makes walls unnecessarily thick.  This band retains the near-horizontal
-- rings while leaving the input used by the matcher otherwise unchanged.
TRAJECTORY_BUILDER_2D.min_z = 0.13
TRAJECTORY_BUILDER_2D.max_z = 0.22
TRAJECTORY_BUILDER_2D.submap_insertion_voxel_filter_size = 0.05

return options
