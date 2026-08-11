include "repeat03_clean.lua"

-- Keep the scan-matching cloud and submap-insertion cloud separate. The
-- matcher still uses the tuned adaptive cloud; map endpoints are downsampled
-- to 5 cm to reduce wall thickening from repeated noisy hits.
TRAJECTORY_BUILDER_2D.submap_insertion_voxel_filter_size = 0.05

return options
