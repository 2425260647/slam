include "repeat03_clean.lua"

-- Experimental repeat03 variant. Keep the input, odometry, submap cadence,
-- and unknown-space policy identical to the tuned probability-grid baseline.
-- Only replace the local map representation and scan-matching residual with
-- Cartographer's existing 2D TSDF implementation.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "TSDF"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type =
    "TSDF_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter = {
  truncation_distance = 0.30,
  maximum_weight = 10.,
  update_free_space = false,
  normal_estimation_options = {
    num_normal_samples = 4,
    sample_radius = 0.50,
  },
  project_sdf_distance_to_scan_normal = true,
  update_weight_range_exponent = 0,
  update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
  update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
}

return options
