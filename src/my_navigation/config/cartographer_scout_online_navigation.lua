include "cartographer_scout_2d.lua"

-- Scout real-robot online mapping and navigation interface.
-- The chassis driver owns odom -> base_link. Cartographer only publishes
-- map -> odom, preventing two nodes from publishing the same TF edge.
options.published_frame = "odom"
options.provide_odom_frame = false
options.publish_frame_projected_to_2d = false
options.use_odometry = true

-- A 50 Hz global correction is sufficient for A* + TEB. The i7-10610U has
-- four physical cores, so Ceres uses four threads and leaves CPU time for
-- lidar, chassis and planning callbacks.
options.pose_publish_period_sec = 0.02
options.submap_publish_period_sec = 0.5
options.trajectory_publish_period_sec = 0.1
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

-- Keep both thesis modules active in the production configuration.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled =
    true
POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = true

-- These values are replaced only after the repeat_03 tuning run and the
-- repeat_01/repeat_02 independent validation pass.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability =
    0.60
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability =
    0.49

return options
