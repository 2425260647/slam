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
-- Publish map->odom corrections faster than the 5 Hz navigation loop. This
-- reduces the chance that move_base asks for a transform a few milliseconds
-- newer than Cartographer's latest sample.
options.pose_publish_period_sec = 0.01
options.submap_publish_period_sec = 0.5
options.trajectory_publish_period_sec = 0.1
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

-- Do not add project-specific Lua keys here unless the Cartographer binary
-- reads them.  Unknown table fields are silently ignored, which can make a
-- claimed adaptive module appear enabled while having no runtime effect.

-- These values are replaced only after the repeat_03 tuning run and the
-- repeat_01/repeat_02 independent validation pass.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability =
    0.60
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability =
    0.49

return options
