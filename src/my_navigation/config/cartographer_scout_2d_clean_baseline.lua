include "cartographer_scout_2d.lua"

-- Clean Cartographer 2D baseline for the common Gazebo replay.  The stock
-- Scout configuration is already the baseline; project-specific keys are not
-- assigned here because Cartographer silently ignores unknown Lua fields.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
