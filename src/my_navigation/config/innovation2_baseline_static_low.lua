include "cartographer_scout_2d.lua"

-- [Innovation 2 experiment] Baseline A: static low backend odometry weight.
-- This reduces the influence of slipped odom, but it also weakens useful
-- odometry everywhere, including non-slip corridor segments.
POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = false
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

return options
