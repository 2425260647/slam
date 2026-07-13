include "cartographer_scout_2d.lua"

-- [Innovation 2 experiment] Baseline B: static high backend odometry weight.
-- Innovation 1 is kept enabled, while slip-adaptive backend weighting is
-- disabled. This case shows what happens when a slipped odom edge is still
-- trusted by the pose graph.
POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = false
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
