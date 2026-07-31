include "innovation2_proposed_slip_adaptive.lua"

-- [Innovation 2 experiment] Gate ablation: keep the same consistency detector,
-- thresholds, recovery, base odometry weights, and Innovation 1 front-end, but
-- remove only the time-aligned D_conf qualification gate.
POSE_GRAPH.optimization_problem.slip_lidar_reliability_gate_enabled = false

return options
