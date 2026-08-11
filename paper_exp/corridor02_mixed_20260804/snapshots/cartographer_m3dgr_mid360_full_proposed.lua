include "cartographer_m3dgr_mid360_innovation1.lua"

-- M3DGR full Proposed variant: keep Innovation 1 enabled and add the IMU-free
-- LiDAR/odometry consistency gate and per-edge backend odometry down-weighting.
POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = true
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.slip_lateral_error_weight = 1.0
POSE_GRAPH.optimization_problem.slip_yaw_error_weight = 1.0
POSE_GRAPH.optimization_problem.slip_high_threshold = 0.03
POSE_GRAPH.optimization_problem.slip_low_threshold = 0.012
POSE_GRAPH.optimization_problem.slip_min_weight_scale = 0.1
POSE_GRAPH.optimization_problem.slip_max_weight_scale = 1.0
POSE_GRAPH.optimization_problem.slip_recovery_alpha = 0.05
POSE_GRAPH.optimization_problem.slip_min_motion_distance = 0.02
POSE_GRAPH.optimization_problem.slip_min_motion_angle = 0.01
POSE_GRAPH.optimization_problem.slip_lidar_reliability_gate_enabled = true
POSE_GRAPH.optimization_problem.slip_lidar_reliability_min = 0.5
POSE_GRAPH.optimization_problem.slip_degeneracy_metric_max_time_delta_sec = 0.25
POSE_GRAPH.optimization_problem.slip_unknown_keep_previous_weight = false
POSE_GRAPH.optimization_problem.slip_min_consecutive_anomalies = 2

return options
