# Innovation 1: Directional Adaptive Fusion

This change adds a 2D directional degeneracy detector and an anisotropic
translation prior to Cartographer's 2D Ceres scan matcher. It uses only the
current 2D LiDAR scan and the existing odometry-aided pose prediction path. No
IMU logic is added.

## Math

For every Ceres scan match, the filtered 2D scan points are used to compute:

```text
mean = average(p_i)
Sigma = average((p_i - mean) * (p_i - mean)^T)
```

`Eigen::SelfAdjointEigenSolver` gives `lambda_max`, `lambda_min`, and the
principal eigenvector `v_long`. A high condition number
`kappa = lambda_max / lambda_min` means the scan is elongated, which is the
typical long-corridor degeneracy direction. The confidence is:

```text
D = sigmoid(slope * (kappa - threshold))
```

The confidence and direction are smoothed over time with a first-order low-pass
filter. Eigenvector sign is aligned with the previous direction before
smoothing to avoid visualization flips.

## Anisotropic Prior

The original Cartographer translation residual was:

```text
r = w * (p - p0)
```

It is now:

```text
r = S * (p - p0)
S = R * diag(w_long, w_lat) * R^T
R = [v_long, v_lat]
```

`v_long` is the degenerate corridor direction and `v_lat` is the orthogonal
non-degenerate direction. The matrix multiplication itself performs the
directional decoupling; there is no separate `D_x` or `D_y` scalar projection.

The occupied-space residual remains Cartographer's original scalar grid cost.
Because that residual is not an explicit 2D translation residual, the relative
laser/odometry balance is changed through the anisotropic motion prior: the
longitudinal prior weight increases when the scan is degenerate, while the
lateral prior can remain near the base value so lateral LiDAR constraints stay
dominant.

## ROS Diagnostics

`cartographer_ros` publishes:

- `/degeneracy_metric` (`std_msgs/Float32`): smoothed confidence in `[0, 1]`.
- `/degeneracy_direction` (`geometry_msgs/Vector3`): principal degeneracy
  direction in the current 2D scan frame.

The latest metric is stored in Cartographer core behind a mutex, then copied by
the ROS timer thread for publishing.

## Configuration

The project configuration is in:

```text
src/my_navigation/config/cartographer_scout_2d.lua
```

Look for:

```text
[Innovation 1: Directional Adaptive Fusion Parameters]
```

If `use_odometry = false`, `cartographer_ros` forces the module off so pure
laser mode keeps the original isotropic behavior.
