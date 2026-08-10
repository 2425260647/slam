# Corridor02 Phase B: MID-360 Deskew Validation

Date: 2026-08-07

## Method

- Interpolate wheel odometry at every Livox point timestamp.
- Transform every MID-360 point into `base_footprint` at scan end.
- Apply the official M3DGR `base_from_lidar` calibration before height filtering.
- Project the deskewed cloud into 1441 angular bins and remove unsupported
  isolated returns.
- Freeze all Phase A V4 lightweight SLAM parameters.
- Feed the identical `/scan + /odom` bag to the lightweight system and the
  clean Cartographer baseline.

## Input Audit

- Source scans: 2934
- Output scans: 2934
- Odometry messages: 5867
- Scan duration range: 99.201--101.167 ms
- Frame: `base_footprint`
- Non-monotonic scan timestamps: 0
- Isolated projected returns after filtering: 0
- Shared input SHA-256:
  `99bdcdfb264af38569874b20a2da2c986992e6a39f2f5de542a0633f69259251`

## Full-Sequence Results

| Metric | Phase A V4 | Phase B lightweight | Phase B Cartographer |
|---|---:|---:|---:|
| Input scans | 2934 | 2934 | 2934 |
| Dropped scans | 0 | 0 | not exposed |
| Tracking / fallback | 2927 / 7 | 2923 / 11 | not comparable |
| Processing mean / P95 / max (ms) | 5.746 / 9.362 / 13.948 | 5.736 / 9.745 / 18.957 | not exposed |
| Maximum online pose step (m) | 0.116 | 0.130 | 3.544 |
| Start/end closure proxy (m) | 0.311 | 0.451 | 0.314 |
| ArUco endpoint translation proxy (m) | 0.497 | 0.395 | 0.383 |
| Occupied cells | 16372 | 17791 | 19491 |
| Occupied components | 494 | 431 | 475 |
| Single-pixel components | 138 | 120 | 143 |
| Components <=3 pixels | 272 | 239 | 258 |
| Longest Hough line (m) | 24.250 | 48.540 | 49.753 |
| Segment heading P95 median (deg) | 1.884 | 1.596 | 1.686 |
| Segment heading P95 worst (deg) | 12.330 | 3.596 | 4.639 |
| Wall residual P95 median (m) | 0.071 | 0.075 | 0.080 |
| Wall residual P95 worst (m) | 0.287 | 0.301 | 0.227 |

## Decision

Phase B is accepted as an engineering improvement over Phase A. It materially
reduces the worst wall-heading deviation, roughly doubles the longest detected
continuous line, further reduces small occupied components, improves the
endpoint translation proxy, and keeps the 10 Hz real-time budget.

The lightweight map is close to the same-input Cartographer map on endpoint,
occupied-cell count, median/worst heading and longest-line proxies. It is not
uniformly better: the start/end closure proxy is larger and the worst wall
residual remains higher. The latter is the next map-quality target.

## Evidence Boundary

`GTCorridor02.txt` contains one start-to-end ArUco transform, not continuous
trajectory truth. Closure, endpoint and wall metrics are engineering proxies;
none is ATE or RPE. General trajectory-accuracy claims require frozen
parameters and an independent sequence with continuous external ground truth.
