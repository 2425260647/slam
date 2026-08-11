#!/usr/bin/env python3
"""Evaluate the official Corridor02 end-to-end ArUco reference protocol.

The file GTCorridor02.txt contains only one surveyed start-to-end relative
transform and a sequence duration.  This is deliberately not called ATE/RPE:
the evaluator reports the official endpoint translation error, rotation-matrix
Frobenius error, their combined RMSE, and the trajectory tracking rate.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def _pose_row(row):
    stamp = float(row["%time"])
    if stamp > 1.0e12:
        stamp *= 1.0e-9
    elif stamp > 1.0e9:
        stamp *= 1.0e-9
    p = np.array([float(row["field.pose.position.x"]),
                  float(row["field.pose.position.y"]),
                  float(row["field.pose.position.z"])])
    qx = float(row["field.pose.orientation.x"])
    qy = float(row["field.pose.orientation.y"])
    qz = float(row["field.pose.orientation.z"])
    qw = float(row["field.pose.orientation.w"])
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    r = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
         2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)],
    ])
    return stamp, p, r


def read_estimator(path):
    with Path(path).open(newline="", encoding="utf-8") as source:
        rows = [_pose_row(row) for row in csv.DictReader(source)]
    if len(rows) < 2:
        raise ValueError("estimator trajectory has fewer than two poses")
    return rows


def read_gt(path):
    values = []
    duration = None
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("bag_time:"):
            duration = float(line.split(":", 1)[1].strip().rstrip("s"))
        else:
            values.extend(float(token) for token in line.split())
    if len(values) != 12 or duration is None:
        raise ValueError("expected 12 transform values and bag_time")
    return np.asarray(values[:9], dtype=float).reshape(3, 3), \
        np.asarray(values[9:12], dtype=float), duration


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    trajectory = read_estimator(args.trajectory)
    r_gt, t_gt, gt_duration = read_gt(args.gt)
    t0, p0, r0 = trajectory[0]
    t1, p1, r1 = trajectory[-1]
    r_est = r0.T @ r1
    t_est = r0.T @ (p1 - p0)
    translation_error = float(np.linalg.norm(t_est - t_gt))
    rotation_error = float(np.linalg.norm(r_est - r_gt, ord="fro"))
    combined_rmse = math.sqrt((translation_error ** 2 + rotation_error ** 2) / 2.0)
    tracking_rate = (t1 - t0) / gt_duration * 100.0
    result = {
        "protocol": "M3DGR official Corridor02 endpoint ArUco protocol",
        "trajectory": str(args.trajectory),
        "ground_truth": str(args.gt),
        "sample_count": len(trajectory),
        "estimator_duration_sec": t1 - t0,
        "ground_truth_duration_sec": gt_duration,
        "tracking_rate_percent": tracking_rate,
        "estimated_relative_rotation": r_est.tolist(),
        "estimated_relative_translation": t_est.tolist(),
        "reference_rotation": r_gt.tolist(),
        "reference_translation": t_gt.tolist(),
        "translation_error_m": translation_error,
        "rotation_matrix_frobenius_error": rotation_error,
        "combined_rmse": combined_rmse,
        "limitations": [
            "This is an endpoint ArUco reference, not continuous ground truth.",
            "The reported values must not be labeled continuous ATE or RPE.",
        ],
    }
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: result[k] for k in (
        "sample_count", "estimator_duration_sec", "tracking_rate_percent",
        "translation_error_m", "rotation_matrix_frobenius_error",
        "combined_rmse")}, indent=2))


if __name__ == "__main__":
    main()
