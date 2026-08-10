#!/usr/bin/env python3
"""Evaluate M3DGR ArUco endpoint and an explicitly non-official SE(2) view."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def pose_from_row(row):
    stamp = float(row["%time"])
    if stamp > 1.0e12:
        stamp *= 1.0e-9
    position = np.array([
        float(row["field.pose.position.x"]),
        float(row["field.pose.position.y"]),
        float(row["field.pose.position.z"]),
    ])
    quaternion = np.array([
        float(row["field.pose.orientation.x"]),
        float(row["field.pose.orientation.y"]),
        float(row["field.pose.orientation.z"]),
        float(row["field.pose.orientation.w"]),
    ])
    quaternion /= np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    return stamp, transform


def read_trajectory(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        poses = [pose_from_row(row) for row in csv.DictReader(stream)]
    if len(poses) < 2:
        raise ValueError("trajectory contains fewer than two poses")
    last = len(poses) - 1
    for index in range(len(poses) - 2, -1, -1):
        if not np.allclose(poses[index][1], poses[last][1]):
            break
        last = index
    return poses[0], poses[last], len(poses)


def read_reference(path):
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
        raise ValueError("invalid M3DGR ArUco reference")
    reference = np.eye(4)
    reference[:3, :3] = np.asarray(values[:9]).reshape(3, 3)
    reference[:3, 3] = values[9:12]
    return reference, duration


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--ground-truth", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    first, last, sample_count = read_trajectory(args.trajectory)
    reference, reference_duration = read_reference(args.ground_truth)
    estimated = np.linalg.inv(first[1]) @ last[1]
    translation_error = float(np.linalg.norm(
        reference[:3, 3] - estimated[:3, 3]))
    rotation_error = float(np.linalg.norm(
        reference[:3, :3] - estimated[:3, :3]))
    official_rmse = math.sqrt(
        (translation_error ** 2 + rotation_error ** 2) / 2.0)
    duration = last[0] - first[0]

    reference_yaw = math.atan2(reference[1, 0], reference[0, 0])
    estimated_yaw = math.atan2(estimated[1, 0], estimated[0, 0])
    planar_translation_error = float(np.linalg.norm(
        reference[:2, 3] - estimated[:2, 3]))
    planar_yaw_error = abs(normalize_angle(estimated_yaw - reference_yaw))

    result = {
        "trajectory": str(Path(args.trajectory).resolve()),
        "ground_truth": str(Path(args.ground_truth).resolve()),
        "sample_count": sample_count,
        "estimator_duration_sec": duration,
        "ground_truth_duration_sec": reference_duration,
        "tracking_rate_percent": min(100.0, duration / reference_duration * 100.0),
        "official_aruco_endpoint": {
            "translation_error_m": translation_error,
            "rotation_matrix_frobenius_error": rotation_error,
            "combined_rmse": official_rmse,
        },
        "non_official_se2_projection": {
            "translation_error_m": planar_translation_error,
            "yaw_error_rad": planar_yaw_error,
            "reference_yaw_rad": reference_yaw,
            "estimated_yaw_rad": estimated_yaw,
        },
        "limitations": [
            "The ArUco file provides one start-to-end transform, not continuous ground truth.",
            "The SE(2) projection is a diagnostic proxy and is not an official M3DGR metric.",
            "Neither result is continuous ATE or RPE.",
        ],
    }
    Path(args.output).write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
