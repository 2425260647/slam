#!/usr/bin/env python3
"""Checks the Innovation 1 tracking-to-local direction-frame correction."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
from statistics import median


def principal_angle_difference(angle_a, angle_b):
    """Returns the unsigned difference between unoriented 2D axes in radians."""
    return 0.5 * abs(math.atan2(math.sin(2.0 * (angle_a - angle_b)),
                                math.cos(2.0 * (angle_a - angle_b))))


def nearest(samples, timestamps, timestamp, max_delta_ns=100_000_000):
    index = bisect.bisect_left(timestamps, timestamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    if not candidates:
        return None
    candidate = min(candidates, key=lambda item: abs(item[0] - timestamp))
    return candidate if abs(candidate[0] - timestamp) <= max_delta_ns else None


def read_direction(path):
    samples = []
    with Path(path).open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            x = float(row["field.x"])
            y = float(row["field.y"])
            if math.hypot(x, y) <= 1e-12:
                continue
            samples.append((int(row["%time"]), math.atan2(y, x)))
    samples.sort()
    return samples, [sample[0] for sample in samples]


def read_metric(path):
    samples = []
    with Path(path).open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            samples.append((int(row["%time"]), float(row["field.data"])))
    samples.sort()
    return samples, [sample[0] for sample in samples]


def read_pose(path):
    samples = []
    with Path(path).open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            qz = float(row["field.pose.orientation.z"])
            qw = float(row["field.pose.orientation.w"])
            samples.append((
                int(row["%time"]),
                float(row["field.pose.position.x"]),
                float(row["field.pose.position.y"]),
                2.0 * math.atan2(qz, qw),
            ))
    samples.sort()
    return samples, [sample[0] for sample in samples]


def trajectory_pca_axis(poses):
    mean_x = sum(pose[1] for pose in poses) / len(poses)
    mean_y = sum(pose[2] for pose in poses) / len(poses)
    covariance_xx = sum((pose[1] - mean_x) ** 2 for pose in poses)
    covariance_xy = sum((pose[1] - mean_x) * (pose[2] - mean_y)
                        for pose in poses)
    covariance_yy = sum((pose[2] - mean_y) ** 2 for pose in poses)
    return 0.5 * math.atan2(2.0 * covariance_xy,
                            covariance_xx - covariance_yy)


def percentile(values, probability):
    if not values:
        return None
    values = sorted(values)
    position = (len(values) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def read_map_stats(run_dir):
    yaml_path = Path(run_dir) / "map.yaml"
    pgm_path = Path(run_dir) / "map.pgm"
    resolution = None
    with yaml_path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1].strip())
                break
    with pgm_path.open("rb") as stream:
        if stream.readline().strip() != b"P5":
            raise RuntimeError("Only binary PGM maps are supported.")
        dimensions = stream.readline()
        while dimensions.startswith(b"#"):
            dimensions = stream.readline()
        width, height = map(int, dimensions.split())
        if int(stream.readline()) != 255:
            raise RuntimeError("Unsupported PGM value range.")
        image = stream.read()
    return {
        "map_area_m2": width * height * resolution * resolution,
        "occupied_cells": image.count(bytes([0])),
        "free_cells": image.count(bytes([254])),
        "unknown_cells": image.count(bytes([205])),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--old-run", required=True)
    parser.add_argument("--new-run", required=True)
    parser.add_argument("--output-json", required=True)
    args = parser.parse_args()

    old_run = Path(args.old_run)
    new_run = Path(args.new_run)
    new_directions, _ = read_direction(new_run / "degeneracy_direction.csv")
    confidences, confidence_times = read_metric(new_run / "degeneracy_metric.csv")
    poses, pose_times = read_pose(new_run / "tracked_pose.csv")
    trajectory_axis = trajectory_pca_axis(poses)

    map_axis_errors_deg = []
    straight_corridor_axis_errors_deg = []
    compared = 0
    high_confidence = 0
    for timestamp, new_angle in new_directions:
        confidence = nearest(confidences, confidence_times, timestamp)
        pose = nearest(poses, pose_times, timestamp)
        if confidence is None or pose is None:
            continue
        compared += 1
        if confidence[1] < 0.8:
            continue
        high_confidence += 1
        direction_error_deg = math.degrees(
            principal_angle_difference(new_angle, trajectory_axis))
        map_axis_errors_deg.append(direction_error_deg)
        if math.degrees(principal_angle_difference(
                pose[3], trajectory_axis)) <= 15.0:
            straight_corridor_axis_errors_deg.append(direction_error_deg)

    report = {
        "purpose": "Validate the Innovation 1 tracking-to-local direction correction.",
        "old_run": str(old_run),
        "new_run": str(new_run),
        "direction_samples_compared": compared,
        "high_confidence_samples": high_confidence,
        "trajectory_pca_axis_deg": math.degrees(trajectory_axis),
        "map_direction_to_trajectory_axis_error_deg": {
            "median": median(map_axis_errors_deg)
            if map_axis_errors_deg else None,
            "p90": percentile(map_axis_errors_deg, 0.90),
            "max": max(map_axis_errors_deg) if map_axis_errors_deg else None,
        },
        "straight_corridor_high_confidence_samples": len(
            straight_corridor_axis_errors_deg),
        "straight_corridor_map_direction_error_deg": {
            "median": median(straight_corridor_axis_errors_deg)
            if straight_corridor_axis_errors_deg else None,
            "p90": percentile(straight_corridor_axis_errors_deg, 0.90),
            "max": max(straight_corridor_axis_errors_deg)
            if straight_corridor_axis_errors_deg else None,
        },
        "old_map": read_map_stats(old_run),
        "new_map": read_map_stats(new_run),
        "scope_note": (
            "The reference is the online trajectory PCA axis, not a surveyed "
            "physical corridor axis. Map metrics are structural proxies, not "
            "ATE/RPE."
        ),
    }
    Path(args.output_json).write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
