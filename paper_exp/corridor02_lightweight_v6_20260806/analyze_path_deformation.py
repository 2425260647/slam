#!/usr/bin/env python3
"""Compare final local and optimized keyframe paths without trajectory truth."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag


def yaw_from_quaternion(quaternion):
    return 2.0 * math.atan2(quaternion.z, quaternion.w)


def last_paths(bag_path, topics):
    result = {}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(topics=topics):
            result[topic] = np.asarray([
                (pose.pose.position.x, pose.pose.position.y,
                 yaw_from_quaternion(pose.pose.orientation))
                for pose in message.poses], dtype=float)
    return result


def rigid_align(source, target):
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = ((source - source_center).T @
                  (target - target_center))
    left, _, right = np.linalg.svd(covariance)
    rotation = right.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right[-1, :] *= -1.0
        rotation = right.T @ left.T
    translation = target_center - rotation @ source_center
    return (rotation @ source.T).T + translation


def point_line_distance(points, first, last):
    delta = last - first
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        return np.linalg.norm(points - first, axis=1)
    normal = np.array([-delta[1], delta[0]]) / length
    return np.abs((points - first) @ normal)


def rdp_indices(points, epsilon):
    def recurse(start, end):
        if end <= start + 1:
            return [start, end]
        distances = point_line_distance(
            points[start + 1:end], points[start], points[end])
        if distances.size == 0 or float(distances.max()) <= epsilon:
            return [start, end]
        split = start + 1 + int(np.argmax(distances))
        return recurse(start, split)[:-1] + recurse(split, end)
    return recurse(0, len(points) - 1)


def fit_segment(points):
    center = points.mean(axis=0)
    covariance = np.cov((points - center).T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    direction = eigenvectors[:, int(np.argmax(eigenvalues))]
    normal = np.array([-direction[1], direction[0]])
    residuals = np.abs((points - center) @ normal)
    headings = []
    window = min(7, max(2, len(points) // 8))
    global_heading = math.atan2(direction[1], direction[0])
    for index in range(window, len(points) - window):
        delta = points[index + window] - points[index - window]
        if np.linalg.norm(delta) < 0.25:
            continue
        heading = math.atan2(delta[1], delta[0])
        difference = abs(math.atan2(
            math.sin(heading - global_heading),
            math.cos(heading - global_heading)))
        difference = min(difference, abs(math.pi - difference))
        headings.append(math.degrees(difference))
    return {
        "point_count": len(points),
        "length_m": float(np.linalg.norm(points[-1] - points[0])),
        "p95_residual_m": float(np.percentile(residuals, 95)),
        "heading_p95_deg": float(np.percentile(headings, 95))
            if headings else 0.0,
    }


def segment_metrics(points, boundaries):
    result = []
    for start, end in zip(boundaries[:-1], boundaries[1:]):
        segment = points[start:end + 1]
        if len(segment) < 20 or np.linalg.norm(segment[-1] - segment[0]) < 8.0:
            continue
        metrics = fit_segment(segment)
        metrics["start_index"] = int(start)
        metrics["end_index"] = int(end)
        result.append(metrics)
    return result


def aggregate_segments(segments):
    if not segments:
        return {"segments": 0}
    return {
        "segments": len(segments),
        "residual_p95_median_m": float(np.median(
            [segment["p95_residual_m"] for segment in segments])),
        "residual_p95_worst_m": float(max(
            segment["p95_residual_m"] for segment in segments)),
        "heading_p95_median_deg": float(np.median(
            [segment["heading_p95_deg"] for segment in segments])),
        "heading_p95_worst_deg": float(max(
            segment["heading_p95_deg"] for segment in segments)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--rdp-epsilon", type=float, default=3.0)
    args = parser.parse_args()
    topics = ["/lightweight_slam/local_path",
              "/lightweight_slam/path",
              "/lightweight_slam/submap_path"]
    paths = last_paths(Path(args.bag), topics)
    local = paths[topics[0]]
    optimized = paths[topics[1]]
    submaps = paths[topics[2]]
    if len(local) != len(optimized) or len(local) < 2:
        raise RuntimeError("local and optimized path sizes do not match")
    aligned_optimized = rigid_align(optimized[:, :2], local[:, :2])
    deformation = np.linalg.norm(aligned_optimized - local[:, :2], axis=1)
    boundaries = rdp_indices(local[:, :2], args.rdp_epsilon)
    local_segments = segment_metrics(local[:, :2], boundaries)
    optimized_segments = segment_metrics(optimized[:, :2], boundaries)
    result = {
        "metric_boundary": (
            "Path self-consistency diagnostic without continuous truth; "
            "not ATE/RPE."),
        "keyframes": len(local),
        "submap_anchors": len(submaps),
        "rdp_epsilon_m": args.rdp_epsilon,
        "rdp_boundaries": boundaries,
        "nonrigid_deformation_after_se2_alignment": {
            "median_m": float(np.median(deformation)),
            "p95_m": float(np.percentile(deformation, 95)),
            "maximum_m": float(deformation.max()),
        },
        "local": aggregate_segments(local_segments),
        "optimized": aggregate_segments(optimized_segments),
        "local_segments": local_segments,
        "optimized_segments": optimized_segments,
    }
    output = Path(args.output)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
