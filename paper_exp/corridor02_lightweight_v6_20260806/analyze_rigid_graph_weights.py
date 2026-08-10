#!/usr/bin/env python3

"""Replay the Corridor02 rigid-submap graph without replaying the ROS bag.

The script reconstructs insertion constraints from the final local path and
the configured 50/25 overlapping-submap topology. Accepted loop measurements
are recovered from the logged correction and the corresponding local node
pose. It is an experiment diagnostic, not an accuracy evaluator: Corridor02
does not provide continuous ground truth.
"""

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
import rosbag
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix


LOOP_PATTERN = re.compile(
    r"Accepted confirmed loop closure \d+ -> (\d+).*?submaps=(\d+)->\d+"
    r".*?correction=([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)"
)
PROPOSAL_PATTERN = re.compile(
    r"Loop proposal candidate=\d+ current=(\d+) submaps=(\d+)->\d+"
    r".*?correction=([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)"
)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(lhs, rhs):
    cosine = math.cos(lhs[2])
    sine = math.sin(lhs[2])
    return np.array(
        [
            lhs[0] + cosine * rhs[0] - sine * rhs[1],
            lhs[1] + sine * rhs[0] + cosine * rhs[1],
            normalize_angle(lhs[2] + rhs[2]),
        ]
    )


def between(lhs, rhs):
    dx = rhs[0] - lhs[0]
    dy = rhs[1] - lhs[1]
    cosine = math.cos(lhs[2])
    sine = math.sin(lhs[2])
    return np.array(
        [
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            normalize_angle(rhs[2] - lhs[2]),
        ]
    )


def read_last_path(bag_path, topic):
    final_message = None
    with rosbag.Bag(str(bag_path)) as bag:
        for _, message, _ in bag.read_messages(topics=[topic]):
            final_message = message
    if final_message is None:
        raise RuntimeError(f"No {topic} message in {bag_path}")
    poses = []
    for stamped in final_message.poses:
        orientation = stamped.pose.orientation
        poses.append(
            [
                stamped.pose.position.x,
                stamped.pose.position.y,
                2.0 * math.atan2(orientation.z, orientation.w),
            ]
        )
    return np.asarray(poses, dtype=float)


def parse_loops(log_path, local_nodes, anchors, include_proposals):
    loops = []
    log_text = log_path.read_text(errors="replace")
    pattern = PROPOSAL_PATTERN if include_proposals else LOOP_PATTERN
    for match in pattern.finditer(log_text):
        node = int(match.group(1))
        submap = int(match.group(2))
        correction = np.asarray(
            [float(match.group(3)), float(match.group(4)), float(match.group(5))]
        )
        matched_local_pose = compose(correction, local_nodes[node])
        measurement = between(anchors[submap], matched_local_pose)
        loops.append((submap, node, measurement))
    if not loops:
        raise RuntimeError(f"No accepted loops found in {log_path}")
    return loops


def huber_block(residual, scale=1.0):
    squared_norm = float(np.dot(residual, residual))
    scale_squared = scale * scale
    if squared_norm <= scale_squared or squared_norm <= 1e-18:
        return residual
    rho = 2.0 * scale * math.sqrt(squared_norm) - scale_squared
    return residual * math.sqrt(rho / squared_norm)


def relative_residual(reference, node, measurement, translation_weight,
                      rotation_weight):
    predicted = between(reference, node)
    return np.asarray(
        [
            translation_weight * (predicted[0] - measurement[0]),
            translation_weight * (predicted[1] - measurement[1]),
            rotation_weight
            * math.sin(normalize_angle(predicted[2] - measurement[2])),
        ]
    )


def fit_line_p95(points):
    centered = points - np.mean(points, axis=0)
    _, _, vectors = np.linalg.svd(centered, full_matrices=False)
    normal = vectors[-1]
    residuals = np.abs(centered @ normal)
    return float(np.percentile(residuals, 95.0))


def solve_graph(local_nodes, submap_size, overlap, loops,
                insertion_translation_weight, insertion_rotation_weight,
                loop_translation_weight, loop_rotation_weight,
                maximum_evaluations):
    stride = submap_size - overlap
    anchor_indices = list(range(0, len(local_nodes), stride))
    anchors = local_nodes[anchor_indices].copy()
    node_count = len(local_nodes)
    submap_count = len(anchors)
    insertion_constraints = []
    for submap, first in enumerate(anchor_indices):
        for node in range(first, min(node_count, first + submap_size)):
            insertion_constraints.append(
                (submap, node, between(anchors[submap], local_nodes[node]))
            )

    # Node poses are followed by submaps 1..N. Submap zero is the fixed gauge.
    initial = np.concatenate(
        [local_nodes.reshape(-1), anchors[1:].reshape(-1)]
    )

    def node_pose(values, index):
        return values[3 * index : 3 * index + 3]

    def submap_pose(values, index):
        if index == 0:
            return anchors[0]
        offset = 3 * node_count + 3 * (index - 1)
        return values[offset : offset + 3]

    def residuals(values):
        output = []
        for submap, node, measurement in insertion_constraints:
            output.extend(
                relative_residual(
                    submap_pose(values, submap), node_pose(values, node),
                    measurement, insertion_translation_weight,
                    insertion_rotation_weight,
                )
            )
        for submap, node, measurement in loops:
            loop_residual = relative_residual(
                submap_pose(values, submap), node_pose(values, node),
                measurement, loop_translation_weight, loop_rotation_weight,
            )
            output.extend(huber_block(loop_residual))
        return np.asarray(output)

    row_count = 3 * (len(insertion_constraints) + len(loops))
    sparsity = lil_matrix((row_count, len(initial)), dtype=int)
    row = 0
    for submap, node, _ in insertion_constraints + loops:
        sparsity[row : row + 3, 3 * node : 3 * node + 3] = 1
        if submap != 0:
            offset = 3 * node_count + 3 * (submap - 1)
            sparsity[row : row + 3, offset : offset + 3] = 1
        row += 3

    solved = least_squares(
        residuals,
        initial,
        jac_sparsity=sparsity.tocsr(),
        max_nfev=maximum_evaluations,
        ftol=1e-8,
        xtol=1e-8,
        gtol=1e-8,
        verbose=0,
    )
    optimized_submaps = np.vstack(
        [submap_pose(solved.x, index) for index in range(submap_count)]
    )
    rendered_nodes = []
    for node in range(node_count):
        primary = min(node // stride, submap_count - 1)
        rendered_nodes.append(
            compose(
                optimized_submaps[primary],
                between(anchors[primary], local_nodes[node]),
            )
        )
    rendered_nodes = np.asarray(rendered_nodes)

    # These ranges are determined from turns in the recorded local trajectory,
    # not from ground truth. They measure how much the optimizer bends locally
    # straight runs relative to the input trajectory.
    straight_submap_ranges = [(3, 8), (10, 19), (21, 29), (31, 39), (41, 44)]
    p95_values = []
    for first, last in straight_submap_ranges:
        p95_values.append(
            fit_line_p95(optimized_submaps[first : last + 1, :2])
        )
    return {
        "success": bool(solved.success),
        "cost": float(solved.cost),
        "function_evaluations": int(solved.nfev),
        "endpoint_translation_m": float(
            np.linalg.norm(rendered_nodes[-1, :2] - rendered_nodes[0, :2])
        ),
        "endpoint_yaw_rad": float(
            abs(normalize_angle(rendered_nodes[-1, 2] - rendered_nodes[0, 2]))
        ),
        "straight_run_position_p95_median_m": float(np.median(p95_values)),
        "straight_run_position_p95_worst_m": float(np.max(p95_values)),
        "submap_yaw_correction_p95_deg": float(
            np.degrees(
                np.percentile(
                    np.abs(
                        [
                            normalize_angle(optimized_submaps[i, 2] - anchors[i, 2])
                            for i in range(submap_count)
                        ]
                    ),
                    95.0,
                )
            )
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--submap-keyframes", type=int, default=50)
    parser.add_argument("--overlap-keyframes", type=int, default=25)
    parser.add_argument(
        "--rotation-weights", type=float, nargs="+", default=[30, 60, 120, 240]
    )
    parser.add_argument(
        "--include-proposals",
        action="store_true",
        help="Use every geometrically validated loop proposal, not only accepted loops.",
    )
    parser.add_argument("--loop-translation-weight", type=float, default=25.0)
    parser.add_argument("--loop-rotation-weight", type=float, default=35.0)
    parser.add_argument("--maximum-evaluations", type=int, default=80)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    local_nodes = read_last_path(
        args.run_directory / "outputs.bag", "/lightweight_slam/local_path"
    )
    stride = args.submap_keyframes - args.overlap_keyframes
    anchors = local_nodes[::stride]
    loops = parse_loops(
        args.run_directory / "roslaunch.log", local_nodes, anchors,
        args.include_proposals,
    )
    results = {}
    for rotation_weight in args.rotation_weights:
        key = f"insertion_rotation_{rotation_weight:g}"
        results[key] = solve_graph(
            local_nodes,
            args.submap_keyframes,
            args.overlap_keyframes,
            loops,
            insertion_translation_weight=20.0,
            insertion_rotation_weight=rotation_weight,
            loop_translation_weight=args.loop_translation_weight,
            loop_rotation_weight=args.loop_rotation_weight,
            maximum_evaluations=args.maximum_evaluations,
        )
    document = {
        "metric_boundary": (
            "Graph-shape sensitivity diagnostic reconstructed from one run; "
            "not ground-truth accuracy and not a replacement for bag replay."
        ),
        "nodes": len(local_nodes),
        "submaps": len(anchors),
        "loops": len(loops),
        "results": results,
    }
    rendered = json.dumps(document, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n")


if __name__ == "__main__":
    main()
