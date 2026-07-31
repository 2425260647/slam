#!/usr/bin/env python3
"""Compare occupied/free-space preservation for the wall-parameter scan.

The metric is deliberately a structural screening metric, not ground truth:
the same corridor bag is replayed for every case, and only hit/miss odds are
changed. Long occupied runs along the dominant map axis indicate wall
continuity; free/unknown ratios expose the cost of weakening clearing.
"""

import argparse
import json
import math
import os

import numpy as np
from PIL import Image
import yaml


def load_map(prefix):
    image = np.asarray(Image.open(prefix + ".pgm"), dtype=np.uint8)
    with open(prefix + ".yaml", "r", encoding="utf-8") as source:
        metadata = yaml.safe_load(source)
    return image, float(metadata["resolution"])


def close_small_gaps(row, max_gap=4):
    occupied = np.flatnonzero(row)
    if occupied.size < 2:
        return row.copy()
    result = row.copy()
    for left, right in zip(occupied[:-1], occupied[1:]):
        if right - left - 1 <= max_gap:
            result[left:right + 1] = True
    return result


def longest_run(row):
    longest = 0
    current = 0
    for value in row:
        if value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def align_occupied_grid(occupied, resolution):
    """Rotate occupied cells into their PCA frame for orientation-free metrics."""
    rows, columns = np.nonzero(occupied)
    if len(rows) < 2:
        return occupied.copy(), 0.0

    coordinates = np.column_stack((columns, rows)).astype(np.float64)
    coordinates *= resolution
    centered = coordinates - coordinates.mean(axis=0)
    covariance = centered.T @ centered / max(1, len(centered) - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    longitudinal = eigenvectors[:, int(np.argmax(eigenvalues))]
    if longitudinal[0] < 0.0:
        longitudinal = -longitudinal
    lateral = np.array([-longitudinal[1], longitudinal[0]])

    projected_longitudinal = centered @ longitudinal
    projected_lateral = centered @ lateral
    longitudinal_indices = np.rint(
        (projected_longitudinal - projected_longitudinal.min()) /
        resolution).astype(np.int64)
    lateral_indices = np.rint(
        (projected_lateral - projected_lateral.min()) /
        resolution).astype(np.int64)

    aligned = np.zeros((int(lateral_indices.max()) + 1,
                        int(longitudinal_indices.max()) + 1), dtype=bool)
    aligned[lateral_indices, longitudinal_indices] = True
    yaw_degrees = math.degrees(math.atan2(longitudinal[1], longitudinal[0]))
    return aligned, yaw_degrees


def wall_row_metrics(occupied, resolution):
    height, width = occupied.shape
    row_counts = occupied.sum(axis=1)
    # The PCA-aligned corridor axis is horizontal. Candidate wall bands must
    # span at least 10% of that longitudinal axis.
    candidate_rows = np.flatnonzero(row_counts >= max(20, 0.10 * width))
    row_records = []
    for row_index in candidate_rows:
        closed = close_small_gaps(occupied[row_index])
        row_records.append({
            "row": int(row_index),
            "occupied_cells": int(row_counts[row_index]),
            "raw_coverage": float(row_counts[row_index] / width),
            "gap_closed_coverage": float(closed.sum() / width),
            "longest_raw_run_m": float(
                longest_run(occupied[row_index]) * resolution),
            "longest_gap_closed_run_m": float(
                longest_run(closed) * resolution),
        })
    row_records.sort(key=lambda item: item["gap_closed_coverage"],
                     reverse=True)
    return candidate_rows, row_records[:10]


def map_metrics(prefix):
    image, resolution = load_map(prefix)
    occupied = image == 0
    free = image == 254
    unknown = image == 205
    height, width = image.shape
    aligned_occupied, dominant_axis_yaw_deg = align_occupied_grid(
        occupied, resolution)
    candidate_rows, top_rows = wall_row_metrics(aligned_occupied, resolution)
    return {
        "map_prefix": prefix,
        "width_cells": int(width),
        "height_cells": int(height),
        "width_m": float(width * resolution),
        "height_m": float(height * resolution),
        "occupied_cells": int(occupied.sum()),
        "free_cells": int(free.sum()),
        "unknown_cells": int(unknown.sum()),
        "occupied_ratio": float(occupied.mean()),
        "free_ratio": float(free.mean()),
        "unknown_ratio": float(unknown.mean()),
        "dominant_axis_yaw_deg": float(dominant_axis_yaw_deg),
        "aligned_width_cells": int(aligned_occupied.shape[1]),
        "aligned_height_cells": int(aligned_occupied.shape[0]),
        "candidate_wall_rows": int(len(candidate_rows)),
        "top_wall_rows": top_rows,
        "best_wall_gap_closed_coverage": float(
            top_rows[0]["gap_closed_coverage"] if top_rows else 0.0),
        "best_wall_continuous_length_m": float(
            top_rows[0]["longest_gap_closed_run_m"] if top_rows else 0.0),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    results = {}
    for name in sorted(os.listdir(args.root)):
        prefix = os.path.join(args.root, name, "map")
        if os.path.isfile(prefix + ".pgm") and os.path.isfile(prefix + ".yaml"):
            results[name] = map_metrics(prefix)
    with open(args.output, "w", encoding="utf-8") as output:
        json.dump(results, output, ensure_ascii=False, indent=2, sort_keys=True)
    print(json.dumps(results, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
