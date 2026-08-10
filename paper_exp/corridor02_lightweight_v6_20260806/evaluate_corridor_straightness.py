#!/usr/bin/env python3
"""Deterministic structural audit for long walls in occupancy-grid maps.

The metrics compare map geometry under one frozen extraction configuration.
They are engineering proxies, not trajectory ATE/RPE or surveyed wall truth.
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def angle_difference(lhs, rhs):
    return abs(math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs)))


def line_axis_difference(angle, axis):
    difference = angle_difference(angle, axis)
    return min(difference, abs(math.pi - difference))


def load_map(prefix):
    image = cv2.imread(str(prefix) + ".pgm", cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError("cannot read map: {}.pgm".format(prefix))
    with open(str(prefix) + ".yaml", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    return image, float(metadata["resolution"])


def robust_fit(longitudinal, lateral):
    keep = np.ones(longitudinal.shape, dtype=bool)
    coefficients = np.array([0.0, float(np.median(lateral))])
    for _ in range(4):
        if keep.sum() < 8:
            break
        coefficients = np.polyfit(longitudinal[keep], lateral[keep], 1)
        residuals = lateral - np.polyval(coefficients, longitudinal)
        center = float(np.median(residuals[keep]))
        mad = float(np.median(np.abs(residuals[keep] - center)))
        threshold = max(0.08, 3.0 * 1.4826 * mad)
        keep = np.abs(residuals - center) <= threshold
    residuals = np.abs(lateral - np.polyval(coefficients, longitudinal))
    return coefficients, keep, residuals


def merge_line_groups(lines, axis, offset_tolerance=0.75,
                      interval_gap=2.0):
    direction = np.array([math.cos(axis), math.sin(axis)])
    normal = np.array([-direction[1], direction[0]])
    records = []
    for x1, y1, x2, y2, length, angle in lines:
        if line_axis_difference(angle, axis) > math.radians(15.0):
            continue
        first = np.array([x1, y1])
        second = np.array([x2, y2])
        midpoint = 0.5 * (first + second)
        interval = sorted((float(first @ direction),
                           float(second @ direction)))
        records.append({
            "offset": float(midpoint @ normal),
            "start": interval[0],
            "end": interval[1],
            "length": length,
        })
    records.sort(key=lambda record: (record["offset"], record["start"]))
    groups = []
    for record in records:
        selected = None
        for group in groups:
            offset_close = abs(record["offset"] - group["offset"]) <= (
                offset_tolerance)
            interval_close = not (
                record["start"] > group["end"] + interval_gap or
                record["end"] < group["start"] - interval_gap)
            if offset_close and interval_close:
                selected = group
                break
        if selected is None:
            groups.append({
                "axis": axis,
                "offset": record["offset"],
                "start": record["start"],
                "end": record["end"],
                "weight": record["length"],
            })
            continue
        total_weight = selected["weight"] + record["length"]
        selected["offset"] = (
            selected["offset"] * selected["weight"] +
            record["offset"] * record["length"]) / total_weight
        selected["weight"] = total_weight
        selected["start"] = min(selected["start"], record["start"])
        selected["end"] = max(selected["end"], record["end"])
    return groups


def evaluate_wall_group(group, occupied_points, band=0.30,
                        window_length=2.0):
    axis = group["axis"]
    direction = np.array([math.cos(axis), math.sin(axis)])
    normal = np.array([-direction[1], direction[0]])
    longitudinal = occupied_points @ direction
    lateral = occupied_points @ normal
    selected = ((longitudinal >= group["start"] - 0.25) &
                (longitudinal <= group["end"] + 0.25) &
                (np.abs(lateral - group["offset"]) <= band))
    if selected.sum() < 30:
        return None
    x = longitudinal[selected]
    y = lateral[selected]
    coefficients, inliers, residuals = robust_fit(x, y)
    if inliers.sum() < 20:
        return None
    global_heading = axis + math.atan(float(coefficients[0]))
    headings = []
    lower = float(x[inliers].min())
    upper = float(x[inliers].max())
    start = lower
    while start < upper:
        window = inliers & (x >= start) & (x < start + window_length)
        if window.sum() >= 10 and np.ptp(x[window]) >= 0.5:
            local_coefficients, _, _ = robust_fit(x[window], y[window])
            local_heading = axis + math.atan(float(local_coefficients[0]))
            headings.append(math.degrees(
                line_axis_difference(local_heading, global_heading)))
        start += 0.5 * window_length
    return {
        "axis": int(group.get("axis_index", 0)),
        "length_m": upper - lower,
        "point_count": int(selected.sum()),
        "inlier_count": int(inliers.sum()),
        "p95_residual_m": float(np.percentile(residuals[inliers], 95)),
        "heading_p95_deg": float(np.percentile(headings, 95))
            if headings else 0.0,
        "fit_slope": float(coefficients[0]),
        "fit_intercept": float(coefficients[1]),
        "longitudinal_min": lower,
        "longitudinal_max": upper,
    }


def spacing_metrics(walls):
    deviations = []
    records = []
    for index, first in enumerate(walls):
        for second in walls[index + 1:]:
            if first["axis"] != second["axis"]:
                continue
            lower = max(first["longitudinal_min"],
                        second["longitudinal_min"])
            upper = min(first["longitudinal_max"],
                        second["longitudinal_max"])
            if upper - lower < 5.0:
                continue
            samples = np.linspace(lower, upper,
                                  max(6, int(upper - lower) + 1))
            first_y = first["fit_slope"] * samples + first["fit_intercept"]
            second_y = (second["fit_slope"] * samples +
                        second["fit_intercept"])
            spacing = np.abs(first_y - second_y)
            mean_spacing = float(np.mean(spacing))
            if mean_spacing < 1.0 or mean_spacing > 12.0:
                continue
            deviation = float(np.std(spacing))
            deviations.append(deviation)
            records.append({
                "overlap_m": upper - lower,
                "mean_spacing_m": mean_spacing,
                "spacing_std_m": deviation,
            })
    return deviations, records


def evaluate(prefix):
    image, resolution = load_map(prefix)
    occupied = np.uint8(image == 0) * 255
    hough = cv2.HoughLinesP(
        occupied, 1.0, np.pi / 720.0, threshold=35,
        minLineLength=max(20, int(round(4.0 / resolution))),
        maxLineGap=max(2, int(round(0.60 / resolution))))
    lines = []
    if hough is not None:
        for x1, y1, x2, y2 in hough[:, 0]:
            dx = (x2 - x1) * resolution
            dy = (y2 - y1) * resolution
            length = math.hypot(dx, dy)
            lines.append((x1 * resolution, y1 * resolution,
                          x2 * resolution, y2 * resolution,
                          length, math.atan2(dy, dx)))
    if not lines:
        return {"map_prefix": str(prefix), "detected_walls": 0}

    weights = np.array([line[4] for line in lines])
    angles = np.array([line[5] for line in lines])
    fourth_sine = float(np.sum(weights * np.sin(4.0 * angles)))
    fourth_cosine = float(np.sum(weights * np.cos(4.0 * angles)))
    primary_axis = 0.25 * math.atan2(fourth_sine, fourth_cosine)
    primary_axis %= 0.5 * math.pi
    axes = [primary_axis, primary_axis + 0.5 * math.pi]

    rows, columns = np.nonzero(image == 0)
    occupied_points = np.column_stack((columns, rows)).astype(float)
    occupied_points *= resolution
    groups = []
    for axis_index, axis in enumerate(axes):
        axis_groups = merge_line_groups(lines, axis)
        for group in axis_groups:
            group["axis_index"] = axis_index
        groups.extend(axis_groups)

    walls = []
    for group in groups:
        wall = evaluate_wall_group(group, occupied_points)
        if wall is not None and wall["length_m"] >= 5.0:
            walls.append(wall)
    walls.sort(key=lambda wall: wall["length_m"], reverse=True)
    walls = walls[:12]
    residuals = [wall["p95_residual_m"] for wall in walls]
    headings = [wall["heading_p95_deg"] for wall in walls]
    spacing_deviations, spacing_records = spacing_metrics(walls)
    return {
        "map_prefix": str(prefix),
        "metric_boundary": (
            "Structural map proxy with fixed extraction parameters; not "
            "surveyed wall truth and not ATE/RPE."),
        "dominant_axis_deg": math.degrees(primary_axis),
        "hough_long_lines": len(lines),
        "longest_hough_line_m": max(line[4] for line in lines),
        "detected_walls": len(walls),
        "wall_p95_residual_median_m": float(np.median(residuals))
            if residuals else None,
        "wall_p95_residual_worst_m": max(residuals) if residuals else None,
        "segment_heading_p95_median_deg": float(np.median(headings))
            if headings else None,
        "segment_heading_p95_worst_deg": max(headings) if headings else None,
        "parallel_spacing_std_median_m": float(np.median(spacing_deviations))
            if spacing_deviations else None,
        "parallel_spacing_std_worst_m": max(spacing_deviations)
            if spacing_deviations else None,
        "walls": walls,
        "parallel_pairs": spacing_records,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--map", action="append", required=True, metavar="LABEL=PREFIX")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    results = {}
    for specification in args.map:
        if "=" not in specification:
            raise ValueError("--map must be LABEL=PREFIX")
        label, prefix = specification.split("=", 1)
        results[label] = evaluate(Path(prefix))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(results, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(results, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
