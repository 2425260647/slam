#!/usr/bin/env python3
"""Orientation-free structural comparison for M3DGR Corridor02 maps.

This is a map-structure proxy, not an ATE/RPE or surveyed-map evaluation.
It sweeps Hough parameters so the conclusion does not depend on one detector
setting, then measures how tightly detected wall segments follow one pair of
orthogonal axes.
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
import yaml


def load_map(prefix):
    prefix = Path(prefix)
    image = np.asarray(Image.open(str(prefix) + ".pgm"), dtype=np.uint8)
    with open(str(prefix) + ".yaml", encoding="utf-8") as source:
        metadata = yaml.safe_load(source)
    return image, float(metadata["resolution"])


def pixel_metrics(image):
    total = image.size
    return {
        "width_cells": int(image.shape[1]),
        "height_cells": int(image.shape[0]),
        "occupied_cells": int(np.count_nonzero(image == 0)),
        "free_cells": int(np.count_nonzero(image == 254)),
        "unknown_cells": int(np.count_nonzero(image == 205)),
        "occupied_ratio": float(np.count_nonzero(image == 0) / total),
        "free_ratio": float(np.count_nonzero(image == 254) / total),
    }


def hough_metrics(occupied, resolution, threshold, min_length_px, max_gap_px):
    cv2.setRNGSeed(0)
    lines = cv2.HoughLinesP(
        occupied.astype(np.uint8) * 255,
        1,
        np.pi / 360.,
        threshold=threshold,
        minLineLength=min_length_px,
        maxLineGap=max_gap_px,
    )
    if lines is None or len(lines) == 0:
        return None
    lines = lines[:, 0]
    dx = lines[:, 2] - lines[:, 0]
    dy = lines[:, 3] - lines[:, 1]
    lengths_px = np.hypot(dx, dy)
    angles = np.arctan2(dy, dx)

    # exp(4j*theta) treats parallel and orthogonal wall directions as equal.
    resultant = np.sum(lengths_px * np.exp(4j * angles)) / np.sum(lengths_px)
    dominant_axis = np.angle(resultant) / 4.
    residuals = np.abs(
        np.angle(np.exp(4j * (angles - dominant_axis)))) / 4.
    angular_rmse = math.sqrt(
        float(np.sum(lengths_px * residuals ** 2) / np.sum(lengths_px)))

    return {
        "line_count": int(len(lines)),
        "total_line_length_m": float(np.sum(lengths_px) * resolution),
        "direction_concentration": float(abs(resultant)),
        "dominant_axis_mod_90_deg": float(
            math.degrees(dominant_axis) % 90.),
        "orthogonal_axis_angular_rmse_deg": float(math.degrees(angular_rmse)),
    }


def summarize(values):
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
    }


def comparison_summary(records, minimum_length_m):
    selected = [
        record for record in records
        if record["min_line_length_m"] >= minimum_length_m
    ]
    concentration_delta = [
        record["proposed"]["direction_concentration"] -
        record["baseline"]["direction_concentration"]
        for record in selected
    ]
    angular_rmse_reduction = [
        record["baseline"]["orthogonal_axis_angular_rmse_deg"] -
        record["proposed"]["orthogonal_axis_angular_rmse_deg"]
        for record in selected
    ]
    return {
        "configuration_count": len(selected),
        "minimum_line_length_m": minimum_length_m,
        "direction_concentration_delta": summarize(concentration_delta),
        "direction_concentration_win_ratio": float(
            np.mean(np.asarray(concentration_delta) > 0.)),
        "angular_rmse_reduction_deg": summarize(angular_rmse_reduction),
        "angular_rmse_win_ratio": float(
            np.mean(np.asarray(angular_rmse_reduction) > 0.)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-prefix", required=True)
    parser.add_argument("--proposed-prefix", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    baseline_image, baseline_resolution = load_map(args.baseline_prefix)
    proposed_image, proposed_resolution = load_map(args.proposed_prefix)
    if abs(baseline_resolution - proposed_resolution) > 1e-9:
        raise RuntimeError("Map resolutions differ")

    images = {
        "baseline": baseline_image,
        "proposed": proposed_image,
    }
    records = []
    for threshold in (15, 20, 25, 30, 40, 50):
        for min_length_px in (20, 40, 60, 80, 120):
            for max_gap_px in (3, 6, 10, 15):
                record = {
                    "threshold": threshold,
                    "min_line_length_px": min_length_px,
                    "min_line_length_m": min_length_px * baseline_resolution,
                    "max_gap_px": max_gap_px,
                }
                for name, image in images.items():
                    record[name] = hough_metrics(
                        image == 0, baseline_resolution, threshold,
                        min_length_px, max_gap_px)
                if record["baseline"] is not None and record["proposed"] is not None:
                    records.append(record)

    result = {
        "dataset": "M3DGR Corridor02",
        "map_resolution_m": baseline_resolution,
        "map_pixels": {
            name: pixel_metrics(image) for name, image in images.items()
        },
        "all_detected_lines": comparison_summary(records, 0.),
        "long_walls": comparison_summary(records, 6.),
        "parameter_sweep": records,
        "limitations": [
            "Corridor02 has no continuous reference pose in the bag.",
            "These are orientation-free map-structure proxies, not ATE/RPE.",
            "No ArUco pose is reported because the bag has no camera_info, marker size, or surveyed marker layout.",
        ],
    }
    output = Path(args.output)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=True) + "\n")
    print(json.dumps({
        "map_pixels": result["map_pixels"],
        "all_detected_lines": result["all_detected_lines"],
        "long_walls": result["long_walls"],
        "limitations": result["limitations"],
    }, indent=2, ensure_ascii=True))


if __name__ == "__main__":
    main()
