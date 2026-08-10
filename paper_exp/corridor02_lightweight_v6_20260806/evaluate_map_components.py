#!/usr/bin/env python3
"""Audit occupied-cell connected components with one fixed threshold.

These are map-rendering proxies, not trajectory accuracy or surveyed truth.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def evaluate(path):
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError("cannot read map: {}".format(path))
    occupied = np.uint8(image == 0)
    count, _, statistics, _ = cv2.connectedComponentsWithStats(
        occupied, connectivity=8)
    sizes = statistics[1:, cv2.CC_STAT_AREA] if count > 1 else np.array([])
    return {
        "map": str(path),
        "metric_boundary": (
            "Occupied-pixel connectivity proxy; not ATE/RPE or surveyed "
            "wall accuracy."),
        "occupied_cells": int(occupied.sum()),
        "components": int(len(sizes)),
        "single_pixel_components": int(np.sum(sizes == 1)),
        "components_le_3_pixels": int(np.sum(sizes <= 3)),
        "largest_component_pixels": int(sizes.max()) if sizes.size else 0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", action="append", required=True,
                        metavar="LABEL=PGM")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    results = {}
    for specification in args.map:
        if "=" not in specification:
            raise ValueError("--map must be LABEL=PGM")
        label, path = specification.split("=", 1)
        results[label] = evaluate(Path(path))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(results, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(results, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
