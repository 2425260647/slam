#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""Compute occupancy-grid quality metrics for the paper experiments."""

from __future__ import print_function

import argparse
import json
import os

import numpy as np
from PIL import Image


OCCUPIED_MAX = 50
FREE_MIN = 250
SMALL_COMPONENT_CELLS = 5


def load_yaml_map(yaml_path):
    data = {}
    with open(yaml_path, "r") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            data[key.strip()] = value.strip()
    image = data.get("image", "")
    if image.startswith("file://"):
        image = image[7:]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    resolution = float(data.get("resolution", "0.05"))
    return image, resolution


def load_image(pgm_path):
    return np.array(Image.open(pgm_path))


def connected_components(mask):
    h, w = mask.shape
    visited = np.zeros((h, w), dtype=np.uint8)
    sizes = []
    stack = []

    for y in range(h):
        for x in range(w):
            if not mask[y, x] or visited[y, x]:
                continue
            visited[y, x] = 1
            stack[:] = [(y, x)]
            size = 0
            while stack:
                cy, cx = stack.pop()
                size += 1
                for ny, nx in (
                    (cy - 1, cx), (cy + 1, cx), (cy, cx - 1), (cy, cx + 1),
                    (cy - 1, cx - 1), (cy - 1, cx + 1), (cy + 1, cx - 1), (cy + 1, cx + 1),
                ):
                    if ny < 0 or ny >= h or nx < 0 or nx >= w:
                        continue
                    if mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = 1
                        stack.append((ny, nx))
            sizes.append(size)
    return sizes


def wall_continuity(occupied):
    total = int(occupied.sum())
    if total == 0:
        return 0.0
    sizes = connected_components(occupied)
    if not sizes:
        return 0.0
    return float(max(sizes)) / float(total)


def free_connectivity(free):
    total = int(free.sum())
    if total == 0:
        return 0.0
    sizes = connected_components(free)
    if not sizes:
        return 0.0
    return float(max(sizes)) / float(total)


def noise_density(occupied, resolution, small_component_cells=SMALL_COMPONENT_CELLS):
    sizes = connected_components(occupied)
    small_count = 0
    for size in sizes:
        if size < small_component_cells:
            small_count += 1
    total_cells = float(occupied.size)
    return small_count / total_cells if total_cells else 0.0


def boundary_sharpness(img, occupied, free):
    img = img.astype(np.float32)
    gx = np.zeros_like(img)
    gy = np.zeros_like(img)

    gx[:, 1:-1] = img[:, 2:] - img[:, :-2]
    gy[1:-1, :] = img[2:, :] - img[:-2, :]
    grad = np.sqrt(gx * gx + gy * gy)

    occ = occupied
    up = np.zeros_like(occ)
    dn = np.zeros_like(occ)
    lf = np.zeros_like(occ)
    rt = np.zeros_like(occ)
    up[1:, :] = occ[:-1, :]
    dn[:-1, :] = occ[1:, :]
    lf[:, 1:] = occ[:, :-1]
    rt[:, :-1] = occ[:, 1:]
    occ_border = occ & (~(up & dn & lf & rt))

    free_border = free & (~((np.pad(free[:-1, :], ((1, 0), (0, 0)), 'constant')) &
                            (np.pad(free[1:, :], ((0, 1), (0, 0)), 'constant')) &
                            (np.pad(free[:, :-1], ((0, 0), (1, 0)), 'constant')) &
                            (np.pad(free[:, 1:], ((0, 0), (0, 1)), 'constant'))))

    boundary = occ_border | free_border
    if not boundary.any():
        return 0.0
    return float(grad[boundary].mean())


def unknown_ratio(img):
    unknown = (img != 0) & (img != 254)
    total = float(img.size)
    return float(unknown.sum()) / total if total else 0.0


def compute_metrics(yaml_path):
    pgm_path, resolution = load_yaml_map(yaml_path)
    img = load_image(pgm_path)

    occupied = img <= OCCUPIED_MAX
    free = img >= FREE_MIN

    return {
        "yaml": os.path.abspath(yaml_path),
        "image": os.path.abspath(pgm_path),
        "resolution": resolution,
        "wall_continuity": wall_continuity(occupied),
        "free_connectivity": free_connectivity(free),
        "noise_density": noise_density(occupied, resolution),
        "boundary_sharpness": boundary_sharpness(img, occupied, free),
        "unknown_ratio": unknown_ratio(img),
        "occupied_cells": int(occupied.sum()),
        "free_cells": int(free.sum()),
        "unknown_cells": int(((img != 0) & (img != 254)).sum()),
        "map_cells": int(img.size),
    }


def main():
    parser = argparse.ArgumentParser(description="Compute paper map metrics for occupancy grids.")
    parser.add_argument("--yaml", required=True, help="ROS map YAML file")
    args = parser.parse_args()

    metrics = compute_metrics(args.yaml)
    print(json.dumps(metrics, sort_keys=True, indent=2))


if __name__ == "__main__":
    main()
