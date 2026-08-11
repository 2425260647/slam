#!/usr/bin/env python3
"""Audit comparable PGM maps without using truth to alter any estimate."""

import json
import os
import re
import sys
from collections import deque

import numpy as np


def read_pgm(path):
    with open(path, "rb") as stream:
        if stream.readline().strip() != b"P5":
            raise RuntimeError("only binary P5 PGM is supported: " + path)
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        width, height = map(int, line.split())
        maximum = int(stream.readline())
        if maximum != 255:
            raise RuntimeError("unsupported PGM maximum: {}".format(maximum))
        data = np.frombuffer(stream.read(), dtype=np.uint8)
    if data.size != width * height:
        raise RuntimeError("PGM payload size mismatch: " + path)
    return data.reshape((height, width))


def read_yaml(path):
    resolution = None
    origin = None
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1])
            elif line.startswith("origin:"):
                origin = [float(value) for value in re.findall(
                    r"[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?", line
                )[:3]]
    if resolution is None:
        raise RuntimeError("missing resolution: " + path)
    return resolution, origin


def components(mask):
    height, width = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    sizes = []
    ys, xs = np.where(mask)
    for y0, x0 in zip(ys, xs):
        if seen[y0, x0]:
            continue
        queue = deque([(int(y0), int(x0))])
        seen[y0, x0] = True
        size = 0
        while queue:
            y, x = queue.popleft()
            size += 1
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if not dx and not dy:
                        continue
                    yy, xx = y + dy, x + dx
                    if (0 <= yy < height and 0 <= xx < width
                            and mask[yy, xx] and not seen[yy, xx]):
                        seen[yy, xx] = True
                        queue.append((yy, xx))
        sizes.append(size)
    return sorted(sizes, reverse=True)


def audit(map_dir):
    pgm_path = os.path.join(map_dir, "map.pgm")
    yaml_path = os.path.join(map_dir, "map.yaml")
    image = read_pgm(pgm_path)
    resolution, origin = read_yaml(yaml_path)
    occupied = image == 0
    free = image == 254
    unknown = image == 205
    known = occupied | free
    sizes = components(occupied)
    occupied_count = int(occupied.sum())
    known_count = int(known.sum())
    result = {
        "map_dir": os.path.abspath(map_dir),
        "width_px": int(image.shape[1]),
        "height_px": int(image.shape[0]),
        "resolution_m": resolution,
        "origin": origin,
        "map_area_m2": float(image.size * resolution * resolution),
        "occupied_cells": occupied_count,
        "free_cells": int(free.sum()),
        "unknown_cells": int(unknown.sum()),
        "known_cells": known_count,
        "occupied_ratio_total": float(occupied.mean()),
        "occupied_ratio_known": float(occupied_count / max(1, known_count)),
        "occupied_components_8_connected": len(sizes),
        "largest_component_cells": sizes[0] if sizes else 0,
        "largest_component_ratio": float(sizes[0] / max(1, occupied_count)),
    }
    if occupied_count >= 10:
        ys, xs = np.where(occupied)
        points = np.column_stack((xs, ys)).astype(float) * resolution
        centered = points - points.mean(axis=0)
        eigenvalues = np.linalg.eigvalsh(np.cov(centered.T))
        eigenvalues = np.sort(eigenvalues)[::-1]
        result["occupied_pca_length_ratio"] = float(
            np.sqrt(eigenvalues[0] / max(eigenvalues[-1], 1e-12))
        )
    return result


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: audit_common_loop_maps.py MAP_DIR [MAP_DIR ...]")
    results = {os.path.basename(os.path.normpath(path)): audit(path)
               for path in sys.argv[1:]}
    print(json.dumps(results, indent=2, sort_keys=True))
    output = os.environ.get("MAP_AUDIT_OUTPUT")
    if output:
        with open(output, "w", encoding="utf-8") as stream:
            json.dump(results, stream, indent=2, sort_keys=True)
            stream.write("\n")


if __name__ == "__main__":
    main()
