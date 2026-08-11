#!/usr/bin/env python3
import argparse
import json
import os
import sys

sys.path.insert(0, '/home/slam/slam_ws/paper_exp/innovation2_slip_adaptive_backend')
from analyze_three_way_experiment import (  # noqa: E402
    connected_components,
    pca_shape_metrics,
    read_pgm,
    read_resolution,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('pgm')
    parser.add_argument('yaml')
    args = parser.parse_args()

    image = read_pgm(args.pgm)
    resolution, origin = read_resolution(args.yaml)
    occupied = image == 0
    free = image == 254
    known = occupied | free
    components = connected_components(occupied)
    largest = components[0] if components else 0
    metrics = {
        'width_px': int(image.shape[1]),
        'height_px': int(image.shape[0]),
        'resolution_m': resolution,
        'origin': origin,
        'map_area_m2': float(image.shape[0] * image.shape[1] * resolution * resolution),
        'occupied_cells': int(occupied.sum()),
        'free_cells': int(free.sum()),
        'known_cells': int(known.sum()),
        'unknown_cells': int((~known).sum()),
        'occupied_components': int(len(components)),
        'largest_component_cells': int(largest),
        'largest_component_ratio': float(largest / max(1, occupied.sum())),
        'occupied_ratio_known': float(occupied.sum() / max(1, known.sum())),
    }
    metrics.update(pca_shape_metrics(occupied, resolution))
    print(json.dumps(metrics, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
