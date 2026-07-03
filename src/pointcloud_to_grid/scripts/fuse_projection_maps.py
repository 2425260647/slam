#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""Fuse baseline, projection V1, and projection V2 maps for paper experiments."""

from __future__ import print_function

import argparse
import os

import numpy as np
from PIL import Image


OCCUPIED_MAX = 50
FREE_MIN = 250
UNKNOWN_VALUE = 205
OCCUPIED_VALUE = 0
FREE_VALUE = 254


def load_yaml(yaml_path):
    data = {}
    with open(yaml_path, "r") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, value = line.split(":", 1)
            data[key.strip()] = value.strip()
    return data


def image_path_from_yaml(yaml_path):
    data = load_yaml(yaml_path)
    image = data.get("image", "")
    if image.startswith("file://"):
        image = image[7:]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    return image


def load_map(yaml_path):
    return np.array(Image.open(image_path_from_yaml(yaml_path)))


def write_yaml(template_yaml, output_yaml, image_name):
    with open(template_yaml, "r") as src:
        lines = src.readlines()
    with open(output_yaml, "w") as dst:
        for line in lines:
            if line.startswith("image:"):
                dst.write("image: %s\n" % image_name)
            else:
                dst.write(line)


def connected_components(mask):
    h, w = mask.shape
    visited = np.zeros((h, w), dtype=np.uint8)
    comps = []
    for y in range(h):
        for x in range(w):
            if not mask[y, x] or visited[y, x]:
                continue
            stack = [(y, x)]
            visited[y, x] = 1
            pixels = []
            while stack:
                cy, cx = stack.pop()
                pixels.append((cy, cx))
                for ny in (cy - 1, cy, cy + 1):
                    for nx in (cx - 1, cx, cx + 1):
                        if ny == cy and nx == cx:
                            continue
                        if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = 1
                            stack.append((ny, nx))
            comps.append(pixels)
    return comps


def add_components_by_size(target, source_mask, min_size, max_size=None):
    for comp in connected_components(source_mask):
        if len(comp) < min_size:
            continue
        if max_size is not None and len(comp) > max_size:
            continue
        for y, x in comp:
            target[y, x] = True


def add_missing_baseline_obstacles(target, baseline_mask, min_size, max_size, coverage_threshold):
    for comp in connected_components(baseline_mask):
        if len(comp) < min_size:
            continue
        if len(comp) > max_size:
            continue
        covered = 0
        for y, x in comp:
            if target[y, x]:
                covered += 1
        coverage = float(covered) / float(len(comp))
        if coverage >= coverage_threshold:
            continue
        for y, x in comp:
            target[y, x] = True


def remove_tiny_components(mask, min_size):
    out = mask.copy()
    for comp in connected_components(mask):
        if len(comp) >= min_size:
            continue
        for y, x in comp:
            out[y, x] = False
    return out


def largest_component_mask(mask):
    comps = connected_components(mask)
    out = np.zeros(mask.shape, dtype=np.bool_)
    if not comps:
        return out
    largest = max(comps, key=len)
    for y, x in largest:
        out[y, x] = True
    return out


def fuse_maps(baseline_img, v1_img, v2_img):
    baseline_occ = baseline_img <= OCCUPIED_MAX
    v1_occ = v1_img <= OCCUPIED_MAX
    v2_occ = v2_img <= OCCUPIED_MAX

    fused_occ = largest_component_mask(v2_occ)
    add_components_by_size(fused_occ, v1_occ, min_size=10, max_size=500)
    add_missing_baseline_obstacles(fused_occ, baseline_occ, min_size=10, max_size=500, coverage_threshold=0.35)
    fused_occ = remove_tiny_components(fused_occ, min_size=3)

    baseline_free = baseline_img >= FREE_MIN
    v1_free = v1_img >= FREE_MIN
    v2_free = v2_img >= FREE_MIN
    free_votes = baseline_free.astype(np.uint8) + v1_free.astype(np.uint8) + v2_free.astype(np.uint8)
    fused_free = (free_votes >= 2) & (~fused_occ)

    out = np.ones(v1_img.shape, dtype=np.uint8) * UNKNOWN_VALUE
    out[fused_free] = FREE_VALUE
    out[fused_occ] = OCCUPIED_VALUE
    return out


def ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def main():
    parser = argparse.ArgumentParser(description="Fuse three M0 map variants.")
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--projection-v1", required=True)
    parser.add_argument("--projection-v2", required=True)
    parser.add_argument("--output-prefix", required=True)
    args = parser.parse_args()

    baseline = load_map(args.baseline)
    v1 = load_map(args.projection_v1)
    v2 = load_map(args.projection_v2)
    if baseline.shape != v1.shape or baseline.shape != v2.shape:
        raise RuntimeError("input maps must have the same shape")

    fused = fuse_maps(baseline, v1, v2)
    out_dir = os.path.dirname(args.output_prefix)
    ensure_dir(out_dir)
    out_pgm = args.output_prefix + ".pgm"
    out_yaml = args.output_prefix + ".yaml"
    Image.fromarray(fused).save(out_pgm)
    write_yaml(args.projection_v2, out_yaml, os.path.basename(out_pgm))
    print("saved fused map: %s" % out_pgm)
    print("saved fused yaml: %s" % out_yaml)


if __name__ == "__main__":
    main()
