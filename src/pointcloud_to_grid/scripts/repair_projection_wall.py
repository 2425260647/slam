#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""Repair wall continuity in projection V1 while preserving its obstacle shapes."""

from __future__ import print_function

import argparse
from collections import deque
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


def largest_component(mask):
    comps = connected_components(mask)
    if not comps:
        return []
    return max(comps, key=len)


def dilate(mask, radius):
    out = mask.copy()
    h, w = out.shape
    for _ in range(radius):
        nxt = out.copy()
        ys, xs = np.where(out)
        for y, x in zip(ys, xs):
            for ny in (y - 1, y, y + 1):
                for nx in (x - 1, x, x + 1):
                    if 0 <= ny < h and 0 <= nx < w:
                        nxt[ny, nx] = True
        out = nxt
    return out


def erode(mask, radius):
    out = mask.copy()
    h, w = out.shape
    for _ in range(radius):
        nxt = out.copy()
        for y in range(h):
            for x in range(w):
                if not out[y, x]:
                    nxt[y, x] = False
                    continue
                keep = True
                for ny in (y - 1, y, y + 1):
                    for nx in (x - 1, x, x + 1):
                        if 0 <= ny < h and 0 <= nx < w and not out[ny, nx]:
                            keep = False
                            break
                    if not keep:
                        break
                nxt[y, x] = keep
        out = nxt
    return out


def closing(mask, radius):
    return erode(dilate(mask, radius), radius)


def component_mask(shape, comp):
    out = np.zeros(shape, dtype=np.bool_)
    for y, x in comp:
        out[y, x] = True
    return out


def shortest_bridge(source, target, allowed, max_bridge_cells):
    h, w = source.shape
    visited = np.zeros((h, w), dtype=np.uint8)
    parent_y = np.full((h, w), -1, dtype=np.int32)
    parent_x = np.full((h, w), -1, dtype=np.int32)
    queue = deque()

    ys, xs = np.where(source)
    for y, x in zip(ys, xs):
        visited[y, x] = 1
        queue.append((y, x, 0))

    end = None
    while queue:
        cy, cx, dist = queue.popleft()
        if target[cy, cx]:
            end = (cy, cx)
            break
        if dist >= max_bridge_cells:
            continue
        for ny in (cy - 1, cy, cy + 1):
            for nx in (cx - 1, cx, cx + 1):
                if ny == cy and nx == cx:
                    continue
                if ny < 0 or ny >= h or nx < 0 or nx >= w:
                    continue
                if visited[ny, nx] or not allowed[ny, nx]:
                    continue
                visited[ny, nx] = 1
                parent_y[ny, nx] = cy
                parent_x[ny, nx] = cx
                queue.append((ny, nx, dist + 1))

    if end is None:
        return np.zeros(source.shape, dtype=np.bool_)

    bridge = np.zeros(source.shape, dtype=np.bool_)
    cy, cx = end
    while parent_y[cy, cx] >= 0:
        bridge[cy, cx] = True
        py = parent_y[cy, cx]
        px = parent_x[cy, cx]
        cy, cx = py, px
    bridge[cy, cx] = True
    return bridge


def close_1d_gaps(line, max_gap, min_run):
    out = line.copy()
    n = len(line)
    i = 0
    while i < n:
        if line[i]:
            i += 1
            continue
        gap_start = i
        while i < n and not line[i]:
            i += 1
        gap_end = i
        gap_len = gap_end - gap_start
        if gap_len > max_gap:
            continue

        left_start = gap_start - 1
        while left_start >= 0 and line[left_start]:
            left_start -= 1
        left_len = gap_start - left_start - 1

        right_end = gap_end
        while right_end < n and line[right_end]:
            right_end += 1
        right_len = right_end - gap_end

        if left_len >= min_run and right_len >= min_run:
            out[gap_start:gap_end] = True
    return out


def close_axis_gaps(mask, max_gap, min_run):
    if max_gap <= 0:
        return mask.copy()

    out = mask.copy()
    h, w = out.shape
    for y in range(h):
        out[y, :] = close_1d_gaps(out[y, :], max_gap, min_run)
    for x in range(w):
        out[:, x] = close_1d_gaps(out[:, x], max_gap, min_run)
    return out


def repair_wall_by_support(v1_img, v2_img, support_radius, close_radius):
    v1_occ = v1_img <= OCCUPIED_MAX
    v2_occ = v2_img <= OCCUPIED_MAX

    v1_wall = component_mask(v1_occ.shape, largest_component(v1_occ))
    v2_wall = component_mask(v2_occ.shape, largest_component(v2_occ))

    support = dilate(v1_wall, support_radius)
    wall_patch = v2_wall & support
    repaired_wall = closing(v1_wall | wall_patch, close_radius)

    out_occ = v1_occ.copy()
    out_occ[repaired_wall] = True

    out = v1_img.copy()
    out[out_occ] = OCCUPIED_VALUE
    out[(~out_occ) & (v1_img >= FREE_MIN)] = FREE_VALUE
    out[(~out_occ) & (v1_img < FREE_MIN) & (v1_img > OCCUPIED_MAX)] = UNKNOWN_VALUE
    return out


def repair_wall_by_bridge(v1_img, v2_img, support_radius, max_bridge_cells,
                          min_component_cells, line_gap_cells, line_gap_min_run):
    v1_occ = v1_img <= OCCUPIED_MAX
    v2_occ = v2_img <= OCCUPIED_MAX

    v1_comps = sorted(connected_components(v1_occ), key=len, reverse=True)
    if not v1_comps:
        return v1_img.copy()

    v2_wall = component_mask(v2_occ.shape, largest_component(v2_occ))
    allowed = dilate(v2_wall, support_radius) | v1_occ
    repaired_wall = component_mask(v1_occ.shape, v1_comps[0])
    v2_support = dilate(v2_wall, support_radius)

    for comp in v1_comps[1:]:
        if len(comp) < min_component_cells:
            continue
        comp_mask = component_mask(v1_occ.shape, comp)
        if not (comp_mask & v2_support).any():
            continue
        bridge = shortest_bridge(comp_mask, repaired_wall, allowed, max_bridge_cells)
        if bridge.any():
            repaired_wall |= comp_mask | bridge

    repaired_wall = close_axis_gaps(repaired_wall, line_gap_cells, line_gap_min_run)

    out_occ = v1_occ | repaired_wall
    out = v1_img.copy()
    out[out_occ] = OCCUPIED_VALUE
    out[(~out_occ) & (v1_img >= FREE_MIN)] = FREE_VALUE
    out[(~out_occ) & (v1_img < FREE_MIN) & (v1_img > OCCUPIED_MAX)] = UNKNOWN_VALUE
    return out


def repair_wall(v1_img, v2_img, mode, support_radius, close_radius,
                max_bridge_cells, min_component_cells, line_gap_cells,
                line_gap_min_run):
    if mode == "bridge":
        return repair_wall_by_bridge(v1_img, v2_img, support_radius,
                                     max_bridge_cells, min_component_cells,
                                     line_gap_cells, line_gap_min_run)
    return repair_wall_by_support(v1_img, v2_img, support_radius, close_radius)


def ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def main():
    parser = argparse.ArgumentParser(description="Repair V1 wall continuity using V2 wall support.")
    parser.add_argument("--projection-v1", required=True)
    parser.add_argument("--projection-v2", required=True)
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--mode", choices=("support", "bridge"), default="support")
    parser.add_argument("--support-radius", type=int, default=2)
    parser.add_argument("--close-radius", type=int, default=1)
    parser.add_argument("--max-bridge-cells", type=int, default=80)
    parser.add_argument("--min-component-cells", type=int, default=20)
    parser.add_argument("--line-gap-cells", type=int, default=0)
    parser.add_argument("--line-gap-min-run", type=int, default=6)
    args = parser.parse_args()

    v1 = load_map(args.projection_v1)
    v2 = load_map(args.projection_v2)
    if v1.shape != v2.shape:
        raise RuntimeError("input maps must have the same shape")

    repaired = repair_wall(v1, v2, args.mode, args.support_radius, args.close_radius,
                           args.max_bridge_cells, args.min_component_cells,
                           args.line_gap_cells, args.line_gap_min_run)
    out_dir = os.path.dirname(args.output_prefix)
    ensure_dir(out_dir)
    out_pgm = args.output_prefix + ".pgm"
    out_yaml = args.output_prefix + ".yaml"
    Image.fromarray(repaired).save(out_pgm)
    write_yaml(args.projection_v1, out_yaml, os.path.basename(out_pgm))
    print("saved repaired map: %s" % out_pgm)
    print("saved repaired yaml: %s" % out_yaml)


if __name__ == "__main__":
    main()
