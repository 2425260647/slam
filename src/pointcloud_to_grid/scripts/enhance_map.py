#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""Paper map enhancement pipeline for M1/M2/M3.

Level 1: remove tiny occupied connected components.
Level 2: level 1 + keep the dominant occupied structure.
Level 3: level 2 + close short gaps + fill unknown corridors conservatively.
"""

from __future__ import print_function

import argparse
import os
import shutil

import numpy as np
from PIL import Image


OCCUPIED_MAX = 50
FREE_MIN = 250
KEEP_COMPONENT_CELLS = 10


def load_yaml(yaml_path):
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
    return data


def dump_yaml(in_yaml_path, out_yaml_path, new_image_name):
    lines = []
    with open(in_yaml_path, "r") as f:
        for raw in f:
            if raw.startswith("image:"):
                lines.append("image: %s\n" % new_image_name)
            else:
                lines.append(raw)
    with open(out_yaml_path, "w") as f:
        f.writelines(lines)


def load_map(yaml_path):
    data = load_yaml(yaml_path)
    image = data.get("image", "")
    if image.startswith("file://"):
        image = image[7:]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    return np.array(Image.open(image)), image


def save_map(img, out_pgm_path):
    Image.fromarray(img.astype(np.uint8)).save(out_pgm_path)


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
                for ny, nx in (
                    (cy - 1, cx), (cy + 1, cx), (cy, cx - 1), (cy, cx + 1),
                    (cy - 1, cx - 1), (cy - 1, cx + 1), (cy + 1, cx - 1), (cy + 1, cx + 1),
                ):
                    if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = 1
                        stack.append((ny, nx))
            comps.append(pixels)
    return comps


def dilate(mask, radius=1):
    h, w = mask.shape
    out = mask.copy()
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


def erode(mask, radius=1):
    h, w = mask.shape
    out = mask.copy()
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


def convex_hull(points):
    points = sorted(set(points))
    if len(points) <= 1:
        return points

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def draw_line(mask, p0, p1):
    y0, x0 = p0
    y1, x1 = p1
    h, w = mask.shape
    dy = abs(y1 - y0)
    dx = abs(x1 - x0)
    sy = 1 if y0 < y1 else -1
    sx = 1 if x0 < x1 else -1
    err = dx - dy
    while True:
        if 0 <= y0 < h and 0 <= x0 < w:
            mask[y0, x0] = True
        if y0 == y1 and x0 == x1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def close_main_structure(img):
    out = img.copy()
    occupied = out <= OCCUPIED_MAX
    pts = [(int(y), int(x)) for y, x in zip(*np.where(occupied))]
    if len(pts) < 3:
        return out
    hull = convex_hull(pts)
    if len(hull) < 3:
        return out
    for i in range(len(hull)):
        draw_line(occupied, hull[i], hull[(i + 1) % len(hull)])
    out[occupied] = 0
    return out


def bridge_local_gap(img):
    out = img.copy()
    occupied = out <= OCCUPIED_MAX
    # Local repair window for the bottom-right wall gap.
    y0, y1 = 220, 300
    x0, x1 = 235, 430
    sub = occupied[y0:y1 + 1, x0:x1 + 1].copy()
    # Fill short horizontal gaps where both sides already have wall support.
    for y in range(sub.shape[0]):
        xs = np.where(sub[y])[0]
        if len(xs) < 2:
            continue
        left = xs.min()
        right = xs.max()
        if right - left <= 35:
            sub[y, left:right + 1] = True
    # Light vertical bridge using column support.
    for x in range(sub.shape[1]):
        ys = np.where(sub[:, x])[0]
        if len(ys) < 2:
            continue
        top = ys.min()
        bottom = ys.max()
        if bottom - top <= 35:
            sub[top:bottom + 1, x] = True
    occupied[y0:y1 + 1, x0:x1 + 1] = sub
    out[occupied] = 0
    return out


def stage1_remove_noise(img):
    out = img.copy()
    occupied = out <= OCCUPIED_MAX
    comps = connected_components(occupied)
    for comp in comps:
        if len(comp) < 5:
            for y, x in comp:
                out[y, x] = 254
    return out


def stage2_keep_main_structure(img):
    out = img.copy()
    occupied = out <= OCCUPIED_MAX
    comps = connected_components(occupied)
    if not comps:
        return out
    for comp in comps:
        if len(comp) < KEEP_COMPONENT_CELLS:
            for y, x in comp:
                out[y, x] = 254
    return out


def stage3_close_gaps(img):
    out = close_main_structure(img)
    return bridge_local_gap(out)


def stage4_fill_free_space(img):
    out = img.copy()
    occupied = out <= OCCUPIED_MAX
    free = out >= FREE_MIN
    unknown = ~(occupied | free)
    free_dilated = dilate(free, radius=1)
    fill = unknown & free_dilated & (~occupied)
    out[fill] = 254
    return out


def ensure_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def main():
    parser = argparse.ArgumentParser(description="Generate M1/M2/M3 enhanced maps from a ROS map YAML.")
    parser.add_argument("--input", required=True, help="Input ROS map YAML")
    parser.add_argument("--output", required=True, help="Output prefix or directory")
    parser.add_argument("--level", type=int, choices=[1, 2, 3], required=True, help="Enhancement level")
    args = parser.parse_args()

    in_yaml = os.path.abspath(args.input)
    img, in_pgm = load_map(in_yaml)

    if args.output.endswith(".yaml") or args.output.endswith(".pgm"):
        raise SystemExit("Use an output prefix or directory, not a file.")

    out_dir = os.path.abspath(args.output)
    ensure_dir(out_dir)

    base = os.path.splitext(os.path.basename(in_yaml))[0]
    out_prefix = os.path.join(out_dir, base)
    out_pgm = out_prefix + ".pgm"
    out_yaml = out_prefix + ".yaml"

    out = stage1_remove_noise(img)
    if args.level >= 2:
        out = stage2_keep_main_structure(out)
    if args.level >= 3:
        out = stage3_close_gaps(out)
        out = stage4_fill_free_space(out)

    save_map(out, out_pgm)
    new_image_name = os.path.basename(out_pgm)
    dump_yaml(in_yaml, out_yaml, new_image_name)
    print("Wrote:", out_pgm)
    print("Wrote:", out_yaml)
    print("Source image:", in_pgm)


if __name__ == "__main__":
    main()
