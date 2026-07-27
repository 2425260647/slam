#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""Project point clouds to a 2D occupancy grid with source-side optimization."""

from __future__ import print_function

import argparse
import os
import struct
import zlib

import numpy as np
from PIL import Image


def parse_header(header_text):
    fields = []
    sizes = []
    counts = []
    points = 0
    for line in header_text.splitlines():
        line = line.strip()
        if line.startswith("FIELDS "):
            fields = line.split()[1:]
        elif line.startswith("SIZE "):
            sizes = [int(x) for x in line.split()[1:]]
        elif line.startswith("COUNT "):
            counts = [int(x) for x in line.split()[1:]]
        elif line.startswith("POINTS "):
            points = int(line.split()[1])
    if not counts:
        counts = [1] * len(fields)
    return fields, sizes, counts, points


def load_pcd(path):
    with open(path, "rb") as f:
        data = f.read()
    marker_pos = data.find(b"DATA ")
    if marker_pos < 0:
        raise RuntimeError("PCD has no DATA line.")
    header_end = data.find(b"\n", marker_pos)
    header = data[:header_end + 1].decode("utf-8", errors="replace")
    body = data[header_end + 1:]
    fields, sizes, counts, points = parse_header(header)
    data_line = header.splitlines()[-1].strip()
    data_type = data_line.split()[1] if data_line.startswith("DATA ") else ""

    step = sum(s * c for s, c in zip(sizes, counts))
    offsets = []
    off = 0
    for s, c in zip(sizes, counts):
        offsets.append(off)
        off += s * c

    x_off = y_off = z_off = None
    for idx, name in enumerate(fields):
        if name == "x":
            x_off = offsets[idx]
        elif name == "y":
            y_off = offsets[idx]
        elif name == "z":
            z_off = offsets[idx]
    if x_off is None or y_off is None or z_off is None:
        raise RuntimeError("PCD must contain x/y/z fields")

    if data_type == "ascii":
        x_idx = fields.index("x")
        y_idx = fields.index("y")
        z_idx = fields.index("z")
        rows = []
        for line in body.decode("utf-8", errors="ignore").splitlines():
            if not line.strip():
                continue
            cols = line.split()
            rows.append((float(cols[x_idx]), float(cols[y_idx]), float(cols[z_idx])))
        if not rows:
            return np.array([], dtype=np.float32), np.array([], dtype=np.float32), np.array([], dtype=np.float32)
        arr = np.array(rows, dtype=np.float32)
        return arr[:, 0], arr[:, 1], arr[:, 2]

    if data_type == "binary_compressed":
        comp_size, uncomp_size = struct.unpack("II", body[:8])
        blob = zlib.decompress(body[8:8 + comp_size])
        if len(blob) != uncomp_size:
            raise RuntimeError("PCD decompression size mismatch")
    elif data_type == "binary":
        blob = body
    else:
        raise RuntimeError("Unsupported PCD DATA type: %s" % data_type)

    xs = np.empty(points, dtype=np.float32)
    ys = np.empty(points, dtype=np.float32)
    zs = np.empty(points, dtype=np.float32)
    for i in range(points):
        start = i * step
        xs[i] = struct.unpack("f", blob[start + x_off:start + x_off + 4])[0]
        ys[i] = struct.unpack("f", blob[start + y_off:start + y_off + 4])[0]
        zs[i] = struct.unpack("f", blob[start + z_off:start + z_off + 4])[0]
    return xs, ys, zs


def estimate_ground(zs):
    if len(zs) == 0:
        return 0.0
    return float(np.percentile(zs, 10))


def write_yaml(yaml_path, pgm_name, resolution, origin):
    with open(yaml_path, "w") as f:
        f.write("image: %s\n" % pgm_name)
        f.write("resolution: %.6f\n" % resolution)
        f.write("origin: [%.6f, %.6f, %.6f]\n" % origin)
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")


def main():
    parser = argparse.ArgumentParser(description="Source-side pointcloud to occupancy-grid projection.")
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--z-ground-band", type=float, default=0.06)
    parser.add_argument("--z-obstacle-min", type=float, default=0.08)
    parser.add_argument("--z-obstacle-max", type=float, default=1.60)
    parser.add_argument("--min-points-per-cell", type=int, default=2)
    parser.add_argument("--map-size", type=int, default=0,
                        help="0 means infer map size from point cloud bounds.")
    parser.add_argument("--origin-x", type=float, default=None)
    parser.add_argument("--origin-y", type=float, default=None)
    parser.add_argument("--origin-yaw", type=float, default=0.0)
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--inflate-cells", type=int, default=2)
    args = parser.parse_args()

    xs, ys, zs = load_pcd(args.pcd)
    finite = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
    xs, ys, zs = xs[finite], ys[finite], zs[finite]
    if len(xs) == 0:
        raise RuntimeError("PCD has no finite x/y/z points")
    z_ground = estimate_ground(zs)
    z_rel = zs - z_ground

    obstacle_mask = (z_rel >= args.z_obstacle_min) & (z_rel <= args.z_obstacle_max)
    free_mask = np.abs(z_rel) <= args.z_ground_band

    if args.origin_x is None:
        args.origin_x = float(np.min(xs)) - args.padding
    if args.origin_y is None:
        args.origin_y = float(np.min(ys)) - args.padding
    if args.map_size > 0:
        width = args.map_size
        height = args.map_size
    else:
        width = int(np.ceil((float(np.max(xs)) - args.origin_x + args.padding) / args.resolution))
        height = int(np.ceil((float(np.max(ys)) - args.origin_y + args.padding) / args.resolution))
    occ_votes = np.zeros((height, width), dtype=np.int32)
    free_votes = np.zeros((height, width), dtype=np.int32)

    def to_cell(x, y):
        ix = int((x - args.origin_x) / args.resolution)
        iy = int((y - args.origin_y) / args.resolution)
        return ix, iy

    for x, y, keep_occ, keep_free in zip(xs, ys, obstacle_mask, free_mask):
        ix, iy = to_cell(float(x), float(y))
        if 0 <= ix < width and 0 <= iy < height:
            if keep_occ:
                occ_votes[iy, ix] += 1
            elif keep_free:
                free_votes[iy, ix] += 1

    img = np.ones((height, width), dtype=np.uint8) * 205
    img[free_votes >= args.min_points_per_cell] = 254
    img[occ_votes >= args.min_points_per_cell] = 0

    # Inflate occupied cells for map server and vehicle safety margin.
    occ = img == 0
    inflated = occ.copy()
    for y in range(height):
        for x in range(width):
            if not occ[y, x]:
                continue
            for ny in range(y - args.inflate_cells, y + args.inflate_cells + 1):
                for nx in range(x - args.inflate_cells, x + args.inflate_cells + 1):
                    if 0 <= ny < height and 0 <= nx < width:
                        inflated[ny, nx] = True
    img[inflated] = 0

    out_pgm = args.output + ".pgm"
    out_yaml = args.output + ".yaml"
    Image.fromarray(img).save(out_pgm)
    write_yaml(out_yaml, os.path.basename(out_pgm), args.resolution, (args.origin_x, args.origin_y, args.origin_yaw))
    print("Wrote", out_pgm)
    print("Wrote", out_yaml)
    print("ground_z", z_ground)
    print("origin", args.origin_x, args.origin_y, args.origin_yaw)
    print("size", width, height)


if __name__ == "__main__":
    main()
