#!/usr/bin/env python2
from __future__ import print_function

import argparse
import os
from collections import deque

import numpy as np


def read_pgm(path):
    with open(path, "rb") as pgm_file:
        magic = _read_token(pgm_file)
        if magic != "P5":
            raise ValueError("Unsupported PGM format: %s" % magic)
        width = int(_read_token(pgm_file))
        height = int(_read_token(pgm_file))
        max_value = int(_read_token(pgm_file))
        if max_value != 255:
            raise ValueError("Unsupported PGM max value: %d" % max_value)
        data = pgm_file.read(width * height)
    image = np.frombuffer(data, dtype=np.uint8).copy().reshape((height, width))
    return image


def _read_token(file_obj):
    token = bytearray()
    while True:
        char = file_obj.read(1)
        if not char:
            return None
        if char == b"#":
            file_obj.readline()
            continue
        if char.isspace():
            continue
        token.extend(char)
        break

    while True:
        char = file_obj.read(1)
        if not char or char.isspace():
            break
        if char == b"#":
            file_obj.readline()
            break
        token.extend(char)
    return bytes(token).decode("ascii")


def write_pgm(path, image):
    height, width = image.shape
    with open(path, "wb") as pgm_file:
        pgm_file.write(b"P5\n")
        pgm_file.write(b"# ROI cropped and optimized map\n")
        pgm_file.write(("%d %d\n255\n" % (width, height)).encode("ascii"))
        pgm_file.write(image.astype(np.uint8).tostring(order="C"))


def parse_yaml(path):
    result = {}
    with open(path, "r") as yaml_file:
        for line in yaml_file:
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            result[key.strip()] = value.strip()

    origin_text = result.get("origin", "[0.0, 0.0, 0.0]").strip()
    origin_text = origin_text.strip("[]")
    origin = [float(part.strip()) for part in origin_text.split(",")]
    while len(origin) < 3:
        origin.append(0.0)

    return {
        "resolution": float(result.get("resolution", "0.05")),
        "origin": origin,
        "negate": int(result.get("negate", "0")),
        "occupied_thresh": float(result.get("occupied_thresh", "0.65")),
        "free_thresh": float(result.get("free_thresh", "0.196")),
    }


def crop_image(image, x_min, x_max, y_min, y_max):
    height, width = image.shape
    if not (0 <= x_min < x_max <= width):
        raise ValueError("Invalid ROI x range [%d, %d) for width %d" % (x_min, x_max, width))
    if not (0 <= y_min < y_max <= height):
        raise ValueError("Invalid ROI y range [%d, %d) for height %d" % (y_min, y_max, height))

    # y_min/y_max are map coordinates from the lower image edge. PGM rows start at top.
    row_min = height - y_max
    row_max = height - y_min
    return image[row_min:row_max, x_min:x_max].copy()


def optimize_roi(image, min_obstacle_area=8, free_expand_cells=2, obstacle_keepout_cells=2, fill_hole_area=80):
    occupied = image <= 50
    free = image >= 250

    clean_occupied = occupied.copy()
    for cells, _touches_border in connected_components(occupied):
        if len(cells) < min_obstacle_area:
            for y, x in cells:
                clean_occupied[y, x] = False

    keepout = inflate_mask(clean_occupied, obstacle_keepout_cells)
    clean_free = free.copy()

    for _ in range(free_expand_cells):
        expanded = inflate_mask(clean_free, 1)
        candidates = np.logical_and.reduce((expanded, np.logical_not(clean_free), np.logical_not(keepout)))
        clean_free[candidates] = True

    unknown = np.logical_not(np.logical_or(clean_occupied, clean_free))
    for cells, touches_border in connected_components(unknown):
        if touches_border or len(cells) > fill_hole_area:
            continue
        safe = True
        for y, x in cells:
            if keepout[y, x]:
                safe = False
                break
        if safe:
            for y, x in cells:
                clean_free[y, x] = True

    clean_free = np.logical_and(clean_free, np.logical_not(clean_occupied))

    result = np.full(image.shape, 205, dtype=np.uint8)
    result[clean_free] = 254
    result[clean_occupied] = 0
    return result


def connected_components(mask):
    height, width = mask.shape
    visited = np.zeros(mask.shape, dtype=np.uint8)
    components = []

    for start_y in range(height):
        starts = np.where(np.logical_and(mask[start_y], visited[start_y] == 0))[0]
        for start_x in starts.tolist():
            if visited[start_y, start_x] or not mask[start_y, start_x]:
                continue

            queue = deque([(start_y, start_x)])
            visited[start_y, start_x] = 1
            cells = []
            touches_border = False

            while queue:
                y, x = queue.popleft()
                cells.append((y, x))
                if x == 0 or y == 0 or x == width - 1 or y == height - 1:
                    touches_border = True

                for next_y in range(max(0, y - 1), min(height, y + 2)):
                    for next_x in range(max(0, x - 1), min(width, x + 2)):
                        if visited[next_y, next_x] or not mask[next_y, next_x]:
                            continue
                        visited[next_y, next_x] = 1
                        queue.append((next_y, next_x))

            components.append((cells, touches_border))

    return components


def inflate_mask(mask, radius_cells):
    if radius_cells <= 0:
        return mask.copy()

    height, width = mask.shape
    result = mask.copy()
    ys, xs = np.where(mask)
    for y, x in zip(ys.tolist(), xs.tolist()):
        y0 = max(0, y - radius_cells)
        y1 = min(height, y + radius_cells + 1)
        x0 = max(0, x - radius_cells)
        x1 = min(width, x + radius_cells + 1)
        result[y0:y1, x0:x1] = True
    return result


def save_yaml(path, image_name, resolution, origin, negate, occupied_thresh, free_thresh):
    with open(path, "w") as yaml_file:
        yaml_file.write("image: %s\n" % image_name)
        yaml_file.write("resolution: %.6f\n" % resolution)
        yaml_file.write("origin: [%.6f, %.6f, %.6f]\n" % (origin[0], origin[1], origin[2]))
        yaml_file.write("negate: %d\n" % negate)
        yaml_file.write("occupied_thresh: %.3f\n" % occupied_thresh)
        yaml_file.write("free_thresh: %.3f\n" % free_thresh)


def write_preview_png(path, image):
    try:
        from PIL import Image, ImageOps
    except ImportError:
        return False
    preview = ImageOps.flip(Image.fromarray(image, "L"))
    preview.save(path)
    return True


def map_stats(image):
    occupied = int((image <= 50).sum())
    free = int((image >= 250).sum())
    unknown = int(image.size - occupied - free)
    return occupied, free, unknown


def make_arg_parser():
    parser = argparse.ArgumentParser(description="Crop and optimize a task ROI from a ROS PGM/YAML map")
    parser.add_argument("--input-pgm", required=True)
    parser.add_argument("--input-yaml", required=True)
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--x-min", type=int, default=0)
    parser.add_argument("--x-max", type=int, default=250)
    parser.add_argument("--y-min", type=int, default=500)
    parser.add_argument("--y-max", type=int, default=850)
    parser.add_argument("--min-obstacle-area", type=int, default=8)
    parser.add_argument("--free-expand-cells", type=int, default=2)
    parser.add_argument("--obstacle-keepout-cells", type=int, default=2)
    parser.add_argument("--fill-hole-area", type=int, default=80)
    parser.add_argument("--preview-png", default="")
    return parser


def main(argv=None):
    args = make_arg_parser().parse_args(argv)
    yaml_info = parse_yaml(args.input_yaml)
    image = read_pgm(args.input_pgm)
    crop = crop_image(image, args.x_min, args.x_max, args.y_min, args.y_max)
    optimized = optimize_roi(
        crop,
        min_obstacle_area=args.min_obstacle_area,
        free_expand_cells=args.free_expand_cells,
        obstacle_keepout_cells=args.obstacle_keepout_cells,
        fill_hole_area=args.fill_hole_area,
    )

    output_dir = os.path.dirname(os.path.abspath(args.output_prefix))
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    output_pgm = args.output_prefix + ".pgm"
    output_yaml = args.output_prefix + ".yaml"
    image_name = os.path.basename(output_pgm)

    resolution = yaml_info["resolution"]
    origin = yaml_info["origin"][:]
    origin[0] += args.x_min * resolution
    origin[1] += args.y_min * resolution

    write_pgm(output_pgm, optimized)
    save_yaml(
        output_yaml,
        image_name,
        resolution,
        origin,
        yaml_info["negate"],
        yaml_info["occupied_thresh"],
        yaml_info["free_thresh"],
    )

    if args.preview_png:
        write_preview_png(args.preview_png, optimized)

    occupied, free, unknown = map_stats(optimized)
    print("ROI map written: %s" % output_pgm)
    print("ROI yaml written: %s" % output_yaml)
    print("ROI pixels: %dx%d" % (optimized.shape[1], optimized.shape[0]))
    print("ROI meters: %.2f x %.2f" % (optimized.shape[1] * resolution, optimized.shape[0] * resolution))
    print("ROI origin: [%.6f, %.6f, %.6f]" % (origin[0], origin[1], origin[2]))
    print("occupied/free/unknown: %d/%d/%d" % (occupied, free, unknown))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
