#!/usr/bin/env python3
import csv
import json
import math
import os
import re
from collections import deque

import numpy as np
from PIL import Image, ImageDraw


EXP_ROOT = "/home/slam/slam_ws/paper_exp/innovation1_directional_adaptive_fusion"
RUNS_DIR = os.path.join(EXP_ROOT, "runs_valid")
CASES = [
    ("baseline_a_static_low", "Baseline A\nstatic low"),
    ("baseline_b_static_high", "Baseline B\nstatic high"),
    ("proposed_dynamic_anisotropic", "Proposed\ndynamic anisotropic"),
]


def read_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise RuntimeError("Only binary P5 PGM is supported: %s" % path)
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = map(int, line.split())
        max_value = int(f.readline())
        if max_value != 255:
            raise RuntimeError("Unsupported PGM max value: %s" % max_value)
        data = np.frombuffer(f.read(), dtype=np.uint8)
    return data.reshape((height, width))


def read_resolution(yaml_path):
    resolution = None
    origin = None
    with open(yaml_path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1].strip())
            if line.startswith("origin:"):
                nums = re.findall(r"[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?", line)
                origin = [float(x) for x in nums[:3]]
    if resolution is None:
        raise RuntimeError("Missing resolution in %s" % yaml_path)
    return resolution, origin


def connected_components(mask):
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    components = []
    ys, xs = np.where(mask)
    for y0, x0 in zip(ys, xs):
        if seen[y0, x0]:
            continue
        q = deque([(y0, x0)])
        seen[y0, x0] = True
        size = 0
        while q:
            y, x = q.popleft()
            size += 1
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < h and 0 <= xx < w and mask[yy, xx] and not seen[yy, xx]:
                        seen[yy, xx] = True
                        q.append((yy, xx))
        components.append(size)
    components.sort(reverse=True)
    return components


def pca_shape_metrics(mask, resolution):
    ys, xs = np.where(mask)
    if len(xs) < 10:
        return {}
    pts = np.column_stack([xs * resolution, ys * resolution])
    mean = pts.mean(axis=0)
    centered = pts - mean
    cov = np.cov(centered.T)
    vals, vecs = np.linalg.eigh(cov)
    order = np.argsort(vals)[::-1]
    vecs = vecs[:, order]
    proj = centered @ vecs
    long = proj[:, 0]
    lat = proj[:, 1]
    long_length_p95 = float(np.percentile(long, 97.5) - np.percentile(long, 2.5))
    lat_width_p95 = float(np.percentile(lat, 97.5) - np.percentile(lat, 2.5))
    long_length_p90 = float(np.percentile(long, 95) - np.percentile(long, 5))
    lat_width_p90 = float(np.percentile(lat, 95) - np.percentile(lat, 5))

    bins = np.linspace(long.min(), long.max(), 80)
    widths = []
    occupied_bins = 0
    for lo, hi in zip(bins[:-1], bins[1:]):
        selected = lat[(long >= lo) & (long < hi)]
        if len(selected) >= 20:
            occupied_bins += 1
            widths.append(float(np.percentile(selected, 90) - np.percentile(selected, 10)))
    median_lateral_spread = float(np.median(widths)) if widths else 0.0
    mean_lateral_spread = float(np.mean(widths)) if widths else 0.0
    return {
        "pca_long_length_p95_m": long_length_p95,
        "pca_lateral_width_p95_m": lat_width_p95,
        "pca_long_length_p90_m": long_length_p90,
        "pca_lateral_width_p90_m": lat_width_p90,
        "median_lateral_spread_m": median_lateral_spread,
        "mean_lateral_spread_m": mean_lateral_spread,
        "occupied_longitudinal_bins": occupied_bins,
    }


def map_metrics(case_id):
    run_dir = os.path.join(RUNS_DIR, case_id)
    pgm = read_pgm(os.path.join(run_dir, "map.pgm"))
    resolution, origin = read_resolution(os.path.join(run_dir, "map.yaml"))
    occupied = pgm == 0
    free = pgm == 254
    unknown = pgm == 205
    known = occupied | free
    components = connected_components(occupied)
    largest = components[0] if components else 0
    metric = {
        "width_px": int(pgm.shape[1]),
        "height_px": int(pgm.shape[0]),
        "resolution_m": resolution,
        "origin": origin,
        "map_area_m2": float(pgm.shape[0] * pgm.shape[1] * resolution * resolution),
        "occupied_cells": int(occupied.sum()),
        "free_cells": int(free.sum()),
        "unknown_cells": int(unknown.sum()),
        "known_cells": int(known.sum()),
        "occupied_ratio_total": float(occupied.mean()),
        "unknown_ratio_total": float(unknown.mean()),
        "occupied_ratio_known": float(occupied.sum() / max(1, known.sum())),
        "occupied_components": int(len(components)),
        "largest_component_cells": int(largest),
        "largest_component_ratio": float(largest / max(1, occupied.sum())),
    }
    metric.update(pca_shape_metrics(occupied, resolution))
    return metric


def parse_trigger_stats(case_id):
    log_path = os.path.join(RUNS_DIR, case_id, "roslaunch.log")
    values = []
    lateral = []
    pattern = re.compile(r"longitudinal weight scaled by ([0-9.]+), lateral weight scaled by ([0-9.]+)")
    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = pattern.search(line)
            if m:
                values.append(float(m.group(1)))
                lateral.append(float(m.group(2)))
    if not values:
        return {"trigger_count": 0}
    arr = np.array(values)
    lat = np.array(lateral)
    return {
        "trigger_count": int(len(values)),
        "longitudinal_scale_min": float(arr.min()),
        "longitudinal_scale_max": float(arr.max()),
        "longitudinal_scale_mean": float(arr.mean()),
        "lateral_scale_min": float(lat.min()),
        "lateral_scale_max": float(lat.max()),
        "lateral_scale_mean": float(lat.mean()),
    }


def parse_degeneracy_csv(case_id):
    metric_path = os.path.join(RUNS_DIR, case_id, "degeneracy_metric.csv")
    direction_path = os.path.join(RUNS_DIR, case_id, "degeneracy_direction.csv")
    result = {}
    if os.path.getsize(metric_path) > 0:
        values = []
        with open(metric_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                values.append(float(row["field.data"]))
        arr = np.array(values)
        result.update({
            "metric_samples": int(len(values)),
            "metric_min": float(arr.min()),
            "metric_max": float(arr.max()),
            "metric_mean": float(arr.mean()),
            "metric_median": float(np.median(arr)),
            "metric_p90": float(np.percentile(arr, 90)),
        })
    if os.path.getsize(direction_path) > 0:
        xs, ys = [], []
        with open(direction_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                xs.append(float(row["field.x"]))
                ys.append(float(row["field.y"]))
        xarr = np.array(xs)
        yarr = np.array(ys)
        result.update({
            "direction_samples": int(len(xs)),
            "direction_mean_x": float(xarr.mean()),
            "direction_mean_y": float(yarr.mean()),
            "direction_mean_abs_x": float(np.abs(xarr).mean()),
            "direction_mean_abs_y": float(np.abs(yarr).mean()),
        })
    return result


def make_comparison_image(metrics):
    panels = []
    for case_id, label in CASES:
        pgm = read_pgm(os.path.join(RUNS_DIR, case_id, "map.pgm"))
        img = Image.fromarray(pgm, mode="L").convert("RGB")
        # Crop to non-unknown content with padding for easier visual comparison.
        arr = np.array(img.convert("L"))
        ys, xs = np.where(arr != 205)
        if len(xs) > 0:
            pad = 30
            x0, x1 = max(0, xs.min() - pad), min(arr.shape[1], xs.max() + pad)
            y0, y1 = max(0, ys.min() - pad), min(arr.shape[0], ys.max() + pad)
            img = img.crop((x0, y0, x1, y1))
        resample_filter = getattr(Image, "Resampling", Image).LANCZOS
        img.thumbnail((520, 260), resample_filter)
        panel = Image.new("RGB", (540, 330), "white")
        panel.paste(img, ((540 - img.width) // 2, 52))
        draw = ImageDraw.Draw(panel)
        draw.text((12, 10), label, fill="black")
        draw.text((12, 282),
                  "area %.1fm2  comp %d  lat %.2fm" %
                  (metrics[case_id]["map_area_m2"],
                   metrics[case_id]["occupied_components"],
                   metrics[case_id].get("median_lateral_spread_m", 0.0)),
                  fill="black")
        panels.append(panel)
    out = Image.new("RGB", (540 * len(panels), 330), "white")
    for i, panel in enumerate(panels):
        out.paste(panel, (i * 540, 0))
    out_path = os.path.join(EXP_ROOT, "map_comparison.png")
    out.save(out_path)
    return out_path


def main():
    metrics = {}
    for case_id, _ in CASES:
        metrics[case_id] = map_metrics(case_id)
        metrics[case_id].update(parse_trigger_stats(case_id))
        metrics[case_id].update(parse_degeneracy_csv(case_id))

    comparison_path = make_comparison_image(metrics)
    out_json = os.path.join(EXP_ROOT, "innovation1_three_way_metrics.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)

    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    print("comparison_image:", comparison_path)
    print("metrics_json:", out_json)


if __name__ == "__main__":
    main()
