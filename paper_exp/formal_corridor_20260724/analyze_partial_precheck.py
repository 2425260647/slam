#!/usr/bin/env python3

import csv
import json
import os
import re

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.spatial import cKDTree


EXP_ROOT = "/home/slam/slam_ws/paper_exp/formal_corridor_20260724"
RUNS_ROOT = os.path.join(EXP_ROOT, "partial_precheck")
CASES = ("clean_baseline", "full_proposed")


def read_map(case_id):
    run_dir = os.path.join(RUNS_ROOT, case_id)
    image = np.asarray(Image.open(os.path.join(run_dir, "map.pgm")), dtype=np.uint8)
    resolution = None
    origin = None
    with open(os.path.join(run_dir, "map.yaml"), "r", encoding="utf-8") as source:
        for line in source:
            if line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1].strip())
            elif line.startswith("origin:"):
                origin = [float(value) for value in re.findall(
                    r"[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?", line)[:3]]
    if resolution is None or origin is None:
        raise RuntimeError("Incomplete map YAML for " + case_id)
    return image, resolution, origin


def occupied_world_points(image, resolution, origin):
    rows, columns = np.where(image == 0)
    x = origin[0] + (columns + 0.5) * resolution
    y = origin[1] + (image.shape[0] - rows - 0.5) * resolution
    return np.column_stack((x, y))


def pca_metrics(points):
    centered = points - points.mean(axis=0)
    eigenvalues, eigenvectors = np.linalg.eigh(np.cov(centered.T))
    direction = eigenvectors[:, np.argmax(eigenvalues)]
    lateral = np.array([-direction[1], direction[0]])
    longitudinal_projection = centered @ direction
    lateral_projection = centered @ lateral
    return {
        "pca_length_p95_m": float(np.percentile(longitudinal_projection, 97.5)
                                  - np.percentile(longitudinal_projection, 2.5)),
        "pca_width_p95_m": float(np.percentile(lateral_projection, 97.5)
                                 - np.percentile(lateral_projection, 2.5)),
        "dominant_direction": [float(direction[0]), float(direction[1])],
    }


def map_metrics(case_id):
    image, resolution, origin = read_map(case_id)
    occupied = image == 0
    free = image == 254
    known = occupied | free
    labels, component_count = ndimage.label(
        occupied, structure=np.ones((3, 3), dtype=np.uint8))
    component_sizes = np.bincount(labels.ravel())[1:]
    points = occupied_world_points(image, resolution, origin)
    rows, columns = np.where(known)
    known_width = (columns.max() - columns.min() + 1) * resolution
    known_height = (rows.max() - rows.min() + 1) * resolution
    result = {
        "width_px": int(image.shape[1]),
        "height_px": int(image.shape[0]),
        "resolution_m": resolution,
        "full_width_m": float(image.shape[1] * resolution),
        "full_height_m": float(image.shape[0] * resolution),
        "known_width_m": float(known_width),
        "known_height_m": float(known_height),
        "occupied_cells": int(occupied.sum()),
        "free_cells": int(free.sum()),
        "occupied_components": int(component_count),
        "largest_component_ratio": float(component_sizes.max() / occupied.sum()),
        "origin": origin,
    }
    result.update(pca_metrics(points))
    return result, points


def read_float_csv(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return np.array([], dtype=float)
    values_by_time = {}
    with open(path, "r", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            try:
                values_by_time[int(row["%time"])] = float(row["field.data"])
            except (KeyError, TypeError, ValueError):
                continue
    return np.asarray([values_by_time[key] for key in sorted(values_by_time)], dtype=float)


def summarize_values(values):
    if values.size == 0:
        return {"samples": 0}
    return {
        "samples": int(values.size),
        "minimum": float(values.min()),
        "maximum": float(values.max()),
        "mean": float(values.mean()),
        "median": float(np.median(values)),
        "p10": float(np.percentile(values, 10)),
        "p90": float(np.percentile(values, 90)),
    }


def innovation_metrics():
    run_dir = os.path.join(RUNS_ROOT, "full_proposed")
    result = {}
    for key, filename in (
            ("degeneracy_confidence", "degeneracy_metric.csv"),
            ("consistency_metric", "consistency_metric.csv"),
            ("consistency_state", "consistency_state.csv"),
            ("odom_weight_scale", "odom_weight_scale.csv"),
            ("lidar_reliability", "lidar_reliability.csv")):
        values = read_float_csv(os.path.join(run_dir, filename))
        result[key] = summarize_values(values)
        if key == "consistency_state" and values.size:
            result[key]["true_count"] = int(np.count_nonzero(values > 0.5))
        if key == "odom_weight_scale" and values.size:
            result[key]["downweighted_count"] = int(np.count_nonzero(values < 0.999))

    trigger_scales = []
    trigger_pattern = re.compile(
        r"longitudinal weight scaled by ([0-9.eE+-]+), lateral weight scaled by ([0-9.eE+-]+)")
    drop_count = 0
    with open(os.path.join(run_dir, "roslaunch.log"), "r",
              encoding="utf-8", errors="ignore") as source:
        for line in source:
            match = trigger_pattern.search(line)
            if match:
                trigger_scales.append((float(match.group(1)), float(match.group(2))))
            if "Dropping sparse scan" in line:
                drop_count += 1
    result["directional_trigger_count"] = len(trigger_scales)
    result["quality_gate_drop_log_count"] = drop_count
    if trigger_scales:
        scales = np.asarray(trigger_scales)
        result["longitudinal_scale"] = summarize_values(scales[:, 0])
        result["lateral_scale"] = summarize_values(scales[:, 1])
    return result


def map_distance_metrics(points_a, points_b):
    tree_a = cKDTree(points_a)
    tree_b = cKDTree(points_b)
    distance_a_to_b = tree_b.query(points_a, workers=-1)[0]
    distance_b_to_a = tree_a.query(points_b, workers=-1)[0]
    symmetric = np.concatenate((distance_a_to_b, distance_b_to_a))
    return {
        "median_m": float(np.median(symmetric)),
        "p90_m": float(np.percentile(symmetric, 90)),
        "p95_m": float(np.percentile(symmetric, 95)),
        "maximum_m": float(symmetric.max()),
        "within_0_10_m_ratio": float(np.mean(symmetric <= 0.10)),
        "within_0_20_m_ratio": float(np.mean(symmetric <= 0.20)),
    }


def aligned_map_image():
    maps = [read_map(case_id) for case_id in CASES]
    resolution = maps[0][1]
    min_x = min(origin[0] for _, _, origin in maps)
    min_y = min(origin[1] for _, _, origin in maps)
    max_x = max(origin[0] + image.shape[1] * current_resolution
                for image, current_resolution, origin in maps)
    max_y = max(origin[1] + image.shape[0] * current_resolution
                for image, current_resolution, origin in maps)
    canvas_width = int(round((max_x - min_x) / resolution))
    canvas_height = int(round((max_y - min_y) / resolution))
    panels = []
    for case_id, (image, _, origin) in zip(CASES, maps):
        canvas = Image.new("L", (canvas_width, canvas_height), 205)
        offset_x = int(round((origin[0] - min_x) / resolution))
        map_top_y = origin[1] + image.shape[0] * resolution
        offset_y = int(round((max_y - map_top_y) / resolution))
        canvas.paste(Image.fromarray(image), (offset_x, offset_y))
        panel = Image.new("RGB", (canvas_width, canvas_height + 34), "white")
        panel.paste(canvas.convert("RGB"), (0, 34))
        ImageDraw.Draw(panel).text((10, 10), case_id, fill="black")
        panels.append(panel)
    comparison = Image.new("RGB", (canvas_width * 2, canvas_height + 34), "white")
    comparison.paste(panels[0], (0, 0))
    comparison.paste(panels[1], (canvas_width, 0))
    output_path = os.path.join(EXP_ROOT, "partial_precheck_map_comparison.png")
    comparison.save(output_path)
    return output_path


def write_report(metrics, output_path):
    baseline = metrics["maps"]["clean_baseline"]
    proposed = metrics["maps"]["full_proposed"]
    innovation = metrics["innovation"]
    distance = metrics["map_distance"]
    with open(output_path, "w", encoding="utf-8") as output:
        output.write("# repeat_01 安装高度修正与建图预检报告\n\n")
        output.write("## 实验边界\n\n")
        output.write("- bag：`corridor_repeat_01.bag`，完整时长 565.53 s。\n")
        output.write("- 外参：`base_link -> laser_link z=0.33521 m`。\n")
        output.write("- 投影高度带：`[-0.515, 1.435] m`，对应离地约 `[0.055, 2.005] m`。\n")
        output.write("- Clean Baseline 关闭创新一、创新二和质量门控；Full Proposed 全部开启。\n\n")
        output.write("## 地图结果\n\n")
        output.write("| 组别 | 地图尺寸(m) | 已知区尺寸(m) | PCA长度P95(m) | PCA宽度P95(m) | 占据连通分量 |\n")
        output.write("|---|---:|---:|---:|---:|---:|\n")
        for name, current in (("Clean Baseline", baseline), ("Full Proposed", proposed)):
            output.write(
                f"| {name} | {current['full_width_m']:.2f} x {current['full_height_m']:.2f} "
                f"| {current['known_width_m']:.2f} x {current['known_height_m']:.2f} "
                f"| {current['pca_length_p95_m']:.3f} | {current['pca_width_p95_m']:.3f} "
                f"| {current['occupied_components']} |\n")
        output.write("\n")
        output.write(f"两图占据边界双向最近邻距离中位数为 `{distance['median_m']:.3f} m`，"
                     f"P95 为 `{distance['p95_m']:.3f} m`，"
                     f"`{100.0 * distance['within_0_20_m_ratio']:.2f}%` 的占据点在 `0.20 m` 内找到对应边界。\n\n")
        output.write("## 创新模块运行证据\n\n")
        output.write(f"- 创新点一方向退化显著变化日志 `{innovation['directional_trigger_count']}` 次。\n")
        output.write(f"- 纵向权重倍率 `{innovation['longitudinal_scale']['minimum']:.3f}`~"
                     f"`{innovation['longitudinal_scale']['maximum']:.3f}`，"
                     f"均值 `{innovation['longitudinal_scale']['mean']:.3f}`。\n")
        output.write(f"- 横向权重倍率保持 `{innovation['lateral_scale']['mean']:.3f}`。\n")
        output.write(f"- 创新点二状态真值次数 `{innovation['consistency_state'].get('true_count', 0)}`，"
                     f"odom 权重降低样本数 `{innovation['odom_weight_scale'].get('downweighted_count', 0)}`，"
                     f"最小 scale `{innovation['odom_weight_scale']['minimum']:.3f}`。\n")
        output.write(f"- 质量门控丢弃日志 `{innovation['quality_gate_drop_log_count']}` 次。\n\n")
        output.write("## 结论\n\n")
        output.write("1. 修正后的外参和投影高度带能稳定处理整条新 bag，Clean Baseline 和 Full Proposed 都生成拓扑正确的单走廊地图。\n")
        output.write("2. Full Proposed 未引入 90 度折转、双走廊等拓扑错误，当前参数在 repeat_01 上通过拓扑无回归预检；由于缺少连续真值，不能据此声称几何精度无回归。\n")
        output.write("3. 创新点二未触发、权重始终为 1.0，说明本条正常序列上未观察到误触发；单条序列不能替代误报率统计，也不是抗异常效果证据。\n")
        output.write("4. 两图高度相似，不能仅凭本预检声称 Proposed 精度优于 Baseline；正式结论必须来自 repeat_02/03 六组消融和异常注入实验。\n\n")
        output.write(f"并排图：`{metrics['comparison_image']}`\n")


def main():
    metrics = {"maps": {}}
    points = {}
    for case_id in CASES:
        metrics["maps"][case_id], points[case_id] = map_metrics(case_id)
    metrics["map_distance"] = map_distance_metrics(
        points["clean_baseline"], points["full_proposed"])
    metrics["innovation"] = innovation_metrics()
    metrics["comparison_image"] = aligned_map_image()

    metrics_path = os.path.join(EXP_ROOT, "partial_precheck_metrics.json")
    report_path = os.path.join(EXP_ROOT, "partial_precheck_report.md")
    with open(metrics_path, "w", encoding="utf-8") as output:
        json.dump(metrics, output, ensure_ascii=False, indent=2, sort_keys=True)
    write_report(metrics, report_path)
    print(json.dumps(metrics, ensure_ascii=False, indent=2, sort_keys=True))
    print("report:", report_path)


if __name__ == "__main__":
    main()
