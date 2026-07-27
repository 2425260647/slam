#!/usr/bin/env python3
import csv
import json
import math
import os
import re
from collections import deque

import numpy as np
from PIL import Image, ImageDraw


EXP_ROOT = "/home/slam/slam_ws/paper_exp/innovation2_slip_adaptive_backend"
RUNS_DIR = os.path.join(EXP_ROOT, "runs")
CASES = [
    ("reference_original_proposed", "Reference\noriginal Proposed"),
    ("slip_static_low", "Slip A\nstatic low odom"),
    ("slip_static_high", "Slip B\nstatic high odom"),
    ("slip_proposed_adaptive", "Slip Proposed\nadaptive"),
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
    centered = pts - pts.mean(axis=0)
    vals, vecs = np.linalg.eigh(np.cov(centered.T))
    vecs = vecs[:, np.argsort(vals)[::-1]]
    proj = centered @ vecs
    long = proj[:, 0]
    lat = proj[:, 1]
    bins = np.linspace(long.min(), long.max(), 90)
    widths = []
    occupied_bins = 0
    for lo, hi in zip(bins[:-1], bins[1:]):
        selected = lat[(long >= lo) & (long < hi)]
        if len(selected) >= 20:
            occupied_bins += 1
            widths.append(float(np.percentile(selected, 90) - np.percentile(selected, 10)))
    return {
        "pca_long_length_p90_m": float(np.percentile(long, 95) - np.percentile(long, 5)),
        "pca_lateral_width_p90_m": float(np.percentile(lat, 95) - np.percentile(lat, 5)),
        "median_lateral_spread_m": float(np.median(widths)) if widths else 0.0,
        "mean_lateral_spread_m": float(np.mean(widths)) if widths else 0.0,
        "occupied_longitudinal_bins": occupied_bins,
    }


def map_metrics(case_id):
    run_dir = os.path.join(RUNS_DIR, case_id)
    pgm = read_pgm(os.path.join(run_dir, "map.pgm"))
    resolution, origin = read_resolution(os.path.join(run_dir, "map.yaml"))
    occupied = pgm == 0
    free = pgm == 254
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
        "known_cells": int(known.sum()),
        "occupied_components": int(len(components)),
        "largest_component_cells": int(largest),
        "largest_component_ratio": float(largest / max(1, occupied.sum())),
        "occupied_ratio_known": float(occupied.sum() / max(1, known.sum())),
    }
    metric.update(pca_shape_metrics(occupied, resolution))
    return metric


def parse_float_csv(path, field):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return []
    values = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                values.append(float(row[field]))
            except (KeyError, ValueError):
                pass
    return values


def parse_bool_csv(path, field):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return []
    values = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            raw = row.get(field, "").strip().lower()
            if raw in ("true", "1"):
                values.append(True)
            elif raw in ("false", "0"):
                values.append(False)
    return values


def slip_metrics(case_id):
    run_dir = os.path.join(RUNS_DIR, case_id)
    scores = parse_float_csv(os.path.join(run_dir, "slip_metric.csv"), "field.data")
    scales = parse_float_csv(os.path.join(run_dir, "odom_weight_scale.csv"), "field.data")
    states = parse_bool_csv(os.path.join(run_dir, "slip_state.csv"), "field.data")
    lidar_reliability = parse_float_csv(os.path.join(run_dir, "slip_lidar_reliability.csv"), "field.data")
    result = {}
    if scores:
        arr = np.array(scores)
        result.update({
            "slip_samples": int(len(scores)),
            "slip_score_min": float(arr.min()),
            "slip_score_max": float(arr.max()),
            "slip_score_mean": float(arr.mean()),
            "slip_score_p90": float(np.percentile(arr, 90)),
        })
    if scales:
        arr = np.array(scales)
        result.update({
            "weight_scale_samples": int(len(scales)),
            "weight_scale_min": float(arr.min()),
            "weight_scale_max": float(arr.max()),
            "weight_scale_mean": float(arr.mean()),
            "weight_scale_p10": float(np.percentile(arr, 10)),
        })
    if states:
        result.update({
            "slip_state_samples": int(len(states)),
            "slip_state_true_count": int(sum(states)),
            "slip_state_true_ratio": float(sum(states) / len(states)),
        })
    if lidar_reliability:
        arr = np.array(lidar_reliability)
        result.update({
            "lidar_reliability_samples": int(len(lidar_reliability)),
            "lidar_reliability_min": float(arr.min()),
            "lidar_reliability_max": float(arr.max()),
            "lidar_reliability_mean": float(arr.mean()),
            "lidar_reliability_p10": float(np.percentile(arr, 10)),
            "lidar_reliability_p90": float(np.percentile(arr, 90)),
        })
    return result


def log_trigger_metrics(case_id):
    path = os.path.join(RUNS_DIR, case_id, "roslaunch.log")
    if not os.path.exists(path):
        return {}
    count = 0
    scores = []
    gated_scores = []
    reliabilities = []
    pattern = re.compile(
        r"Wheel slip detected\. score=([0-9.eE+-]+), gated_score=([0-9.eE+-]+).*lidar_reliability=([0-9.eE+-]+)")
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                count += 1
                scores.append(float(match.group(1)))
                gated_scores.append(float(match.group(2)))
                reliabilities.append(float(match.group(3)))
    result = {"log_slip_trigger_count": count}
    if scores:
        arr = np.array(scores)
        result.update({
            "log_slip_score_min": float(arr.min()),
            "log_slip_score_max": float(arr.max()),
            "log_slip_score_mean": float(arr.mean()),
        })
    if gated_scores:
        arr = np.array(gated_scores)
        result.update({
            "log_gated_slip_score_min": float(arr.min()),
            "log_gated_slip_score_max": float(arr.max()),
            "log_gated_slip_score_mean": float(arr.mean()),
        })
    if reliabilities:
        arr = np.array(reliabilities)
        result.update({
            "log_lidar_reliability_min": float(arr.min()),
            "log_lidar_reliability_max": float(arr.max()),
            "log_lidar_reliability_mean": float(arr.mean()),
        })
    return result


def make_comparison_image(metrics):
    panels = []
    for case_id, label in CASES:
        pgm = read_pgm(os.path.join(RUNS_DIR, case_id, "map.pgm"))
        img = Image.fromarray(pgm, mode="L").convert("RGB")
        arr = np.array(img.convert("L"))
        ys, xs = np.where(arr != 205)
        if len(xs) > 0:
            pad = 30
            img = img.crop((max(0, xs.min() - pad), max(0, ys.min() - pad),
                            min(arr.shape[1], xs.max() + pad),
                            min(arr.shape[0], ys.max() + pad)))
        resample_filter = getattr(Image, "Resampling", Image).LANCZOS
        img.thumbnail((520, 260), resample_filter)
        panel = Image.new("RGB", (540, 340), "white")
        panel.paste(img, ((540 - img.width) // 2, 52))
        draw = ImageDraw.Draw(panel)
        draw.text((12, 10), label, fill="black")
        draw.text((12, 288),
                  "area %.1fm2 comp %d lat %.2fm scale %.2f" %
                  (metrics[case_id]["map_area_m2"],
                   metrics[case_id]["occupied_components"],
                   metrics[case_id].get("median_lateral_spread_m", 0.0),
                   metrics[case_id].get("weight_scale_min", 1.0)),
                  fill="black")
        panels.append(panel)
    out = Image.new("RGB", (540 * len(panels), 340), "white")
    for i, panel in enumerate(panels):
        out.paste(panel, (i * 540, 0))
    out_path = os.path.join(EXP_ROOT, "innovation2_map_comparison.png")
    out.save(out_path)
    return out_path


def write_report(metrics, comparison_path):
    report_path = os.path.join(EXP_ROOT, "innovation2_four_way_report.md")
    p = metrics["slip_proposed_adaptive"]
    high = metrics["slip_static_high"]
    low = metrics["slip_static_low"]
    ref = metrics["reference_original_proposed"]
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("# 创新点二四组对比实验记录\n\n")
        f.write("## 实验设置\n\n")
        f.write("- Reference：原始 `corridor_01.bag`，使用 Proposed 配置，证明正常输入下地图本身应正常。\n")
        f.write("- 轻度 slip bag：`corridor_01_slip_injected.bag`，只对 `/odom` 注入轻度横向漂移和航向偏置，`/velodyne_points` 保持原始数据。\n")
        f.write("- Slip A：静态低后端 odometry 权重，关闭 slip-adaptive。\n")
        f.write("- Slip B：静态高后端 odometry 权重，关闭 slip-adaptive。\n")
        f.write("- Slip Proposed：高基础 odometry 权重，启用无 IMU 抗打滑 per-edge 动态降权。\n\n")
        f.write("## 关键结果\n\n")
        f.write("| 组别 | map面积(m2) | 占据连通分量 | PCA横向宽度P90(m) | 横向扩散中位数(m) | 最小权重scale | slip触发比例 | LiDAR可靠性均值 |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|---:|\n")
        for case_id, label in CASES:
            m = metrics[case_id]
            f.write("| %s | %.2f | %d | %.3f | %.3f | %.3f | %.3f | %.3f |\n" %
                    (label.replace("\n", " "),
                     m.get("map_area_m2", 0.0),
                     m.get("occupied_components", 0),
                     m.get("pca_lateral_width_p90_m", 0.0),
                     m.get("median_lateral_spread_m", 0.0),
                     m.get("weight_scale_min", 1.0),
                     m.get("slip_state_true_ratio", 0.0),
                     m.get("lidar_reliability_mean", 0.0)))
        f.write("\n## 触发证明\n\n")
        f.write("- Reference 原始 bag 地图面积：`%.2f m2`，用于确认正常输入下地图不应大幅歪斜。\n" % ref.get("map_area_m2", 0.0))
        f.write("- Slip Proposed `/slip_metric` 最大值：`%.4f`。\n" % p.get("slip_score_max", 0.0))
        f.write("- Slip Proposed `/odom_weight_scale` 最小值：`%.4f`。\n" % p.get("weight_scale_min", 1.0))
        f.write("- Slip Proposed `/slip_state=true` 采样比例：`%.4f`。\n" % p.get("slip_state_true_ratio", 0.0))
        f.write("- Slip Proposed `/slip_lidar_reliability` 均值：`%.4f`，P10：`%.4f`，P90：`%.4f`。\n" %
                (p.get("lidar_reliability_mean", 0.0),
                 p.get("lidar_reliability_p10", 0.0),
                 p.get("lidar_reliability_p90", 0.0)))
        f.write("- 后端日志 `[Innovation2] Wheel slip detected` 次数：`%d`。\n" % p.get("log_slip_trigger_count", 0))
        f.write("- 日志中 gated slip score 最大值：`%.4f`，触发段 LiDAR reliability 均值：`%.4f`。\n" %
                (p.get("log_gated_slip_score_max", 0.0),
                 p.get("log_lidar_reliability_mean", 0.0)))
        f.write("\n## 结论\n\n")
        f.write("Reference 组用于证明原始输入下 Proposed 配置能正常建图，不把正常地图跑歪。三组 slip 对比用于证明轻度 odom 异常下动态降权是否比静态高权重更稳。\n\n")
        f.write("相对 Slip B 静态高 odom 权重，Slip Proposed 若地图面积、横向宽度或连通分量下降，说明动态降权减轻了错误 odometry 对地图的牵引。第二版额外观察 `/slip_lidar_reliability` 和日志中的 gated score：只有可靠性门控通过后，slip score 才会真正进入迟滞判定并降低 odometry 权重。相对 Slip A 静态低 odom 权重，Slip Proposed 的目标不是所有指标绝对最小，而是在正常段保留高 odometry 约束，在打滑段自动降权。\n\n")
        f.write("本次 slip 是对 `/odom` 做轻度人工注入，不是真实地面打滑；它适合证明算法链路和鲁棒性趋势。当前只使用地图图像指标，没有外部真值轨迹，不能严格给出 ATE/RPE 级别结论。\n\n")
        f.write("对比图：`%s`\n" % comparison_path)
    return report_path


def main():
    metrics = {}
    for case_id, _ in CASES:
        metrics[case_id] = map_metrics(case_id)
        metrics[case_id].update(slip_metrics(case_id))
        metrics[case_id].update(log_trigger_metrics(case_id))
    comparison_path = make_comparison_image(metrics)
    out_json = os.path.join(EXP_ROOT, "innovation2_four_way_metrics.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    report_path = write_report(metrics, comparison_path)
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    print("comparison_image:", comparison_path)
    print("metrics_json:", out_json)
    print("report:", report_path)


if __name__ == "__main__":
    main()
