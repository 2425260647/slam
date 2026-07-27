#!/usr/bin/env python3
import csv
import json
import os
import re
from collections import deque

import numpy as np


EXP_ROOT = "/home/slam/slam_ws/paper_exp/innovation2_slip_adaptive_backend"
RUNS_DIR = os.path.join(EXP_ROOT, "parameter_scan_runs")
OUT_JSON = os.path.join(EXP_ROOT, "innovation2_parameter_scan_metrics.json")
OUT_REPORT = os.path.join(EXP_ROOT, "innovation2_parameter_scan_report.md")


def read_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise RuntimeError("Only P5 PGM is supported: %s" % path)
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
    with open(yaml_path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("resolution:"):
                return float(line.split(":", 1)[1].strip())
    raise RuntimeError("Missing resolution in %s" % yaml_path)


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
                    if (0 <= yy < h and 0 <= xx < w and mask[yy, xx]
                            and not seen[yy, xx]):
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
    for lo, hi in zip(bins[:-1], bins[1:]):
        selected = lat[(long >= lo) & (long < hi)]
        if len(selected) >= 20:
            widths.append(
                float(np.percentile(selected, 90) -
                      np.percentile(selected, 10)))
    return {
        "pca_lateral_width_p90_m":
            float(np.percentile(lat, 95) - np.percentile(lat, 5)),
        "median_lateral_spread_m":
            float(np.median(widths)) if widths else 0.0,
        "mean_lateral_spread_m":
            float(np.mean(widths)) if widths else 0.0,
    }


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


def log_metrics(run_dir):
    path = os.path.join(run_dir, "roslaunch.log")
    count = 0
    scores = []
    gated = []
    reliabilities = []
    pattern = re.compile(
        r"Wheel slip detected\. score=([0-9.eE+-]+), gated_score=([0-9.eE+-]+).*lidar_reliability=([0-9.eE+-]+)"
    )
    if not os.path.exists(path):
        return {"log_slip_trigger_count": 0}
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                count += 1
                scores.append(float(match.group(1)))
                gated.append(float(match.group(2)))
                reliabilities.append(float(match.group(3)))
    result = {"log_slip_trigger_count": count}
    if scores:
        result.update({
            "log_slip_score_max": float(np.max(scores)),
            "log_gated_slip_score_max": float(np.max(gated)),
            "log_lidar_reliability_mean": float(np.mean(reliabilities)),
        })
    return result


def config_params(case_id):
    match = re.match(r"scan_r(\d+)_h(\d+)", case_id)
    if not match:
        return {}
    reliability = int(match.group(1)) / 100.0
    high = int(match.group(2)) / 1000.0
    return {
        "slip_lidar_reliability_min": reliability,
        "slip_high_threshold": high,
        "slip_low_threshold": high * 0.4,
    }


def case_metrics(case_id):
    run_dir = os.path.join(RUNS_DIR, case_id)
    pgm = read_pgm(os.path.join(run_dir, "map.pgm"))
    resolution = read_resolution(os.path.join(run_dir, "map.yaml"))
    occupied = pgm == 0
    free = pgm == 254
    known = occupied | free
    components = connected_components(occupied)
    metrics = {
        "map_area_m2":
            float(pgm.shape[0] * pgm.shape[1] * resolution * resolution),
        "occupied_cells":
            int(occupied.sum()),
        "known_cells":
            int(known.sum()),
        "occupied_components":
            int(len(components)),
        "largest_component_ratio":
            float((components[0] if components else 0) / max(1, occupied.sum())),
    }
    metrics.update(pca_shape_metrics(occupied, resolution))

    scores = parse_float_csv(os.path.join(run_dir, "slip_metric.csv"),
                             "field.data")
    scales = parse_float_csv(os.path.join(run_dir, "odom_weight_scale.csv"),
                             "field.data")
    states = parse_bool_csv(os.path.join(run_dir, "slip_state.csv"),
                            "field.data")
    reliability = parse_float_csv(
        os.path.join(run_dir, "slip_lidar_reliability.csv"), "field.data")
    if scores:
        arr = np.asarray(scores)
        metrics.update({
            "slip_score_max": float(arr.max()),
            "slip_score_mean": float(arr.mean()),
            "slip_score_p90": float(np.percentile(arr, 90)),
        })
    if scales:
        arr = np.asarray(scales)
        metrics.update({
            "weight_scale_min": float(arr.min()),
            "weight_scale_mean": float(arr.mean()),
            "weight_scale_p10": float(np.percentile(arr, 10)),
        })
    if states:
        metrics.update({
            "slip_state_true_count": int(sum(states)),
            "slip_state_true_ratio": float(sum(states) / len(states)),
        })
    if reliability:
        arr = np.asarray(reliability)
        metrics.update({
            "lidar_reliability_mean": float(arr.mean()),
            "lidar_reliability_p10": float(np.percentile(arr, 10)),
            "lidar_reliability_p90": float(np.percentile(arr, 90)),
        })
    metrics.update(log_metrics(run_dir))
    metrics.update(config_params(case_id))
    return metrics


def score_candidate(m):
    # [Innovation 2] Ranking score for a thesis-friendly parameter: reduce
    # map area and fragmentation, trigger non-trivially, but avoid making
    # lateral width much worse. Lower is better.
    trigger_count = m.get("log_slip_trigger_count", 0)
    trigger_bonus = min(0.05, m.get("slip_state_true_ratio", 0.0)) * 800.0
    trigger_bonus += min(10, trigger_count) * 2.0
    no_effect_penalty = 250.0 if trigger_count == 0 and m.get(
        "weight_scale_min", 1.0) >= 0.999 else 0.0
    over_trigger_penalty = 300.0 if m.get("slip_state_true_ratio",
                                          0.0) > 0.05 else 0.0
    return (m["map_area_m2"] + 0.8 * m["occupied_components"] +
            20.0 * m.get("pca_lateral_width_p90_m", 0.0) -
            trigger_bonus + no_effect_penalty + over_trigger_penalty)


def write_report(metrics):
    ordered = sorted(metrics.items(), key=lambda item: item[1]["rank_score"])
    with open(OUT_REPORT, "w", encoding="utf-8") as f:
        f.write("# 创新点二参数扫描实验\n\n")
        f.write("本轮只在轻度 slip bag 上扫描 6 组参数，用于寻找更适合后续 Reference 复验的候选配置。\n\n")
        f.write("| 排名 | case | reliability_min | high/low | map面积 | 连通分量 | 横向宽度P90 | 最小scale | slip比例 | 日志触发 | rank_score |\n")
        f.write("|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for rank, (case_id, m) in enumerate(ordered, 1):
            f.write("| %d | %s | %.2f | %.3f/%.3f | %.2f | %d | %.3f | %.3f | %.4f | %d | %.2f |\n" %
                    (rank, case_id,
                     m.get("slip_lidar_reliability_min", 0.0),
                     m.get("slip_high_threshold", 0.0),
                     m.get("slip_low_threshold", 0.0),
                     m.get("map_area_m2", 0.0),
                     m.get("occupied_components", 0),
                     m.get("pca_lateral_width_p90_m", 0.0),
                     m.get("weight_scale_min", 1.0),
                     m.get("slip_state_true_ratio", 0.0),
                     m.get("log_slip_trigger_count", 0),
                     m.get("rank_score", 0.0)))
        f.write("\n## 建议\n\n")
        best_id, best = ordered[0]
        f.write("当前排序第一候选为 `%s`：`slip_lidar_reliability_min=%.2f`，`slip_high_threshold=%.3f`，`slip_low_threshold=%.3f`。\n\n" %
                (best_id, best.get("slip_lidar_reliability_min", 0.0),
                 best.get("slip_high_threshold", 0.0),
                 best.get("slip_low_threshold", 0.0)))
        f.write("下一步应对排名前 2 的配置跑 Reference 原始 bag，确认正常场景仍无误触发，再决定最终 Proposed 参数。\n")
    return ordered


def main():
    metrics = {}
    for case_id in sorted(os.listdir(RUNS_DIR)):
        run_dir = os.path.join(RUNS_DIR, case_id)
        if not os.path.isdir(run_dir):
            continue
        if not os.path.exists(os.path.join(run_dir, "map.pgm")):
            continue
        metrics[case_id] = case_metrics(case_id)
        metrics[case_id]["rank_score"] = score_candidate(metrics[case_id])
    ordered = write_report(metrics)
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    print("best:", ordered[0][0] if ordered else "none")
    print("metrics_json:", OUT_JSON)
    print("report:", OUT_REPORT)


if __name__ == "__main__":
    main()
