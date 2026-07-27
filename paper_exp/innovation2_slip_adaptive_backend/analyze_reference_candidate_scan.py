#!/usr/bin/env python3
import csv
import json
import os
import re

import numpy as np


EXP_ROOT = "/home/slam/slam_ws/paper_exp/innovation2_slip_adaptive_backend"
RUNS_DIR = os.path.join(EXP_ROOT, "reference_candidate_runs")
OUT_JSON = os.path.join(EXP_ROOT, "innovation2_reference_candidate_metrics.json")
OUT_REPORT = os.path.join(EXP_ROOT, "innovation2_reference_candidate_report.md")


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


def read_map_area(run_dir):
    yaml_path = os.path.join(run_dir, "map.yaml")
    pgm_path = os.path.join(run_dir, "map.pgm")
    if not os.path.exists(yaml_path) or not os.path.exists(pgm_path):
        return 0.0
    resolution = None
    with open(yaml_path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1].strip())
                break
    if resolution is None:
        return 0.0
    with open(pgm_path, "rb") as f:
        if f.readline().strip() != b"P5":
            return 0.0
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = map(int, line.split())
    return float(width * height * resolution * resolution)


def log_trigger_count(run_dir):
    path = os.path.join(run_dir, "roslaunch.log")
    if not os.path.exists(path):
        return 0
    pattern = re.compile(r"Wheel slip detected")
    count = 0
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if pattern.search(line):
                count += 1
    return count


def case_metrics(case_id):
    run_dir = os.path.join(RUNS_DIR, case_id)
    scores = parse_float_csv(os.path.join(run_dir, "slip_metric.csv"),
                             "field.data")
    scales = parse_float_csv(os.path.join(run_dir, "odom_weight_scale.csv"),
                             "field.data")
    states = parse_bool_csv(os.path.join(run_dir, "slip_state.csv"),
                            "field.data")
    reliability = parse_float_csv(
        os.path.join(run_dir, "slip_lidar_reliability.csv"), "field.data")
    result = {
        "map_area_m2": read_map_area(run_dir),
        "log_slip_trigger_count": log_trigger_count(run_dir),
    }
    if scores:
        arr = np.asarray(scores)
        result.update({
            "slip_score_max": float(arr.max()),
            "slip_score_mean": float(arr.mean()),
            "slip_score_p90": float(np.percentile(arr, 90)),
        })
    if scales:
        arr = np.asarray(scales)
        result.update({
            "weight_scale_min": float(arr.min()),
            "weight_scale_mean": float(arr.mean()),
        })
    if states:
        result.update({
            "slip_state_true_count": int(sum(states)),
            "slip_state_true_ratio": float(sum(states) / len(states)),
        })
    if reliability:
        arr = np.asarray(reliability)
        result.update({
            "lidar_reliability_mean": float(arr.mean()),
            "lidar_reliability_p90": float(np.percentile(arr, 90)),
        })
    return result


def main():
    metrics = {}
    for case_id in sorted(os.listdir(RUNS_DIR)):
        run_dir = os.path.join(RUNS_DIR, case_id)
        if os.path.isdir(run_dir):
            metrics[case_id] = case_metrics(case_id)
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    with open(OUT_REPORT, "w", encoding="utf-8") as f:
        f.write("# 创新点二候选参数 Reference 复验\n\n")
        f.write("| case | map面积 | slip最大值 | 最小scale | slip比例 | 日志触发 | reliability均值/P90 |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|\n")
        for case_id, m in sorted(metrics.items()):
            f.write("| %s | %.2f | %.4f | %.3f | %.4f | %d | %.3f / %.3f |\n" %
                    (case_id, m.get("map_area_m2", 0.0),
                     m.get("slip_score_max", 0.0),
                     m.get("weight_scale_min", 1.0),
                     m.get("slip_state_true_ratio", 0.0),
                     m.get("log_slip_trigger_count", 0),
                     m.get("lidar_reliability_mean", 0.0),
                     m.get("lidar_reliability_p90", 0.0)))
        f.write("\nReference 复验的通过标准：`slip_state_true_ratio=0`、`weight_scale_min=1.0`、日志触发为 0。\n")
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    print("metrics_json:", OUT_JSON)
    print("report:", OUT_REPORT)


if __name__ == "__main__":
    main()
