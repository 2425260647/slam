#!/usr/bin/env python3
"""Analyze Innovation 2 P0 runs against the matching clean-bag run.

The clean run is an internal reference, not external ground truth. Therefore
the trajectory statistics produced here must not be reported as ATE or RPE.
"""

import argparse
import csv
import json
import math
import os
import re

import numpy as np
import yaml
from PIL import Image
from scipy.spatial import cKDTree


WORKSPACE = "/home/slam/slam_ws"
P0_ROOT = os.path.join(WORKSPACE, "paper_exp/innovation2_p0_20260727")
CLEAN_ROOT = os.path.join(
    WORKSPACE, "paper_exp/formal_corridor_20260724/runs/normal_full/proposed"
)

# Fixed before formal evaluation from the repeat_03 clean map. The intervals
# retain long straight wall sections and omit the two large side openings.
CORRIDOR_X_INTERVALS = ((0.0, 28.0), (38.0, 58.0), (67.0, 77.0))
CORRIDOR_Y_MIN = -4.0
CORRIDOR_Y_MAX = 5.0
OCCUPIED_PIXEL_MAX = 100
MATCH_TOLERANCE_M = 0.15


def normalize_axis_angle_deg(angle):
    while angle >= 90.0:
        angle -= 180.0
    while angle < -90.0:
        angle += 180.0
    return angle


def axis_difference_deg(first, second):
    return abs(normalize_axis_angle_deg(first - second))


def load_map_points(run_dir):
    with open(os.path.join(run_dir, "map.yaml"), encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    pixels = np.asarray(Image.open(os.path.join(run_dir, "map.pgm")))
    rows, columns = np.where(pixels <= OCCUPIED_PIXEL_MAX)
    resolution = float(metadata["resolution"])
    origin_x, origin_y = map(float, metadata["origin"][:2])
    x = origin_x + (columns.astype(float) + 0.5) * resolution
    y = origin_y + (pixels.shape[0] - rows.astype(float) - 0.5) * resolution
    return np.column_stack((x, y))


def corridor_roi(points):
    longitudinal = np.zeros(points.shape[0], dtype=bool)
    for lower, upper in CORRIDOR_X_INTERVALS:
        longitudinal |= (points[:, 0] >= lower) & (points[:, 0] <= upper)
    lateral = (points[:, 1] >= CORRIDOR_Y_MIN) & (points[:, 1] <= CORRIDOR_Y_MAX)
    return points[longitudinal & lateral]


def pca_axis_deg(points):
    centered = points - np.mean(points, axis=0)
    _, eigenvectors = np.linalg.eigh(np.cov(centered.T))
    axis = eigenvectors[:, -1]
    return normalize_axis_angle_deg(math.degrees(math.atan2(axis[1], axis[0])))


def map_reference_metrics(reference_points, candidate_points):
    reference_roi = corridor_roi(reference_points)
    candidate_roi = corridor_roi(candidate_points)
    if min(reference_roi.shape[0], candidate_roi.shape[0]) == 0:
        raise RuntimeError("Fixed corridor ROI contains no occupied cells")

    reference_tree = cKDTree(reference_points)
    candidate_tree = cKDTree(candidate_points)
    candidate_to_reference = reference_tree.query(candidate_roi, k=1)[0]
    reference_to_candidate = candidate_tree.query(reference_roi, k=1)[0]
    distances = np.concatenate((candidate_to_reference, reference_to_candidate))

    precision = float(np.mean(candidate_to_reference <= MATCH_TOLERANCE_M))
    recall = float(np.mean(reference_to_candidate <= MATCH_TOLERANCE_M))
    f1 = 0.0 if precision + recall == 0.0 else 2.0 * precision * recall / (precision + recall)
    reference_axis = pca_axis_deg(reference_roi)
    candidate_axis = pca_axis_deg(candidate_roi)
    return {
        "reference_occupied_cells_roi": int(reference_roi.shape[0]),
        "candidate_occupied_cells_roi": int(candidate_roi.shape[0]),
        "occupied_cell_ratio_to_clean": float(
            candidate_roi.shape[0] / reference_roi.shape[0]
        ),
        "symmetric_chamfer_mean_m": float(np.mean(distances)),
        "symmetric_chamfer_p95_m": float(np.percentile(distances, 95.0)),
        "occupied_precision_at_0_15_m": precision,
        "occupied_recall_at_0_15_m": recall,
        "occupied_f1_at_0_15_m": f1,
        "clean_corridor_axis_deg": reference_axis,
        "candidate_corridor_axis_deg": candidate_axis,
        "corridor_axis_error_deg": axis_difference_deg(candidate_axis, reference_axis),
    }


def load_trajectory(path):
    stamps = []
    x = []
    y = []
    yaw = []
    with open(path, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            stamps.append(float(row["field.header.stamp"]) * 1e-9)
            x.append(float(row["field.pose.position.x"]))
            y.append(float(row["field.pose.position.y"]))
            z = float(row["field.pose.orientation.z"])
            w = float(row["field.pose.orientation.w"])
            yaw.append(2.0 * math.atan2(z, w))
    stamps = np.asarray(stamps)
    keep = np.concatenate(([True], np.diff(stamps) > 0.0))
    return (
        stamps[keep],
        np.asarray(x)[keep],
        np.asarray(y)[keep],
        np.unwrap(np.asarray(yaw)[keep]),
    )


def trajectory_reference_metrics(reference_path, candidate_path):
    reference = load_trajectory(reference_path)
    candidate = load_trajectory(candidate_path)
    start = max(reference[0][0], candidate[0][0])
    end = min(reference[0][-1], candidate[0][-1])
    sample_times = np.arange(start, end, 0.1)
    reference_x = np.interp(sample_times, reference[0], reference[1])
    reference_y = np.interp(sample_times, reference[0], reference[2])
    reference_yaw = np.interp(sample_times, reference[0], reference[3])
    candidate_x = np.interp(sample_times, candidate[0], candidate[1])
    candidate_y = np.interp(sample_times, candidate[0], candidate[2])
    candidate_yaw = np.interp(sample_times, candidate[0], candidate[3])
    position_error = np.hypot(candidate_x - reference_x, candidate_y - reference_y)
    yaw_error = np.abs(
        np.arctan2(
            np.sin(candidate_yaw - reference_yaw),
            np.cos(candidate_yaw - reference_yaw),
        )
    )
    endpoint_delta = math.hypot(
        candidate_x[-1] - reference_x[-1], candidate_y[-1] - reference_y[-1]
    )
    return {
        "reference_kind": "same-bag clean-run proxy; not ground truth",
        "sample_period_sec": 0.1,
        "sample_count": int(sample_times.size),
        "position_difference_mean_m": float(np.mean(position_error)),
        "position_difference_p95_m": float(np.percentile(position_error, 95.0)),
        "position_difference_max_m": float(np.max(position_error)),
        "yaw_difference_mean_deg": float(np.degrees(np.mean(yaw_error))),
        "yaw_difference_p95_deg": float(np.degrees(np.percentile(yaw_error, 95.0))),
        "endpoint_position_difference_m": endpoint_delta,
        "endpoint_yaw_difference_deg": float(np.degrees(yaw_error[-1])),
    }


def load_scalar_csv(path):
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        return np.asarray([])
    values = []
    with open(path, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            values.append(float(row["field.data"]))
    return np.asarray(values)


def diagnostic_metrics(run_dir):
    result = {}
    files = {
        "consistency_metric": "consistency_metric.csv",
        "weight_scale": "odom_weight_scale.csv",
        "minimum_weight_scale": "odom_min_weight_scale.csv",
        "trigger_count": "trigger_count.csv",
        "lidar_reliability": "lidar_reliability.csv",
        "anomaly_time": "anomaly_time.csv",
    }
    for name, filename in files.items():
        values = load_scalar_csv(os.path.join(run_dir, filename))
        if values.size == 0:
            continue
        result[name + "_min"] = float(np.min(values))
        result[name + "_max"] = float(np.max(values))
        result[name + "_mean"] = float(np.mean(values))
        if name == "anomaly_time":
            result["distinct_latest_anomaly_times"] = [
                float(value) for value in np.unique(np.round(values, decimals=6))
            ]

    log_path = os.path.join(run_dir, "roslaunch.log")
    anomaly_nodes = set()
    if os.path.isfile(log_path):
        pattern = re.compile(r"Consistency anomaly detected at NodeId\((\d+), (\d+)\)")
        with open(log_path, encoding="utf-8", errors="replace") as stream:
            for line in stream:
                match = pattern.search(line)
                if match:
                    anomaly_nodes.add((int(match.group(1)), int(match.group(2))))
    result["logged_unique_anomaly_node_ids"] = [list(value) for value in sorted(anomaly_nodes)]
    return result


def metadata_value(path, key):
    if not os.path.isfile(path):
        return None
    prefix = key + "="
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if line.startswith(prefix):
                return line[len(prefix) :].strip()
    return None


def methods_for_suite(suite):
    if suite == "map_protection_v2":
        return ("static_low", "static_high", "proposed_gate")
    if suite == "gate_ablation":
        return ("gate_off", "gate_on")
    raise ValueError("Unsupported suite: {}".format(suite))


def write_markdown(report, path):
    lines = [
        "# 创新点二 P0 实验机器可复核摘要",
        "",
        "> 注意：clean reference 是同一 bag 的正常算法运行，不是外部真值。",
        "> 下列轨迹差异只能称为干净参考偏差，不能称为 ATE 或 RPE。",
        "",
        "- 实验组：`{}`".format(report["suite"]),
        "- Bag：`corridor_repeat_{:02d}`".format(report["repeat"]),
        "- 固定走廊 ROI：x=`{}` m，y=`[{:.1f}, {:.1f}]` m".format(
            list(CORRIDOR_X_INTERVALS), CORRIDOR_Y_MIN, CORRIDOR_Y_MAX
        ),
        "- 墙体匹配容差：`{:.2f} m`".format(MATCH_TOLERANCE_M),
        "",
        "| 方法 | Chamfer均值/m | Chamfer P95/m | 墙体F1 | 主轴角误差/deg | 轨迹P95偏差/m | 最小权重 | 上升沿数 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for method, data in report["methods"].items():
        map_data = data["map_reference"]
        trajectory = data["trajectory_reference"]
        diagnostics = data["diagnostics"]
        lines.append(
            "| {} | {:.4f} | {:.4f} | {:.4f} | {:.4f} | {:.4f} | {} | {} |".format(
                method,
                map_data["symmetric_chamfer_mean_m"],
                map_data["symmetric_chamfer_p95_m"],
                map_data["occupied_f1_at_0_15_m"],
                map_data["corridor_axis_error_deg"],
                trajectory["position_difference_p95_m"],
                "{:.3f}".format(diagnostics["weight_scale_min"])
                if "weight_scale_min" in diagnostics
                else "N/A",
                "{:.0f}".format(diagnostics["trigger_count_max"])
                if "trigger_count_max" in diagnostics
                else "N/A",
            )
        )
    lines.extend(
        [
            "",
            "## 证据边界",
            "",
            "- 地图指标衡量相对干净运行的结构保持程度，不代表绝对地图精度。",
            "- 人工注入是可控 odometry 一致性压力，不等同于真实轮胎打滑采样。",
            "- 迟滞状态机会把相邻注入段合并为一个异常事件，上升沿数不等于注入段数。",
            "",
        ]
    )
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", choices=("map_protection_v2", "gate_ablation"), required=True)
    parser.add_argument("--repeat", type=int, choices=(1, 2, 3), required=True)
    args = parser.parse_args()

    clean_dir = os.path.join(CLEAN_ROOT, "repeat_{:02d}".format(args.repeat))
    suite_dir = os.path.join(P0_ROOT, "runs", args.suite)
    reference_points = load_map_points(clean_dir)
    report = {
        "schema_version": 1,
        "suite": args.suite,
        "repeat": args.repeat,
        "reference": clean_dir,
        "reference_limit": "same-bag clean run, not external ground truth",
        "fixed_roi": {
            "x_intervals_m": list(CORRIDOR_X_INTERVALS),
            "y_min_m": CORRIDOR_Y_MIN,
            "y_max_m": CORRIDOR_Y_MAX,
            "match_tolerance_m": MATCH_TOLERANCE_M,
        },
        "methods": {},
    }
    for method in methods_for_suite(args.suite):
        run_dir = os.path.join(suite_dir, method, "repeat_{:02d}".format(args.repeat))
        if not os.path.isfile(os.path.join(run_dir, "SUCCESS")):
            raise RuntimeError("Run is incomplete: {}".format(run_dir))
        report["methods"][method] = {
            "run_dir": run_dir,
            "bag_sha256": metadata_value(os.path.join(run_dir, "metadata.txt"), "bag_sha256"),
            "config_sha256": metadata_value(
                os.path.join(run_dir, "metadata.txt"), "config_sha256"
            ),
            "map_reference": map_reference_metrics(
                reference_points, load_map_points(run_dir)
            ),
            "trajectory_reference": trajectory_reference_metrics(
                os.path.join(clean_dir, "tracked_pose.csv"),
                os.path.join(run_dir, "tracked_pose.csv"),
            ),
            "diagnostics": diagnostic_metrics(run_dir),
        }

    basename = "{}_repeat_{:02d}_metrics".format(args.suite, args.repeat)
    json_path = os.path.join(P0_ROOT, basename + ".json")
    markdown_path = os.path.join(P0_ROOT, basename + ".md")
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    write_markdown(report, markdown_path)
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
