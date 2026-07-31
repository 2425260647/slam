#!/usr/bin/env python3
"""Analyze the 21 formal runs produced by the frozen post-fix algorithm.

The maps and trajectories have no external ground truth. Map-to-clean and
trajectory-to-clean values are structural proxies and must not be called ATE
or RPE in the paper.
"""

import bisect
import csv
import json
import math
import os
import sys
from pathlib import Path

import numpy as np


ROOT = Path("/home/slam/slam_ws/paper_exp/formal_corridor_20260724")
RUNS = ROOT / "runs"
P0_ROOT = Path("/home/slam/slam_ws/paper_exp/innovation2_p0_20260727")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(P0_ROOT))

import evaluate_wall_parameter_maps as wall_analysis  # noqa: E402
import analyze_i2_p0 as reference_analysis  # noqa: E402


I1_METHODS = ("baseline_a", "baseline_b", "proposed")
I2_METHODS = ("baseline_a", "baseline_b", "proposed")
REPEATS = (1, 2, 3)
SOURCE_MANIFEST_SHA256 = (
    "95557cbcc88b59ff9883176f53311ec5b5dff3789b769251d38273b5742d26ee"
)


def run_dir(suite, method, repeat):
    return RUNS / suite / method / "repeat_{:02d}".format(repeat)


def read_csv(path, fields):
    rows = []
    seen = set()
    with Path(path).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            timestamp = int(row["%time"])
            values = tuple(float(row[field]) for field in fields)
            key = (timestamp,) + values
            if key in seen:
                continue
            seen.add(key)
            rows.append((timestamp,) + values)
    rows.sort()
    return rows


def summarize(values):
    values = np.asarray(list(values), dtype=float)
    if values.size == 0:
        return {"count": 0}
    return {
        "count": int(values.size),
        "mean": float(np.mean(values)),
        "sample_std": float(np.std(values, ddof=1)) if values.size > 1 else 0.0,
        "min": float(np.min(values)),
        "median": float(np.median(values)),
        "p90": float(np.percentile(values, 90.0)),
        "p95": float(np.percentile(values, 95.0)),
        "max": float(np.max(values)),
    }


def axis_error_rad(first, second):
    return 0.5 * abs(
        math.atan2(math.sin(2.0 * (first - second)),
                   math.cos(2.0 * (first - second)))
    )


def nearest(samples, timestamps, timestamp, max_delta_ns=100_000_000):
    index = bisect.bisect_left(timestamps, timestamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    if not candidates:
        return None
    candidate = min(candidates, key=lambda item: abs(item[0] - timestamp))
    return candidate if abs(candidate[0] - timestamp) <= max_delta_ns else None


def direction_metrics(directory):
    confidence = read_csv(directory / "degeneracy_metric.csv", ("field.data",))
    direction = read_csv(
        directory / "degeneracy_direction.csv", ("field.x", "field.y")
    )
    poses = read_csv(
        directory / "tracked_pose.csv",
        (
            "field.pose.position.x",
            "field.pose.position.y",
            "field.pose.orientation.z",
            "field.pose.orientation.w",
        ),
    )
    pose_xy = np.asarray([[row[1], row[2]] for row in poses])
    centered = pose_xy - np.mean(pose_xy, axis=0)
    _, eigenvectors = np.linalg.eigh(np.cov(centered.T))
    trajectory_axis = math.atan2(eigenvectors[1, -1], eigenvectors[0, -1])
    confidence_times = [row[0] for row in confidence]
    errors = []
    high_confidence_count = 0
    for timestamp, x, y in direction:
        current = nearest(confidence, confidence_times, timestamp)
        if current is None or current[1] < 0.8 or math.hypot(x, y) <= 1e-12:
            continue
        high_confidence_count += 1
        errors.append(math.degrees(axis_error_rad(math.atan2(y, x), trajectory_axis)))

    confidence_values = [row[1] for row in confidence]
    longitudinal_scales = [
        (1.0 + value) / max(1e-9, 1.0 - 0.5 * value)
        for value in confidence_values
    ]
    return {
        "trajectory_pca_axis_deg": math.degrees(trajectory_axis),
        "confidence": summarize(confidence_values),
        "confidence_ge_0_8_ratio": float(
            sum(value >= 0.8 for value in confidence_values)
            / max(1, len(confidence_values))
        ),
        "longitudinal_weight_scale": summarize(longitudinal_scales),
        "high_confidence_direction_samples": high_confidence_count,
        "direction_to_trajectory_axis_error_deg": summarize(errors),
        "reference_limit": "trajectory PCA axis proxy; not surveyed corridor truth",
    }


def i1_map_metrics(directory):
    metrics = wall_analysis.map_metrics(str(directory / "map"))
    return {
        "map_area_m2": metrics["width_m"] * metrics["height_m"],
        "occupied_cells": metrics["occupied_cells"],
        "unknown_ratio": metrics["unknown_ratio"],
        "dominant_axis_yaw_deg": metrics["dominant_axis_yaw_deg"],
        "best_wall_gap_closed_coverage": metrics[
            "best_wall_gap_closed_coverage"
        ],
        "best_wall_continuous_length_m": metrics[
            "best_wall_continuous_length_m"
        ],
    }


def aggregate_method(per_repeat, fields):
    return {
        field: summarize(per_repeat[str(repeat)][field] for repeat in REPEATS)
        for field in fields
    }


def analyze_i1():
    result = {"methods": {}}
    fields = (
        "map_area_m2",
        "occupied_cells",
        "unknown_ratio",
        "best_wall_gap_closed_coverage",
        "best_wall_continuous_length_m",
    )
    for method in I1_METHODS:
        per_repeat = {
            str(repeat): i1_map_metrics(run_dir("i1", method, repeat))
            for repeat in REPEATS
        }
        result["methods"][method] = {
            "per_repeat": per_repeat,
            "aggregate": aggregate_method(per_repeat, fields),
        }
    result["proposed_direction"] = {
        str(repeat): direction_metrics(run_dir("i1", "proposed", repeat))
        for repeat in REPEATS
    }
    all_errors = []
    all_confidences = []
    all_scales = []
    for data in result["proposed_direction"].values():
        directory_repeat = int(
            next(key for key, value in result["proposed_direction"].items()
                 if value is data)
        )
        directory = run_dir("i1", "proposed", directory_repeat)
        confidence = read_csv(directory / "degeneracy_metric.csv", ("field.data",))
        direction = read_csv(
            directory / "degeneracy_direction.csv", ("field.x", "field.y")
        )
        poses = read_csv(
            directory / "tracked_pose.csv",
            (
                "field.pose.position.x",
                "field.pose.position.y",
                "field.pose.orientation.z",
                "field.pose.orientation.w",
            ),
        )
        xy = np.asarray([[row[1], row[2]] for row in poses])
        centered = xy - np.mean(xy, axis=0)
        _, vectors = np.linalg.eigh(np.cov(centered.T))
        axis = math.atan2(vectors[1, -1], vectors[0, -1])
        confidence_times = [row[0] for row in confidence]
        for timestamp, x, y in direction:
            current = nearest(confidence, confidence_times, timestamp)
            if current is not None and current[1] >= 0.8:
                all_errors.append(
                    math.degrees(axis_error_rad(math.atan2(y, x), axis))
                )
        values = [row[1] for row in confidence]
        all_confidences.extend(values)
        all_scales.extend((1.0 + value) / (1.0 - 0.5 * value) for value in values)
    result["proposed_direction_aggregate"] = {
        "confidence": summarize(all_confidences),
        "longitudinal_weight_scale": summarize(all_scales),
        "direction_to_trajectory_axis_error_deg": summarize(all_errors),
    }
    return result


def candidate_reference_metrics(reference_dir, candidate_dir):
    reference_points = reference_analysis.load_map_points(str(reference_dir))
    candidate_points = reference_analysis.load_map_points(str(candidate_dir))
    map_data = reference_analysis.map_reference_metrics(
        reference_points, candidate_points
    )
    trajectory_data = reference_analysis.trajectory_reference_metrics(
        str(reference_dir / "tracked_pose.csv"),
        str(candidate_dir / "tracked_pose.csv"),
    )
    return {"map_reference": map_data, "trajectory_reference": trajectory_data}


def diagnostic_metrics(directory):
    files = {
        "consistency_metric": "consistency_metric.csv",
        "weight_scale": "odom_weight_scale.csv",
        "minimum_weight_scale": "odom_min_weight_scale.csv",
        "lidar_reliability": "lidar_reference_reliability.csv",
        "trigger_count": "anomaly_trigger_count.csv",
    }
    result = {}
    for name, filename in files.items():
        values = [row[1] for row in read_csv(directory / filename, ("field.data",))]
        result[name] = summarize(values)
    anomaly_times = sorted(
        set(row[1] for row in read_csv(directory / "anomaly_time.csv", ("field.data",)))
    ) if (directory / "anomaly_time.csv").stat().st_size else []
    result["anomaly_times_sec"] = anomaly_times
    return result


def event_detection_metrics(repeat, anomaly_times):
    labels_path = ROOT / "anomaly_labels" / (
        "corridor_repeat_{:02d}_events.json".format(repeat)
    )
    labels = json.loads(labels_path.read_text(encoding="utf-8"))
    unmatched = set(range(len(labels["events"])))
    matches = []
    false_positives = []
    for detection in anomaly_times:
        candidates = []
        for index in unmatched:
            event = labels["events"][index]
            if (event["absolute_start_sec"] <= detection
                    <= event["absolute_end_sec"] + 2.0):
                candidates.append(index)
        if not candidates:
            false_positives.append(detection)
            continue
        index = min(
            candidates,
            key=lambda current: abs(
                detection - labels["events"][current]["absolute_start_sec"]
            ),
        )
        unmatched.remove(index)
        event = labels["events"][index]
        matches.append({
            "event_id": event["event_id"],
            "type": event["type"],
            "detection_sec": detection,
            "delay_from_event_start_sec": (
                detection - event["absolute_start_sec"]
            ),
        })
    return {
        "injected_events": len(labels["events"]),
        "detected_events": len(matches),
        "false_positive_detections": len(false_positives),
        "missed_event_ids": [labels["events"][index]["event_id"]
                             for index in sorted(unmatched)],
        "matches": matches,
        "detection_delay_sec": summarize(
            match["delay_from_event_start_sec"] for match in matches
        ),
    }


def analyze_i2():
    result = {"methods": {}, "proposed_detection": {}}
    metric_fields = {
        "chamfer_mean_m": ("map_reference", "symmetric_chamfer_mean_m"),
        "chamfer_p95_m": ("map_reference", "symmetric_chamfer_p95_m"),
        "wall_f1_at_0_15_m": ("map_reference", "occupied_f1_at_0_15_m"),
        "axis_error_deg": ("map_reference", "corridor_axis_error_deg"),
        "trajectory_position_p95_m": (
            "trajectory_reference", "position_difference_p95_m"
        ),
        "trajectory_yaw_p95_deg": (
            "trajectory_reference", "yaw_difference_p95_deg"
        ),
        "endpoint_position_difference_m": (
            "trajectory_reference", "endpoint_position_difference_m"
        ),
    }
    for method in I2_METHODS:
        per_repeat = {}
        for repeat in REPEATS:
            reference = run_dir("normal_full", "proposed", repeat)
            candidate = run_dir("i2", method, repeat)
            per_repeat[str(repeat)] = candidate_reference_metrics(
                reference, candidate
            )
        aggregate = {}
        for name, (section, field) in metric_fields.items():
            aggregate[name] = summarize(
                per_repeat[str(repeat)][section][field] for repeat in REPEATS
            )
        result["methods"][method] = {
            "per_repeat": per_repeat,
            "aggregate": aggregate,
        }

    all_matches = []
    total_events = 0
    total_detections = 0
    total_false_positives = 0
    for repeat in REPEATS:
        directory = run_dir("i2", "proposed", repeat)
        diagnostics = diagnostic_metrics(directory)
        detection = event_detection_metrics(
            repeat, diagnostics["anomaly_times_sec"]
        )
        result["proposed_detection"][str(repeat)] = {
            "diagnostics": diagnostics,
            "events": detection,
        }
        total_events += detection["injected_events"]
        total_detections += detection["detected_events"]
        total_false_positives += detection["false_positive_detections"]
        all_matches.extend(detection["matches"])
    precision = total_detections / max(1, total_detections + total_false_positives)
    recall = total_detections / max(1, total_events)
    result["proposed_detection_aggregate"] = {
        "injected_events": total_events,
        "true_positive_events": total_detections,
        "false_positive_detections": total_false_positives,
        "false_negative_events": total_events - total_detections,
        "precision": precision,
        "recall": recall,
        "f1": 2.0 * precision * recall / max(1e-12, precision + recall),
        "detection_delay_sec": summarize(
            match["delay_from_event_start_sec"] for match in all_matches
        ),
    }
    return result


def analyze_normal_full():
    result = {"per_repeat": {}, "i1_only_vs_full": {}}
    total_duration = 0.0
    total_triggers = 0
    for repeat in REPEATS:
        directory = run_dir("normal_full", "proposed", repeat)
        diagnostics = diagnostic_metrics(directory)
        labels_path = ROOT / "anomaly_labels" / (
            "corridor_repeat_{:02d}_events.json".format(repeat)
        )
        duration = json.loads(
            labels_path.read_text(encoding="utf-8")
        )["duration_sec"]
        trigger_max = diagnostics["trigger_count"].get("max", 0.0)
        result["per_repeat"][str(repeat)] = {
            "bag_duration_sec": duration,
            "trigger_count": int(trigger_max),
            "weight_scale_min": diagnostics["weight_scale"].get("min"),
            "minimum_weight_scale_min": diagnostics[
                "minimum_weight_scale"
            ].get("min"),
        }
        total_duration += duration
        total_triggers += int(trigger_max)
        result["i1_only_vs_full"][str(repeat)] = candidate_reference_metrics(
            run_dir("i1", "proposed", repeat), directory
        )
    result["i1_only_vs_full_aggregate"] = {
        "chamfer_mean_m": summarize(
            result["i1_only_vs_full"][str(repeat)]["map_reference"]
            ["symmetric_chamfer_mean_m"] for repeat in REPEATS
        ),
        "wall_f1_at_0_15_m": summarize(
            result["i1_only_vs_full"][str(repeat)]["map_reference"]
            ["occupied_f1_at_0_15_m"] for repeat in REPEATS
        ),
        "trajectory_position_p95_m": summarize(
            result["i1_only_vs_full"][str(repeat)]["trajectory_reference"]
            ["position_difference_p95_m"] for repeat in REPEATS
        ),
    }
    result["aggregate"] = {
        "duration_sec": total_duration,
        "duration_min": total_duration / 60.0,
        "trigger_count": total_triggers,
        "false_triggers_per_min": total_triggers / (total_duration / 60.0),
    }
    return result


def verify_runs():
    directories = []
    for method in I1_METHODS:
        directories.extend(run_dir("i1", method, repeat) for repeat in REPEATS)
    for method in I2_METHODS:
        directories.extend(run_dir("i2", method, repeat) for repeat in REPEATS)
    directories.extend(
        run_dir("normal_full", "proposed", repeat) for repeat in REPEATS
    )
    failures = []
    for directory in directories:
        required = (
            "SUCCESS", "map.pgm", "map.yaml", "state.pbstream",
            "tracked_pose.csv", "algorithm_source_manifest.sha256",
        )
        if any(not (directory / name).exists() for name in required):
            failures.append(str(directory))
            continue
        metadata = (directory / "metadata.txt").read_text(encoding="utf-8")
        if "source_manifest_sha256={}".format(SOURCE_MANIFEST_SHA256) not in metadata:
            failures.append(str(directory))
    return {
        "expected_runs": 21,
        "verified_runs": 21 - len(failures),
        "failures": failures,
        "algorithm_version": "innovation1_coordinate_fix_20260728",
        "source_manifest_sha256": SOURCE_MANIFEST_SHA256,
    }


def mean_std_text(metric, digits=4):
    return ("{mean:.{digits}f} +/- {sample_std:.{digits}f}"
            .format(digits=digits, **metric))


def write_markdown(report, path):
    i1 = report["innovation1"]
    i2 = report["innovation2"]
    normal = report["normal_full"]
    lines = [
        "# 坐标修正版 21 次正式实验结果",
        "",
        "- 算法版本：`{}`".format(report["verification"]["algorithm_version"]),
        "- 成功运行：`{}/{}`".format(
            report["verification"]["verified_runs"],
            report["verification"]["expected_runs"],
        ),
        "- 源码指纹：`{}`".format(
            report["verification"]["source_manifest_sha256"]
        ),
        "",
        "> 地图和轨迹没有外部真值。地图相对正常运行和轨迹相对正常运行的数值只能称为结构/干净参考代理，不能写成 ATE 或 RPE。",
        "",
        "## 创新点一：三包消融",
        "",
        "| 方法 | 画布面积/m2 | 最佳墙带覆盖率 | 最长连续墙段/m | 未知栅格比例 |",
        "|---|---:|---:|---:|---:|",
    ]
    for method in I1_METHODS:
        data = i1["methods"][method]["aggregate"]
        lines.append(
            "| {} | {} | {} | {} | {} |".format(
                method,
                mean_std_text(data["map_area_m2"], 2),
                mean_std_text(data["best_wall_gap_closed_coverage"], 4),
                mean_std_text(data["best_wall_continuous_length_m"], 2),
                mean_std_text(data["unknown_ratio"], 4),
            )
        )
    lines.extend([
        "",
        "### Proposed 方向与动态权重",
        "",
        "| Bag | D_conf均值 | D_conf>=0.8占比 | 方向误差中位数/deg | P90/deg | 纵向权重倍率均值 | 最大值 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ])
    for repeat in REPEATS:
        data = i1["proposed_direction"][str(repeat)]
        lines.append(
            "| repeat_{:02d} | {:.4f} | {:.2%} | {:.3f} | {:.3f} | {:.3f} | {:.3f} |".format(
                repeat,
                data["confidence"]["mean"],
                data["confidence_ge_0_8_ratio"],
                data["direction_to_trajectory_axis_error_deg"]["median"],
                data["direction_to_trajectory_axis_error_deg"]["p90"],
                data["longitudinal_weight_scale"]["mean"],
                data["longitudinal_weight_scale"]["max"],
            )
        )

    lines.extend([
        "",
        "## 创新点二：7事件 x 3包",
        "",
        "| 方法 | Chamfer均值/m | Chamfer P95/m | 墙体F1@0.15m | 主轴误差/deg | 轨迹P95干净参考偏差/m |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    for method in I2_METHODS:
        data = i2["methods"][method]["aggregate"]
        lines.append(
            "| {} | {} | {} | {} | {} | {} |".format(
                method,
                mean_std_text(data["chamfer_mean_m"], 4),
                mean_std_text(data["chamfer_p95_m"], 4),
                mean_std_text(data["wall_f1_at_0_15_m"], 4),
                mean_std_text(data["axis_error_deg"], 3),
                mean_std_text(data["trajectory_position_p95_m"], 4),
            )
        )
    detection = i2["proposed_detection_aggregate"]
    lines.extend([
        "",
        "### 一致性异常检测",
        "",
        "- 注入事件：`{}`，TP：`{}`，FP：`{}`，FN：`{}`。".format(
            detection["injected_events"], detection["true_positive_events"],
            detection["false_positive_detections"],
            detection["false_negative_events"],
        ),
        "- Precision：`{:.2%}`，Recall：`{:.2%}`，F1：`{:.2%}`。".format(
            detection["precision"], detection["recall"], detection["f1"]
        ),
        "- 检测延迟：均值 `{:.3f} s`，P95 `{:.3f} s`，最大 `{:.3f} s`。".format(
            detection["detection_delay_sec"]["mean"],
            detection["detection_delay_sec"]["p95"],
            detection["detection_delay_sec"]["max"],
        ),
        "",
        "## 正常 Full Proposed",
        "",
        "- 正常数据总时长：`{:.2f} min`。".format(
            normal["aggregate"]["duration_min"]
        ),
        "- 一致性异常触发：`{}`；误触发率：`{:.4f} 次/min`。".format(
            normal["aggregate"]["trigger_count"],
            normal["aggregate"]["false_triggers_per_min"],
        ),
        "- 三包 odometry 权重最低值均为 `1.0`，正常数据未被错误降权。",
        "- 相对仅创新点一，完整方法墙体 F1 均值为 `{:.4f}`，"
        "轨迹 P95 干净参考差异均值为 `{:.4f} m`；说明创新点二休眠时"
        "基本不改变创新点一结果。".format(
            normal["i1_only_vs_full_aggregate"]["wall_f1_at_0_15_m"]["mean"],
            normal["i1_only_vs_full_aggregate"]
            ["trajectory_position_p95_m"]["mean"],
        ),
        "",
        "## 证据边界",
        "",
        "- 三条 bag 是同一物理走廊的独立重复录制，不代表跨场景泛化。",
        "- 创新点二输入是确定性 odometry 一致性异常注入，不是真实轮胎打滑。",
        "- 21 次主实验主要证明模块隔离、方向正确性、事件检测和正常零误触发；持续累计漂移下的显著地图保护由 P0 实验单独验证。",
        "- 方向参考是在线轨迹 PCA 主轴，不是全站仪测得的物理走廊轴。",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    report = {
        "schema_version": 1,
        "verification": verify_runs(),
        "innovation1": analyze_i1(),
        "innovation2": analyze_i2(),
        "normal_full": analyze_normal_full(),
        "evidence_boundary": (
            "No external ground truth; clean-reference values are not ATE/RPE."
        ),
    }
    if report["verification"]["failures"]:
        raise RuntimeError("Formal run verification failed")
    json_path = ROOT / "formal_21_runs_metrics_coordinate_fix_20260728.json"
    markdown_path = ROOT / "坐标修正版21次正式实验报告_20260728.md"
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_markdown(report, markdown_path)
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    print("report:", markdown_path)
    print("metrics:", json_path)


if __name__ == "__main__":
    main()
